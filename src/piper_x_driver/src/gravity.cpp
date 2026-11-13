#include "piper_x_driver/gravity.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Dense>
#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>

namespace piper_x_driver {
namespace {

Mat3 euler_xyz_matrix(const Vec3 & rpy) {
  const double cx = std::cos(rpy[0]), sx = std::sin(rpy[0]);
  const double cy = std::cos(rpy[1]), sy = std::sin(rpy[1]);
  const double cz = std::cos(rpy[2]), sz = std::sin(rpy[2]);
  return Mat3{{
    {{cy * cz, cz * sx * sy - cx * sz, cx * cz * sy + sx * sz}},
    {{cy * sz, cx * cz + sx * sy * sz, cx * sy * sz - cz * sx}},
    {{-sy, cy * sx, cx * cy}},
  }};
}

Vec3 parse_vec3_attr(const char * text, const Vec3 & fallback = {{0.0, 0.0, 0.0}}) {
  if (text == nullptr) {
    return fallback;
  }
  std::istringstream iss(text);
  Vec3 out = fallback;
  if (!(iss >> out[0] >> out[1] >> out[2])) {
    throw std::runtime_error(std::string("Invalid 3D vector attribute in URDF: ") + text);
  }
  return out;
}

ArmChainModel parse_arm_chain_urdf(const std::string & path) {
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Failed to load PiPER-X URDF from piper_description: " + path);
  }
  const auto * robot = doc.FirstChildElement("robot");
  if (robot == nullptr) {
    throw std::runtime_error("Invalid URDF (missing <robot>): " + path);
  }

  std::map<std::string, const tinyxml2::XMLElement *> link_elems;
  for (const auto * l = robot->FirstChildElement("link"); l != nullptr;
       l = l->NextSiblingElement("link"))
  {
    const char * name = l->Attribute("name");
    if (name != nullptr) {
      link_elems[name] = l;
    }
  }
  std::map<std::string, const tinyxml2::XMLElement *> joint_elems;
  for (const auto * j = robot->FirstChildElement("joint"); j != nullptr;
       j = j->NextSiblingElement("joint"))
  {
    const char * name = j->Attribute("name");
    if (name != nullptr) {
      joint_elems[name] = j;
    }
  }

  ArmChainModel model;
  model.urdf_path = path;
  for (int i = 0; i < 6; ++i) {
    const std::string jname = "joint" + std::to_string(i + 1);
    const std::string lname = "link" + std::to_string(i + 1);
    auto jit = joint_elems.find(jname);
    auto lit = link_elems.find(lname);
    if (jit == joint_elems.end() || lit == link_elems.end()) {
      throw std::runtime_error("URDF missing " + jname + " or " + lname + " in " + path);
    }
    const auto * origin = jit->second->FirstChildElement("origin");
    model.joint_xyz[i] = origin ? parse_vec3_attr(origin->Attribute("xyz")) : Vec3{{0, 0, 0}};
    model.joint_rpy[i] = origin ? parse_vec3_attr(origin->Attribute("rpy")) : Vec3{{0, 0, 0}};
    model.joint_rot[i] = euler_xyz_matrix(model.joint_rpy[i]);
    const auto * limit_el = jit->second->FirstChildElement("limit");
    double lo = -12.5, hi = 12.5;
    if (limit_el != nullptr) {
      limit_el->QueryDoubleAttribute("lower", &lo);
      limit_el->QueryDoubleAttribute("upper", &hi);
    }
    model.joint_lower[i] = lo;
    model.joint_upper[i] = hi;

    const auto * inertial = lit->second->FirstChildElement("inertial");
    if (inertial == nullptr) {
      throw std::runtime_error("URDF missing <inertial> for " + lname + " in " + path);
    }
    const auto * i_orig = inertial->FirstChildElement("origin");
    model.link_com[i] = i_orig ? parse_vec3_attr(i_orig->Attribute("xyz")) : Vec3{{0, 0, 0}};
    const auto * mass_el = inertial->FirstChildElement("mass");
    if (mass_el == nullptr || mass_el->QueryDoubleAttribute("value", &model.link_mass[i]) != tinyxml2::XML_SUCCESS) {
      throw std::runtime_error("URDF missing mass value for " + lname);
    }
    const auto * iner_el = inertial->FirstChildElement("inertia");
    if (iner_el == nullptr) {
      throw std::runtime_error("URDF missing inertia tensor for " + lname);
    }
    double ixx = 0, ixy = 0, ixz = 0, iyy = 0, iyz = 0, izz = 0;
    iner_el->QueryDoubleAttribute("ixx", &ixx);
    iner_el->QueryDoubleAttribute("ixy", &ixy);
    iner_el->QueryDoubleAttribute("ixz", &ixz);
    iner_el->QueryDoubleAttribute("iyy", &iyy);
    iner_el->QueryDoubleAttribute("iyz", &iyz);
    iner_el->QueryDoubleAttribute("izz", &izz);
    model.link_inertia[i] = Mat3{{
      {{ixx, ixy, ixz}},
      {{ixy, iyy, iyz}},
      {{ixz, iyz, izz}},
    }};
  }
  return model;
}

const RigidBody & default_gripper_body() {
  static const RigidBody b = terminal_body("gripper");
  return b;
}

std::tuple<double, double, double> solve_sym3(
  double a00, double a01, double a02, double a11, double a12, double a22,
  double b0, double b1, double b2) noexcept
{
  const double inv0 = 1.0 / a00;
  const double f10 = a01 * inv0;
  const double f20 = a02 * inv0;
  const double m11 = a11 - f10 * a01;
  const double m12 = a12 - f10 * a02;
  const double rhs1 = b1 - f10 * b0;
  double m22 = a22 - f20 * a02;
  double rhs2 = b2 - f20 * b0;
  const double inv1 = 1.0 / m11;
  const double f21 = m12 * inv1;
  m22 -= f21 * m12;
  rhs2 -= f21 * rhs1;
  const double x2 = rhs2 / m22;
  const double x1 = (rhs1 - m12 * x2) * inv1;
  const double x0 = (b0 - a01 * x1 - a02 * x2) * inv0;
  return {x0, x1, x2};
}

Vec6 solve_damped_wrench(
  const Vec6 & tau,
  const std::array<Vec3, 6> & origins,
  const std::array<Vec3, 6> & axes,
  const Vec3 & tip,
  const Mat3 & rot5,
  double damping,
  bool use_gripper) noexcept
{
  const double t0 = tau[0], t1 = tau[1], t2 = tau[2];
  const double t3 = tau[3], t4 = tau[4], t5 = tau[5];
  const double ex = tip[0], ey = tip[1], ez = tip[2];
  const double r00 = rot5[0][0], r01 = rot5[0][1], r02 = rot5[0][2];
  const double r10 = rot5[1][0], r11 = rot5[1][1], r12 = rot5[1][2];
  const double r20 = rot5[2][0], r21 = rot5[2][1], r22 = rot5[2][2];
  double j0[6], j1[6], j2[6], j3[6], j4[6], j5[6];
  for (int i = 0; i < 6; ++i) {
    const double zx = axes[i][0], zy = axes[i][1], zz = axes[i][2];
    const double rx = ex - origins[i][0];
    const double ry = ey - origins[i][1];
    const double rz = ez - origins[i][2];
    const double vx = zy * rz - zz * ry;
    const double vy = zz * rx - zx * rz;
    const double vz = zx * ry - zy * rx;
    if (use_gripper) {
      j0[i] = -(r00 * vx + r10 * vy + r20 * vz);
      j1[i] = -(r01 * vx + r11 * vy + r21 * vz);
      j2[i] = r02 * vx + r12 * vy + r22 * vz;
      j3[i] = -(r00 * zx + r10 * zy + r20 * zz);
      j4[i] = -(r01 * zx + r11 * zy + r21 * zz);
      j5[i] = r02 * zx + r12 * zy + r22 * zz;
    } else {
      j0[i] = vx;
      j1[i] = vy;
      j2[i] = vz;
      j3[i] = zx;
      j4[i] = zy;
      j5[i] = zz;
    }
  }
  const double lam2 = damping * damping;
  auto dot6 = [](const double a[6], const double b[6]) noexcept {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3] + a[4] * b[4] + a[5] * b[5];
  };
  const double a00 = dot6(j0, j0) + lam2;
  const double a01 = dot6(j0, j1);
  const double a02 = dot6(j0, j2);
  const double a11 = dot6(j1, j1) + lam2;
  const double a12 = dot6(j1, j2);
  const double a22 = dot6(j2, j2) + lam2;

  const double b00 = dot6(j0, j3), b01 = dot6(j0, j4), b02 = dot6(j0, j5);
  const double b10 = dot6(j1, j3), b11 = dot6(j1, j4), b12 = dot6(j1, j5);
  const double b20 = dot6(j2, j3), b21 = dot6(j2, j4), b22 = dot6(j2, j5);

  const double d00 = dot6(j3, j3) + lam2;
  const double d01 = dot6(j3, j4);
  const double d02 = dot6(j3, j5);
  const double d11 = dot6(j4, j4) + lam2;
  const double d12 = dot6(j4, j5);
  const double d22 = dot6(j5, j5) + lam2;

  const double tv[6] = {t0, t1, t2, t3, t4, t5};
  const double bv0 = dot6(j0, tv), bv1 = dot6(j1, tv), bv2 = dot6(j2, tv);
  const double bw0 = dot6(j3, tv), bw1 = dot6(j4, tv), bw2 = dot6(j5, tv);

  const auto [y00, y10, y20] = solve_sym3(a00, a01, a02, a11, a12, a22, b00, b10, b20);
  const auto [y01, y11, y21] = solve_sym3(a00, a01, a02, a11, a12, a22, b01, b11, b21);
  const auto [y02, y12, y22] = solve_sym3(a00, a01, a02, a11, a12, a22, b02, b12, b22);
  const auto [f00, f01, f02] = solve_sym3(a00, a01, a02, a11, a12, a22, bv0, bv1, bv2);

  const double s00 = d00 - (b00 * y00 + b10 * y10 + b20 * y20);
  const double s01 = d01 - (b00 * y01 + b10 * y11 + b20 * y21);
  const double s02 = d02 - (b00 * y02 + b10 * y12 + b20 * y22);
  const double s11 = d11 - (b01 * y01 + b11 * y11 + b21 * y21);
  const double s12 = d12 - (b01 * y02 + b11 * y12 + b21 * y22);
  const double s22 = d22 - (b02 * y02 + b12 * y12 + b22 * y22);

  const double rw0 = bw0 - (b00 * f00 + b10 * f01 + b20 * f02);
  const double rw1 = bw1 - (b01 * f00 + b11 * f01 + b21 * f02);
  const double rw2 = bw2 - (b02 * f00 + b12 * f01 + b22 * f02);

  const auto [mx, my, mz] = solve_sym3(s00, s01, s02, s11, s12, s22, rw0, rw1, rw2);
  const double fx = f00 - (y00 * mx + y01 * my + y02 * mz);
  const double fy = f01 - (y10 * mx + y11 * my + y12 * mz);
  const double fz = f02 - (y20 * mx + y21 * my + y22 * mz);
  return Vec6{{fx, fy, fz, mx, my, mz}};
}

double round_2(double v) noexcept {
  return std::round(v * 100.0) / 100.0;
}

}  // namespace

std::string default_arm_urdf_path() {
  try {
    auto share = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("piper_description"));
    auto p = share / "urdf" / "piper_no_gripper_description.urdf";
    if (std::filesystem::is_regular_file(p)) {
      return p.string();
    }
  } catch (const std::exception &) {
  }
#ifdef PIPER_X_DRIVER_SOURCE_DIR
  auto src = std::filesystem::path(PIPER_X_DRIVER_SOURCE_DIR).parent_path() /
             "piper_description" / "urdf" / "piper_no_gripper_description.urdf";
  if (std::filesystem::is_regular_file(src)) {
    return src.string();
  }
#endif
  throw std::runtime_error("Cannot locate piper_description/urdf/piper_no_gripper_description.urdf");
}

const ArmChainModel & load_arm_chain_model(const std::string & urdf_path) {
  if (urdf_path.empty()) {
    static const ArmChainModel kDefault = parse_arm_chain_urdf(default_arm_urdf_path());
    return kDefault;
  }
  static std::mutex mu;
  static std::map<std::string, ArmChainModel> cache;
  std::lock_guard<std::mutex> lock(mu);
  auto it = cache.find(urdf_path);
  if (it == cache.end()) {
    it = cache.emplace(urdf_path, parse_arm_chain_urdf(urdf_path)).first;
  }
  return it->second;
}

double default_effector_mass_kg() {
  return default_gripper_body().mass_kg;
}

Vec3 default_effector_com_m() {
  return default_gripper_body().com_m;
}

const std::array<double, 7> & link_masses_default() {
  static const std::array<double, 7> masses = [] {
    const auto & m = load_arm_chain_model().link_mass;
    return std::array<double, 7>{{m[0], m[1], m[2], m[3], m[4], m[5], default_effector_mass_kg()}};
  }();
  return masses;
}

const std::array<Mat3, 6> & link_inertias() {
  return load_arm_chain_model().link_inertia;
}

FkResult fast_fk(const Vec6 & position, std::optional<Vec3> effector_com) {
  const auto & chain = load_arm_chain_model();
  FkResult out;
  double r00 = 1.0, r01 = 0.0, r02 = 0.0;
  double r10 = 0.0, r11 = 1.0, r12 = 0.0;
  double r20 = 0.0, r21 = 0.0, r22 = 1.0;
  double tx = 0.0, ty = 0.0, tz = 0.0;
  for (int i = 0; i < 6; ++i) {
    const double q = position[i];
    const double ox = chain.joint_xyz[i][0];
    const double oy = chain.joint_xyz[i][1];
    const double oz = chain.joint_xyz[i][2];
    tx += r00 * ox + r01 * oy + r02 * oz;
    ty += r10 * ox + r11 * oy + r12 * oz;
    tz += r20 * ox + r21 * oy + r22 * oz;
    const auto & m = chain.joint_rot[i];
    const double n00 = r00 * m[0][0] + r01 * m[1][0] + r02 * m[2][0];
    const double n01 = r00 * m[0][1] + r01 * m[1][1] + r02 * m[2][1];
    const double n02 = r00 * m[0][2] + r01 * m[1][2] + r02 * m[2][2];
    const double n10 = r10 * m[0][0] + r11 * m[1][0] + r12 * m[2][0];
    const double n11 = r10 * m[0][1] + r11 * m[1][1] + r12 * m[2][1];
    const double n12 = r10 * m[0][2] + r11 * m[1][2] + r12 * m[2][2];
    const double n20 = r20 * m[0][0] + r21 * m[1][0] + r22 * m[2][0];
    const double n21 = r20 * m[0][1] + r21 * m[1][1] + r22 * m[2][1];
    const double n22 = r20 * m[0][2] + r21 * m[1][2] + r22 * m[2][2];
    out.origins[i] = {{tx, ty, tz}};
    out.axes[i] = {{n02, n12, n22}};
    double s = 0.0, c = 0.0;
#if defined(__GLIBC__) || defined(_GNU_SOURCE)
    ::sincos(q, &s, &c);
#else
    s = std::sin(q);
    c = std::cos(q);
#endif
    r00 = n00 * c + n01 * s;
    r01 = n01 * c - n00 * s;
    r02 = n02;
    r10 = n10 * c + n11 * s;
    r11 = n11 * c - n10 * s;
    r12 = n12;
    r20 = n20 * c + n21 * s;
    r21 = n21 * c - n20 * s;
    r22 = n22;
    const double cx = chain.link_com[i][0];
    const double cy = chain.link_com[i][1];
    const double cz = chain.link_com[i][2];
    out.centers[i] = {{
      tx + r00 * cx + r01 * cy + r02 * cz,
      ty + r10 * cx + r11 * cy + r12 * cz,
      tz + r20 * cx + r21 * cy + r22 * cz,
    }};
    out.rots[i] = {{{{r00, r01, r02}}, {{r10, r11, r12}}, {{r20, r21, r22}}}};
  }
  const Vec3 ec = effector_com.value_or(default_effector_com_m());
  out.centers[6] = {{
    tx + r00 * ec[0] + r01 * ec[1] + r02 * ec[2],
    ty + r10 * ec[0] + r11 * ec[1] + r12 * ec[2],
    tz + r20 * ec[0] + r21 * ec[1] + r22 * ec[2],
  }};
  out.rots[6] = out.rots[5];
  return out;
}

Mat3 link6_rotation_in_base(const Vec6 & position) {
  return fast_fk(position, Vec3{{0.0, 0.0, 0.0}}).rots[5];
}

Vec6 gravity_torques(
  const Vec6 & position,
  std::optional<double> mass_kg,
  std::optional<Vec3> com_m)
{
  const auto & def_masses = link_masses_default();
  const double m6 = mass_kg.value_or(def_masses[6]);
  const auto fk = fast_fk(position, com_m);
  Vec6 out{};
  double acc_m = m6;
  double acc_mx = acc_m * fk.centers[6][0];
  double acc_my = acc_m * fk.centers[6][1];
  for (int j = 5; j >= 0; --j) {
    const double mj = def_masses[j];
    acc_m += mj;
    acc_mx += mj * fk.centers[j][0];
    acc_my += mj * fk.centers[j][1];
    const double arm_x = acc_mx - acc_m * fk.origins[j][0];
    const double arm_y = acc_my - acc_m * fk.origins[j][1];
    out[j] = 9.81 * (arm_y * fk.axes[j][0] - arm_x * fk.axes[j][1]);
  }
  return out;
}

Vec6 joint_inertias(
  const Vec6 & position,
  const std::string & end_effector,
  std::optional<double> mass_kg,
  std::optional<Vec3> com_m,
  const Vec6 & scales,
  std::optional<Mat3> inertia_tensor)
{
  const auto & def_masses = link_masses_default();
  const auto & chain_inertias = link_inertias();
  auto term = terminal_for_model(end_effector);
  std::optional<RigidBody> nominal;
  if (term.has_value() && term->payload_link.has_value()) {
    nominal = terminal_body(end_effector);
  }
  const double m6 = mass_kg.has_value()
    ? *mass_kg
    : (nominal.has_value() ? nominal->mass_kg : def_masses[6]);
  const Vec3 c6 = com_m.has_value()
    ? *com_m
    : (nominal.has_value() ? nominal->com_m : default_effector_com_m());
  Mat3 eff_i{};
  if (inertia_tensor.has_value()) {
    eff_i = *inertia_tensor;
  } else if (term.has_value()) {
    eff_i = term->body().inertia;
  } else {
    eff_i = terminal_body("teaching_handle").inertia;
  }
  const auto fk = fast_fk(position, c6);
  std::array<double, 7> masses{};
  for (int i = 0; i < 6; ++i) {
    masses[i] = def_masses[i];
  }
  masses[6] = m6;
  Vec6 out{};
  for (int j = 0; j < 6; ++j) {
    const double zx = fk.axes[j][0], zy = fk.axes[j][1], zz = fk.axes[j][2];
    const double ox = fk.origins[j][0], oy = fk.origins[j][1], oz = fk.origins[j][2];
    double total_j = 0.0;
    for (int k = j; k < 7; ++k) {
      const double rx = fk.centers[k][0] - ox;
      const double ry = fk.centers[k][1] - oy;
      const double rz = fk.centers[k][2] - oz;
      const double vx = zy * rz - zz * ry;
      const double vy = zz * rx - zx * rz;
      const double vz = zx * ry - zy * rx;
      const auto & rk = fk.rots[k];
      const double lx = rk[0][0] * zx + rk[1][0] * zy + rk[2][0] * zz;
      const double ly = rk[0][1] * zx + rk[1][1] * zy + rk[2][1] * zz;
      const double lz = rk[0][2] * zx + rk[1][2] * zy + rk[2][2] * zz;
      const Mat3 & tk = (k < 6) ? chain_inertias[k] : eff_i;
      const double rot_part =
        lx * (tk[0][0] * lx + tk[0][1] * ly + tk[0][2] * lz) +
        ly * (tk[1][0] * lx + tk[1][1] * ly + tk[1][2] * lz) +
        lz * (tk[2][0] * lx + tk[2][1] * ly + tk[2][2] * lz);
      total_j += masses[k] * (vx * vx + vy * vy + vz * vz) + rot_part;
    }
    out[j] = scales[j] * total_j;
  }
  return out;
}

Mat6 geometric_jacobian(
  const Vec6 & position,
  const Vec3 & tool_offset_m,
  const std::string & frame)
{
  if (frame != "gripper" && frame != "base_link") {
    throw std::invalid_argument("frame must be 'gripper' or 'base_link'");
  }
  const bool use_gripper = (frame == "gripper");
  const auto fk = fast_fk(position, tool_offset_m);
  const double ex = fk.centers[6][0], ey = fk.centers[6][1], ez = fk.centers[6][2];
  const auto & r = fk.rots[5];
  Mat6 j{};
  for (int i = 0; i < 6; ++i) {
    const double zx = fk.axes[i][0], zy = fk.axes[i][1], zz = fk.axes[i][2];
    const double rx = ex - fk.origins[i][0];
    const double ry = ey - fk.origins[i][1];
    const double rz = ez - fk.origins[i][2];
    const double vx = zy * rz - zz * ry;
    const double vy = zz * rx - zx * rz;
    const double vz = zx * ry - zy * rx;
    if (use_gripper) {
      j[0][i] = -(r[0][0] * vx + r[1][0] * vy + r[2][0] * vz);
      j[1][i] = -(r[0][1] * vx + r[1][1] * vy + r[2][1] * vz);
      j[2][i] = r[0][2] * vx + r[1][2] * vy + r[2][2] * vz;
      j[3][i] = -(r[0][0] * zx + r[1][0] * zy + r[2][0] * zz);
      j[4][i] = -(r[0][1] * zx + r[1][1] * zy + r[2][1] * zz);
      j[5][i] = r[0][2] * zx + r[1][2] * zy + r[2][2] * zz;
    } else {
      j[0][i] = vx;
      j[1][i] = vy;
      j[2][i] = vz;
      j[3][i] = zx;
      j[4][i] = zy;
      j[5][i] = zz;
    }
  }
  return j;
}

namespace {

Vec3 rotation_error_world(const Mat3 & r_des, const Mat3 & r_curr) noexcept {
  // R_err = R_des * R_curr^T; extract axis-angle vector in base_link frame
  Eigen::Matrix3d rd, rc;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      rd(i, j) = r_des[i][j];
      rc(i, j) = r_curr[i][j];
    }
  }
  const Eigen::Matrix3d r_err = rd * rc.transpose();
  Eigen::AngleAxisd aa(r_err);
  double angle = aa.angle();
  if (angle > 3.14159265358979323846) {
    angle -= 2.0 * 3.14159265358979323846;
  }
  const Eigen::Vector3d vec = aa.axis() * angle;
  return Vec3{{vec.x(), vec.y(), vec.z()}};
}

Eigen::Matrix3d inv_sym3_damped(const Eigen::Matrix3d & m, double lam2) noexcept {
  Eigen::Matrix3d d = m;
  d(0, 0) += lam2;
  d(1, 1) += lam2;
  d(2, 2) += lam2;
  return d.ldlt().solve(Eigen::Matrix3d::Identity());
}

}  // namespace

IkResult inverse_kinematics(
  const Vec3 & target_position,
  const Mat3 & target_rotation,
  const Vec6 & q_seed,
  const Vec3 & tool_offset_m,
  std::optional<Vec6> lower_limits,
  std::optional<Vec6> upper_limits,
  double damping,
  int max_iters,
  double pos_tol_m,
  double rot_tol_rad)
{
  const auto & chain = load_arm_chain_model();
  const Vec6 lo = lower_limits.value_or(chain.joint_lower);
  const Vec6 hi = upper_limits.value_or(chain.joint_upper);
  Vec6 q{};
  for (int i = 0; i < 6; ++i) {
    q[i] = std::clamp(q_seed[i], lo[i], hi[i]);
  }

  const double lam2_p = std::max(damping * damping, 1e-12);
  const double lam2_r = std::max(lam2_p, 1e-4);
  const int iters_cap = std::max(1, max_iters);

  int iter_used = 0;
  double pos_err = 0.0;
  double rot_err = 0.0;

  for (int iter = 0; iter < iters_cap; ++iter) {
    const FkResult fk = fast_fk(q, tool_offset_m);
    const Vec3 & tip = fk.centers[6];
    const Eigen::Vector3d ep(
      target_position[0] - tip[0],
      target_position[1] - tip[1],
      target_position[2] - tip[2]);
    const Vec3 er_v = rotation_error_world(target_rotation, fk.rots[5]);
    const Eigen::Vector3d er(er_v[0], er_v[1], er_v[2]);

    pos_err = ep.norm();
    rot_err = er.norm();
    if (pos_err <= pos_tol_m && rot_err <= rot_tol_rad) {
      return IkResult{q, true, iter_used, pos_err, rot_err};
    }
    ++iter_used;

    Eigen::Matrix<double, 6, 6> jac;
    for (int i = 0; i < 6; ++i) {
      const double zx = fk.axes[i][0], zy = fk.axes[i][1], zz = fk.axes[i][2];
      const double rx = tip[0] - fk.origins[i][0];
      const double ry = tip[1] - fk.origins[i][1];
      const double rz = tip[2] - fk.origins[i][2];
      jac(0, i) = zy * rz - zz * ry;
      jac(1, i) = zz * rx - zx * rz;
      jac(2, i) = zx * ry - zy * rx;
      jac(3, i) = zx;
      jac(4, i) = zy;
      jac(5, i) = zz;
    }

    std::array<bool, 6> free_j{{true, true, true, true, true, true}};
    Eigen::Matrix<double, 6, 1> dq = Eigen::Matrix<double, 6, 1>::Zero();
    uint32_t seen_masks = 0;

    for (int sub = 0; sub < 12; ++sub) {
      Eigen::Matrix<double, 6, 1> dq_pinned = Eigen::Matrix<double, 6, 1>::Zero();
      int n_free = 0;
      int free_idx[6];
      for (int i = 0; i < 6; ++i) {
        if (free_j[i]) {
          free_idx[n_free++] = i;
        } else {
          dq_pinned(i) = dq(i);
        }
      }
      const Eigen::Vector3d ep_rem = ep - jac.topRows<3>() * dq_pinned;
      const Eigen::Vector3d er_rem = er - jac.bottomRows<3>() * dq_pinned;

      Eigen::Vector3d w_p = Eigen::Vector3d::Zero();
      Eigen::Vector3d w_rot = Eigen::Vector3d::Zero();
      Eigen::Matrix3d jw_pv = Eigen::Matrix3d::Zero();
      Eigen::Matrix<double, 6, 1> dq_target = dq_pinned;

      if (n_free > 0) {
        Eigen::MatrixXd jv(3, n_free), jw(3, n_free);
        for (int k = 0; k < n_free; ++k) {
          jv.col(k) = jac.block<3, 1>(0, free_idx[k]);
          jw.col(k) = jac.block<3, 1>(3, free_idx[k]);
        }
        const Eigen::Matrix3d w_p_dual = inv_sym3_damped(jv * jv.transpose(), lam2_p);
        const Eigen::MatrixXd pv = jv.transpose() * w_p_dual;
        w_p = w_p_dual * ep_rem;
        const Eigen::VectorXd dq_p = jv.transpose() * w_p;
        jw_pv = jw * pv;
        const Eigen::MatrixXd jw_null = jw - jw_pv * jv;
        w_rot = inv_sym3_damped(jw_null * jw_null.transpose(), lam2_r) * (er_rem - jw * dq_p);
        const Eigen::VectorXd dq_r = jw_null.transpose() * w_rot;
        const Eigen::VectorXd dq_f = dq_p + (dq_r - pv * (jv * dq_r));
        for (int k = 0; k < n_free; ++k) {
          dq_target(free_idx[k]) = dq_f(k);
        }
      } else {
        w_p = inv_sym3_damped(Eigen::Matrix3d::Zero(), lam2_p) * ep_rem;
        w_rot = inv_sym3_damped(Eigen::Matrix3d::Zero(), lam2_r) * er_rem;
      }

      const Eigen::Matrix<double, 6, 1> d = dq_target - dq;
      int best_idx = -1;
      double best_alpha = 1.0;
      for (int k = 0; k < n_free; ++k) {
        const int idx = free_idx[k];
        const double q_cand = q[idx] + dq_target(idx);
        if (q_cand < lo[idx]) {
          const double alpha = (d(idx) < -1e-15) ? ((lo[idx] - (q[idx] + dq(idx))) / d(idx)) : 0.0;
          if (alpha < best_alpha) {
            best_alpha = std::max(0.0, alpha);
            best_idx = idx;
          }
        } else if (q_cand > hi[idx]) {
          const double alpha = (d(idx) > 1e-15) ? ((hi[idx] - (q[idx] + dq(idx))) / d(idx)) : 0.0;
          if (alpha < best_alpha) {
            best_alpha = std::max(0.0, alpha);
            best_idx = idx;
          }
        }
      }
      if (best_idx >= 0) {
        dq += best_alpha * d;
        dq(best_idx) = ((d(best_idx) < 0.0) ? lo[best_idx] : hi[best_idx]) - q[best_idx];
        free_j[best_idx] = false;
        continue;
      }

      dq = dq_target;
      uint32_t mask = 0;
      for (int i = 0; i < 6; ++i) {
        if (free_j[i]) mask |= (1u << i);
      }
      if (mask == 0x3Fu || ((seen_masks >> mask) & 1u) != 0) {
        break;
      }
      seen_masks |= (1u << mask);
      int release_idx = -1;
      double best_inward = 1e-9;
      for (int i = 0; i < 6; ++i) {
        if (free_j[i]) continue;
        const Eigen::Vector3d jv_i = jac.block<3, 1>(0, i);
        const Eigen::Vector3d jw_null_i = jac.block<3, 1>(3, i) - jw_pv * jv_i;
        const double drive = jv_i.dot(w_p) + jw_null_i.dot(w_rot);
        const bool at_lo = std::abs((q[i] + dq(i)) - lo[i]) <= std::abs((q[i] + dq(i)) - hi[i]);
        const double inward = at_lo ? drive : -drive;
        if (inward > best_inward) {
          best_inward = inward;
          release_idx = i;
        }
      }
      if (release_idx < 0) {
        break;
      }
      free_j[release_idx] = true;
    }

    double step_norm2 = 0.0;
    for (int i = 0; i < 6; ++i) {
      const double q_new = std::clamp(q[i] + dq(i), lo[i], hi[i]);
      const double ds = q_new - q[i];
      step_norm2 += ds * ds;
      q[i] = q_new;
    }
    if (step_norm2 < 1e-12) {
      break;
    }
  }

  const FkResult final_fk = fast_fk(q, tool_offset_m);
  const Eigen::Vector3d ep_f(
    target_position[0] - final_fk.centers[6][0],
    target_position[1] - final_fk.centers[6][1],
    target_position[2] - final_fk.centers[6][2]);
  const Vec3 er_fv = rotation_error_world(target_rotation, final_fk.rots[5]);
  pos_err = ep_f.norm();
  rot_err = std::sqrt(er_fv[0] * er_fv[0] + er_fv[1] * er_fv[1] + er_fv[2] * er_fv[2]);
  return IkResult{q, (pos_err <= pos_tol_m && rot_err <= rot_tol_rad), iter_used, pos_err, rot_err};
}

void JointGravityModel::validate() const {
  if (!std::isfinite(scale)) {
    throw std::invalid_argument("scale must be finite");
  }
  if (!std::isfinite(angle_offset)) {
    throw std::invalid_argument("angle_offset must be finite");
  }
  if (!std::isfinite(bias)) {
    throw std::invalid_argument("bias must be finite");
  }
  if (!std::isfinite(static_friction)) {
    throw std::invalid_argument("static_friction must be finite");
  }
  if (scale < 0.0) {
    throw std::invalid_argument("scale must be non-negative");
  }
  if (static_friction < 0.0) {
    throw std::invalid_argument("static_friction must be non-negative");
  }
}

void ArmGravityModel::validate() const {
  if (!std::isfinite(added_mass_kg) || added_mass_kg < 0.0) {
    throw std::invalid_argument("Added gripper mass must be finite and non-negative");
  }
  for (const auto & kv : joints) {
    if (kv.first < 1 || kv.first > 6) {
      throw std::invalid_argument("Joint index must be in 1..6");
    }
    kv.second.validate();
  }
}

JointGravityModel ArmGravityModel::joint(int index) const {
  if (index < 1 || index > 6) {
    throw std::invalid_argument("Joint index must be in 1..6");
  }
  auto it = joints.find(index);
  return (it != joints.end()) ? it->second : JointGravityModel{};
}

ArmGravityModel ArmGravityModel::for_effector(const std::string & model) const {
  auto terminal = terminal_for_model(model);
  ArmGravityModel copy = *this;
  if (!terminal.has_value() || !terminal->payload_link.has_value()) {
    copy.effector_model.clear();
    copy.added_mass_kg = 0.0;
    return copy;
  }
  const double mass =
    (effector_model.empty() || effector_model == terminal->model) ? added_mass_kg : 0.0;
  copy.effector_model = terminal->model;
  copy.added_mass_kg = mass;
  return copy;
}

ArmGravityModel ArmGravityModel::with_added_mass(double mass_kg) const {
  ArmGravityModel copy = *this;
  copy.added_mass_kg = mass_kg;
  copy.validate();
  return copy;
}

ArmGravityModel ArmGravityModel::with_joint(int index, const JointGravityModel & model) const {
  if (index < 1 || index > 6) {
    throw std::invalid_argument("Joint index must be in 1..6");
  }
  model.validate();
  ArmGravityModel copy = *this;
  copy.joints[index] = model;
  return copy;
}

RigidBody ArmGravityModel::effector_body(
  std::optional<double> mass_kg,
  std::optional<Vec3> com_m,
  const std::string & end_effector) const
{
  const std::string name = effector_model.empty() ? end_effector : effector_model;
  std::optional<double> m = mass_kg;
  std::optional<Vec3> c = com_m;
  if (effector_model.empty()) {
    if (!m.has_value()) {
      m = default_effector_mass_kg();
    }
    if (!c.has_value()) {
      c = default_effector_com_m();
    }
  }
  const std::string resolved = terminal_for_model(name).has_value() ? name : "teaching_handle";
  return terminal_body(resolved, added_mass_kg, m, c);
}

Vec6 ArmGravityModel::torques(
  const Vec6 & position,
  std::optional<double> mass_kg,
  std::optional<Vec3> com_m) const
{
  Vec6 q_eff{};
  for (int i = 0; i < 6; ++i) {
    q_eff[i] = position[i] + joint(i + 1).angle_offset;
  }
  const RigidBody body = effector_body(mass_kg, com_m);
  const Vec6 cad = gravity_torques(q_eff, body.mass_kg, body.com_m);
  Vec6 out{};
  for (int i = 0; i < 6; ++i) {
    const auto jm = joint(i + 1);
    out[i] = jm.scale * cad[i] + jm.bias;
  }
  return out;
}

Vec6 ArmGravityModel::inertias(
  const Vec6 & position,
  const std::string & end_effector,
  std::optional<double> mass_kg,
  std::optional<Vec3> com_m) const
{
  Vec6 q_eff{};
  Vec6 scales{};
  for (int i = 0; i < 6; ++i) {
    const auto jm = joint(i + 1);
    q_eff[i] = position[i] + jm.angle_offset;
    scales[i] = (jm.scale != 0.0) ? jm.scale : 1.0;
  }
  const RigidBody body = effector_body(mass_kg, com_m, end_effector);
  return joint_inertias(q_eff, end_effector, body.mass_kg, body.com_m, scales, body.inertia);
}

Mat6 ArmGravityModel::jacobian(
  const Vec6 & position,
  const Vec3 & tool_offset_m,
  const std::string & frame) const
{
  Vec6 q_eff{};
  for (int i = 0; i < 6; ++i) {
    q_eff[i] = position[i] + joint(i + 1).angle_offset;
  }
  return geometric_jacobian(q_eff, tool_offset_m, frame);
}

IkResult ArmGravityModel::inverse_kinematics(
  const Vec3 & target_position,
  const Mat3 & target_rotation,
  const Vec6 & q_seed,
  const Vec3 & tool_offset_m,
  std::optional<Vec6> lower_limits,
  std::optional<Vec6> upper_limits,
  double damping,
  int max_iters,
  double pos_tol_m,
  double rot_tol_rad) const
{
  const auto & chain = load_arm_chain_model();
  const Vec6 raw_lo = lower_limits.value_or(chain.joint_lower);
  const Vec6 raw_hi = upper_limits.value_or(chain.joint_upper);
  Vec6 q_eff_seed{}, eff_lo{}, eff_hi{};
  for (int i = 0; i < 6; ++i) {
    const double off = joint(i + 1).angle_offset;
    q_eff_seed[i] = q_seed[i] + off;
    eff_lo[i] = raw_lo[i] + off;
    eff_hi[i] = raw_hi[i] + off;
  }
  IkResult res = piper_x_driver::inverse_kinematics(
    target_position, target_rotation, q_eff_seed, tool_offset_m,
    eff_lo, eff_hi, damping, max_iters, pos_tol_m, rot_tol_rad);
  for (int i = 0; i < 6; ++i) {
    res.joints[i] -= joint(i + 1).angle_offset;
  }
  return res;
}
std::array<Mat3, 7> ArmGravityModel::scaled_inertia_tensors(
  const std::string & end_effector) const
{
  const Mat3 eff_i = effector_body(std::nullopt, std::nullopt, end_effector).inertia;
  const auto & chain_inertias = link_inertias();
  std::array<Mat3, 7> out{};
  for (int i = 0; i < 6; ++i) {
    const double s = (joint(i + 1).scale != 0.0) ? joint(i + 1).scale : 1.0;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        out[i][r][c] = chain_inertias[i][r][c] * s;
      }
    }
  }
  const double s6 = (joint(6).scale != 0.0) ? joint(6).scale : 1.0;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out[6][r][c] = eff_i[r][c] * s6;
    }
  }
  return out;
}

Vec6 max_critical_kd(
  const Vec6 & kp,
  const std::string & end_effector,
  const ArmGravityModel * gravity_model)
{
  std::string eff = end_effector;
  if (gravity_model != nullptr && !gravity_model->effector_model.empty()) {
    eff = gravity_model->effector_model;
  }
  Vec6 scales{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  double added_mass = 0.0;
  if (gravity_model != nullptr) {
    for (int i = 0; i < 6; ++i) {
      const double s = gravity_model->joint(i + 1).scale;
      scales[i] = (s != 0.0) ? s : 1.0;
    }
    added_mass = gravity_model->added_mass_kg;
  }
  auto term = terminal_for_model(eff);
  Vec6 inertias = term.has_value()
    ? term->max_joint_inertia_kg_m2
    : terminal_for_model("teaching_handle")->max_joint_inertia_kg_m2;
  if (term.has_value() && term->payload_link.has_value() && added_mass != 0.0) {
    const auto & shape = term->components.at(*term->payload_link);
    Eigen::Matrix3d unit;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        unit(r, c) = shape.inertia[r][c] / shape.mass_kg;
      }
    }
    const double radius = std::sqrt(
      shape.com_m[0] * shape.com_m[0] +
      shape.com_m[1] * shape.com_m[1] +
      shape.com_m[2] * shape.com_m[2]);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(unit);
    const double unit_max = eig.eigenvalues().maxCoeff();
    const auto & chain_xyz = load_arm_chain_model().joint_xyz;
    Vec6 increment{};
    for (int j = 0; j < 6; ++j) {
      double len_sum = 0.0;
      for (int k = j + 1; k < 6; ++k) {
        len_sum += std::sqrt(
          chain_xyz[k][0] * chain_xyz[k][0] +
          chain_xyz[k][1] * chain_xyz[k][1] +
          chain_xyz[k][2] * chain_xyz[k][2]);
      }
      const double arm_r = len_sum + radius;
      increment[j] = arm_r * arm_r + unit_max;
    }
    increment[5] = unit(2, 2) + shape.com_m[0] * shape.com_m[0] + shape.com_m[1] * shape.com_m[1];
    for (int j = 0; j < 6; ++j) {
      inertias[j] += added_mass * increment[j];
    }
  }
  Vec6 out{};
  for (int i = 0; i < 6; ++i) {
    out[i] = round_2(std::sqrt(2.0 * (scales[i] * inertias[i]) * std::max(0.0, kp[i])));
  }
  return out;
}

Vec6 estimate_joint_torques(
  const Vec6 & position,
  const Vec6 & velocity,
  const Vec6 & target_position,
  const Vec6 & target_velocity,
  const Vec6 & kp,
  const Vec6 & kd,
  const Vec6 & torque_cmd,
  const Vec6 & acceleration,
  const std::string & end_effector,
  std::optional<double> mass_kg,
  std::optional<Vec3> com_m,
  const ArmGravityModel * gravity_model)
{
  const Vec6 inertias = (gravity_model == nullptr)
    ? joint_inertias(position, end_effector, mass_kg, com_m)
    : gravity_model->inertias(position, end_effector, mass_kg, com_m);
  Vec6 out{};
  for (int i = 0; i < 6; ++i) {
    out[i] =
      kp[i] * (position[i] - target_position[i]) +
      kd[i] * (velocity[i] - target_velocity[i]) -
      torque_cmd[i] +
      inertias[i] * acceleration[i];
  }
  return out;
}

Vec6 end_effector_wrench(
  const Vec6 & joint_torques,
  const Vec6 & position,
  const ArmGravityModel * gravity_model,
  const Vec3 & tool_offset_m,
  double damping,
  const std::string & frame)
{
  if (!std::isfinite(damping) || damping < 0.0) {
    throw std::invalid_argument("damping must be finite and non-negative");
  }
  if (frame != "gripper" && frame != "base_link") {
    throw std::invalid_argument("frame must be 'gripper' or 'base_link'");
  }
  bool any_nonzero = false;
  for (double t : joint_torques) {
    if (t != 0.0) {
      any_nonzero = true;
      break;
    }
  }
  if (!any_nonzero) {
    return Vec6{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  }
  if (damping > 0.0) {
    Vec6 q_eff = position;
    if (gravity_model != nullptr) {
      for (int i = 0; i < 6; ++i) {
        q_eff[i] += gravity_model->joint(i + 1).angle_offset;
      }
    }
    const auto fk = fast_fk(q_eff, tool_offset_m);
    return solve_damped_wrench(
      joint_torques, fk.origins, fk.axes, fk.centers[6], fk.rots[5],
      damping, frame == "gripper");
  }
  const Mat6 j_mat = (gravity_model == nullptr)
    ? geometric_jacobian(position, tool_offset_m, frame)
    : gravity_model->jacobian(position, tool_offset_m, frame);
  Eigen::Matrix<double, 6, 6> jt;
  Eigen::Matrix<double, 6, 1> tau_vec;
  for (int r = 0; r < 6; ++r) {
    tau_vec(r) = joint_torques[r];
    for (int c = 0; c < 6; ++c) {
      jt(r, c) = j_mat[c][r];
    }
  }
  Eigen::Matrix<double, 6, 1> w =
    jt.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(tau_vec);
  Vec6 out{};
  for (int i = 0; i < 6; ++i) {
    out[i] = w(i);
  }
  return out;
}

Vec6 estimate_end_wrench(
  const Vec6 & position,
  const Vec6 & velocity,
  const Vec6 & target_position,
  const Vec6 & target_velocity,
  const Vec6 & kp,
  const Vec6 & kd,
  const Vec6 & torque_cmd,
  const Vec6 & acceleration,
  const std::string & end_effector,
  std::optional<double> mass_kg,
  std::optional<Vec3> com_m,
  const ArmGravityModel * gravity_model,
  const Vec3 & tool_offset_m,
  double damping,
  const std::string & frame)
{
  const Vec6 tau_ext = estimate_joint_torques(
    position, velocity, target_position, target_velocity, kp, kd,
    torque_cmd, acceleration, end_effector, mass_kg, com_m, gravity_model);
  return end_effector_wrench(
    tau_ext, position, gravity_model, tool_offset_m, damping, frame);
}

ArmGravityModel load_gravity_model(const std::string & path) {
  YAML::Node data = YAML::LoadFile(path);
  if (!data || !data.IsMap()) {
    throw std::invalid_argument("Unsupported gravity model format or version");
  }
  const int version = data["version"] ? data["version"].as<int>() : 1;
  if (version != 1) {
    throw std::invalid_argument("Unsupported gravity model format or version");
  }
  ArmGravityModel model;
  model.arm_id = data["arm_id"] ? data["arm_id"].as<std::string>() : "arm_1";
  if (data["effector_model"] && !data["effector_model"].IsNull()) {
    model.effector_model = data["effector_model"].as<std::string>();
  }
  if (data["added_mass_kg"] && !data["added_mass_kg"].IsNull()) {
    model.added_mass_kg = data["added_mass_kg"].as<double>();
  }
  if (data["joints"]) {
    const YAML::Node raw_joints = data["joints"];
    if (!raw_joints.IsMap()) {
      throw std::invalid_argument("joints must be a mapping");
    }
    for (auto it = raw_joints.begin(); it != raw_joints.end(); ++it) {
      const std::string key = it->first.as<std::string>();
      int idx = 0;
      if (key.rfind("joint", 0) == 0) {
        idx = std::stoi(key.substr(5));
      } else {
        idx = std::stoi(key);
      }
      if (idx < 1 || idx > 6) {
        throw std::invalid_argument("Joint index out of range: " + key);
      }
      const YAML::Node jn = it->second;
      if (!jn.IsMap()) {
        throw std::invalid_argument("Joint gravity model must be a mapping");
      }
      JointGravityModel jm;
      jm.scale = jn["scale"] ? jn["scale"].as<double>() : 1.0;
      jm.angle_offset = jn["angle_offset"] ? jn["angle_offset"].as<double>() : 0.0;
      jm.bias = jn["bias"] ? jn["bias"].as<double>() : 0.0;
      jm.static_friction = jn["static_friction"] ? jn["static_friction"].as<double>() : 0.0;
      jm.validate();
      model.joints[idx] = jm;
    }
  }
  model.validate();
  return model;
}

void save_gravity_model(const ArmGravityModel & model, const std::string & path) {
  std::filesystem::path target(path);
  if (target.has_parent_path()) {
    std::filesystem::create_directories(target.parent_path());
  }
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "version" << YAML::Value << 1;
  out << YAML::Key << "arm_id" << YAML::Value << model.arm_id;
  out << YAML::Key << "joints" << YAML::Value << YAML::BeginMap;
  for (const auto & kv : model.joints) {
    out << YAML::Key << ("joint" + std::to_string(kv.first)) << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "scale" << YAML::Value << kv.second.scale;
    out << YAML::Key << "angle_offset" << YAML::Value << kv.second.angle_offset;
    out << YAML::Key << "bias" << YAML::Value << kv.second.bias;
    out << YAML::Key << "static_friction" << YAML::Value << kv.second.static_friction;
    out << YAML::EndMap;
  }
  out << YAML::EndMap;
  if (!model.effector_model.empty()) {
    out << YAML::Key << "effector_model" << YAML::Value << model.effector_model;
    out << YAML::Key << "added_mass_kg" << YAML::Value << model.added_mass_kg;
  }
  out << YAML::EndMap;
  std::ofstream ofs(target);
  if (!ofs) {
    throw std::runtime_error("Cannot open gravity model path for writing: " + path);
  }
  ofs << out.c_str() << '\n';
}

}  // namespace piper_x_driver
