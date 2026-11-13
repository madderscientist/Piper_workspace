#include "piper_x_driver/friction.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace piper_x_driver {

double interp_1d(
  double x,
  const std::vector<double> & xs,
  const std::vector<double> & ys)
{
  const size_t n = xs.size();
  if (n == 0 || ys.size() != n) {
    return 0.0;
  }
  if (n == 1 || x <= xs.front()) {
    return ys.front();
  }
  if (x >= xs.back()) {
    return ys.back();
  }
  auto it = std::upper_bound(xs.begin(), xs.end(), x);
  const size_t idx = static_cast<size_t>(std::distance(xs.begin(), it));
  const double x0 = xs[idx - 1];
  const double x1 = xs[idx];
  const double y0 = ys[idx - 1];
  const double y1 = ys[idx];
  const double span = x1 - x0;
  if (span <= 1e-12) {
    return y0;
  }
  const double alpha = (x - x0) / span;
  return y0 + alpha * (y1 - y0);
}

void SpatialHarmonic::validate() const {
  if (!std::isfinite(omega) || !std::isfinite(a) || !std::isfinite(b) ||
      !std::isfinite(c) || !std::isfinite(d))
  {
    throw std::invalid_argument("Harmonic parameters must be finite numbers");
  }
  if (omega <= 0.0) {
    throw std::invalid_argument("Harmonic omega must be positive");
  }
}

double SpatialHarmonic::cogging(double theta) const noexcept {
  const double phase = omega * theta;
  return a * std::cos(phase) + b * std::sin(phase);
}

double SpatialHarmonic::modulation(double theta) const noexcept {
  const double phase = omega * theta;
  return c * std::cos(phase) + d * std::sin(phase);
}

double SpatialHarmonic::cogging_amplitude() const noexcept {
  return std::hypot(a, b);
}

double SpatialHarmonic::modulation_amplitude() const noexcept {
  return std::hypot(c, d);
}

void JointFrictionModel::validate() const {
  for (const auto & kv : {
    std::pair{"fc", fc}, {"fs", fs}, {"vs", vs}, {"delta", delta},
    {"fv", fv}, {"v0", v0}, {"vr", vr}, {"load_ratio", load_ratio},
    {"error_epsilon", error_epsilon}, {"velocity_epsilon", velocity_epsilon}})
  {
    if (!std::isfinite(kv.second)) {
      throw std::invalid_argument(std::string(kv.first) + " must be finite");
    }
  }
  if (load_ratio < 0.0 || error_epsilon < 0.0 || velocity_epsilon < 0.0) {
    throw std::invalid_argument("load_ratio/epsilons must be non-negative");
  }
  if (vs <= 0.0 || v0 <= 0.0 || vr <= 0.0 || delta <= 0.0) {
    throw std::invalid_argument("vs, v0, vr, delta must be positive");
  }
  for (const auto & h : harmonics) {
    h.validate();
  }
}

double JointFrictionModel::cogging_torque(double theta) const noexcept {
  double acc = 0.0;
  for (const auto & h : harmonics) {
    acc += h.cogging(theta);
  }
  return acc;
}

double JointFrictionModel::modulation_torque(double theta) const noexcept {
  double acc = 0.0;
  for (const auto & h : harmonics) {
    acc += h.modulation(theta);
  }
  return acc;
}

std::pair<double, double> JointFrictionModel::mod_and_cogging(double theta) const noexcept {
  if (harmonics.empty()) {
    return {0.0, 0.0};
  }
  double mod = 0.0;
  double cog = 0.0;
  for (const auto & h : harmonics) {
    const double phase = h.omega * theta;
    double sn = 0.0, cs = 0.0;
#if defined(__GLIBC__) || defined(_GNU_SOURCE)
    ::sincos(phase, &sn, &cs);
#else
    sn = std::sin(phase);
    cs = std::cos(phase);
#endif
    cog += h.a * cs + h.b * sn;
    mod += h.c * cs + h.d * sn;
  }
  return {mod, cog};
}

double JointFrictionModel::envelope(double velocity) const noexcept {
  const double ratio = velocity / vr;
  return std::exp(-(ratio * ratio));
}

double JointFrictionModel::stribeck_magnitude(double speed) const noexcept {
  const double s = std::abs(speed);
  const double stribeck_term = (fs - fc) * std::exp(-std::pow(s / vs, delta));
  return fc + stribeck_term + fv * s;
}

double JointFrictionModel::stribeck_torque(double velocity) const noexcept {
  const double s = std::abs(velocity);
  const double dry = fc + (fs - fc) * std::exp(-std::pow(s / vs, delta));
  return dry * std::tanh(velocity / v0) + fv * velocity;
}

double JointFrictionModel::stiction_torque(
  double theta,
  double position_error,
  double target_velocity,
  double gravity_torque,
  double static_friction,
  std::optional<double> error_eps,
  std::optional<double> velocity_eps) const noexcept
{
  const double eps_e = error_eps.value_or(error_epsilon);
  const double eps_v = velocity_eps.value_or(velocity_epsilon);
  double drive = 0.0;
  if (eps_e > 0.0) {
    drive += position_error / eps_e;
  }
  if (eps_v > 0.0) {
    drive += target_velocity / eps_v;
  }
  if (drive == 0.0) {
    return 0.0;
  }
  const double floor_val = (static_friction > 0.0) ? static_friction : std::max(fs, fc);
  const double magnitude = std::max(
    0.0, load_ratio * std::abs(gravity_torque) + floor_val + modulation_torque(theta));
  return magnitude * std::tanh(drive);
}

double JointFrictionModel::torque(
  double theta,
  double velocity,
  double position_error,
  double target_velocity,
  double gravity_torque,
  double static_friction,
  std::optional<double> error_eps,
  std::optional<double> velocity_eps,
  bool active_impedance,
  double cogging_scale,
  double friction_scale) const noexcept
{
  const auto [mod_raw, cog_raw] = mod_and_cogging(theta);
  const double eps_e = error_eps.value_or(error_epsilon);
  const double eps_v = velocity_eps.value_or(velocity_epsilon);
  double drive = 0.0;
  if (eps_e > 0.0) {
    drive += position_error / eps_e;
  }
  if (eps_v > 0.0) {
    drive += target_velocity / eps_v;
  }
  double stiction = 0.0;
  if (drive != 0.0) {
    const double floor_val = (static_friction > 0.0) ? static_friction : std::max(fs, fc);
    const double magnitude = std::max(
      0.0, load_ratio * std::abs(gravity_torque) + floor_val + mod_raw);
    stiction = magnitude * std::tanh(drive);
  }
  const double s = std::abs(velocity);
  if (s == 0.0) {
    return friction_scale * stiction;
  }
  const double env = envelope(velocity);
  double v0_eff = v0;
  if (!active_impedance && v0 <= 0.005) {
    v0_eff = std::max({
      v0,
      UNDAMPED_MIN_V0,
      (std::max(fs, fc) + std::abs(mod_raw)) / UNDAMPED_MAX_ZERO_SLOPE
    });
  }
  const double tanh_v = std::tanh(velocity / v0_eff);
  const double motion_gate = std::abs(tanh_v);
  const double dry =
    load_ratio * std::abs(gravity_torque) +
    fc +
    (fs - fc) * std::exp(-((delta == 2.0) ? ((s / vs) * (s / vs)) : std::pow(s / vs, delta)));
  const double mod = env * mod_raw;
  const double kinetic = std::max(0.0, dry + mod) * tanh_v + fv * velocity;
  const double directional = (1.0 - motion_gate) * stiction + motion_gate * kinetic;
  const double cog = env * motion_gate * cog_raw;
  return friction_scale * directional + cogging_scale * cog;
}

void GripperFrictionModel::validate() const {
  for (const auto & kv : {
    std::pair{"fc", fc}, {"fs", fs}, {"vs", vs}, {"delta", delta},
    {"fv", fv}, {"v0", v0}, {"vr", vr}, {"kp_equiv", kp_equiv}, {"kv_equiv", kv_equiv}})
  {
    if (!std::isfinite(kv.second)) {
      throw std::invalid_argument(std::string(kv.first) + " must be finite");
    }
  }
  if (vs <= 0.0 || v0 <= 0.0 || vr <= 0.0 || delta <= 0.0 || kp_equiv <= 0.0 || kv_equiv <= 0.0) {
    throw std::invalid_argument("Gripper positive parameters must be finite and positive");
  }
  const size_t n = w_grid.size();
  if (bias_grid.size() != n || modulation_grid.size() != n) {
    throw std::invalid_argument("w_grid, bias_grid, and modulation_grid must have identical lengths");
  }
  for (size_t i = 0; i < n; ++i) {
    if (!std::isfinite(w_grid[i]) || !std::isfinite(bias_grid[i]) ||
        !std::isfinite(modulation_grid[i]))
    {
      throw std::invalid_argument("grid entries must be finite numbers");
    }
    if (i > 0 && w_grid[i] <= w_grid[i - 1]) {
      throw std::invalid_argument("w_grid must be strictly increasing");
    }
  }
}

double GripperFrictionModel::bias_force(double width) const noexcept {
  return interp_1d(width, w_grid, bias_grid);
}

double GripperFrictionModel::modulation_force(double width) const noexcept {
  return interp_1d(width, w_grid, modulation_grid);
}

double GripperFrictionModel::local_fc(double width) const noexcept {
  return std::max(0.0, fc + modulation_force(width));
}

double GripperFrictionModel::envelope(double velocity) const noexcept {
  const double ratio = velocity / vr;
  return std::exp(-(ratio * ratio));
}

double GripperFrictionModel::stribeck_magnitude(double speed) const noexcept {
  const double s = std::abs(speed);
  const double stribeck_term = (fs - fc) * std::exp(-std::pow(s / vs, delta));
  return fc + stribeck_term + fv * s;
}

double GripperFrictionModel::force(
  double width,
  double velocity,
  double bias_scale,
  double friction_scale) const noexcept
{
  const double s = std::abs(velocity);
  if (s == 0.0) {
    return 0.0;
  }
  const double env = envelope(velocity);
  const double motion_gate = std::tanh(s / v0);
  const double dry = fc + (fs - fc) * std::exp(-((delta == 2.0) ? ((s / vs) * (s / vs)) : std::pow(s / vs, delta)));
  const double mod = env * modulation_force(width);
  const double directional = std::max(0.0, dry + mod) * std::tanh(velocity / v0) + fv * velocity;
  const double bias = motion_gate * bias_force(width);
  return friction_scale * directional + bias_scale * bias;
}

std::tuple<double, double, double> GripperFrictionModel::compensate_command(
  double w_cmd,
  double effort_cmd,
  double w_meas,
  double velocity,
  double scale) const noexcept
{
  const double base_effort = std::max(0.0, effort_cmd);
  if (scale <= 0.0 || velocity == 0.0) {
    return {w_cmd, base_effort, 0.0};
  }
  const double f_comp = scale * force(w_meas, velocity);
  if (std::abs(f_comp) < 1e-3) {
    return {w_cmd, base_effort, 0.0};
  }
  const double effort_out = base_effort + std::abs(f_comp);
  const double w_out = (effort_cmd > 0.0)
    ? w_cmd
    : (w_meas + f_comp / kp_equiv + velocity / kv_equiv);
  return {w_out, effort_out, f_comp};
}

JointFrictionModel ArmFrictionModel::joint(int index) const {
  if (index < 1 || index > 6) {
    throw std::invalid_argument("Joint index must be in 1..6");
  }
  auto it = joints.find(index);
  return (it != joints.end()) ? it->second : JointFrictionModel{};
}

ArmFrictionModel ArmFrictionModel::with_joint(
  int index, const JointFrictionModel & model) const
{
  if (index < 1 || index > 6) {
    throw std::invalid_argument("Joint index must be in 1..6");
  }
  model.validate();
  ArmFrictionModel copy = *this;
  copy.joints[index] = model;
  return copy;
}

ArmFrictionModel ArmFrictionModel::with_gripper(
  std::optional<GripperFrictionModel> model) const
{
  if (model.has_value()) {
    model->validate();
  }
  ArmFrictionModel copy = *this;
  copy.gripper = std::move(model);
  return copy;
}

double ArmFrictionModel::gripper_force(
  double width,
  double velocity,
  double bias_scale,
  double friction_scale) const noexcept
{
  if (!gripper.has_value()) {
    return 0.0;
  }
  return gripper->force(width, velocity, bias_scale, friction_scale);
}

Vec6 ArmFrictionModel::torques(
  const Vec6 & positions,
  const Vec6 & velocities,
  const Vec6 & position_errors,
  const Vec6 & target_velocities,
  const Vec6 & gravity_torques,
  const Vec6 & static_frictions,
  std::optional<double> error_epsilon,
  std::optional<double> velocity_epsilon,
  const std::array<bool, 6> & active_impedances,
  double cogging_scale,
  double friction_scale) const
{
  Vec6 out{};
  for (int i = 0; i < 6; ++i) {
    auto it = joints.find(i + 1);
    static const JointFrictionModel default_joint{};
    const JointFrictionModel & jm = (it != joints.end()) ? it->second : default_joint;
    out[i] = jm.torque(
      positions[i], velocities[i],
      position_errors[i], target_velocities[i],
      gravity_torques[i], static_frictions[i],
      error_epsilon, velocity_epsilon,
      active_impedances[i], cogging_scale, friction_scale);
  }
  return out;
}

ArmFrictionModel load_friction_model(const std::string & path) {
  YAML::Node data = YAML::LoadFile(path);
  if (!data || !data.IsMap()) {
    throw std::invalid_argument("Unsupported friction model format or version");
  }
  const int version = data["version"] ? data["version"].as<int>() : 1;
  if (version != 1) {
    throw std::invalid_argument("Unsupported friction model format or version");
  }
  ArmFrictionModel model;
  model.arm_id = data["arm_id"] ? data["arm_id"].as<std::string>() : "arm_1";
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
        throw std::invalid_argument("Joint friction model must be a mapping");
      }
      JointFrictionModel jm;
      jm.fc = jn["fc"] ? jn["fc"].as<double>() : 0.0;
      jm.fs = jn["fs"] ? jn["fs"].as<double>() : 0.0;
      jm.vs = jn["vs"] ? jn["vs"].as<double>() : 0.1;
      jm.delta = jn["delta"] ? jn["delta"].as<double>() : 2.0;
      jm.fv = jn["fv"] ? jn["fv"].as<double>() : 0.0;
      jm.v0 = jn["v0"] ? jn["v0"].as<double>() : 0.01;
      jm.vr = jn["vr"] ? jn["vr"].as<double>() : 0.5;
      jm.load_ratio = jn["load_ratio"] ? jn["load_ratio"].as<double>() : 0.0;
      jm.error_epsilon = jn["error_epsilon"] ? jn["error_epsilon"].as<double>() : 0.05;
      jm.velocity_epsilon = jn["velocity_epsilon"] ? jn["velocity_epsilon"].as<double>() : 0.02;
      if (jn["harmonics"]) {
        if (!jn["harmonics"].IsSequence()) {
          throw std::invalid_argument("harmonics must be a list");
        }
        for (const auto & hn : jn["harmonics"]) {
          if (!hn.IsMap() || !hn["omega"]) {
            throw std::invalid_argument("Invalid harmonic entry");
          }
          SpatialHarmonic sh;
          sh.omega = hn["omega"].as<double>();
          sh.a = hn["a"] ? hn["a"].as<double>() : 0.0;
          sh.b = hn["b"] ? hn["b"].as<double>() : 0.0;
          sh.c = hn["c"] ? hn["c"].as<double>() : 0.0;
          sh.d = hn["d"] ? hn["d"].as<double>() : 0.0;
          sh.validate();
          jm.harmonics.push_back(sh);
        }
      }
      jm.validate();
      model.joints[idx] = std::move(jm);
    }
  }
  if (data["gripper"] && data["gripper"].IsMap()) {
    const YAML::Node gn = data["gripper"];
    GripperFrictionModel gm;
    gm.fc = gn["fc"] ? gn["fc"].as<double>() : 0.0;
    gm.fs = gn["fs"] ? gn["fs"].as<double>() : 0.0;
    gm.vs = gn["vs"] ? gn["vs"].as<double>() : 0.015;
    gm.delta = gn["delta"] ? gn["delta"].as<double>() : 2.0;
    gm.fv = gn["fv"] ? gn["fv"].as<double>() : 0.0;
    gm.v0 = gn["v0"] ? gn["v0"].as<double>() : 0.002;
    gm.vr = gn["vr"] ? gn["vr"].as<double>() : 0.08;
    gm.kp_equiv = gn["kp_equiv"] ? gn["kp_equiv"].as<double>() : 45.0;
    gm.kv_equiv = gn["kv_equiv"] ? gn["kv_equiv"].as<double>() : 12.5;
    if (gn["w_grid"]) {
      gm.w_grid = gn["w_grid"].as<std::vector<double>>();
    }
    if (gn["bias_grid"]) {
      gm.bias_grid = gn["bias_grid"].as<std::vector<double>>();
    }
    if (gn["modulation_grid"]) {
      gm.modulation_grid = gn["modulation_grid"].as<std::vector<double>>();
    }
    gm.validate();
    model.gripper = std::move(gm);
  }
  return model;
}

void save_friction_model(const ArmFrictionModel & model, const std::string & path) {
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
    const auto & jm = kv.second;
    out << YAML::Key << ("joint" + std::to_string(kv.first)) << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "fc" << YAML::Value << jm.fc;
    out << YAML::Key << "fs" << YAML::Value << jm.fs;
    out << YAML::Key << "vs" << YAML::Value << jm.vs;
    out << YAML::Key << "delta" << YAML::Value << jm.delta;
    out << YAML::Key << "fv" << YAML::Value << jm.fv;
    out << YAML::Key << "v0" << YAML::Value << jm.v0;
    out << YAML::Key << "vr" << YAML::Value << jm.vr;
    out << YAML::Key << "harmonics" << YAML::Value << YAML::BeginSeq;
    for (const auto & h : jm.harmonics) {
      out << YAML::BeginMap;
      out << YAML::Key << "omega" << YAML::Value << h.omega;
      out << YAML::Key << "a" << YAML::Value << h.a;
      out << YAML::Key << "b" << YAML::Value << h.b;
      out << YAML::Key << "c" << YAML::Value << h.c;
      out << YAML::Key << "d" << YAML::Value << h.d;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    if (jm.load_ratio != 0.0) {
      out << YAML::Key << "load_ratio" << YAML::Value << jm.load_ratio;
    }
    if (jm.error_epsilon != 0.05) {
      out << YAML::Key << "error_epsilon" << YAML::Value << jm.error_epsilon;
    }
    if (jm.velocity_epsilon != 0.02) {
      out << YAML::Key << "velocity_epsilon" << YAML::Value << jm.velocity_epsilon;
    }
    out << YAML::EndMap;
  }
  out << YAML::EndMap;
  if (model.gripper.has_value()) {
    const auto & gm = *model.gripper;
    out << YAML::Key << "gripper" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "fc" << YAML::Value << gm.fc;
    out << YAML::Key << "fs" << YAML::Value << gm.fs;
    out << YAML::Key << "vs" << YAML::Value << gm.vs;
    out << YAML::Key << "delta" << YAML::Value << gm.delta;
    out << YAML::Key << "fv" << YAML::Value << gm.fv;
    out << YAML::Key << "v0" << YAML::Value << gm.v0;
    out << YAML::Key << "vr" << YAML::Value << gm.vr;
    out << YAML::Key << "kp_equiv" << YAML::Value << gm.kp_equiv;
    out << YAML::Key << "kv_equiv" << YAML::Value << gm.kv_equiv;
    out << YAML::Key << "w_grid" << YAML::Value << gm.w_grid;
    out << YAML::Key << "bias_grid" << YAML::Value << gm.bias_grid;
    out << YAML::Key << "modulation_grid" << YAML::Value << gm.modulation_grid;
    out << YAML::EndMap;
  }
  out << YAML::EndMap;
  std::ofstream ofs(target);
  if (!ofs) {
    throw std::runtime_error("Cannot open friction model path for writing: " + path);
  }
  ofs << out.c_str() << '\n';
}

}  // namespace piper_x_driver
