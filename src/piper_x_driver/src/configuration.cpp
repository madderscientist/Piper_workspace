#include "piper_x_driver/configuration.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace piper_x_driver {

bool Effector::operator==(const Effector & other) const noexcept {
  return label == other.label && mass_kg == other.mass_kg && com_m == other.com_m &&
         source == other.source && model == other.model;
}

bool ArmConfig::operator==(const ArmConfig & other) const noexcept {
  return id == other.id && name == other.name && namespace_str == other.namespace_str &&
         can_port == other.can_port && end_effector == other.end_effector &&
         gravity_scale == other.gravity_scale && mounting == other.mounting &&
         kp == other.kp && kd == other.kd && s_a == other.s_a &&
         friction_scale == other.friction_scale &&
         friction_config == other.friction_config &&
         gravity_config == other.gravity_config &&
         friction_error_epsilon == other.friction_error_epsilon &&
         friction_velocity_epsilon == other.friction_velocity_epsilon &&
         target_velocity_cutoff_hz == other.target_velocity_cutoff_hz &&
         adaptive_stiffness_b == other.adaptive_stiffness_b &&
         adaptive_stiffness_power == other.adaptive_stiffness_power &&
         offset_ramp_s == other.offset_ramp_s;
}

double bounded(double value, const std::string & name, double lower, double upper) {
  if (!std::isfinite(value) || !(lower <= value && value <= upper)) {
    std::ostringstream oss;
    oss << name << " must be finite and in [" << lower << ", " << upper << "]";
    throw std::invalid_argument(oss.str());
  }
  return value;
}

std::string default_config_path() {
  try {
    auto share = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("piper_x_driver"));
    auto p = share / "config" / "arms.yaml";
    if (std::filesystem::exists(p)) {
      return p.string();
    }
  } catch (const std::exception &) {
  }
#ifdef PIPER_X_DRIVER_SOURCE_DIR
  auto src = std::filesystem::path(PIPER_X_DRIVER_SOURCE_DIR) / "config" / "arms.yaml";
  if (std::filesystem::exists(src)) {
    return src.string();
  }
#endif
  throw std::runtime_error("Cannot locate piper_x_driver config/arms.yaml");
}

std::string json_escape(const std::string & s) {
  std::string out;
  out.reserve(s.size() + 4);
  for (unsigned char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned int>(c));
          out += buf;
        } else {
          out.push_back(static_cast<char>(c));
        }
    }
  }
  return out;
}

std::string json_number(double v) {
  if (!std::isfinite(v)) {
    throw std::invalid_argument("Non-finite float not allowed in JSON");
  }
  if (v == 0.0) {
    return "0.0";
  }
  char buf[40];
  auto [ptr, ec] = std::to_chars(buf, buf + sizeof(buf) - 3, v);
  if (ec != std::errc{}) {
    std::snprintf(buf, sizeof(buf), "%.16g", v);
    ptr = buf + std::strlen(buf);
  }
  bool has_dot_or_exp = false;
  for (const char * p = buf; p < ptr; ++p) {
    if (*p == '.' || *p == 'e' || *p == 'E') {
      has_dot_or_exp = true;
      break;
    }
  }
  if (!has_dot_or_exp) {
    *ptr++ = '.';
    *ptr++ = '0';
  }
  return std::string(buf, static_cast<size_t>(ptr - buf));
}

namespace {

double yaml_strict_number(const YAML::Node & n, const std::string & name) {
  if (!n || !n.IsScalar()) {
    throw std::invalid_argument(name + " must be a finite number");
  }
  const std::string raw = n.Scalar();
  if (raw == "true" || raw == "True" || raw == "TRUE" ||
      raw == "false" || raw == "False" || raw == "FALSE")
  {
    throw std::invalid_argument(name + " must not be boolean");
  }
  return n.as<double>();
}

template<size_t N>
std::array<double, N> yaml_vector(
  const YAML::Node & n, const std::string & name, double lower, double upper)
{
  if (!n || !n.IsSequence() || n.size() != N) {
    throw std::invalid_argument(name + " requires " + std::to_string(N) + " values");
  }
  std::array<double, N> out{};
  for (size_t i = 0; i < N; ++i) {
    out[i] = bounded(yaml_strict_number(n[i], name), name, lower, upper);
  }
  return out;
}

std::string yaml_strict_string(const YAML::Node & n, const std::string & name) {
  if (!n || n.IsNull() || !n.IsScalar()) {
    throw std::invalid_argument(name + " must be a string");
  }
  if (n.Tag() != "!") {
    const std::string raw = n.Scalar();
    if (raw == "true" || raw == "false" || raw == "True" || raw == "False") {
      throw std::invalid_argument(name + " must be a string");
    }
  }
  return n.as<std::string>();
}

bool is_blank(const std::string & s) {
  return std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isspace(c); });
}

std::string trim(const std::string & s) {
  size_t b = 0;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) {
    ++b;
  }
  size_t e = s.size();
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) {
    --e;
  }
  return s.substr(b, e - b);
}

std::string json_vec6(const Vec6 & v) {
  std::string out = "[";
  for (size_t i = 0; i < 6; ++i) {
    if (i > 0) out += ", ";
    out += json_number(v[i]);
  }
  out += "]";
  return out;
}

std::string json_vec3(const Vec3 & v) {
  return "[" + json_number(v[0]) + ", " + json_number(v[1]) + ", " + json_number(v[2]) + "]";
}

std::string json_effector(const Effector & eff) {
  std::ostringstream oss;
  oss << "{\"label\": \"" << json_escape(eff.label) << "\", "
      << "\"mass_kg\": " << json_number(eff.mass_kg) << ", "
      << "\"com_m\": " << json_vec3(eff.com_m) << ", "
      << "\"source\": \"" << json_escape(eff.source) << "\", "
      << "\"model\": \"" << json_escape(eff.model) << "\"}";
  return oss.str();
}

}  // namespace

FleetConfig::FleetConfig(const std::string & raw_path) {
  std::error_code ec;
  auto resolved = std::filesystem::canonical(raw_path, ec);
  if (ec) {
    throw std::invalid_argument("Configuration path not found: " + raw_path);
  }
  path = resolved.string();
  YAML::Node data = YAML::LoadFile(path);
  if (!data || !data.IsMap() || !data["version"] || !data["arms"]) {
    throw std::invalid_argument("Fleet configuration requires version and arms");
  }
  for (auto it = data.begin(); it != data.end(); ++it) {
    const std::string k = it->first.as<std::string>();
    if (k != "version" && k != "effectors" && k != "arms") {
      throw std::invalid_argument("Fleet configuration requires version and arms");
    }
  }
  if (!data["version"].IsScalar() || data["version"].Scalar() != "1") {
    throw std::invalid_argument("Unsupported fleet configuration version");
  }

  static const std::set<std::string> allowed_models = {
    "piper_description.urdf",
    "piper_description_with_teach.urdf",
    "piper_description_with_wrench.urdf",
    "piper_no_gripper_description.urdf",
  };

  if (data["effectors"] && !data["effectors"].IsNull()) {
    has_inline_effectors = true;
    const YAML::Node profiles = data["effectors"];
    if (!profiles.IsMap() || profiles.size() == 0) {
      throw std::invalid_argument("At least one effector profile is required");
    }
    static const std::set<std::string> eff_fields = {
      "label", "mass_kg", "com_m", "source", "model"
    };
    for (auto it = profiles.begin(); it != profiles.end(); ++it) {
      const std::string key = yaml_strict_string(it->first, "effector_key");
      const YAML::Node p = it->second;
      if (!p.IsMap() || p.size() != eff_fields.size()) {
        throw std::invalid_argument("Invalid effector profile");
      }
      for (auto pit = p.begin(); pit != p.end(); ++pit) {
        if (!eff_fields.count(pit->first.as<std::string>())) {
          throw std::invalid_argument("Invalid effector profile");
        }
      }
      const std::string label = yaml_strict_string(p["label"], "label");
      const std::string source = yaml_strict_string(p["source"], "source");
      const std::string model = yaml_strict_string(p["model"], "model");
      if (is_blank(label) || is_blank(source) || is_blank(model)) {
        throw std::invalid_argument("Effector string fields must be non-empty");
      }
      if (!allowed_models.count(model)) {
        throw std::invalid_argument("Unsupported visualization model");
      }
      Effector eff;
      eff.label = label;
      eff.mass_kg = bounded(yaml_strict_number(p["mass_kg"], "mass_kg"), "mass_kg", 0.0, 2.0);
      eff.com_m = yaml_vector<3>(p["com_m"], "com_m", -0.5, 0.5);
      eff.source = source;
      eff.model = model;
      effectors.emplace(key, std::move(eff));
    }
  } else {
    const auto & profiles = terminal_profiles();
    for (const auto & kv : profiles) {
      const RigidBody body = terminal_body(kv.first);
      Effector eff;
      eff.label = kv.second.label;
      eff.mass_kg = bounded(body.mass_kg, "mass_kg", 0.0, 2.0);
      eff.com_m = body.com_m;
      eff.source = kv.second.source;
      eff.model = kv.second.model;
      effectors.emplace(kv.first, std::move(eff));
    }
  }

  const YAML::Node arms_node = data["arms"];
  if (!arms_node.IsSequence() || arms_node.size() != 4) {
    throw std::invalid_argument("Exactly four configured arms are required");
  }

  static const std::set<std::string> known_arm_keys = {
    "id", "name", "namespace", "can_port", "end_effector", "gravity_scale", "mounting",
    "kp", "kd", "s_a", "friction_scale", "friction_config", "gravity_config",
    "friction_error_epsilon", "friction_velocity_epsilon", "target_velocity_cutoff_hz",
    "adaptive_stiffness_b", "adaptive_stiffness_power", "offset_ramp_s",
    "debug_kp", "debug_kd", "adaptive_stiffness_scale"
  };
  static const std::set<std::string> required_base_keys = {
    "id", "name", "namespace", "can_port", "end_effector", "gravity_scale", "mounting"
  };
  static const std::regex id_re("^[A-Za-z_][A-Za-z0-9_]*$");
  static const std::regex ns_re("^(?:/[A-Za-z_][A-Za-z0-9_]*)+$");
  static const std::regex can_re("^[A-Za-z0-9_.-]{1,15}$");

  for (const auto & raw : arms_node) {
    if (!raw.IsMap()) {
      throw std::invalid_argument("Invalid arm configuration fields");
    }
    for (auto it = raw.begin(); it != raw.end(); ++it) {
      if (!known_arm_keys.count(it->first.as<std::string>())) {
        throw std::invalid_argument("Invalid arm configuration fields");
      }
    }
    for (const auto & req : required_base_keys) {
      if (!raw[req]) {
        throw std::invalid_argument("Invalid arm configuration fields");
      }
    }
    Vec6 kp_vec{};
    if (raw["debug_kp"]) {
      const Vec6 legacy_kp = yaml_vector<6>(raw["debug_kp"], "debug_kp", 0.0, 500.0);
      if (!raw["kp"]) {
        kp_vec = legacy_kp;
      }
    }
    if (raw["kp"]) {
      kp_vec = yaml_vector<6>(raw["kp"], "kp", 0.0, 500.0);
    } else if (!raw["debug_kp"]) {
      throw std::invalid_argument("kp requires 6 values");
    }

    std::optional<Vec6> raw_kd;
    if (raw["debug_kd"]) {
      const Vec6 legacy_kd = yaml_vector<6>(raw["debug_kd"], "debug_kd", -100.0, 100.0);
      if (!raw["kd"]) {
        raw_kd = legacy_kd;
      }
    }
    if (raw["kd"]) {
      raw_kd = yaml_vector<6>(raw["kd"], "kd", -100.0, 100.0);
    }

    double sa_val = 2.0;
    if (raw["adaptive_stiffness_scale"]) {
      const double legacy_sa = bounded(
        yaml_strict_number(raw["adaptive_stiffness_scale"], "adaptive_stiffness_scale"),
        "adaptive_stiffness_scale", 0.0, 20.0);
      if (!raw["s_a"]) {
        sa_val = legacy_sa;
      }
    }
    if (raw["s_a"]) {
      sa_val = bounded(yaml_strict_number(raw["s_a"], "s_a"), "s_a", 0.0, 20.0);
    }

    ArmConfig cfg;
    cfg.id = yaml_strict_string(raw["id"], "id");
    cfg.name = yaml_strict_string(raw["name"], "name");
    cfg.namespace_str = yaml_strict_string(raw["namespace"], "namespace");
    cfg.can_port = yaml_strict_string(raw["can_port"], "can_port");
    cfg.end_effector = yaml_strict_string(raw["end_effector"], "end_effector");
    cfg.mounting = yaml_strict_string(raw["mounting"], "mounting");
    if (cfg.id.empty() || cfg.name.empty() || cfg.namespace_str.empty() ||
        cfg.can_port.empty() || cfg.end_effector.empty() || cfg.mounting.empty())
    {
      throw std::invalid_argument("Arm identity strings must be non-empty");
    }
    if (!std::regex_match(cfg.id, id_re)) {
      throw std::invalid_argument("Invalid arm id");
    }
    if (!std::regex_match(cfg.namespace_str, ns_re)) {
      throw std::invalid_argument("Invalid arm namespace");
    }
    if (!std::regex_match(cfg.can_port, can_re)) {
      throw std::invalid_argument("Invalid SocketCAN interface");
    }
    if (cfg.mounting != "upright") {
      throw std::invalid_argument("Only upright mounting is supported by this gravity model");
    }
    if (!effectors.count(cfg.end_effector)) {
      throw std::invalid_argument("Unknown end_effector");
    }

    cfg.gravity_scale = bounded(
      yaml_strict_number(raw["gravity_scale"], "gravity_scale"), "gravity_scale", 0.0, 1.0);
    cfg.friction_scale = raw["friction_scale"]
      ? bounded(
          yaml_strict_number(raw["friction_scale"], "friction_scale"), "friction_scale", 0.0, 1.0)
      : 1.0;
    if (raw["friction_config"] && !raw["friction_config"].IsNull()) {
      cfg.friction_config = trim(yaml_strict_string(raw["friction_config"], "friction_config"));
    }
    if (raw["gravity_config"] && !raw["gravity_config"].IsNull()) {
      cfg.gravity_config = trim(yaml_strict_string(raw["gravity_config"], "gravity_config"));
    }

    std::optional<ArmFrictionModel> f_model;
    if (!cfg.friction_config.empty()) {
      std::filesystem::path fp(cfg.friction_config);
      if (!fp.is_absolute()) {
        fp = std::filesystem::path(path).parent_path() / fp;
      }
      std::error_code fec;
      auto fcanon = std::filesystem::canonical(fp, fec);
      if (fec) {
        throw std::invalid_argument("friction_config file not found: " + cfg.friction_config);
      }
      f_model = load_friction_model(fcanon.string());
    }
    friction_models[cfg.id] = f_model;

    std::optional<ArmGravityModel> g_model;
    if (!cfg.gravity_config.empty()) {
      std::filesystem::path gp(cfg.gravity_config);
      if (!gp.is_absolute()) {
        gp = std::filesystem::path(path).parent_path() / gp;
      }
      std::error_code gec;
      auto gcanon = std::filesystem::canonical(gp, gec);
      if (gec) {
        throw std::invalid_argument("gravity_config file not found: " + cfg.gravity_config);
      }
      g_model = load_gravity_model(gcanon.string());
    }
    ArmGravityModel base_g = g_model.value_or(ArmGravityModel{cfg.id, {}, "", 0.0});
    ArmGravityModel candidate = base_g.for_effector(effectors.at(cfg.end_effector).model);
    if (g_model.has_value() || !candidate.effector_model.empty()) {
      g_model = candidate;
    }
    gravity_models[cfg.id] = g_model;

    const Vec6 auto_kd = max_critical_kd(
      kp_vec, cfg.end_effector, g_model.has_value() ? &(*g_model) : nullptr);
    cfg.kp = kp_vec;
    if (raw_kd.has_value()) {
      for (size_t i = 0; i < 6; ++i) {
        cfg.kd[i] = ((*raw_kd)[i] < 0.0) ? auto_kd[i] : (*raw_kd)[i];
      }
    } else {
      cfg.kd = auto_kd;
    }
    cfg.s_a = sa_val;
    cfg.friction_error_epsilon = raw["friction_error_epsilon"]
      ? bounded(
          yaml_strict_number(raw["friction_error_epsilon"], "friction_error_epsilon"),
          "friction_error_epsilon", 0.0, 10.0)
      : 0.05;
    cfg.friction_velocity_epsilon = raw["friction_velocity_epsilon"]
      ? bounded(
          yaml_strict_number(raw["friction_velocity_epsilon"], "friction_velocity_epsilon"),
          "friction_velocity_epsilon", 0.0, 100.0)
      : 0.02;
    cfg.target_velocity_cutoff_hz = raw["target_velocity_cutoff_hz"]
      ? bounded(
          yaml_strict_number(raw["target_velocity_cutoff_hz"], "target_velocity_cutoff_hz"),
          "target_velocity_cutoff_hz", 0.0, 1000.0)
      : 5.0;
    if (cfg.target_velocity_cutoff_hz <= 0.0) {
      throw std::invalid_argument("target_velocity_cutoff_hz must be finite and positive");
    }
    cfg.adaptive_stiffness_b = raw["adaptive_stiffness_b"]
      ? bounded(
          yaml_strict_number(raw["adaptive_stiffness_b"], "adaptive_stiffness_b"),
          "adaptive_stiffness_b", 0.0, 10.0)
      : 0.08;
    if (cfg.adaptive_stiffness_b <= 0.0) {
      throw std::invalid_argument("adaptive_stiffness_b must be finite and positive");
    }
    cfg.adaptive_stiffness_power = raw["adaptive_stiffness_power"]
      ? bounded(
          yaml_strict_number(raw["adaptive_stiffness_power"], "adaptive_stiffness_power"),
          "adaptive_stiffness_power", 0.0, 20.0)
      : 3.0;
    if (cfg.adaptive_stiffness_power <= 0.0) {
      throw std::invalid_argument("adaptive_stiffness_power must be finite and positive");
    }
    cfg.offset_ramp_s = raw["offset_ramp_s"]
      ? bounded(
          yaml_strict_number(raw["offset_ramp_s"], "offset_ramp_s"),
          "offset_ramp_s", 0.0, 30.0)
      : 2.0;

    arms.push_back(std::move(cfg));
  }

  std::set<std::string> ids, namespaces, ports;
  for (const auto & a : arms) {
    ids.insert(a.id);
    namespaces.insert(a.namespace_str);
    ports.insert(a.can_port);
  }
  if (ids.size() != 4) {
    throw std::invalid_argument("Duplicate id");
  }
  if (namespaces.size() != 4) {
    throw std::invalid_argument("Duplicate namespace");
  }
  if (ports.size() != 4) {
    throw std::invalid_argument("Duplicate can_port");
  }
}

const ArmConfig & FleetConfig::arm(const std::string & identifier) const {
  for (const auto & a : arms) {
    if (a.id == identifier) {
      return a;
    }
  }
  throw std::invalid_argument("Unknown configured arm: " + identifier);
}

const ArmFrictionModel * FleetConfig::friction_model(const std::string & arm_id) const {
  auto it = friction_models.find(arm_id);
  if (it != friction_models.end() && it->second.has_value()) {
    return &(*it->second);
  }
  return nullptr;
}

const ArmGravityModel * FleetConfig::gravity_model(const std::string & arm_id) const {
  auto it = gravity_models.find(arm_id);
  if (it != gravity_models.end() && it->second.has_value()) {
    return &(*it->second);
  }
  return nullptr;
}

std::string FleetConfig::public_json(const ArmConfig & a) const {
  std::ostringstream oss;
  oss << "{"
      << "\"id\": \"" << json_escape(a.id) << "\", "
      << "\"name\": \"" << json_escape(a.name) << "\", "
      << "\"namespace\": \"" << json_escape(a.namespace_str) << "\", "
      << "\"can_port\": \"" << json_escape(a.can_port) << "\", "
      << "\"end_effector\": \"" << json_escape(a.end_effector) << "\", "
      << "\"gravity_scale\": " << json_number(a.gravity_scale) << ", "
      << "\"mounting\": \"" << json_escape(a.mounting) << "\", "
      << "\"kp\": " << json_vec6(a.kp) << ", "
      << "\"kd\": " << json_vec6(a.kd) << ", "
      << "\"s_a\": " << json_number(a.s_a) << ", "
      << "\"friction_scale\": " << json_number(a.friction_scale) << ", "
      << "\"friction_config\": \"" << json_escape(a.friction_config) << "\", "
      << "\"gravity_config\": \"" << json_escape(a.gravity_config) << "\", "
      << "\"friction_error_epsilon\": " << json_number(a.friction_error_epsilon) << ", "
      << "\"friction_velocity_epsilon\": " << json_number(a.friction_velocity_epsilon) << ", "
      << "\"target_velocity_cutoff_hz\": " << json_number(a.target_velocity_cutoff_hz) << ", "
      << "\"adaptive_stiffness_b\": " << json_number(a.adaptive_stiffness_b) << ", "
      << "\"adaptive_stiffness_power\": " << json_number(a.adaptive_stiffness_power) << ", "
      << "\"offset_ramp_s\": " << json_number(a.offset_ramp_s) << ", "
      << "\"config_file\": \"" << json_escape(path) << "\", "
      << "\"debug_kp\": " << json_vec6(a.kp) << ", "
      << "\"debug_kd\": " << json_vec6(a.kd) << ", "
      << "\"adaptive_stiffness_scale\": " << json_number(a.s_a) << ", "
      << "\"effector\": " << json_effector(effectors.at(a.end_effector)) << ", "
      << "\"effectors\": {";
  bool first = true;
  for (const auto & kv : effectors) {
    if (!first) oss << ", ";
    first = false;
    oss << "\"" << json_escape(kv.first) << "\": " << json_effector(kv.second);
  }
  oss << "}";
  const ArmGravityModel * gm = gravity_model(a.id);
  if (gm != nullptr && !gm->effector_model.empty()) {
    const Effector & eff = effectors.at(a.end_effector);
    const RigidBody body = gm->effector_body(eff.mass_kg, eff.com_m);
    oss << ", \"gravity_payload\": {"
        << "\"added_mass_kg\": " << json_number(gm->added_mass_kg) << ", "
        << "\"total_mass_kg\": " << json_number(body.mass_kg) << ", "
        << "\"com_m\": " << json_vec3(body.com_m) << ", "
        << "\"inertia_kg_m2\": [";
    for (int r = 0; r < 3; ++r) {
      if (r > 0) oss << ", ";
      oss << "[" << json_number(body.inertia[r][0]) << ", "
          << json_number(body.inertia[r][1]) << ", "
          << json_number(body.inertia[r][2]) << "]";
    }
    oss << "]}";
  }
  oss << "}";
  return oss.str();
}

FleetConfig FleetConfig::save_arm(
  const ArmConfig & current,
  const std::string & end_effector,
  double gravity_scale,
  std::optional<double> friction_scale) const
{
  bounded(gravity_scale, "gravity_scale", 0.0, 1.0);
  const double next_friction = friction_scale.has_value()
    ? bounded(*friction_scale, "friction_scale", 0.0, 1.0)
    : current.friction_scale;
  if (!effectors.count(end_effector)) {
    throw std::invalid_argument("Unknown end_effector");
  }

  const std::string lock_path = path + ".lock";
  const int lock_fd = ::open(lock_path.c_str(), O_CREAT | O_RDWR, 0666);
  if (lock_fd < 0) {
    throw std::runtime_error("Cannot open lock file: " + lock_path);
  }
  struct LockGuard {
    int fd;
    ~LockGuard() {
      if (fd >= 0) {
        ::flock(fd, LOCK_UN);
        ::close(fd);
      }
    }
  } guard{lock_fd};
  if (::flock(lock_fd, LOCK_EX) < 0) {
    throw std::runtime_error("Cannot lock configuration file");
  }

  FleetConfig latest(path);
  if (!(latest.arm(current.id) == current) || latest.effectors != effectors) {
    throw std::invalid_argument("Configuration changed externally; restart drivers before saving");
  }

  YAML::Node raw_doc = YAML::LoadFile(path);
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "version" << YAML::Value << 1;
  out << YAML::Key << "arms" << YAML::Value << YAML::BeginSeq;
  for (const auto & a : latest.arms) {
    ArmConfig item = a;
    if (item.id == current.id) {
      item.end_effector = end_effector;
      item.gravity_scale = gravity_scale;
      item.friction_scale = next_friction;
    }
    out << YAML::BeginMap;
    out << YAML::Key << "id" << YAML::Value << item.id;
    out << YAML::Key << "name" << YAML::Value << item.name;
    out << YAML::Key << "namespace" << YAML::Value << item.namespace_str;
    out << YAML::Key << "can_port" << YAML::Value << item.can_port;
    out << YAML::Key << "end_effector" << YAML::Value << item.end_effector;
    out << YAML::Key << "gravity_scale" << YAML::Value << item.gravity_scale;
    out << YAML::Key << "mounting" << YAML::Value << item.mounting;
    out << YAML::Key << "kp" << YAML::Value << std::vector<double>(item.kp.begin(), item.kp.end());
    out << YAML::Key << "kd" << YAML::Value << std::vector<double>(item.kd.begin(), item.kd.end());
    out << YAML::Key << "s_a" << YAML::Value << item.s_a;
    out << YAML::Key << "friction_scale" << YAML::Value << item.friction_scale;
    out << YAML::Key << "friction_config" << YAML::Value << item.friction_config;
    out << YAML::Key << "gravity_config" << YAML::Value << item.gravity_config;
    out << YAML::Key << "friction_error_epsilon" << YAML::Value << item.friction_error_epsilon;
    out << YAML::Key << "friction_velocity_epsilon" << YAML::Value << item.friction_velocity_epsilon;
    out << YAML::Key << "target_velocity_cutoff_hz" << YAML::Value << item.target_velocity_cutoff_hz;
    out << YAML::Key << "adaptive_stiffness_b" << YAML::Value << item.adaptive_stiffness_b;
    out << YAML::Key << "adaptive_stiffness_power" << YAML::Value << item.adaptive_stiffness_power;
    out << YAML::Key << "offset_ramp_s" << YAML::Value << item.offset_ramp_s;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  if (latest.has_inline_effectors && raw_doc["effectors"]) {
    out << YAML::Key << "effectors" << YAML::Value << raw_doc["effectors"];
  }
  out << YAML::EndMap;

  std::filesystem::path target_path(path);
  std::string tmpl = (target_path.parent_path() / ".arms-XXXXXX").string();
  std::vector<char> buf(tmpl.begin(), tmpl.end());
  buf.push_back('\0');
  const int tmp_fd = ::mkstemp(buf.data());
  if (tmp_fd < 0) {
    throw std::runtime_error("Cannot create temporary configuration file");
  }
  const std::string tmp_path(buf.data());
  try {
    const std::string content = std::string(out.c_str()) + "\n";
    const ssize_t written = ::write(tmp_fd, content.data(), content.size());
    if (written < 0 || static_cast<size_t>(written) != content.size()) {
      ::close(tmp_fd);
      throw std::runtime_error("Failed to write temporary configuration file");
    }
    ::fsync(tmp_fd);
    ::close(tmp_fd);

    FleetConfig updated(tmp_path);
    struct stat st{};
    if (::stat(path.c_str(), &st) == 0) {
      ::chmod(tmp_path.c_str(), st.st_mode & 0777);
    }
    if (::rename(tmp_path.c_str(), path.c_str()) != 0) {
      throw std::runtime_error("Failed to atomically replace configuration file");
    }
    updated.path = path;
    return updated;
  } catch (...) {
    ::unlink(tmp_path.c_str());
    throw;
  }
}

}  // namespace piper_x_driver
