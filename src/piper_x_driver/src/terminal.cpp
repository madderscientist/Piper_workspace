#include "piper_x_driver/terminal.hpp"

#include <filesystem>
#include <mutex>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace piper_x_driver {

RigidBody combine_bodies(const std::vector<RigidBody> & bodies) {
  if (bodies.empty()) {
    return RigidBody{};
  }
  double mass = 0.0;
  Vec3 moment{{0.0, 0.0, 0.0}};
  for (const auto & b : bodies) {
    mass += b.mass_kg;
    moment[0] += b.mass_kg * b.com_m[0];
    moment[1] += b.mass_kg * b.com_m[1];
    moment[2] += b.mass_kg * b.com_m[2];
  }
  Vec3 center{{0.0, 0.0, 0.0}};
  if (mass != 0.0) {
    center[0] = moment[0] / mass;
    center[1] = moment[1] / mass;
    center[2] = moment[2] / mass;
  }
  Mat3 inertia{{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};
  for (const auto & b : bodies) {
    const double dx = b.com_m[0] - center[0];
    const double dy = b.com_m[1] - center[1];
    const double dz = b.com_m[2] - center[2];
    const double d2 = dx * dx + dy * dy + dz * dz;
    const double d[3] = {dx, dy, dz};
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        const double eye = (r == c) ? d2 : 0.0;
        inertia[r][c] += b.inertia[r][c] + b.mass_kg * (eye - d[r] * d[c]);
      }
    }
  }
  return RigidBody{mass, center, inertia};
}

RigidBody CompositeTerminal::body() const {
  std::vector<RigidBody> parts;
  parts.reserve(components.size());
  for (const auto & kv : components) {
    parts.push_back(kv.second);
  }
  return combine_bodies(parts);
}

std::string default_effectors_yaml_path() {
  try {
    auto share = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("piper_x_driver"));
    auto p = share / "config" / "effectors.yaml";
    if (std::filesystem::is_regular_file(p)) {
      return p.string();
    }
  } catch (const std::exception &) {
  }
#ifdef PIPER_X_DRIVER_SOURCE_DIR
  auto src = std::filesystem::path(PIPER_X_DRIVER_SOURCE_DIR) / "config" / "effectors.yaml";
  if (std::filesystem::is_regular_file(src)) {
    return src.string();
  }
#endif
  throw std::runtime_error("Cannot locate piper_x_driver config/effectors.yaml");
}

namespace {

std::map<std::string, TerminalProfileEntry> load_profiles_from_yaml(const std::string & path) {
  YAML::Node root = YAML::LoadFile(path);
  YAML::Node effs = root["effectors"];
  if (!effs || !effs.IsMap()) {
    throw std::runtime_error("Invalid effectors.yaml: missing effectors map");
  }
  std::map<std::string, TerminalProfileEntry> out;
  for (auto it = effs.begin(); it != effs.end(); ++it) {
    const std::string key = it->first.as<std::string>();
    const YAML::Node node = it->second;
    TerminalProfileEntry entry;
    entry.key = key;
    entry.label = node["label"].as<std::string>();
    entry.model = node["model"].as<std::string>();
    entry.source = node["source"].as<std::string>();
    if (node["payload_link"] && !node["payload_link"].IsNull()) {
      entry.payload_link = node["payload_link"].as<std::string>();
    }
    const YAML::Node max_j = node["max_joint_inertia_kg_m2"];
    if (!max_j || !max_j.IsSequence() || max_j.size() != 6) {
      throw std::runtime_error("Invalid max_joint_inertia_kg_m2 in effectors.yaml");
    }
    for (size_t i = 0; i < 6; ++i) {
      entry.max_joint_inertia_kg_m2[i] = max_j[i].as<double>();
    }
    const YAML::Node comps = node["components"];
    for (auto cit = comps.begin(); cit != comps.end(); ++cit) {
      const std::string cname = cit->first.as<std::string>();
      const YAML::Node part = cit->second;
      RigidBody rb;
      rb.mass_kg = part["mass_kg"].as<double>();
      for (size_t i = 0; i < 3; ++i) {
        rb.com_m[i] = part["com_m"][i].as<double>();
      }
      const YAML::Node ipk = part["inertia_per_kg"];
      for (size_t r = 0; r < 3; ++r) {
        for (size_t c = 0; c < 3; ++c) {
          rb.inertia[r][c] = ipk[r][c].as<double>() * rb.mass_kg;
        }
      }
      entry.components.emplace(cname, rb);
    }
    out.emplace(key, std::move(entry));
  }
  return out;
}

}  // namespace

const std::map<std::string, TerminalProfileEntry> & terminal_profiles(
  const std::string & yaml_path)
{
  if (yaml_path.empty()) {
    static const std::map<std::string, TerminalProfileEntry> kDefault =
      load_profiles_from_yaml(default_effectors_yaml_path());
    return kDefault;
  }
  static std::mutex mu;
  static std::map<std::string, std::map<std::string, TerminalProfileEntry>> cache;
  std::lock_guard<std::mutex> lock(mu);
  auto found = cache.find(yaml_path);
  if (found == cache.end()) {
    found = cache.emplace(yaml_path, load_profiles_from_yaml(yaml_path)).first;
  }
  return found->second;
}

std::optional<CompositeTerminal> terminal_for_model(const std::string & model) {
  const auto & profiles = terminal_profiles();
  const TerminalProfileEntry * matched = nullptr;
  auto direct = profiles.find(model);
  if (direct != profiles.end()) {
    matched = &direct->second;
  } else {
    for (const auto & kv : profiles) {
      if (kv.second.model == model) {
        matched = &kv.second;
        break;
      }
    }
  }
  if (matched == nullptr) {
    return std::nullopt;
  }
  return CompositeTerminal{
    matched->model, matched->payload_link, matched->components,
    matched->max_joint_inertia_kg_m2};
}

RigidBody terminal_body(
  const std::string & model,
  double added_mass_kg,
  std::optional<double> nominal_mass_kg,
  std::optional<Vec3> nominal_com_m)
{
  auto terminal = terminal_for_model(model);
  if (!terminal.has_value()) {
    throw std::invalid_argument("No composite terminal geometry for " + model);
  }
  const RigidBody nominal = terminal->body();
  const double mass = nominal_mass_kg.value_or(nominal.mass_kg);
  const Vec3 center = nominal_com_m.value_or(nominal.com_m);
  const double scale = (nominal.mass_kg != 0.0) ? (mass / nominal.mass_kg) : 0.0;
  Mat3 scaled_inertia{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      scaled_inertia[r][c] = nominal.inertia[r][c] * scale;
    }
  }
  std::vector<RigidBody> bodies;
  bodies.push_back(RigidBody{mass, center, scaled_inertia});
  if (added_mass_kg != 0.0 && terminal->payload_link.has_value()) {
    const auto & shape = terminal->components.at(*terminal->payload_link);
    const double pscale = added_mass_kg / shape.mass_kg;
    Mat3 pinertia{};
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        pinertia[r][c] = shape.inertia[r][c] * pscale;
      }
    }
    bodies.push_back(RigidBody{added_mass_kg, shape.com_m, pinertia});
  }
  return combine_bodies(bodies);
}

}  // namespace piper_x_driver
