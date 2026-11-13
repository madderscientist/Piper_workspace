#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace piper_x_driver {

using Vec3 = std::array<double, 3>;
using Vec6 = std::array<double, 6>;
using Mat3 = std::array<std::array<double, 3>, 3>;
using Mat6 = std::array<std::array<double, 6>, 6>;

struct RigidBody {
  double mass_kg{0.0};
  Vec3 com_m{{0.0, 0.0, 0.0}};
  Mat3 inertia{{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};
};

struct TerminalProfileEntry {
  std::string key;
  std::string label;
  std::string model;
  std::string source;
  std::optional<std::string> payload_link;
  Vec6 max_joint_inertia_kg_m2{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::map<std::string, RigidBody> components;
};

RigidBody combine_bodies(const std::vector<RigidBody> & bodies);

struct CompositeTerminal {
  std::string model;
  std::optional<std::string> payload_link;
  std::map<std::string, RigidBody> components;
  Vec6 max_joint_inertia_kg_m2{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

  RigidBody body() const;
};

std::string default_effectors_yaml_path();
const std::map<std::string, TerminalProfileEntry> & terminal_profiles(
  const std::string & yaml_path = "");
std::optional<CompositeTerminal> terminal_for_model(const std::string & model);
RigidBody terminal_body(
  const std::string & model,
  double added_mass_kg = 0.0,
  std::optional<double> nominal_mass_kg = std::nullopt,
  std::optional<Vec3> nominal_com_m = std::nullopt);

}  // namespace piper_x_driver
