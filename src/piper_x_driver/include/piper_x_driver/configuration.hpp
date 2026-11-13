#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "piper_x_driver/friction.hpp"
#include "piper_x_driver/gravity.hpp"
#include "piper_x_driver/terminal.hpp"

namespace piper_x_driver {

struct Effector {
  std::string label;
  double mass_kg{0.0};
  Vec3 com_m{{0.0, 0.0, 0.0}};
  std::string source;
  std::string model;

  bool operator==(const Effector & other) const noexcept;
};

struct ArmConfig {
  std::string id;
  std::string name;
  std::string namespace_str;
  std::string can_port;
  std::string end_effector;
  double gravity_scale{1.0};
  std::string mounting{"upright"};
  Vec6 kp{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 kd{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double s_a{2.0};
  double friction_scale{1.0};
  std::string friction_config;
  std::string gravity_config;
  double friction_error_epsilon{0.05};
  double friction_velocity_epsilon{0.02};
  double target_velocity_cutoff_hz{5.0};
  double adaptive_stiffness_b{0.08};
  double adaptive_stiffness_power{3.0};
  double offset_ramp_s{2.0};

  bool operator==(const ArmConfig & other) const noexcept;
};

double bounded(double value, const std::string & name, double lower, double upper);
std::string default_config_path();
std::string json_escape(const std::string & s);
std::string json_number(double v);

class FleetConfig {
public:
  explicit FleetConfig(const std::string & path);

  std::string path;
  bool has_inline_effectors{false};
  std::map<std::string, Effector> effectors;
  std::vector<ArmConfig> arms;
  std::map<std::string, std::optional<ArmFrictionModel>> friction_models;
  std::map<std::string, std::optional<ArmGravityModel>> gravity_models;

  const ArmConfig & arm(const std::string & identifier) const;
  const ArmFrictionModel * friction_model(const std::string & arm_id) const;
  const ArmGravityModel * gravity_model(const std::string & arm_id) const;
  std::string public_json(const ArmConfig & arm_cfg) const;

  FleetConfig save_arm(
    const ArmConfig & current,
    const std::string & end_effector,
    double gravity_scale,
    std::optional<double> friction_scale = std::nullopt) const;
};

}  // namespace piper_x_driver
