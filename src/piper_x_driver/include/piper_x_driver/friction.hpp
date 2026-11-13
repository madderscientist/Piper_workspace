#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "piper_x_driver/terminal.hpp"

namespace piper_x_driver {

inline constexpr double UNDAMPED_MAX_ZERO_SLOPE = 10.0;
inline constexpr double UNDAMPED_MIN_V0 = 0.03;

double interp_1d(
  double x,
  const std::vector<double> & xs,
  const std::vector<double> & ys);

struct SpatialHarmonic {
  double omega{1.0};
  double a{0.0};
  double b{0.0};
  double c{0.0};
  double d{0.0};

  void validate() const;
  double cogging(double theta) const noexcept;
  double modulation(double theta) const noexcept;
  double cogging_amplitude() const noexcept;
  double modulation_amplitude() const noexcept;
};

struct JointFrictionModel {
  double fc{0.0};
  double fs{0.0};
  double vs{0.1};
  double delta{2.0};
  double fv{0.0};
  double v0{0.01};
  double vr{0.5};
  double load_ratio{0.0};
  double error_epsilon{0.05};
  double velocity_epsilon{0.02};
  std::vector<SpatialHarmonic> harmonics;

  void validate() const;
  double cogging_torque(double theta) const noexcept;
  double modulation_torque(double theta) const noexcept;
  std::pair<double, double> mod_and_cogging(double theta) const noexcept;
  double envelope(double velocity) const noexcept;
  double stribeck_magnitude(double speed) const noexcept;
  double stribeck_torque(double velocity) const noexcept;
  double stiction_torque(
    double theta,
    double position_error = 0.0,
    double target_velocity = 0.0,
    double gravity_torque = 0.0,
    double static_friction = 0.0,
    std::optional<double> error_eps = std::nullopt,
    std::optional<double> velocity_eps = std::nullopt) const noexcept;
  double torque(
    double theta,
    double velocity,
    double position_error = 0.0,
    double target_velocity = 0.0,
    double gravity_torque = 0.0,
    double static_friction = 0.0,
    std::optional<double> error_eps = std::nullopt,
    std::optional<double> velocity_eps = std::nullopt,
    bool active_impedance = true,
    double cogging_scale = 1.0,
    double friction_scale = 1.0) const noexcept;
};

struct GripperFrictionModel {
  double fc{0.0};
  double fs{0.0};
  double vs{0.015};
  double delta{2.0};
  double fv{0.0};
  double v0{0.002};
  double vr{0.08};
  double kp_equiv{45.0};
  double kv_equiv{12.5};
  std::vector<double> w_grid;
  std::vector<double> bias_grid;
  std::vector<double> modulation_grid;

  void validate() const;
  double bias_force(double width) const noexcept;
  double modulation_force(double width) const noexcept;
  double local_fc(double width) const noexcept;
  double envelope(double velocity) const noexcept;
  double stribeck_magnitude(double speed) const noexcept;
  double force(
    double width,
    double velocity,
    double bias_scale = 1.0,
    double friction_scale = 1.0) const noexcept;
  std::tuple<double, double, double> compensate_command(
    double w_cmd,
    double effort_cmd,
    double w_meas,
    double velocity,
    double scale = 1.0) const noexcept;
};

struct ArmFrictionModel {
  std::string arm_id{"arm_1"};
  std::map<int, JointFrictionModel> joints;
  std::optional<GripperFrictionModel> gripper;

  JointFrictionModel joint(int index) const;
  ArmFrictionModel with_joint(int index, const JointFrictionModel & model) const;
  ArmFrictionModel with_gripper(std::optional<GripperFrictionModel> model) const;

  double gripper_force(
    double width,
    double velocity,
    double bias_scale = 1.0,
    double friction_scale = 1.0) const noexcept;

  Vec6 torques(
    const Vec6 & positions,
    const Vec6 & velocities,
    const Vec6 & position_errors = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
    const Vec6 & target_velocities = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
    const Vec6 & gravity_torques = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
    const Vec6 & static_frictions = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
    std::optional<double> error_epsilon = std::nullopt,
    std::optional<double> velocity_epsilon = std::nullopt,
    const std::array<bool, 6> & active_impedances = {{true, true, true, true, true, true}},
    double cogging_scale = 1.0,
    double friction_scale = 1.0) const;
};

ArmFrictionModel load_friction_model(const std::string & path);
void save_friction_model(const ArmFrictionModel & model, const std::string & path);

}  // namespace piper_x_driver
