#pragma once

#include <array>
#include <deque>
#include <functional>
#include <optional>
#include <string>

#include "piper_x_driver/can_protocol.hpp"
#include "piper_x_driver/configuration.hpp"
#include "piper_x_driver/friction.hpp"
#include "piper_x_driver/gravity.hpp"
#include "piper_x_driver/socketcan_arm.hpp"

namespace piper_x_driver {

inline constexpr Vec6 SLIDER_LOWER = {{
  -150.0 * 3.14159265358979323846 / 180.0,
  0.0,
  -170.0 * 3.14159265358979323846 / 180.0,
  -89.0 * 3.14159265358979323846 / 180.0,
  -89.0 * 3.14159265358979323846 / 180.0,
  -170.0 * 3.14159265358979323846 / 180.0,
}};
inline constexpr Vec6 SLIDER_UPPER = {{
  150.0 * 3.14159265358979323846 / 180.0,
  180.0 * 3.14159265358979323846 / 180.0,
  0.0,
  89.0 * 3.14159265358979323846 / 180.0,
  89.0 * 3.14159265358979323846 / 180.0,
  170.0 * 3.14159265358979323846 / 180.0,
}};

inline constexpr double TARGET_STALE_SECONDS = 0.05;

double compute_adaptive_stiffness_scale(
  double position_error,
  double scale,
  double b = 0.08,
  double power = 3.0);

class RateMeter {
public:
  explicit RateMeter(double window = 1.0) noexcept : window_(window) {}

  void mark(double now) noexcept {
    stamps_[(head_ + size_) & (kCapacity - 1)] = now;
    if (size_ < kCapacity) {
      ++size_;
    } else {
      head_ = (head_ + 1) & (kCapacity - 1);
    }
  }
  double hz(double now) noexcept;
  void clear() noexcept {
    head_ = 0;
    size_ = 0;
  }
  void pop_last() noexcept {
    if (size_ > 0) --size_;
  }

private:
  static constexpr size_t kCapacity = 1024;
  std::array<double, kCapacity> stamps_{};
  size_t head_{0};
  size_t size_{0};
  double window_{1.0};
};

struct ControllerReconfigureArgs {
  std::optional<Effector> effector;
  std::optional<double> gravity_scale;
  std::optional<ArmGravityModel> gravity_model;
  std::optional<double> friction_scale;
  std::optional<ArmFrictionModel> friction_model;
  std::optional<double> friction_error_epsilon;
  std::optional<double> friction_velocity_epsilon;
  std::optional<double> target_velocity_cutoff_hz;
  std::optional<double> s_a;
  std::optional<double> adaptive_stiffness_b;
  std::optional<double> adaptive_stiffness_power;
  std::optional<double> offset_ramp_s;
  bool update_models{false};
};

class Controller {
public:
  Controller(
    ArmBackend & arm,
    std::function<double()> clock_fn,
    std::optional<Effector> effector,
    double gravity_scale,
    std::optional<ArmGravityModel> gravity_model,
    double friction_scale,
    std::optional<ArmFrictionModel> friction_model,
    double friction_error_epsilon,
    double friction_velocity_epsilon,
    double target_velocity_cutoff_hz,
    double s_a,
    double adaptive_stiffness_b,
    double adaptive_stiffness_power,
    double offset_ramp_s,
    const Vec6 & kp,
    const Vec6 & kd);

  double clock() const { return clock_(); }

  Feedback feedback(std::optional<Feedback> sample = std::nullopt);
  Feedback healthy(bool require_enabled = true, std::optional<Feedback> sample = std::nullopt);
  void power(bool enabled);
  void clear_power_state();
  bool shutdown_to_drag();
  void start_mit();
  void set_mode(const std::string & next_mode, bool preload_mit = false);
  Feedback require_mode(const std::string & m1, const std::string & m2 = "");

  void joints(const Vec6 & position, uint8_t speed_percent);
  void cartesian(const Vec6 & pose, uint8_t speed_percent);
  void mit(const MitTarget & command);
  void release_mit();
  void gripper(double width, double effort);

  void reconfigure(const ControllerReconfigureArgs & args);
  void reset_wrench_estimation();
  Feedback watchdog(bool compute_wrench = true);

  Vec3 end_force() const noexcept {
    return Vec3{{end_wrench[0], end_wrench[1], end_wrench[2]}};
  }
  Vec3 end_torque() const noexcept {
    return Vec3{{end_wrench[3], end_wrench[4], end_wrench[5]}};
  }

  // Public observable state
  std::string mode{"standby"};
  std::optional<MitTarget> mit_target;
  std::optional<double> mit_received;
  std::optional<Effector> effector;
  std::optional<ArmGravityModel> gravity_model;
  std::optional<ArmFrictionModel> friction_model;
  double gravity_scale{1.0};
  double gravity_scale_applied{1.0};
  double friction_scale{1.0};
  double friction_scale_applied{1.0};
  double friction_error_epsilon{0.05};
  double friction_velocity_epsilon{0.02};
  double target_velocity_cutoff_hz{5.0};
  double adaptive_stiffness_scale_val{0.0};
  double adaptive_stiffness_scale_applied{0.0};
  double adaptive_stiffness_b{0.08};
  double adaptive_stiffness_power{3.0};
  double offset_ramp_s{2.0};
  double gravity_updated{0.0};
  Vec6 kp{{17.0, 16.0, 15.0, 4.0, 3.0, 3.0}};
  Vec6 kd{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double mit_started{0.0};
  bool mit_ready{false};
  Vec6 gravity{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 friction{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 output_torque{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 applied_kp{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 applied_kd{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 target_velocity{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 joint_acceleration{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 inertia_torque{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 external_torque{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 end_wrench{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double gripper_friction{0.0};
  std::string hold_reason;

  RateMeter comp_rate;
  RateMeter cmd_tx_rate;
  RateMeter mit_tx_rate;
  RateMeter grip_tx_rate;
  RateMeter ros_cmd_rate;

private:
  void reset_dynamics_state(bool clear_compensation);
  Vec6 resolve_kd(const Vec6 & kp_in, const Vec6 & kd_in) const;
  std::pair<Vec6, Vec6> resolve_command_gains(const MitTarget & cmd) const;
  std::pair<Vec6, Vec6> resolve_adaptive_gains(
    const Vec6 & base_kp, const Vec6 & base_kd, const Vec6 & position_errors) const;
  Vec6 update_acceleration(const Feedback & sample, const MitTarget & command, double now);
  Vec6 update_target_velocity(const Vec6 & target_position, double elapsed);
  MitTarget gravity_command(const Feedback & sample) const;
  void update_wrench_estimate(
    const MitTarget & command,
    const Feedback & sample,
    const Vec6 & resolved_kp,
    const Vec6 & resolved_kd,
    const Vec6 & velocity,
    const std::optional<Vec6> & raw_gravity,
    const std::optional<Vec6> & applied_gravity,
    const std::optional<Vec6> & raw_friction,
    const std::optional<Vec6> & applied_friction);
  void send_mit(const MitTarget & command, const Feedback & sample, bool compute_wrench = true);
  void apply_gripper(double width_cmd, double effort_cmd, const Feedback & sample, bool explicit_cmd);

  ArmBackend & arm_;
  std::function<double()> clock_;
  std::array<double, 6> target_hold_time_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::optional<Vec6> previous_target_;
  std::optional<MitTarget> last_mit_command_;
  std::optional<Vec6> prev_velocity_;
  std::optional<double> vel_updated_;
  bool gripper_assist_active_{false};
  std::optional<double> gripper_received_;
  mutable std::optional<Vec6> cached_auto_kd_kp_;
  mutable Vec6 cached_auto_kd_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
};

}  // namespace piper_x_driver
