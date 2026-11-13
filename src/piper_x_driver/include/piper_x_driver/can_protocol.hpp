#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "piper_x_driver/terminal.hpp"

namespace piper_x_driver {

inline constexpr std::array<double, 6> MIT_GAIN_SCALE = {{
  24.75, 24.75, 28.41, 2.10, 1.36, 1.0
}};
inline constexpr double MIT_KP_MAX = 500.0;
inline constexpr double MIT_KD_SPAN = 10.0;
inline constexpr int MIT_KP_STEPS = 4095;
inline constexpr double MAX_GRAVITY_OFFSET = 1.0;
inline constexpr double MIT_POSITION_LIMIT = 12.5;
inline constexpr int SOCKETCAN_RCVBUF_REQUEST = 65536;

inline constexpr std::array<double, 6> MIT_KP_LSB = {{
  24.75 * 500.0 / 4095.0,
  24.75 * 500.0 / 4095.0,
  28.41 * 500.0 / 4095.0,
  2.10 * 500.0 / 4095.0,
  1.36 * 500.0 / 4095.0,
  1.0 * 500.0 / 4095.0,
}};

inline constexpr std::array<double, 6> MIT_KD_LSB = {{
  24.75 * 10.0 / 4095.0,
  24.75 * 10.0 / 4095.0,
  28.41 * 10.0 / 4095.0,
  2.10 * 10.0 / 4095.0,
  1.36 * 10.0 / 4095.0,
  1.0 * 10.0 / 4095.0,
}};

inline constexpr std::array<double, 6> JOINT_TORQUE_K = {{
  1.18125, 1.65375, 1.18125, 0.9576, 0.9576, 0.9576
}};
inline constexpr std::array<double, 6> JOINT_TORQUE_B = {{
  4.0, 2.5, 4.0, 1.2, 0.8, 1.0
}};
inline constexpr std::array<double, 6> JOINT_TORQUE_C = {{
  1.0, 1.0, 1.0, 1.0723, 1.016565, 0.813252
}};
inline constexpr std::array<double, 6> HIGH_SPD_TORQUE_SCALE = {{
  1.18125 * 4.0 * 1.0,
  1.65375 * 2.5 * 1.0,
  1.18125 * 4.0 * 1.0,
  0.9576 * 1.2 * 1.0723,
  0.9576 * 0.8 * 1.016565,
  0.9576 * 1.0 * 0.813252,
}};

inline constexpr std::array<const char *, 11> FEEDBACK_FIELDS = {{
  "joint12(0x2A5)", "joint34(0x2A6)", "joint56(0x2A7)",
  "gripper(0x2A8)", "arm_status(0x2A1)",
  "driver1(0x261)", "driver2(0x262)", "driver3(0x263)",
  "driver4(0x264)", "driver5(0x265)", "driver6(0x266)",
}};

struct RawCanFrame {
  uint32_t can_id{0};
  uint8_t dlc{8};
  std::array<uint8_t, 8> data{};
};

struct ArmStatusFields {
  int ctrl_mode{0};
  int arm_status{0};
  int mode_feedback{0};
  int teach_status{0};
  int motion_status{0};
  int trajectory_num{0};
  int err_code{0};
};

struct MitTarget {
  Vec6 position{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 velocity{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 kp{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 kd{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 torque{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 gravity_offset{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

  bool operator==(const MitTarget & other) const noexcept {
    return position == other.position && velocity == other.velocity &&
           kp == other.kp && kd == other.kd && torque == other.torque &&
           gravity_offset == other.gravity_offset;
  }
};

struct Feedback {
  std::optional<std::array<double, 7>> position;
  std::optional<ArmStatusFields> status;
  std::array<bool, 7> enabled{{false, false, false, false, false, false, false}};
  std::array<double, 11> age{{-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0}};
  std::optional<Vec6> pose;
  bool has_velocity_torque{false};
  Vec6 velocity{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 torque{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double gripper_velocity{0.0};
  double gripper_force{0.0};
};

inline constexpr double normalize_feedback_age(double now, double stamp) noexcept {
  if (stamp < 0.0) {
    return -1.0;
  }
  const double raw = now - stamp;
  return (raw < 0.0 && raw >= -0.050) ? 0.0 : raw;
}

bool fresh_ages(const double * ages, size_t count, double timeout = 1.0) noexcept;
template<size_t N>
bool fresh(const std::array<double, N> & ages, double timeout = 1.0) noexcept {
  return fresh_ages(ages.data(), N, timeout);
}
std::string feedback_problem(const std::array<double, 11> & ages, double timeout = 1.0);

// Protocol frame encoders (exact byte-for-byte match with pyAgxArm V189)
uint32_t float_to_uint(double x, double x_min, double x_max, int bits) noexcept;
RawCanFrame encode_mit_joint_wire(
  int joint_index,
  double p_des,
  double v_des,
  double kp_wire,
  double kd_wire,
  double t_ff_joint) noexcept;
std::array<RawCanFrame, 6> encode_mit_command(const MitTarget & command) noexcept;
RawCanFrame encode_mode_ctrl(
  uint8_t ctrl_mode,
  uint8_t move_mode,
  uint8_t speed_percent,
  uint8_t mit_mode,
  uint8_t residence_time = 0,
  uint8_t installation_pos = 0) noexcept;
RawCanFrame encode_motion_ctrl(
  uint8_t emergency_stop = 0,
  uint8_t track_ctrl = 0,
  uint8_t grag_teach_ctrl = 0) noexcept;
RawCanFrame encode_leader_follower_config(
  uint8_t linkage_config,
  uint8_t feedback_offset = 0,
  uint8_t ctrl_offset = 0,
  uint8_t linkage_offset = 0) noexcept;
RawCanFrame encode_leader_move_to_home(uint8_t mode) noexcept;
RawCanFrame encode_motor_enable(uint8_t joint_index, bool enable) noexcept;
RawCanFrame encode_param_enquiry_and_config(
  uint8_t param_enquiry = 0,
  uint8_t param_setting = 0,
  uint8_t data_feedback_0x48x = 0,
  uint8_t end_load_effective = 0,
  uint8_t set_end_load = 0) noexcept;
RawCanFrame encode_req_firmware() noexcept;
RawCanFrame encode_gripper_ctrl(
  int32_t value_um_or_mdeg,
  uint16_t force_mn,
  uint8_t status_code,
  uint8_t set_zero = 0) noexcept;
std::array<RawCanFrame, 3> encode_joint_ctrl(const Vec6 & joints_rad) noexcept;
std::array<RawCanFrame, 3> encode_end_pose_ctrl(const Vec6 & pose_xyz_rpy) noexcept;
std::array<RawCanFrame, 5> encode_joint_move_batch(
  const Vec6 & joints_rad, uint8_t speed_percent, uint8_t cached_mit_mode = 0) noexcept;
std::array<RawCanFrame, 5> encode_cartesian_move_batch(
  const Vec6 & pose_xyz_rpy, uint8_t speed_percent, bool linear, uint8_t cached_mit_mode = 0) noexcept;

}  // namespace piper_x_driver
