#include "piper_x_driver/can_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace piper_x_driver {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRad2Deg = 180.0 / kPi;

void put_be16(uint8_t * dst, uint16_t v) noexcept {
  dst[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
  dst[1] = static_cast<uint8_t>(v & 0xFF);
}

void put_be32(uint8_t * dst, int32_t v) noexcept {
  const uint32_t u = static_cast<uint32_t>(v);
  dst[0] = static_cast<uint8_t>((u >> 24) & 0xFF);
  dst[1] = static_cast<uint8_t>((u >> 16) & 0xFF);
  dst[2] = static_cast<uint8_t>((u >> 8) & 0xFF);
  dst[3] = static_cast<uint8_t>(u & 0xFF);
}

long round_ties_to_even(double x) noexcept {
  return std::lrint(x);
}

}  // namespace

bool fresh_ages(const double * ages, size_t count, double timeout) noexcept {
  for (size_t i = 0; i < count; ++i) {
    if (!std::isfinite(ages[i]) || ages[i] < 0.0 || ages[i] > timeout) {
      return false;
    }
  }
  return true;
}

std::string feedback_problem(const std::array<double, 11> & ages, double timeout) {
  std::ostringstream oss;
  bool first = true;
  for (size_t i = 0; i < 11; ++i) {
    const double age = ages[i];
    if (age == -1.0) {
      if (!first) oss << "; ";
      first = false;
      oss << FEEDBACK_FIELDS[i] << "=missing";
    } else if (!std::isfinite(age) || age < 0.0 || age > timeout) {
      if (!first) oss << "; ";
      first = false;
      char buf[128];
      std::snprintf(
        buf, sizeof(buf), "%s age=%.3f ms (required 0..%g ms)",
        FEEDBACK_FIELDS[i], age * 1000.0, timeout * 1000.0);
      oss << buf;
    }
  }
  return oss.str();
}

uint32_t float_to_uint(double x, double x_min, double x_max, int bits) noexcept {
  const double clamped = std::clamp(x, x_min, x_max);
  const double span = x_max - x_min;
  const double scaled = (clamped - x_min) * static_cast<double>((1u << bits) - 1u) / span;
  const int64_t val = static_cast<int64_t>(scaled);
  const int64_t hi = static_cast<int64_t>((1u << bits) - 1u);
  return static_cast<uint32_t>(std::clamp<int64_t>(val, 0, hi));
}

RawCanFrame encode_mit_joint_wire(
  int joint_index,
  double p_des,
  double v_des,
  double kp_wire,
  double kd_wire,
  double t_ff_joint) noexcept
{
  const int idx = std::clamp(joint_index - 1, 0, 5);
  const double c = JOINT_TORQUE_C[idx];
  const double t_limit = 16.0 * c;
  const double p_clamped = std::clamp(p_des, -12.5, 12.5);
  const double v_clamped = std::clamp(v_des, -45.0, 45.0);
  const double kp_clamped = std::clamp(kp_wire, 0.0, 500.0);
  const double kd_clamped = std::clamp(kd_wire, -5.0, 5.0);
  const double t_scaled = std::clamp(t_ff_joint, -t_limit, t_limit) / c;

  const uint32_t pu = float_to_uint(p_clamped, -12.5, 12.5, 16);
  const uint32_t vu = float_to_uint(v_clamped, -45.0, 45.0, 12);
  const uint32_t kpu = float_to_uint(kp_clamped, 0.0, 500.0, 12);
  const uint32_t kdu = float_to_uint(kd_clamped, -5.0, 5.0, 12);
  const uint32_t tu = float_to_uint(t_scaled, -16.0, 16.0, 12);

  RawCanFrame f;
  f.can_id = static_cast<uint32_t>(0x15A + idx);
  f.dlc = 8;
  f.data[0] = static_cast<uint8_t>((pu >> 8) & 0xFF);
  f.data[1] = static_cast<uint8_t>(pu & 0xFF);
  f.data[2] = static_cast<uint8_t>((vu >> 4) & 0xFF);
  f.data[3] = static_cast<uint8_t>(((vu & 0xF) << 4) | ((kpu >> 8) & 0xF));
  f.data[4] = static_cast<uint8_t>(kpu & 0xFF);
  f.data[5] = static_cast<uint8_t>((kdu >> 4) & 0xFF);
  f.data[6] = static_cast<uint8_t>(((kdu & 0xF) << 4) | ((tu >> 8) & 0xF));
  f.data[7] = static_cast<uint8_t>(tu & 0xFF);
  return f;
}

std::array<RawCanFrame, 6> encode_mit_command(const MitTarget & command) noexcept {
  std::array<RawCanFrame, 6> frames{};
  for (int i = 0; i < 6; ++i) {
    const double scale = MIT_GAIN_SCALE[i];
    const double kp = command.kp[i];
    const double kd = command.kd[i];
    double position = command.position[i];
    const double offset = command.gravity_offset[i];
    const double base_pos = position - offset;
    double torque = command.torque[i];
    const long raw_uint = (kp > 0.0) ? round_ties_to_even(kp / MIT_KP_LSB[i]) : 0;
    double kp_wire = 0.0;
    if (raw_uint <= 0 || kd < MIT_KD_LSB[i]) {
      kp_wire = (raw_uint <= 0)
        ? 0.0
        : ((std::min<long>(MIT_KP_STEPS, raw_uint) + 0.25) * MIT_KP_MAX / MIT_KP_STEPS);
      position = std::clamp(base_pos, -12.5, 12.5);
      torque += offset * kp;
    } else {
      const long kp_uint = std::min<long>(MIT_KP_STEPS, raw_uint);
      kp_wire = (kp_uint + 0.25) * MIT_KP_MAX / MIT_KP_STEPS;
      const double kp_realized = scale * (kp_uint * MIT_KP_MAX / MIT_KP_STEPS);
      if (offset != 0.0) {
        const double lo = std::max(-MAX_GRAVITY_OFFSET, -12.5 - base_pos);
        const double hi = std::min(MAX_GRAVITY_OFFSET, -base_pos + 12.5);
        const double wire_offset = std::clamp(offset * (kp / kp_realized), lo, hi);
        position = base_pos + wire_offset;
        torque += offset * kp - wire_offset * kp_realized;
      } else {
        position = std::clamp(position, -12.5, 12.5);
      }
    }
    const double kd_wire = std::clamp(kd / scale, 0.0, 5.0);
    frames[i] = encode_mit_joint_wire(
      i + 1, position, command.velocity[i], kp_wire, kd_wire, torque);
  }
  return frames;
}

RawCanFrame encode_mode_ctrl(
  uint8_t ctrl_mode,
  uint8_t move_mode,
  uint8_t speed_percent,
  uint8_t mit_mode,
  uint8_t residence_time,
  uint8_t installation_pos) noexcept
{
  RawCanFrame f;
  f.can_id = 0x151;
  f.dlc = 8;
  f.data = {{ctrl_mode, move_mode, speed_percent, mit_mode, residence_time, installation_pos, 0, 0}};
  return f;
}

RawCanFrame encode_motion_ctrl(
  uint8_t emergency_stop,
  uint8_t track_ctrl,
  uint8_t grag_teach_ctrl) noexcept
{
  RawCanFrame f;
  f.can_id = 0x150;
  f.dlc = 8;
  f.data = {{emergency_stop, track_ctrl, grag_teach_ctrl, 0, 0, 0, 0, 0}};
  return f;
}

RawCanFrame encode_leader_follower_config(
  uint8_t linkage_config,
  uint8_t feedback_offset,
  uint8_t ctrl_offset,
  uint8_t linkage_offset) noexcept
{
  RawCanFrame f;
  f.can_id = 0x470;
  f.dlc = 8;
  f.data = {{linkage_config, feedback_offset, ctrl_offset, linkage_offset, 0, 0, 0, 0}};
  return f;
}

RawCanFrame encode_leader_move_to_home(uint8_t mode) noexcept {
  RawCanFrame f;
  f.can_id = 0x191;
  f.dlc = 8;
  if (mode == 1) {
    f.data = {{0x01, 0x01, 0x01, 0, 0, 0, 0, 0}};
  } else if (mode == 2) {
    f.data = {{0x01, 0x00, 0x01, 0, 0, 0, 0, 0}};
  } else {
    f.data = {{0x01, 0x00, 0x00, 0, 0, 0, 0, 0}};
  }
  return f;
}

RawCanFrame encode_motor_enable(uint8_t joint_index, bool enable) noexcept {
  RawCanFrame f;
  f.can_id = 0x471;
  f.dlc = 8;
  f.data = {{joint_index, static_cast<uint8_t>(enable ? 0x02 : 0x01), 0, 0, 0, 0, 0, 0}};
  return f;
}

RawCanFrame encode_param_enquiry_and_config(
  uint8_t param_enquiry,
  uint8_t param_setting,
  uint8_t data_feedback_0x48x,
  uint8_t end_load_effective,
  uint8_t set_end_load) noexcept
{
  RawCanFrame f;
  f.can_id = 0x477;
  f.dlc = 8;
  f.data = {{
    param_enquiry, param_setting, data_feedback_0x48x,
    end_load_effective, set_end_load, 0, 0, 0
  }};
  return f;
}

RawCanFrame encode_req_firmware() noexcept {
  RawCanFrame f;
  f.can_id = 0x4AF;
  f.dlc = 1;
  f.data = {{0x01, 0, 0, 0, 0, 0, 0, 0}};
  return f;
}

RawCanFrame encode_gripper_ctrl(
  int32_t value_um_or_mdeg,
  uint16_t force_mn,
  uint8_t status_code,
  uint8_t set_zero) noexcept
{
  RawCanFrame f;
  f.can_id = 0x159;
  f.dlc = 8;
  put_be32(f.data.data(), value_um_or_mdeg);
  put_be16(f.data.data() + 4, force_mn);
  f.data[6] = status_code;
  f.data[7] = set_zero;
  return f;
}

std::array<RawCanFrame, 3> encode_joint_ctrl(const Vec6 & joints_rad) noexcept {
  constexpr double kMax = 2.0 * kPi;
  int32_t mdeg[6];
  for (int i = 0; i < 6; ++i) {
    const double clamped = std::clamp(joints_rad[i], -kMax, kMax);
    mdeg[i] = static_cast<int32_t>(round_ties_to_even(clamped * kRad2Deg * 1e3));
  }
  std::array<RawCanFrame, 3> out{};
  for (int p = 0; p < 3; ++p) {
    out[p].can_id = static_cast<uint32_t>(0x155 + p);
    out[p].dlc = 8;
    put_be32(out[p].data.data(), mdeg[2 * p]);
    put_be32(out[p].data.data() + 4, mdeg[2 * p + 1]);
  }
  return out;
}

std::array<RawCanFrame, 3> encode_end_pose_ctrl(const Vec6 & pose_xyz_rpy) noexcept {
  const int32_t x = static_cast<int32_t>(round_ties_to_even(pose_xyz_rpy[0] * 1e6));
  const int32_t y = static_cast<int32_t>(round_ties_to_even(pose_xyz_rpy[1] * 1e6));
  const int32_t z = static_cast<int32_t>(round_ties_to_even(pose_xyz_rpy[2] * 1e6));
  const double r = std::clamp(pose_xyz_rpy[3], -kPi, kPi);
  const double p = std::clamp(pose_xyz_rpy[4], -0.5 * kPi, 0.5 * kPi);
  const double yw = std::clamp(pose_xyz_rpy[5], -kPi, kPi);
  const int32_t roll = static_cast<int32_t>(round_ties_to_even(r * kRad2Deg * 1e3));
  const int32_t pitch = static_cast<int32_t>(round_ties_to_even(p * kRad2Deg * 1e3));
  const int32_t yaw = static_cast<int32_t>(round_ties_to_even(yw * kRad2Deg * 1e3));

  std::array<RawCanFrame, 3> out{};
  out[0].can_id = 0x152;
  out[0].dlc = 8;
  put_be32(out[0].data.data(), x);
  put_be32(out[0].data.data() + 4, y);
  out[1].can_id = 0x153;
  out[1].dlc = 8;
  put_be32(out[1].data.data(), z);
  put_be32(out[1].data.data() + 4, roll);
  out[2].can_id = 0x154;
  out[2].dlc = 8;
  put_be32(out[2].data.data(), pitch);
  put_be32(out[2].data.data() + 4, yaw);
  return out;
}

std::array<RawCanFrame, 5> encode_joint_move_batch(
  const Vec6 & joints_rad, uint8_t speed_percent, uint8_t cached_mit_mode) noexcept
{
  std::array<RawCanFrame, 5> batch{};
  batch[0] = encode_mode_ctrl(1, 255, speed_percent, cached_mit_mode);
  batch[1] = encode_mode_ctrl(1, 1, speed_percent, 0x00);
  const auto target = encode_joint_ctrl(joints_rad);
  batch[2] = target[0];
  batch[3] = target[1];
  batch[4] = target[2];
  return batch;
}

std::array<RawCanFrame, 5> encode_cartesian_move_batch(
  const Vec6 & pose_xyz_rpy, uint8_t speed_percent, bool linear, uint8_t cached_mit_mode) noexcept
{
  std::array<RawCanFrame, 5> batch{};
  batch[0] = encode_mode_ctrl(1, 255, speed_percent, cached_mit_mode);
  batch[1] = encode_mode_ctrl(1, linear ? 2 : 0, speed_percent, 0x00);
  const auto target = encode_end_pose_ctrl(pose_xyz_rpy);
  batch[2] = target[0];
  batch[3] = target[1];
  batch[4] = target[2];
  return batch;
}


}  // namespace piper_x_driver
