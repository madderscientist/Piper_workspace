#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "piper_x_driver/can_protocol.hpp"

namespace piper_x_driver {

class ArmBackend {
public:
  virtual ~ArmBackend() = default;

  virtual std::string channel() const = 0;
  virtual bool is_real_hardware() const { return true; }
  virtual bool is_powered() const = 0;
  virtual uint64_t can_rx_updates() const = 0;

  virtual Feedback snapshot() = 0;
  virtual void calibrate_effector() = 0;
  virtual void power(bool enabled, double width = 0.0) = 0;
  virtual void mode(const std::string & name) = 0;
  virtual void leave_drag() = 0;
  virtual void enter_drag() = 0;
  virtual void joints(const Vec6 & position, uint8_t speed_percent) = 0;
  virtual void cartesian(const Vec6 & pose, uint8_t speed_percent, const std::string & move_mode) = 0;
  virtual void mit(const MitTarget & command) = 0;
  virtual void retain_mit() = 0;
  virtual void grip(double width, double effort, bool interpolate = true) = 0;
  virtual void close() = 0;
};

class SocketCanArm final : public ArmBackend {
public:
  explicit SocketCanArm(const std::string & channel);
  ~SocketCanArm() override;

  SocketCanArm(const SocketCanArm &) = delete;
  SocketCanArm & operator=(const SocketCanArm &) = delete;

  std::string channel() const override { return channel_; }
  bool is_real_hardware() const override { return true; }
  bool is_powered() const override { return powered_.load(std::memory_order_relaxed); }
  uint64_t can_rx_updates() const override {
    return can_rx_updates_.load(std::memory_order_relaxed);
  }

  Feedback snapshot() override;
  void calibrate_effector() override;
  void power(bool enabled, double width = 0.0) override;
  void mode(const std::string & name) override;
  void leave_drag() override;
  void enter_drag() override;
  void joints(const Vec6 & position, uint8_t speed_percent) override;
  void cartesian(const Vec6 & pose, uint8_t speed_percent, const std::string & move_mode) override;
  void mit(const MitTarget & command) override;
  void retain_mit() override;
  void grip(double width, double effort, bool interpolate = true) override;
  void close() override;

private:
  void open_and_configure_socket();
  void rx_loop();
  void grip_interp_loop();
  void parse_frame(uint32_t can_id, uint8_t dlc, const uint8_t * data, double rx_stamp);
  void send_frame(const RawCanFrame & frame);
  void send_frames_batch(const RawCanFrame * frames, size_t count);
  void write_power_once(bool enabled);
  void disable_gripper();
  void set_motion_mode_code(const std::string & motion_mode);
  void write_mode_impl(const std::string & name);
  void disconnect_internal() noexcept;

  std::string channel_;
  int sock_{-1};
  std::atomic<bool> stop_rx_{false};
  std::thread rx_thread_;

  mutable std::mutex rx_mu_;
  std::condition_variable rx_cv_;
  uint64_t rx_frame_counter_{0};

  // Cached parsed feedback segments and kernel SO_TIMESTAMPNS wall-clock timestamps
  bool has_joint_pair_[3]{false, false, false};
  double joint_pair_stamp_[3]{-1.0, -1.0, -1.0};
  Vec6 joint_pos_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

  bool has_leader_pair_[3]{false, false, false};
  double leader_pair_stamp_[3]{-1.0, -1.0, -1.0};
  Vec6 leader_pos_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

  bool has_grip_{false};
  double grip_stamp_{-1.0};
  bool grip_angle_mode_{false};
  double grip_value_{0.0};
  double grip_force_{0.0};
  bool grip_enabled_{false};

  bool has_status_{false};
  double status_stamp_{-1.0};
  ArmStatusFields status_{};

  bool has_driver_[6]{false, false, false, false, false, false};
  double driver_stamp_[6]{-1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
  bool driver_enabled_[6]{false, false, false, false, false, false};

  bool has_motor_[6]{false, false, false, false, false, false};
  double motor_stamp_[6]{-1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
  Vec6 motor_vel_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 motor_torque_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

  bool has_pose_part_[3]{false, false, false};
  double pose_stamp_[3]{-1.0, -1.0, -1.0};
  Vec6 pose_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

  // Low-frequency response caches (0x4AF firmware, 0x476 set_instruction ACK)
  std::vector<uint8_t> fw_buf_;
  bool has_resp_476_{false};
  uint8_t resp_476_index_{0};
  uint8_t resp_476_zero_ok_{0};

  // Gripper velocity sliding window & state tracked in snapshot()
  std::deque<std::pair<double, double>> grip_history_;
  double grip_vel_{0.0};
  double last_joint_rx_stamp_{-1.0};
  std::atomic<uint64_t> can_rx_updates_{0};

  // Mode / MIT / TX state
  std::mutex tx_mu_;
  std::mutex mode_tx_mu_;
  uint8_t cached_move_mode_{0};
  uint8_t cached_speed_percent_{50};
  uint8_t cached_mit_mode_{0};
  RawCanFrame last_0x151_{};
  double last_0x151_tx_mono_{-1.0};
  std::atomic<bool> in_drag_mode_{false};
  std::string current_mode_{"standby"};
  std::atomic<bool> powered_{false};
  std::optional<std::array<RawCanFrame, 6>> mit_output_;

  // Gripper 200Hz command interpolation worker
  std::mutex grip_mu_;
  std::condition_variable grip_cv_;
  bool grip_stop_{false};
  bool grip_interp_active_{false};
  std::thread grip_thread_;
  std::optional<double> grip_cmd_pos_;
  double grip_start_pos_{0.0};
  double grip_target_pos_{0.0};
  double grip_target_effort_{0.0};
  double grip_interp_t0_{0.0};
  double grip_last_tx_time_{0.0};
  double grip_last_cmd_time_{0.0};
  bool closed_{false};
};

}  // namespace piper_x_driver
