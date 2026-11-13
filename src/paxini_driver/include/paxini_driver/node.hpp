#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "paxini_driver/protocol.hpp"
#include "paxini_driver/transport.hpp"

namespace paxini_driver {

// 固定容量环形缓冲滑动时间窗帧率统计器（1 kHz 热路径零堆内存分配）
class RateTracker {
public:
  explicit RateTracker(double window_sec = 1.0, size_t initial_cap = 2048)
  : window_sec_(window_sec), ring_(initial_cap, 0.0) {}

  void mark(double now) {
    prune(now);
    if (count_ == ring_.size()) {
      grow();
    }
    ring_[(head_ + count_) % ring_.size()] = now;
    ++count_;
  }

  double hz(double now) {
    prune(now);
    if (count_ < 2) {
      return 0.0;
    }
    const double first = ring_[head_];
    const double last = ring_[(head_ + count_ - 1) % ring_.size()];
    const double span = last - first;
    return (span > 1e-6) ? static_cast<double>(count_ - 1) / span : 0.0;
  }

private:
  void prune(double now) {
    const double cutoff = now - window_sec_;
    while (count_ > 0 && ring_[head_] < cutoff) {
      head_ = (head_ + 1) % ring_.size();
      --count_;
    }
  }

  void grow() {
    std::vector<double> next(ring_.size() * 2, 0.0);
    for (size_t i = 0; i < count_; ++i) {
      next[i] = ring_[(head_ + i) % ring_.size()];
    }
    ring_.swap(next);
    head_ = 0;
  }

  double window_sec_;
  std::vector<double> ring_;
  size_t head_{0};
  size_t count_{0};
};

// 帕西尼 PX6D 六维力传感器 ROS 2 常驻驱动节点：负责硬件收发、自动重连与零偏/工具重力实时补偿
class PaxiniDriverNode : public rclcpp::Node {
public:
  explicit PaxiniDriverNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::unique_ptr<SensorTransport> transport = nullptr,
    bool start_worker = true);

  ~PaxiniDriverNode() override;

  void shutdown_driver() noexcept;

  int step(double timeout_sec = 0.002);
  void process_event(const ProtocolEvent & ev, double now_sec);
  void check_stream_timeout(double now_sec);
  void handle_disconnect(const std::string & reason);
  void reset_reconnect_timer() noexcept;

  std::pair<bool, std::string> calibrate_hardware();
  std::pair<bool, std::string> reload_config();
  std::pair<bool, std::string> start_stream();
  std::pair<bool, std::string> stop_stream();
  std::pair<bool, std::string> request_single_frame();
  std::pair<bool, std::string> request_version();

  std::string status_json();
  void on_joints(const sensor_msgs::msg::JointState & msg);

  std::optional<std::array<double, 6>> latest_joint_positions() const;
  std::optional<WrenchSample> latest_raw() const;
  std::optional<WrenchSample> latest_wrench() const;
  std::array<double, 6> tare_offset() const;
  double tool_mass_kg() const;
  std::array<double, 3> tool_com_m() const;

  static double steady_now() noexcept;

private:
  bool try_connect_locked(double now_sec, bool initial = false, bool force = false);
  void send_stream_start_locked();
  void warn_communication_once(const std::string & message);
  void handle_disconnect_locked(const std::string & reason);
  int step_locked(double timeout_sec);
  std::pair<bool, std::string> exec_cmd(
    uint8_t cmd,
    std::optional<int> arg = std::nullopt,
    const std::string & desc = "");
  void publish_wrench(const WrenchSample & sample, bool raw);
  void publish_status();
  void on_command_msg(const std_msgs::msg::String & msg);
  void io_loop();

  std::string arm_id_;
  std::filesystem::path config_file_;
  std::string interface_;
  std::string expected_version_;
  int device_id_{DEFAULT_DEVICE_ID};
  std::string model_;
  std::string frame_id_;
  std::string acquisition_mode_;
  std::atomic<int> frequency_hz_{1000};
  double poll_rate_hz_{1000.0};
  bool autostart_{true};
  bool calibrate_on_start_{false};
  double reconnect_sec_{1.0};
  double stream_timeout_sec_{1.0};
  bool require_hardware_{false};

  std::unique_ptr<SensorTransport> transport_;
  // 锁顺序约定：如需同时持锁，必须先获取 io_mutex_ 再获取 state_mutex_，避免死锁
  mutable std::mutex io_mutex_;
  mutable std::mutex state_mutex_;

  WrenchFilter filter_;
  bool connected_{false};
  bool streaming_{false};
  std::string firmware_version_;
  std::string last_error_;
  std::optional<WrenchSample> latest_raw_;
  std::optional<WrenchSample> latest_wrench_;
  bool overload_warning_{false};
  std::array<double, 6> tare_offset_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double tool_mass_kg_{0.0};
  std::array<double, 3> tool_com_m_{0.0, 0.0, 0.0};
  std::string calibration_quality_json_{"{}"};
  std::optional<std::array<double, 6>> latest_joint_positions_;
  std::optional<std::array<double, 3>> latest_gravity_dir_;
  uint64_t sample_count_{0};
  RateTracker rate_tracker_;
  double last_connect_attempt_{0.0};
  double last_poll_sent_{0.0};
  uint32_t polls_without_rx_{0};
  double last_wrench_time_{0.0};
  double stream_started_time_{0.0};
  bool communication_warning_active_{false};
  bool stream_gap_reported_{false};
  std::vector<ProtocolEvent> events_buf_;

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr raw_wrench_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> shutdown_done_{false};
  std::mutex stop_mutex_;
  std::condition_variable stop_cv_;
  bool worker_enabled_{false};
  std::thread worker_thread_;
};

}  // namespace paxini_driver
