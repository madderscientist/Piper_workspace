#pragma once

#include <array>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "piper_x_interfaces/msg/arm_status.hpp"
#include "piper_x_interfaces/msg/cartesian_command.hpp"
#include "piper_x_interfaces/msg/gripper_command.hpp"
#include "piper_x_interfaces/msg/joint_command.hpp"
#include "piper_x_interfaces/msg/mit_command.hpp"
#include "piper_x_interfaces/srv/calibrate_effector.hpp"
#include "piper_x_interfaces/srv/configure_arm.hpp"
#include "piper_x_interfaces/srv/enable.hpp"
#include "piper_x_interfaces/srv/set_mode.hpp"

#include "piper_x_driver/configuration.hpp"
#include "piper_x_driver/controller.hpp"
#include "piper_x_driver/socketcan_arm.hpp"

namespace piper_x_driver {

class DriverNode final : public rclcpp::Node {
public:
  explicit DriverNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::unique_ptr<ArmBackend> custom_arm = nullptr,
    bool create_default_timer = true);
  ~DriverNode() override;

  static double steady_now() noexcept;

  void fast_control_step();
  void publish_step();
  void shutdown_driver() noexcept;
  void trigger_teaching_retry();

  double rate_hz() const noexcept { return rate_hz_; }
  double telemetry_hz() const noexcept { return telemetry_hz_; }
  bool is_first_fleet_arm() const noexcept {
    return !fleet_->arms.empty() && arm_config_.id == fleet_->arms.front().id;
  }
  bool has_pending_async() const noexcept {
    return power_request_.has_value() || calibration_active_;
  }
  const ArmConfig & arm_config() const noexcept { return arm_config_; }
  Controller & controller() noexcept { return *control_; }
  ArmBackend & arm() noexcept { return *arm_; }

private:
  using EnableSrv = piper_x_interfaces::srv::Enable;
  using SetModeSrv = piper_x_interfaces::srv::SetMode;
  using ConfigureArmSrv = piper_x_interfaces::srv::ConfigureArm;
  using CalibrateEffectorSrv = piper_x_interfaces::srv::CalibrateEffector;

  bool wrench_estimation_enabled() const;
  bool wrench_estimate_available() const;
  bool sync_wrench_estimation();
  rcl_interfaces::msg::SetParametersResult on_parameters_set(
    const std::vector<rclcpp::Parameter> & parameters);

  void set_error(const std::string & message);
  static std::string error_key(const std::string & message);
  void record_can_rx(const Feedback * sample, double now);
  void publish_control_state(const Feedback * sample = nullptr);

  Feedback calibration_feedback(std::optional<Feedback> sample = std::nullopt);
  std::optional<std::string> calibration_block_reason(
    std::optional<Feedback> sample = std::nullopt);

  void on_teaching(const std_msgs::msg::Bool::SharedPtr msg);
  void on_joints_cmd(const piper_x_interfaces::msg::JointCommand::SharedPtr msg);
  void on_cartesian_cmd(const piper_x_interfaces::msg::CartesianCommand::SharedPtr msg);
  void on_mit_cmd(const piper_x_interfaces::msg::MitCommand::SharedPtr msg);
  void on_gripper_cmd(const piper_x_interfaces::msg::GripperCommand::SharedPtr msg);

  void handle_enable(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<EnableSrv::Request> request);
  void check_power();

  void handle_calibrate_effector(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<CalibrateEffectorSrv::Request> request);
  void check_calibration();

  void handle_set_mode(
    const std::shared_ptr<SetModeSrv::Request> request,
    std::shared_ptr<SetModeSrv::Response> response);
  void handle_configure(
    const std::shared_ptr<ConfigureArmSrv::Request> request,
    std::shared_ptr<ConfigureArmSrv::Response> response);
  void handle_set_wrench_estimation(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  std::unique_ptr<FleetConfig> fleet_;
  ArmConfig arm_config_;
  std::string public_config_prefix_;
  double rate_hz_{200.0};
  double telemetry_hz_{200.0};

  std::unique_ptr<ArmBackend> arm_;
  std::unique_ptr<Controller> control_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  std::string last_error_;
  bool teaching_active_{false};
  double teaching_received_{0.0};

  struct PendingPower {
    std::shared_ptr<rmw_request_id_t> header;
    bool enabled{false};
    double started{0.0};
    uint64_t serial{0};
    rclcpp::TimerBase::SharedPtr timer;
  };
  std::optional<bool> power_request_;
  uint64_t power_serial_{0};
  std::optional<PendingPower> pending_power_;

  struct PendingCalibration {
    std::shared_ptr<rmw_request_id_t> header;
    uint64_t serial{0};
    std::shared_future<void> worker_future;
    std::optional<double> confirm_after;
    rclcpp::TimerBase::SharedPtr timer;
  };
  bool calibration_active_{false};
  std::optional<PendingCalibration> pending_calibration_;
  std::thread calibration_thread_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Publisher<piper_x_interfaces::msg::ArmStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_pub_;

  rclcpp::Service<EnableSrv>::SharedPtr enable_srv_;
  rclcpp::Service<CalibrateEffectorSrv>::SharedPtr calibrate_srv_;
  rclcpp::Service<SetModeSrv>::SharedPtr mode_srv_;
  rclcpp::Service<ConfigureArmSrv>::SharedPtr configure_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr wrench_srv_;

  rclcpp::Subscription<piper_x_interfaces::msg::JointCommand>::SharedPtr joints_sub_;
  rclcpp::Subscription<piper_x_interfaces::msg::CartesianCommand>::SharedPtr cartesian_sub_;
  rclcpp::Subscription<piper_x_interfaces::msg::MitCommand>::SharedPtr mit_sub_;
  rclcpp::Subscription<piper_x_interfaces::msg::GripperCommand>::SharedPtr gripper_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr teaching_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr teaching_start_client_;
  rclcpp::TimerBase::SharedPtr main_timer_;

  sensor_msgs::msg::JointState joint_msg_;
  geometry_msgs::msg::PoseStamped pose_msg_;
  geometry_msgs::msg::WrenchStamped wrench_msg_;
  piper_x_interfaces::msg::ArmStatus status_msg_;
  std_msgs::msg::String control_msg_;

  RateMeter loop_rate_;
  RateMeter can_rx_rate_;
  RateMeter joints_pub_rate_;
  RateMeter wrench_pub_rate_;
  RateMeter pose_pub_rate_;
  RateMeter status_pub_rate_;
  RateMeter control_pub_rate_;
  uint64_t last_can_rx_updates_{0};
  double last_control_state_pub_{0.0};
  int last_err_code_{-1};
  bool wrench_estimation_active_{false};
  bool wrench_estimate_ready_{false};
  bool wrench_estimation_param_{false};
  bool shutdown_done_{false};

  struct StateKey {
    std::string mode;
    bool mit_ready{false};
    bool holding{false};
    std::string err_key;
    bool wrench_enabled{false};
    bool wrench_available{false};
    bool calibration_active{false};
    std::array<bool, 7> enabled{};

    bool operator==(const StateKey & o) const noexcept {
      return mode == o.mode && mit_ready == o.mit_ready && holding == o.holding &&
             err_key == o.err_key && wrench_enabled == o.wrench_enabled &&
             wrench_available == o.wrench_available &&
             calibration_active == o.calibration_active && enabled == o.enabled;
    }
    bool operator!=(const StateKey & o) const noexcept { return !(*this == o); }
  };
  std::optional<StateKey> last_state_key_;
};

}  // namespace piper_x_driver
