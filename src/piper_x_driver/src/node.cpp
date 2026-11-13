#include "piper_x_driver/node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <sstream>
#include <stdexcept>

namespace piper_x_driver {
namespace {

void append_json_vec6(std::string & out, const Vec6 & v) {
  out.push_back('[');
  for (size_t i = 0; i < 6; ++i) {
    if (i > 0) out.append(", ", 2);
    out += json_number(v[i]);
  }
  out.push_back(']');
}

void append_json_vec3(std::string & out, const Vec3 & v) {
  out.push_back('[');
  out += json_number(v[0]);
  out.append(", ", 2);
  out += json_number(v[1]);
  out.append(", ", 2);
  out += json_number(v[2]);
  out.push_back(']');
}

}  // namespace

double DriverNode::steady_now() noexcept {
  using Clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

DriverNode::DriverNode(
  const rclcpp::NodeOptions & options,
  std::unique_ptr<ArmBackend> custom_arm,
  bool create_default_timer)
: rclcpp::Node("piper_x_driver", options)
{
  this->declare_parameter<std::string>("config_file", default_config_path());
  fleet_ = std::make_unique<FleetConfig>(this->get_parameter("config_file").as_string());

  const std::string ns = this->get_namespace();
  std::string default_arm;
  for (const auto & a : fleet_->arms) {
    if (a.namespace_str == ns) {
      default_arm = a.id;
      break;
    }
  }
  if (default_arm.empty() && ns == "/") {
    default_arm = fleet_->arms.front().id;
  }
  this->declare_parameter<std::string>("arm_id", default_arm);
  this->declare_parameter<double>("rate_hz", 200.0);
  this->declare_parameter<double>("telemetry_hz", 200.0);
  this->declare_parameter<bool>("wrench_estimation_enabled", false);
  wrench_estimation_param_ = this->get_parameter("wrench_estimation_enabled").as_bool();

  rate_hz_ = bounded(this->get_parameter("rate_hz").as_double(), "rate_hz", 10.0, 1000.0);
  telemetry_hz_ = bounded(
    this->get_parameter("telemetry_hz").as_double(), "telemetry_hz", 10.0, rate_hz_);

  arm_config_ = fleet_->arm(this->get_parameter("arm_id").as_string());
  if (ns != "/" && ns != arm_config_.namespace_str) {
    throw std::invalid_argument("Driver namespace does not match configured arm");
  }
  public_config_prefix_ =
    "{\"configuration\": " + fleet_->public_json(arm_config_) + ", ";

  this->declare_parameter<double>("friction_error_epsilon", arm_config_.friction_error_epsilon);
  this->declare_parameter<double>(
    "friction_velocity_epsilon", arm_config_.friction_velocity_epsilon);
  this->declare_parameter<double>(
    "target_velocity_cutoff_hz", arm_config_.target_velocity_cutoff_hz);
  this->declare_parameter<double>("s_a", arm_config_.s_a);
  this->declare_parameter<double>("adaptive_stiffness_scale", arm_config_.s_a);
  this->declare_parameter<double>("adaptive_stiffness_b", arm_config_.adaptive_stiffness_b);
  this->declare_parameter<double>(
    "adaptive_stiffness_power", arm_config_.adaptive_stiffness_power);
  this->declare_parameter<double>("offset_ramp_s", arm_config_.offset_ramp_s);

  if (custom_arm) {
    arm_ = std::move(custom_arm);
  } else {
    arm_ = std::make_unique<SocketCanArm>(arm_config_.can_port);
  }

  std::optional<ArmGravityModel> gm;
  if (const auto * gptr = fleet_->gravity_model(arm_config_.id)) {
    gm = *gptr;
  }
  std::optional<ArmFrictionModel> fm;
  if (const auto * fptr = fleet_->friction_model(arm_config_.id)) {
    fm = *fptr;
  }
  control_ = std::make_unique<Controller>(
    *arm_,
    &DriverNode::steady_now,
    fleet_->effectors.at(arm_config_.end_effector),
    arm_config_.gravity_scale,
    gm,
    arm_config_.friction_scale,
    fm,
    this->get_parameter("friction_error_epsilon").as_double(),
    this->get_parameter("friction_velocity_epsilon").as_double(),
    this->get_parameter("target_velocity_cutoff_hz").as_double(),
    this->get_parameter("s_a").as_double(),
    this->get_parameter("adaptive_stiffness_b").as_double(),
    this->get_parameter("adaptive_stiffness_power").as_double(),
    this->get_parameter("offset_ramp_s").as_double(),
    arm_config_.kp,
    arm_config_.kd);

  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&DriverNode::on_parameters_set, this, std::placeholders::_1));

  joints_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states_feedback", 10);
  status_pub_ = this->create_publisher<piper_x_interfaces::msg::ArmStatus>("arm_status", 10);
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("end_pose_stamped", 10);
  wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("end_wrench_stamped", 10);
  control_pub_ = this->create_publisher<std_msgs::msg::String>("control_state", 10);

  joint_msg_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"};
  joint_msg_.position.resize(7, 0.0);
  joint_msg_.velocity.reserve(6);
  joint_msg_.effort.reserve(6);
  control_msg_.data.reserve(4096);
  pose_msg_.header.frame_id = "base_link";
  wrench_msg_.header.frame_id = "gripper";

  last_can_rx_updates_ = arm_->can_rx_updates();
  wrench_estimation_active_ = wrench_estimation_enabled();

  enable_srv_ = this->create_service<EnableSrv>(
    "enable_srv",
    [this](
      const std::shared_ptr<rmw_request_id_t> header,
      const std::shared_ptr<EnableSrv::Request> req)
    {
      this->handle_enable(header, req);
    });
  calibrate_srv_ = this->create_service<CalibrateEffectorSrv>(
    "calibrate_effector",
    [this](
      const std::shared_ptr<rmw_request_id_t> header,
      const std::shared_ptr<CalibrateEffectorSrv::Request> req)
    {
      this->handle_calibrate_effector(header, req);
    });
  mode_srv_ = this->create_service<SetModeSrv>(
    "set_mode",
    std::bind(&DriverNode::handle_set_mode, this, std::placeholders::_1, std::placeholders::_2));
  configure_srv_ = this->create_service<ConfigureArmSrv>(
    "configure_arm",
    std::bind(&DriverNode::handle_configure, this, std::placeholders::_1, std::placeholders::_2));
  wrench_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "set_wrench_estimation",
    std::bind(
      &DriverNode::handle_set_wrench_estimation, this,
      std::placeholders::_1, std::placeholders::_2));

  const auto realtime_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
  joints_sub_ = this->create_subscription<piper_x_interfaces::msg::JointCommand>(
    "command/joints", 10,
    std::bind(&DriverNode::on_joints_cmd, this, std::placeholders::_1));
  cartesian_sub_ = this->create_subscription<piper_x_interfaces::msg::CartesianCommand>(
    "command/cartesian", 10,
    std::bind(&DriverNode::on_cartesian_cmd, this, std::placeholders::_1));
  mit_sub_ = this->create_subscription<piper_x_interfaces::msg::MitCommand>(
    "command/mit", realtime_qos,
    std::bind(&DriverNode::on_mit_cmd, this, std::placeholders::_1));
  gripper_sub_ = this->create_subscription<piper_x_interfaces::msg::GripperCommand>(
    "command/gripper", realtime_qos,
    std::bind(&DriverNode::on_gripper_cmd, this, std::placeholders::_1));
  teaching_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/bilateral_teaching/active", realtime_qos,
    std::bind(&DriverNode::on_teaching, this, std::placeholders::_1));
  teaching_start_client_ = this->create_client<std_srvs::srv::Trigger>("/bilateral_teaching/start");

  try {
    control_->start_mit();
  } catch (const std::exception & error) {
    set_error(error.what());
  }
  if (create_default_timer) {
    main_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / telemetry_hz_),
      std::bind(&DriverNode::publish_step, this));
  }
  RCLCPP_INFO(
    this->get_logger(),
    "REAL HARDWARE: started in MIT mode with gravity compensation (inner=%g Hz, telemetry=%g Hz)",
    rate_hz_, telemetry_hz_);
}

DriverNode::~DriverNode() {
  shutdown_driver();
}

void DriverNode::shutdown_driver() noexcept {
  if (shutdown_done_) {
    return;
  }
  shutdown_done_ = true;
  if (main_timer_) {
    main_timer_->cancel();
    main_timer_.reset();
  }
  if (pending_power_.has_value() && pending_power_->timer) {
    pending_power_->timer->cancel();
  }
  if (pending_calibration_.has_value() && pending_calibration_->timer) {
    pending_calibration_->timer->cancel();
  }
  if (calibration_thread_.joinable()) {
    calibration_thread_.join();
  }
  try {
    if (control_ && control_->shutdown_to_drag() && rclcpp::ok()) {
      publish_control_state();
    }
  } catch (const std::exception & error) {
    try {
      set_error(std::string("Driver shutdown: ") + error.what() + "; actual motor power unconfirmed");
    } catch (...) {
    }
  }
  try {
    if (arm_) {
      arm_->close();
    }
  } catch (...) {
  }
}

void DriverNode::trigger_teaching_retry() {
  if (teaching_start_client_ && teaching_start_client_->service_is_ready()) {
    RCLCPP_INFO(this->get_logger(), "Enter pressed: requesting teaching retry");
    teaching_start_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  }
}

bool DriverNode::wrench_estimation_enabled() const {
  return wrench_estimation_param_;
}

bool DriverNode::wrench_estimate_available() const {
  return wrench_estimation_enabled() && wrench_estimate_ready_ &&
         control_->mode == "mit" && control_->mit_ready && control_->hold_reason.empty();
}

bool DriverNode::sync_wrench_estimation() {
  const bool enabled = wrench_estimation_enabled();
  if (enabled != wrench_estimation_active_) {
    control_->reset_wrench_estimation();
    wrench_pub_rate_.clear();
    wrench_estimate_ready_ = false;
    wrench_estimation_active_ = enabled;
  }
  return enabled;
}

rcl_interfaces::msg::SetParametersResult DriverNode::on_parameters_set(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  ControllerReconfigureArgs updates;
  bool any = false;
  std::optional<bool> next_wrench_param;
  for (const auto & param : parameters) {
    const std::string & name = param.get_name();
    if (name == "wrench_estimation_enabled") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "wrench_estimation_enabled must be bool";
        return result;
      }
      next_wrench_param = param.as_bool();
      continue;
    }
    auto get_num = [&](double & dst) -> bool {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        dst = param.as_double();
        return true;
      }
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        dst = static_cast<double>(param.as_int());
        return true;
      }
      result.successful = false;
      result.reason = name + " must be a finite number";
      return false;
    };
    double val = 0.0;
    if (name == "friction_error_epsilon") {
      if (!get_num(val)) return result;
      updates.friction_error_epsilon = val;
      any = true;
    } else if (name == "friction_velocity_epsilon") {
      if (!get_num(val)) return result;
      updates.friction_velocity_epsilon = val;
      any = true;
    } else if (name == "target_velocity_cutoff_hz") {
      if (!get_num(val)) return result;
      updates.target_velocity_cutoff_hz = val;
      any = true;
    } else if (name == "s_a" || name == "adaptive_stiffness_scale") {
      if (!get_num(val)) return result;
      updates.s_a = val;
      any = true;
    } else if (name == "adaptive_stiffness_b") {
      if (!get_num(val)) return result;
      updates.adaptive_stiffness_b = val;
      any = true;
    } else if (name == "adaptive_stiffness_power") {
      if (!get_num(val)) return result;
      updates.adaptive_stiffness_power = val;
      any = true;
    } else if (name == "offset_ramp_s") {
      if (!get_num(val)) return result;
      updates.offset_ramp_s = val;
      any = true;
    }
  }
  if (any) {
    try {
      control_->reconfigure(updates);
    } catch (const std::exception & error) {
      result.successful = false;
      result.reason = error.what();
      return result;
    }
  }
  if (next_wrench_param.has_value()) {
    wrench_estimation_param_ = *next_wrench_param;
  }
  return result;
}

std::string DriverNode::error_key(const std::string & message) {
  static constexpr const char * kPrefix =
    "Complete fresh joint/gripper/status/enable feedback required:";
  if (message.rfind(kPrefix, 0) == 0) {
    return "feedback_stale";
  }
  return message;
}

void DriverNode::set_error(const std::string & message) {
  if (error_key(message) != error_key(last_error_)) {
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
  }
  last_error_ = message;
}

void DriverNode::on_teaching(const std_msgs::msg::Bool::SharedPtr msg) {
  teaching_active_ = msg->data;
  teaching_received_ = steady_now();
}

void DriverNode::on_joints_cmd(const piper_x_interfaces::msg::JointCommand::SharedPtr msg) {
  try {
    if (calibration_active_) {
      throw std::invalid_argument("Effector calibration in progress; commands rejected");
    }
    control_->joints(msg->position, msg->speed_percent);
    last_error_.clear();
  } catch (const std::exception & error) {
    set_error(error.what());
  }
}

void DriverNode::on_cartesian_cmd(const piper_x_interfaces::msg::CartesianCommand::SharedPtr msg) {
  try {
    if (calibration_active_) {
      throw std::invalid_argument("Effector calibration in progress; commands rejected");
    }
    control_->cartesian(msg->pose, msg->speed_percent);
    last_error_.clear();
  } catch (const std::exception & error) {
    set_error(error.what());
  }
}

void DriverNode::on_mit_cmd(const piper_x_interfaces::msg::MitCommand::SharedPtr msg) {
  try {
    if (calibration_active_) {
      throw std::invalid_argument("Effector calibration in progress; commands rejected");
    }
    MitTarget t;
    t.position = msg->position;
    t.velocity = msg->velocity;
    t.kp = msg->kp;
    t.kd = msg->kd;
    t.torque = msg->torque;
    control_->mit(t);
    last_error_.clear();
  } catch (const std::exception & error) {
    set_error(error.what());
  }
}

void DriverNode::on_gripper_cmd(const piper_x_interfaces::msg::GripperCommand::SharedPtr msg) {
  try {
    if (calibration_active_) {
      throw std::invalid_argument("Effector calibration in progress; commands rejected");
    }
    control_->gripper(msg->width, msg->effort);
    last_error_.clear();
  } catch (const std::exception & error) {
    set_error(error.what());
  }
}

Feedback DriverNode::calibration_feedback(std::optional<Feedback> sample) {
  if (power_request_.has_value() || teaching_active_) {
    throw std::invalid_argument(
      "Stop teaching and wait for power requests to finish before calibration");
  }
  if (control_->mode != "standby" ||
      (control_->mit_received.has_value() &&
       control_->clock() - *control_->mit_received <= 0.2))
  {
    throw std::invalid_argument("Stop MIT/other controllers and select standby before calibration");
  }
  Feedback s = control_->healthy(false, std::move(sample));
  for (bool en : s.enabled) {
    if (en) {
      throw std::invalid_argument(
        "Fresh feedback must confirm all six joints and effector disabled");
    }
  }
  const auto & st = *s.status;
  const bool teach_ok = (st.teach_status == 0 || st.teach_status == 2 || st.teach_status == 6);
  const bool ctrl_ok = (st.ctrl_mode == 0 || st.ctrl_mode == 1 || st.ctrl_mode == 2);
  const bool mode_ok = (st.mode_feedback == 0 || st.mode_feedback == 1 ||
                        st.mode_feedback == 2 || st.mode_feedback == 3 || st.mode_feedback == 5);
  if (st.arm_status != 0 || !teach_ok || !ctrl_ok || !mode_ok) {
    throw std::invalid_argument(
      "Leave active native teaching/MIT/remote control before calibration");
  }
  return s;
}

std::optional<std::string> DriverNode::calibration_block_reason(std::optional<Feedback> sample) {
  if (calibration_active_) {
    return "Effector calibration in progress; do not repeat the request";
  }
  if (power_request_.has_value() || teaching_active_) {
    return "Stop teaching and wait for power requests to finish before calibration";
  }
  if (control_->mode != "standby" ||
      (control_->mit_received.has_value() &&
       control_->clock() - *control_->mit_received <= 0.2))
  {
    return "Stop MIT/other controllers and select standby before calibration";
  }
  try {
    calibration_feedback(std::move(sample));
  } catch (const std::exception & error) {
    return error.what();
  }
  return std::nullopt;
}

void DriverNode::publish_control_state(const Feedback * sample) {
  const double now = steady_now();
  control_pub_rate_.mark(now);
  last_control_state_pub_ = now;
  const double ctrl_now = control_->clock();
  const bool w_avail = wrench_estimate_available();
  const auto block_reason = calibration_block_reason(
    sample != nullptr ? std::optional<Feedback>(*sample) : std::nullopt);
  const bool mit_target_live =
    control_->mode == "mit" && control_->mit_target.has_value() &&
    control_->mit_received.has_value() && (ctrl_now - *control_->mit_received <= 0.2);

  std::string & out = control_msg_.data;
  out.clear();
  out += public_config_prefix_;
  out += "\"mode\": \""; out += json_escape(control_->mode); out += "\", ";
  out += "\"mit_ready\": "; out += ((control_->mode == "mit" && control_->mit_ready) ? "true" : "false"); out += ", ";
  out += "\"mit_target\": ";
  if (mit_target_live) append_json_vec6(out, control_->mit_target->position); else out += "null";
  out += ", \"joint_lower\": "; append_json_vec6(out, SLIDER_LOWER);
  out += ", \"joint_upper\": "; append_json_vec6(out, SLIDER_UPPER);
  out += ", \"gravity_torque\": "; append_json_vec6(out, control_->gravity);
  out += ", \"friction_torque\": "; append_json_vec6(out, control_->friction);
  out += ", \"gripper_friction\": "; out += json_number(control_->gripper_friction);
  out += ", \"gripper_velocity\": "; out += json_number(sample ? sample->gripper_velocity : 0.0);
  out += ", \"gripper_force\": "; out += json_number(sample ? sample->gripper_force : 0.0);
  out += ", \"output_torque\": "; append_json_vec6(out, control_->output_torque);
  out += ", \"applied_kp\": "; append_json_vec6(out, control_->applied_kp);
  out += ", \"applied_kd\": "; append_json_vec6(out, control_->applied_kd);
  out += ", \"wrench_estimation_enabled\": "; out += (wrench_estimation_enabled() ? "true" : "false");
  out += ", \"wrench_estimate_available\": "; out += (w_avail ? "true" : "false");
  out += ", \"inertia_torque\": "; if (w_avail) append_json_vec6(out, control_->inertia_torque); else out += "null";
  out += ", \"external_torque\": "; if (w_avail) append_json_vec6(out, control_->external_torque); else out += "null";
  out += ", \"end_wrench\": "; if (w_avail) append_json_vec6(out, control_->end_wrench); else out += "null";
  out += ", \"end_force\": "; if (w_avail) append_json_vec3(out, control_->end_force()); else out += "null";
  out += ", \"end_torque\": "; if (w_avail) append_json_vec3(out, control_->end_torque()); else out += "null";
  const std::string sa_str = json_number(control_->adaptive_stiffness_scale_applied);
  out += ", \"gravity_scale_applied\": "; out += json_number(control_->gravity_scale_applied);
  out += ", \"friction_scale_applied\": "; out += json_number(control_->friction_scale_applied);
  out += ", \"s_a_applied\": "; out += sa_str;
  out += ", \"adaptive_stiffness_scale_applied\": "; out += sa_str;
  out += ", \"holding\": "; out += (!control_->hold_reason.empty() ? "true" : "false");
  out += ", \"hold_reason\": \""; out += json_escape(control_->hold_reason);
  out += "\", \"calibration_active\": "; out += (calibration_active_ ? "true" : "false");
  out += ", \"calibration_blocked_reason\": ";
  if (block_reason.has_value()) { out.push_back('"'); out += json_escape(*block_reason); out.push_back('"'); } else { out += "null"; }
  out += ", \"last_error\": \""; out += json_escape(last_error_);
  out += "\", \"rates\": {\"target_hz\": "; out += json_number(rate_hz_);
  out += ", \"telemetry_target_hz\": "; out += json_number(telemetry_hz_);
  out += ", \"can_rx_hz\": "; out += json_number(can_rx_rate_.hz(now));
  out += ", \"loop_hz\": "; out += json_number(loop_rate_.hz(now));
  out += ", \"comp_hz\": "; out += json_number(control_->comp_rate.hz(ctrl_now));
  out += ", \"cmd_tx_hz\": "; out += json_number(control_->cmd_tx_rate.hz(ctrl_now));
  out += ", \"mit_tx_hz\": "; out += json_number(control_->mit_tx_rate.hz(ctrl_now));
  out += ", \"grip_tx_hz\": "; out += json_number(control_->grip_tx_rate.hz(ctrl_now));
  out += ", \"ros_cmd_hz\": "; out += json_number(control_->ros_cmd_rate.hz(ctrl_now));
  out += ", \"joints_pub_hz\": "; out += json_number(joints_pub_rate_.hz(now));
  out += ", \"wrench_pub_hz\": "; out += json_number(wrench_pub_rate_.hz(now));
  out += ", \"pose_pub_hz\": "; out += json_number(pose_pub_rate_.hz(now));
  out += ", \"status_pub_hz\": "; out += json_number(status_pub_rate_.hz(now));
  out += ", \"control_pub_hz\": "; out += json_number(control_pub_rate_.hz(now));
  out += "}}";
  control_pub_->publish(control_msg_);
}

void DriverNode::handle_enable(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<EnableSrv::Request> request)
{
  EnableSrv::Response immediate;
  if (request->enable_request && calibration_active_) {
    immediate.enable_response = false;
    immediate.message = "Effector calibration in progress; enabling rejected";
    enable_srv_->send_response(*request_header, immediate);
    return;
  }

  // If an earlier async power request is still pending, supersede it immediately.
  if (pending_power_.has_value()) {
    if (pending_power_->timer) {
      pending_power_->timer->cancel();
    }
    if (pending_power_->enabled) {
      EnableSrv::Response sup;
      sup.enable_response = false;
      sup.message = "Power request superseded by a later request";
      set_error(sup.message);
      enable_srv_->send_response(*pending_power_->header, sup);
    } else {
      EnableSrv::Response sup;
      sup.enable_response = false;
      sup.message = "Power request superseded";
      enable_srv_->send_response(*pending_power_->header, sup);
    }
    pending_power_.reset();
  }

  const double started = steady_now();
  ++power_serial_;
  const uint64_t serial = power_serial_;
  power_request_ = request->enable_request;
  try {
    control_->power(request->enable_request);
  } catch (const std::exception & error) {
    set_error(error.what());
    immediate.enable_response = false;
    immediate.message = error.what();
    if (power_serial_ == serial) {
      power_request_.reset();
    }
    enable_srv_->send_response(*request_header, immediate);
    return;
  }

  PendingPower pp;
  pp.header = request_header;
  pp.enabled = request->enable_request;
  pp.started = started;
  pp.serial = serial;
  pp.timer = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&DriverNode::check_power, this));
  pending_power_ = std::move(pp);
  check_power();
}

void DriverNode::check_power() {
  if (!pending_power_.has_value()) {
    return;
  }
  auto finish = [&](bool ok, const std::string & msg) {
    auto pp = std::move(*pending_power_);
    pending_power_.reset();
    if (pp.timer) {
      pp.timer->cancel();
    }
    if (power_serial_ == pp.serial) {
      power_request_.reset();
    }
    if (ok) {
      last_error_.clear();
    } else {
      set_error(msg);
    }
    EnableSrv::Response resp;
    resp.enable_response = ok;
    resp.message = msg;
    enable_srv_->send_response(*pp.header, resp);
  };

  const auto & pp = *pending_power_;
  if (pp.enabled && pp.serial != power_serial_) {
    finish(false, "Power request superseded by a later request");
    return;
  }
  try {
    const Feedback sample = arm_->snapshot();
    const double elapsed = steady_now() - pp.started;
    const double power_ages[7] = {
      sample.age[3], sample.age[5], sample.age[6], sample.age[7],
      sample.age[8], sample.age[9], sample.age[10]
    };
    bool fresh_after_req = fresh_ages(power_ages, 7, 1.0);
    if (fresh_after_req) {
      for (double a : power_ages) {
        if (a < 0.0 || a > elapsed) {
          fresh_after_req = false;
          break;
        }
      }
    }
    bool state_matched = false;
    if (pp.enabled) {
      state_matched = sample.enabled[0] && sample.enabled[1] && sample.enabled[2] &&
                      sample.enabled[3] && sample.enabled[4] && sample.enabled[5];
    } else {
      state_matched = !(sample.enabled[0] || sample.enabled[1] || sample.enabled[2] ||
                        sample.enabled[3] || sample.enabled[4] || sample.enabled[5] ||
                        sample.enabled[6]);
    }
    if (!pp.enabled && control_->mode == "drag") {
      finish(true, "Fresh feedback confirms all seven enable flags");
    } else if (fresh_after_req && state_matched) {
      finish(true, "Fresh feedback confirms all seven enable flags");
    } else if (elapsed > 3.0) {
      finish(false, "Power request not confirmed within 3 s");
    }
  } catch (const std::exception & error) {
    if (steady_now() - pp.started > 3.0) {
      finish(false, error.what());
    }
  }
}

void DriverNode::handle_calibrate_effector(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<CalibrateEffectorSrv::Request> request)
{
  CalibrateEffectorSrv::Response immediate;
  if (!request->confirm_closed) {
    immediate.success = false;
    immediate.message = "Explicit confirmation of the physically closed, unloaded zero required";
    set_error(immediate.message);
    calibrate_srv_->send_response(*request_header, immediate);
    return;
  }
  const auto reason = calibration_block_reason();
  if (reason.has_value()) {
    immediate.success = false;
    immediate.message = *reason;
    set_error(*reason);
    calibrate_srv_->send_response(*request_header, immediate);
    return;
  }

  if (calibration_thread_.joinable()) {
    calibration_thread_.join();
  }
  calibration_active_ = true;
  const uint64_t serial = power_serial_;
  RCLCPP_INFO(this->get_logger(), "Operator requested effector zero calibration; all axes disabled");

  auto promise = std::make_shared<std::promise<void>>();
  std::shared_future<void> fut = promise->get_future().share();
  calibration_thread_ = std::thread([this, promise]() {
    try {
      arm_->calibrate_effector();
      promise->set_value();
    } catch (...) {
      promise->set_exception(std::current_exception());
    }
  });

  PendingCalibration pc;
  pc.header = request_header;
  pc.serial = serial;
  pc.worker_future = fut;
  pc.timer = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&DriverNode::check_calibration, this));
  pending_calibration_ = std::move(pc);
  publish_control_state();
}

void DriverNode::check_calibration() {
  if (!pending_calibration_.has_value()) {
    return;
  }
  auto & pc = *pending_calibration_;
  if (pc.worker_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return;
  }
  auto finish = [&](bool ok, const std::string & msg) {
    auto done = std::move(*pending_calibration_);
    pending_calibration_.reset();
    if (done.timer) {
      done.timer->cancel();
    }
    if (calibration_thread_.joinable()) {
      calibration_thread_.join();
    }
    calibration_active_ = false;
    CalibrateEffectorSrv::Response resp;
    resp.success = ok;
    if (ok) {
      resp.message = msg;
      RCLCPP_INFO(this->get_logger(), "%s", resp.message.c_str());
      last_error_.clear();
    } else {
      resp.message = "Calibration failed or unconfirmed: " + msg + "; no automatic retry";
      set_error(resp.message);
    }
    publish_control_state();
    calibrate_srv_->send_response(*done.header, resp);
  };

  try {
    if (!pc.confirm_after.has_value()) {
      pc.worker_future.get();
      pc.confirm_after = steady_now();
    }
    if (pc.serial != power_serial_) {
      throw std::invalid_argument("Calibration superseded by a power request; inspect actual feedback");
    }
    const Feedback sample = calibration_feedback();
    const double elapsed = steady_now() - *pc.confirm_after;
    bool all_fresh_since = true;
    for (double a : sample.age) {
      if (!(0.0 <= a && a <= elapsed)) {
        all_fresh_since = false;
        break;
      }
    }
    const double width = (*sample.position)[6];
    if (all_fresh_since && 0.0 <= width && width <= 0.0001) {
      char buf[256];
      std::snprintf(
        buf, sizeof(buf),
        "Device acknowledged effector zero calibration; fresh opening %.3f mm; "
        "all seven axes disabled, no enable or motion targets sent. "
        "Power-cycle persistence not verified.",
        width * 1000.0);
      finish(true, buf);
    } else if (elapsed > 0.5) {
      throw std::invalid_argument(
        "Firmware acknowledged calibration but fresh disabled feedback with "
        "opening 0..0.1 mm was not confirmed within 0.5 s; result unverified");
    }
  } catch (const std::exception & error) {
    finish(false, error.what());
  }
}

void DriverNode::handle_set_mode(
  const std::shared_ptr<SetModeSrv::Request> request,
  std::shared_ptr<SetModeSrv::Response> response)
{
  try {
    if (calibration_active_) {
      throw std::invalid_argument("Wait for effector calibration to finish before changing modes");
    }
    if (request->mode == "mit" && control_->mode == "mit" && control_->mit_ready) {
      control_->release_mit();
      last_error_.clear();
      response->success = true;
      response->message = "MIT target released; holding gravity compensation";
      publish_control_state();
      return;
    }
    control_->set_mode(request->mode, true);
    last_error_.clear();
    response->success = true;
    response->message = "Request sent; inspect ctrl_mode/mode_feedback/teach_status";
  } catch (const std::exception & error) {
    set_error(error.what());
    response->success = false;
    response->message = error.what();
  }
  publish_control_state();
}

void DriverNode::handle_configure(
  const std::shared_ptr<ConfigureArmSrv::Request> request,
  std::shared_ptr<ConfigureArmSrv::Response> response)
{
  try {
    if (calibration_active_) {
      throw std::invalid_argument("Wait for effector calibration to finish before configuring");
    }
    if (request->end_effector != arm_config_.end_effector) {
      const Feedback sample = arm_->snapshot();
      const double p_ages[7] = {
        sample.age[3], sample.age[5], sample.age[6], sample.age[7],
        sample.age[8], sample.age[9], sample.age[10]
      };
      bool any_en = false;
      for (bool en : sample.enabled) {
        if (en) any_en = true;
      }
      if (teaching_active_ || control_->mode == "mit" || any_en || !fresh_ages(p_ages, 7, 1.0)) {
        throw std::invalid_argument(
          "Stop MIT/teaching and confirm all seven axes disabled before changing the end effector");
      }
    }
    const double raw_f_scale = request->friction_scale;
    if (raw_f_scale < -1.0) {
      const bool is_restore = (raw_f_scale <= -3.5);
      const double offset = is_restore ? 4.0 : 2.0;
      const double rounded = std::round((-raw_f_scale - offset) * 1e12) / 1e12;
      const double temp_scale = bounded(rounded, "friction_scale", 0.0, 1.0);
      const bool calibrating_zero = (!is_restore) && (temp_scale == 0.0);
      if (calibrating_zero) {
        ControllerReconfigureArgs args;
        args.friction_scale = 0.0;
        args.s_a = 0.0;
        control_->reconfigure(args);
      } else {
        auto latest = std::make_unique<FleetConfig>(fleet_->path);
        arm_config_ = latest->arm(arm_config_.id);
        public_config_prefix_ = "{\"configuration\": " + latest->public_json(arm_config_) + ", ";
        ControllerReconfigureArgs args;
        args.effector = latest->effectors.at(arm_config_.end_effector);
        if (const auto * gp = latest->gravity_model(arm_config_.id)) {
          args.gravity_model = *gp;
        }
        args.friction_scale = temp_scale;
        if (const auto * fp = latest->friction_model(arm_config_.id)) {
          args.friction_model = *fp;
        }
        args.s_a = arm_config_.s_a;
        args.update_models = true;
        control_->reconfigure(args);
        fleet_ = std::move(latest);
      }
      last_error_.clear();
      response->success = true;
      response->message = "Temporary runtime friction_scale applied without modifying arms.yaml";
      publish_control_state();
      return;
    }

    const double next_f_scale =
      (raw_f_scale == -1.0) ? arm_config_.friction_scale : raw_f_scale;
    FleetConfig updated = fleet_->save_arm(
      arm_config_, request->end_effector, request->gravity_scale, next_f_scale);
    fleet_ = std::make_unique<FleetConfig>(std::move(updated));
    arm_config_ = fleet_->arm(arm_config_.id);
    public_config_prefix_ = "{\"configuration\": " + fleet_->public_json(arm_config_) + ", ";

    ControllerReconfigureArgs args;
    args.effector = fleet_->effectors.at(arm_config_.end_effector);
    args.gravity_scale = arm_config_.gravity_scale;
    if (const auto * gp = fleet_->gravity_model(arm_config_.id)) {
      args.gravity_model = *gp;
    }
    args.friction_scale = arm_config_.friction_scale;
    if (const auto * fp = fleet_->friction_model(arm_config_.id)) {
      args.friction_model = *fp;
    }
    args.friction_error_epsilon = arm_config_.friction_error_epsilon;
    args.friction_velocity_epsilon = arm_config_.friction_velocity_epsilon;
    args.target_velocity_cutoff_hz = arm_config_.target_velocity_cutoff_hz;
    args.s_a = arm_config_.s_a;
    args.adaptive_stiffness_b = arm_config_.adaptive_stiffness_b;
    args.adaptive_stiffness_power = arm_config_.adaptive_stiffness_power;
    args.offset_ramp_s = arm_config_.offset_ramp_s;
    args.update_models = true;
    control_->reconfigure(args);

    last_error_.clear();
    response->success = true;
    response->message = "Configuration saved and applied; no power or mode change";
  } catch (const std::exception & error) {
    set_error(error.what());
    response->success = false;
    response->message = error.what();
  }
  publish_control_state();
}

void DriverNode::handle_set_wrench_estimation(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  auto res = this->set_parameter(rclcpp::Parameter("wrench_estimation_enabled", request->data));
  response->success = res.successful;
  response->message = !res.reason.empty()
    ? res.reason
    : (request->data ? "Wrench estimation enabled" : "Wrench estimation disabled");
  sync_wrench_estimation();
  publish_control_state();
}

void DriverNode::record_can_rx(const Feedback * sample, double now) {
  if (arm_->is_real_hardware()) {
    const uint64_t updates = arm_->can_rx_updates();
    if (updates != last_can_rx_updates_) {
      last_can_rx_updates_ = updates;
      can_rx_rate_.mark(now);
    }
  } else if (sample != nullptr && sample->position.has_value() &&
             fresh_ages(sample->age.data(), 4, 1.0))
  {
    can_rx_rate_.mark(now);
  }
}

void DriverNode::fast_control_step() {
  const double now = steady_now();
  loop_rate_.mark(now);
  try {
    const Feedback sample = control_->watchdog(false);
    record_can_rx(&sample, now);
  } catch (const std::exception & error) {
    set_error(error.what());
  }
}

void DriverNode::publish_step() {
  const double now = steady_now();
  const bool estimate = sync_wrench_estimation();
  loop_rate_.mark(now);
  if (teaching_active_ && now - teaching_received_ > 0.3) {
    teaching_active_ = false;
    RCLCPP_ERROR(this->get_logger(), "Teaching heartbeat expired");
  }
  Feedback sample;
  try {
    sample = control_->watchdog(estimate);
    if (estimate && control_->mode == "mit" && control_->mit_ready) {
      wrench_estimate_ready_ = true;
    }
    if (last_error_.find("retaining last MIT output") != std::string::npos) {
      last_error_.clear();
    }
  } catch (const std::exception & error) {
    set_error(error.what());
    try {
      sample = arm_->snapshot();
    } catch (const std::exception & fb_error) {
      set_error(std::string(error.what()) + "; feedback unavailable: " + fb_error.what());
      publish_control_state();
      return;
    }
  }
  record_can_rx(&sample, now);

  StateKey skey{
    control_->mode,
    control_->mit_ready,
    !control_->hold_reason.empty(),
    error_key(last_error_),
    wrench_estimation_enabled(),
    wrench_estimate_available(),
    calibration_active_,
    sample.enabled
  };
  const bool publish_slow =
    (last_control_state_pub_ == 0.0) ||
    (!last_state_key_.has_value() || *last_state_key_ != skey) ||
    (now - last_control_state_pub_ >= 0.0195);
  if (publish_slow) {
    last_state_key_ = std::move(skey);
    publish_control_state(&sample);
  }

  const auto stamp = this->get_clock()->now();
  if (publish_slow && sample.status.has_value() && 0.0 <= sample.age[4] && sample.age[4] <= 1.0) {
    const auto & st = *sample.status;
    status_msg_.header.stamp = stamp;
    status_msg_.ctrl_mode = static_cast<uint8_t>(st.ctrl_mode);
    status_msg_.arm_status = static_cast<uint8_t>(st.arm_status);
    status_msg_.mode_feedback = static_cast<uint8_t>(st.mode_feedback);
    status_msg_.teach_status = static_cast<uint8_t>(st.teach_status);
    status_msg_.motion_status = static_cast<uint8_t>(st.motion_status);
    status_msg_.trajectory_num = static_cast<uint8_t>(st.trajectory_num);
    const int err = st.err_code;
    status_msg_.err_code = static_cast<uint16_t>(err);
    status_msg_.feedback_age = sample.age;
    for (size_t i = 0; i < 6; ++i) {
      status_msg_.motor_enabled[i] = sample.enabled[i];
      status_msg_.motor_enable_valid[i] = (0.0 <= sample.age[5 + i] && sample.age[5 + i] <= 1.0);
    }
    status_msg_.gripper_enabled = sample.enabled[6];
    status_msg_.gripper_valid = (0.0 <= sample.age[3] && sample.age[3] <= 1.0);
    status_msg_.teaching_active = teaching_active_;
    status_msg_.last_error = last_error_;
    if (err != last_err_code_) {
      last_err_code_ = err;
      status_msg_.joint_1_angle_limit = (err & (1 << 8)) != 0;
      status_msg_.joint_2_angle_limit = (err & (1 << 9)) != 0;
      status_msg_.joint_3_angle_limit = (err & (1 << 10)) != 0;
      status_msg_.joint_4_angle_limit = (err & (1 << 11)) != 0;
      status_msg_.joint_5_angle_limit = (err & (1 << 12)) != 0;
      status_msg_.joint_6_angle_limit = (err & (1 << 13)) != 0;
      status_msg_.communication_status_joint_1 = (err & (1 << 0)) != 0;
      status_msg_.communication_status_joint_2 = (err & (1 << 1)) != 0;
      status_msg_.communication_status_joint_3 = (err & (1 << 2)) != 0;
      status_msg_.communication_status_joint_4 = (err & (1 << 3)) != 0;
      status_msg_.communication_status_joint_5 = (err & (1 << 4)) != 0;
      status_msg_.communication_status_joint_6 = (err & (1 << 5)) != 0;
    }
    status_pub_->publish(status_msg_);
    status_pub_rate_.mark(now);
  }

  if (sample.position.has_value() && fresh_ages(sample.age.data(), 4, 1.0)) {
    joint_msg_.header.stamp = stamp;
    for (size_t i = 0; i < 7; ++i) {
      joint_msg_.position[i] = (*sample.position)[i];
    }
    if (sample.has_velocity_torque) {
      joint_msg_.velocity.assign(sample.velocity.begin(), sample.velocity.end());
      joint_msg_.effort.assign(sample.torque.begin(), sample.torque.end());
    } else {
      joint_msg_.velocity.clear();
      joint_msg_.effort.clear();
    }
    joints_pub_->publish(joint_msg_);
    joints_pub_rate_.mark(now);
  }

  if (publish_slow && sample.pose.has_value()) {
    pose_msg_.header.stamp = stamp;
    const auto & p = *sample.pose;
    pose_msg_.pose.position.x = p[0];
    pose_msg_.pose.position.y = p[1];
    pose_msg_.pose.position.z = p[2];
    const double rx = 0.5 * p[3], ry = 0.5 * p[4], rz = 0.5 * p[5];
    const double cx = std::cos(rx), sx = std::sin(rx);
    const double cy = std::cos(ry), sy = std::sin(ry);
    const double cz = std::cos(rz), sz = std::sin(rz);
    pose_msg_.pose.orientation.x = sx * cy * cz - cx * sy * sz;
    pose_msg_.pose.orientation.y = cx * sy * cz + sx * cy * sz;
    pose_msg_.pose.orientation.z = cx * cy * sz - sx * sy * cz;
    pose_msg_.pose.orientation.w = cx * cy * cz + sx * sy * sz;
    pose_pub_->publish(pose_msg_);
    pose_pub_rate_.mark(now);
  }

  if (wrench_estimate_available() && sample.position.has_value() &&
      fresh_ages(sample.age.data(), 3, 1.0))
  {
    wrench_msg_.header.stamp = stamp;
    const Vec3 f = control_->end_force();
    const Vec3 t = control_->end_torque();
    wrench_msg_.wrench.force.x = f[0];
    wrench_msg_.wrench.force.y = f[1];
    wrench_msg_.wrench.force.z = f[2];
    wrench_msg_.wrench.torque.x = t[0];
    wrench_msg_.wrench.torque.y = t[1];
    wrench_msg_.wrench.torque.z = t[2];
    wrench_pub_->publish(wrench_msg_);
    wrench_pub_rate_.mark(now);
  }
}

}  // namespace piper_x_driver
