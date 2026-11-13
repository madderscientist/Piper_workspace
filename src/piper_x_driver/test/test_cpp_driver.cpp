#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "piper_x_driver/can_protocol.hpp"
#include "piper_x_driver/configuration.hpp"
#include "piper_x_driver/controller.hpp"
#include "piper_x_driver/node.hpp"
#include "piper_x_interfaces/msg/cartesian_command.hpp"
#include "piper_x_interfaces/msg/joint_command.hpp"
#include "piper_x_interfaces/msg/mit_command.hpp"
#include "piper_x_interfaces/srv/calibrate_effector.hpp"
#include "piper_x_interfaces/srv/configure_arm.hpp"
#include "piper_x_interfaces/srv/enable.hpp"
#include "piper_x_interfaces/srv/set_mode.hpp"

namespace piper_x_driver {
namespace {

class MockArm final : public ArmBackend {
public:
  explicit MockArm(std::string ch = "can0") : ch_(std::move(ch)) {
    pos_ = {{0.0, 0.5, -0.5, 0.0, 0.0, 0.0, 0.02}};
    status_ = ArmStatusFields{0, 0, 1, 0, 0, 0, 0};
  }

  std::string channel() const override { return ch_; }
  bool is_real_hardware() const override { return false; }
  bool is_powered() const override { return powered_; }
  uint64_t can_rx_updates() const override { return updates_; }

  Feedback snapshot() override {
    Feedback fb;
    fb.position = pos_;
    fb.status = status_;
    fb.enabled = enabled_;
    fb.age.fill(0.0);
    fb.pose = Vec6{{0.2, 0.0, 0.2, 0.0, 0.0, 0.0}};
    fb.has_velocity_torque = true;
    return fb;
  }

  void calibrate_effector() override {
    calls.emplace_back("calibrate_effector");
    pos_[6] = 0.0;
  }

  void power(bool en, double /*width*/) override {
    calls.emplace_back(en ? "power_on" : "power_off");
    powered_ = en;
    enabled_.fill(en);
    if (!en) {
      mode_ = "standby";
      mit_out_.reset();
    }
  }

  void mode(const std::string & name) override {
    calls.emplace_back("mode:" + name);
    mode_ = name;
    if (name == "drag" || name == "hold") {
      status_.ctrl_mode = 2;
      status_.teach_status = (name == "drag") ? 1 : 2;
      status_.mode_feedback = 0;
    } else if (name == "mit") {
      status_.ctrl_mode = 1;
      status_.teach_status = 0;
      status_.mode_feedback = 6;
    } else {
      // Match real arm_3/arm_4 firmware: standalone 0x151 without target frames
      // drops ctrl_mode back to 0 and keeps previous mode_feedback (e.g. 6).
      status_.ctrl_mode = 0;
      status_.teach_status = 0;
    }
  }

  void leave_drag() override {
    if (mode_ == "drag") {
      mode_ = "standby";
    }
  }

  void enter_drag() override {
    mode("drag");
    powered_ = false;
    mit_out_.reset();
  }

  void joints(const Vec6 & p, uint8_t /*speed*/) override {
    calls.emplace_back("joints");
    status_.ctrl_mode = 1;
    status_.mode_feedback = 1;
    for (size_t i = 0; i < 6; ++i) pos_[i] = p[i];
  }

  void cartesian(const Vec6 & p, uint8_t /*speed*/, const std::string & m) override {
    calls.emplace_back("cartesian:" + m);
    status_.ctrl_mode = 1;
    status_.mode_feedback = (m == "point") ? 0 : 2;
    pose_ = p;
  }

  void mit(const MitTarget & cmd) override {
    calls.emplace_back("mit");
    mit_out_ = cmd;
    mit_history.push_back(cmd);
  }

  void retain_mit() override {
    if (!mit_out_.has_value()) {
      throw std::runtime_error("no mit output");
    }
    calls.emplace_back("retain_mit");
  }

  void grip(double w, double eff, bool) override {
    calls.emplace_back("grip");
    if (eff > 0.0) pos_[6] = w;
  }

  void close() override {
    if (mode_ != "drag" && (mode_ == "mit" || powered_)) {
      enter_drag();
    }
  }

  std::string ch_;
  std::array<double, 7> pos_{};
  Vec6 pose_{{0.2, 0.0, 0.2, 0.0, 0.0, 0.0}};
  std::array<bool, 7> enabled_{{false, false, false, false, false, false, false}};
  ArmStatusFields status_{};
  bool powered_{false};
  std::string mode_{"standby"};
  std::optional<MitTarget> mit_out_;
  std::vector<MitTarget> mit_history;
  uint64_t updates_{1};
  std::vector<std::string> calls;
};

TEST(PiperXDriverCppTest, FleetConfigAndDynamicsLoadCleanly) {
  FleetConfig fleet(default_config_path());
  ASSERT_EQ(fleet.arms.size(), 4u);
  EXPECT_EQ(fleet.arms[0].can_port, "can0");
  EXPECT_EQ(fleet.arms[3].can_port, "can3");

  const auto & a3 = fleet.arm("arm_3");
  const auto * gm = fleet.gravity_model("arm_3");
  const auto * fm = fleet.friction_model("arm_3");
  ASSERT_NE(gm, nullptr);
  ASSERT_NE(fm, nullptr);

  const Vec6 q{{0.1, 0.5, -0.8, 0.2, -0.3, 0.1}};
  const Vec6 g = gm->torques(q, fleet.effectors.at(a3.end_effector).mass_kg,
                             fleet.effectors.at(a3.end_effector).com_m);
  for (double val : g) {
    EXPECT_TRUE(std::isfinite(val));
  }
  const Vec6 f = fm->torques(q, {{0.02, -0.01, 0.03, 0.0, 0.01, -0.02}});
  for (double val : f) {
    EXPECT_TRUE(std::isfinite(val));
  }
}

TEST(PiperXDriverCppTest, InverseKinematicsAndFeedbackAgeSkewHandling) {
  const Vec6 q_true{{0.25, 0.95, -1.35, 0.2, 0.45, -0.4}};
  const Vec3 tool_off{{0.0, 0.0, 0.16}};
  const FkResult fk = fast_fk(q_true, tool_off);
  const Vec6 q_seed{{0.15, 0.80, -1.15, 0.05, 0.30, -0.20}};
  const IkResult ik = inverse_kinematics(fk.centers[6], fk.rots[5], q_seed, tool_off);
  EXPECT_TRUE(ik.converged);
  EXPECT_LT(ik.position_error_m, 1e-4);
  EXPECT_LT(ik.orientation_error_rad, 1e-3);
  const FkResult fk_sol = fast_fk(ik.joints, tool_off);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(fk_sol.centers[6][i], fk.centers[6][i], 1e-4);
  }

  // Joint-limit clamping and calibrated joint angle_offset consistency
  const auto & chain = load_arm_chain_model();
  ArmGravityModel gm;
  gm.joints[2] = JointGravityModel{1.0, 0.03, 0.0, 0.0};
  gm.joints[3] = JointGravityModel{1.0, -0.02, 0.0, 0.0};
  Vec6 q_eff_true = q_true;
  q_eff_true[1] += 0.03;
  q_eff_true[2] -= 0.02;
  const FkResult fk_cal = fast_fk(q_eff_true, tool_off);
  const IkResult ik_cal = gm.inverse_kinematics(fk_cal.centers[6], fk_cal.rots[5], q_seed, tool_off);
  EXPECT_TRUE(ik_cal.converged);
  const FkResult fk_cal_check = fast_fk(
    Vec6{{ik_cal.joints[0], ik_cal.joints[1] + 0.03, ik_cal.joints[2] - 0.02,
          ik_cal.joints[3], ik_cal.joints[4], ik_cal.joints[5]}},
    tool_off);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(fk_cal_check.centers[6][i], fk_cal.centers[6][i], 1e-4);
  }
  const IkResult ik_unreachable = inverse_kinematics(
    Vec3{{2.0, 0.0, 2.0}}, fk.rots[5], q_seed, tool_off);
  EXPECT_FALSE(ik_unreachable.converged);
  for (int i = 0; i < 6; ++i) {
    EXPECT_GE(ik_unreachable.joints[i], chain.joint_lower[i] - 1e-12);
    EXPECT_LE(ik_unreachable.joints[i], chain.joint_upper[i] + 1e-12);
  }

  // Microsecond SO_TIMESTAMPNS vs CLOCK_REALTIME skew clamps cleanly to 0 without failing fresh()
  EXPECT_DOUBLE_EQ(normalize_feedback_age(100.0, -1.0), -1.0);
  EXPECT_DOUBLE_EQ(normalize_feedback_age(100.0, 100.000005), 0.0);
  EXPECT_DOUBLE_EQ(normalize_feedback_age(100.0, 100.000091), 0.0);
  EXPECT_LT(normalize_feedback_age(100.0, 100.200), 0.0);
  std::array<double, 11> ages{};
  ages.fill(normalize_feedback_age(100.0, 100.000005));
  EXPECT_TRUE(fresh(ages));
  EXPECT_TRUE(feedback_problem(ages).empty());
}

TEST(PiperXDriverCppTest, ProtocolMoveBatchesPreserveModeAndFrameOrder) {
  const Vec6 position{{0.25, 0.5, -0.5, 0.0, 0.0, 0.0}};
  const auto joint_batch = encode_joint_move_batch(position, 20, 0xAD);
  EXPECT_EQ(joint_batch[0].can_id, 0x151u);
  EXPECT_EQ(joint_batch[0].data[1], 255u);
  EXPECT_EQ(joint_batch[0].data[2], 20u);
  EXPECT_EQ(joint_batch[0].data[3], 0xADu);
  EXPECT_EQ(joint_batch[1].can_id, 0x151u);
  EXPECT_EQ(joint_batch[1].data[1], 1u);
  EXPECT_EQ(joint_batch[1].data[3], 0x00u);
  EXPECT_EQ(joint_batch[2].can_id, 0x155u);
  EXPECT_EQ(joint_batch[3].can_id, 0x156u);
  EXPECT_EQ(joint_batch[4].can_id, 0x157u);
  EXPECT_EQ(encode_joint_move_batch(position, 20)[0].data[3], 0x00u);

  const Vec6 pose{{0.28, -0.05, 0.24, 0.0, 1.2, 0.0}};
  const auto cartesian_batch = encode_cartesian_move_batch(pose, 15, true, 0xAD);
  EXPECT_EQ(cartesian_batch[0].can_id, 0x151u);
  EXPECT_EQ(cartesian_batch[0].data[1], 255u);
  EXPECT_EQ(cartesian_batch[0].data[2], 15u);
  EXPECT_EQ(cartesian_batch[0].data[3], 0xADu);
  EXPECT_EQ(cartesian_batch[1].can_id, 0x151u);
  EXPECT_EQ(cartesian_batch[1].data[1], 2u);
  EXPECT_EQ(cartesian_batch[1].data[3], 0x00u);
  EXPECT_EQ(cartesian_batch[2].can_id, 0x152u);
  EXPECT_EQ(cartesian_batch[3].can_id, 0x153u);
  EXPECT_EQ(cartesian_batch[4].can_id, 0x154u);

  const auto mit_frames = encode_mit_command(MitTarget{});
  EXPECT_EQ(mit_frames[0].can_id, 0x15Au);
  EXPECT_EQ(mit_frames[5].can_id, 0x15Fu);
}

TEST(PiperXDriverCppTest, DriverNodeServicesCommandsAndShutdownToDrag) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto mock = std::make_unique<MockArm>("can0");
  MockArm * raw_mock = mock.get();

  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "-r", "__ns:=/arm_1"});
  auto driver = std::make_shared<DriverNode>(opts, std::move(mock), true);
  auto client_node = std::make_shared<rclcpp::Node>("cpp_driver_test_client");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(driver);
  exec.add_node(client_node);

  std::string latest_ctrl;
  auto sub = client_node->create_subscription<std_msgs::msg::String>(
    "/arm_1/control_state", 10,
    [&](const std_msgs::msg::String::SharedPtr msg) { latest_ctrl = msg->data; });

  auto mode_cli = client_node->create_client<piper_x_interfaces::srv::SetMode>("/arm_1/set_mode");
  auto power_cli = client_node->create_client<piper_x_interfaces::srv::Enable>("/arm_1/enable_srv");
  auto j_pub = client_node->create_publisher<piper_x_interfaces::msg::JointCommand>(
    "/arm_1/command/joints", 10);
  auto c_pub = client_node->create_publisher<piper_x_interfaces::msg::CartesianCommand>(
    "/arm_1/command/cartesian", 10);

  auto spin_until = [&](auto pred, double timeout_s = 2.0) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (!pred() && std::chrono::steady_clock::now() < deadline) {
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return pred();
  };

  ASSERT_TRUE(spin_until([&] {
    return mode_cli->service_is_ready() && power_cli->service_is_ready() && !latest_ctrl.empty();
  }));
  EXPECT_EQ(driver->controller().mode, "mit");
  EXPECT_TRUE(raw_mock->is_powered());

  auto mode_req = std::make_shared<piper_x_interfaces::srv::SetMode::Request>();
  mode_req->mode = "joint";
  auto mode_fut = mode_cli->async_send_request(mode_req).future.share();
  ASSERT_TRUE(spin_until([&] {
    return mode_fut.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }));
  EXPECT_TRUE(mode_fut.get()->success);
  // Before any JointCommand target is sent, firmware still reports ctrl_mode=0, mode_feedback=6
  EXPECT_EQ(raw_mock->status_.ctrl_mode, 0);
  EXPECT_EQ(raw_mock->status_.mode_feedback, 6);

  piper_x_interfaces::msg::JointCommand jcmd;
  jcmd.position = {{0.25, 0.5, -0.5, 0.0, 0.0, 0.0}};
  jcmd.speed_percent = 20;
  j_pub->publish(jcmd);
  ASSERT_TRUE(spin_until([&] { return std::abs(raw_mock->pos_[0] - 0.25) < 1e-9; }));
  EXPECT_EQ(raw_mock->status_.ctrl_mode, 1);
  EXPECT_EQ(raw_mock->status_.mode_feedback, 1);

  // Switch to point / linear mode: firmware drops back to ctrl_mode=0 until CartesianCommand arrives
  mode_req->mode = "point";
  auto p_mode_fut = mode_cli->async_send_request(mode_req).future.share();
  ASSERT_TRUE(spin_until([&] {
    return p_mode_fut.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }));
  ASSERT_TRUE(p_mode_fut.get()->success);
  EXPECT_EQ(raw_mock->status_.ctrl_mode, 0);

  piper_x_interfaces::msg::CartesianCommand ccmd;
  ccmd.pose = {{0.28, -0.05, 0.24, 0.0, 1.2, 0.0}};
  ccmd.speed_percent = 15;
  c_pub->publish(ccmd);
  ASSERT_TRUE(spin_until([&] { return std::abs(raw_mock->pose_[0] - 0.28) < 1e-9; }));
  EXPECT_EQ(raw_mock->status_.ctrl_mode, 1);
  EXPECT_EQ(raw_mock->status_.mode_feedback, 0);

  auto p_req = std::make_shared<piper_x_interfaces::srv::Enable::Request>();
  p_req->enable_request = false;
  auto p_fut = power_cli->async_send_request(p_req).future.share();
  ASSERT_TRUE(spin_until([&] {
    return p_fut.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }));
  EXPECT_TRUE(p_fut.get()->enable_response);
  EXPECT_FALSE(raw_mock->is_powered());

  p_req->enable_request = true;
  auto p_on = power_cli->async_send_request(p_req).future.share();
  ASSERT_TRUE(spin_until([&] {
    return p_on.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }));
  EXPECT_TRUE(p_on.get()->enable_response);

  exec.remove_node(driver);
  exec.remove_node(client_node);
  driver->shutdown_driver();
  ASSERT_FALSE(raw_mock->calls.empty());
  EXPECT_EQ(raw_mock->calls.back(), "mode:drag");
}

TEST(PiperXDriverCppTest, ControllerAdaptiveStiffnessStictionGripperAndWrenchLaws) {
  MockArm arm("can0");
  double now = 0.0;
  JointFrictionModel j1_fric;
  j1_fric.fc = 0.2;
  j1_fric.fs = 0.4;
  j1_fric.load_ratio = 0.15;
  j1_fric.error_epsilon = 0.05;
  j1_fric.velocity_epsilon = 0.02;
  ArmFrictionModel f_model;
  f_model.arm_id = "arm_4";
  f_model.joints[1] = j1_fric;
  f_model.joints[2] = j1_fric;

  GripperFrictionModel gf;
  gf.fc = 0.4;
  gf.fs = 0.7;
  gf.vs = 0.015;
  gf.fv = 2.0;
  gf.v0 = 0.002;
  gf.vr = 0.08;
  gf.kp_equiv = 100.0;
  gf.w_grid = {0.005, 0.035, 0.065};
  gf.bias_grid = {-0.20, -0.35, -0.55};
  gf.modulation_grid = {0.15, -0.10, 0.20};
  f_model.gripper = gf;

  ArmGravityModel g_model;
  g_model.arm_id = "arm_4";
  g_model.joints[1] = JointGravityModel{1.0, 0.0, 0.0, 0.4};
  g_model.joints[2] = JointGravityModel{1.0, 0.0, 0.0, 0.5};

  const Vec6 base_kp{{17.0, 16.0, 15.0, 4.0, 3.0, 3.0}};
  const Vec6 base_kd{{6.0, 6.0, 3.0, 0.6, 0.4, 0.1}};
  Controller ctrl(
    arm, [&] { return now; }, std::nullopt, 1.0, g_model, 1.0, f_model,
    0.05, 0.02, 5.0, 2.0, 0.05, 2.0, 2.0, base_kp, base_kd);

  ctrl.start_mit();
  EXPECT_DOUBLE_EQ(ctrl.applied_kp[0], base_kp[0]);

  MitTarget cmd;
  for (int i = 0; i < 6; ++i) cmd.position[i] = arm.pos_[i];
  cmd.kp = base_kp;
  cmd.kd = base_kd;

  now = 1.0;
  ctrl.mit(cmd);
  ctrl.watchdog();
  EXPECT_NEAR(ctrl.applied_kp[0], 2.0 * base_kp[0], 1e-9);

  now = 2.5;
  ctrl.mit(cmd);
  ctrl.watchdog();
  EXPECT_NEAR(ctrl.applied_kp[0], 3.0 * base_kp[0], 1e-9);
  EXPECT_NEAR(ctrl.applied_kd[0], std::sqrt(3.0) * base_kd[0], 1e-9);

  // Negative kd auto-derives critical damping
  MitTarget auto_cmd = cmd;
  auto_cmd.kp = {{60.0, 60.0, 60.0, 12.0, 8.0, 8.0}};
  auto_cmd.kd = {{-1.0, -1.0, -1.0, -1.0, -1.0, -1.0}};
  ctrl.mit(auto_cmd);
  ASSERT_TRUE(ctrl.mit_target.has_value());
  EXPECT_DOUBLE_EQ(ctrl.mit_target->kd[0], 9.25);

  // Out-of-range MIT fields clamp cleanly without rejecting execution; release_mit drops target to gravity fallback
  MitTarget over_cmd = cmd;
  over_cmd.position[0] = 20.0;
  over_cmd.kp[0] = 600.0;
  ctrl.mit(over_cmd);
  EXPECT_DOUBLE_EQ(ctrl.mit_target->position[0], 12.5);
  EXPECT_DOUBLE_EQ(ctrl.mit_target->kp[0], 500.0);
  ctrl.release_mit();
  EXPECT_FALSE(ctrl.mit_target.has_value());

  // Simulate arm_3/arm_4 firmware reporting ctrl_mode=0 while in MIT mode:
  // watchdog(!ready) must still send full gravity/friction-compensated MIT frames (never zero offset/torque)
  arm.mit_history.clear();
  arm.status_.ctrl_mode = 0;
  arm.status_.mode_feedback = 6;
  now = 4.0;
  ctrl.watchdog();
  ASSERT_FALSE(arm.mit_history.empty());
  const MitTarget & recovery_mit = arm.mit_history.back();
  const Vec6 expected_g = g_model.torques({{arm.pos_[0], arm.pos_[1], arm.pos_[2], arm.pos_[3], arm.pos_[4], arm.pos_[5]}});
  EXPECT_GT(std::abs(expected_g[1]), 0.4);
  EXPECT_NEAR(
    recovery_mit.gravity_offset[1] * ctrl.applied_kp[1] + recovery_mit.torque[1],
    expected_g[1],
    1e-9);

  const Vec6 estimated_wrench = ctrl.end_wrench;
  Vec6 held_wrench;
  held_wrench.fill(123.0);
  ctrl.end_wrench = held_wrench;
  for (const int control_mode : {1, 0}) {
    arm.status_.ctrl_mode = control_mode;
    now += 0.005;
    ctrl.watchdog(false);
    EXPECT_EQ(ctrl.end_wrench, held_wrench);
  }
  ctrl.watchdog();
  EXPECT_EQ(ctrl.end_wrench, estimated_wrench);

  MitTarget zero_gain_command = cmd;
  zero_gain_command.kp.fill(0.0);
  zero_gain_command.kd.fill(0.0);
  ctrl.mit(zero_gain_command);
  ctrl.watchdog(false);
  ASSERT_TRUE(arm.mit_out_.has_value());
  EXPECT_EQ(arm.mit_out_->gravity_offset, Vec6{});
  EXPECT_EQ(arm.mit_out_->position, zero_gain_command.position);
  for (size_t joint = 0; joint < 6; ++joint) {
    EXPECT_DOUBLE_EQ(arm.mit_out_->torque[joint], ctrl.gravity[joint] + ctrl.friction[joint]);
  }

  ControllerReconfigureArgs invalid_updates;
  invalid_updates.s_a = 0.0;
  invalid_updates.adaptive_stiffness_b = 0.0;
  EXPECT_THROW(ctrl.reconfigure(invalid_updates), std::invalid_argument);
  EXPECT_DOUBLE_EQ(ctrl.adaptive_stiffness_scale_val, 2.0);
  EXPECT_DOUBLE_EQ(ctrl.adaptive_stiffness_scale_applied, 2.0);
}

TEST(PiperXDriverCppTest, EffectorCalibrationSafetyGatesAndConfigureProtocol) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  const auto tmp_yaml = std::filesystem::temp_directory_path() / "piper_cpp_test_arms.yaml";
  std::filesystem::copy_file(
    default_config_path(), tmp_yaml, std::filesystem::copy_options::overwrite_existing);

  auto mock = std::make_unique<MockArm>("can1");
  MockArm * raw_mock = mock.get();

  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "-r", "__ns:=/arm_2"});
  opts.append_parameter_override("config_file", tmp_yaml.string());
  auto driver = std::make_shared<DriverNode>(opts, std::move(mock), true);
  auto client_node = std::make_shared<rclcpp::Node>("cpp_calib_client");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(driver);
  exec.add_node(client_node);

  auto cal_cli = client_node->create_client<piper_x_interfaces::srv::CalibrateEffector>(
    "/arm_2/calibrate_effector");
  auto power_cli = client_node->create_client<piper_x_interfaces::srv::Enable>(
    "/arm_2/enable_srv");
  auto cfg_cli = client_node->create_client<piper_x_interfaces::srv::ConfigureArm>(
    "/arm_2/configure_arm");

  auto spin_until = [&](auto pred, double timeout_s = 2.0) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (!pred() && std::chrono::steady_clock::now() < deadline) {
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return pred();
  };
  ASSERT_TRUE(spin_until([&] {
    return cal_cli->service_is_ready() && power_cli->service_is_ready() && cfg_cli->service_is_ready();
  }));

  // Temporary calibration override friction_scale=-2.0 zeroes friction & s_a without touching YAML
  auto cfg_req = std::make_shared<piper_x_interfaces::srv::ConfigureArm::Request>();
  cfg_req->end_effector = driver->arm_config().end_effector;
  cfg_req->gravity_scale = 1.0;
  cfg_req->friction_scale = -2.0;
  auto cfg_fut = cfg_cli->async_send_request(cfg_req).future.share();
  ASSERT_TRUE(spin_until([&] {
    return cfg_fut.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }));
  EXPECT_TRUE(cfg_fut.get()->success);
  EXPECT_DOUBLE_EQ(driver->controller().friction_scale, 0.0);
  EXPECT_DOUBLE_EQ(driver->controller().adaptive_stiffness_scale_val, 0.0);

  // Reject effector calibration while enabled/MIT
  auto cal_req = std::make_shared<piper_x_interfaces::srv::CalibrateEffector::Request>();
  cal_req->confirm_closed = true;
  auto cal_bad = cal_cli->async_send_request(cal_req).future.share();
  ASSERT_TRUE(spin_until([&] {
    return cal_bad.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }));
  EXPECT_FALSE(cal_bad.get()->success);

  // Disable -> standby -> calibrate succeeds and leaves axes disabled
  auto p_off = std::make_shared<piper_x_interfaces::srv::Enable::Request>();
  p_off->enable_request = false;
  auto p_fut = power_cli->async_send_request(p_off).future.share();
  ASSERT_TRUE(spin_until([&] {
    return p_fut.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }));
  ASSERT_TRUE(p_fut.get()->enable_response);
  raw_mock->pos_[6] = -0.0108;
  raw_mock->status_.mode_feedback = 1;

  auto cal_ok = cal_cli->async_send_request(cal_req).future.share();
  ASSERT_TRUE(spin_until([&] {
    return cal_ok.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }));
  EXPECT_TRUE(cal_ok.get()->success);
  EXPECT_DOUBLE_EQ(raw_mock->pos_[6], 0.0);
  EXPECT_FALSE(raw_mock->is_powered());

  exec.remove_node(driver);
  exec.remove_node(client_node);
  driver->shutdown_driver();
  std::filesystem::remove(tmp_yaml);
}

}  // namespace
}  // namespace piper_x_driver

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  const int ret = RUN_ALL_TESTS();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return ret;
}
