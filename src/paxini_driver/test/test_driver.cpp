#include <gtest/gtest.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>

#include "paxini_driver/calibration.hpp"
#include "paxini_driver/configuration.hpp"
#include "paxini_driver/node.hpp"
#include "paxini_driver/protocol.hpp"
#include "paxini_driver/transport.hpp"

namespace paxini_driver {
namespace {

std::vector<uint8_t> from_hex(const std::string & hex) {
  std::istringstream iss(hex);
  std::vector<uint8_t> out;
  std::string tok;
  while (iss >> tok) {
    out.push_back(static_cast<uint8_t>(std::stoul(tok, nullptr, 16)));
  }
  return out;
}

class FakeTransport final : public SensorTransport {
public:
  struct SharedState {
    bool available{false};
    bool open{false};
    std::vector<std::pair<uint8_t, std::optional<int>>> commands;
    std::vector<ProtocolEvent> events;
  };

  explicit FakeTransport(std::shared_ptr<SharedState> state)
  : state_(std::move(state)) {}

  const std::string & interface_name() const noexcept override { return iface_; }
  bool is_open() const noexcept override { return state_->open; }

  void open() override {
    if (!state_->available) {
      throw std::runtime_error("No such device /dev/ttyUSB0");
    }
    state_->open = true;
  }

  void close() noexcept override { state_->open = false; }

  std::vector<ProtocolEvent> poll_events(double /*timeout_sec*/) override {
    auto out = std::move(state_->events);
    state_->events.clear();
    return out;
  }

  void send_command(uint8_t cmd, std::optional<int> arg) override {
    state_->commands.emplace_back(cmd, arg);
  }

private:
  std::string iface_{"usb"};
  std::shared_ptr<SharedState> state_;
};

}  // namespace

TEST(PaxiniDriverTest, SerialProtocolMatchesManualExamples) {
  EXPECT_EQ(build_serial_frame(CMD_SET_ID, 0x01), from_hex("AA 55 7F 01 01 AE"));
  EXPECT_EQ(build_serial_frame(CMD_SET_FREQ, freq_to_code(4)), from_hex("AA 55 7F 02 01 91"));
  EXPECT_EQ(build_serial_frame(CMD_SET_FREQ, freq_to_code(100)), from_hex("AA 55 7F 02 19 D9"));
  EXPECT_EQ(build_serial_frame(CMD_START_AUTO), from_hex("AA 55 7F 03 04 9F"));
  EXPECT_EQ(build_serial_frame(CMD_STOP_AUTO), from_hex("AA 55 7F 04 00 E8"));
  EXPECT_EQ(build_serial_frame(CMD_READ_ONCE), from_hex("AA 55 7F 05 01 FA"));
  EXPECT_EQ(build_serial_frame(CMD_GET_VERSION), from_hex("AA 55 7F 07 01 D0"));
  EXPECT_EQ(build_serial_frame(CMD_CALIBRATE), from_hex("AA 55 7F 10 01 EC"));

  SerialFrameParser parser(0x7F);
  std::vector<uint8_t> stream = {0x00, 0xAA, 0x55, 0x7F, 0x03, 0xFF};
  const auto ver = from_hex("AA 55 7F 07 00 76 30 2E 31 2E 31 00 5B");
  const auto wrench = from_hex(
    "AA 55 7F 03 31 0C 0B BD 5C 76 38 BD FC 35 A2 3C "
    "6E E8 B7 BA E6 9C 24 39 36 C8 9F B9 43");
  const auto cal = from_hex("AA 55 7F 10 00 00 00 00 00 00 00 00 E2");
  stream.insert(stream.end(), ver.begin(), ver.end());
  stream.insert(stream.end(), wrench.begin(), wrench.end());
  stream.insert(stream.end(), cal.begin(), cal.end());

  auto events = parser.feed(stream.data(), 15);
  const auto tail = parser.feed(stream.data() + 15, stream.size() - 15);
  events.insert(events.end(), tail.begin(), tail.end());

  ASSERT_EQ(events.size(), 3U);
  EXPECT_EQ(events[0].version, std::optional<std::string>("v0.1.1"));
  ASSERT_TRUE(events[1].wrench.has_value());
  EXPECT_NEAR(events[1].wrench->fx, -0.03394717, 1e-6);
  EXPECT_NEAR(events[1].wrench->fy, -0.04503475, 1e-6);
  EXPECT_NEAR(events[1].wrench->fz, 0.01980113, 1e-6);
  EXPECT_EQ(events[2].ack_cmd, std::optional<uint8_t>(CMD_CALIBRATE));
}

TEST(PaxiniDriverTest, SerialTransportWaitReceivesDelayedFrameAndRs485Prefix) {
  int rx_pipe[2]{-1, -1};
  int tx_pipe[2]{-1, -1};
  ASSERT_EQ(::pipe(rx_pipe), 0);
  ASSERT_EQ(::pipe(tx_pipe), 0);

  const auto frame = from_hex(
    "AA 55 7F 03 31 0C 0B BD 5C 76 38 BD FC 35 A2 3C "
    "6E E8 B7 BA E6 9C 24 39 36 C8 9F B9 43");

  SerialSensorTransport rx_transport(
    "pipe_rx", DEFAULT_BAUDRATE, DEFAULT_DEVICE_ID, "usb",
    [&](const std::string &, int) { return rx_pipe[0]; });
  rx_transport.open();

  std::thread writer([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_EQ(
      ::write(rx_pipe[1], frame.data(), frame.size()),
      static_cast<ssize_t>(frame.size()));
  });
  const auto events = rx_transport.poll_events(0.25);
  writer.join();
  rx_transport.close();
  ::close(rx_pipe[1]);

  ASSERT_EQ(events.size(), 1U);
  ASSERT_TRUE(events[0].wrench.has_value());
  const auto expected = WrenchSample::from_bytes(frame.data() + 4).as_array();
  EXPECT_EQ(events[0].wrench->as_array(), expected);

  SerialSensorTransport rs485_tx(
    "pipe_tx", DEFAULT_BAUDRATE, DEFAULT_DEVICE_ID, "rs485",
    [&](const std::string &, int) { return tx_pipe[1]; });
  rs485_tx.open();
  rs485_tx.send_command(CMD_READ_ONCE);
  uint8_t tx_buf[16]{};
  const ssize_t n = ::read(tx_pipe[0], tx_buf, sizeof(tx_buf));
  rs485_tx.close();
  ::close(tx_pipe[0]);
  ASSERT_EQ(n, 7);
  EXPECT_EQ(std::vector<uint8_t>(tx_buf, tx_buf + n), from_hex("FF AA 55 7F 05 01 FA"));
}

TEST(PaxiniDriverTest, CanAndCanfdProtocolMatchesManual) {
  auto can_vec = [](uint8_t cmd, std::optional<int> arg = std::nullopt) {
    auto [id, arr] = build_can_command(cmd, arg);
    return std::make_pair(id, std::vector<uint8_t>(arr.begin(), arr.end()));
  };
  EXPECT_EQ(
    can_vec(CMD_SET_ID, 0x01),
    std::make_pair(0x27FU, from_hex("01 01 00 00 00 00 00 00")));
  EXPECT_EQ(
    can_vec(CMD_SET_FREQ, freq_to_code(4)),
    std::make_pair(0x27FU, from_hex("02 01 00 00 00 00 00 00")));
  EXPECT_EQ(
    can_vec(CMD_START_AUTO),
    std::make_pair(0x27FU, from_hex("03 00 00 00 00 00 00 00")));
  EXPECT_EQ(
    can_vec(CMD_STOP_AUTO),
    std::make_pair(0x27FU, from_hex("04 00 00 00 00 00 00 00")));
  EXPECT_EQ(
    can_vec(CMD_READ_ONCE),
    std::make_pair(0x27FU, from_hex("05 00 00 00 00 00 00 00")));
  EXPECT_EQ(
    can_vec(CMD_GET_VERSION),
    std::make_pair(0x27FU, from_hex("07 00 00 00 00 00 00 00")));
  EXPECT_EQ(
    can_vec(CMD_CALIBRATE),
    std::make_pair(0x27FU, from_hex("10 01 00 00 00 00 00 00")));

  const auto [fd_id, fd_cmd] = build_can_set_protocol(true);
  EXPECT_EQ(fd_id, 0x27FU);
  EXPECT_EQ(
    std::vector<uint8_t>(fd_cmd.begin(), fd_cmd.end()),
    from_hex("11 01 00 01 03 01 00 00"));

  CanFrameParser parser(0x7F);
  const auto ver_ev = parser.push(0x1FFU, from_hex("00 07 76 30 2E 31 2E 31"));
  ASSERT_TRUE(ver_ev.has_value());
  EXPECT_EQ(ver_ev->version, std::optional<std::string>("v0.1.1"));

  const WrenchSample sample{1.5, -2.25, 12.0, 0.15, -0.25, 0.05};
  const auto raw24 = sample.to_bytes();
  const auto fd_ev = parser.push(0x2FFU, raw24.data(), raw24.size());
  ASSERT_TRUE(fd_ev.has_value() && fd_ev->wrench.has_value());
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(fd_ev->wrench->as_array()[i], sample.as_array()[i], 1e-6);
  }

  EXPECT_FALSE(parser.push(0x2FFU, raw24.data() + 0, 8).has_value());
  EXPECT_FALSE(parser.push(0x3FFU, raw24.data() + 8, 8).has_value());
  const auto can_ev = parser.push(0x4FFU, raw24.data() + 16, 8);
  ASSERT_TRUE(can_ev.has_value() && can_ev->wrench.has_value());
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(can_ev->wrench->as_array()[i], sample.as_array()[i], 1e-6);
  }
}

TEST(PaxiniDriverTest, SensorRotationMatchesIndependentSdkPose) {
  const std::array<double, 6> q = {
    1.1168012817661317,
    0.5668305811701985,
    -0.9810670258385326,
    1.1209028055083183,
    0.2017251549455046,
    0.17170549181120215,
  };
  const double x = -0.8273785772239776;
  const double y = 0.31607928379407524;
  const double z = -0.15271415557520154;
  const double w = 0.438425550115415;
  const std::array<std::array<double, 3>, 3> sdk_rot = {{
    {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)},
    {2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)},
    {2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)},
  }};
  const auto rot = sensor_rotation_in_base(q);
  for (size_t r = 0; r < 3; ++r) {
    for (size_t c = 0; c < 3; ++c) {
      EXPECT_NEAR(rot[r][c], sdk_rot[r][c], 5e-5);
    }
  }
}

TEST(PaxiniDriverTest, RosDriverRawCompensationReloadAndShutdown) {
  const auto tmp_dir =
    std::filesystem::temp_directory_path() /
    ("paxini_cpp_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(tmp_dir);
  const auto cfg_path = tmp_dir / "px6d.yaml";
  {
    std::ofstream ofs(cfg_path);
    ofs << "sensors:\n  arm_3:\n    namespace: /paxini_cpp_test/arm_3\n";
  }

  const auto src_cfg = std::filesystem::path(PAXINI_SOURCE_DIR) / "config" / "px6d.yaml";
  EXPECT_EQ(resolve_sensor_calibration("右手", src_cfg).arm_id, "arm_4");

  rclcpp::init(0, nullptr);
  auto state = std::make_shared<FakeTransport::SharedState>();
  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "-r", "__ns:=/paxini_cpp_test/arm_3"});
  opts.parameter_overrides({
    rclcpp::Parameter("arm_id", "arm_3"),
    rclcpp::Parameter("config_file", cfg_path.string()),
    rclcpp::Parameter("reconnect_sec", 0.01),
    rclcpp::Parameter("frequency_hz", 200),
  });

  auto node = std::make_shared<PaxiniDriverNode>(
    opts, std::make_unique<FakeTransport>(state), false);
  auto probe = std::make_shared<rclcpp::Node>("probe", "/paxini_cpp_test/arm_3");

  std::vector<geometry_msgs::msg::WrenchStamped> received;
  std::vector<geometry_msgs::msg::WrenchStamped> raw_received;
  std::vector<rcl_interfaces::msg::Log> ros_logs;

  auto sub_w = probe->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "wrench_stamped", 10,
    [&](const geometry_msgs::msg::WrenchStamped::SharedPtr m) { received.push_back(*m); });
  auto sub_raw = probe->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "wrench_raw_stamped", 10,
    [&](const geometry_msgs::msg::WrenchStamped::SharedPtr m) { raw_received.push_back(*m); });
  auto sub_log = probe->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout", 50,
    [&](const rcl_interfaces::msg::Log::SharedPtr m) {
      if (m->name.find("paxini_driver") != std::string::npos) {
        ros_logs.push_back(*m);
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(probe);

  sensor_msgs::msg::JointState malformed_joints;
  malformed_joints.name = {"gripper", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  malformed_joints.position.assign(6, 0.0);
  node->on_joints(malformed_joints);
  EXPECT_FALSE(node->latest_joint_positions().has_value());

  const YAML::Node init_status = YAML::Load(node->status_json());
  EXPECT_EQ(init_status["frequency_hz"].as<int>(), 200);
  EXPECT_DOUBLE_EQ(init_status["poll_rate_hz"].as<double>(), 1000.0);
  EXPECT_FALSE(init_status["connected"].as<bool>());
  EXPECT_FALSE(node->calibrate_hardware().first);

  state->available = true;
  node->reset_reconnect_timer();
  ProtocolEvent ver_ev;
  ver_ev.cmd = CMD_GET_VERSION;
  ver_ev.version = "v1.0.3";
  ProtocolEvent wrench_ev;
  wrench_ev.cmd = CMD_START_AUTO;
  wrench_ev.wrench = WrenchSample{10.0, -5.0, 20.0, 0.1, -0.2, 0.3};
  state->events = {ver_ev, wrench_ev};

  EXPECT_EQ(node->step(), 2);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while ((received.empty() || raw_received.empty()) &&
         std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_FALSE(raw_received.empty());
  ASSERT_FALSE(received.empty());
  EXPECT_DOUBLE_EQ(raw_received.back().wrench.force.z, 20.0);
  EXPECT_DOUBLE_EQ(received.back().wrench.force.z, 20.0);
  ASSERT_GE(state->commands.size(), 3U);
  EXPECT_EQ(state->commands[0], std::make_pair(CMD_GET_VERSION, std::optional<int>()));
  EXPECT_EQ(state->commands[1], std::make_pair(CMD_START_AUTO, std::optional<int>()));
  EXPECT_EQ(state->commands[2], std::make_pair(CMD_SET_FREQ, std::optional<int>(200)));

  {
    std::ofstream ofs(cfg_path);
    ofs << "sensors:\n  arm_3:\n    namespace: /paxini_cpp_test/arm_3\n"
        << "    tare_offset: [10.0, -5.0, 20.0, 0.1, -0.2, 0.3]\n";
  }
  const auto before_commands = state->commands;
  EXPECT_TRUE(node->reload_config().first);
  EXPECT_EQ(state->commands, before_commands);

  ProtocolEvent next_ev;
  next_ev.cmd = CMD_START_AUTO;
  next_ev.wrench = WrenchSample{12.0, -5.0, 25.0, 0.1, -0.2, 0.3};
  state->events = {next_ev};
  node->step();
  ASSERT_TRUE(node->latest_raw().has_value() && node->latest_wrench().has_value());
  EXPECT_NEAR(node->latest_wrench()->fx, 2.0, 1e-6);
  EXPECT_NEAR(node->latest_wrench()->fz, 5.0, 1e-6);

  const double t0 = PaxiniDriverNode::steady_now();
  node->process_event(next_ev, t0);
  ros_logs.clear();
  node->check_stream_timeout(t0 + 0.5);
  node->check_stream_timeout(t0 + 1.2);
  node->check_stream_timeout(t0 + 2.0);
  EXPECT_TRUE(YAML::Load(node->status_json())["stream_gap"].as<bool>());
  node->process_event(next_ev, t0 + 2.1);
  EXPECT_FALSE(YAML::Load(node->status_json())["stream_gap"].as<bool>());

  const auto log_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  while (ros_logs.empty() && std::chrono::steady_clock::now() < log_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  size_t warn_count = 0;
  size_t info_count = 0;
  for (const auto & lg : ros_logs) {
    if (lg.level == rcl_interfaces::msg::Log::WARN) ++warn_count;
    if (lg.level == rcl_interfaces::msg::Log::INFO) ++info_count;
  }
  EXPECT_EQ(info_count, 0U);
  EXPECT_EQ(warn_count, 1U);

  executor.remove_node(probe);
  executor.remove_node(node);
  node->shutdown_driver();
  probe.reset();
  node.reset();
  rclcpp::shutdown();
  std::filesystem::remove_all(tmp_dir);

  ASSERT_FALSE(state->commands.empty());
  EXPECT_EQ(state->commands.back(), std::make_pair(CMD_STOP_AUTO, std::optional<int>()));
}

}  // namespace paxini_driver
