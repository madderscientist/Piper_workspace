#include "paxini_driver/node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "paxini_driver/calibration.hpp"
#include "paxini_driver/configuration.hpp"

namespace paxini_driver {
namespace {

constexpr std::array<const char *, 6> kJointNames = {
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
};

std::string trim(const std::string & s) {
  size_t b = 0;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) {
    ++b;
  }
  size_t e = s.size();
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) {
    --e;
  }
  return s.substr(b, e - b);
}

std::string trim_lower(const std::string & s) {
  std::string out = trim(s);
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return out;
}

std::string trim_upper(const std::string & s) {
  std::string out = trim(s);
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return out;
}

bool all_zero_6(const std::array<double, 6> & a) noexcept {
  return a[0] == 0.0 && a[1] == 0.0 && a[2] == 0.0 &&
         a[3] == 0.0 && a[4] == 0.0 && a[5] == 0.0;
}

bool all_zero_3(const std::array<double, 3> & a) noexcept {
  return a[0] == 0.0 && a[1] == 0.0 && a[2] == 0.0;
}

std::string format_round(double v, int decimals) {
  const double scale = std::pow(10.0, decimals);
  double rounded = std::round(v * scale) / scale;
  if (rounded == 0.0) {
    rounded = 0.0;
  }
  std::ostringstream oss;
  oss << std::setprecision(15) << rounded;
  std::string s = oss.str();
  if (s.find('.') == std::string::npos && s.find('e') == std::string::npos &&
      s.find('E') == std::string::npos)
  {
    s += ".0";
  }
  return s;
}

}  // namespace

double PaxiniDriverNode::steady_now() noexcept {
  using Clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

PaxiniDriverNode::PaxiniDriverNode(
  const rclcpp::NodeOptions & options,
  std::unique_ptr<SensorTransport> transport,
  bool start_worker)
: rclcpp::Node("paxini_driver", options),
  filter_("none", 5, 0.2),
  worker_enabled_(start_worker)
{
  arm_id_ = trim(declare_parameter<std::string>("arm_id", ""));
  const std::string cfg_param = trim(declare_parameter<std::string>("config_file", ""));
  if (!cfg_param.empty()) {
    config_file_ = std::filesystem::path(cfg_param);
  }
  if (arm_id_.empty()) {
    // 未显式指定 arm_id 时，支持从 /arm_3 或 /arm_4 命名空间自动推断
    std::string ns = get_namespace();
    while (!ns.empty() && ns.front() == '/') {
      ns.erase(ns.begin());
    }
    while (!ns.empty() && ns.back() == '/') {
      ns.pop_back();
    }
    if (ns == "arm_3" || ns == "arm_4") {
      arm_id_ = ns;
    }
  }

  interface_ = trim_lower(declare_parameter<std::string>("interface", "usb"));
  const std::string serial_port = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  expected_version_ = trim(declare_parameter<std::string>("expected_version", ""));
  const int baudrate = static_cast<int>(declare_parameter<int64_t>("baudrate", DEFAULT_BAUDRATE));
  const std::string can_channel = declare_parameter<std::string>("can_channel", "can0");
  device_id_ = static_cast<int>(declare_parameter<int64_t>("device_id", DEFAULT_DEVICE_ID));
  model_ = trim_upper(declare_parameter<std::string>("model", "PX6D-75A"));
  frame_id_ = declare_parameter<std::string>("frame_id", "wrench_sensor");
  const std::string wrench_topic =
    declare_parameter<std::string>("wrench_topic", "wrench_stamped");
  const std::string raw_wrench_topic =
    declare_parameter<std::string>("raw_wrench_topic", "wrench_raw_stamped");
  const std::string status_topic =
    declare_parameter<std::string>("status_topic", "sensor_status");
  const std::string acq =
    trim_lower(declare_parameter<std::string>("acquisition_mode", "default"));
  frequency_hz_.store(static_cast<int>(declare_parameter<int64_t>("frequency_hz", 1000)));
  poll_rate_hz_ = declare_parameter<double>("poll_rate_hz", 1000.0);
  autostart_ = declare_parameter<bool>("autostart", true);
  calibrate_on_start_ = declare_parameter<bool>("calibrate_on_start", false);
  reconnect_sec_ = declare_parameter<double>("reconnect_sec", 1.0);
  stream_timeout_sec_ = declare_parameter<double>("stream_timeout_sec", 1.0);
  require_hardware_ = declare_parameter<bool>("require_hardware", false);

  const std::string filter_mode = declare_parameter<std::string>("filter_mode", "none");
  const int filter_window = static_cast<int>(declare_parameter<int64_t>("filter_window", 5));
  const double filter_alpha = declare_parameter<double>("filter_alpha", 0.2);

  const auto tare_vec =
    declare_parameter<std::vector<double>>("tare_offset", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  if (tare_vec.size() == 6) {
    std::copy(tare_vec.begin(), tare_vec.end(), tare_offset_.begin());
  }
  tool_mass_kg_ = declare_parameter<double>("tool_mass_kg", 0.0);
  const auto com_vec =
    declare_parameter<std::vector<double>>("tool_com_m", {0.0, 0.0, 0.0});
  if (com_vec.size() == 3) {
    std::copy(com_vec.begin(), com_vec.end(), tool_com_m_.begin());
  }

  if (!arm_id_.empty()) {
    try {
      // 若启动参数未覆盖标定值，则从 px6d.yaml 读取对应臂的零偏、工具质量/质心及拟合质量摘要
      const auto cfg = resolve_sensor_calibration(arm_id_, config_file_);
      calibration_quality_json_ = cfg.calibration_quality_json;
      if (all_zero_6(tare_offset_) && cfg.has_tare_offset) {
        tare_offset_ = cfg.tare_offset;
      }
      if (tool_mass_kg_ == 0.0 && cfg.has_tool_mass) {
        tool_mass_kg_ = cfg.tool_mass_kg;
      }
      if (all_zero_3(tool_com_m_) && cfg.has_tool_com) {
        tool_com_m_ = cfg.tool_com_m;
      }
    } catch (const std::exception &) {
    }
  }

  if (acq == "default") {
    // RS485 半双工默认采用 0x05 主从轮询（poll），USB / CAN / CANFD 默认采用 0x03 自动连续回传（auto）
    acquisition_mode_ = (interface_ == "rs485") ? "poll" : "auto";
  } else if (acq == "auto" || acq == "poll" || acq == "manual") {
    acquisition_mode_ = acq;
  } else {
    throw std::invalid_argument(
      "Unsupported acquisition_mode '" + acq + "'; expected default, auto, poll, or manual");
  }

  filter_ = WrenchFilter(filter_mode, filter_window, filter_alpha);
  if (transport) {
    transport_ = std::move(transport);
  } else {
    transport_ = create_transport(
      interface_,
      serial_port,
      baudrate,
      can_channel,
      device_id_);
  }

  const auto wrench_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
  events_buf_.reserve(16);
  wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(wrench_topic, wrench_qos);
  raw_wrench_pub_ =
    create_publisher<geometry_msgs::msg::WrenchStamped>(raw_wrench_topic, wrench_qos);
  status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, 10);

  cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_;

  cmd_sub_ = create_subscription<std_msgs::msg::String>(
    "command", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) { on_command_msg(*msg); },
    sub_opts);

  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states_feedback", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joints(*msg); },
    sub_opts);

  auto register_trigger = [this](
    const char * name,
    std::pair<bool, std::string> (PaxiniDriverNode::*handler)())
  {
    services_.push_back(create_service<std_srvs::srv::Trigger>(
      name,
      [this, handler](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
      {
        auto [ok, msg] = (this->*handler)();
        resp->success = ok;
        resp->message = std::move(msg);
      },
      rmw_qos_profile_services_default,
      cb_group_));
  };

  register_trigger("calibrate", &PaxiniDriverNode::calibrate_hardware);
  register_trigger("start_stream", &PaxiniDriverNode::start_stream);
  register_trigger("stop_stream", &PaxiniDriverNode::stop_stream);
  register_trigger("read_once", &PaxiniDriverNode::request_single_frame);
  register_trigger("get_version", &PaxiniDriverNode::request_version);
  register_trigger("reload_config", &PaxiniDriverNode::reload_config);

  {
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    try_connect_locked(steady_now(), true, false);
  }

  status_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() { publish_status(); },
    cb_group_);

  if (worker_enabled_) {
    worker_thread_ = std::thread(&PaxiniDriverNode::io_loop, this);
  }
}

PaxiniDriverNode::~PaxiniDriverNode() {
  shutdown_driver();
}

void PaxiniDriverNode::shutdown_driver() noexcept {
  if (shutdown_done_.exchange(true)) {
    return;
  }
  // 先通知并等待后台采集线程退出，再安全下发 0x04 停流指令并关闭底层设备描述符
  {
    std::lock_guard<std::mutex> lk(stop_mutex_);
    stop_requested_.store(true);
  }
  stop_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
  std::lock_guard<std::mutex> io_lock(io_mutex_);
  if (transport_ && transport_->is_open()) {
    bool was_streaming = false;
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      was_streaming = streaming_;
      streaming_ = false;
    }
    if (was_streaming) {
      try {
        transport_->send_command(CMD_STOP_AUTO);
      } catch (...) {
      }
    }
    transport_->close();
  }
}

void PaxiniDriverNode::on_joints(const sensor_msgs::msg::JointState & msg) {
  if (msg.position.size() < 6) {
    return;
  }
  // 快路径匹配 piper_x_driver 固定顺序 joint1..joint6；若顺序不同则按名称检索映射
  std::array<size_t, 6> order{0, 1, 2, 3, 4, 5};
  bool fast_match = msg.name.size() >= 6;
  if (fast_match) {
    for (size_t i = 0; i < 6; ++i) {
      if (msg.name[i] != kJointNames[i]) {
        fast_match = false;
        break;
      }
    }
  }
  if (!fast_match) {
    for (size_t j = 0; j < 6; ++j) {
      auto it = std::find(msg.name.begin(), msg.name.end(), kJointNames[j]);
      if (it == msg.name.end()) {
        return;
      }
      const auto idx = static_cast<size_t>(std::distance(msg.name.begin(), it));
      if (idx >= msg.position.size()) {
        return;
      }
      order[j] = idx;
    }
  }

  std::array<double, 6> pos6{};
  for (size_t j = 0; j < 6; ++j) {
    const double v = msg.position[order[j]];
    if (!std::isfinite(v)) {
      return;
    }
    pos6[j] = v;
  }
  // 在 200 Hz 关节反馈回调中预计算单位重力方向，避免 1 kHz 力采集线程重复做正运动学三角函数运算
  const auto g_dir = gravity_unit_in_sensor(pos6);
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_joint_positions_ = pos6;
  latest_gravity_dir_ = g_dir;
}

void PaxiniDriverNode::warn_communication_once(const std::string & message) {
  // 同一轮通信故障期间仅打印一次告警，恢复收到有效六维力样本后自动复位
  bool report = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    report = !communication_warning_active_;
    communication_warning_active_ = true;
  }
  if (report) {
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
  }
}

void PaxiniDriverNode::check_stream_timeout(double now_sec) {
  std::string message;
  bool report = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const double last = std::max(last_wrench_time_, stream_started_time_);
    report = connected_ && streaming_ && stream_timeout_sec_ > 0.0 &&
             (now_sec - last) >= stream_timeout_sec_ && !stream_gap_reported_;
    if (report) {
      stream_gap_reported_ = true;
      char buf[128];
      std::snprintf(
        buf, sizeof(buf), "PX6D 数据断流: %.2f 秒未收到有效六维力", now_sec - last);
      last_error_ = buf;
      message = last_error_;
    }
  }
  if (report) {
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
  }
}

void PaxiniDriverNode::handle_disconnect_locked(const std::string & reason) {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    connected_ = false;
    streaming_ = false;
    last_error_ = reason;
  }
  warn_communication_once("PX6D 通信中断: " + reason);
  if (transport_) {
    transport_->close();
  }
}

void PaxiniDriverNode::handle_disconnect(const std::string & reason) {
  std::lock_guard<std::mutex> io_lock(io_mutex_);
  handle_disconnect_locked(reason);
}

void PaxiniDriverNode::reset_reconnect_timer() noexcept {
  std::lock_guard<std::mutex> io_lock(io_mutex_);
  last_connect_attempt_ = 0.0;
}

void PaxiniDriverNode::send_stream_start_locked() {
  const int freq = frequency_hz_.load();
  if (acquisition_mode_ == "auto") {
    transport_->send_command(CMD_START_AUTO);
    if (freq > 0 && freq != 1000) {
      transport_->send_command(CMD_SET_FREQ, freq);
    }
  } else if (acquisition_mode_ == "poll" && freq > 0) {
    transport_->send_command(CMD_SET_FREQ, freq);
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  streaming_ = true;
  stream_started_time_ = steady_now();
}

bool PaxiniDriverNode::try_connect_locked(double now_sec, bool initial, bool force) {
  if (transport_->is_open()) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    connected_ = true;
    return true;
  }
  if (!initial && !force) {
    if (reconnect_sec_ <= 0.0 || (now_sec - last_connect_attempt_) < reconnect_sec_) {
      return false;
    }
  }
  last_connect_attempt_ = now_sec;

  std::set<std::string> tried_ports;
  while (true) {
    std::string open_err;
    const int max_attempts = tried_ports.empty() ? 1 : 10;
    for (int attempt = 0; attempt < max_attempts && !stop_requested_.load(); ++attempt) {
      try {
        transport_->open();
        open_err.clear();
        break;
      } catch (const std::exception & exc) {
        open_err = exc.what();
        if (attempt + 1 < max_attempts) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      }
    }
    if (!transport_->is_open()) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        connected_ = false;
        streaming_ = false;
        last_error_ = open_err;
      }
      if (tried_ports.empty()) {
        if (initial && require_hardware_) {
          throw std::runtime_error(open_err);
        }
        warn_communication_once("PX6D 通信不可用: " + open_err);
      } else {
        last_connect_attempt_ = 0.0;
      }
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      connected_ = true;
      firmware_version_.clear();
    }

    try {
      // 连接后先查固件版本（0x07）；若真机双串口顺序互换，在此循环内直接切换端口重连，无递归调用
      transport_->send_command(CMD_GET_VERSION);
      if (!transport_->is_real_serial()) {
        break;
      }
      const double deadline = steady_now() + 0.02;
      while (steady_now() < deadline) {
        transport_->poll_events_into(0.002, events_buf_);
        const double rx_now = steady_now();
        for (const auto & ev : events_buf_) {
          process_event(ev, rx_now);
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!firmware_version_.empty()) {
          break;
        }
      }

      std::string ver;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ver = firmware_version_;
      }
      if (expected_version_.empty() || ver.empty() || ver == expected_version_ ||
          !transport_->has_serial_port())
      {
        break;
      }

      const std::string cur_port = transport_->port();
      tried_ports.insert(cur_port);
      std::vector<std::string> candidates;
      std::error_code ec;
      for (const auto & entry : std::filesystem::directory_iterator("/dev", ec)) {
        const std::string fname = entry.path().filename().string();
        const std::string full = entry.path().string();
        if (fname.rfind("ttyUSB", 0) == 0 && tried_ports.find(full) == tried_ports.end()) {
          candidates.push_back(full);
        }
      }
      std::sort(candidates.begin(), candidates.end());
      if (candidates.empty()) {
        break;
      }
      RCLCPP_WARN(
        get_logger(), "Port %s reported firmware %s (expected %s); switching to %s",
        cur_port.c_str(), ver.c_str(), expected_version_.c_str(), candidates.front().c_str());
      transport_->close();
      transport_->set_port(candidates.front());
      std::lock_guard<std::mutex> lock(state_mutex_);
      connected_ = false;
    } catch (const std::exception & exc) {
      handle_disconnect_locked(exc.what());
      return false;
    }
  }

  try {
    if (calibrate_on_start_) {
      transport_->send_command(CMD_CALIBRATE);
    }
    const int freq = frequency_hz_.load();
    if (autostart_) {
      send_stream_start_locked();
    } else if (freq > 0) {
      transport_->send_command(CMD_SET_FREQ, freq);
    }
    last_poll_sent_ = steady_now();
    polls_without_rx_ = 0;
  } catch (const std::exception & exc) {
    handle_disconnect_locked(exc.what());
    return false;
  }
  return true;
}

int PaxiniDriverNode::step_locked(double timeout_sec) {
  const double now = steady_now();
  if (!transport_->is_open() && !try_connect_locked(now)) {
    return 0;
  }
  try {
    bool is_streaming = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      is_streaming = streaming_;
    }
    if (is_streaming && acquisition_mode_ == "poll" && poll_rate_hz_ > 0.0) {
      // 按固定节拍发送 0x05 单帧请求，并将本次 ppoll 超时截断至下一拍时刻，兼顾及时收包与精准定时
      const double period = 1.0 / poll_rate_hz_;
      if ((now - last_poll_sent_) >= period) {
        transport_->send_command(CMD_READ_ONCE);
        ++polls_without_rx_;
        if (last_poll_sent_ <= 0.0 || (now - last_poll_sent_) > 2.0 * period) {
          last_poll_sent_ = now;
        } else if (interface_ == "rs485" && polls_without_rx_ >= 3) {
          // RS485 半双工轮询若恰好与传感器回包窗口同相碰撞，微移 0.35 个周期错开收发相位
          last_poll_sent_ = now - 0.65 * period;
          polls_without_rx_ = 0;
        } else {
          last_poll_sent_ += period;
        }
      }
      timeout_sec = std::min(timeout_sec, std::max(0.0, last_poll_sent_ + period - steady_now()));
    }
    transport_->poll_events_into(timeout_sec, events_buf_);
  } catch (const std::exception & exc) {
    handle_disconnect_locked(exc.what());
    return 0;
  }

  if (events_buf_.empty()) {
    return 0;
  }
  const double rx_now = steady_now();
  const int n = static_cast<int>(events_buf_.size());
  for (const auto & ev : events_buf_) {
    if (ev.wrench.has_value()) {
      polls_without_rx_ = 0;
    }
    process_event(ev, rx_now);
  }
  return n;
}

int PaxiniDriverNode::step(double timeout_sec) {
  std::lock_guard<std::mutex> io_lock(io_mutex_);
  return step_locked(timeout_sec);
}

void PaxiniDriverNode::io_loop() {
  while (!stop_requested_.load(std::memory_order_relaxed)) {
    bool open_now = false;
    {
      std::lock_guard<std::mutex> io_lock(io_mutex_);
      open_now = transport_->is_open();
      if (!open_now) {
        open_now = try_connect_locked(steady_now());
      }
      if (open_now) {
        step_locked(0.002);
      }
    }
    if (!open_now) {
      std::unique_lock<std::mutex> lk(stop_mutex_);
      stop_cv_.wait_for(
        lk, std::chrono::milliseconds(50), [this]() { return stop_requested_.load(); });
    }
  }
}

void PaxiniDriverNode::process_event(const ProtocolEvent & ev, double now_sec) {
  std::optional<WrenchSample> wrench_to_pub;
  std::optional<WrenchSample> raw_to_pub;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (ev.version.has_value() && !ev.version->empty()) {
      firmware_version_ = *ev.version;
    }

    if (ev.wrench.has_value() && ev.wrench->is_finite()) {
      const WrenchSample & raw = *ev.wrench;
      latest_raw_ = raw;
      last_wrench_time_ = now_sec;
      communication_warning_active_ = false;
      stream_gap_reported_ = false;
      last_error_.clear();
      raw_to_pub = raw;
      const WrenchSample compensated = compensate_wrench_with_gravity_dir(
        raw, tare_offset_, latest_gravity_dir_, tool_mass_kg_, tool_com_m_);
      const WrenchSample filtered = filter_.apply(compensated);
      latest_wrench_ = filtered;
      overload_warning_ = raw.exceeds_overload_alarm(model_);
      ++sample_count_;
      rate_tracker_.mark(now_sec);
      wrench_to_pub = filtered;
    }
  }

  if (raw_to_pub.has_value() &&
      (raw_wrench_pub_->get_subscription_count() +
       raw_wrench_pub_->get_intra_process_subscription_count()) > 0)
  {
    // 仅当存在订阅者（如独立标定客户端 calibrate）时才序列化并发布原始未补偿六维力
    publish_wrench(*raw_to_pub, true);
  }
  if (wrench_to_pub.has_value()) {
    publish_wrench(*wrench_to_pub, false);
  }
}

void PaxiniDriverNode::publish_wrench(const WrenchSample & sample, bool raw) {
  geometry_msgs::msg::WrenchStamped msg;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = frame_id_;
  msg.wrench.force.x = sample.fx;
  msg.wrench.force.y = sample.fy;
  msg.wrench.force.z = sample.fz;
  msg.wrench.torque.x = sample.mx;
  msg.wrench.torque.y = sample.my;
  msg.wrench.torque.z = sample.mz;
  if (raw) {
    raw_wrench_pub_->publish(msg);
  } else {
    wrench_pub_->publish(msg);
  }
}

std::string PaxiniDriverNode::status_json() {
  const double now = steady_now();
  const bool transport_open = transport_ && transport_->is_open();
  std::lock_guard<std::mutex> lock(state_mutex_);

  std::ostringstream oss;
  oss << "{";
  oss << "\"connected\": " << ((connected_ && transport_open) ? "true" : "false") << ", ";
  oss << "\"interface\": \"" << json_escape(interface_) << "\", ";
  oss << "\"device_id\": " << device_id_ << ", ";
  oss << "\"model\": \"" << json_escape(model_) << "\", ";
  oss << "\"frame_id\": \"" << json_escape(frame_id_) << "\", ";
  oss << "\"acquisition_mode\": \"" << json_escape(acquisition_mode_) << "\", ";
  oss << "\"frequency_hz\": " << frequency_hz_.load() << ", ";
  oss << "\"poll_rate_hz\": " << format_round(poll_rate_hz_, 2) << ", ";
  oss << "\"streaming\": " << (streaming_ ? "true" : "false") << ", ";
  oss << "\"firmware_version\": \"" << json_escape(firmware_version_) << "\", ";
  oss << "\"rx_hz\": " << format_round(rate_tracker_.hz(now), 2) << ", ";
  oss << "\"sample_count\": " << sample_count_ << ", ";
  oss << "\"overload_warning\": " << (overload_warning_ ? "true" : "false") << ", ";
  oss << "\"filter_mode\": \"" << json_escape(filter_.mode()) << "\", ";
  oss << "\"tare_offset\": [";
  for (size_t i = 0; i < 6; ++i) {
    if (i > 0) oss << ", ";
    oss << format_round(tare_offset_[i], 6);
  }
  oss << "], ";
  oss << "\"tool_mass_kg\": " << format_round(tool_mass_kg_, 6) << ", ";
  oss << "\"tool_com_m\": [";
  for (size_t i = 0; i < 3; ++i) {
    if (i > 0) oss << ", ";
    oss << format_round(tool_com_m_[i], 6);
  }
  oss << "], ";
  oss << "\"calibration_quality\": " << calibration_quality_json_ << ", ";
  oss << "\"latest_wrench\": ";
  if (latest_wrench_.has_value()) {
    const auto arr = latest_wrench_->as_array();
    oss << "[";
    for (size_t i = 0; i < 6; ++i) {
      if (i > 0) oss << ", ";
      oss << format_round(arr[i], 6);
    }
    oss << "]";
  } else {
    oss << "null";
  }
  oss << ", ";
  oss << "\"last_error\": \"" << json_escape(last_error_) << "\", ";
  oss << "\"stream_gap\": " << (stream_gap_reported_ ? "true" : "false");
  oss << "}";
  return oss.str();
}

void PaxiniDriverNode::publish_status() {
  check_stream_timeout(steady_now());
  std_msgs::msg::String msg;
  msg.data = status_json();
  status_pub_->publish(msg);
}

std::pair<bool, std::string> PaxiniDriverNode::exec_cmd(
  uint8_t cmd,
  std::optional<int> arg,
  const std::string & desc)
{
  if (!transport_->is_open()) {
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    if (!transport_->is_open() && !try_connect_locked(steady_now(), false, true)) {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      return {
        false,
        last_error_.empty() ? std::string("Sensor hardware is not connected") : last_error_
      };
    }
  }
  try {
    transport_->send_command(cmd, arg);
    return {true, desc};
  } catch (const std::invalid_argument & exc) {
    return {false, exc.what()};
  } catch (const std::exception & exc) {
    handle_disconnect(exc.what());
    return {false, exc.what()};
  }
}

std::pair<bool, std::string> PaxiniDriverNode::calibrate_hardware() {
  auto [ok, msg] = exec_cmd(CMD_CALIBRATE, std::nullopt, "Hardware calibration (0x10) sent");
  if (ok) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    filter_.reset();
  }
  return {ok, std::move(msg)};
}

std::pair<bool, std::string> PaxiniDriverNode::reload_config() {
  try {
    // 热重载仅原子替换内存中的零偏与工具重力参数并清空滤波器历史，不向硬件发送任何指令，也不重放旧样本
    const auto entry = resolve_sensor_calibration(arm_id_, config_file_);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      tare_offset_ = entry.tare_offset;
      tool_mass_kg_ = entry.tool_mass_kg;
      tool_com_m_ = entry.tool_com_m;
      calibration_quality_json_ = entry.calibration_quality_json;
      filter_.reset();
      latest_wrench_.reset();
    }
    return {true, arm_id_ + " 已重载标定参数"};
  } catch (const std::exception & exc) {
    return {false, exc.what()};
  }
}

std::pair<bool, std::string> PaxiniDriverNode::start_stream() {
  std::lock_guard<std::mutex> io_lock(io_mutex_);
  if (!transport_->is_open() && !try_connect_locked(steady_now(), false, true)) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return {
      false,
      last_error_.empty() ? std::string("Sensor hardware is not connected") : last_error_
    };
  }
  if (acquisition_mode_ == "poll") {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      streaming_ = true;
      stream_started_time_ = steady_now();
    }
    char buf[160];
    std::snprintf(
      buf, sizeof(buf),
      "Polling requested at %.1f Hz (0x05); actual reception rate is sensor_status.rx_hz",
      poll_rate_hz_);
    return {true, buf};
  }
  try {
    send_stream_start_locked();
    const int freq = frequency_hz_.load();
    const std::string rate_info =
      (freq > 0 && freq != 1000) ? (std::to_string(freq) + " Hz (0x03+0x02)") : "1000 Hz (0x03)";
    return {
      true,
      "Automatic transmission requested at " + rate_info +
        "; actual reception rate is sensor_status.rx_hz"
    };
  } catch (const std::exception & exc) {
    handle_disconnect_locked(exc.what());
    return {false, exc.what()};
  }
}

std::pair<bool, std::string> PaxiniDriverNode::stop_stream() {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    streaming_ = false;
  }
  if (!transport_->is_open()) {
    return {true, "Stream stopped (hardware offline)"};
  }
  return exec_cmd(CMD_STOP_AUTO, std::nullopt, "Automatic transmission stopped (0x04)");
}

std::pair<bool, std::string> PaxiniDriverNode::request_single_frame() {
  auto [ok, msg] = exec_cmd(CMD_READ_ONCE, std::nullopt, "Single-frame request (0x05) sent");
  if (ok && !worker_enabled_) {
    step(0.01);
  }
  return {ok, std::move(msg)};
}

std::pair<bool, std::string> PaxiniDriverNode::request_version() {
  auto [ok, msg] = exec_cmd(CMD_GET_VERSION, std::nullopt, "Version query (0x07) sent");
  if (ok && !worker_enabled_) {
    step(0.01);
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  return {ok, (ok && !firmware_version_.empty()) ? firmware_version_ : msg};
}

void PaxiniDriverNode::on_command_msg(const std_msgs::msg::String & msg) {
  const std::string raw = trim(msg.data);
  const std::string cmd = trim_lower(raw);

  bool ok = false;
  std::string info;
  if (cmd == "calibrate") {
    std::tie(ok, info) = calibrate_hardware();
  } else if (cmd == "start") {
    std::tie(ok, info) = start_stream();
  } else if (cmd == "stop") {
    std::tie(ok, info) = stop_stream();
  } else if (cmd == "once" || cmd == "read_once") {
    std::tie(ok, info) = request_single_frame();
  } else if (cmd == "version") {
    std::tie(ok, info) = request_version();
  } else if (cmd == "reload_config") {
    std::tie(ok, info) = reload_config();
  } else if (cmd.rfind("set_freq:", 0) == 0) {
    try {
      const int hz = std::stoi(cmd.substr(9), nullptr, 0);
      std::tie(ok, info) = exec_cmd(
        CMD_SET_FREQ, hz, "Frequency set to " + std::to_string(hz) + " Hz (0x02)");
      if (ok) {
        frequency_hz_.store(hz);
      }
    } catch (const std::exception & exc) {
      ok = false;
      info = exc.what();
    }
  } else if (cmd.rfind("set_id:", 0) == 0) {
    try {
      const int new_id = std::stoi(cmd.substr(7), nullptr, 0);
      char desc[48];
      std::snprintf(desc, sizeof(desc), "Set ID 0x%02X (0x01)", new_id & 0xFF);
      std::tie(ok, info) = exec_cmd(CMD_SET_ID, new_id, desc);
    } catch (const std::exception & exc) {
      ok = false;
      info = exc.what();
    }
  } else if (cmd == "set_mode:can" || cmd == "set_mode:canfd") {
    const int canfd = (cmd == "set_mode:canfd") ? 1 : 0;
    std::tie(ok, info) = exec_cmd(CMD_SET_PORT_PARAMS, canfd, "Set protocol (" + cmd + ")");
  } else {
    RCLCPP_WARN(get_logger(), "Unknown command '%s'", raw.c_str());
    return;
  }
  if (!ok) {
    RCLCPP_WARN(get_logger(), "Command '%s' failed: %s", raw.c_str(), info.c_str());
  }
}

std::optional<std::array<double, 6>> PaxiniDriverNode::latest_joint_positions() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_joint_positions_;
}

std::optional<WrenchSample> PaxiniDriverNode::latest_raw() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_raw_;
}

std::optional<WrenchSample> PaxiniDriverNode::latest_wrench() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_wrench_;
}

std::array<double, 6> PaxiniDriverNode::tare_offset() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return tare_offset_;
}

double PaxiniDriverNode::tool_mass_kg() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return tool_mass_kg_;
}

std::array<double, 3> PaxiniDriverNode::tool_com_m() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return tool_com_m_;
}

}  // namespace paxini_driver
