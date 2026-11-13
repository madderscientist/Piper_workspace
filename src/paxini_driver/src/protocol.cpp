#include "paxini_driver/protocol.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace paxini_driver {
namespace {

constexpr std::array<uint8_t, 256> make_crc8_table() noexcept {
  std::array<uint8_t, 256> table{};
  for (size_t b = 0; b < 256; ++b) {
    uint8_t crc = static_cast<uint8_t>(b);
    for (int i = 0; i < 8; ++i) {
      crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1U) ^ 0x07U)
                          : static_cast<uint8_t>(crc << 1U);
    }
    table[b] = crc;
  }
  return table;
}

constexpr auto kCrc8Table = make_crc8_table();

std::string trim_lower(const std::string & s) {
  size_t b = 0;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) {
    ++b;
  }
  size_t e = s.size();
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) {
    --e;
  }
  std::string out = s.substr(b, e - b);
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return out;
}

std::string to_upper_ascii(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return s;
}

float read_f32_le(const uint8_t * p) noexcept {
  uint32_t u = static_cast<uint32_t>(p[0]) |
               (static_cast<uint32_t>(p[1]) << 8U) |
               (static_cast<uint32_t>(p[2]) << 16U) |
               (static_cast<uint32_t>(p[3]) << 24U);
  float v = 0.0f;
  std::memcpy(&v, &u, sizeof(float));
  return v;
}

void write_f32_le(float v, uint8_t * p) noexcept {
  uint32_t u = 0;
  std::memcpy(&u, &v, sizeof(float));
  p[0] = static_cast<uint8_t>(u & 0xFFU);
  p[1] = static_cast<uint8_t>((u >> 8U) & 0xFFU);
  p[2] = static_cast<uint8_t>((u >> 16U) & 0xFFU);
  p[3] = static_cast<uint8_t>((u >> 24U) & 0xFFU);
}

uint8_t default_serial_arg(uint8_t cmd) {
  switch (cmd) {
    case CMD_START_AUTO: return 0x04;
    case CMD_STOP_AUTO: return 0x00;
    case CMD_READ_ONCE: return 0x01;
    case CMD_GET_VERSION: return 0x01;
    case CMD_CALIBRATE: return 0x01;
    default:
      throw std::invalid_argument("Command requires an explicit argument");
  }
}

uint8_t default_can_arg(uint8_t cmd) {
  switch (cmd) {
    case CMD_START_AUTO: return 0x00;
    case CMD_STOP_AUTO: return 0x00;
    case CMD_READ_ONCE: return 0x00;
    case CMD_GET_VERSION: return 0x00;
    case CMD_CALIBRATE: return 0x01;
    default:
      throw std::invalid_argument("CAN command requires an explicit argument");
  }
}

// 按手册 5.1 / 5.2 节定义每条串口指令允许的应答负载字节长度（如 0x03 既可能是 8 字节应答，也可能是 24 字节六维力）
size_t candidate_payload_lengths(uint8_t cmd, std::array<size_t, 2> & out) noexcept {
  switch (cmd) {
    case CMD_SET_ID:
      out[0] = 8;
      return 1;
    case CMD_SET_FREQ:
      out[0] = 1;
      out[1] = 8;
      return 2;
    case CMD_START_AUTO:
      out[0] = 24;
      out[1] = 8;
      return 2;
    case CMD_STOP_AUTO:
      out[0] = 8;
      return 1;
    case CMD_READ_ONCE:
      out[0] = 24;
      return 1;
    case CMD_GET_VERSION:
      out[0] = 8;
      return 1;
    case CMD_CALIBRATE:
      out[0] = 8;
      return 1;
    default:
      return 0;
  }
}

}  // namespace

uint8_t crc8(const uint8_t * data, size_t len) noexcept {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc = kCrc8Table[crc ^ data[i]];
  }
  return crc;
}

uint8_t check_u8(int value, const char * name) {
  if (value < 0 || value > 0xFF) {
    throw std::invalid_argument(
      std::string(name) + " must fit in 1 byte (0..255), got " + std::to_string(value));
  }
  return static_cast<uint8_t>(value);
}

uint8_t freq_to_code(int freq_hz) {
  if (freq_hz < 4 || (freq_hz % 4) != 0) {
    throw std::invalid_argument(
      "Frequency " + std::to_string(freq_hz) + " Hz must be a positive multiple of 4 Hz");
  }
  return check_u8(freq_hz / 4, "freq_hz // 4");
}

std::string decode_version_bytes(const uint8_t * data, size_t len) {
  // 串口版本应答为 8 字节（首尾补 0x00），CAN 版本应答为后 6 字节，统一提取中间的 ASCII 版本串
  if (len == 8 && data[0] == 0 && data[7] == 0) {
    data += 1;
    len = 6;
  }
  size_t b = 0;
  while (b < len && (data[b] == 0x00 || data[b] == ' ')) {
    ++b;
  }
  size_t e = len;
  while (e > b && (data[e - 1] == 0x00 || data[e - 1] == ' ')) {
    --e;
  }
  return std::string(reinterpret_cast<const char *>(data + b), e - b);
}

bool WrenchSample::is_finite() const noexcept {
  return std::isfinite(fx) && std::isfinite(fy) && std::isfinite(fz) &&
         std::isfinite(mx) && std::isfinite(my) && std::isfinite(mz);
}

WrenchSample WrenchSample::minus(const std::array<double, 6> & offset) const noexcept {
  return WrenchSample{
    fx - offset[0],
    fy - offset[1],
    fz - offset[2],
    mx - offset[3],
    my - offset[4],
    mz - offset[5],
  };
}

bool WrenchSample::exceeds_overload_alarm(const std::string & model, double ratio) const {
  // 手册 1.2 与 4.2 节：任一轴超过额定满量程的 200%（ratio=2.0）时置位过载告警标志
  const std::string key = to_upper_ascii(model);
  double f_range = 0.0;
  double m_range = 0.0;
  if (key == "PX6D-75A") { f_range = 50.0; m_range = 2.0; }
  else if (key == "PX6D-75B") { f_range = 100.0; m_range = 4.0; }
  else if (key == "PX6D-75C") { f_range = 200.0; m_range = 8.0; }
  else if (key == "PX6D-75E") { f_range = 500.0; m_range = 20.0; }
  else { return false; }
  const double f_max = f_range * ratio;
  const double m_max = m_range * ratio;
  return std::abs(fx) > f_max || std::abs(fy) > f_max || std::abs(fz) > f_max ||
         std::abs(mx) > m_max || std::abs(my) > m_max || std::abs(mz) > m_max;
}

WrenchSample WrenchSample::from_bytes(const uint8_t * payload) noexcept {
  return WrenchSample{
    static_cast<double>(read_f32_le(payload + 0)),
    static_cast<double>(read_f32_le(payload + 4)),
    static_cast<double>(read_f32_le(payload + 8)),
    static_cast<double>(read_f32_le(payload + 12)),
    static_cast<double>(read_f32_le(payload + 16)),
    static_cast<double>(read_f32_le(payload + 20)),
  };
}

std::array<uint8_t, 24> WrenchSample::to_bytes() const noexcept {
  std::array<uint8_t, 24> out{};
  write_f32_le(static_cast<float>(fx), out.data() + 0);
  write_f32_le(static_cast<float>(fy), out.data() + 4);
  write_f32_le(static_cast<float>(fz), out.data() + 8);
  write_f32_le(static_cast<float>(mx), out.data() + 12);
  write_f32_le(static_cast<float>(my), out.data() + 16);
  write_f32_le(static_cast<float>(mz), out.data() + 20);
  return out;
}

SerialCommandFrame build_serial_command_frame(
  uint8_t cmd,
  std::optional<int> arg,
  int device_id,
  bool rs485_wakeup_prefix)
{
  const uint8_t dev = check_u8(device_id, "device_id");
  const uint8_t byte_arg = arg.has_value() ? check_u8(*arg, "data") : default_serial_arg(cmd);

  SerialCommandFrame out;
  uint8_t idx = 0;
  if (rs485_wakeup_prefix) {
    out.bytes[idx++] = 0xFF;
  }
  const uint8_t body_start = idx;
  out.bytes[idx++] = SERIAL_HEADER_0;
  out.bytes[idx++] = SERIAL_HEADER_1;
  out.bytes[idx++] = dev;
  out.bytes[idx++] = cmd;
  out.bytes[idx++] = byte_arg;
  out.bytes[idx] = crc8(out.bytes.data() + body_start, idx - body_start);
  out.size = static_cast<uint8_t>(idx + 1);
  return out;
}

std::vector<uint8_t> build_serial_frame(
  uint8_t cmd,
  std::optional<int> arg,
  int device_id)
{
  const auto frame = build_serial_command_frame(cmd, arg, device_id, false);
  return std::vector<uint8_t>(frame.bytes.begin(), frame.bytes.begin() + frame.size);
}

SerialFrameParser::SerialFrameParser(std::optional<int> device_id) {
  if (device_id.has_value()) {
    device_id_ = check_u8(*device_id, "device_id");
  }
  buf_.reserve(512);
}

void SerialFrameParser::reset() noexcept {
  buf_.clear();
  head_ = 0;
}

void SerialFrameParser::feed(
  const uint8_t * data,
  size_t len,
  std::vector<ProtocolEvent> & events)
{
  if (len > 0 && data != nullptr) {
    // 使用读偏移 head_ 避免每帧擦除搬移内存；仅在已消费前缀累积较大或全部读完时集中回收
    if (head_ > 0 && (head_ >= buf_.size() || head_ > 4096)) {
      buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(head_));
      head_ = 0;
    }
    buf_.insert(buf_.end(), data, data + len);
  }

  while (true) {
    const size_t avail = buf_.size() - head_;
    if (avail == 0) {
      buf_.clear();
      head_ = 0;
      break;
    }

    // 查找帧头 0xAA 0x55；若末尾恰好停在半个帧头 0xAA，保留该字节等待下一批数据拼接
    size_t header_pos = buf_.size();
    for (size_t i = head_; i + 1 < buf_.size(); ++i) {
      if (buf_[i] == SERIAL_HEADER_0 && buf_[i + 1] == SERIAL_HEADER_1) {
        header_pos = i;
        break;
      }
    }
    if (header_pos == buf_.size()) {
      if (buf_.back() == SERIAL_HEADER_0) {
        buf_[0] = SERIAL_HEADER_0;
        buf_.resize(1);
      } else {
        buf_.clear();
      }
      head_ = 0;
      break;
    }
    head_ = header_pos;
    const size_t rem = buf_.size() - head_;
    if (rem < 6) {
      break;
    }

    const uint8_t * frame = buf_.data() + head_;
    const uint8_t dev_id = frame[2];
    const uint8_t cmd = frame[3];
    if (device_id_.has_value() && dev_id != *device_id_) {
      head_ += 2;
      continue;
    }

    std::array<size_t, 2> candidates{};
    const size_t num_candidates = candidate_payload_lengths(cmd, candidates);
    if (num_candidates == 0) {
      head_ += 2;
      continue;
    }

    size_t matched_frame_len = 0;
    bool need_more_bytes = false;
    for (size_t i = 0; i < num_candidates; ++i) {
      const size_t frame_len = 5 + candidates[i];
      if (rem < frame_len) {
        need_more_bytes = true;
        continue;
      }
      if (crc8(frame, frame_len - 1) == frame[frame_len - 1]) {
        matched_frame_len = frame_len;
        break;
      }
    }

    if (matched_frame_len == 0) {
      // 存在候选帧长尚未收齐时继续等待后续字节；若所有候选长度均已收齐但 CRC 不匹配，说明是伪帧头，跳过这 2 字节重同步
      if (need_more_bytes) {
        break;
      }
      head_ += 2;
      continue;
    }

    const size_t payload_len = matched_frame_len - 5;
    const uint8_t * payload_ptr = frame + 4;
    ProtocolEvent ev;
    ev.cmd = cmd;
    ev.payload_len = static_cast<uint8_t>(std::min<size_t>(payload_len, ev.payload.size()));
    std::memcpy(ev.payload.data(), payload_ptr, ev.payload_len);
    if ((cmd == CMD_START_AUTO || cmd == CMD_READ_ONCE) && payload_len == 24) {
      ev.wrench = WrenchSample::from_bytes(payload_ptr);
    } else if (cmd == CMD_GET_VERSION && payload_len == 8) {
      ev.version = decode_version_bytes(payload_ptr, 8);
      ev.ack_cmd = cmd;
    } else {
      ev.ack_cmd = cmd;
    }
    events.push_back(std::move(ev));
    head_ += matched_frame_len;
  }

  if (head_ >= buf_.size()) {
    buf_.clear();
    head_ = 0;
  }
}

std::vector<ProtocolEvent> SerialFrameParser::feed(const uint8_t * data, size_t len) {
  std::vector<ProtocolEvent> events;
  feed(data, len, events);
  return events;
}

uint32_t can_tx_id(int device_id) {
  return 0x200U + static_cast<uint32_t>(check_u8(device_id, "device_id"));
}

uint32_t can_ack_id(int device_id) {
  return 0x180U + static_cast<uint32_t>(check_u8(device_id, "device_id"));
}

std::array<uint32_t, 3> can_data_ids(int device_id) {
  const uint32_t dev = check_u8(device_id, "device_id");
  return {0x280U + dev, 0x380U + dev, 0x480U + dev};
}

std::pair<uint32_t, std::array<uint8_t, 8>> build_can_command(
  uint8_t cmd,
  std::optional<int> arg,
  int device_id)
{
  const uint8_t byte_arg = arg.has_value() ? check_u8(*arg, "arg") : default_can_arg(cmd);
  return {can_tx_id(device_id), {cmd, byte_arg, 0, 0, 0, 0, 0, 0}};
}

std::pair<uint32_t, std::array<uint8_t, 8>> build_can_set_protocol(
  bool canfd,
  int device_id)
{
  const uint8_t mode = canfd ? 0x01 : 0x00;
  return {
    can_tx_id(device_id),
    {CMD_SET_PORT_PARAMS, mode, 0x00, 0x01, 0x03, mode, 0x00, 0x00},
  };
}

CanFrameParser::CanFrameParser(int device_id)
: device_id_(check_u8(device_id, "device_id")),
  ack_id_(can_ack_id(device_id_)),
  data_ids_(can_data_ids(device_id_))
{}

std::optional<ProtocolEvent> CanFrameParser::push(
  uint32_t can_id,
  const uint8_t * data,
  size_t len)
{
  // 0x180 + ID：指令应答帧（含状态码、回显命令字及固件版本号）
  if (can_id == ack_id_) {
    if (len < 2) {
      return std::nullopt;
    }
    ProtocolEvent ev;
    ev.status = data[0];
    ev.cmd = data[1];
    ev.ack_cmd = data[1];
    if (data[1] == CMD_GET_VERSION && len >= 8) {
      ev.version = decode_version_bytes(data + 2, 6);
    }
    ev.payload_len = static_cast<uint8_t>(std::min<size_t>(len, ev.payload.size()));
    std::memcpy(ev.payload.data(), data, ev.payload_len);
    return ev;
  }

  const auto [id1, id2, id3] = data_ids_;
  // CANFD 模式：0x280 + ID 单帧携带全部 24 字节（6 个小端 float32）
  if (can_id == id1 && len >= 24) {
    stage_ = 0;
    ProtocolEvent ev;
    ev.wrench = WrenchSample::from_bytes(data);
    ev.payload_len = 24;
    std::memcpy(ev.payload.data(), data, 24);
    return ev;
  }
  // CAN 2.0 模式：严格按 0x280+ID(Fx,Fy) -> 0x380+ID(Fz,Mx) -> 0x480+ID(My,Mz) 顺序拼齐三帧后输出
  if (can_id == id1 && len >= 8) {
    fx_ = static_cast<double>(read_f32_le(data));
    fy_ = static_cast<double>(read_f32_le(data + 4));
    stage_ = 1;
    return std::nullopt;
  }
  if (can_id == id2 && len >= 8) {
    if (stage_ != 1) {
      stage_ = 0;
      return std::nullopt;
    }
    fz_ = static_cast<double>(read_f32_le(data));
    mx_ = static_cast<double>(read_f32_le(data + 4));
    stage_ = 2;
    return std::nullopt;
  }
  if (can_id == id3 && len >= 8) {
    if (stage_ != 2) {
      stage_ = 0;
      return std::nullopt;
    }
    const double my = static_cast<double>(read_f32_le(data));
    const double mz = static_cast<double>(read_f32_le(data + 4));
    stage_ = 0;
    ProtocolEvent ev;
    ev.wrench = WrenchSample{fx_, fy_, fz_, mx_, my, mz};
    ev.payload_len = 8;
    std::memcpy(ev.payload.data(), data, 8);
    return ev;
  }
  return std::nullopt;
}

WrenchFilter::WrenchFilter(const std::string & mode, int window, double alpha)
: mode_(trim_lower(mode)),
  window_(window >= 1 ? static_cast<size_t>(window) : 0),
  alpha_(alpha)
{
  if (mode_ != "none" && mode_ != "moving_average" && mode_ != "low_pass") {
    throw std::invalid_argument(
      "Unsupported filter_mode '" + mode + "'; expected none, moving_average, or low_pass");
  }
  if (window < 1) {
    throw std::invalid_argument(
      "filter_window must be >= 1, got " + std::to_string(window));
  }
  if (!(alpha > 0.0 && alpha <= 1.0)) {
    throw std::invalid_argument(
      "filter_alpha must be in (0, 1], got " + std::to_string(alpha));
  }
}

void WrenchFilter::reset() noexcept {
  history_.clear();
  sums_.fill(0.0);
  state_initialized_ = false;
  state_.fill(0.0);
}

WrenchSample WrenchFilter::apply(const WrenchSample & sample) {
  if (mode_ == "none") {
    return sample;
  }
  const auto vals = sample.as_array();
  if (mode_ == "moving_average") {
    if (history_.size() == window_) {
      const auto oldest = history_.front();
      history_.pop_front();
      for (size_t i = 0; i < 6; ++i) {
        sums_[i] -= oldest[i];
      }
    }
    history_.push_back(vals);
    for (size_t i = 0; i < 6; ++i) {
      sums_[i] += vals[i];
    }
    const double inv_n = 1.0 / static_cast<double>(history_.size());
    return WrenchSample{
      sums_[0] * inv_n,
      sums_[1] * inv_n,
      sums_[2] * inv_n,
      sums_[3] * inv_n,
      sums_[4] * inv_n,
      sums_[5] * inv_n,
    };
  }
  if (!state_initialized_) {
    state_ = vals;
    state_initialized_ = true;
    return sample;
  }
  for (size_t i = 0; i < 6; ++i) {
    state_[i] = alpha_ * vals[i] + (1.0 - alpha_) * state_[i];
  }
  return WrenchSample{state_[0], state_[1], state_[2], state_[3], state_[4], state_[5]};
}

}  // namespace paxini_driver
