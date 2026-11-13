#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace paxini_driver {

// PX6D 串口协议帧头（0xAA 0x55）、默认广播/单机地址（0x7F）及默认波特率（921600 8N1）
inline constexpr uint8_t SERIAL_HEADER_0 = 0xAA;
inline constexpr uint8_t SERIAL_HEADER_1 = 0x55;
inline constexpr int DEFAULT_DEVICE_ID = 0x7F;
inline constexpr int DEFAULT_BAUDRATE = 921600;

// 官方手册第 5 章指令码定义
inline constexpr uint8_t CMD_SET_ID = 0x01;
inline constexpr uint8_t CMD_SET_FREQ = 0x02;
inline constexpr uint8_t CMD_START_AUTO = 0x03;
inline constexpr uint8_t CMD_STOP_AUTO = 0x04;
inline constexpr uint8_t CMD_READ_ONCE = 0x05;
inline constexpr uint8_t CMD_GET_VERSION = 0x07;
inline constexpr uint8_t CMD_CALIBRATE = 0x10;
inline constexpr uint8_t CMD_SET_PORT_PARAMS = 0x11;

// CRC-8 校验：poly=0x07, init=0x00, refin=false, refout=false, xorout=0x00
uint8_t crc8(const uint8_t * data, size_t len) noexcept;
uint8_t check_u8(int value, const char * name);
// 将采样频率（Hz，须为 4 的正整数倍）编码为协议字节（Freq / 4）
uint8_t freq_to_code(int freq_hz);
std::string decode_version_bytes(const uint8_t * data, size_t len);

// 传感器原始测量坐标系下的六维力/力矩样本（单位：N 与 N·m）
struct WrenchSample {
  double fx{0.0};
  double fy{0.0};
  double fz{0.0};
  double mx{0.0};
  double my{0.0};
  double mz{0.0};

  std::array<double, 6> as_array() const noexcept {
    return {fx, fy, fz, mx, my, mz};
  }

  bool is_finite() const noexcept;
  WrenchSample minus(const std::array<double, 6> & offset) const noexcept;
  bool exceeds_overload_alarm(const std::string & model = "PX6D-75A", double ratio = 2.0) const;

  static WrenchSample from_bytes(const uint8_t * payload_24) noexcept;
  std::array<uint8_t, 24> to_bytes() const noexcept;
};

// 串口或 CAN/CANFD 解析得到的统一协议事件（六维力数据、固件版本或指令应答）
struct ProtocolEvent {
  std::optional<uint8_t> cmd;
  std::optional<WrenchSample> wrench;
  std::optional<std::string> version;
  std::optional<uint8_t> ack_cmd;
  uint8_t status{0};
  std::array<uint8_t, 24> payload{};
  uint8_t payload_len{0};
};

// 定长栈上串口命令帧缓冲区，避免 1 kHz 轮询路径产生堆内存分配
struct SerialCommandFrame {
  std::array<uint8_t, 16> bytes{};
  uint8_t size{0};
};

SerialCommandFrame build_serial_command_frame(
  uint8_t cmd,
  std::optional<int> arg = std::nullopt,
  int device_id = DEFAULT_DEVICE_ID,
  bool rs485_wakeup_prefix = false);

std::vector<uint8_t> build_serial_frame(
  uint8_t cmd,
  std::optional<int> arg = std::nullopt,
  int device_id = DEFAULT_DEVICE_ID);

// RS485 / USB 字节流状态机解析器：支持粘包、拆包及伪帧头自动重同步
class SerialFrameParser {
public:
  explicit SerialFrameParser(std::optional<int> device_id = DEFAULT_DEVICE_ID);

  void reset() noexcept;
  void feed(const uint8_t * data, size_t len, std::vector<ProtocolEvent> & out);
  std::vector<ProtocolEvent> feed(const uint8_t * data, size_t len);
  std::vector<ProtocolEvent> feed(const std::vector<uint8_t> & chunk) {
    return feed(chunk.data(), chunk.size());
  }

private:
  std::optional<uint8_t> device_id_;
  std::vector<uint8_t> buf_;
  size_t head_{0};
};

uint32_t can_tx_id(int device_id = DEFAULT_DEVICE_ID);
uint32_t can_ack_id(int device_id = DEFAULT_DEVICE_ID);
std::array<uint32_t, 3> can_data_ids(int device_id = DEFAULT_DEVICE_ID);

std::pair<uint32_t, std::array<uint8_t, 8>> build_can_command(
  uint8_t cmd,
  std::optional<int> arg = std::nullopt,
  int device_id = DEFAULT_DEVICE_ID);

std::pair<uint32_t, std::array<uint8_t, 8>> build_can_set_protocol(
  bool canfd,
  int device_id = DEFAULT_DEVICE_ID);

// CANFD（0x280+ID 单帧 24 字节）与 CAN 2.0（0x280/0x380/0x480+ID 连续三帧 8 字节）解析器
class CanFrameParser {
public:
  explicit CanFrameParser(int device_id = DEFAULT_DEVICE_ID);

  void reset() noexcept { stage_ = 0; }
  std::optional<ProtocolEvent> push(uint32_t can_id, const uint8_t * data, size_t len);
  std::optional<ProtocolEvent> push(uint32_t can_id, const std::vector<uint8_t> & data) {
    return push(can_id, data.data(), data.size());
  }

private:
  uint8_t device_id_;
  uint32_t ack_id_;
  std::array<uint32_t, 3> data_ids_;
  int stage_{0};
  double fx_{0.0};
  double fy_{0.0};
  double fz_{0.0};
  double mx_{0.0};
};

// 可选六维力滤波器（手册 4.3 节：none 不滤波 / moving_average 滑动均值 / low_pass 一阶低通）
class WrenchFilter {
public:
  WrenchFilter(const std::string & mode = "none", int window = 5, double alpha = 0.2);

  const std::string & mode() const noexcept { return mode_; }
  void reset() noexcept;
  WrenchSample apply(const WrenchSample & sample);

private:
  std::string mode_;
  size_t window_;
  double alpha_;
  std::deque<std::array<double, 6>> history_;
  std::array<double, 6> sums_{};
  bool state_initialized_{false};
  std::array<double, 6> state_{};
};

}  // namespace paxini_driver
