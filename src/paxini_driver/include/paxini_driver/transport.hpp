#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "paxini_driver/protocol.hpp"

namespace paxini_driver {

// 单个 PX6D 传感器的底层物理传输抽象接口（RS485 / USB 串口 / CAN 2.0 / CANFD）
class SensorTransport {
public:
  virtual ~SensorTransport() = default;

  virtual const std::string & interface_name() const noexcept = 0;
  virtual bool is_open() const noexcept = 0;
  virtual void open() = 0;
  virtual void close() noexcept = 0;
  virtual void poll_events_into(double timeout_sec, std::vector<ProtocolEvent> & out) {
    out = poll_events(timeout_sec);
  }
  virtual std::vector<ProtocolEvent> poll_events(double timeout_sec = 0.002) = 0;
  virtual void send_command(uint8_t cmd, std::optional<int> arg = std::nullopt) = 0;

  virtual bool is_real_serial() const noexcept { return false; }
  virtual bool has_serial_port() const noexcept { return false; }
  virtual std::string port() const { return {}; }
  virtual void set_port(const std::string &) {}
};

// 基于 Linux termios + ppoll 的 RS485 / USB 串口传输实现（默认 921600 8N1，无流控）
class SerialSensorTransport final : public SensorTransport {
public:
  using FdOpener = std::function<int(const std::string & port, int baudrate)>;

  SerialSensorTransport(
    std::string port = "/dev/ttyUSB0",
    int baudrate = DEFAULT_BAUDRATE,
    int device_id = DEFAULT_DEVICE_ID,
    std::string interface_name = "usb",
    FdOpener fd_opener = nullptr);

  ~SerialSensorTransport() override;

  const std::string & interface_name() const noexcept override { return interface_name_; }
  bool is_open() const noexcept override { return fd_ >= 0; }
  void open() override;
  void close() noexcept override;
  void poll_events_into(double timeout_sec, std::vector<ProtocolEvent> & out) override;
  std::vector<ProtocolEvent> poll_events(double timeout_sec = 0.002) override;
  void send_command(uint8_t cmd, std::optional<int> arg = std::nullopt) override;

  bool is_real_serial() const noexcept override { return fd_opener_ == nullptr; }
  bool has_serial_port() const noexcept override { return true; }
  std::string port() const override { return port_; }
  void set_port(const std::string & p) override { port_ = p; }

private:
  std::string port_;
  int baudrate_;
  int device_id_;
  std::string interface_name_;
  FdOpener fd_opener_;
  int fd_{-1};
  SerialFrameParser parser_;
  std::mutex write_mutex_;
};

// 基于 Linux SocketCAN（PF_CAN / CAN_RAW）的 CAN 2.0 与 CANFD 传输实现
class CanSensorTransport final : public SensorTransport {
public:
  CanSensorTransport(
    std::string channel = "can0",
    bool fd = false,
    int device_id = DEFAULT_DEVICE_ID);

  ~CanSensorTransport() override;

  const std::string & interface_name() const noexcept override { return interface_name_; }
  bool is_open() const noexcept override { return sock_ >= 0; }
  void open() override;
  void close() noexcept override;
  void poll_events_into(double timeout_sec, std::vector<ProtocolEvent> & out) override;
  std::vector<ProtocolEvent> poll_events(double timeout_sec = 0.002) override;
  void send_command(uint8_t cmd, std::optional<int> arg = std::nullopt) override;

private:
  std::string channel_;
  bool fd_;
  int device_id_;
  std::string interface_name_;
  int sock_{-1};
  CanFrameParser parser_;
  std::mutex tx_mutex_;
};

std::unique_ptr<SensorTransport> create_transport(
  const std::string & interface,
  const std::string & serial_port = "/dev/ttyUSB0",
  int baudrate = DEFAULT_BAUDRATE,
  const std::string & can_channel = "can0",
  int device_id = DEFAULT_DEVICE_ID);

}  // namespace paxini_driver
