#include "paxini_driver/transport.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/serial.h>
#include <net/if.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <system_error>

namespace paxini_driver {
namespace {

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

speed_t to_termios_baud(int baudrate) {
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1152000: return B1152000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    default:
      throw std::invalid_argument("Unsupported serial baudrate: " + std::to_string(baudrate));
  }
}

timespec to_timespec(double sec) noexcept {
  if (!(sec > 0.0)) {
    return timespec{0, 0};
  }
  const auto whole = static_cast<time_t>(sec);
  const auto nsec = static_cast<long>((sec - static_cast<double>(whole)) * 1e9);
  return timespec{whole, std::clamp<long>(nsec, 0L, 999999999L)};
}

bool wait_fd_readable(int fd, double timeout_sec) {
  if (timeout_sec <= 0.0) {
    return false;
  }
  // 使用纳秒精度的 ppoll，避免 1 kHz 轮询下亚毫秒剩余等待时间被毫秒级 poll 截断成 0 导致忙等
  pollfd pfd{};
  pfd.fd = fd;
  pfd.events = POLLIN;
  const timespec ts = to_timespec(timeout_sec);
  const int rc = ::ppoll(&pfd, 1, &ts, nullptr);
  if (rc == 0 || (rc < 0 && errno == EINTR)) {
    return false;
  }
  if (rc < 0) {
    throw std::system_error(errno, std::generic_category(), "ppoll(POLLIN) failed");
  }
  if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) && !(pfd.revents & POLLIN)) {
    throw std::runtime_error("Device hangup or I/O error");
  }
  return (pfd.revents & (POLLIN | POLLERR | POLLHUP)) != 0;
}

void write_all_with_timeout(int fd, const uint8_t * data, size_t len, double timeout_sec) {
  size_t written = 0;
  while (written < len) {
    const ssize_t n = ::write(fd, data + written, len - written);
    if (n > 0) {
      written += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd pfd{};
      pfd.fd = fd;
      pfd.events = POLLOUT;
      const timespec ts = to_timespec(timeout_sec);
      const int rc = ::ppoll(&pfd, 1, &ts, nullptr);
      if (rc <= 0) {
        throw std::runtime_error("Write timeout on sensor transport");
      }
      if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        throw std::runtime_error("Transport disconnected during write");
      }
      continue;
    }
    throw std::system_error(errno, std::generic_category(), "Transport write failed");
  }
}

}  // namespace

SerialSensorTransport::SerialSensorTransport(
  std::string port,
  int baudrate,
  int device_id,
  std::string interface_name,
  FdOpener fd_opener)
: port_(std::move(port)),
  baudrate_(baudrate),
  device_id_(device_id),
  interface_name_(trim_lower(interface_name)),
  fd_opener_(std::move(fd_opener)),
  parser_(device_id_)
{}

SerialSensorTransport::~SerialSensorTransport() {
  close();
}

void SerialSensorTransport::open() {
  if (is_open()) {
    return;
  }
  if (fd_opener_) {
    const int opened = fd_opener_(port_, baudrate_);
    if (opened < 0) {
      throw std::runtime_error("Custom serial opener failed for " + port_);
    }
    const int flags = ::fcntl(opened, F_GETFL, 0);
    if (flags >= 0 && !(flags & O_NONBLOCK)) {
      (void)::fcntl(opened, F_SETFL, flags | O_NONBLOCK);
    }
    fd_ = opened;
    parser_.reset();
    return;
  }

  const speed_t speed = to_termios_baud(baudrate_);
  const int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    throw std::system_error(errno, std::generic_category(), "Cannot open serial port " + port_);
  }

  // 独占锁定串口，防止双臂同时启动或自动切口瞬态下两个驱动进程并发读写同一 ttyUSB 分流字节
  if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
    const int err = errno;
    ::close(fd);
    throw std::system_error(err, std::generic_category(), "Serial port is busy: " + port_);
  }
  (void)::ioctl(fd, TIOCEXCL);

  termios tio{};
  if (::tcgetattr(fd, &tio) != 0) {
    const int err = errno;
    ::close(fd);
    throw std::system_error(err, std::generic_category(), "tcgetattr failed on " + port_);
  }
  ::cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD | CS8);
  tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  // 配合 O_NONBLOCK 设置 VMIN=1, VTIME=0：缓冲区为空时返回 EAGAIN，仅在串口物理断开/挂起时返回 0(EOF) 或 EIO
  tio.c_cc[VMIN] = 1;
  tio.c_cc[VTIME] = 0;
  ::cfsetispeed(&tio, speed);
  ::cfsetospeed(&tio, speed);
  if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
    const int err = errno;
    ::close(fd);
    throw std::system_error(err, std::generic_category(), "tcsetattr failed on " + port_);
  }

  // 对支持 serial_struct 的 USB 转串口芯片（如 FTDI）开启低延迟模式，消除默认 16 ms 批量缓冲延迟
  serial_struct ser_info{};
  if (::ioctl(fd, TIOCGSERIAL, &ser_info) == 0 && !(ser_info.flags & ASYNC_LOW_LATENCY)) {
    ser_info.flags |= ASYNC_LOW_LATENCY;
    (void)::ioctl(fd, TIOCSSERIAL, &ser_info);
  }
  (void)::tcflush(fd, TCIFLUSH);

  fd_ = fd;
  parser_.reset();
}

void SerialSensorTransport::close() noexcept {
  std::lock_guard<std::mutex> lock(write_mutex_);
  const int fd = fd_;
  fd_ = -1;
  parser_.reset();
  if (fd >= 0) {
    (void)::ioctl(fd, TIOCNXCL);
    (void)::flock(fd, LOCK_UN);
    ::close(fd);
  }
}

void SerialSensorTransport::send_command(uint8_t cmd, std::optional<int> arg) {
  if (cmd == CMD_SET_FREQ && arg.has_value()) {
    arg = static_cast<int>(freq_to_code(*arg));
  }
  // RS485 半双工收发切换前导唤醒字节 0xFF
  const auto frame =
    build_serial_command_frame(cmd, arg, device_id_, interface_name_ == "rs485");
  std::lock_guard<std::mutex> lock(write_mutex_);
  if (fd_ < 0) {
    throw std::runtime_error("Serial transport is not open");
  }
  write_all_with_timeout(fd_, frame.bytes.data(), frame.size, 0.2);
}

void SerialSensorTransport::poll_events_into(
  double timeout_sec,
  std::vector<ProtocolEvent> & out)
{
  out.clear();
  if (fd_ < 0) {
    throw std::runtime_error("Serial transport is not open");
  }
  uint8_t tmp[512];
  ssize_t n = ::read(fd_, tmp, sizeof(tmp));
  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    if (!wait_fd_readable(fd_, timeout_sec)) {
      return;
    }
    n = ::read(fd_, tmp, sizeof(tmp));
  }
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return;
    }
    throw std::system_error(errno, std::generic_category(), "Serial read failed");
  }
  if (n == 0) {
    throw std::runtime_error("Serial port EOF / disconnected");
  }

  // 直接在栈缓冲上喂入解析器，仅当单次读满缓冲区时才继续排空内核 FIFO，省去多余 read 系统调用和临时堆分配
  parser_.feed(tmp, static_cast<size_t>(n), out);
  while (n == static_cast<ssize_t>(sizeof(tmp))) {
    const ssize_t more = ::read(fd_, tmp, sizeof(tmp));
    if (more > 0) {
      parser_.feed(tmp, static_cast<size_t>(more), out);
      n = more;
    } else {
      break;
    }
  }
}

std::vector<ProtocolEvent> SerialSensorTransport::poll_events(double timeout_sec) {
  std::vector<ProtocolEvent> out;
  poll_events_into(timeout_sec, out);
  return out;
}

CanSensorTransport::CanSensorTransport(
  std::string channel,
  bool fd,
  int device_id)
: channel_(std::move(channel)),
  fd_(fd),
  device_id_(device_id),
  interface_name_(fd ? "canfd" : "can"),
  parser_(device_id_)
{}

CanSensorTransport::~CanSensorTransport() {
  close();
}

void CanSensorTransport::open() {
  if (is_open()) {
    return;
  }

  const int sock = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (sock < 0) {
    throw std::system_error(errno, std::generic_category(), "socket(PF_CAN) failed");
  }

  const auto data_ids = can_data_ids(device_id_);
  // 在内核层只过滤本传感器相关的应答帧与数据帧（11 位标准帧，排除远程帧和扩展帧）
  const std::array<uint32_t, 4> rx_ids = {
    can_ack_id(device_id_), data_ids[0], data_ids[1], data_ids[2]
  };
  std::array<can_filter, 4> filters{};
  for (size_t i = 0; i < rx_ids.size(); ++i) {
    filters[i].can_id = rx_ids[i];
    filters[i].can_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
  }
  if (::setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(filters)) != 0) {
    const int err = errno;
    ::close(sock);
    throw std::system_error(err, std::generic_category(), "setsockopt(CAN_RAW_FILTER) failed");
  }

  if (fd_) {
    const int enable_canfd = 1;
    if (::setsockopt(
          sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) != 0)
    {
      const int err = errno;
      ::close(sock);
      throw std::system_error(err, std::generic_category(), "setsockopt(CAN_RAW_FD_FRAMES) failed");
    }
  }

  ifreq ifr{};
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", channel_.c_str());
  if (::ioctl(sock, SIOCGIFINDEX, &ifr) != 0) {
    const int err = errno;
    ::close(sock);
    throw std::system_error(
      err, std::generic_category(), "CAN interface '" + channel_ + "' not found");
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    const int err = errno;
    ::close(sock);
    throw std::system_error(
      err, std::generic_category(), "bind(" + channel_ + ") failed");
  }

  sock_ = sock;
  parser_.reset();
}

void CanSensorTransport::close() noexcept {
  std::lock_guard<std::mutex> lock(tx_mutex_);
  const int sock = sock_;
  sock_ = -1;
  parser_.reset();
  if (sock >= 0) {
    ::close(sock);
  }
}

void CanSensorTransport::send_command(uint8_t cmd, std::optional<int> arg) {
  uint32_t can_id = 0;
  std::array<uint8_t, 8> payload{};
  if (cmd == CMD_SET_PORT_PARAMS) {
    std::tie(can_id, payload) = build_can_set_protocol(arg.value_or(0) != 0, device_id_);
  } else {
    if (cmd == CMD_SET_FREQ && arg.has_value()) {
      arg = static_cast<int>(freq_to_code(*arg));
    }
    std::tie(can_id, payload) = build_can_command(cmd, arg, device_id_);
  }

  std::lock_guard<std::mutex> lock(tx_mutex_);
  if (sock_ < 0) {
    throw std::runtime_error("CAN transport is not open");
  }
  if (fd_) {
    canfd_frame frame{};
    frame.can_id = can_id;
    frame.len = 8;
    std::memcpy(frame.data, payload.data(), frame.len);
    write_all_with_timeout(
      sock_, reinterpret_cast<const uint8_t *>(&frame), sizeof(frame), 0.1);
  } else {
    can_frame frame{};
    frame.can_id = can_id;
    frame.can_dlc = 8;
    std::memcpy(frame.data, payload.data(), frame.can_dlc);
    write_all_with_timeout(
      sock_, reinterpret_cast<const uint8_t *>(&frame), sizeof(frame), 0.1);
  }
}

void CanSensorTransport::poll_events_into(
  double timeout_sec,
  std::vector<ProtocolEvent> & events)
{
  events.clear();
  if (sock_ < 0) {
    throw std::runtime_error("CAN transport is not open");
  }
  for (int i = 0; i < 64; ++i) {
    canfd_frame frame{};
    ssize_t n = ::read(sock_, &frame, sizeof(frame));
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      if (i > 0 || !wait_fd_readable(sock_, timeout_sec)) {
        break;
      }
      n = ::read(sock_, &frame, sizeof(frame));
    }
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        break;
      }
      throw std::system_error(errno, std::generic_category(), "CAN socket read failed");
    }
    if (n != static_cast<ssize_t>(CAN_MTU) && n != static_cast<ssize_t>(CANFD_MTU)) {
      continue;
    }
    if (frame.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG)) {
      continue;
    }
    const size_t dlc = (n == static_cast<ssize_t>(CANFD_MTU))
      ? std::min<size_t>(frame.len, CANFD_MAX_DLEN)
      : std::min<size_t>(frame.len, CAN_MAX_DLEN);
    auto ev = parser_.push(frame.can_id & CAN_SFF_MASK, frame.data, dlc);
    if (ev.has_value()) {
      events.push_back(std::move(*ev));
    }
  }
}

std::vector<ProtocolEvent> CanSensorTransport::poll_events(double timeout_sec) {
  std::vector<ProtocolEvent> events;
  poll_events_into(timeout_sec, events);
  return events;
}

std::unique_ptr<SensorTransport> create_transport(
  const std::string & interface,
  const std::string & serial_port,
  int baudrate,
  const std::string & can_channel,
  int device_id)
{
  const std::string iface = trim_lower(interface);
  if (iface == "usb" || iface == "rs485" || iface == "serial") {
    return std::make_unique<SerialSensorTransport>(
      serial_port, baudrate, device_id, iface);
  }
  if (iface == "can" || iface == "canfd") {
    return std::make_unique<CanSensorTransport>(
      can_channel, iface == "canfd", device_id);
  }
  throw std::invalid_argument(
    "Unsupported interface '" + interface + "'; expected rs485, usb, can, or canfd");
}

}  // namespace paxini_driver
