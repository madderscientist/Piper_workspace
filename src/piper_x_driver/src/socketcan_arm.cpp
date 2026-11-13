#include "piper_x_driver/socketcan_arm.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/net_tstamp.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace piper_x_driver {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

#ifndef SO_TIMESTAMPNS
#define SO_TIMESTAMPNS 35
#endif

double wall_now() noexcept {
  struct timespec ts{};
  ::clock_gettime(CLOCK_REALTIME, &ts);
  return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

double mono_now() noexcept {
  struct timespec ts{};
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

int16_t get_be_i16(const uint8_t * p) noexcept {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | static_cast<uint16_t>(p[1]));
}

int32_t get_be_i32(const uint8_t * p) noexcept {
  return static_cast<int32_t>(
    (static_cast<uint32_t>(p[0]) << 24) |
    (static_cast<uint32_t>(p[1]) << 16) |
    (static_cast<uint32_t>(p[2]) << 8) |
    static_cast<uint32_t>(p[3]));
}

std::string trim_fw(const uint8_t * p, size_t n) {
  size_t b = 0;
  while (b < n && (p[b] == 0 || p[b] == ' ')) {
    ++b;
  }
  size_t e = n;
  while (e > b && (p[e - 1] == 0 || p[e - 1] == ' ')) {
    --e;
  }
  return std::string(reinterpret_cast<const char *>(p + b), e - b);
}

bool supported_firmware(const std::string & sw) {
  return sw == "S-V1.9-0" || sw == "S-V1.9-1";
}

}  // namespace

SocketCanArm::SocketCanArm(const std::string & channel)
: channel_(channel)
{
  try {
    open_and_configure_socket();
    rx_thread_ = std::thread(&SocketCanArm::rx_loop, this);

    // Exit native Leader mode if left from a previous session so 200Hz CAN push resumes.
    send_frame(encode_leader_follower_config(0xFC));

    // Wait for receive stream to become active (up to 2.0 s).
    const double stream_deadline = mono_now() + 2.0;
    {
      std::unique_lock<std::mutex> lock(rx_mu_);
      while (rx_frame_counter_ == 0) {
        if (mono_now() >= stream_deadline) {
          throw std::runtime_error(channel_ + ": CAN receive stream did not become ready");
        }
        rx_cv_.wait_for(lock, std::chrono::milliseconds(20));
      }
    }

    // Query and verify firmware (S-V1.9-0 / S-V1.9-1), up to 4 attempts.
    std::string sw_version;
    for (int attempt = 0; attempt < 4; ++attempt) {
      {
        std::lock_guard<std::mutex> lock(rx_mu_);
        fw_buf_.clear();
      }
      send_frame(encode_req_firmware());
      const double timeout_s = (attempt == 0) ? 2.0 : 0.5;
      const double fw_deadline = mono_now() + timeout_s;
      std::unique_lock<std::mutex> lock(rx_mu_);
      while (mono_now() < fw_deadline) {
        if (fw_buf_.size() >= 3 &&
            !(fw_buf_[0] == 'H' && fw_buf_[1] == '-' && fw_buf_[2] == 'V'))
        {
          fw_buf_.clear();
        }
        if (fw_buf_.size() >= 88) {
          sw_version = trim_fw(fw_buf_.data() + 60, 8);
          break;
        }
        rx_cv_.wait_for(lock, std::chrono::milliseconds(5));
      }
      lock.unlock();
      if (supported_firmware(sw_version)) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!supported_firmware(sw_version)) {
      throw std::runtime_error(
        channel_ + ": expected ('S-V1.9-0', 'S-V1.9-1'), received '" + sw_version + "'");
    }

    // Wait up to 1.5 s for complete fresh feedback; exit residual drag-recording if teach_status == 1.
    const double wait_until = mono_now() + 1.5;
    while (mono_now() < wait_until) {
      const Feedback sample = snapshot();
      if (sample.status.has_value() && sample.position.has_value() && fresh(sample.age)) {
        if (sample.status->teach_status == 1) {
          send_frame(encode_motion_ctrl(0, 0, 2));
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    grip_thread_ = std::thread(&SocketCanArm::grip_interp_loop, this);
  } catch (...) {
    disconnect_internal();
    throw;
  }
}

SocketCanArm::~SocketCanArm() {
  try {
    close();
  } catch (...) {
  }
}

void SocketCanArm::open_and_configure_socket() {
  sock_ = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
  if (sock_ < 0) {
    throw std::runtime_error(
      channel_ + ": socket(PF_CAN) failed: " + std::string(std::strerror(errno)));
  }

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, channel_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
    const int err = errno;
    ::close(sock_);
    sock_ = -1;
    throw std::runtime_error(
      "Device '" + channel_ + "' does not exist: " + std::string(std::strerror(err)));
  }
  const int ifindex = ifr.ifr_ifindex;

  // Enable kernel nanosecond receive timestamps (matches python-can SO_TIMESTAMPNS).
  int ts_enable = 1;
  (void)::setsockopt(sock_, SOL_SOCKET, SO_TIMESTAMPNS, &ts_enable, sizeof(ts_enable));

  int loopback = 1;
  (void)::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
  int recv_own = 0;
  (void)::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own, sizeof(recv_own));

  // Kernel-level whitelist filter for all arm + gripper RX CAN IDs.
  static constexpr uint32_t kAllowedIds[] = {
    0x155, 0x156, 0x157, 0x159,
    0x251, 0x252, 0x253, 0x254, 0x255, 0x256,
    0x261, 0x262, 0x263, 0x264, 0x265, 0x266,
    0x2A1, 0x2A2, 0x2A3, 0x2A4, 0x2A5, 0x2A6, 0x2A7, 0x2A8,
    0x476, 0x4AF,
  };
  struct can_filter filters[sizeof(kAllowedIds) / sizeof(kAllowedIds[0])];
  for (size_t i = 0; i < sizeof(kAllowedIds) / sizeof(kAllowedIds[0]); ++i) {
    filters[i].can_id = kAllowedIds[i];
    filters[i].can_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
  }
  (void)::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters));

  int rcvbuf = SOCKETCAN_RCVBUF_REQUEST;
  (void)::setsockopt(sock_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifindex;
  if (::bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    const int err = errno;
    ::close(sock_);
    sock_ = -1;
    throw std::runtime_error(
      channel_ + ": bind failed: " + std::string(std::strerror(err)));
  }

  // Drain any stale queued frames and immediately detect ENETDOWN if interface is DOWN.
  struct can_frame drain_frame{};
  while (true) {
    const ssize_t n = ::recv(sock_, &drain_frame, sizeof(drain_frame), MSG_DONTWAIT);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      const int err = errno;
      ::close(sock_);
      sock_ = -1;
      throw std::runtime_error(
        "Error receiving on " + channel_ + ": " + std::string(std::strerror(err)) +
        " [Error Code " + std::to_string(err) + "]");
    }
    if (n == 0) {
      break;
    }
  }
}

void SocketCanArm::rx_loop() {
  constexpr size_t kBatchSize = 32;
  struct can_frame frames[kBatchSize];
  struct iovec iovs[kBatchSize];
  alignas(struct cmsghdr) char ctrl_bufs[kBatchSize][CMSG_SPACE(sizeof(struct timespec))];
  struct mmsghdr msgs[kBatchSize];
  std::memset(msgs, 0, sizeof(msgs));
  for (size_t i = 0; i < kBatchSize; ++i) {
    iovs[i].iov_base = &frames[i];
    iovs[i].iov_len = sizeof(struct can_frame);
    msgs[i].msg_hdr.msg_iov = &iovs[i];
    msgs[i].msg_hdr.msg_iovlen = 1;
    msgs[i].msg_hdr.msg_control = ctrl_bufs[i];
    msgs[i].msg_hdr.msg_controllen = sizeof(ctrl_bufs[i]);
  }

  struct pollfd pfd{};
  pfd.fd = sock_;
  pfd.events = POLLIN;

  while (!stop_rx_.load(std::memory_order_relaxed)) {
    const int pr = ::poll(&pfd, 1, 50);
    if (pr <= 0) {
      continue;
    }
    for (size_t i = 0; i < kBatchSize; ++i) {
      msgs[i].msg_hdr.msg_controllen = sizeof(ctrl_bufs[i]);
      msgs[i].msg_hdr.msg_flags = 0;
    }
    const int nmsgs = ::recvmmsg(sock_, msgs, kBatchSize, MSG_DONTWAIT, nullptr);
    if (nmsgs < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR ||
          errno == ENETDOWN || errno == ENOBUFS)
      {
        continue;
      }
      break;
    }
    if (nmsgs == 0) {
      continue;
    }
    const double fallback_ts = wall_now();
    bool wake_waiters = false;
    {
      std::lock_guard<std::mutex> lock(rx_mu_);
      const bool was_zero = (rx_frame_counter_ == 0);
      for (int i = 0; i < nmsgs; ++i) {
        if (msgs[i].msg_len < sizeof(struct can_frame)) {
          continue;
        }
        const struct can_frame & cf = frames[i];
        if ((cf.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG)) != 0) {
          continue;
        }
        double stamp = fallback_ts;
        for (struct cmsghdr * cmsg = CMSG_FIRSTHDR(&msgs[i].msg_hdr);
             cmsg != nullptr;
             cmsg = CMSG_NXTHDR(&msgs[i].msg_hdr, cmsg))
        {
          if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMPNS) {
            struct timespec ts{};
            std::memcpy(&ts, CMSG_DATA(cmsg), sizeof(ts));
            stamp = static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
            break;
          }
        }
        const uint32_t cid = cf.can_id & CAN_SFF_MASK;
        if (cid == 0x4AF || cid == 0x476) {
          wake_waiters = true;
        }
        parse_frame(cid, cf.can_dlc, cf.data, stamp);
        ++rx_frame_counter_;
      }
      if (was_zero && rx_frame_counter_ > 0) {
        wake_waiters = true;
      }
    }
    if (wake_waiters) {
      rx_cv_.notify_all();
    }
  }
}

void SocketCanArm::parse_frame(
  uint32_t can_id, uint8_t dlc, const uint8_t * d, double rx_stamp)
{
  switch (can_id) {
    case 0x2A5:
    case 0x2A6:
    case 0x2A7: {
      if (dlc < 8) return;
      const int p = static_cast<int>(can_id - 0x2A5);
      joint_pos_[2 * p] = static_cast<double>(get_be_i32(d)) * 1e-3 * kDeg2Rad;
      joint_pos_[2 * p + 1] = static_cast<double>(get_be_i32(d + 4)) * 1e-3 * kDeg2Rad;
      joint_pair_stamp_[p] = rx_stamp;
      has_joint_pair_[p] = true;
      break;
    }
    case 0x2A8: {
      if (dlc < 8) return;
      const int32_t raw_val = get_be_i32(d);
      grip_force_ = static_cast<double>(get_be_i16(d + 4)) * 1e-3;
      const uint8_t status_code = d[6];
      const uint8_t g_mode = d[7];
      grip_angle_mode_ = (g_mode == 0x01);
      grip_value_ = grip_angle_mode_
        ? (static_cast<double>(raw_val) * 1e-3)
        : (static_cast<double>(raw_val) * 1e-6);
      grip_enabled_ = ((status_code & (1u << 6)) != 0);
      grip_stamp_ = rx_stamp;
      has_grip_ = true;
      break;
    }
    case 0x2A1: {
      if (dlc < 8) return;
      status_.ctrl_mode = d[0];
      status_.arm_status = d[1];
      status_.mode_feedback = d[2];
      status_.teach_status = d[3];
      status_.motion_status = d[4];
      status_.trajectory_num = d[5];
      status_.err_code = static_cast<int>((static_cast<uint16_t>(d[6]) << 8) | d[7]);
      status_stamp_ = rx_stamp;
      has_status_ = true;
      break;
    }
    case 0x2A2: {
      if (dlc < 8) return;
      pose_[0] = static_cast<double>(get_be_i32(d)) * 1e-6;
      pose_[1] = static_cast<double>(get_be_i32(d + 4)) * 1e-6;
      pose_stamp_[0] = rx_stamp;
      has_pose_part_[0] = true;
      break;
    }
    case 0x2A3: {
      if (dlc < 8) return;
      pose_[2] = static_cast<double>(get_be_i32(d)) * 1e-6;
      pose_[3] = static_cast<double>(get_be_i32(d + 4)) * 1e-3 * kDeg2Rad;
      pose_stamp_[1] = rx_stamp;
      has_pose_part_[1] = true;
      break;
    }
    case 0x2A4: {
      if (dlc < 8) return;
      pose_[4] = static_cast<double>(get_be_i32(d)) * 1e-3 * kDeg2Rad;
      pose_[5] = static_cast<double>(get_be_i32(d + 4)) * 1e-3 * kDeg2Rad;
      pose_stamp_[2] = rx_stamp;
      has_pose_part_[2] = true;
      break;
    }
    case 0x251:
    case 0x252:
    case 0x253:
    case 0x254:
    case 0x255:
    case 0x256: {
      if (dlc < 8) return;
      const int j = static_cast<int>(can_id - 0x251);
      motor_vel_[j] = static_cast<double>(get_be_i16(d)) * 1e-3;
      const double current = static_cast<double>(get_be_i16(d + 2)) * 1e-3;
      motor_torque_[j] = current * HIGH_SPD_TORQUE_SCALE[j];
      motor_stamp_[j] = rx_stamp;
      has_motor_[j] = true;
      break;
    }
    case 0x261:
    case 0x262:
    case 0x263:
    case 0x264:
    case 0x265:
    case 0x266: {
      if (dlc < 8) return;
      const int j = static_cast<int>(can_id - 0x261);
      driver_enabled_[j] = ((d[5] & (1u << 6)) != 0);
      driver_stamp_[j] = rx_stamp;
      has_driver_[j] = true;
      break;
    }
    case 0x155:
    case 0x156:
    case 0x157: {
      if (dlc < 8) return;
      const int p = static_cast<int>(can_id - 0x155);
      leader_pos_[2 * p] = static_cast<double>(get_be_i32(d)) * 1e-3 * kDeg2Rad;
      leader_pos_[2 * p + 1] = static_cast<double>(get_be_i32(d + 4)) * 1e-3 * kDeg2Rad;
      leader_pair_stamp_[p] = rx_stamp;
      has_leader_pair_[p] = true;
      break;
    }
    case 0x476: {
      if (dlc < 2) return;
      has_resp_476_ = true;
      resp_476_index_ = d[0];
      resp_476_zero_ok_ = d[1];
      break;
    }
    case 0x4AF: {
      if (dlc != 8) return;
      if (d[0] == 0x01 && d[1] == 0 && d[2] == 0 && d[3] == 0 &&
          d[4] == 0 && d[5] == 0 && d[6] == 0 && d[7] == 0)
      {
        return;
      }
      if (fw_buf_.size() + 8 > 88) {
        fw_buf_.clear();
      }
      fw_buf_.insert(fw_buf_.end(), d, d + 8);
      break;
    }
    default:
      break;
  }
}

Feedback SocketCanArm::snapshot() {
  const bool drag_active = in_drag_mode_.load(std::memory_order_relaxed);

  std::lock_guard<std::mutex> lock(rx_mu_);
  // Sample CLOCK_REALTIME *after* acquiring rx_mu_ so frames committed just before
  // rx_mu_ was released never appear in the future; clamp small kernel/NTP skew (<0) to 0.
  const double now = wall_now();
  auto compute_age = [now](double stamp) noexcept -> double {
    return normalize_feedback_age(now, stamp);
  };
  Feedback out;
  double stamps[11] = {
    has_joint_pair_[0] ? joint_pair_stamp_[0] : -1.0,
    has_joint_pair_[1] ? joint_pair_stamp_[1] : -1.0,
    has_joint_pair_[2] ? joint_pair_stamp_[2] : -1.0,
    has_grip_ ? grip_stamp_ : -1.0,
    has_status_ ? status_stamp_ : -1.0,
    has_driver_[0] ? driver_stamp_[0] : -1.0,
    has_driver_[1] ? driver_stamp_[1] : -1.0,
    has_driver_[2] ? driver_stamp_[2] : -1.0,
    has_driver_[3] ? driver_stamp_[3] : -1.0,
    has_driver_[4] ? driver_stamp_[4] : -1.0,
    has_driver_[5] ? driver_stamp_[5] : -1.0,
  };
  if (stamps[0] >= 0.0 && stamps[0] != last_joint_rx_stamp_) {
    last_joint_rx_stamp_ = stamps[0];
    can_rx_updates_.fetch_add(1, std::memory_order_relaxed);
  }
  for (int i = 0; i < 11; ++i) {
    out.age[i] = compute_age(stamps[i]);
  }

  if (has_joint_pair_[0] && has_joint_pair_[1] && has_joint_pair_[2] && has_grip_) {
    if (grip_angle_mode_) {
      throw std::invalid_argument("This workspace requires width-mode gripper feedback");
    }
    out.position = std::array<double, 7>{{
      joint_pos_[0], joint_pos_[1], joint_pos_[2],
      joint_pos_[3], joint_pos_[4], joint_pos_[5],
      grip_value_
    }};
  } else if (drag_active &&
             has_leader_pair_[0] && has_leader_pair_[1] && has_leader_pair_[2])
  {
    const double la0 = compute_age(leader_pair_stamp_[0]);
    const double la1 = compute_age(leader_pair_stamp_[1]);
    const double la2 = compute_age(leader_pair_stamp_[2]);
    if (0.0 <= la0 && la0 <= 1.0 && 0.0 <= la1 && la1 <= 1.0 && 0.0 <= la2 && la2 <= 1.0) {
      bool in_bounds = true;
      for (int i = 0; i < 6; ++i) {
        if (std::abs(leader_pos_[i]) > 2.0 * kPi) {
          in_bounds = false;
          break;
        }
      }
      if (in_bounds) {
        const double w = has_grip_ ? grip_value_ : 0.0;
        out.position = std::array<double, 7>{{
          leader_pos_[0], leader_pos_[1], leader_pos_[2],
          leader_pos_[3], leader_pos_[4], leader_pos_[5], w
        }};
        out.age[0] = la0;
        out.age[1] = la1;
        out.age[2] = la2;
        out.age[3] = 0.0;
      }
    }
  }

  for (int i = 0; i < 6; ++i) {
    out.enabled[i] = has_driver_[i] && driver_enabled_[i];
  }
  out.enabled[6] = has_grip_ && grip_enabled_;

  if (has_status_) {
    out.status = status_;
  }

  bool motors_fresh = true;
  for (int i = 0; i < 6; ++i) {
    const double ma = compute_age(has_motor_[i] ? motor_stamp_[i] : -1.0);
    if (ma < 0.0 || ma > 1.0) {
      motors_fresh = false;
      break;
    }
  }
  if (motors_fresh) {
    out.has_velocity_torque = true;
    out.velocity = motor_vel_;
    out.torque = motor_torque_;
  }

  bool pose_fresh = true;
  for (int i = 0; i < 3; ++i) {
    const double pa = compute_age(has_pose_part_[i] ? pose_stamp_[i] : -1.0);
    if (pa < 0.0 || pa > 1.0) {
      pose_fresh = false;
      break;
    }
  }
  if (pose_fresh) {
    out.pose = pose_;
  }

  if (has_grip_ && 0.0 <= out.age[3] && out.age[3] <= 1.0) {
    if (std::isfinite(grip_force_)) {
      out.gripper_force = grip_force_;
    }
    const double g_stamp = grip_stamp_;
    if (grip_history_.empty() || g_stamp > grip_history_.back().first) {
      if (!grip_history_.empty() && g_stamp - grip_history_.back().first > 0.5) {
        grip_history_.clear();
      }
      grip_history_.emplace_back(g_stamp, grip_value_);
      while (grip_history_.size() > 2 && g_stamp - grip_history_.front().first > 0.030) {
        grip_history_.pop_front();
      }
    }
    if (grip_history_.size() >= 4 &&
        std::abs(grip_history_.back().second - grip_history_[grip_history_.size() - 4].second) < 5e-5)
    {
      grip_vel_ = 0.0;
    } else if (grip_history_.size() >= 2) {
      const double dt = grip_history_.back().first - grip_history_.front().first;
      grip_vel_ = (dt >= 1e-4)
        ? ((grip_history_.back().second - grip_history_.front().second) / dt)
        : 0.0;
    } else {
      grip_vel_ = 0.0;
    }
    out.gripper_velocity = grip_vel_;
  }
  return out;
}

void SocketCanArm::send_frame(const RawCanFrame & frame) {
  if (sock_ < 0) {
    throw std::runtime_error("CAN bus is not connected");
  }
  if (frame.can_id == 0x151) {
    std::lock_guard<std::mutex> lock(mode_tx_mu_);
    const double now = mono_now();
    if (last_0x151_tx_mono_ >= 0.0 &&
        frame.dlc == last_0x151_.dlc &&
        frame.data == last_0x151_.data &&
        (now - last_0x151_tx_mono_) < 0.10)
    {
      return;
    }
    last_0x151_ = frame;
    last_0x151_tx_mono_ = now;
  }
  struct can_frame cf{};
  cf.can_id = frame.can_id & CAN_SFF_MASK;
  cf.can_dlc = frame.dlc;
  std::memcpy(cf.data, frame.data.data(), frame.dlc);
  const ssize_t sent = ::send(sock_, &cf, sizeof(cf), MSG_DONTWAIT);
  if (sent < 0) {
    const int err = errno;
    if (err == ENOBUFS || err == EAGAIN || err == EWOULDBLOCK || err == ENETDOWN) {
      return;
    }
    throw std::runtime_error(
      "Failed to send CAN frame on '" + channel_ + "': " + std::string(std::strerror(err)));
  }
}

void SocketCanArm::send_frames_batch(const RawCanFrame * frames, size_t count) {
  if (sock_ < 0) {
    throw std::runtime_error("CAN bus is not connected");
  }
  if (count == 0 || count > 8) {
    for (size_t i = 0; i < count; ++i) {
      send_frame(frames[i]);
    }
    return;
  }
  struct can_frame cfs[8]{};
  struct iovec iovs[8]{};
  struct mmsghdr msgs[8]{};
  for (size_t i = 0; i < count; ++i) {
    cfs[i].can_id = frames[i].can_id & CAN_SFF_MASK;
    cfs[i].can_dlc = frames[i].dlc;
    std::memcpy(cfs[i].data, frames[i].data.data(), frames[i].dlc);
    iovs[i].iov_base = &cfs[i];
    iovs[i].iov_len = sizeof(struct can_frame);
    msgs[i].msg_hdr.msg_iov = &iovs[i];
    msgs[i].msg_hdr.msg_iovlen = 1;
  }
  size_t sent_count = 0;
  while (sent_count < count) {
    const int sent = ::sendmmsg(
      sock_, msgs + sent_count, static_cast<unsigned int>(count - sent_count), MSG_DONTWAIT);
    if (sent < 0) {
      const int err = errno;
      if (err == ENOBUFS || err == EAGAIN || err == EWOULDBLOCK || err == ENETDOWN) {
        return;
      }
      throw std::runtime_error(
        "Failed to send CAN batch on '" + channel_ + "': " + std::string(std::strerror(err)));
    }
    if (sent == 0) {
      return;
    }
    sent_count += static_cast<size_t>(sent);
  }
}

void SocketCanArm::disable_gripper() {
  uint8_t status_code = 0;
  {
    std::lock_guard<std::mutex> rlock(rx_mu_);
    if (has_grip_ && grip_angle_mode_) {
      status_code = 4;
    }
  }
  std::lock_guard<std::mutex> glock(grip_mu_);
  grip_interp_active_ = false;
  grip_cmd_pos_.reset();
  send_frame(encode_gripper_ctrl(0, 0, status_code, 0));
}

void SocketCanArm::calibrate_effector() {
  {
    std::lock_guard<std::mutex> lock(rx_mu_);
    has_resp_476_ = false;
    resp_476_index_ = 0;
    resp_476_zero_ok_ = 0;
  }
  send_frame(encode_gripper_ctrl(0, 0, 0, 0xAE));
  const double deadline = mono_now() + 1.0;
  std::unique_lock<std::mutex> lock(rx_mu_);
  while (mono_now() < deadline) {
    if (has_resp_476_ && resp_476_index_ == 0x75) {
      if (resp_476_zero_ok_ == 1) {
        has_resp_476_ = false;
        return;
      }
      break;
    }
    rx_cv_.wait_for(lock, std::chrono::milliseconds(5));
  }
  throw std::runtime_error(
    "Device did not acknowledge effector zero calibration; "
    "result unknown, inspect feedback before any further operation");
}

void SocketCanArm::write_power_once(bool enabled) {
  RawCanFrame batch[6];
  for (int i = 0; i < 6; ++i) {
    batch[i] = encode_motor_enable(static_cast<uint8_t>(i + 1), enabled);
  }
  send_frames_batch(batch, 6);
  disable_gripper();
}

void SocketCanArm::power(bool enabled, double /*width*/) {
  leave_drag();
  write_power_once(enabled);
  const double deadline = mono_now() + 1.5;
  double next_retry = mono_now() + 0.12;
  while (true) {
    const Feedback s = snapshot();
    bool done = false;
    if (enabled) {
      done = s.enabled[0] && s.enabled[1] && s.enabled[2] &&
             s.enabled[3] && s.enabled[4] && s.enabled[5];
    } else {
      done = !(s.enabled[0] || s.enabled[1] || s.enabled[2] ||
               s.enabled[3] || s.enabled[4] || s.enabled[5] || s.enabled[6]);
    }
    if (done) {
      break;
    }
    const double now = mono_now();
    if (now >= deadline) {
      break;
    }
    if (now >= next_retry) {
      write_power_once(enabled);
      next_retry = now + 0.12;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  powered_.store(enabled, std::memory_order_relaxed);
  if (!enabled) {
    std::lock_guard<std::mutex> tlock(tx_mu_);
    current_mode_ = "standby";
    in_drag_mode_.store(false, std::memory_order_relaxed);
    mit_output_.reset();
  }
}

void SocketCanArm::enter_drag() {
  mode("drag");
  powered_.store(false, std::memory_order_relaxed);
  std::lock_guard<std::mutex> tlock(tx_mu_);
  mit_output_.reset();
}

void SocketCanArm::leave_drag() {
  {
    std::lock_guard<std::mutex> tlock(tx_mu_);
    if (current_mode_ != "drag") {
      return;
    }
    send_frame(encode_leader_follower_config(0xFC));
    current_mode_ = "standby";
    in_drag_mode_.store(false, std::memory_order_relaxed);
  }
  const double deadline = mono_now() + 1.0;
  while (!fresh(snapshot().age)) {
    if (mono_now() >= deadline) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

void SocketCanArm::set_motion_mode_code(const std::string & motion_mode) {
  if (motion_mode == "p") {
    cached_move_mode_ = 0;
    cached_mit_mode_ = 0x00;
  } else if (motion_mode == "j") {
    cached_move_mode_ = 1;
    cached_mit_mode_ = 0x00;
  } else if (motion_mode == "l") {
    cached_move_mode_ = 2;
    cached_mit_mode_ = 0x00;
  } else if (motion_mode == "mit") {
    cached_move_mode_ = 6;
    cached_mit_mode_ = 0xAD;
  } else {
    throw std::invalid_argument("Unsupported motion mode: " + motion_mode);
  }
  send_frame(encode_mode_ctrl(1, cached_move_mode_, cached_speed_percent_, cached_mit_mode_));
}

void SocketCanArm::write_mode_impl(const std::string & name) {
  if (name == "drag") {
    send_frame(encode_param_enquiry_and_config(0, 0, 0, 0xAE, 0x02));
    disable_gripper();
    for (int retry = 0; retry < 3; ++retry) {
      send_frame(encode_leader_follower_config(0xFA, 0, 0, 0x10));
      send_frame(encode_leader_move_to_home(0));
      const double wait_until = mono_now() + 0.04;
      bool exited_can_ctrl = false;
      while (mono_now() < wait_until) {
        const Feedback s = snapshot();
        if (!s.status.has_value() || s.age[4] > 0.025 || s.status->ctrl_mode == 2) {
          exited_can_ctrl = true;
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      if (exited_can_ctrl) {
        break;
      }
    }
    disable_gripper();
    return;
  }
  leave_drag();
  std::unique_lock<std::mutex> tlock(tx_mu_);
  if (name == "hold") {
    send_frame(encode_motion_ctrl(0, 0, 2));
  } else if (name == "standby") {
    send_frame(encode_mode_ctrl(0, 0, 50, 0));
  } else {
    std::string code;
    if (name == "joint") code = "j";
    else if (name == "point") code = "p";
    else if (name == "linear") code = "l";
    else if (name == "mit") code = "mit";
    else throw std::invalid_argument("Unknown mode: " + name);
    set_motion_mode_code(code);
    if (current_mode_ == "mit" && name != "mit") {
      tlock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      tlock.lock();
      {
        std::lock_guard<std::mutex> mlock(mode_tx_mu_);
        last_0x151_tx_mono_ = -1.0;
      }
      set_motion_mode_code(code);
    }
  }
}

void SocketCanArm::mode(const std::string & name) {
  write_mode_impl(name);
  std::lock_guard<std::mutex> tlock(tx_mu_);
  current_mode_ = name;
  in_drag_mode_.store(name == "drag", std::memory_order_relaxed);
}

void SocketCanArm::joints(const Vec6 & position, uint8_t speed_percent) {
  if (speed_percent > 100) {
    throw std::invalid_argument("Percent should be between 0 and 100");
  }
  std::lock_guard<std::mutex> tlock(tx_mu_);
  const auto frames = encode_joint_move_batch(position, speed_percent, cached_mit_mode_);
  cached_speed_percent_ = speed_percent;
  cached_move_mode_ = 1;
  cached_mit_mode_ = 0x00;
  send_frames_batch(frames.data(), frames.size());
}

void SocketCanArm::cartesian(
  const Vec6 & pose, uint8_t speed_percent, const std::string & move_mode)
{
  if (speed_percent > 100) {
    throw std::invalid_argument("Percent should be between 0 and 100");
  }
  const bool linear = (move_mode != "point");
  std::lock_guard<std::mutex> tlock(tx_mu_);
  const auto frames = encode_cartesian_move_batch(
    pose, speed_percent, linear, cached_mit_mode_);
  cached_speed_percent_ = speed_percent;
  cached_move_mode_ = linear ? 2 : 0;
  cached_mit_mode_ = 0x00;
  send_frames_batch(frames.data(), frames.size());
}

void SocketCanArm::mit(const MitTarget & command) {
  std::lock_guard<std::mutex> tlock(tx_mu_);
  const auto frames = encode_mit_command(command);
  send_frames_batch(frames.data(), frames.size());
  mit_output_ = frames;
}

void SocketCanArm::retain_mit() {
  std::lock_guard<std::mutex> tlock(tx_mu_);
  if (!mit_output_.has_value()) {
    throw std::runtime_error("No transmitted MIT output available to retain");
  }
  send_frames_batch(mit_output_->data(), mit_output_->size());
}

void SocketCanArm::grip_interp_loop() {
  while (true) {
    std::unique_lock<std::mutex> lock(grip_mu_);
    while (!grip_stop_ && !grip_interp_active_) {
      grip_cv_.wait(lock);
    }
    if (grip_stop_) {
      return;
    }
    double now = mono_now();
    while (!grip_stop_ && grip_interp_active_) {
      const double wait_sec = 0.005 - (now - grip_last_tx_time_);
      if (wait_sec <= 0.0005) {
        break;
      }
      grip_cv_.wait_for(lock, std::chrono::duration<double>(wait_sec));
      now = mono_now();
    }
    if (grip_stop_ || !grip_interp_active_) {
      continue;
    }
    const double alpha = std::min(1.0, (std::max(0.0, now - grip_interp_t0_) + 0.005) / 0.020);
    const double w_sub = grip_start_pos_ + alpha * (grip_target_pos_ - grip_start_pos_);
    grip_cmd_pos_ = w_sub;
    grip_last_tx_time_ = now;
    if (alpha >= 1.0) {
      grip_interp_active_ = false;
    }
    const int32_t value_um = static_cast<int32_t>(std::lrint(w_sub * 1e6));
    const uint16_t force_mn = static_cast<uint16_t>(
      std::clamp<long>(std::lrint(grip_target_effort_ * 1e3), 0, 65535));
    try {
      send_frame(encode_gripper_ctrl(value_um, force_mn, 1, 0));
    } catch (...) {
    }
  }
}

void SocketCanArm::grip(double width, double effort, bool interpolate) {
  if (effort <= 0.0) {
    disable_gripper();
    return;
  }
  double hist_last = width;
  bool has_hist = false;
  {
    std::lock_guard<std::mutex> rlock(rx_mu_);
    if (!grip_history_.empty()) {
      hist_last = grip_history_.back().second;
      has_hist = true;
    }
  }
  std::lock_guard<std::mutex> lock(grip_mu_);
  const auto send_grip = [this](double w, double eff) {
    const int32_t value_um = static_cast<int32_t>(std::lrint(w * 1e6));
    const uint16_t force_mn = static_cast<uint16_t>(
      std::clamp<long>(std::lrint(eff * 1e3), 0, 65535));
    send_frame(encode_gripper_ctrl(value_um, force_mn, 1, 0));
  };
  if (!interpolate) {
    grip_interp_active_ = false;
    grip_cmd_pos_.reset();
    send_grip(width, effort);
    return;
  }
  const double now = mono_now();
  const double dt = now - grip_last_cmd_time_;
  grip_last_cmd_time_ = now;
  const double start_w = (!grip_cmd_pos_.has_value() || dt > 0.080)
    ? (has_hist ? hist_last : width)
    : *grip_cmd_pos_;
  grip_target_pos_ = width;
  grip_target_effort_ = effort;
  if (std::abs(width - start_w) <= 0.0002 || (0.0 < dt && dt <= 0.007)) {
    grip_cmd_pos_ = width;
    grip_interp_active_ = false;
    grip_last_tx_time_ = now;
    send_grip(width, effort);
  } else {
    grip_start_pos_ = start_w;
    grip_interp_t0_ = now;
    grip_interp_active_ = true;
    if (now - grip_last_tx_time_ >= 0.004) {
      const double w_step = start_w + 0.25 * (width - start_w);
      grip_cmd_pos_ = w_step;
      grip_last_tx_time_ = now;
      send_grip(w_step, effort);
    }
    grip_cv_.notify_one();
  }
}

void SocketCanArm::close() {
  if (closed_) {
    return;
  }
  closed_ = true;
  try {
    bool need_drag = false;
    {
      std::lock_guard<std::mutex> tlock(tx_mu_);
      need_drag = (current_mode_ != "drag") &&
                  (current_mode_ == "mit" || powered_.load(std::memory_order_relaxed));
    }
    if (need_drag && sock_ >= 0) {
      enter_drag();
    }
  } catch (...) {
  }
  disconnect_internal();
}

void SocketCanArm::disconnect_internal() noexcept {
  {
    std::lock_guard<std::mutex> glock(grip_mu_);
    grip_stop_ = true;
    grip_interp_active_ = false;
  }
  grip_cv_.notify_all();
  if (grip_thread_.joinable()) {
    grip_thread_.join();
  }

  stop_rx_.store(true, std::memory_order_relaxed);
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  if (sock_ >= 0) {
    ::close(sock_);
    sock_ = -1;
  }
}

}  // namespace piper_x_driver
