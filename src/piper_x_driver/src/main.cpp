#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <exception>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "piper_x_driver/node.hpp"

namespace {

std::atomic<bool> g_shutdown_requested{false};

void handle_stop_signal(int /*signum*/) {
  g_shutdown_requested.store(true, std::memory_order_relaxed);
}

}  // namespace

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, handle_stop_signal);
  std::signal(SIGTERM, handle_stop_signal);

  std::shared_ptr<piper_x_driver::DriverNode> node;
  int tty_fd = -1;
  try {
    node = std::make_shared<piper_x_driver::DriverNode>(
      rclcpp::NodeOptions(), nullptr, /*create_default_timer=*/false);
  } catch (const std::exception & exc) {
    RCLCPP_FATAL(rclcpp::get_logger("piper_x_driver"), "%s", exc.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  if (node->is_first_fleet_arm()) {
    tty_fd = ::open("/dev/tty", O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  }

  const double period = 1.0 / node->rate_hz();
  const uint64_t telemetry_stride = std::max<uint64_t>(
    1, static_cast<uint64_t>(std::llround(node->rate_hz() / node->telemetry_hz())));
  uint64_t step = 0;
  double next_tick = piper_x_driver::DriverNode::steady_now();

  while (rclcpp::ok() && !g_shutdown_requested.load(std::memory_order_relaxed)) {
    executor.spin_some();

    if (tty_fd >= 0 && (step % 2 == 0)) {
      struct pollfd pfd{tty_fd, POLLIN, 0};
      if (::poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
        char buf[256];
        const ssize_t n = ::read(tty_fd, buf, sizeof(buf));
        for (ssize_t i = 0; i < n; ++i) {
          if (buf[i] == '\n' || buf[i] == '\r') {
            node->trigger_teaching_retry();
            break;
          }
        }
      }
    }

    if (step % telemetry_stride == 0) {
      node->publish_step();
    } else {
      node->fast_control_step();
    }
    ++step;

    next_tick += period;
    const double now = piper_x_driver::DriverNode::steady_now();
    const double sleep_s = next_tick - now;
    if (sleep_s > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(sleep_s));
    } else if (now - next_tick > 0.02) {
      next_tick = now;
    }
  }

  if (tty_fd >= 0) {
    ::close(tty_fd);
  }
  executor.remove_node(node);
  node->shutdown_driver();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
