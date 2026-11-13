#include <csignal>
#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "paxini_driver/node.hpp"

namespace {

void handle_stop_signal(int /*signum*/) {
  rclcpp::shutdown();
}

}  // namespace

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  // 同时捕获 SIGINT 与 SIGTERM，确保 Ctrl+C 或 launch/服务停止时都能执行下发 0x04 停流收尾
  std::signal(SIGINT, handle_stop_signal);
  std::signal(SIGTERM, handle_stop_signal);

  std::shared_ptr<paxini_driver::PaxiniDriverNode> node;
  try {
    node = std::make_shared<paxini_driver::PaxiniDriverNode>();
  } catch (const std::exception & exc) {
    RCLCPP_FATAL(rclcpp::get_logger("paxini_driver"), "%s", exc.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node->shutdown_driver();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
