#include <chrono>
#include <exception>
#include <future>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "piper_x_driver/configuration.hpp"
#include "piper_x_driver/socketcan_arm.hpp"
#include "piper_x_interfaces/srv/enable.hpp"

int main(int argc, char ** argv) {
  using EnableSrv = piper_x_interfaces::srv::Enable;
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("piper_x_power_off");
  node->declare_parameter<std::string>(
    "config_file", piper_x_driver::default_config_path());

  int exit_code = 0;
  try {
    piper_x_driver::FleetConfig fleet(node->get_parameter("config_file").as_string());
    std::vector<std::pair<piper_x_driver::ArmConfig, rclcpp::Client<EnableSrv>::SharedPtr>> clients;
    for (const auto & arm : fleet.arms) {
      clients.emplace_back(
        arm, node->create_client<EnableSrv>(arm.namespace_str + "/enable_srv"));
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    const auto ready_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now() < ready_deadline) {
      bool all_ready = true;
      for (const auto & kv : clients) {
        if (!kv.second->service_is_ready()) {
          all_ready = false;
          break;
        }
      }
      if (all_ready) break;
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    struct PendingCall {
      piper_x_driver::ArmConfig arm;
      rclcpp::Client<EnableSrv>::SharedFuture future;
    };
    std::vector<PendingCall> pending;
    std::vector<piper_x_driver::ArmConfig> offline_arms;
    for (const auto & kv : clients) {
      if (kv.second->service_is_ready()) {
        auto req = std::make_shared<EnableSrv::Request>();
        req->enable_request = false;
        pending.push_back(PendingCall{kv.first, kv.second->async_send_request(req).future.share()});
      } else {
        offline_arms.push_back(kv.first);
      }
    }

    const auto wait_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!pending.empty() && std::chrono::steady_clock::now() < wait_deadline) {
      bool all_done = true;
      for (const auto & p : pending) {
        if (p.future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
          all_done = false;
          break;
        }
      }
      if (all_done) break;
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    for (const auto & p : pending) {
      if (p.future.wait_for(std::chrono::seconds(0)) == std::future_status::ready &&
          p.future.get()->enable_response)
      {
        RCLCPP_INFO(
          node->get_logger(), "%s (%s): powered off via running driver",
          p.arm.id.c_str(), p.arm.can_port.c_str());
      } else {
        offline_arms.push_back(p.arm);
      }
    }

    for (const auto & arm : offline_arms) {
      try {
        piper_x_driver::SocketCanArm direct_arm(arm.can_port);
        direct_arm.power(false, 0.0);
        direct_arm.close();
        RCLCPP_INFO(
          node->get_logger(), "%s (%s): powered off directly",
          arm.id.c_str(), arm.can_port.c_str());
      } catch (const std::exception & exc) {
        RCLCPP_ERROR(
          node->get_logger(), "%s (%s): power off failed: %s",
          arm.id.c_str(), arm.can_port.c_str(), exc.what());
        exit_code = 1;
      }
    }
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(node->get_logger(), "power_off failed: %s", exc.what());
    exit_code = 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
