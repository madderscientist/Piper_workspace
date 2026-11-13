#pragma once

#include <array>
#include <optional>

#include "paxini_driver/protocol.hpp"

namespace paxini_driver {

inline constexpr double GRAVITY_ACCEL = 9.81;

// 使用与 PiPER-X SDK 一致的 link6 正运动学计算基座系下传感器测量轴的旋转矩阵 R_BS(q)
std::array<std::array<double, 3>, 3> sensor_rotation_in_base(
  const std::array<double, 6> & q) noexcept;

// 计算传感器坐标系下的单位向下重力方向向量 g_hat_S = R_BS(q)^T * [0, 0, -1]^T
std::array<double, 3> gravity_unit_in_sensor(
  const std::array<double, 6> & q) noexcept;

// 计算传感器下游负载（夹爪/工装）在当前关节角 q 下对传感器产生的六维重力与重力矩
std::array<double, 6> tool_gravity_wrench(
  const std::array<double, 6> & q,
  double tool_mass_kg,
  const std::array<double, 3> & tool_com_m) noexcept;

// 扣除传感器常数零偏 tare_offset；当关节角 q 与工具质量有效时，同时扣除姿态相关的工具重力项
WrenchSample compensate_wrench(
  const WrenchSample & raw,
  const std::array<double, 6> & tare_offset,
  const std::optional<std::array<double, 6>> & q = std::nullopt,
  double tool_mass_kg = 0.0,
  const std::array<double, 3> & tool_com_m = {0.0, 0.0, 0.0}) noexcept;

// 使用预先由 FK 算好的单位重力方向执行零偏与工具重力补偿（供 1 kHz 采集热路径直接复用）
WrenchSample compensate_wrench_with_gravity_dir(
  const WrenchSample & raw,
  const std::array<double, 6> & tare_offset,
  const std::optional<std::array<double, 3>> & gravity_dir,
  double tool_mass_kg,
  const std::array<double, 3> & tool_com_m) noexcept;

}  // namespace paxini_driver
