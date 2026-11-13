#include "paxini_driver/calibration.hpp"

#include <cmath>

#include "piper_x_driver/gravity.hpp"

namespace paxini_driver {

// 复用 piper_x_driver 中与 SDK 一致的 link6 正运动学（显示 URDF 关节链与 SDK 关节反馈定义不同，不可混用）
std::array<std::array<double, 3>, 3> sensor_rotation_in_base(
  const std::array<double, 6> & q) noexcept
{
  return piper_x_driver::link6_rotation_in_base(q);
}

// 基座系向下重力方向为 [0, 0, -1]^T，左乘 R_BS^T 即取 -R_BS 的第 3 行
std::array<double, 3> gravity_unit_in_sensor(const std::array<double, 6> & q) noexcept {
  const auto r = sensor_rotation_in_base(q);
  return {-r[2][0], -r[2][1], -r[2][2]};
}

// 相对传感器原始力矩参考点计算下游工具重力 F_g = m*g*d 与重力矩 M_g = c × F_g
std::array<double, 6> tool_gravity_wrench(
  const std::array<double, 6> & q,
  double tool_mass_kg,
  const std::array<double, 3> & tool_com_m) noexcept
{
  if (tool_mass_kg <= 0.0) {
    return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }
  const auto g = gravity_unit_in_sensor(q);
  const double scale = tool_mass_kg * GRAVITY_ACCEL;
  const double fx = scale * g[0];
  const double fy = scale * g[1];
  const double fz = scale * g[2];
  const double cx = tool_com_m[0], cy = tool_com_m[1], cz = tool_com_m[2];
  return {
    fx,
    fy,
    fz,
    cy * fz - cz * fy,
    cz * fx - cx * fz,
    cx * fy - cy * fx,
  };
}

WrenchSample compensate_wrench_with_gravity_dir(
  const WrenchSample & raw,
  const std::array<double, 6> & tare_offset,
  const std::optional<std::array<double, 3>> & gravity_dir,
  double tool_mass_kg,
  const std::array<double, 3> & tool_com_m) noexcept
{
  // 常数零偏 tare_offset 不依赖 FK，始终扣除；收到关节反馈算出 gravity_dir 后一并扣除随姿态变化的工具重力
  if (gravity_dir.has_value() && tool_mass_kg > 0.0) {
    const double scale = tool_mass_kg * GRAVITY_ACCEL;
    const double fx = scale * (*gravity_dir)[0];
    const double fy = scale * (*gravity_dir)[1];
    const double fz = scale * (*gravity_dir)[2];
    const double cx = tool_com_m[0], cy = tool_com_m[1], cz = tool_com_m[2];
    return WrenchSample{
      raw.fx - (tare_offset[0] + fx),
      raw.fy - (tare_offset[1] + fy),
      raw.fz - (tare_offset[2] + fz),
      raw.mx - (tare_offset[3] + (cy * fz - cz * fy)),
      raw.my - (tare_offset[4] + (cz * fx - cx * fz)),
      raw.mz - (tare_offset[5] + (cx * fy - cy * fx)),
    };
  }
  return raw.minus(tare_offset);
}

WrenchSample compensate_wrench(
  const WrenchSample & raw,
  const std::array<double, 6> & tare_offset,
  const std::optional<std::array<double, 6>> & q,
  double tool_mass_kg,
  const std::array<double, 3> & tool_com_m) noexcept
{
  if (q.has_value() && tool_mass_kg > 0.0) {
    return compensate_wrench_with_gravity_dir(
      raw, tare_offset, gravity_unit_in_sensor(*q), tool_mass_kg, tool_com_m);
  }
  return raw.minus(tare_offset);
}

}  // namespace paxini_driver
