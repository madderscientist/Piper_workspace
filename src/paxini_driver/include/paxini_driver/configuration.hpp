#pragma once

#include <array>
#include <filesystem>
#include <string>

namespace paxini_driver {

// 从 px6d.yaml 解析出的单传感器标定补偿参数与质量摘要
struct SensorCalibrationConfig {
  std::string arm_id;
  std::array<double, 6> tare_offset{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool has_tare_offset{false};
  double tool_mass_kg{0.0};
  bool has_tool_mass{false};
  std::array<double, 3> tool_com_m{0.0, 0.0, 0.0};
  bool has_tool_com{false};
  std::string calibration_quality_json{"{}"};
};

// 获取默认配置文件路径（symlink-install 下自动指向源码中的 config/px6d.yaml）
std::filesystem::path default_config_path();

// 按臂标识或别名（如 arm_3 / arm_4 / 左手 / 右手）读取合并 defaults 后的标定配置
SensorCalibrationConfig resolve_sensor_calibration(
  const std::string & arm,
  const std::filesystem::path & config_file = {});

std::string json_escape(const std::string & s);

}  // namespace paxini_driver
