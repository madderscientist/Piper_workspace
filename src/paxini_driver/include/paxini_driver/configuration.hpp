#pragma once

#include <array>
#include <filesystem>
#include <string>

namespace paxini_driver {

// 单传感器标定文件中的补偿参数与质量摘要
struct SensorCalibrationConfig {
  std::array<double, 6> tare_offset{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool has_tare_offset{false};
  double tool_mass_kg{0.0};
  bool has_tool_mass{false};
  std::array<double, 3> tool_com_m{0.0, 0.0, 0.0};
  bool has_tool_com{false};
  std::string calibration_quality_json{"{}"};
};

// 读取单个传感器的标定文件；文件缺失或格式无效时抛出异常，由调用方按未标定处理
SensorCalibrationConfig load_sensor_calibration(const std::filesystem::path & file);

std::string json_escape(const std::string & s);

}  // namespace paxini_driver
