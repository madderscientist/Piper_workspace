#include "paxini_driver/configuration.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <stdexcept>

namespace paxini_driver {
namespace {

// 将 YAML 标定质量字段无损转换为 JSON 片段，供 sensor_status 话题直接内嵌输出
std::string yaml_node_to_json(const YAML::Node & node) {
  if (!node || node.IsNull()) {
    return "null";
  }
  if (node.IsScalar()) {
    const std::string raw = node.Scalar();
    if (node.Tag() != "!") {
      if (raw == "true" || raw == "True" || raw == "TRUE") {
        return "true";
      }
      if (raw == "false" || raw == "False" || raw == "FALSE") {
        return "false";
      }
      char * end_i = nullptr;
      const long long iv = std::strtoll(raw.c_str(), &end_i, 10);
      if (end_i != raw.c_str() && *end_i == '\0') {
        return std::to_string(iv);
      }
      char * end_d = nullptr;
      const double dv = std::strtod(raw.c_str(), &end_d);
      if (end_d != raw.c_str() && *end_d == '\0' && std::isfinite(dv)) {
        std::ostringstream oss;
        oss.precision(12);
        oss << dv;
        return oss.str();
      }
    }
    return "\"" + json_escape(raw) + "\"";
  }
  if (node.IsSequence()) {
    std::ostringstream oss;
    oss << "[";
    bool first = true;
    for (const auto & item : node) {
      if (!first) {
        oss << ", ";
      }
      first = false;
      oss << yaml_node_to_json(item);
    }
    oss << "]";
    return oss.str();
  }
  if (node.IsMap()) {
    std::ostringstream oss;
    oss << "{";
    bool first = true;
    for (const auto & kv : node) {
      if (!first) {
        oss << ", ";
      }
      first = false;
      oss << "\"" << json_escape(kv.first.as<std::string>()) << "\": "
          << yaml_node_to_json(kv.second);
    }
    oss << "}";
    return oss.str();
  }
  return "{}";
}

}  // namespace

std::string json_escape(const std::string & s) {
  std::string out;
  out.reserve(s.size() + 4);
  for (unsigned char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          char hex[7];
          std::snprintf(hex, sizeof(hex), "\\u%04x", static_cast<unsigned int>(c));
          out += hex;
        } else {
          out.push_back(static_cast<char>(c));
        }
        break;
    }
  }
  return out;
}

SensorCalibrationConfig load_sensor_calibration(const std::filesystem::path & file)
{
  // 标定程序写盘采用临时文件 + rename 原子替换，此处直接读取即可拿到完整一致的 YAML
  const YAML::Node root = YAML::LoadFile(file.string());
  SensorCalibrationConfig out;
  if (const YAML::Node bias = root["tare_offset"]; bias && bias.IsSequence()) {
    if (bias.size() != 6) {
      throw std::invalid_argument("配置中的标定参数格式无效");
    }
    for (size_t i = 0; i < 6; ++i) {
      out.tare_offset[i] = bias[i].as<double>();
    }
    out.has_tare_offset = true;
  }
  if (const YAML::Node mass = root["tool_mass_kg"]; mass && mass.IsScalar()) {
    out.tool_mass_kg = mass.as<double>();
    if (!std::isfinite(out.tool_mass_kg) || out.tool_mass_kg < 0.0) {
      throw std::invalid_argument("配置中的标定参数格式无效");
    }
    out.has_tool_mass = true;
  }
  if (const YAML::Node com = root["tool_com_m"]; com && com.IsSequence()) {
    if (com.size() != 3) {
      throw std::invalid_argument("配置中的标定参数格式无效");
    }
    for (size_t i = 0; i < 3; ++i) {
      out.tool_com_m[i] = com[i].as<double>();
    }
    out.has_tool_com = true;
  }
  if (const YAML::Node qual = root["calibration_quality"]; qual && qual.IsMap()) {
    out.calibration_quality_json = yaml_node_to_json(qual);
  }
  return out;
}

}  // namespace paxini_driver
