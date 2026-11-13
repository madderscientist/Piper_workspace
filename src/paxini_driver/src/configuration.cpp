#include "paxini_driver/configuration.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <vector>

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

std::filesystem::path default_config_path() {
  try {
    const auto share = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("paxini_driver"));
    return share / "config" / "px6d.yaml";
  } catch (const std::exception &) {
    return std::filesystem::path("config") / "px6d.yaml";
  }
}

SensorCalibrationConfig resolve_sensor_calibration(
  const std::string & arm,
  const std::filesystem::path & config_file)
{
  // 标定程序写盘采用临时文件 + rename 原子替换，此处直接读取即可拿到完整一致的 YAML
  const std::filesystem::path path = config_file.empty() ? default_config_path() : config_file;
  const YAML::Node root = YAML::LoadFile(path.string());
  const YAML::Node defaults = root["defaults"];
  const YAML::Node sensors = root["sensors"];
  if (!sensors || !sensors.IsMap()) {
    throw std::invalid_argument("No 'sensors' map in " + path.string());
  }

  const std::string selector = trim_lower(arm);
  for (const auto & kv : sensors) {
    const std::string key = kv.first.as<std::string>();
    const YAML::Node entry = kv.second;

    std::unordered_set<std::string> aliases{trim_lower(key)};
    const YAML::Node alias_node =
      (entry && entry["aliases"]) ? entry["aliases"]
                                  : ((defaults && defaults["aliases"]) ? defaults["aliases"]
                                                                       : YAML::Node{});
    if (alias_node && alias_node.IsSequence()) {
      for (const auto & a : alias_node) {
        aliases.insert(trim_lower(a.as<std::string>()));
      }
    }

    if (!selector.empty() && selector != "all" && aliases.find(selector) == aliases.end()) {
      continue;
    }

    auto get_field = [&](const char * field) -> YAML::Node {
      if (entry && entry[field]) {
        return entry[field];
      }
      if (defaults && defaults[field]) {
        return defaults[field];
      }
      return YAML::Node{};
    };

    SensorCalibrationConfig out;
    out.arm_id = key;

    if (const YAML::Node bias = get_field("tare_offset"); bias && bias.IsSequence()) {
      if (bias.size() != 6) {
        throw std::invalid_argument("配置中的标定参数格式无效");
      }
      for (size_t i = 0; i < 6; ++i) {
        out.tare_offset[i] = bias[i].as<double>();
      }
      out.has_tare_offset = true;
    }

    if (const YAML::Node mass = get_field("tool_mass_kg"); mass && mass.IsScalar()) {
      out.tool_mass_kg = mass.as<double>();
      if (!std::isfinite(out.tool_mass_kg) || out.tool_mass_kg < 0.0) {
        throw std::invalid_argument("配置中的标定参数格式无效");
      }
      out.has_tool_mass = true;
    }

    if (const YAML::Node com = get_field("tool_com_m"); com && com.IsSequence()) {
      if (com.size() != 3) {
        throw std::invalid_argument("配置中的标定参数格式无效");
      }
      for (size_t i = 0; i < 3; ++i) {
        out.tool_com_m[i] = com[i].as<double>();
      }
      out.has_tool_com = true;
    }

    if (const YAML::Node qual = get_field("calibration_quality"); qual && qual.IsMap()) {
      out.calibration_quality_json = yaml_node_to_json(qual);
    }

    return out;
  }

  throw std::invalid_argument("Unknown sensor arm selector '" + arm + "'");
}

}  // namespace paxini_driver
