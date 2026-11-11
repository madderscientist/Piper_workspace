#include "head_camera/head_camera.hpp"

#include <charconv>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace head_camera
{
namespace
{
uint64_t unsigned_value(const std::string & value, const std::string & name)
{
  uint64_t result = 0;
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  if (parsed.ec != std::errc() || parsed.ptr != value.data() + value.size()) {
    throw std::invalid_argument(name + " must be a nonnegative integer");
  }
  return result;
}

double seconds_value(const std::string & value, const std::string & name)
{
  size_t used = 0;
  double result;
  try {
    result = std::stod(value, &used);
  } catch (const std::invalid_argument &) {
    throw std::invalid_argument(name + " must be a finite nonnegative number of seconds");
  } catch (const std::out_of_range &) {
    throw std::invalid_argument(name + " is out of range");
  }
  if (used != value.size() || !std::isfinite(result) || result < 0.0) {
    throw std::invalid_argument(name + " must be a finite nonnegative number of seconds");
  }
  return result;
}

void set_option(Options & options, const std::string & key, const std::string & value)
{
  if (key == "device") {
    const auto device = unsigned_value(value, key);
    if (device > std::numeric_limits<uint32_t>::max()) {
      throw std::invalid_argument("device index is out of range");
    }
    options.device = static_cast<uint32_t>(device);
  } else if (key == "camera") {
    options.camera = value;
  } else if (key == "serial") {
    options.serial = value;
  } else if (key == "video-device") {
    options.video_device = value;
  } else if (key == "depth-mode") {
    options.depth_mode = value;
  } else if (key == "fps") {
    const auto fps = unsigned_value(value, key);
    if (fps != 5 && fps != 15 && fps != 30) {
      throw std::invalid_argument("Azure Kinect supports only 5, 15 or 30 fps");
    }
    options.fps = static_cast<int>(fps);
  } else if (key == "color") {
    options.color = value;
  } else if (key == "output") {
    options.output = value;
  } else if (key == "input") {
    options.input = value;
  } else if (key == "intrinsic") {
    options.intrinsic_file = value;
  } else if (key == "squares-x") {
    const auto v = unsigned_value(value, key);
    if (v < 3 || v > 100) {
      throw std::invalid_argument("squares-x must be between 3 and 100");
    }
    options.squares_x = static_cast<int>(v);
  } else if (key == "squares-y") {
    const auto v = unsigned_value(value, key);
    if (v < 3 || v > 100) {
      throw std::invalid_argument("squares-y must be between 3 and 100");
    }
    options.squares_y = static_cast<int>(v);
  } else if (key == "square-size") {
    options.square_size = seconds_value(value, key);
    if (options.square_size <= 0.0) {
      throw std::invalid_argument("square-size must be positive");
    }
  } else if (key == "marker-size") {
    options.marker_size = seconds_value(value, key);
    if (options.marker_size <= 0.0) {
      throw std::invalid_argument("marker-size must be positive");
    }
  } else if (key == "dictionary") {
    options.dictionary = value;
  } else if (key == "count") {
    options.count = unsigned_value(value, key);
    if (options.count == 0) {
      throw std::invalid_argument("count must be greater than zero");
    }
  } else if (key == "interval") {
    options.interval = seconds_value(value, key);
  } else if (key == "duration") {
    options.duration = seconds_value(value, key);
  } else if (key == "host") {
    options.host = value;
    if (value.empty()) {
      throw std::invalid_argument("host must not be empty");
    }
  } else if (key == "port") {
    const auto port = unsigned_value(value, key);
    if (port > std::numeric_limits<uint16_t>::max()) {
      throw std::invalid_argument("port is out of range");
    }
    options.port = static_cast<uint16_t>(port);
  } else {
    throw std::invalid_argument("unknown option: " + key);
  }
}

std::vector<CameraSpec> parse_cameras_node(const YAML::Node & node)
{
  std::vector<CameraSpec> result;
  if (!node || !node.IsMap()) {
    return result;
  }
  for (const auto & item : node) {
    CameraSpec spec;
    spec.name = item.first.as<std::string>();
    if (item.second.IsScalar()) {
      spec.serial = item.second.as<std::string>();
    } else if (item.second.IsMap()) {
      if (item.second["serial"]) spec.serial = item.second["serial"].as<std::string>();
      if (item.second["video_device"]) spec.video_device = item.second["video_device"].as<std::string>();
    }
    result.push_back(std::move(spec));
  }
  return result;
}

void load_config(Options & options, const fs::path & path, bool & color_explicit)
{
  const auto config = YAML::LoadFile(path.string());
  if (!config.IsMap()) {
    throw std::invalid_argument("config must be a YAML mapping: " + path.string());
  }
  for (const auto & item : config) {
    auto key = item.first.as<std::string>();
    if (key == "cameras") {
      options.cameras = parse_cameras_node(item.second);
    } else if (key == "preview") {
      options.preview = item.second.as<bool>();
    } else {
      if (key == "depth_mode") {
        key = "depth-mode";
      } else if (key == "video_device") {
        key = "video-device";
      }
      if (key == "color") {
        color_explicit = true;
      }
      set_option(options, key, item.second.as<std::string>());
    }
  }
}
}  // namespace

fs::path default_config_path()
{
  try {
    const fs::path share(ament_index_cpp::get_package_share_directory("head_camera"));
    const fs::path cfg = share / "config" / "head_camera.yaml";
    if (fs::exists(cfg)) {
      return fs::canonical(cfg);
    }
  } catch (...) {
  }
  return fs::absolute("src/head_camera/config/head_camera.yaml");
}

std::vector<CameraSpec> load_camera_specs(const fs::path & path)
{
  if (!fs::exists(path)) {
    return {};
  }
  try {
    const auto root = YAML::LoadFile(path.string());
    if (root && root.IsMap() && root["cameras"]) {
      return parse_cameras_node(root["cameras"]);
    }
  } catch (...) {
  }
  return {};
}

std::pair<int, int> color_dimensions(const std::string & color)
{
  if (color == "OFF") {
    return {0, 0};
  }
  if (color == "720p") {
    return {1280, 720};
  }
  if (color == "1080p") {
    return {1920, 1080};
  }
  throw std::invalid_argument("color must be OFF, 720p or 1080p");
}

Options parse_options(const std::vector<std::string> & args)
{
  if (args.empty()) {
    throw std::invalid_argument("missing command; use --help");
  }
  Options options;
  options.command = args.front();
  if (options.command != "list" && options.command != "capture" &&
    options.command != "record" && options.command != "export" &&
    options.command != "serve" && options.command != "calibrate")
  {
    throw std::invalid_argument("unknown command: " + options.command);
  }
  if (options.command == "serve") {
    options.depth_mode = "OFF";
    options.color = "1080p";
  }
  std::set<std::string> allowed;
  if (options.command == "capture" || options.command == "record" || options.command == "serve") {
    allowed = {"--config", "--device", "--camera", "--serial", "--depth-mode", "--fps", "--color", "--output"};
    if (options.command == "capture") {
      allowed.insert({"--video-device", "--count", "--interval", "--preview"});
    } else if (options.command == "record") {
      allowed.insert("--duration");
    } else {
      allowed.insert({"--video-device", "--host", "--port", "--intrinsic"});
    }
  } else if (options.command == "export") {
    allowed = {"--input", "--output", "--preview"};
  } else if (options.command == "calibrate") {
    allowed = {
      "--input", "--intrinsic", "--camera", "--serial",
      "--squares-x", "--squares-y", "--square-size", "--marker-size", "--dictionary"
    };
  }
  std::vector<std::pair<std::string, std::string>> values;
  for (size_t i = 1; i < args.size(); ++i) {
    const auto & flag = args[i];
    if (allowed.count(flag) == 0) {
      throw std::invalid_argument("unsupported option for " + options.command + ": " + flag);
    }
    if (flag == "--preview") {
      values.emplace_back("preview", "true");
      continue;
    }
    if (i + 1 == args.size() || args[i + 1].rfind("--", 0) == 0) {
      throw std::invalid_argument("missing value for " + flag);
    }
    values.emplace_back(flag.substr(2), args[++i]);
  }
  bool color_explicit = false;
  bool config_loaded = false;
  for (const auto & value : values) {
    if (value.first == "config") {
      load_config(options, value.second, color_explicit);
      config_loaded = true;
    }
  }
  if (!config_loaded) {
    options.cameras = load_camera_specs(default_config_path());
  }
  for (const auto & value : values) {
    if (value.first == "preview") {
      options.preview = true;
    } else if (value.first != "config") {
      if (value.first == "color") {
        color_explicit = true;
      }
      if (value.first == "device" || value.first == "video-device" || value.first == "serial") {
        options.device_explicit = true;
      }
      set_option(options, value.first, value.second);
    }
  }
  if (options.output.empty()) {
    throw std::invalid_argument("output directory must not be empty");
  }
  if (options.command == "export" && options.input.empty()) {
    throw std::invalid_argument("export requires --input FILE.mkv");
  }
  if (options.command == "calibrate") {
    if (options.input.empty()) {
      throw std::invalid_argument("calibrate requires --input DIRECTORY");
    }
    if (options.marker_size >= options.square_size) {
      throw std::invalid_argument("marker-size must be smaller than square-size");
    }
  }
  if (options.command == "capture" || options.command == "record" || options.command == "serve") {
    if (options.fps != 5 && options.fps != 15 && options.fps != 30) {
      throw std::invalid_argument("Azure Kinect supports only 5, 15 or 30 fps");
    }
    color_dimensions(options.color);
    if (options.depth_mode == "OFF") {
      if (options.command == "record") {
        throw std::invalid_argument("record requires an active depth mode");
      }
      if (!color_explicit && options.color == "OFF") {
        options.color = "1080p";
      }
      if (options.color == "OFF") {
        throw std::invalid_argument("color cannot be OFF when depth-mode is OFF");
      }
    } else {
#if HEAD_CAMERA_HAS_K4A
      device_configuration(options);
#else
      throw std::runtime_error("depth modes require Azure Kinect SDK; use --depth-mode OFF");
#endif
    }
  }
  return options;
}

#if HEAD_CAMERA_HAS_K4A
k4a_device_configuration_t device_configuration(const Options & options)
{
  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  if (options.depth_mode == "NFOV_UNBINNED") {
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
  } else if (options.depth_mode == "NFOV_2X2BINNED") {
    config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
  } else if (options.depth_mode == "WFOV_UNBINNED") {
    config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
  } else if (options.depth_mode == "WFOV_2X2BINNED") {
    config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
  } else {
    throw std::invalid_argument("unknown depth mode: " + options.depth_mode);
  }
  if (options.fps != 5 && options.fps != 15 && options.fps != 30) {
    throw std::invalid_argument("Azure Kinect supports only 5, 15 or 30 fps");
  }
  if (config.depth_mode == K4A_DEPTH_MODE_WFOV_UNBINNED && options.fps == 30) {
    throw std::invalid_argument("WFOV_UNBINNED hardware supports only 5 or 15 fps");
  }
  config.camera_fps = options.fps == 30 ? K4A_FRAMES_PER_SECOND_30 :
    options.fps == 15 ? K4A_FRAMES_PER_SECOND_15 : K4A_FRAMES_PER_SECOND_5;
  config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
  if (options.color == "720p") {
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.synchronized_images_only = true;
  } else if (options.color == "1080p") {
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.synchronized_images_only = true;
  } else if (options.color != "OFF") {
    throw std::invalid_argument("color must be OFF, 720p or 1080p");
  }
  return config;
}

std::string depth_mode_name(k4a_depth_mode_t mode)
{
  switch (mode) {
    case K4A_DEPTH_MODE_NFOV_UNBINNED: return "NFOV_UNBINNED";
    case K4A_DEPTH_MODE_NFOV_2X2BINNED: return "NFOV_2X2BINNED";
    case K4A_DEPTH_MODE_WFOV_UNBINNED: return "WFOV_UNBINNED";
    case K4A_DEPTH_MODE_WFOV_2X2BINNED: return "WFOV_2X2BINNED";
    default: throw std::invalid_argument("recording does not contain an active depth mode");
  }
}

int frame_rate(k4a_fps_t fps)
{
  switch (fps) {
    case K4A_FRAMES_PER_SECOND_5: return 5;
    case K4A_FRAMES_PER_SECOND_15: return 15;
    case K4A_FRAMES_PER_SECOND_30: return 30;
    default: throw std::invalid_argument("invalid SDK frame rate");
  }
}

void check(k4a_result_t result, const std::string & operation)
{
  if (result != K4A_RESULT_SUCCEEDED) {
    throw std::runtime_error(operation + " failed");
  }
}
#endif
}  // namespace head_camera
