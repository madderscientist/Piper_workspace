#pragma once

#include <csignal>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#ifndef HEAD_CAMERA_HAS_K4A
#define HEAD_CAMERA_HAS_K4A 1
#endif

#if HEAD_CAMERA_HAS_K4A
#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <k4arecord/record.h>
#endif

namespace head_camera
{
namespace fs = std::filesystem;

#if HEAD_CAMERA_HAS_K4A
using Image = std::unique_ptr<std::remove_pointer_t<k4a_image_t>, decltype(&k4a_image_release)>;
using Capture = std::unique_ptr<std::remove_pointer_t<k4a_capture_t>, decltype(&k4a_capture_release)>;
using Device = std::unique_ptr<std::remove_pointer_t<k4a_device_t>, decltype(&k4a_device_close)>;
using Playback = std::unique_ptr<std::remove_pointer_t<k4a_playback_t>, decltype(&k4a_playback_close)>;
using Recording = std::unique_ptr<std::remove_pointer_t<k4a_record_t>, decltype(&k4a_record_close)>;
#endif

struct CameraSpec
{
  std::string name;
  std::string serial;
  std::string video_device;
};

struct UvcDeviceInfo
{
  int number = 0;
  std::string path;
  std::string card;
  std::string serial;
  std::string bus_info;
  bool is_kinect = false;
};

struct Options
{
  std::string command;
  uint32_t device = 0;
  bool device_explicit = false;
  std::string camera;
  std::string camera_name;
  std::string serial;
  std::string video_device;
  std::string depth_mode = "WFOV_2X2BINNED";
  int fps = 30;
  std::string color = "OFF";
  fs::path output = "head_camera_data";
  fs::path input;
  fs::path intrinsic_file;
  int squares_x = 12;
  int squares_y = 9;
  double square_size = 0.015;
  double marker_size = 0.01125;
  std::string dictionary = "DICT_5X5_100";
  uint64_t count = 1;
  double interval = 0.0;
  double duration = 0.0;
  bool preview = false;
  std::string host = "127.0.0.1";
  uint16_t port = 8310;
  std::vector<CameraSpec> cameras;
};

fs::path default_config_path();
std::vector<CameraSpec> load_camera_specs(const fs::path & path);
Options parse_options(const std::vector<std::string> & args);
std::pair<int, int> color_dimensions(const std::string & color);
std::vector<UvcDeviceInfo> discover_uvc_devices();
std::vector<Options> resolve_serve_targets(const Options & options);
std::string json_string(const std::string & value);
void write_text(const fs::path & path, const std::string & text);
std::vector<uint8_t> depth_preview_jpeg(
  const uint16_t * data, int width, int height, size_t stride_bytes);
fs::path create_session(const fs::path & root, const std::string & command);

#if HEAD_CAMERA_HAS_K4A
k4a_device_configuration_t device_configuration(const Options & options);
std::string depth_mode_name(k4a_depth_mode_t mode);
int frame_rate(k4a_fps_t fps);
void check(k4a_result_t result, const std::string & operation);
void save_depth(const fs::path & path, k4a_image_t image, bool preview);
std::vector<uint8_t> depth_preview_jpeg(k4a_image_t image);
#endif

struct Result
{
  fs::path directory;
  uint64_t frames;
  bool interrupted;
};

void list_devices();
Result acquire(const Options & options, const volatile std::sig_atomic_t & stopped);
Result export_recording(const Options & options, const volatile std::sig_atomic_t & stopped);

}  // namespace head_camera
