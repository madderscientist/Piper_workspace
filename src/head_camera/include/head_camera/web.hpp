#pragma once

#include "head_camera/calibration.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>

namespace head_camera
{
struct WebSnapshot
{
  std::string camera_name;
  std::string serial;
  std::string video_device;
  bool camera_ready = false;
  bool depth_enabled = false;
  bool color_enabled = false;
  int width = 0;
  int height = 0;
  std::string camera_error;
  std::string recording_state = "idle";
  std::string recording_error;
  std::string output;
  uint64_t frames = 0;
  uint64_t sequence = 0;
  double seconds = 0.0;
  double frame_age = -1.0;
  std::shared_ptr<const std::vector<uint8_t>> depth_jpeg;
  std::shared_ptr<const std::vector<uint8_t>> color_jpeg;
};

class WebState
{
public:
  explicit WebState(const Options & options);
  WebSnapshot snapshot(bool include_depth_jpeg = false) const;
  std::string request_recording(bool start);
  bool wants_recording() const;
  void camera_connected(int width = 0, int height = 0, const std::string & dev_path = "", const std::string & serial = "");
  void camera_failed(const std::string & error);
  void publish_color(std::vector<uint8_t> color);
  void publish_rgbd(
    std::vector<uint16_t> raw_depth, int depth_width, int depth_height,
    std::vector<uint8_t> color);
  void wait_next_frame(uint64_t after_sequence, std::function<void()> callback);
  void notify_all();
  void recording_started(const fs::path & directory);
  void recording_progress(uint64_t count);
  void recording_finished(const std::string & error = "");

  const Options options;
  CalibrationManager calib;

private:
  void fire_waiters_unlocked(std::vector<std::function<void()>> & waiters);

  mutable std::mutex mutex_;
  mutable WebSnapshot state_;
  std::vector<uint16_t> raw_depth_;
  int depth_width_ = 0;
  int depth_height_ = 0;
  mutable uint64_t depth_jpeg_seq_ = 0;
  std::vector<std::function<void()>> waiters_;
  bool wanted_ = false;
  std::chrono::steady_clock::time_point record_start_;
  std::chrono::steady_clock::time_point frame_time_;
};

struct WebResponse
{
  unsigned status;
  std::string content_type;
  std::string body;
};

WebResponse web_request(
  WebState & state, const std::string & method, const std::string & path,
  const std::string & page, const std::string & body = "");
void stream_camera(const Options & options, const std::atomic_bool & stopped, WebState & state);
void serve(
  const Options & options, const volatile std::sig_atomic_t & stopped,
  uint16_t * bound_port = nullptr,
  std::function<void(const std::atomic_bool &, WebState &)> producer = {});

}  // namespace head_camera
