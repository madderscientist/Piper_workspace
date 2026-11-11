#pragma once

#include "head_camera/head_camera.hpp"

#include <array>
#include <chrono>
#include <mutex>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>

namespace head_camera
{
struct BoardConfig
{
  int squares_x = 12;
  int squares_y = 9;
  double square_size = 0.015;
  double marker_size = 0.01125;
  std::string dictionary = "DICT_5X5_100";
};

struct CharucoDetection
{
  int width = 0;
  int height = 0;
  int marker_count = 0;
  bool collinear = true;
  double coverage = 0.0;
  std::vector<cv::Point2f> corners;
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> marker_ids;
};

struct CalibShot
{
  std::string name;
  int width = 0;
  int height = 0;
  int corners = 0;
  int markers = 0;
  double coverage = 0.0;
  double error_px = -1.0;
  std::vector<cv::Point2f> charuco_corners;
  std::vector<int> charuco_ids;
};

struct IntrinsicProfile
{
  bool available = false;
  int width = 0;
  int height = 0;
  std::array<double, 9> camera_matrix{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  std::string distortion_model = "plumb_bob";
  std::array<double, 5> distortion_coefficients{0.0, 0.0, 0.0, 0.0, 0.0};
  double rms = 0.0;
  std::array<double, 2> fov_deg{0.0, 0.0};
  int images = 0;
  double coverage = 0.0;
  std::string stamp;
  BoardConfig board;
};

fs::path default_intrinsic_path();
std::string normalize_dictionary_name(const std::string & name, int needed_markers = 0);
cv::Ptr<cv::aruco::CharucoBoard> make_charuco_board(
  const BoardConfig & config, cv::Ptr<cv::aruco::Dictionary> * out_dict = nullptr);
CharucoDetection detect_charuco(const cv::Mat & image, const BoardConfig & config);
double combined_coverage(const std::vector<CalibShot> & shots, int width, int height);
std::vector<uint8_t> render_calib_overlay_jpeg(
  const std::vector<uint8_t> & color_jpeg, const BoardConfig & config);
IntrinsicProfile solve_charuco(const BoardConfig & config, std::vector<CalibShot> & shots);
void save_intrinsic_profile(
  const fs::path & path, const IntrinsicProfile * profile, const BoardConfig & board,
  const std::string & camera_key = "default");
IntrinsicProfile load_intrinsic_profile(
  const fs::path & path, int width, int height, BoardConfig * out_board = nullptr,
  const std::string & camera_key = "default");
IntrinsicProfile calibrate_directory(const Options & options);

class CalibrationManager
{
public:
  CalibrationManager(
    const fs::path & intrinsic_file, const fs::path & output_root,
    const std::string & camera_key = "default");

  const std::string & camera_key() const {return camera_key_;}
  BoardConfig board() const;
  void set_board(const BoardConfig & config);
  CalibShot capture_shot(const std::vector<uint8_t> & color_jpeg);
  void delete_shot(const std::string & name, int width, int height);
  void clear_shots(int width, int height);
  IntrinsicProfile solve_and_save(int width, int height);
  IntrinsicProfile saved_profile(int width, int height) const;
  std::string intrinsic_json(int width, int height) const;
  std::string status_json(int width, int height) const;
  std::shared_ptr<const std::vector<uint8_t>> overlay_jpeg(
    uint64_t sequence, const std::shared_ptr<const std::vector<uint8_t>> & color_jpeg);

private:
  fs::path shots_dir(int width, int height) const;
  std::vector<CalibShot> & shots_for(int width, int height);
  const std::vector<CalibShot> * find_shots(int width, int height) const;

  fs::path intrinsic_file_;
  fs::path output_root_;
  std::string camera_key_;
  mutable std::mutex mutex_;
  BoardConfig board_;
  uint64_t board_gen_ = 1;
  uint64_t next_shot_index_ = 1;
  std::vector<std::pair<std::pair<int, int>, std::vector<CalibShot>>> shots_by_size_;
  uint64_t overlay_seq_ = 0;
  uint64_t overlay_board_gen_ = 0;
  std::chrono::steady_clock::time_point overlay_time_{};
  std::shared_ptr<const std::vector<uint8_t>> overlay_jpeg_;
};

}  // namespace head_camera
