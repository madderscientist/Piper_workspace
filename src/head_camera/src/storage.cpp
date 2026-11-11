#include "head_camera/head_camera.hpp"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace head_camera
{
namespace
{
#if HEAD_CAMERA_HAS_K4A
cv::Mat depth_matrix(k4a_image_t image)
{
  if (!image || k4a_image_get_format(image) != K4A_IMAGE_FORMAT_DEPTH16) {
    throw std::invalid_argument("expected an SDK DEPTH16 image");
  }
  const auto width = k4a_image_get_width_pixels(image);
  const auto height = k4a_image_get_height_pixels(image);
  const auto stride = k4a_image_get_stride_bytes(image);
  if (width <= 0 || height <= 0 || stride < static_cast<int64_t>(width) * 2 ||
    !k4a_image_get_buffer(image) ||
    k4a_image_get_size(image) < static_cast<size_t>(height) * static_cast<size_t>(stride))
  {
    throw std::invalid_argument("invalid SDK depth image dimensions or buffer");
  }
  return cv::Mat(
    height, width, CV_16UC1, k4a_image_get_buffer(image), static_cast<size_t>(stride));
}
#endif

cv::Mat colorize(const cv::Mat & depth)
{
  cv::Mat gray, colored;
  double maximum = 0;
  cv::minMaxLoc(depth, nullptr, &maximum);
  depth.convertTo(gray, CV_8UC1, maximum > 0 ? 255.0 / maximum : 1.0);
  cv::applyColorMap(gray, colored, cv::COLORMAP_TURBO);
  colored.setTo(cv::Scalar(0, 0, 0), depth == 0);
  return colored;
}
}  // namespace

std::string json_string(const std::string & value)
{
  std::ostringstream out;
  out << '"';
  for (const unsigned char ch : value) {
    switch (ch) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (ch < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(ch);
        } else {
          out << ch;
        }
    }
  }
  out << '"';
  return out.str();
}

void write_text(const fs::path & path, const std::string & text)
{
  std::ofstream stream;
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream.open(path, std::ios::binary);
  stream.write(text.data(), static_cast<std::streamsize>(text.size()));
  stream.close();
}

std::vector<uint8_t> depth_preview_jpeg(
  const uint16_t * data, int width, int height, size_t stride_bytes)
{
  if (!data || width <= 0 || height <= 0 || stride_bytes < static_cast<size_t>(width) * 2) {
    throw std::invalid_argument("invalid depth buffer dimensions");
  }
  const cv::Mat depth(height, width, CV_16UC1, const_cast<uint16_t *>(data), stride_bytes);
  std::vector<uint8_t> jpeg;
  if (!cv::imencode(".jpg", colorize(depth), jpeg)) {
    throw std::runtime_error("could not encode live depth preview");
  }
  return jpeg;
}

#if HEAD_CAMERA_HAS_K4A
std::vector<uint8_t> depth_preview_jpeg(k4a_image_t image)
{
  const auto depth = depth_matrix(image);
  return depth_preview_jpeg(
    reinterpret_cast<const uint16_t *>(depth.data), depth.cols, depth.rows, depth.step);
}

void save_depth(const fs::path & path, k4a_image_t image, bool preview)
{
  const auto depth = depth_matrix(image);
  if (!cv::imwrite(path.string(), depth)) {
    throw std::runtime_error("could not write depth image: " + path.string());
  }
  if (preview) {
    const auto preview_path = path.parent_path() / (path.stem().string() + "_preview.jpg");
    if (!cv::imwrite(preview_path.string(), colorize(depth))) {
      throw std::runtime_error("could not write depth preview: " + preview_path.string());
    }
  }
}
#endif

fs::path create_session(const fs::path & root, const std::string & command)
{
  fs::create_directories(root);
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  if (!gmtime_r(&time, &utc)) {
    throw std::runtime_error("could not format session timestamp");
  }
  const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
    now.time_since_epoch()).count() % 1000000;
  std::ostringstream name;
  name << command << '_' << std::put_time(&utc, "%Y%m%dT%H%M%S") << '_'
       << std::setfill('0') << std::setw(6) << micros << 'Z';
  for (uint64_t suffix = 0; ; ++suffix) {
    const auto path = root / (name.str() + (suffix ? "_" + std::to_string(suffix) : ""));
    if (fs::create_directory(path)) {
      return fs::absolute(path);
    }
  }
}
}  // namespace head_camera
