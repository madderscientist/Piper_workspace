#include "head_camera/calibration.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace head_camera
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

std::string utc_iso_timestamp()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  gmtime_r(&time, &utc);
  std::ostringstream out;
  out << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return out.str();
}

double round4(double v) {return std::round(v * 10000.0) / 10000.0;}
double round2(double v) {return std::round(v * 100.0) / 100.0;}

cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_id_from_name(const std::string & key)
{
  if (key == "DICT_4X4_50") return cv::aruco::DICT_4X4_50;
  if (key == "DICT_4X4_100") return cv::aruco::DICT_4X4_100;
  if (key == "DICT_4X4_250") return cv::aruco::DICT_4X4_250;
  if (key == "DICT_4X4_1000") return cv::aruco::DICT_4X4_1000;
  if (key == "DICT_5X5_50") return cv::aruco::DICT_5X5_50;
  if (key == "DICT_5X5_100") return cv::aruco::DICT_5X5_100;
  if (key == "DICT_5X5_250") return cv::aruco::DICT_5X5_250;
  if (key == "DICT_5X5_1000") return cv::aruco::DICT_5X5_1000;
  if (key == "DICT_6X6_50") return cv::aruco::DICT_6X6_50;
  if (key == "DICT_6X6_100") return cv::aruco::DICT_6X6_100;
  if (key == "DICT_6X6_250") return cv::aruco::DICT_6X6_250;
  if (key == "DICT_6X6_1000") return cv::aruco::DICT_6X6_1000;
  if (key == "DICT_7X7_50") return cv::aruco::DICT_7X7_50;
  if (key == "DICT_7X7_100") return cv::aruco::DICT_7X7_100;
  if (key == "DICT_7X7_250") return cv::aruco::DICT_7X7_250;
  if (key == "DICT_7X7_1000") return cv::aruco::DICT_7X7_1000;
  if (key == "DICT_ARUCO_ORIGINAL") return cv::aruco::DICT_ARUCO_ORIGINAL;
  throw std::invalid_argument("unsupported ArUco dictionary: " + key);
}

double hull_ratio(const std::vector<cv::Point2f> & pts, int width, int height)
{
  if (pts.size() < 3 || width <= 0 || height <= 0) {
    return 0.0;
  }
  std::vector<cv::Point2f> hull;
  cv::convexHull(pts, hull);
  return cv::contourArea(hull) / (static_cast<double>(width) * static_cast<double>(height));
}

std::string profile_key(int width, int height)
{
  return std::to_string(width) + "x" + std::to_string(height);
}

void append_profile_json(
  std::ostringstream & out, const IntrinsicProfile & p, const fs::path & file,
  const std::string & cam_key)
{
  out << std::boolalpha
      << "{\"available\":" << p.available
      << ",\"camera_key\":" << json_string(cam_key)
      << ",\"width\":" << p.width
      << ",\"height\":" << p.height
      << ",\"camera_matrix\":[";
  for (size_t i = 0; i < p.camera_matrix.size(); ++i) {
    if (i) out << ',';
    out << std::setprecision(12) << p.camera_matrix[i];
  }
  out << "],\"distortion_model\":" << json_string(p.distortion_model)
      << ",\"distortion_coefficients\":[";
  for (size_t i = 0; i < p.distortion_coefficients.size(); ++i) {
    if (i) out << ',';
    out << std::setprecision(12) << p.distortion_coefficients[i];
  }
  out << "],\"rms\":" << round4(p.rms)
      << ",\"fov_deg\":[" << round2(p.fov_deg[0]) << "," << round2(p.fov_deg[1]) << "]"
      << ",\"images\":" << p.images
      << ",\"coverage\":" << round4(p.coverage)
      << ",\"stamp\":" << json_string(p.stamp)
      << ",\"file\":" << json_string(file.string()) << "}";
}
}  // namespace

fs::path default_intrinsic_path()
{
  try {
    const fs::path share(ament_index_cpp::get_package_share_directory("head_camera"));
    const fs::path anchor = share / "config" / "head_camera.yaml";
    if (fs::exists(anchor)) {
      return fs::canonical(anchor).parent_path() / "intrinsic.yaml";
    }
  } catch (...) {
  }
  return fs::absolute("src/head_camera/config/intrinsic.yaml");
}

std::string normalize_dictionary_name(const std::string & name, int needed_markers)
{
  std::string raw;
  for (const unsigned char ch : name) {
    if (std::isspace(ch) || ch == '-') {
      if (!raw.empty() && raw.back() != '_') raw.push_back('_');
    } else {
      raw.push_back(static_cast<char>(std::toupper(ch)));
    }
  }
  while (!raw.empty() && raw.back() == '_') raw.pop_back();
  if (raw.rfind("ARUCO_", 0) == 0) {
    raw = raw.substr(6);
  }
  if (raw.empty()) {
    throw std::invalid_argument("dictionary name must not be empty");
  }
  if (raw.rfind("DICT_", 0) != 0) {
    raw = "DICT_" + raw;
  }
  if (raw == "DICT_4X4" || raw == "DICT_5X5" || raw == "DICT_6X6" || raw == "DICT_7X7") {
    for (const int cap : {50, 100, 250, 1000}) {
      if (cap >= std::max(1, needed_markers)) {
        return raw + "_" + std::to_string(cap);
      }
    }
    return raw + "_1000";
  }
  dictionary_id_from_name(raw);
  return raw;
}

cv::Ptr<cv::aruco::CharucoBoard> make_charuco_board(
  const BoardConfig & config, cv::Ptr<cv::aruco::Dictionary> * out_dict)
{
  if (config.squares_x < 3 || config.squares_y < 3) {
    throw std::invalid_argument("squares_x and squares_y must be at least 3");
  }
  if (config.square_size <= 0.0 || config.marker_size <= 0.0 ||
    config.marker_size >= config.square_size)
  {
    throw std::invalid_argument("marker_size must be positive and smaller than square_size");
  }
  const int needed = (config.squares_x * config.squares_y) / 2;
  const auto norm = normalize_dictionary_name(config.dictionary, needed);
  auto dict = cv::aruco::getPredefinedDictionary(dictionary_id_from_name(norm));
  if (dict->bytesList.rows < needed) {
    throw std::invalid_argument(norm + " has too few markers for " +
      std::to_string(config.squares_x) + "x" + std::to_string(config.squares_y) + " board");
  }
  if (out_dict) {
    *out_dict = dict;
  }
  return cv::aruco::CharucoBoard::create(
    config.squares_x, config.squares_y,
    static_cast<float>(config.square_size), static_cast<float>(config.marker_size), dict);
}

CharucoDetection detect_charuco(const cv::Mat & image, const BoardConfig & config)
{
  if (image.empty()) {
    throw std::invalid_argument("cannot detect ChArUco on empty image");
  }
  cv::Ptr<cv::aruco::Dictionary> dict;
  auto board = make_charuco_board(config, &dict);
  cv::Mat gray;
  if (image.channels() == 1) {
    gray = image;
  } else if (image.channels() == 4) {
    cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
  } else {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  }
  CharucoDetection det;
  det.width = gray.cols;
  det.height = gray.rows;
  auto params = cv::aruco::DetectorParameters::create();
  params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

  std::vector<std::vector<cv::Point2f>> rejected;
  if (gray.cols > 1400) {
    cv::Mat small;
    cv::resize(gray, small, cv::Size(gray.cols / 2, gray.rows / 2), 0.0, 0.0, cv::INTER_LINEAR);
    std::vector<std::vector<cv::Point2f>> small_corners;
    cv::aruco::detectMarkers(small, dict, small_corners, det.marker_ids, params, rejected);
    if (!det.marker_ids.empty()) {
      cv::aruco::refineDetectedMarkers(small, board, small_corners, det.marker_ids, rejected);
    }
    const float sx = static_cast<float>(gray.cols) / static_cast<float>(small.cols);
    const float sy = static_cast<float>(gray.rows) / static_cast<float>(small.rows);
    det.marker_corners.reserve(small_corners.size());
    for (const auto & mc : small_corners) {
      std::vector<cv::Point2f> scaled;
      scaled.reserve(mc.size());
      for (const auto & pt : mc) {
        scaled.emplace_back(pt.x * sx, pt.y * sy);
      }
      det.marker_corners.push_back(std::move(scaled));
    }
  }
  if (gray.cols <= 1400 || det.marker_ids.size() < 4) {
    det.marker_corners.clear();
    det.marker_ids.clear();
    rejected.clear();
    cv::aruco::detectMarkers(gray, dict, det.marker_corners, det.marker_ids, params, rejected);
    if (!det.marker_ids.empty()) {
      cv::aruco::refineDetectedMarkers(gray, board, det.marker_corners, det.marker_ids, rejected);
    }
  }
  det.marker_count = static_cast<int>(det.marker_ids.size());
  if (!det.marker_ids.empty()) {
    cv::Mat ch_corners, ch_ids;
    const int n = cv::aruco::interpolateCornersCharuco(
      det.marker_corners, det.marker_ids, gray, board, ch_corners, ch_ids);
    if (n > 0 && !ch_corners.empty() && !ch_ids.empty()) {
      det.corners.resize(static_cast<size_t>(n));
      det.ids.resize(static_cast<size_t>(n));
      for (int i = 0; i < n; ++i) {
        det.corners[static_cast<size_t>(i)] = ch_corners.at<cv::Point2f>(i, 0);
        det.ids[static_cast<size_t>(i)] = ch_ids.at<int>(i, 0);
      }
      det.collinear = (n < 4) || cv::aruco::testCharucoCornersCollinear(board, ch_ids);
      det.coverage = hull_ratio(det.corners, det.width, det.height);
    }
  }
  return det;
}

double combined_coverage(const std::vector<CalibShot> & shots, int width, int height)
{
  std::vector<cv::Point2f> all;
  for (const auto & shot : shots) {
    all.insert(all.end(), shot.charuco_corners.begin(), shot.charuco_corners.end());
  }
  return hull_ratio(all, width, height);
}

std::vector<uint8_t> render_calib_overlay_jpeg(
  const std::vector<uint8_t> & color_jpeg, const BoardConfig & config)
{
  if (color_jpeg.empty()) {
    return {};
  }
  const cv::Mat raw(1, static_cast<int>(color_jpeg.size()), CV_8UC1,
    const_cast<uint8_t *>(color_jpeg.data()));
  cv::Mat image = cv::imdecode(raw, cv::IMREAD_COLOR);
  if (image.empty()) {
    return {};
  }
  const auto det = detect_charuco(image, config);
  cv::Mat canvas;
  float scale = 1.0f;
  if (image.cols > 960) {
    scale = 960.0f / static_cast<float>(image.cols);
    const int out_h = std::max(2, static_cast<int>(std::round(image.rows * scale)));
    cv::resize(image, canvas, cv::Size(960, out_h), 0.0, 0.0, cv::INTER_AREA);
  } else {
    canvas = image;
  }
  if (!det.marker_ids.empty()) {
    auto markers = det.marker_corners;
    if (scale != 1.0f) {
      for (auto & m : markers) {
        for (auto & p : m) p *= scale;
      }
    }
    cv::aruco::drawDetectedMarkers(canvas, markers, det.marker_ids);
  }
  if (!det.corners.empty()) {
    auto corners = det.corners;
    if (scale != 1.0f) {
      for (auto & p : corners) p *= scale;
    }
    cv::aruco::drawDetectedCornersCharuco(canvas, corners, det.ids, cv::Scalar(60, 60, 255));
  }
  const int max_corners = (config.squares_x - 1) * (config.squares_y - 1);
  const bool enough = det.corners.size() >= 6 && !det.collinear;
  const std::string hud = "Corners: " + std::to_string(det.corners.size()) + "/" +
    std::to_string(max_corners) + "  Markers: " + std::to_string(det.marker_count);
  cv::rectangle(canvas, cv::Point(12, 12), cv::Point(370, 48), cv::Scalar(12, 18, 32), cv::FILLED);
  cv::putText(canvas, hud, cv::Point(22, 37), cv::FONT_HERSHEY_SIMPLEX, 0.65,
    enough ? cv::Scalar(140, 235, 180) : cv::Scalar(130, 170, 255), 2, cv::LINE_AA);
  std::vector<uint8_t> out;
  cv::imencode(".jpg", canvas, out, {cv::IMWRITE_JPEG_QUALITY, 80});
  return out;
}

IntrinsicProfile solve_charuco(const BoardConfig & config, std::vector<CalibShot> & shots)
{
  auto board = make_charuco_board(config);
  BoardConfig norm_cfg = config;
  norm_cfg.dictionary = normalize_dictionary_name(
    config.dictionary, (config.squares_x * config.squares_y) / 2);

  std::vector<size_t> usable;
  int width = 0;
  int height = 0;
  for (size_t i = 0; i < shots.size(); ++i) {
    if (shots[i].corners < 6) continue;
    if (width == 0) {
      width = shots[i].width;
      height = shots[i].height;
    } else if (shots[i].width != width || shots[i].height != height) {
      throw std::invalid_argument("calibration images have mismatched resolutions");
    }
    usable.push_back(i);
  }
  if (usable.size() < 4) {
    throw std::runtime_error("有效标定图仅 " + std::to_string(usable.size()) +
      " 张（每张需 >= 6 个非共线角点），至少需要 4 张才能求解内参");
  }

  std::vector<std::vector<cv::Point2f>> all_corners;
  std::vector<std::vector<int>> all_ids;
  all_corners.reserve(usable.size());
  all_ids.reserve(usable.size());
  for (const size_t idx : usable) {
    all_corners.push_back(shots[idx].charuco_corners);
    all_ids.push_back(shots[idx].charuco_ids);
  }

  cv::Mat camera_matrix, dist_coeffs, std_in, std_ex, per_view;
  std::vector<cv::Mat> rvecs, tvecs;
  const double rms = cv::aruco::calibrateCameraCharuco(
    all_corners, all_ids, board, cv::Size(width, height),
    camera_matrix, dist_coeffs, rvecs, tvecs, std_in, std_ex, per_view, 0);

  for (size_t k = 0; k < usable.size(); ++k) {
    if (static_cast<int>(k) < per_view.rows) {
      shots[usable[k]].error_px = per_view.at<double>(static_cast<int>(k), 0);
    }
  }

  IntrinsicProfile solved;
  solved.available = true;
  solved.width = width;
  solved.height = height;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      solved.camera_matrix[static_cast<size_t>(r * 3 + c)] = camera_matrix.at<double>(r, c);
    }
  }
  const cv::Mat flat_dist = dist_coeffs.reshape(1, 1);
  for (int i = 0; i < std::min(5, flat_dist.cols); ++i) {
    solved.distortion_coefficients[static_cast<size_t>(i)] = flat_dist.at<double>(0, i);
  }
  solved.rms = rms;
  const double fx = solved.camera_matrix[0];
  const double fy = solved.camera_matrix[4];
  solved.fov_deg = {
    fx > 0.0 ? 2.0 * std::atan(static_cast<double>(width) / (2.0 * fx)) * 180.0 / kPi : 0.0,
    fy > 0.0 ? 2.0 * std::atan(static_cast<double>(height) / (2.0 * fy)) * 180.0 / kPi : 0.0,
  };
  solved.images = static_cast<int>(usable.size());
  solved.coverage = combined_coverage(shots, width, height);
  solved.stamp = utc_iso_timestamp();
  solved.board = norm_cfg;
  return solved;
}

void save_intrinsic_profile(
  const fs::path & path, const IntrinsicProfile * profile, const BoardConfig & board,
  const std::string & camera_key)
{
  const fs::path target = fs::is_symlink(path) ? fs::canonical(path) : path;
  if (target.has_parent_path()) {
    fs::create_directories(target.parent_path());
  }
  YAML::Node root;
  if (fs::exists(target)) {
    try {
      root = YAML::LoadFile(target.string());
    } catch (...) {
      root = YAML::Node(YAML::NodeType::Map);
    }
  }
  if (!root.IsMap()) {
    root = YAML::Node(YAML::NodeType::Map);
  }
  YAML::Node cams = root["cameras"];
  if (!cams || !cams.IsMap()) {
    cams = YAML::Node(YAML::NodeType::Map);
  }
  cams.SetStyle(YAML::EmitterStyle::Block);
  if (profile && profile->available) {
    const std::string key = camera_key.empty() ? "default" : camera_key;
    YAML::Node cam_node = cams[key];
    if (!cam_node || !cam_node.IsMap()) {
      cam_node = YAML::Node(YAML::NodeType::Map);
    }
    cam_node.SetStyle(YAML::EmitterStyle::Block);
    if (!cam_node["profiles"] || !cam_node["profiles"].IsMap()) {
      cam_node["profiles"] = YAML::Node(YAML::NodeType::Map);
    }
    cam_node["profiles"].SetStyle(YAML::EmitterStyle::Block);
    YAML::Node p;
    p.SetStyle(YAML::EmitterStyle::Block);
    p["width"] = profile->width;
    p["height"] = profile->height;
    YAML::Node km(YAML::NodeType::Sequence);
    km.SetStyle(YAML::EmitterStyle::Flow);
    for (const double v : profile->camera_matrix) km.push_back(v);
    p["camera_matrix"] = km;
    p["distortion_model"] = profile->distortion_model;
    YAML::Node dc(YAML::NodeType::Sequence);
    dc.SetStyle(YAML::EmitterStyle::Flow);
    for (const double v : profile->distortion_coefficients) dc.push_back(v);
    p["distortion_coefficients"] = dc;
    p["rms"] = round4(profile->rms);
    YAML::Node fov(YAML::NodeType::Sequence);
    fov.SetStyle(YAML::EmitterStyle::Flow);
    fov.push_back(round2(profile->fov_deg[0]));
    fov.push_back(round2(profile->fov_deg[1]));
    p["fov_deg"] = fov;
    p["images"] = profile->images;
    p["coverage"] = round4(profile->coverage);
    p["stamp"] = profile->stamp;
    cam_node["profiles"][profile_key(profile->width, profile->height)] = p;
    cams[key] = cam_node;
  }
  const std::string dict_norm = normalize_dictionary_name(
    board.dictionary, (board.squares_x * board.squares_y) / 2);
  YAML::Emitter emitter;
  emitter << YAML::BeginMap
          << YAML::Key << "version" << YAML::Value << 1
          << YAML::Key << "board" << YAML::Value << YAML::BeginMap
          << YAML::Key << "squares_x" << YAML::Value << board.squares_x
          << YAML::Key << "squares_y" << YAML::Value << board.squares_y
          << YAML::Key << "square_size" << YAML::Value << board.square_size
          << YAML::Key << "marker_size" << YAML::Value << board.marker_size
          << YAML::Key << "dictionary" << YAML::Value << dict_norm
          << YAML::EndMap
          << YAML::Key << "cameras" << YAML::Value << YAML::BeginMap;
  for (const auto & item : cams) {
    YAML::Node val = item.second;
    val.SetStyle(YAML::EmitterStyle::Block);
    if (val["profiles"] && val["profiles"].IsMap()) {
      val["profiles"].SetStyle(YAML::EmitterStyle::Block);
      for (auto p_it : val["profiles"]) {
        p_it.second.SetStyle(YAML::EmitterStyle::Block);
      }
    }
    emitter << YAML::Key << YAML::DoubleQuoted << item.first.as<std::string>()
            << YAML::Value << val;
  }
  emitter << YAML::EndMap << YAML::EndMap;
  const fs::path tmp = target.string() + ".tmp";
  write_text(tmp, std::string(emitter.c_str()) + "\n");
  fs::rename(tmp, target);
}

IntrinsicProfile load_intrinsic_profile(
  const fs::path & path, int width, int height, BoardConfig * out_board,
  const std::string & camera_key)
{
  IntrinsicProfile result;
  result.width = width;
  result.height = height;
  if (!fs::exists(path)) {
    return result;
  }
  const auto root = YAML::LoadFile(path.string());
  if (!root.IsMap()) {
    return result;
  }
  BoardConfig board;
  if (const auto b = root["board"]; b && b.IsMap()) {
    if (b["squares_x"]) board.squares_x = b["squares_x"].as<int>();
    if (b["squares_y"]) board.squares_y = b["squares_y"].as<int>();
    if (b["square_size"]) board.square_size = b["square_size"].as<double>();
    if (b["marker_size"]) board.marker_size = b["marker_size"].as<double>();
    if (b["dictionary"]) board.dictionary = b["dictionary"].as<std::string>();
  }
  if (out_board) {
    *out_board = board;
  }
  const std::string key = camera_key.empty() ? "default" : camera_key;
  YAML::Node profiles;
  if (const auto cams = root["cameras"]; cams && cams.IsMap() && cams[key] && cams[key]["profiles"]) {
    profiles = cams[key]["profiles"];
  } else if (root["profiles"] && root["profiles"].IsMap()) {
    profiles = root["profiles"];
  }
  if (!profiles || !profiles.IsMap()) {
    return result;
  }
  YAML::Node p = profiles[profile_key(width, height)];
  if (!p && width == 0 && height == 0 && profiles.begin() != profiles.end()) {
    p = profiles.begin()->second;
  }
  if (!p || !p.IsMap()) {
    return result;
  }
  result.available = true;
  result.width = p["width"].as<int>();
  result.height = p["height"].as<int>();
  if (const auto km = p["camera_matrix"]; km && km.IsSequence() && km.size() == 9) {
    for (size_t i = 0; i < 9; ++i) result.camera_matrix[i] = km[i].as<double>();
  }
  if (p["distortion_model"]) {
    result.distortion_model = p["distortion_model"].as<std::string>();
  }
  if (const auto dc = p["distortion_coefficients"]; dc && dc.IsSequence()) {
    for (size_t i = 0; i < std::min<size_t>(5, dc.size()); ++i) {
      result.distortion_coefficients[i] = dc[i].as<double>();
    }
  }
  if (p["rms"]) result.rms = p["rms"].as<double>();
  if (const auto fov = p["fov_deg"]; fov && fov.IsSequence() && fov.size() >= 2) {
    result.fov_deg = {fov[0].as<double>(), fov[1].as<double>()};
  }
  if (p["images"]) result.images = p["images"].as<int>();
  if (p["coverage"]) result.coverage = p["coverage"].as<double>();
  if (p["stamp"]) result.stamp = p["stamp"].as<std::string>();
  result.board = board;
  return result;
}

IntrinsicProfile calibrate_directory(const Options & options)
{
  if (!fs::is_directory(options.input)) {
    throw std::runtime_error("input directory does not exist: " + options.input.string());
  }
  BoardConfig board{
    options.squares_x, options.squares_y,
    options.square_size, options.marker_size, options.dictionary};
  make_charuco_board(board);
  std::vector<fs::path> files;
  for (const auto & entry : fs::directory_iterator(options.input)) {
    if (!entry.is_regular_file()) continue;
    auto ext = entry.path().extension().string();
    for (auto & ch : ext) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    if (ext == ".jpg" || ext == ".jpeg" || ext == ".png") {
      const auto stem = entry.path().stem().string();
      if (stem.rfind("depth_", 0) == 0) continue;
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  if (files.empty()) {
    throw std::runtime_error("no RGB images (.jpg/.png) found in " + options.input.string());
  }
  std::vector<CalibShot> shots;
  for (const auto & file : files) {
    const cv::Mat img = cv::imread(file.string(), cv::IMREAD_COLOR);
    if (img.empty()) continue;
    const auto det = detect_charuco(img, board);
    if (det.corners.size() < 6 || det.collinear) continue;
    shots.push_back({
      file.filename().string(), det.width, det.height,
      static_cast<int>(det.corners.size()), det.marker_count,
      det.coverage, -1.0, det.corners, det.ids,
    });
  }
  const auto solved = solve_charuco(board, shots);
  const fs::path target = options.intrinsic_file.empty() ?
    default_intrinsic_path() : options.intrinsic_file;
  std::string cam_key = !options.serial.empty() ? options.serial : options.camera;
  for (const auto & spec : options.cameras) {
    if (spec.name == cam_key && !spec.serial.empty()) {
      cam_key = spec.serial;
      break;
    }
  }
  if (cam_key.empty()) cam_key = "default";
  save_intrinsic_profile(target, &solved, solved.board, cam_key);
  return solved;
}

CalibrationManager::CalibrationManager(
  const fs::path & intrinsic_file, const fs::path & output_root, const std::string & camera_key)
: intrinsic_file_(intrinsic_file.empty() ? default_intrinsic_path() : intrinsic_file),
  output_root_(output_root),
  camera_key_(camera_key.empty() ? "default" : camera_key)
{
  try {
    load_intrinsic_profile(intrinsic_file_, 0, 0, &board_, camera_key_);
    board_.dictionary = normalize_dictionary_name(
      board_.dictionary, (board_.squares_x * board_.squares_y) / 2);
  } catch (...) {
    board_ = BoardConfig{};
  }
}

BoardConfig CalibrationManager::board() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return board_;
}

void CalibrationManager::set_board(const BoardConfig & config)
{
  BoardConfig normalized = config;
  make_charuco_board(normalized);
  normalized.dictionary = normalize_dictionary_name(
    normalized.dictionary, (normalized.squares_x * normalized.squares_y) / 2);
  std::lock_guard<std::mutex> lock(mutex_);
  board_ = normalized;
  ++board_gen_;
  shots_by_size_.clear();
  save_intrinsic_profile(intrinsic_file_, nullptr, board_, camera_key_);
}

fs::path CalibrationManager::shots_dir(int width, int height) const
{
  return fs::absolute(output_root_) / ("calib_" + camera_key_ + "_" + profile_key(width, height));
}

std::vector<CalibShot> & CalibrationManager::shots_for(int width, int height)
{
  const auto key = std::make_pair(width, height);
  for (auto & item : shots_by_size_) {
    if (item.first == key) return item.second;
  }
  shots_by_size_.push_back({key, {}});
  return shots_by_size_.back().second;
}

const std::vector<CalibShot> * CalibrationManager::find_shots(int width, int height) const
{
  const auto key = std::make_pair(width, height);
  for (const auto & item : shots_by_size_) {
    if (item.first == key) return &item.second;
  }
  return nullptr;
}

CalibShot CalibrationManager::capture_shot(const std::vector<uint8_t> & color_jpeg)
{
  if (color_jpeg.empty()) {
    throw std::runtime_error("当前没有可用的彩色图像帧");
  }
  const cv::Mat raw(1, static_cast<int>(color_jpeg.size()), CV_8UC1,
    const_cast<uint8_t *>(color_jpeg.data()));
  const cv::Mat img = cv::imdecode(raw, cv::IMREAD_COLOR);
  if (img.empty()) {
    throw std::runtime_error("无法解码相机 JPEG 帧");
  }
  BoardConfig cfg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cfg = board_;
  }
  const auto det = detect_charuco(img, cfg);
  if (det.corners.size() < 6 || det.collinear) {
    throw std::runtime_error("仅检测到 " + std::to_string(det.corners.size()) +
      " 个角点（需 >= 6 个非共线角点），请调整标定板距离或角度");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream name;
  name << "shot_" << std::setw(4) << std::setfill('0') << next_shot_index_++;
  const fs::path dir = shots_dir(det.width, det.height);
  fs::create_directories(dir);
  write_text(dir / (name.str() + ".jpg"), std::string(
    reinterpret_cast<const char *>(color_jpeg.data()), color_jpeg.size()));
  CalibShot shot{
    name.str(), det.width, det.height,
    static_cast<int>(det.corners.size()), det.marker_count,
    det.coverage, -1.0, det.corners, det.ids,
  };
  shots_for(det.width, det.height).push_back(shot);
  return shot;
}

void CalibrationManager::delete_shot(const std::string & name, int width, int height)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & vec = shots_for(width, height);
  vec.erase(std::remove_if(vec.begin(), vec.end(), [&](const CalibShot & s) {
    return s.name == name;
  }), vec.end());
  std::error_code ec;
  fs::remove(shots_dir(width, height) / (name + ".jpg"), ec);
}

void CalibrationManager::clear_shots(int width, int height)
{
  std::lock_guard<std::mutex> lock(mutex_);
  shots_for(width, height).clear();
  std::error_code ec;
  fs::remove_all(shots_dir(width, height), ec);
}

IntrinsicProfile CalibrationManager::solve_and_save(int width, int height)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & vec = shots_for(width, height);
  const auto solved = solve_charuco(board_, vec);
  save_intrinsic_profile(intrinsic_file_, &solved, solved.board, camera_key_);
  return solved;
}

IntrinsicProfile CalibrationManager::saved_profile(int width, int height) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    return load_intrinsic_profile(intrinsic_file_, width, height, nullptr, camera_key_);
  } catch (...) {
    return IntrinsicProfile{};
  }
}

std::string CalibrationManager::intrinsic_json(int width, int height) const
{
  const auto profile = saved_profile(width, height);
  std::ostringstream out;
  append_profile_json(out, profile, intrinsic_file_, camera_key_);
  return out.str();
}

std::string CalibrationManager::status_json(int width, int height) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  IntrinsicProfile saved;
  try {
    saved = load_intrinsic_profile(intrinsic_file_, width, height, nullptr, camera_key_);
  } catch (...) {
  }
  const auto * vec = find_shots(width, height);
  const double cov = (vec && !vec->empty()) ? combined_coverage(*vec, width, height) : 0.0;
  std::ostringstream out;
  out << std::boolalpha
      << "{\"camera_key\":" << json_string(camera_key_)
      << ",\"board\":{\"squares_x\":" << board_.squares_x
      << ",\"squares_y\":" << board_.squares_y
      << ",\"square_size\":" << board_.square_size
      << ",\"marker_size\":" << board_.marker_size
      << ",\"dictionary\":" << json_string(board_.dictionary) << "}"
      << ",\"calib_file\":" << json_string(intrinsic_file_.string())
      << ",\"shots_dir\":" << json_string(shots_dir(width, height).string())
      << ",\"total_coverage\":" << round4(cov)
      << ",\"shots\":[";
  if (vec) {
    for (size_t i = 0; i < vec->size(); ++i) {
      if (i) out << ',';
      const auto & s = (*vec)[i];
      out << "{\"name\":" << json_string(s.name)
          << ",\"corners\":" << s.corners
          << ",\"markers\":" << s.markers
          << ",\"coverage\":" << round4(s.coverage)
          << ",\"error_px\":" << (s.error_px >= 0.0 ? round4(s.error_px) : -1.0) << "}";
    }
  }
  out << "],\"saved\":";
  append_profile_json(out, saved, intrinsic_file_, camera_key_);
  out << '}';
  return out.str();
}

std::shared_ptr<const std::vector<uint8_t>> CalibrationManager::overlay_jpeg(
  uint64_t sequence, const std::shared_ptr<const std::vector<uint8_t>> & color_jpeg)
{
  if (!color_jpeg || color_jpeg->empty()) {
    return {};
  }
  BoardConfig cfg;
  uint64_t gen = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (overlay_jpeg_ && overlay_board_gen_ == board_gen_ &&
      (overlay_seq_ == sequence || now - overlay_time_ < std::chrono::milliseconds(75)))
    {
      return overlay_jpeg_;
    }
    cfg = board_;
    gen = board_gen_;
  }
  auto encoded = std::make_shared<const std::vector<uint8_t>>(
    render_calib_overlay_jpeg(*color_jpeg, cfg));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    overlay_seq_ = sequence;
    overlay_board_gen_ = gen;
    overlay_time_ = std::chrono::steady_clock::now();
    overlay_jpeg_ = encoded;
  }
  return encoded;
}
}  // namespace head_camera
