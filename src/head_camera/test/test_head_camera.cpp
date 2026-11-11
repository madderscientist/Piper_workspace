#include "head_camera/web.hpp"

#include <cstring>
#include <fstream>
#include <iterator>

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace hc = head_camera;

class HeadCameraTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root = hc::create_session(hc::fs::temp_directory_path(), "head_camera_test");
  }

  void TearDown() override
  {
    hc::fs::remove_all(root);
  }

  hc::fs::path root;
};

TEST_F(HeadCameraTest, PureRgbAndDepthOptionValidation)
{
  const auto serve_defaults = hc::parse_options({"serve"});
  EXPECT_EQ(serve_defaults.depth_mode, "OFF");
  EXPECT_EQ(serve_defaults.color, "1080p");
  EXPECT_EQ(hc::color_dimensions(serve_defaults.color), std::make_pair(1920, 1080));

  const auto rgb_capture = hc::parse_options({"capture", "--depth-mode", "OFF", "--video-device", "/dev/video0"});
  EXPECT_EQ(rgb_capture.depth_mode, "OFF");
  EXPECT_EQ(rgb_capture.color, "1080p");
  EXPECT_EQ(rgb_capture.video_device, "/dev/video0");
  EXPECT_TRUE(rgb_capture.device_explicit);

  const auto dual_serve = hc::parse_options({"serve", "--camera", "head_1,head_2"});
  EXPECT_EQ(dual_serve.camera, "head_1,head_2");
  const auto targets = hc::resolve_serve_targets(dual_serve);
  ASSERT_EQ(targets.size(), 2u);
  EXPECT_EQ(targets[0].camera_name, "head_1");
  EXPECT_EQ(targets[0].serial, "000314605112");
  EXPECT_EQ(targets[1].camera_name, "head_2");
  EXPECT_EQ(targets[1].serial, "000456614112");

  EXPECT_THROW(hc::parse_options({"serve", "--depth-mode", "OFF", "--color", "OFF"}), std::invalid_argument);
  EXPECT_THROW(hc::parse_options({"record", "--depth-mode", "OFF"}), std::invalid_argument);
  EXPECT_THROW(hc::parse_options({"capture", "--count", "-1"}), std::invalid_argument);
  EXPECT_THROW(hc::parse_options({"record", "--duration", "nan"}), std::invalid_argument);
  EXPECT_THROW(hc::parse_options({"calibrate"}), std::invalid_argument);

#if HEAD_CAMERA_HAS_K4A
  hc::write_text(root / "config.yaml",
    "depth_mode: WFOV_UNBINNED\nfps: 15\ncount: 2\ncolor: '1080p'\npreview: true\n");
  const auto options = hc::parse_options({
    "capture", "--count", "3", "--config", (root / "config.yaml").string()});
  EXPECT_EQ(options.count, 3u);
  EXPECT_TRUE(options.preview);
  const auto config = hc::device_configuration(options);
  EXPECT_EQ(config.depth_mode, K4A_DEPTH_MODE_WFOV_UNBINNED);
  EXPECT_EQ(config.camera_fps, K4A_FRAMES_PER_SECOND_15);
  EXPECT_EQ(config.color_resolution, K4A_COLOR_RESOLUTION_1080P);
  EXPECT_TRUE(config.synchronized_images_only);
  EXPECT_THROW(hc::parse_options({"record", "--depth-mode", "WFOV_UNBINNED", "--fps", "30"}),
    std::invalid_argument);
#endif
}

#if HEAD_CAMERA_HAS_K4A
TEST_F(HeadCameraTest, DepthPngAndMkvRoundTrip)
{
  k4a_image_t handle = nullptr;
  hc::check(k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, 6, 2, 16, &handle), "test depth image");
  hc::Image image(handle, k4a_image_release);
  const uint16_t values[] = {0, 1, 255, 256, 1200, 65535};
  auto * buffer = k4a_image_get_buffer(image.get());
  std::memset(buffer, 0xaa, k4a_image_get_size(image.get()));
  std::memcpy(buffer, values, sizeof(values));
  std::memcpy(buffer + 16, values, sizeof(values));
  hc::save_depth(root / "depth.png", image.get(), true);
  const auto saved = cv::imread((root / "depth.png").string(), cv::IMREAD_UNCHANGED);
  ASSERT_EQ(saved.type(), CV_16UC1);
  for (int row = 0; row < saved.rows; ++row) {
    for (int col = 0; col < saved.cols; ++col) {
      EXPECT_EQ(saved.at<uint16_t>(row, col), values[col]);
    }
  }

  hc::Options settings;
  const auto config = hc::device_configuration(settings);
  k4a_record_t record_handle = nullptr;
  hc::check(k4a_record_create((root / "input.mkv").c_str(), nullptr, config, &record_handle),
    "test recording creation");
  hc::Recording recording(record_handle, k4a_record_close);
  const std::string calibration = "{\"test_fixture\":true}";
  hc::check(k4a_record_add_attachment(recording.get(), "calibration.json",
    reinterpret_cast<const uint8_t *>(calibration.data()), calibration.size()), "test attachment");
  hc::check(k4a_record_add_tag(recording.get(), "K4A_CALIBRATION_FILE", "calibration.json"),
    "test calibration tag");
  hc::check(k4a_record_write_header(recording.get()), "test recording header");
  for (uint64_t frame = 0; frame < 2; ++frame) {
    k4a_image_t image_handle = nullptr;
    hc::check(k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, 512, 512, 1024, &image_handle),
      "test image creation");
    hc::Image depth(image_handle, k4a_image_release);
    auto * data = reinterpret_cast<uint16_t *>(k4a_image_get_buffer(depth.get()));
    std::fill(data, data + 512 * 512, static_cast<uint16_t>(700 + frame));
    k4a_image_set_device_timestamp_usec(depth.get(), 1000000 + frame * 40000);
    k4a_capture_t capture_handle = nullptr;
    hc::check(k4a_capture_create(&capture_handle), "test capture creation");
    hc::Capture capture(capture_handle, k4a_capture_release);
    k4a_capture_set_depth_image(capture.get(), depth.get());
    hc::check(k4a_record_write_capture(recording.get(), capture.get()), "test recording capture");
  }
  hc::check(k4a_record_flush(recording.get()), "test recording flush");
  recording.reset();

  const auto export_opts = hc::parse_options({
    "export", "--input", (root / "input.mkv").string(), "--output", root.string()});
  const volatile std::sig_atomic_t stopped = 0;
  const auto result = hc::export_recording(export_opts, stopped);
  EXPECT_EQ(result.frames, 2u);
}
#endif

TEST_F(HeadCameraTest, CharucoIntrinsicCalibrationWebAndCliRecoverCameraMatrix)
{
  hc::BoardConfig board_cfg;
  auto board = hc::make_charuco_board(board_cfg);
  cv::Mat tex;
  board->draw(cv::Size(2520, 1920), tex, 60);
  cv::cvtColor(tex, tex, cv::COLOR_GRAY2BGR);
  cv::GaussianBlur(tex, tex, cv::Size(3, 3), 0.6);
  const auto ref_det = hc::detect_charuco(tex, board_cfg);
  ASSERT_GT(ref_det.corners.size(), 40u);

  std::vector<cv::Point2f> tex_pts, plane_pts;
  for (size_t i = 0; i < ref_det.ids.size(); ++i) {
    const auto & p3 = board->chessboardCorners[static_cast<size_t>(ref_det.ids[i])];
    tex_pts.push_back(ref_det.corners[i]);
    plane_pts.emplace_back(p3.x, p3.y);
  }
  const cv::Mat h_tex_to_plane = cv::findHomography(tex_pts, plane_pts);

  const cv::Mat K = (cv::Mat_<double>(3, 3) << 920.0, 0.0, 640.0, 0.0, 915.0, 360.0, 0.0, 0.0, 1.0);
  constexpr double kPi = 3.14159265358979323846;
  const struct Pose { double rx, ry, rz, tx, ty, tz; } poses[] = {
    {kPi + 0.38, -0.22, 0.05, -0.11, 0.07, 0.30},
    {kPi - 0.40, 0.25, -0.08, -0.06, 0.05, 0.32},
    {kPi + 0.32, 0.35, 0.15, -0.12, 0.08, 0.34},
    {kPi - 0.35, -0.36, -0.12, -0.07, 0.06, 0.29},
    {kPi + 0.08, 0.44, 0.30, -0.10, 0.04, 0.31},
    {kPi - 0.42, -0.05, -0.28, -0.08, 0.09, 0.30},
  };

  const auto img_dir = root / "calib_inputs";
  hc::fs::create_directories(img_dir);
  const auto intrinsic_yaml = root / "intrinsic.yaml";
  const auto serve_opts = hc::parse_options({
    "serve", "--color", "720p", "--output", root.string(),
    "--intrinsic", intrinsic_yaml.string()});
  hc::WebState state(serve_opts);
  state.camera_connected(1280, 720);

  for (size_t idx = 0; idx < 6; ++idx) {
    const auto & p = poses[idx];
    cv::Mat R;
    cv::Rodrigues(cv::Vec3d(p.rx, p.ry, p.rz), R);
    cv::Mat Rt(3, 3, CV_64F);
    R.col(0).copyTo(Rt.col(0));
    R.col(1).copyTo(Rt.col(1));
    Rt.at<double>(0, 2) = p.tx;
    Rt.at<double>(1, 2) = p.ty;
    Rt.at<double>(2, 2) = p.tz;
    const cv::Mat H = K * Rt * h_tex_to_plane;
    cv::Mat view;
    cv::warpPerspective(
      tex, view, H, cv::Size(1280, 720), cv::INTER_LINEAR,
      cv::BORDER_CONSTANT, cv::Scalar(220, 220, 220));
    const auto path = img_dir / ("view_" + std::to_string(idx) + ".png");
    ASSERT_TRUE(cv::imwrite(path.string(), view));

    std::vector<uint8_t> jpg;
    cv::imencode(".jpg", view, jpg, {cv::IMWRITE_JPEG_QUALITY, 95});
    state.publish_color(jpg);
    EXPECT_EQ(hc::web_request(state, "POST", "/api/calib/capture", "", "{}").status, 200u);
  }

  const auto solve_res = hc::web_request(state, "POST", "/api/calib/solve", "", "{}");
  ASSERT_EQ(solve_res.status, 200u) << solve_res.body;
  const auto intr_json = YAML::Load(hc::web_request(state, "GET", "/api/intrinsic", "").body);
  ASSERT_TRUE(intr_json["available"].as<bool>());
  EXPECT_LT(intr_json["rms"].as<double>(), 1.0);
  EXPECT_NEAR(intr_json["camera_matrix"][0].as<double>(), 920.0, 12.0);
  EXPECT_NEAR(intr_json["camera_matrix"][4].as<double>(), 915.0, 12.0);
  EXPECT_NEAR(intr_json["camera_matrix"][2].as<double>(), 640.0, 10.0);
  EXPECT_NEAR(intr_json["camera_matrix"][5].as<double>(), 360.0, 10.0);

  // Offline CLI calibrate_directory produces matching profile in YAML
  const auto cli_yaml = root / "cli_intrinsic.yaml";
  const auto cli_opts = hc::parse_options({
    "calibrate", "--camera", "000314605112", "--input", img_dir.string(), "--intrinsic", cli_yaml.string()});
  const auto cli_solved = hc::calibrate_directory(cli_opts);
  EXPECT_TRUE(cli_solved.available);
  EXPECT_EQ(cli_solved.images, 6);
  EXPECT_LT(cli_solved.rms, 1.0);
  const auto loaded = hc::load_intrinsic_profile(cli_yaml, 1280, 720, nullptr, "000314605112");
  EXPECT_TRUE(loaded.available);
  EXPECT_NEAR(loaded.camera_matrix[0], 920.0, 12.0);
  EXPECT_FALSE(hc::load_intrinsic_profile(cli_yaml, 1280, 720, nullptr, "000456614112").available);
  std::ifstream yaml_text_in(cli_yaml);
  const std::string yaml_text((std::istreambuf_iterator<char>(yaml_text_in)), std::istreambuf_iterator<char>());
  EXPECT_NE(yaml_text.find("\"000314605112\":"), std::string::npos);
}
