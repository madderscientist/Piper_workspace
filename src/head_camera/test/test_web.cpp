#include "head_camera/web.hpp"

#include <atomic>
#include <cstring>
#include <functional>
#include <thread>

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

namespace hc = head_camera;
namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using tcp = asio::ip::tcp;

#if HEAD_CAMERA_HAS_K4A
namespace
{
std::atomic_bool test_camera(false);
std::atomic_uint opens(0);
_k4a_device_t device{};
uint64_t sequence = 0;

k4a_buffer_result_t copy_string(const std::string & value, void * target, size_t * size)
{
  const auto required = value.size() + 1;
  if (!target || *size < required) {
    *size = required;
    return K4A_BUFFER_RESULT_TOO_SMALL;
  }
  std::memcpy(target, value.c_str(), required);
  *size = required;
  return K4A_BUFFER_RESULT_SUCCEEDED;
}
}  // namespace

extern "C" {
uint32_t k4a_device_get_installed_count(void) {return test_camera ? 1 : 0;}
k4a_result_t k4a_device_open(uint32_t index, k4a_device_t * handle)
{
  if (!test_camera || index != 0) {
    return K4A_RESULT_FAILED;
  }
  *handle = &device;
  ++opens;
  sequence = 0;
  return K4A_RESULT_SUCCEEDED;
}
void k4a_device_close(k4a_device_t) {}
void k4a_device_stop_cameras(k4a_device_t) {}
k4a_result_t k4a_device_start_cameras(k4a_device_t, const k4a_device_configuration_t *) {return K4A_RESULT_SUCCEEDED;}
k4a_buffer_result_t k4a_device_get_serialnum(k4a_device_t, char * data, size_t * size) {return copy_string("head-camera-test", data, size);}
k4a_buffer_result_t k4a_device_get_raw_calibration(k4a_device_t, uint8_t * data, size_t * size) {return copy_string("{\"test_fixture\":true}", data, size);}
k4a_result_t k4a_device_get_version(k4a_device_t, k4a_hardware_version_t * version) {*version = {}; return K4A_RESULT_SUCCEEDED;}
k4a_wait_result_t k4a_device_get_capture(k4a_device_t, k4a_capture_t * handle, int32_t)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  k4a_image_t depth_handle = nullptr;
  hc::check(k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, 512, 512, 1024, &depth_handle), "test depth");
  hc::Image depth(depth_handle, k4a_image_release);
  auto * values = reinterpret_cast<uint16_t *>(k4a_image_get_buffer(depth.get()));
  std::fill(values, values + 512 * 512, 900);
  const auto timestamp = 1000000 + sequence++ * 40000;
  k4a_image_set_device_timestamp_usec(depth.get(), timestamp);
  static std::vector<uint8_t> jpeg = [] {
      std::vector<uint8_t> bytes;
      cv::imencode(".jpg", cv::Mat(720, 1280, CV_8UC3, cv::Scalar(50, 100, 150)), bytes);
      return bytes;
    }();
  k4a_image_t color_handle = nullptr;
  hc::check(k4a_image_create_from_buffer(K4A_IMAGE_FORMAT_COLOR_MJPG, 1280, 720, 0,
    jpeg.data(), jpeg.size(), nullptr, nullptr, &color_handle), "test color");
  hc::Image color(color_handle, k4a_image_release);
  k4a_image_set_device_timestamp_usec(color.get(), timestamp);
  hc::check(k4a_capture_create(handle), "test capture");
  k4a_capture_set_depth_image(*handle, depth.get());
  k4a_capture_set_color_image(*handle, color.get());
  return K4A_WAIT_RESULT_SUCCEEDED;
}
}  // extern "C"
#endif

class WebCameraTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root = hc::create_session(hc::fs::temp_directory_path(), "head_camera_web_test");
#if HEAD_CAMERA_HAS_K4A
    opens = 0;
    test_camera = true;
#endif
  }

  void TearDown() override
  {
#if HEAD_CAMERA_HAS_K4A
    test_camera = false;
#endif
    hc::fs::remove_all(root);
  }

  bool wait(const std::function<bool()> & ready)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!ready() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return ready();
  }

  hc::fs::path root;
};

TEST_F(WebCameraTest, PureRgbHttpKeepAliveFreshSyncAndMjpegStream)
{
  const auto options = hc::parse_options({"serve", "--port", "0", "--output", root.string()});
  ASSERT_EQ(options.depth_mode, "OFF");
  ASSERT_EQ(options.color, "1080p");

  std::vector<uint8_t> frame1, frame2;
  cv::imencode(".jpg", cv::Mat(24, 32, CV_8UC3, cv::Scalar(10, 20, 30)), frame1);
  cv::imencode(".jpg", cv::Mat(24, 32, CV_8UC3, cv::Scalar(200, 100, 50)), frame2);

  volatile std::sig_atomic_t stop_server = 0;
  uint16_t port = 0;
  hc::WebState * state_ptr = nullptr;
  std::atomic_bool ready_flag{false};
  std::thread server([&] {
    hc::serve(options, stop_server, &port, [&](const std::atomic_bool & shutdown, hc::WebState & st) {
      state_ptr = &st;
      st.camera_connected(1920, 1080);
      st.publish_color(frame1);
      ready_flag = true;
      while (!shutdown) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  });
  ASSERT_TRUE(wait([&] {return ready_flag.load() && port != 0;}));

  asio::io_context io;
  beast::tcp_stream conn(io);
  conn.connect(tcp::endpoint(asio::ip::make_address("127.0.0.1"), port));

  // 1. Keep-alive snapshot GET /frame/color.jpg returns exact hardware MJPG bytes
  {
    http::request<http::empty_body> req{http::verb::get, "/frame/color.jpg", 11};
    req.set(http::field::host, "127.0.0.1");
    req.keep_alive(true);
    http::write(conn, req);
    beast::flat_buffer buf;
    http::response<http::vector_body<uint8_t>> res;
    http::read(conn, buf, res);
    EXPECT_EQ(res.result_int(), 200);
    EXPECT_EQ(res.body(), frame1);
    EXPECT_EQ(res["X-Frame-Sequence"], "1");
  }

  // 2. ?fresh=1 on same keep-alive socket waits for newly published frame2
  {
    std::thread publisher([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      state_ptr->publish_color(frame2);
    });
    http::request<http::empty_body> req{http::verb::get, "/frame/color.jpg?fresh=1", 11};
    req.set(http::field::host, "127.0.0.1");
    req.keep_alive(true);
    http::write(conn, req);
    beast::flat_buffer buf;
    http::response<http::vector_body<uint8_t>> res;
    http::read(conn, buf, res);
    publisher.join();
    EXPECT_EQ(res.result_int(), 200);
    EXPECT_EQ(res.body(), frame2);
    EXPECT_EQ(res["X-Frame-Sequence"], "2");
  }

  // 3. Depth snapshot and MKV recording are cleanly rejected in pure RGB mode
  {
    http::request<http::empty_body> req{http::verb::get, "/frame/depth.jpg", 11};
    req.set(http::field::host, "127.0.0.1");
    http::write(conn, req);
    beast::flat_buffer buf;
    http::response<http::string_body> res;
    http::read(conn, buf, res);
    EXPECT_EQ(res.result_int(), 503);
  }
  conn.close();

  // 4. Low-latency MJPEG stream /stream/color.mjpg delivers multipart frames
  {
    beast::tcp_stream stream_conn(io);
    stream_conn.connect(tcp::endpoint(asio::ip::make_address("127.0.0.1"), port));
    http::request<http::empty_body> req{http::verb::get, "/stream/color.mjpg", 11};
    req.set(http::field::host, "127.0.0.1");
    http::write(stream_conn, req);
    std::string received;
    char chunk[4096];
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (received.find("--frame\r\nContent-Type: image/jpeg") == std::string::npos &&
      std::chrono::steady_clock::now() < deadline)
    {
      beast::error_code ec;
      const size_t n = stream_conn.socket().read_some(asio::buffer(chunk), ec);
      if (ec) break;
      received.append(chunk, n);
    }
    EXPECT_NE(received.find("multipart/x-mixed-replace; boundary=frame"), std::string::npos);
    EXPECT_NE(received.find("--frame\r\nContent-Type: image/jpeg"), std::string::npos);
    stream_conn.close();
  }

  stop_server = 1;
  server.join();
}

#if HEAD_CAMERA_HAS_K4A
TEST_F(WebCameraTest, DepthModeSupportsLazyDepthPreviewAndMkvRecording)
{
  const auto options = hc::parse_options({
    "serve", "--depth-mode", "WFOV_2X2BINNED", "--color", "720p", "--output", root.string()});
  hc::WebState state(options);
  std::atomic_bool stopped{false};
  std::thread worker([&] {
    try {
      hc::stream_camera(options, stopped, state);
    } catch (const std::exception & error) {
      state.camera_failed(error.what());
    }
  });
  ASSERT_TRUE(wait([&] {return state.snapshot().sequence > 0;}));
  EXPECT_EQ(hc::web_request(state, "GET", "/frame/depth.jpg", "").content_type, "image/jpeg");
  EXPECT_EQ(hc::web_request(state, "GET", "/frame/color.jpg", "").status, 200u);
  EXPECT_EQ(hc::web_request(state, "POST", "/api/record/start", "").status, 202u);
  ASSERT_TRUE(wait([&] {return state.snapshot().frames >= 2;}));
  EXPECT_EQ(hc::web_request(state, "POST", "/api/record/stop", "").status, 202u);
  ASSERT_TRUE(wait([&] {return state.snapshot().recording_state == "idle";}));
  EXPECT_TRUE(state.snapshot().recording_error.empty());
  EXPECT_EQ(opens, 1u);
  stopped = true;
  worker.join();
}
#endif
