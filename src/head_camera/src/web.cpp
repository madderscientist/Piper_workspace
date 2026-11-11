#include "head_camera/web.hpp"

#include <array>
#include <cctype>
#include <cerrno>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>
#include <sys/socket.h>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <yaml-cpp/yaml.h>

namespace head_camera
{
namespace
{
namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using tcp = asio::ip::tcp;

enum class StreamKind { Color, Depth, Calib };

std::string slot_summary_json(const WebState & state)
{
  const auto value = state.snapshot(false);
  const auto saved = state.calib.saved_profile(value.width, value.height);
  std::ostringstream out;
  out << std::boolalpha
      << "{\"name\":" << json_string(value.camera_name)
      << ",\"serial\":" << json_string(value.serial)
      << ",\"video_device\":" << json_string(value.video_device)
      << ",\"camera_ready\":" << value.camera_ready
      << ",\"depth_enabled\":" << value.depth_enabled
      << ",\"color_enabled\":" << value.color_enabled
      << ",\"intrinsic_available\":" << saved.available
      << ",\"intrinsic_rms\":" << saved.rms
      << ",\"width\":" << value.width
      << ",\"height\":" << value.height
      << ",\"fps\":" << state.options.fps
      << ",\"sequence\":" << value.sequence
      << ",\"frame_age\":" << value.frame_age
      << ",\"camera_error\":" << json_string(value.camera_error) << "}";
  return out.str();
}

std::string status_json(
  const WebState & state,
  const std::vector<std::unique_ptr<WebState>> * all_slots = nullptr)
{
  const auto value = state.snapshot(false);
  const auto saved = state.calib.saved_profile(value.width, value.height);
  std::ostringstream out;
  out << std::boolalpha
      << "{\"name\":" << json_string(value.camera_name)
      << ",\"serial\":" << json_string(value.serial)
      << ",\"video_device\":" << json_string(value.video_device)
      << ",\"camera_ready\":" << value.camera_ready
      << ",\"depth_enabled\":" << value.depth_enabled
      << ",\"color_enabled\":" << value.color_enabled
      << ",\"intrinsic_available\":" << saved.available
      << ",\"intrinsic_rms\":" << saved.rms
      << ",\"width\":" << value.width
      << ",\"height\":" << value.height
      << ",\"camera_error\":" << json_string(value.camera_error)
      << ",\"recording_state\":" << json_string(value.recording_state)
      << ",\"recording_error\":" << json_string(value.recording_error)
      << ",\"output\":" << json_string(value.output)
      << ",\"frames\":" << value.frames
      << ",\"seconds\":" << value.seconds
      << ",\"frame_age\":" << value.frame_age
      << ",\"sequence\":" << value.sequence
      << ",\"depth_mode\":" << json_string(state.options.depth_mode)
      << ",\"color_mode\":" << json_string(state.options.color)
      << ",\"fps\":" << state.options.fps
      << ",\"output_root\":" << json_string(fs::absolute(state.options.output).string());
  if (all_slots) {
    out << ",\"cameras\":[";
    for (size_t i = 0; i < all_slots->size(); ++i) {
      if (i) out << ',';
      out << slot_summary_json(*(*all_slots)[i]);
    }
    out << ']';
  }
  out << '}';
  return out.str();
}

WebResponse error_response(unsigned code, const std::string & message)
{
  return {code, "application/json", "{\"error\":" + json_string(message) + "}"};
}

std::string query_param(const std::string & query, const std::string & key)
{
  const std::string prefix = key + "=";
  size_t pos = 0;
  while (pos < query.size()) {
    const size_t amp = query.find('&', pos);
    const auto token = query.substr(pos, amp == std::string::npos ? std::string::npos : amp - pos);
    if (token.rfind(prefix, 0) == 0) {
      return token.substr(prefix.size());
    }
    if (amp == std::string::npos) break;
    pos = amp + 1;
  }
  return "";
}

bool has_fresh_param(const std::string & query)
{
  const auto val = query_param(query, "fresh");
  return val == "1" || val == "true";
}

struct ParsedTarget
{
  std::string camera;
  std::string path;
  std::string query;
};

ParsedTarget parse_target(const std::string & target)
{
  const auto qpos = target.find('?');
  std::string path = target.substr(0, qpos);
  std::string query = qpos == std::string::npos ? std::string() : target.substr(qpos + 1);
  std::string camera = query_param(query, "camera");
  if (path.rfind("/camera/", 0) == 0) {
    const size_t next_slash = path.find('/', 8);
    if (next_slash != std::string::npos && next_slash > 8) {
      if (camera.empty()) {
        camera = path.substr(8, next_slash - 8);
      }
      path = path.substr(next_slash);
    }
  }
  return {std::move(camera), std::move(path), std::move(query)};
}

WebState * select_slot(
  const std::vector<std::unique_ptr<WebState>> & slots, const std::string & selector)
{
  if (slots.empty()) return nullptr;
  if (selector.empty()) return slots.front().get();
  for (const auto & slot : slots) {
    const auto snap = slot->snapshot(false);
    if (snap.camera_name == selector || (!snap.serial.empty() && snap.serial == selector) ||
      (!snap.video_device.empty() && snap.video_device == selector))
    {
      return slot.get();
    }
  }
  bool all_digits = !selector.empty();
  for (const unsigned char ch : selector) {
    if (!std::isdigit(ch)) { all_digits = false; break; }
  }
  if (all_digits) {
    const size_t idx = static_cast<size_t>(std::stoul(selector));
    if (idx < slots.size()) return slots[idx].get();
  }
  return nullptr;
}

class MjpegSession : public std::enable_shared_from_this<MjpegSession>
{
public:
  MjpegSession(
    beast::tcp_stream stream, WebState & state, const std::atomic_bool & stopped, StreamKind kind)
  : stream_(std::move(stream)),
    state_(state),
    stopped_(stopped),
    kind_(kind),
    timer_(stream_.get_executor()) {}

  void start()
  {
    static const std::string header =
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
      "Cache-Control: no-store, no-cache, must-revalidate\r\n"
      "Pragma: no-cache\r\n"
      "Connection: close\r\n\r\n";
    asio::async_write(stream_, asio::buffer(header),
      [self = shared_from_this()](beast::error_code error, size_t) {
        if (error) {
          self->close();
          return;
        }
        self->pump();
      });
  }

private:
  bool peer_disconnected()
  {
    char byte = 0;
    const ssize_t n = ::recv(stream_.socket().native_handle(), &byte, 1, MSG_PEEK | MSG_DONTWAIT);
    if (n == 0) return true;
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) return true;
    return false;
  }

  void pump()
  {
    if (closed_ || stopped_.load() || peer_disconnected()) {
      close();
      return;
    }
    const auto snap = state_.snapshot(kind_ == StreamKind::Depth);
    if (!snap.camera_error.empty()) {
      close();
      return;
    }
    std::shared_ptr<const std::vector<uint8_t>> jpeg;
    if (snap.camera_ready && snap.sequence > sent_seq_) {
      if (kind_ == StreamKind::Depth) {
        jpeg = snap.depth_jpeg;
      } else if (kind_ == StreamKind::Calib) {
        jpeg = state_.calib.overlay_jpeg(snap.sequence, snap.color_jpeg);
      } else {
        jpeg = snap.color_jpeg;
      }
    }
    if (jpeg && !jpeg->empty()) {
      sent_seq_ = snap.sequence;
      auto part_header = std::make_shared<std::string>(
        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
        std::to_string(jpeg->size()) + "\r\n\r\n");
      static constexpr char crlf[] = "\r\n";
      std::array<asio::const_buffer, 3> buffers{
        asio::buffer(*part_header),
        asio::buffer(jpeg->data(), jpeg->size()),
        asio::buffer(crlf, 2),
      };
      asio::async_write(stream_, buffers,
        [self = shared_from_this(), part_header, jpeg](beast::error_code error, size_t) {
          if (error) {
            self->close();
            return;
          }
          self->pump();
        });
      return;
    }

    auto gate = std::make_shared<bool>(false);
    timer_.expires_after(std::chrono::milliseconds(500));
    timer_.async_wait([self = shared_from_this(), gate](beast::error_code error) {
      if (error || *gate) return;
      *gate = true;
      self->pump();
    });
    auto exec = stream_.get_executor();
    state_.wait_next_frame(sent_seq_, [self = shared_from_this(), exec, gate] {
      asio::post(exec, [self, gate] {
        if (*gate) return;
        *gate = true;
        self->timer_.cancel();
        self->pump();
      });
    });
  }

  void close()
  {
    if (closed_) return;
    closed_ = true;
    timer_.cancel();
    beast::error_code ignored;
    stream_.socket().shutdown(tcp::socket::shutdown_both, ignored);
    stream_.socket().close(ignored);
  }

  beast::tcp_stream stream_;
  WebState & state_;
  const std::atomic_bool & stopped_;
  StreamKind kind_;
  asio::steady_timer timer_;
  uint64_t sent_seq_ = 0;
  bool closed_ = false;
};

class HttpSession : public std::enable_shared_from_this<HttpSession>
{
public:
  HttpSession(
    tcp::socket socket, const std::vector<std::unique_ptr<WebState>> & slots,
    const std::string & page, const std::atomic_bool & stopped)
  : stream_(std::move(socket)),
    slots_(slots),
    page_(page),
    stopped_(stopped),
    fresh_timer_(stream_.get_executor()) {}

  void read()
  {
    request_ = {};
    http::async_read(stream_, buffer_, request_,
      [self = shared_from_this()](beast::error_code error, size_t) {
        if (error) {
          if (error != http::error::end_of_stream && error != asio::error::operation_aborted &&
            error != asio::error::connection_reset)
          {
            std::cerr << "HTTP read: " << error.message() << '\n';
          }
          return;
        }
        self->respond();
      });
  }

private:
  void respond()
  {
    const auto parsed = parse_target(request_.target().to_string());
    if (request_.method() == http::verb::get && parsed.path == "/") {
      write_response({200, "text/html; charset=utf-8", page_});
      return;
    }
    WebState * slot = select_slot(slots_, parsed.camera);
    if (!slot) {
      write_response(error_response(404, "unknown camera: " + parsed.camera));
      return;
    }
    auto & state = *slot;
    if (request_.method() == http::verb::get && parsed.path == "/api/status") {
      write_response({200, "application/json", status_json(state, &slots_)});
      return;
    }

    if (request_.method() == http::verb::get &&
      (parsed.path == "/stream/color.mjpg" || parsed.path == "/color.mjpg" ||
       parsed.path == "/stream/depth.mjpg" || parsed.path == "/depth.mjpg" ||
       parsed.path == "/stream/calib.mjpg"))
    {
      const StreamKind kind =
        (parsed.path == "/stream/depth.mjpg" || parsed.path == "/depth.mjpg") ? StreamKind::Depth :
        (parsed.path == "/stream/calib.mjpg") ? StreamKind::Calib : StreamKind::Color;
      const auto snap = state.snapshot(false);
      if (kind == StreamKind::Depth && !snap.depth_enabled) {
        write_response(error_response(503, "depth stream is disabled in pure RGB mode"));
        return;
      }
      if (kind != StreamKind::Depth && !snap.color_enabled) {
        write_response(error_response(503, "color stream is disabled"));
        return;
      }
      if (!snap.camera_error.empty()) {
        write_response(error_response(503, snap.camera_error));
        return;
      }
      std::make_shared<MjpegSession>(std::move(stream_), state, stopped_, kind)->start();
      return;
    }

    if (request_.method() == http::verb::get &&
      (parsed.path == "/frame/color.jpg" || parsed.path == "/color.jpg" ||
       parsed.path == "/frame/depth.jpg" || parsed.path == "/depth.jpg"))
    {
      const bool want_depth = (parsed.path == "/frame/depth.jpg" || parsed.path == "/depth.jpg");
      if (has_fresh_param(parsed.query)) {
        const auto snap = state.snapshot(false);
        if (want_depth && !snap.depth_enabled) {
          write_response(error_response(503, "depth is disabled in pure RGB mode"));
          return;
        }
        if (!want_depth && !snap.color_enabled) {
          write_response(error_response(503, "color is disabled"));
          return;
        }
        if (!snap.camera_error.empty()) {
          write_response(error_response(503, snap.camera_error));
          return;
        }
        const uint64_t start_seq = snap.sequence;
        auto gate = std::make_shared<bool>(false);
        fresh_timer_.expires_after(std::chrono::seconds(3));
        fresh_timer_.async_wait([self = shared_from_this(), gate](beast::error_code error) {
          if (error || *gate) return;
          *gate = true;
          self->write_response(error_response(504, "timed out waiting for fresh camera frame"));
        });
        auto exec = stream_.get_executor();
        WebState * slot_ptr = &state;
        state.wait_next_frame(start_seq, [self = shared_from_this(), exec, gate, slot_ptr, want_depth, start_seq] {
          asio::post(exec, [self, gate, slot_ptr, want_depth, start_seq] {
            if (*gate) return;
            *gate = true;
            self->fresh_timer_.cancel();
            self->write_jpeg_snapshot(*slot_ptr, want_depth, start_seq);
          });
        });
        return;
      }
      write_jpeg_snapshot(state, want_depth, 0);
      return;
    }

    WebResponse value;
    const auto content_type = request_[http::field::content_type].to_string();
    if (request_.method() == http::verb::post &&
      content_type.rfind("application/json", 0) != 0)
    {
      value = error_response(415, "POST requests require Content-Type: application/json");
    } else {
      value = web_request(
        state, request_.method_string().to_string(), parsed.path, page_, request_.body());
    }
    write_response(std::move(value));
  }

  void write_jpeg_snapshot(WebState & state, bool want_depth, uint64_t min_after_seq)
  {
    const auto snap = state.snapshot(want_depth);
    if (want_depth && !snap.depth_enabled) {
      write_response(error_response(503, "depth is disabled in pure RGB mode"));
      return;
    }
    const auto & jpeg = want_depth ? snap.depth_jpeg : snap.color_jpeg;
    if (!snap.camera_ready || snap.sequence <= min_after_seq || !jpeg || jpeg->empty()) {
      write_response(error_response(503, snap.camera_error.empty() ?
        "preview frame is not available" : snap.camera_error));
      return;
    }
    const bool keep_alive = request_.keep_alive();
    std::ostringstream hdr;
    hdr << "HTTP/1." << (request_.version() == 10 ? '0' : '1') << " 200 OK\r\n"
        << "Content-Type: image/jpeg\r\n"
        << "Content-Length: " << jpeg->size() << "\r\n"
        << "Cache-Control: no-store\r\n"
        << "X-Camera-Name: " << snap.camera_name << "\r\n"
        << "X-Camera-Serial: " << snap.serial << "\r\n"
        << "X-Frame-Sequence: " << snap.sequence << "\r\n"
        << "X-Frame-Age-Ms: " << std::fixed << std::setprecision(2) << (snap.frame_age * 1000.0) << "\r\n"
        << "Connection: " << (keep_alive ? "keep-alive" : "close") << "\r\n\r\n";
    auto header = std::make_shared<std::string>(hdr.str());
    std::array<asio::const_buffer, 2> buffers{
      asio::buffer(*header),
      asio::buffer(jpeg->data(), jpeg->size()),
    };
    asio::async_write(stream_, buffers,
      [self = shared_from_this(), header, jpeg, keep_alive](beast::error_code error, size_t) {
        if (error) return;
        if (keep_alive) {
          self->read();
        } else {
          beast::error_code ignored;
          self->stream_.socket().shutdown(tcp::socket::shutdown_send, ignored);
        }
      });
  }

  void write_response(WebResponse value)
  {
    auto response = std::make_shared<http::response<http::string_body>>(
      static_cast<http::status>(value.status), request_.version());
    response->set(http::field::content_type, value.content_type);
    response->set(http::field::cache_control, "no-store");
    response->keep_alive(request_.keep_alive());
    response->body() = std::move(value.body);
    response->prepare_payload();
    http::async_write(stream_, *response,
      [self = shared_from_this(), response](beast::error_code error, size_t) {
        if (error) {
          if (error != asio::error::operation_aborted && error != asio::error::broken_pipe &&
            error != asio::error::connection_reset)
          {
            std::cerr << "HTTP write: " << error.message() << '\n';
          }
          return;
        }
        if (response->keep_alive()) {
          self->read();
        } else {
          beast::error_code ignored;
          self->stream_.socket().shutdown(tcp::socket::shutdown_send, ignored);
        }
      });
  }

  beast::tcp_stream stream_;
  beast::flat_buffer buffer_;
  http::request<http::string_body> request_;
  const std::vector<std::unique_ptr<WebState>> & slots_;
  const std::string & page_;
  const std::atomic_bool & stopped_;
  asio::steady_timer fresh_timer_;
};
}  // namespace

WebState::WebState(const Options & options)
: options(options),
  calib(
    options.intrinsic_file, options.output,
    !options.serial.empty() ? options.serial :
    (!options.camera_name.empty() ? options.camera_name : "default"))
{
  state_.camera_name = options.camera_name.empty() ? "head_1" : options.camera_name;
  state_.serial = options.serial;
  state_.video_device = options.video_device;
  state_.depth_enabled = (options.depth_mode != "OFF");
  state_.color_enabled = (options.color != "OFF");
  const auto [w, h] = color_dimensions(options.color);
  state_.width = w;
  state_.height = h;
}

void WebState::fire_waiters_unlocked(std::vector<std::function<void()>> & waiters)
{
  for (auto & fn : waiters) {
    if (fn) fn();
  }
}

WebSnapshot WebState::snapshot(bool include_depth_jpeg) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (include_depth_jpeg && state_.depth_enabled && !raw_depth_.empty() &&
    depth_jpeg_seq_ != state_.sequence)
  {
    state_.depth_jpeg = std::make_shared<const std::vector<uint8_t>>(depth_preview_jpeg(
      raw_depth_.data(), depth_width_, depth_height_,
      static_cast<size_t>(depth_width_) * sizeof(uint16_t)));
    depth_jpeg_seq_ = state_.sequence;
  }
  auto result = state_;
  const auto now = std::chrono::steady_clock::now();
  if (state_.sequence > 0) {
    result.frame_age = std::chrono::duration<double>(now - frame_time_).count();
  }
  if (state_.recording_state == "recording" || state_.recording_state == "stopping") {
    result.seconds = std::chrono::duration<double>(now - record_start_).count();
  }
  return result;
}

std::string WebState::request_recording(bool start)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!state_.depth_enabled) {
    return "MKV recording requires an active depth mode (e.g. --depth-mode WFOV_2X2BINNED)";
  }
  if (!state_.camera_ready) {
    return state_.camera_error.empty() ? "camera is not ready" : state_.camera_error;
  }
  if (start) {
    if (state_.recording_state != "idle" && state_.recording_state != "error") {
      return "a recording is already active or changing state";
    }
    wanted_ = true;
    state_.recording_state = "starting";
    state_.recording_error.clear();
    state_.frames = 0;
    state_.seconds = 0.0;
    state_.output.clear();
    record_start_ = std::chrono::steady_clock::now();
  } else {
    if (state_.recording_state != "recording" && state_.recording_state != "starting") {
      return "no active recording to stop";
    }
    wanted_ = false;
    state_.recording_state = "stopping";
  }
  return "";
}

bool WebState::wants_recording() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return wanted_;
}

void WebState::camera_connected(
  int width, int height, const std::string & dev_path, const std::string & serial)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.camera_ready = true;
  state_.camera_error.clear();
  if (width > 0 && height > 0) {
    state_.width = width;
    state_.height = height;
  }
  if (!dev_path.empty()) state_.video_device = dev_path;
  if (!serial.empty()) state_.serial = serial;
}

void WebState::camera_failed(const std::string & error)
{
  std::vector<std::function<void()>> ready;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.camera_ready = false;
    state_.camera_error = error;
    state_.depth_jpeg.reset();
    state_.color_jpeg.reset();
    raw_depth_.clear();
    if (state_.recording_state == "starting" || state_.recording_state == "recording" ||
      state_.recording_state == "stopping")
    {
      state_.recording_state = "error";
      state_.recording_error = error;
    }
    wanted_ = false;
    ready.swap(waiters_);
  }
  fire_waiters_unlocked(ready);
}

void WebState::publish_color(std::vector<uint8_t> color)
{
  auto color_data = std::make_shared<const std::vector<uint8_t>>(std::move(color));
  std::vector<std::function<void()>> ready;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.color_jpeg = std::move(color_data);
    ++state_.sequence;
    frame_time_ = std::chrono::steady_clock::now();
    ready.swap(waiters_);
  }
  fire_waiters_unlocked(ready);
}

void WebState::publish_rgbd(
  std::vector<uint16_t> raw_depth, int depth_width, int depth_height,
  std::vector<uint8_t> color)
{
  auto color_data = std::make_shared<const std::vector<uint8_t>>(std::move(color));
  std::vector<std::function<void()>> ready;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    raw_depth_ = std::move(raw_depth);
    depth_width_ = depth_width;
    depth_height_ = depth_height;
    state_.color_jpeg = std::move(color_data);
    ++state_.sequence;
    frame_time_ = std::chrono::steady_clock::now();
    ready.swap(waiters_);
  }
  fire_waiters_unlocked(ready);
}

void WebState::wait_next_frame(uint64_t after_sequence, std::function<void()> callback)
{
  bool immediate = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.camera_error.empty() || (state_.camera_ready && state_.sequence > after_sequence)) {
      immediate = true;
    } else {
      waiters_.push_back(callback);
    }
  }
  if (immediate && callback) {
    callback();
  }
}

void WebState::notify_all()
{
  std::vector<std::function<void()>> ready;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ready.swap(waiters_);
  }
  fire_waiters_unlocked(ready);
}

void WebState::recording_started(const fs::path & directory)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.recording_state = wanted_ ? "recording" : "stopping";
  state_.output = (directory / "depth.mkv").string();
  record_start_ = std::chrono::steady_clock::now();
}

void WebState::recording_progress(uint64_t count)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.frames = count;
}

void WebState::recording_finished(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.seconds = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - record_start_).count();
  state_.recording_state = error.empty() ? "idle" : "error";
  state_.recording_error = error;
  wanted_ = false;
}

WebResponse web_request(
  WebState & state, const std::string & method, const std::string & target,
  const std::string & page, const std::string & body)
{
  const auto path = target.substr(0, target.find('?'));
  if (method == "GET" && path == "/") {
    return {200, "text/html; charset=utf-8", page};
  }
  if (method == "GET" && path == "/api/status") {
    return {200, "application/json", status_json(state)};
  }
  if (method == "GET" && path == "/api/intrinsic") {
    const auto snap = state.snapshot(false);
    return {200, "application/json", state.calib.intrinsic_json(snap.width, snap.height)};
  }
  if (method == "GET" && path == "/api/calib") {
    const auto snap = state.snapshot(false);
    return {200, "application/json", state.calib.status_json(snap.width, snap.height)};
  }
  if (method == "POST" && path.rfind("/api/calib/", 0) == 0) {
    try {
      const auto snap = state.snapshot(false);
      if (path == "/api/calib/board") {
        const auto payload = YAML::Load(body.empty() ? "{}" : body);
        auto cfg = state.calib.board();
        if (payload["squares_x"]) cfg.squares_x = payload["squares_x"].as<int>();
        if (payload["squares_y"]) cfg.squares_y = payload["squares_y"].as<int>();
        if (payload["square_size"]) cfg.square_size = payload["square_size"].as<double>();
        if (payload["square_size_mm"]) cfg.square_size = payload["square_size_mm"].as<double>() / 1000.0;
        if (payload["marker_size"]) cfg.marker_size = payload["marker_size"].as<double>();
        if (payload["marker_size_mm"]) cfg.marker_size = payload["marker_size_mm"].as<double>() / 1000.0;
        if (payload["dictionary"]) cfg.dictionary = payload["dictionary"].as<std::string>();
        state.calib.set_board(cfg);
        return {200, "application/json", state.calib.status_json(snap.width, snap.height)};
      }
      if (path == "/api/calib/capture") {
        if (!snap.camera_ready || !snap.color_jpeg || snap.color_jpeg->empty()) {
          return error_response(503, "彩色相机未就绪，暂时无法采集标定帧");
        }
        state.calib.capture_shot(*snap.color_jpeg);
        return {200, "application/json", state.calib.status_json(snap.width, snap.height)};
      }
      if (path == "/api/calib/delete") {
        const auto payload = YAML::Load(body.empty() ? "{}" : body);
        if (!payload["name"]) {
          return error_response(400, "missing shot name");
        }
        state.calib.delete_shot(payload["name"].as<std::string>(), snap.width, snap.height);
        return {200, "application/json", state.calib.status_json(snap.width, snap.height)};
      }
      if (path == "/api/calib/clear") {
        state.calib.clear_shots(snap.width, snap.height);
        return {200, "application/json", state.calib.status_json(snap.width, snap.height)};
      }
      if (path == "/api/calib/solve") {
        state.calib.solve_and_save(snap.width, snap.height);
        return {200, "application/json", state.calib.status_json(snap.width, snap.height)};
      }
      return error_response(404, "unknown calibration route");
    } catch (const std::invalid_argument & error) {
      return error_response(400, error.what());
    } catch (const std::exception & error) {
      return error_response(422, error.what());
    }
  }
  if (method == "GET" && (path == "/frame/depth.jpg" || path == "/depth.jpg" ||
      path == "/frame/color.jpg" || path == "/color.jpg"))
  {
    const bool want_depth = (path == "/frame/depth.jpg" || path == "/depth.jpg");
    const auto value = state.snapshot(want_depth);
    if (want_depth && !value.depth_enabled) {
      return error_response(503, "depth is disabled in pure RGB mode");
    }
    const auto & jpeg = want_depth ? value.depth_jpeg : value.color_jpeg;
    if (!value.camera_ready || !jpeg || jpeg->empty()) {
      return error_response(503, value.camera_error.empty() ?
        "preview frame is not available" : value.camera_error);
    }
    return {200, "image/jpeg", std::string(jpeg->begin(), jpeg->end())};
  }
  if (method == "POST" && (path == "/api/record/start" || path == "/api/record/stop")) {
    const auto error = state.request_recording(path == "/api/record/start");
    if (!error.empty()) {
      return error_response(409, error);
    }
    return {202, "application/json", status_json(state)};
  }
  return error_response(404, "unknown route");
}

void serve(
  const Options & options, const volatile std::sig_atomic_t & stopped,
  uint16_t * bound_port,
  std::function<void(const std::atomic_bool &, WebState &)> producer)
{
  const auto page_path = fs::path(ament_index_cpp::get_package_share_directory("head_camera")) /
    "web" / "index.html";
  std::ifstream page_file(page_path);
  if (!page_file) {
    throw std::runtime_error("could not read web page: " + page_path.string());
  }
  const std::string page(
    (std::istreambuf_iterator<char>(page_file)), std::istreambuf_iterator<char>());

  std::vector<std::unique_ptr<WebState>> slots;
  if (producer) {
    slots.push_back(std::make_unique<WebState>(options));
  } else {
    for (const auto & target : resolve_serve_targets(options)) {
      slots.push_back(std::make_unique<WebState>(target));
    }
  }

  asio::io_context io;
  tcp::acceptor acceptor(io);
  const auto endpoint = tcp::endpoint(asio::ip::make_address(options.host), options.port);
  acceptor.open(endpoint.protocol());
  acceptor.set_option(tcp::acceptor::reuse_address(true));
  beast::error_code bind_ec;
  acceptor.bind(endpoint, bind_ec);
  if (bind_ec) {
    if (bind_ec == asio::error::address_in_use) {
      throw std::runtime_error("port " + std::to_string(options.port) +
        " is already in use; stop existing head_camera serve or pass --port " +
        std::to_string(options.port + 1));
    }
    throw std::runtime_error("binding HTTP port failed: " + bind_ec.message());
  }
  acceptor.listen();
  const uint16_t actual_port = acceptor.local_endpoint().port();
  if (bound_port) {
    *bound_port = actual_port;
  }
  std::cout << "Head camera HTTP: http://" << options.host << ':' << actual_port << "/ [";
  for (size_t i = 0; i < slots.size(); ++i) {
    if (i) std::cout << ", ";
    const auto s = slots[i]->snapshot(false);
    std::cout << s.camera_name;
    if (!s.serial.empty()) std::cout << "(" << s.serial << ")";
  }
  std::cout << "]\n" << std::flush;

  std::atomic_bool shutdown(false);
  std::vector<std::thread> workers;
  for (auto & slot : slots) {
    WebState * st = slot.get();
    workers.emplace_back([&, st] {
      if (producer) {
        try {
          producer(shutdown, *st);
        } catch (const std::exception & error) {
          st->camera_failed(error.what());
        }
        return;
      }
      while (!shutdown) {
        try {
          stream_camera(st->options, shutdown, *st);
          if (shutdown) break;
        } catch (const std::exception & error) {
          std::cerr << "[" << st->options.camera_name << "] " << error.what() << '\n';
          st->camera_failed(error.what());
        }
        for (int i = 0; i < 20 && !shutdown; ++i) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    });
  }

  std::function<void()> accept_next;
  accept_next = [&] {
    acceptor.async_accept([&](beast::error_code error, tcp::socket socket) {
      if (!error) {
        beast::error_code nodelay_ec;
        socket.set_option(tcp::no_delay(true), nodelay_ec);
        std::make_shared<HttpSession>(std::move(socket), slots, page, shutdown)->read();
      } else if (error != asio::error::operation_aborted) {
        std::cerr << "HTTP accept: " << error.message() << '\n';
      }
      if (acceptor.is_open()) {
        accept_next();
      }
    });
  };
  asio::steady_timer timer(io);
  std::function<void()> check_stop;
  check_stop = [&] {
    timer.expires_after(std::chrono::milliseconds(50));
    timer.async_wait([&](beast::error_code error) {
      if (error) return;
      if (stopped) {
        shutdown = true;
        for (auto & s : slots) s->notify_all();
        beast::error_code ignored;
        acceptor.close(ignored);
        io.stop();
      } else {
        check_stop();
      }
    });
  };
  auto join_all = [&] {
    shutdown = true;
    for (auto & s : slots) s->notify_all();
    for (auto & w : workers) {
      if (w.joinable()) w.join();
    }
  };
  try {
    accept_next();
    check_stop();
    io.run();
  } catch (const std::exception &) {
    join_all();
    throw;
  }
  join_all();
}
}  // namespace head_camera
