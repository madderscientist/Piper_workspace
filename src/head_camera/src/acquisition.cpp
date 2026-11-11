#include "head_camera/head_camera.hpp"
#include "head_camera/web.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <linux/videodev2.h>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

namespace head_camera
{
namespace
{
int xioctl(int fd, unsigned long request, void * arg)
{
  int r = 0;
  do {
    r = ::ioctl(fd, request, arg);
  } while (r == -1 && errno == EINTR);
  return r;
}

std::string read_uvc_serial(const std::string & dev_name)
{
  std::error_code ec;
  fs::path cur = fs::canonical(fs::path("/sys/class/video4linux") / dev_name / "device", ec);
  if (ec) return "";
  for (int depth = 0; !cur.empty() && depth < 4; ++depth, cur = cur.parent_path()) {
    const fs::path serial_file = cur / "serial";
    std::error_code file_ec;
    if (fs::is_regular_file(serial_file, file_ec)) {
      std::ifstream in(serial_file);
      std::string val;
      std::getline(in, val);
      while (!val.empty() && std::isspace(static_cast<unsigned char>(val.back()))) {
        val.pop_back();
      }
      if (!val.empty()) {
        return val;
      }
    }
  }
  return "";
}

std::string trim_copy(const std::string & s)
{
  size_t b = 0;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
  size_t e = s.size();
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
  return s.substr(b, e - b);
}

std::string alias_for_serial(const std::vector<CameraSpec> & specs, const std::string & serial)
{
  if (serial.empty()) return "";
  for (const auto & spec : specs) {
    if (spec.serial == serial && !spec.name.empty()) {
      return spec.name;
    }
  }
  return "";
}

UvcDeviceInfo resolve_uvc_device(const Options & options)
{
  const auto devices = discover_uvc_devices();
  if (!options.serial.empty()) {
    for (const auto & d : devices) {
      if (d.serial == options.serial) {
        return d;
      }
    }
    throw std::runtime_error("camera with serial " + options.serial + " not found; check USB connection");
  }
  if (!options.camera.empty() && options.camera != "all" && options.camera != "both" && options.camera != "*") {
    for (const auto & spec : options.cameras) {
      if (spec.name == options.camera) {
        if (!spec.serial.empty()) {
          for (const auto & d : devices) {
            if (d.serial == spec.serial) return d;
          }
          throw std::runtime_error("camera " + spec.name + " (serial " + spec.serial + ") not found");
        }
        if (!spec.video_device.empty()) {
          return {0, spec.video_device, spec.name, "", "", false};
        }
      }
    }
    for (const auto & d : devices) {
      if (d.serial == options.camera || d.path == options.camera) {
        return d;
      }
    }
  }
  if (!options.video_device.empty()) {
    for (const auto & d : devices) {
      if (d.path == options.video_device) return d;
    }
    return {0, options.video_device, "", "", "", false};
  }
  if (options.device >= devices.size()) {
    throw std::runtime_error("UVC MJPG camera index " + std::to_string(options.device) +
      " not found; check USB connection or run 'head_camera list'");
  }
  return devices[options.device];
}

class UvcCamera
{
public:
  explicit UvcCamera(const Options & options)
  {
    const auto info = resolve_uvc_device(options);
    path_ = info.path;
    serial_ = info.serial;
    fd_ = ::open(path_.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
      throw std::runtime_error("opening " + path_ + " failed: " + std::strerror(errno));
    }
    try {
      const auto [req_w, req_h] = color_dimensions(options.color);
      v4l2_format fmt{};
      fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      fmt.fmt.pix.width = static_cast<uint32_t>(req_w);
      fmt.fmt.pix.height = static_cast<uint32_t>(req_h);
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
      if (xioctl(fd_, VIDIOC_S_FMT, &fmt) != 0) {
        throw std::runtime_error("setting V4L2 MJPG format on " + path_ + " failed");
      }
      if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG ||
        static_cast<int>(fmt.fmt.pix.width) != req_w ||
        static_cast<int>(fmt.fmt.pix.height) != req_h)
      {
        throw std::runtime_error(path_ + " rejected MJPG " +
          std::to_string(req_w) + "x" + std::to_string(req_h));
      }
      width_ = static_cast<int>(fmt.fmt.pix.width);
      height_ = static_cast<int>(fmt.fmt.pix.height);

      v4l2_streamparm parm{};
      parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      parm.parm.capture.timeperframe.numerator = 1;
      parm.parm.capture.timeperframe.denominator = static_cast<uint32_t>(options.fps);
      xioctl(fd_, VIDIOC_S_PARM, &parm);

      v4l2_requestbuffers req{};
      req.count = 3;
      req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      req.memory = V4L2_MEMORY_MMAP;
      if (xioctl(fd_, VIDIOC_REQBUFS, &req) != 0 || req.count < 2) {
        throw std::runtime_error("allocating V4L2 mmap buffers on " + path_ + " failed");
      }
      buffers_.resize(req.count);
      for (uint32_t i = 0; i < req.count; ++i) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) != 0) {
          throw std::runtime_error("querying V4L2 buffer failed");
        }
        void * ptr = ::mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
        if (ptr == MAP_FAILED) {
          throw std::runtime_error("mmap V4L2 buffer failed");
        }
        buffers_[i] = {ptr, buf.length};
      }
      for (uint32_t i = 0; i < req.count; ++i) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (xioctl(fd_, VIDIOC_QBUF, &buf) != 0) {
          throw std::runtime_error("queueing V4L2 buffer failed");
        }
      }
      v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (xioctl(fd_, VIDIOC_STREAMON, &type) != 0) {
        if (errno == ENOSPC) {
          throw std::runtime_error("starting stream on " + path_ +
            " failed (ENOSPC: USB 2.0 hub bandwidth full; move one camera to a USB 3.0 port or separate USB controller)");
        }
        throw std::runtime_error("starting V4L2 stream on " + path_ + " failed: " + std::strerror(errno));
      }
      streaming_ = true;
    } catch (...) {
      cleanup();
      throw;
    }
  }

  ~UvcCamera() {cleanup();}

  int width() const {return width_;}
  int height() const {return height_;}
  const std::string & path() const {return path_;}
  const std::string & serial() const {return serial_;}

  std::vector<uint8_t> poll_jpeg(uint64_t & timestamp_usec)
  {
    pollfd pfd{fd_, POLLIN, 0};
    const int r = ::poll(&pfd, 1, 100);
    if (r == 0 || (r < 0 && errno == EINTR)) {
      return {};
    }
    if (r < 0 || (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
      throw std::runtime_error("V4L2 camera disconnected or failed on " + path_);
    }
    v4l2_buffer latest{};
    bool have_latest = false;
    while (true) {
      v4l2_buffer buf{};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      if (xioctl(fd_, VIDIOC_DQBUF, &buf) != 0) {
        if (errno == EAGAIN) {
          break;
        }
        throw std::runtime_error("VIDIOC_DQBUF failed on " + path_);
      }
      if (have_latest) {
        xioctl(fd_, VIDIOC_QBUF, &latest);
      }
      latest = buf;
      have_latest = true;
    }
    if (!have_latest) {
      return {};
    }
    std::vector<uint8_t> jpeg;
    if (!(latest.flags & V4L2_BUF_FLAG_ERROR) && latest.bytesused >= 4 &&
      latest.index < buffers_.size() && latest.bytesused <= buffers_[latest.index].length)
    {
      const auto * ptr = static_cast<const uint8_t *>(buffers_[latest.index].data);
      size_t len = latest.bytesused;
      while (len > 2 && ptr[len - 1] == 0) {
        --len;
      }
      if (len >= 4 && ptr[0] == 0xff && ptr[1] == 0xd8 &&
        ptr[len - 2] == 0xff && ptr[len - 1] == 0xd9)
      {
        jpeg.assign(ptr, ptr + len);
        timestamp_usec = static_cast<uint64_t>(latest.timestamp.tv_sec) * 1000000ULL +
          static_cast<uint64_t>(latest.timestamp.tv_usec);
      }
    }
    if (xioctl(fd_, VIDIOC_QBUF, &latest) != 0) {
      throw std::runtime_error("VIDIOC_QBUF failed on " + path_);
    }
    return jpeg;
  }

private:
  struct MappedBuffer
  {
    void * data = nullptr;
    size_t length = 0;
  };

  void cleanup()
  {
    if (streaming_ && fd_ >= 0) {
      v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(fd_, VIDIOC_STREAMOFF, &type);
      streaming_ = false;
    }
    for (auto & buf : buffers_) {
      if (buf.data && buf.data != MAP_FAILED) {
        ::munmap(buf.data, buf.length);
      }
    }
    buffers_.clear();
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  std::string path_;
  std::string serial_;
  int fd_ = -1;
  int width_ = 0;
  int height_ = 0;
  bool streaming_ = false;
  std::vector<MappedBuffer> buffers_;
};

#if HEAD_CAMERA_HAS_K4A
template<typename Reader>
std::vector<uint8_t> read_blob(Reader reader, const std::string & name)
{
  size_t size = 0;
  if (reader(nullptr, &size) != K4A_BUFFER_RESULT_TOO_SMALL || size == 0) {
    throw std::runtime_error("could not read " + name + " size");
  }
  std::vector<uint8_t> buffer(size);
  if (reader(buffer.data(), &size) != K4A_BUFFER_RESULT_SUCCEEDED) {
    throw std::runtime_error("could not read " + name);
  }
  buffer.resize(size);
  while (!buffer.empty() && buffer.back() == 0) {
    buffer.pop_back();
  }
  return buffer;
}

std::string serial_number(k4a_device_t device)
{
  const auto bytes = read_blob(
    [device](uint8_t * data, size_t * size) {
      return k4a_device_get_serialnum(device, reinterpret_cast<char *>(data), size);
    }, "device serial number");
  return std::string(bytes.begin(), bytes.end());
}

class Camera
{
public:
  explicit Camera(uint32_t index) : device_(nullptr, k4a_device_close)
  {
    if (index >= k4a_device_get_installed_count()) {
      throw std::runtime_error("Azure Kinect device " + std::to_string(index) +
        " not found; check USB 3, external power and 'head_camera list'");
    }
    k4a_device_t handle = nullptr;
    check(k4a_device_open(index, &handle), "opening Azure Kinect depth device (check external power and USB 3, or use --depth-mode OFF)");
    device_.reset(handle);
  }

  ~Camera()
  {
    if (started_) {
      k4a_device_stop_cameras(device_.get());
    }
  }

  k4a_device_t handle() const {return device_.get();}

  void start(const k4a_device_configuration_t & config)
  {
    check(k4a_device_start_cameras(handle(), &config), "starting Azure Kinect cameras");
    started_ = true;
  }

  Capture poll()
  {
    k4a_capture_t handle = nullptr;
    const auto result = k4a_device_get_capture(device_.get(), &handle, 100);
    Capture capture(handle, k4a_capture_release);
    if (result == K4A_WAIT_RESULT_FAILED) {
      throw std::runtime_error("capturing frame failed; check device connection");
    }
    return capture;
  }

private:
  Device device_;
  bool started_ = false;
};
#endif

class Dataset
{
public:
  Dataset(
    const Options & options, const std::vector<uint8_t> & calibration,
    const std::string & serial)
  : directory(create_session(options.output, options.command)), preview_(options.preview)
  {
    metadata_["format_version"] = 1;
    metadata_["operation"] = options.command;
    metadata_["depth_mode"] = options.depth_mode;
    metadata_["fps"] = options.fps;
    metadata_["color"] = options.color;
    metadata_["timestamp_unit"] = "microsecond";
    metadata_["timestamp_clock"] = "device";
    if (!serial.empty()) {
      metadata_["serial_number"] = serial;
    }
    if (options.depth_mode != "OFF") {
      metadata_["depth_encoding"] = "16UC1";
      metadata_["depth_unit"] = "millimeter";
      metadata_["invalid_depth_value"] = 0;
      metadata_["depth_geometry"] = "native_depth_camera";
    }
    if (!calibration.empty()) {
      metadata_["calibration_file"] = "calibration.json";
      write_text(directory / "calibration.json", std::string(calibration.begin(), calibration.end()));
    }
    if (options.command == "export") {
      metadata_["source_mkv"] = fs::absolute(options.input).string();
    } else {
      metadata_["device_index"] = options.device;
      metadata_["requested_count"] = options.count;
      metadata_["interval_seconds"] = options.interval;
      metadata_["duration_seconds"] = options.duration;
    }
    if (options.command == "record") {
      metadata_["recording_file"] = "depth.mkv";
    }
    frames_.exceptions(std::ios::failbit | std::ios::badbit);
    frames_.open(directory / "frames.csv");
    frames_ << "index,depth_timestamp_usec,color_timestamp_usec,depth_file,color_file,"
            << "depth_width,depth_height\n";
    finish("running");
    std::cout << "Output: " << directory << std::endl;
  }

  void append_color_jpeg(const std::vector<uint8_t> & jpeg, uint64_t timestamp, int width, int height)
  {
    std::ostringstream stem;
    stem << std::setfill('0') << std::setw(6) << count << '_' << timestamp;
    const std::string color_file = "color_" + stem.str() + ".jpg";
    write_text(directory / color_file, std::string(reinterpret_cast<const char *>(jpeg.data()), jpeg.size()));
    frames_ << count << ",," << timestamp << ",," << color_file << ",,\n";
    frames_.flush();
    metadata_["color_width"] = width;
    metadata_["color_height"] = height;
    ++count;
  }

#if HEAD_CAMERA_HAS_K4A
  void append(k4a_capture_t capture, bool images)
  {
    Image depth(k4a_capture_get_depth_image(capture), k4a_image_release);
    Image color(k4a_capture_get_color_image(capture), k4a_image_release);
    if (!depth) {
      throw std::runtime_error("capture contains no depth image");
    }
    const auto timestamp = k4a_image_get_device_timestamp_usec(depth.get());
    const auto width = k4a_image_get_width_pixels(depth.get());
    const auto height = k4a_image_get_height_pixels(depth.get());
    std::ostringstream stem;
    stem << std::setfill('0') << std::setw(6) << count << '_' << timestamp;
    std::string depth_file, color_file;
    if (images) {
      depth_file = "depth_" + stem.str() + ".png";
      save_depth(directory / depth_file, depth.get(), preview_);
      if (color) {
        const auto format = k4a_image_get_format(color.get());
        if (format == K4A_IMAGE_FORMAT_COLOR_MJPG) {
          color_file = "color_" + stem.str() + ".jpg";
          write_text(directory / color_file, std::string(
            reinterpret_cast<const char *>(k4a_image_get_buffer(color.get())),
            k4a_image_get_size(color.get())));
        } else if (format == K4A_IMAGE_FORMAT_COLOR_BGRA32) {
          color_file = "color_" + stem.str() + ".png";
          const cv::Mat image(
            k4a_image_get_height_pixels(color.get()), k4a_image_get_width_pixels(color.get()),
            CV_8UC4, k4a_image_get_buffer(color.get()),
            static_cast<size_t>(k4a_image_get_stride_bytes(color.get())));
          if (!cv::imwrite((directory / color_file).string(), image)) {
            throw std::runtime_error("could not write color image: " + color_file);
          }
        } else {
          throw std::runtime_error("unsupported color format; decode playback to BGRA32");
        }
      }
    }
    frames_ << count << ',' << timestamp << ',';
    if (color) {
      frames_ << k4a_image_get_device_timestamp_usec(color.get());
    }
    frames_ << ',' << depth_file << ',' << color_file << ',' << width << ',' << height << '\n';
    frames_.flush();
    metadata_["depth_width"] = width;
    metadata_["depth_height"] = height;
    ++count;
  }
#endif

  void finish(const std::string & state, const std::string & error = "")
  {
    metadata_["state"] = state;
    metadata_["frame_count"] = count;
    if (!error.empty()) {
      metadata_["error"] = error;
    }
    YAML::Emitter emitter;
    emitter << metadata_;
    if (!emitter.good()) {
      throw std::runtime_error("could not serialize session metadata: " + emitter.GetLastError());
    }
    write_text(directory / "metadata.yaml", emitter.c_str());
    frames_.flush();
  }

  fs::path directory;
  uint64_t count = 0;

private:
  bool preview_;
  YAML::Node metadata_;
  std::ofstream frames_;
};

void mark_failure(Dataset & data, const std::exception & original)
{
  std::cerr << "Partial output: " << data.directory << '\n';
  try {
    data.finish("error", original.what());
  } catch (const std::exception & error) {
    std::cerr << "Could not update failure metadata: " << error.what() << '\n';
  }
}
}  // namespace

std::vector<UvcDeviceInfo> discover_uvc_devices()
{
  std::vector<std::pair<int, std::string>> candidates;
  std::error_code ec;
  for (const auto & entry : fs::directory_iterator("/dev", ec)) {
    const auto name = entry.path().filename().string();
    if (name.rfind("video", 0) != 0 || name.size() <= 5) {
      continue;
    }
    bool digits = true;
    for (size_t i = 5; i < name.size(); ++i) {
      if (!std::isdigit(static_cast<unsigned char>(name[i]))) {
        digits = false;
        break;
      }
    }
    if (digits) {
      candidates.emplace_back(std::stoi(name.substr(5)), entry.path().string());
    }
  }
  std::sort(candidates.begin(), candidates.end());

  std::vector<UvcDeviceInfo> devices;
  for (const auto & [num, path] : candidates) {
    const int fd = ::open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
      continue;
    }
    v4l2_capability cap{};
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
      const uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ?
        cap.device_caps : cap.capabilities;
      if ((caps & V4L2_CAP_VIDEO_CAPTURE) && (caps & V4L2_CAP_STREAMING)) {
        bool has_mjpg = false;
        for (uint32_t idx = 0; ; ++idx) {
          v4l2_fmtdesc fmt{};
          fmt.index = idx;
          fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          if (xioctl(fd, VIDIOC_ENUM_FMT, &fmt) != 0) {
            break;
          }
          if (fmt.pixelformat == V4L2_PIX_FMT_MJPEG) {
            has_mjpg = true;
            break;
          }
        }
        if (has_mjpg) {
          std::string card(reinterpret_cast<const char *>(cap.card));
          std::string bus(reinterpret_cast<const char *>(cap.bus_info));
          const bool kinect = card.find("Azure Kinect") != std::string::npos;
          const std::string dev_name = "video" + std::to_string(num);
          devices.push_back({num, path, std::move(card), read_uvc_serial(dev_name), std::move(bus), kinect});
        }
      }
    }
    ::close(fd);
  }
  std::sort(devices.begin(), devices.end(), [](const UvcDeviceInfo & a, const UvcDeviceInfo & b) {
    if (a.is_kinect != b.is_kinect) return a.is_kinect > b.is_kinect;
    if (!a.serial.empty() && !b.serial.empty() && a.serial != b.serial) return a.serial < b.serial;
    return a.number < b.number;
  });
  return devices;
}

std::vector<Options> resolve_serve_targets(const Options & options)
{
  if (options.depth_mode != "OFF") {
    auto single = options;
    if (single.camera_name.empty()) {
      single.camera_name = "head_" + std::to_string(single.device + 1);
    }
    return {single};
  }

  const auto uvc = discover_uvc_devices();
  if (options.device_explicit) {
    auto single = options;
    try {
      const auto info = resolve_uvc_device(options);
      single.video_device = info.path;
      single.serial = info.serial;
      single.camera_name = alias_for_serial(options.cameras, info.serial);
    } catch (...) {
    }
    if (single.camera_name.empty()) {
      single.camera_name = !single.serial.empty() ? single.serial :
        ("head_" + std::to_string(single.device + 1));
    }
    return {single};
  }

  const std::string sel = trim_copy(options.camera);
  if (sel.empty() || sel == "all" || sel == "both" || sel == "*") {
    if (uvc.empty()) {
      auto fallback = options;
      fallback.camera_name = "head_1";
      return {fallback};
    }
    std::vector<Options> targets;
    std::vector<std::string> used_names;
    for (size_t i = 0; i < uvc.size(); ++i) {
      if (uvc.front().is_kinect && !uvc[i].is_kinect) {
        continue;
      }
      auto t = options;
      t.device = static_cast<uint32_t>(i);
      t.serial = uvc[i].serial;
      t.video_device = uvc[i].path;
      std::string name = alias_for_serial(options.cameras, uvc[i].serial);
      if (name.empty()) {
        for (size_t n = 1; ; ++n) {
          const std::string cand = "head_" + std::to_string(n);
          if (std::find(used_names.begin(), used_names.end(), cand) == used_names.end() &&
            std::find_if(options.cameras.begin(), options.cameras.end(),
              [&](const CameraSpec & s) {return s.name == cand;}) == options.cameras.end())
          {
            name = cand;
            break;
          }
        }
      }
      used_names.push_back(name);
      t.camera_name = name;
      targets.push_back(std::move(t));
    }
    return targets;
  }

  std::vector<Options> targets;
  std::stringstream ss(sel);
  std::string item;
  while (std::getline(ss, item, ',')) {
    const auto tok = trim_copy(item);
    if (tok.empty()) continue;
    auto t = options;
    t.camera = tok;
    bool matched_spec = false;
    for (const auto & spec : options.cameras) {
      if (spec.name == tok || (!spec.serial.empty() && spec.serial == tok)) {
        t.camera_name = spec.name;
        t.serial = spec.serial;
        t.video_device = spec.video_device;
        matched_spec = true;
        break;
      }
    }
    if (!matched_spec) {
      bool matched_live = false;
      for (size_t i = 0; i < uvc.size(); ++i) {
        if (uvc[i].serial == tok || uvc[i].path == tok || std::to_string(i) == tok) {
          t.serial = uvc[i].serial;
          t.video_device = uvc[i].path;
          t.camera_name = alias_for_serial(options.cameras, uvc[i].serial);
          if (t.camera_name.empty()) {
            t.camera_name = !uvc[i].serial.empty() ? uvc[i].serial : ("head_" + std::to_string(i + 1));
          }
          matched_live = true;
          break;
        }
      }
      if (!matched_live) {
        t.camera_name = tok;
        if (tok.rfind("/dev/", 0) == 0) {
          t.video_device = tok;
        } else {
          t.serial = tok;
        }
      }
    }
    targets.push_back(std::move(t));
  }
  if (targets.empty()) {
    throw std::invalid_argument("empty --camera selector");
  }
  return targets;
}

void list_devices()
{
  const auto specs = load_camera_specs(default_config_path());
  const auto uvc = discover_uvc_devices();
  for (size_t i = 0; i < uvc.size(); ++i) {
    const auto alias = alias_for_serial(specs, uvc[i].serial);
    std::cout << i << ": [" << (alias.empty() ? "new" : alias) << "] "
              << "serial=" << (uvc[i].serial.empty() ? "unknown" : uvc[i].serial)
              << "  " << uvc[i].path << "  (" << uvc[i].card;
    if (!uvc[i].bus_info.empty()) {
      std::cout << ", " << uvc[i].bus_info;
    }
    std::cout << ")\n";
  }
#if HEAD_CAMERA_HAS_K4A
  ::setenv("K4A_ENABLE_LOG_TO_STDOUT", "0", 0);
  const auto k4a_count = k4a_device_get_installed_count();
  for (uint32_t index = 0; index < k4a_count; ++index) {
    k4a_device_t handle = nullptr;
    if (k4a_device_open(index, &handle) == K4A_RESULT_SUCCEEDED) {
      Device dev(handle, k4a_device_close);
      std::cout << "K4A " << index << ": serial=" << serial_number(dev.get()) << " (depth ready)\n";
    }
  }
  if (uvc.empty() && k4a_count == 0) {
    std::cout << "No head camera devices detected.\n";
  }
#else
  if (uvc.empty()) {
    std::cout << "No UVC MJPG camera devices detected.\n";
  }
#endif
}

Result acquire(const Options & options, const volatile std::sig_atomic_t & stopped)
{
  if (options.depth_mode == "OFF") {
    UvcCamera camera(options);
    Dataset data(options, {}, camera.serial().empty() ? camera.path() : camera.serial());
    try {
      auto last_frame = std::chrono::steady_clock::now();
      auto last_warning = last_frame;
      uint64_t last_saved_timestamp = 0;
      while (!stopped) {
        uint64_t timestamp = 0;
        auto jpeg = camera.poll_jpeg(timestamp);
        if (jpeg.empty()) {
          const auto now = std::chrono::steady_clock::now();
          if (now - last_frame >= std::chrono::seconds(5) &&
            now - last_warning >= std::chrono::seconds(5))
          {
            std::cerr << "Waiting for UVC RGB frames on " << camera.path() << "\n";
            last_warning = now;
          }
          continue;
        }
        last_frame = std::chrono::steady_clock::now();
        if (data.count > 0 && (timestamp < last_saved_timestamp ||
          static_cast<double>(timestamp - last_saved_timestamp) / 1000000.0 < options.interval))
        {
          continue;
        }
        data.append_color_jpeg(jpeg, timestamp, camera.width(), camera.height());
        last_saved_timestamp = timestamp;
        if (data.count >= options.count) {
          break;
        }
      }
      if (data.count == 0) {
        throw std::runtime_error("no RGB frames were captured");
      }
      data.finish(stopped ? "stopped" : "completed");
    } catch (const std::exception & error) {
      mark_failure(data, error);
      throw;
    }
    return {data.directory, data.count, stopped != 0};
  }

#if HEAD_CAMERA_HAS_K4A
  const auto config = device_configuration(options);
  Camera camera(options.device);
  const auto calibration = read_blob(
    [&camera](uint8_t * data, size_t * size) {
      return k4a_device_get_raw_calibration(camera.handle(), data, size);
    }, "device calibration");
  Dataset data(options, calibration, serial_number(camera.handle()));
  Recording recording(nullptr, k4a_record_close);
  try {
    if (options.command == "record") {
      k4a_record_t handle = nullptr;
      check(k4a_record_create(
        (data.directory / "depth.mkv").c_str(), camera.handle(), config, &handle),
        "creating MKV recording");
      recording.reset(handle);
      check(k4a_record_write_header(recording.get()), "writing MKV header");
    }
    camera.start(config);
    const auto start = std::chrono::steady_clock::now();
    auto last_frame = start;
    auto last_warning = start;
    uint64_t last_saved_timestamp = 0;
    while (!stopped) {
      const auto now = std::chrono::steady_clock::now();
      if (options.command == "record" && options.duration > 0.0 &&
        std::chrono::duration<double>(now - start).count() >= options.duration)
      {
        break;
      }
      auto capture = camera.poll();
      if (!capture) {
        const auto elapsed = std::chrono::steady_clock::now();
        if (elapsed - last_frame >= std::chrono::seconds(5) &&
          elapsed - last_warning >= std::chrono::seconds(5))
        {
          std::cerr << "Waiting for camera frames; check USB, power and SDK diagnostics.\n";
          last_warning = elapsed;
        }
        continue;
      }
      last_frame = std::chrono::steady_clock::now();
      Image depth(k4a_capture_get_depth_image(capture.get()), k4a_image_release);
      if (!depth) {
        throw std::runtime_error("camera returned a capture without depth");
      }
      const auto timestamp = k4a_image_get_device_timestamp_usec(depth.get());
      if (options.command == "capture" && data.count > 0 &&
        (timestamp < last_saved_timestamp ||
        static_cast<double>(timestamp - last_saved_timestamp) / 1000000.0 < options.interval))
      {
        continue;
      }
      if (recording) {
        check(k4a_record_write_capture(recording.get(), capture.get()), "writing MKV capture");
      }
      data.append(capture.get(), !recording);
      last_saved_timestamp = timestamp;
      if (options.command == "capture" && data.count >= options.count) {
        break;
      }
    }
    if (recording) {
      check(k4a_record_flush(recording.get()), "flushing MKV recording");
      recording.reset();
    }
    if (data.count == 0) {
      throw std::runtime_error("no depth frames were captured");
    }
    data.finish(stopped ? "stopped" : "completed");
  } catch (const std::exception & error) {
    if (recording) {
      if (k4a_record_flush(recording.get()) != K4A_RESULT_SUCCEEDED) {
        std::cerr << "Could not flush partial MKV; recording may be incomplete.\n";
      }
      recording.reset();
    }
    mark_failure(data, error);
    throw;
  }
  return {data.directory, data.count, stopped != 0};
#else
  throw std::runtime_error("depth modes require Azure Kinect SDK; use --depth-mode OFF");
#endif
}

Result export_recording(const Options & options, const volatile std::sig_atomic_t & stopped)
{
#if HEAD_CAMERA_HAS_K4A
  k4a_playback_t handle = nullptr;
  check(k4a_playback_open(options.input.c_str(), &handle), "opening input MKV");
  Playback playback(handle, k4a_playback_close);
  k4a_record_configuration_t config{};
  check(k4a_playback_get_record_configuration(playback.get(), &config), "reading MKV configuration");
  if (!config.depth_track_enabled) {
    throw std::runtime_error("input MKV has no depth track");
  }
  const auto calibration = read_blob(
    [&playback](uint8_t * data, size_t * size) {
      return k4a_playback_get_raw_calibration(playback.get(), data, size);
    }, "recording calibration");
  auto source = options;
  source.depth_mode = depth_mode_name(config.depth_mode);
  source.fps = frame_rate(config.camera_fps);
  source.color = config.color_track_enabled ? "recorded_color" : "OFF";
  if (config.color_track_enabled) {
    check(k4a_playback_set_color_conversion(playback.get(), K4A_IMAGE_FORMAT_COLOR_BGRA32),
      "configuring playback color decoding");
  }
  Dataset data(source, calibration, "");
  try {
    while (!stopped) {
      k4a_capture_t next = nullptr;
      const auto result = k4a_playback_get_next_capture(playback.get(), &next);
      Capture capture(next, k4a_capture_release);
      if (result == K4A_STREAM_RESULT_EOF) {
        break;
      }
      if (result != K4A_STREAM_RESULT_SUCCEEDED || !capture) {
        throw std::runtime_error("reading MKV capture failed");
      }
      Image depth(k4a_capture_get_depth_image(capture.get()), k4a_image_release);
      if (depth) {
        data.append(capture.get(), true);
      }
    }
    if (data.count == 0) {
      throw std::runtime_error("input MKV contains no readable depth frames");
    }
    data.finish(stopped ? "stopped" : "completed");
  } catch (const std::exception & error) {
    mark_failure(data, error);
    throw;
  }
  return {data.directory, data.count, stopped != 0};
#else
  (void)options;
  (void)stopped;
  throw std::runtime_error("export requires Azure Kinect SDK");
#endif
}

void stream_camera(const Options & options, const std::atomic_bool & stopped, WebState & state)
{
  if (options.depth_mode == "OFF") {
    UvcCamera camera(options);
    state.camera_connected(camera.width(), camera.height(), camera.path(), camera.serial());
    auto last_frame = std::chrono::steady_clock::now();
    auto last_warning = last_frame;
    while (!stopped) {
      uint64_t timestamp = 0;
      auto jpeg = camera.poll_jpeg(timestamp);
      if (jpeg.empty()) {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_frame >= std::chrono::seconds(5) &&
          now - last_warning >= std::chrono::seconds(5))
        {
          std::cerr << "Waiting for live UVC RGB frames on " << camera.path() << "\n";
          last_warning = now;
        }
        continue;
      }
      last_frame = std::chrono::steady_clock::now();
      state.publish_color(std::move(jpeg));
    }
    return;
  }

#if HEAD_CAMERA_HAS_K4A
  const auto config = device_configuration(options);
  Camera camera(options.device);
  const auto calibration = read_blob(
    [&camera](uint8_t * data, size_t * size) {
      return k4a_device_get_raw_calibration(camera.handle(), data, size);
    }, "device calibration");
  const auto serial = serial_number(camera.handle());
  camera.start(config);
  const auto [cw, ch] = color_dimensions(options.color);
  state.camera_connected(cw, ch, "", serial);
  std::unique_ptr<Dataset> data;
  Recording recording(nullptr, k4a_record_close);
  auto finish = [&](const std::string & error) {
      std::string failure = error;
      if (recording) {
        if (k4a_record_flush(recording.get()) != K4A_RESULT_SUCCEEDED) {
          failure += (failure.empty() ? "" : "; ") + std::string("could not flush MKV");
          std::cerr << failure << '\n';
        }
        recording.reset();
      }
      if (data) {
        if (data->count == 0 && failure.empty()) {
          failure = "no depth frames were recorded";
        }
        try {
          data->finish(failure.empty() ? "completed" : "error", failure);
        } catch (const std::exception & metadata_error) {
          failure += (failure.empty() ? "" : "; ") + std::string(metadata_error.what());
          std::cerr << "Recording metadata: " << metadata_error.what() << '\n';
        }
        data.reset();
      }
      if (!failure.empty()) {
        std::cerr << "Web recording: " << failure << '\n';
      }
      state.recording_finished(failure);
    };
  auto last_frame = std::chrono::steady_clock::now();
  auto last_warning = last_frame;
  try {
    while (!stopped) {
      if (state.wants_recording() && !recording) {
        try {
          auto settings = options;
          settings.command = "record";
          settings.duration = 0.0;
          data = std::make_unique<Dataset>(settings, calibration, serial);
          k4a_record_t handle = nullptr;
          check(k4a_record_create(
            (data->directory / "depth.mkv").c_str(), camera.handle(), config, &handle),
            "creating web recording");
          recording.reset(handle);
          check(k4a_record_write_header(recording.get()), "writing web recording header");
          state.recording_started(data->directory);
        } catch (const std::exception & error) {
          finish(error.what());
        }
      } else if (!state.wants_recording() &&
        (recording || state.snapshot().recording_state == "stopping"))
      {
        finish("");
      }
      auto capture = camera.poll();
      if (!capture) {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_frame >= std::chrono::seconds(5) &&
          now - last_warning >= std::chrono::seconds(5))
        {
          std::cerr << "Waiting for live camera frames; check USB and power.\n";
          last_warning = now;
        }
        continue;
      }
      last_frame = std::chrono::steady_clock::now();
      if (recording) {
        try {
          check(k4a_record_write_capture(recording.get(), capture.get()), "writing web recording");
          data->append(capture.get(), false);
          state.recording_progress(data->count);
        } catch (const std::exception & error) {
          finish(error.what());
        }
      }
      Image depth(k4a_capture_get_depth_image(capture.get()), k4a_image_release);
      Image color(k4a_capture_get_color_image(capture.get()), k4a_image_release);
      std::vector<uint16_t> raw_depth;
      int dw = 0;
      int dh = 0;
      if (depth) {
        dw = k4a_image_get_width_pixels(depth.get());
        dh = k4a_image_get_height_pixels(depth.get());
        const auto stride = static_cast<size_t>(k4a_image_get_stride_bytes(depth.get()));
        const auto * src = k4a_image_get_buffer(depth.get());
        raw_depth.resize(static_cast<size_t>(dw) * static_cast<size_t>(dh));
        for (int r = 0; r < dh; ++r) {
          std::memcpy(
            raw_depth.data() + static_cast<size_t>(r) * static_cast<size_t>(dw),
            src + static_cast<size_t>(r) * stride,
            static_cast<size_t>(dw) * sizeof(uint16_t));
        }
      }
      std::vector<uint8_t> color_jpeg;
      if (color) {
        const auto * buffer = k4a_image_get_buffer(color.get());
        color_jpeg.assign(buffer, buffer + k4a_image_get_size(color.get()));
      }
      state.publish_rgbd(std::move(raw_depth), dw, dh, std::move(color_jpeg));
    }
    if (recording || state.snapshot().recording_state == "starting" ||
      state.snapshot().recording_state == "stopping")
    {
      finish("");
    }
  } catch (const std::exception & error) {
    if (recording || data) {
      finish(error.what());
    }
    throw;
  }
#else
  (void)stopped;
  (void)state;
  throw std::runtime_error("depth modes require Azure Kinect SDK; use --depth-mode OFF");
#endif
}
}  // namespace head_camera
