#include "head_camera/calibration.hpp"
#include "head_camera/web.hpp"

#include <exception>
#include <iostream>

namespace
{
volatile std::sig_atomic_t stopped = 0;

void stop(int)
{
  stopped = 1;
}

constexpr const char * help = R"(Azure Kinect head camera
Usage:
  head_camera list
  head_camera serve [--camera all|NAME|SERIAL] [--host ADDRESS] [--port PORT] [camera options]
  head_camera calibrate --input DIRECTORY [--camera SERIAL|NAME] [--intrinsic FILE.yaml] [board options]
  head_camera capture [--camera NAME|SERIAL] [--count N] [--interval SECONDS] [camera options]
  head_camera record [--duration SECONDS] [camera options]
  head_camera export --input FILE.mkv [--output DIRECTORY] [--preview]

Camera selection & options:
  --camera SELECTOR     'all' (serve default: start all connected cameras), alias (e.g. head_1),
                        12-digit hardware serial, or comma-separated list (e.g. head_1,head_2)
  --serial SERIAL       Explicit 12-digit Azure Kinect hardware serial number
  --device INDEX        Device index from 'list' (default: 0)
  --video-device PATH   Explicit V4L2 device path (e.g. /dev/video0)
  --depth-mode MODE     OFF (pure RGB via V4L2), WFOV_2X2BINNED, WFOV_UNBINNED,
                        NFOV_UNBINNED or NFOV_2X2BINNED
  --fps RATE            5, 15 or 30 (default: 30; WFOV_UNBINNED max: 15)
  --color MODE          OFF, 720p (1280x720) or 1080p (1920x1080)
  --output DIRECTORY    Parent directory (default: head_camera_data)

Board options (for calibrate; defaults match 12x9 / 15mm / 11.25mm / DICT_5X5_100):
  --squares-x N         Chessboard squares in X (default: 12)
  --squares-y N         Chessboard squares in Y (default: 9)
  --square-size METERS  Square side length in meters (default: 0.015)
  --marker-size METERS  Marker side length in meters (default: 0.01125)
  --dictionary NAME     ArUco dictionary (default: DICT_5X5_100)
  --intrinsic FILE.yaml Intrinsic YAML path (default: config/intrinsic.yaml)
)";
}  // namespace

int main(int argc, char ** argv)
{
  std::vector<std::string> args(argv + 1, argv + argc);
  for (const auto & arg : args) {
    if (arg == "--help" || arg == "-h") {
      std::cout << help;
      return 0;
    }
  }
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  try {
    const auto options = head_camera::parse_options(args);
    if (options.command == "list") {
      head_camera::list_devices();
      return 0;
    }
    if (options.command == "serve") {
      head_camera::serve(options, stopped);
      return stopped ? 130 : 0;
    }
    if (options.command == "calibrate") {
      const auto solved = head_camera::calibrate_directory(options);
      const auto target = options.intrinsic_file.empty() ?
        head_camera::default_intrinsic_path() : options.intrinsic_file;
      std::cout << "Calibrated " << solved.width << 'x' << solved.height
                << " from " << solved.images << " images (RMS: " << solved.rms << " px)\n"
                << "Saved: " << target << '\n';
      return 0;
    }
    const auto result = options.command == "export" ?
      head_camera::export_recording(options, stopped) : head_camera::acquire(options, stopped);
    std::cout << (result.interrupted ? "Stopped" : "Completed") << ": " << result.frames
              << " frames\nOutput: " << result.directory << '\n';
    return result.interrupted ? 130 : 0;
  } catch (const std::exception & error) {
    std::cerr << "head_camera: " << error.what() << '\n';
    return 1;
  }
}
