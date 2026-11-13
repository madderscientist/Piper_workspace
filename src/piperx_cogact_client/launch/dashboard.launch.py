"""Launch PiPER-X CogACT local stream collector, MIT executor, and Web Dashboard."""
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _build_process(context):
    def val(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    exe = Path(get_package_prefix("piperx_cogact_client")) / "lib" / "piperx_cogact_client" / "dashboard"
    cmd = [str(exe), "--host", val("host"), "--port", val("port")]
    if val("config"):
        cmd.extend(["--config", val("config")])
    if val("calibration_file"):
        cmd.extend(["--calibration-file", val("calibration_file")])
    if val("no_ros").lower() == "true":
        cmd.append("--no-ros")
    return [ExecuteProcess(cmd=cmd, shell=False, output="screen", respawn=False)]


def generate_launch_description():
    default_cfg = str(Path(get_package_share_directory("piperx_cogact_client")) / "config" / "default.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_cfg, description="Path to client YAML config"),
        DeclareLaunchArgument("calibration_file", default_value="", description="Optional override path for persisted calibration YAML"),
        DeclareLaunchArgument("host", default_value="0.0.0.0", description="Dashboard HTTP bind host"),
        DeclareLaunchArgument("port", default_value="8095", description="Dashboard HTTP bind port"),
        DeclareLaunchArgument("no_ros", default_value="false", description="Run without ROS 2 node for offline UI/API testing"),
        OpaqueFunction(function=_build_process),
    ])

