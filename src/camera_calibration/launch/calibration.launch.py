"""PiPER-X 单臂相机标定 dashboard + 外参 static TF 发布。

启动时必须显式指定 arm（arm_3 或 arm_4）和 camera（如 camera_arm_3 或 camera_arm_4），
一次只标定所选的那一条机械臂及其腕部相机。
可选 start_camera:=true 一并拉起对应腕部相机的 camera_node。
"""

from pathlib import Path
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from camera_calibration.sources import select_arm_camera
from camera_node.camera_node import CYCLONEDDS_DEFAULT_URI

_DEFAULTS = {
    'arm': ('', str),
    'camera': ('', str),
    'start_camera': ('false', str),
    'dashboard_port': ('8300', int),
    'data_root': ('~/camera_calib_data', str),
    'board_config': ('', str),
    'cameras_config': ('', str),
    'calib_file': ('', str),
    'base_frame': ('base_link', str),
    'preview_width': ('720', int),
    'jpeg_quality': ('75', int),
    'detect_period_s': ('0.15', float),
}


def _launch_nodes(context):
    os.environ.setdefault('CYCLONEDDS_URI', CYCLONEDDS_DEFAULT_URI)
    share = Path(get_package_share_directory('camera_calibration')) / 'config'
    cameras_cfg_path = LaunchConfiguration('cameras_config').perform(context).strip()
    cfg_file = Path(cameras_cfg_path).expanduser() if cameras_cfg_path else (share / 'cameras.yaml')
    cameras_config = yaml.safe_load(cfg_file.read_text(encoding='utf-8')) or {}

    arm_raw = LaunchConfiguration('arm').perform(context).strip()
    cam_raw = LaunchConfiguration('camera').perform(context).strip()
    arm_id, camera_name, camera_cfg = select_arm_camera(cameras_config, arm_raw, cam_raw)

    calib_file = LaunchConfiguration('calib_file').perform(context).strip()
    start_cam = LaunchConfiguration('start_camera').perform(context).strip().lower() in ('1', 'true', 'yes')

    params = {
        'arm': arm_id,
        'camera': camera_name,
        'start_camera': start_cam,
        'dashboard_port': int(LaunchConfiguration('dashboard_port').perform(context)),
        'data_root': LaunchConfiguration('data_root').perform(context),
        'board_config': LaunchConfiguration('board_config').perform(context),
        'cameras_config': cameras_cfg_path,
        'calib_file': calib_file,
        'base_frame': LaunchConfiguration('base_frame').perform(context),
        'preview_width': int(LaunchConfiguration('preview_width').perform(context)),
        'jpeg_quality': int(LaunchConfiguration('jpeg_quality').perform(context)),
        'detect_period_s': float(LaunchConfiguration('detect_period_s').perform(context)),
    }
    nodes = []
    nodes.append(
        Node(
            package='camera_calibration',
            executable='calib_node',
            name='camera_calibration',
            output='screen',
            parameters=[params],
        )
    )
    nodes.append(
        Node(
            package='camera_calibration',
            executable='calib_tf_node',
            name='camera_calib_tf',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_tf')),
            parameters=[{'calib_file': calib_file}],
        )
    )
    return nodes


def generate_launch_description() -> LaunchDescription:
    arguments = [DeclareLaunchArgument(name, default_value=default)
                 for name, (default, _) in _DEFAULTS.items()]
    arguments.append(DeclareLaunchArgument(
        'publish_tf', default_value='true',
        description='同时把已标定的外参发成 static TF'))
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_nodes)])
