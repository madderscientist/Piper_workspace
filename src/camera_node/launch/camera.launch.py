"""PiPER-X 腕部 IP 相机启动文件。

- 默认不传 arm / camera（或传 arm:=all）：按 config/cameras.yaml 同时启动 arm_3 (.97) 与 arm_4 (.98) 两台腕部相机；
- 传入 arm:=arm_3 或 camera:=arm4_camera（支持别名与 IP）：只启动所选的那一台腕部相机。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from camera_node.camera_node import default_cameras_config_path, resolve_camera_selection

_ARGS = {
    'config_file': (str(default_cameras_config_path()), str),
    'arm': ('', str),
    'camera': ('', str),
    'name': ('', str),
    'rtsp_url': ('', str),
    'image_topic': ('', str),
    'camera_info_topic': ('', str),
    'calib_file': ('', str),
    'frame_id': ('', str),
    'image_width': ('0', int),
    'image_height': ('0', int),
    'fps': ('0', int),
    'server_port': ('0', int),
    'jpeg_quality': ('60', int),
}


def _launch_camera(context):
    config_file = LaunchConfiguration('config_file').perform(context).strip()
    arm_raw = LaunchConfiguration('arm').perform(context).strip()
    cam_raw = LaunchConfiguration('camera').perform(context).strip()
    name_raw = LaunchConfiguration('name').perform(context).strip()
    url_raw = LaunchConfiguration('rtsp_url').perform(context).strip()
    topic_raw = LaunchConfiguration('image_topic').perform(context).strip()
    info_raw = LaunchConfiguration('camera_info_topic').perform(context).strip()
    frame_raw = LaunchConfiguration('frame_id').perform(context).strip()
    calib_file = LaunchConfiguration('calib_file').perform(context).strip()
    width = int(LaunchConfiguration('image_width').perform(context))
    height = int(LaunchConfiguration('image_height').perform(context))
    fps = int(LaunchConfiguration('fps').perform(context))
    server_port = int(LaunchConfiguration('server_port').perform(context))
    jpeg_quality = int(LaunchConfiguration('jpeg_quality').perform(context))

    # 若显式传了自定义 rtsp_url 且未传 arm/camera，则按单台自定义相机启动
    if url_raw and not arm_raw and not cam_raw:
        node_name = name_raw or 'camera'
        return [
            Node(
                package='camera_node',
                executable='camera_node',
                name=node_name,
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'config_file': config_file,
                    'rtsp_url': url_raw,
                    'image_topic': topic_raw or '~/image_raw',
                    'camera_info_topic': info_raw or '~/camera_info',
                    'calib_file': calib_file,
                    'frame_id': frame_raw or node_name,
                    'image_width': width,
                    'image_height': height,
                    'fps': fps,
                    'server_port': server_port,
                    'jpeg_quality': jpeg_quality,
                }],
            )
        ]

    # 若通过 name 指定了某台配置里的相机别名，也视为单相机选择
    effective_cam = cam_raw or (name_raw if not arm_raw else '')
    selected = resolve_camera_selection(
        arm=arm_raw, camera=effective_cam, config_file=config_file or None)
    single = len(selected) == 1

    nodes = []
    for idx, entry in enumerate(selected):
        node_name = (name_raw if single and name_raw else entry['name'])
        rtsp_url = (url_raw if single and url_raw else entry['rtsp_url'])
        image_topic = (topic_raw if single and topic_raw else entry.get('image_topic', '~/image_raw'))
        info_topic = (info_raw if single and info_raw else entry.get('camera_info_topic', '~/camera_info'))
        frame_id = (frame_raw if single and frame_raw else entry.get('frame_id', node_name))
        port = (server_port + idx if server_port > 0 and not single
                else (server_port if server_port > 0 else int(entry.get('server_port', 0))))
        nodes.append(
            Node(
                package='camera_node',
                executable='camera_node',
                name=node_name,
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'config_file': config_file,
                    'arm': entry.get('arm', ''),
                    'camera': entry['name'],
                    'rtsp_url': rtsp_url,
                    'image_topic': image_topic,
                    'camera_info_topic': info_topic,
                    'calib_file': calib_file,
                    'frame_id': frame_id,
                    'image_width': width,
                    'image_height': height,
                    'fps': fps,
                    'server_port': port,
                    'jpeg_quality': jpeg_quality,
                }],
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default)
          for name, (default, _) in _ARGS.items()),
        OpaqueFunction(function=_launch_camera),
    ])
