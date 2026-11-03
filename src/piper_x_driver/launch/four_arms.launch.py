from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from piper_x_driver.configuration import default_config_path, FleetConfig


def _drivers(context):
    fleet = FleetConfig(LaunchConfiguration('config_file').perform(context))
    extra = {}
    if 'rate_hz' in context.launch_configurations:
        extra['rate_hz'] = float(LaunchConfiguration('rate_hz').perform(context))
    if 'telemetry_hz' in context.launch_configurations:
        extra['telemetry_hz'] = float(LaunchConfiguration('telemetry_hz').perform(context))
    return [
        Node(
            package='piper_x_driver', executable='driver',
            namespace=arm.namespace, output='screen',
            parameters=[{
                'config_file': str(fleet.path),
                'arm_id': arm.id,
                **extra,
            }],
        ) for arm in fleet.arms
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(default_config_path())),
        DeclareLaunchArgument('rate_hz', default_value='200.0'),
        DeclareLaunchArgument('telemetry_hz', default_value='200.0'),
        OpaqueFunction(function=_drivers),
    ])
