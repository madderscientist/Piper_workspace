from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from piper_x_driver.configuration import default_config_path


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(default_config_path())),
        Node(
            package='piper_x_driver',
            executable='power_off',
            output='screen',
            parameters=[{'config_file': LaunchConfiguration('config_file')}],
        ),
    ])
