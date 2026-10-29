from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from piper_x_driver.configuration import default_config_path, FleetConfig


def _teaching(context):
    FleetConfig(LaunchConfiguration('config_file').perform(context))
    parameters = [
        get_package_share_directory('piper_teaching')
        + '/config/bilateral_teaching.yaml',
        {
            'config_file': LaunchConfiguration('config_file'),
        },
        {
            name: ParameterValue(LaunchConfiguration(name), value_type=bool)
            for name in ('dry_run', 'auto_start')
        },
    ]
    mode = LaunchConfiguration('mode').perform(context).strip()
    if mode:
        parameters.append({'mode': mode})
    return [
        Node(
            package='piper_teaching', executable='bilateral_teaching', output='screen',
            parameters=parameters,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value=''),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('config_file', default_value=str(default_config_path())),
        OpaqueFunction(function=_teaching),
    ])
