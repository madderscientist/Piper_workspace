from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def display(model, rviz_config, gui_rate=None):
    share = get_package_share_path('piper_description')
    description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true', choices=['true', 'false'],
            description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(
            'model', default_value=str(share / 'urdf' / model),
            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(
            'rvizconfig', default_value=str(share / 'rviz' / rviz_config),
            description='Absolute path to rviz config file'),
        Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            condition=UnlessCondition(LaunchConfiguration('gui'))),
        Node(
            package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
            parameters=[] if gui_rate is None else [{'rate': gui_rate}],
            condition=IfCondition(LaunchConfiguration('gui'))),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': description}]),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')]),
    ])
