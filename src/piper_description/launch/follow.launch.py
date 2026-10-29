from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from piper_x_driver.configuration import default_config_path, FleetConfig


def _follow(context):
    share = get_package_share_path('piper_description')
    namespace = LaunchConfiguration('namespace')
    model = LaunchConfiguration('model').perform(context)
    if not model:
        fleet = FleetConfig(LaunchConfiguration('config_file').perform(context))
        selected = '/' + namespace.perform(context).strip('/')
        arm = next((arm for arm in fleet.arms if arm.namespace == selected), None)
        if arm is None:
            raise ValueError('Namespace not in fleet configuration; supply an explicit model')
        model = str(share / 'urdf' / fleet.effectors[arm.end_effector].model)
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    description = ParameterValue(
        Command(['xacro ', model]), value_type=str)
    return [
        Node(
            package='piper_description', executable='joint_state_relay',
            namespace=namespace, output='screen'),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=namespace, output='screen',
            parameters=[{'robot_description': description}],
            remappings=[('joint_states', 'joint_states_model'), *tf_remaps]),
        Node(
            package='rviz2', executable='rviz2', namespace=namespace,
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            remappings=[('/robot_description', 'robot_description'), *tf_remaps],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    share = get_package_share_path('piper_description')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='arm_1'),
        DeclareLaunchArgument('config_file', default_value=str(default_config_path())),
        DeclareLaunchArgument('model', default_value=''),
        DeclareLaunchArgument(
            'rvizconfig', default_value=str(share / 'rviz/piper_ctrl.rviz')),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=_follow),
    ])
