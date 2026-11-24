import os
from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


_EFFECTOR_MODELS = {
    'teaching_handle': 'piper_description_with_teach.urdf',
    'gripper': 'piper_description.urdf',
    'gripper_with_wrench': 'piper_description_with_wrench.urdf',
}


def _default_config_file() -> str:
    root = os.environ.get('PIPER_CONFIG_DIR')
    return str(Path(root) / 'arms') if root else ''


def _follow(context):
    share = get_package_share_path('piper_description')
    namespace = LaunchConfiguration('namespace')
    model = LaunchConfiguration('model').perform(context)
    if not model:
        cfg_dir = LaunchConfiguration('config_file').perform(context)
        if not cfg_dir or not Path(cfg_dir).is_dir():
            raise ValueError('Supply model:=... or a valid arm directory config_file:=...')
        selected = '/' + namespace.perform(context).strip('/')
        arms = (
            yaml.safe_load(path.read_text(encoding='utf-8')) or {}
            for path in sorted(Path(cfg_dir).glob('*.yaml'))
        )
        arm = next((a for a in arms if a.get('namespace') == selected), None)
        if arm is None:
            raise ValueError('Namespace not in fleet configuration; supply an explicit model')
        model_name = _EFFECTOR_MODELS.get(arm['end_effector'])
        if not model_name:
            raise ValueError(f"Unknown end_effector: {arm['end_effector']}")
        model = str(share / 'urdf' / model_name)
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
        DeclareLaunchArgument('config_file', default_value=_default_config_file()),
        DeclareLaunchArgument('model', default_value=''),
        DeclareLaunchArgument(
            'rvizconfig', default_value=str(share / 'rviz/piper_ctrl.rviz')),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=_follow),
    ])
