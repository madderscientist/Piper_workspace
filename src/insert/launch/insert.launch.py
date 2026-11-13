from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from piper_x_driver.configuration import default_config_path


def _launch_nodes(context):
    config_file = LaunchConfiguration('config_file').perform(context)
    arm_id = LaunchConfiguration('arm_id').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    host = LaunchConfiguration('host').perform(context)
    port = LaunchConfiguration('port').perform(context)
    wrench_source = LaunchConfiguration('wrench_source').perform(context)
    start_driver = LaunchConfiguration('start_driver').perform(context).lower() in ('1', 'true', 'yes')

    nodes = []
    if start_driver:
        nodes.append(
            Node(
                package='piper_x_driver',
                executable='driver',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'config_file': config_file,
                    'arm_id': arm_id,
                }],
            )
        )
    nodes.append(
        Node(
            package='insert',
            executable='web',
            output='screen',
            arguments=[
                '--config', config_file,
                '--arm', arm_id,
                '--namespace', namespace,
                '--host', host,
                '--port', port,
                '--wrench-source', wrench_source,
            ],
        )
    )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(default_config_path())),
        DeclareLaunchArgument('arm_id', default_value='arm_4'),
        DeclareLaunchArgument('namespace', default_value='/arm_4'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8092'),
        DeclareLaunchArgument('wrench_source', default_value='paxini'),
        DeclareLaunchArgument('start_driver', default_value='false'),
        OpaqueFunction(function=_launch_nodes),
    ])
