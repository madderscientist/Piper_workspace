"""Launch file for the PaXini PX6D 6-axis force/torque sensor driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from paxini_driver.configuration import default_config_path, resolve_sensor_configs

_NODE_KEYS = (
    "arm_id",
    "interface",
    "serial_port",
    "expected_version",
    "baudrate",
    "can_channel",
    "device_id",
    "model",
    "frame_id",
    "wrench_topic",
    "raw_wrench_topic",
    "status_topic",
    "acquisition_mode",
    "frequency_hz",
    "poll_rate_hz",
    "autostart",
    "calibrate_on_start",
    "reconnect_sec",
    "stream_timeout_sec",
    "require_hardware",
    "filter_mode",
    "filter_window",
    "filter_alpha",
    "tare_offset",
    "tool_mass_kg",
    "tool_com_m",
)


def _launch_nodes(context):
    config_file = LaunchConfiguration("config_file").perform(context).strip()
    arm = LaunchConfiguration("arm").perform(context).strip()
    iface = LaunchConfiguration("interface").perform(context).strip()
    serial_port = LaunchConfiguration("serial_port").perform(context).strip()
    can_channel = LaunchConfiguration("can_channel").perform(context).strip()
    namespace = LaunchConfiguration("namespace").perform(context).strip()
    device_id = LaunchConfiguration("device_id").perform(context).strip()
    frequency_hz = LaunchConfiguration("frequency_hz").perform(context).strip()
    frame_id = LaunchConfiguration("frame_id").perform(context).strip()
    wrench_topic = LaunchConfiguration("wrench_topic").perform(context).strip()

    selected = resolve_sensor_configs(arm=arm, config_file=config_file or None)
    single = len(selected) == 1
    nodes = []
    for entry in selected:
        params = {k: entry[k] for k in _NODE_KEYS if k in entry and entry[k] is not None}
        for k in ("tare_offset", "tool_com_m"):
            if k in params:
                params[k] = [float(x) for x in params[k]]
        params["config_file"] = config_file or str(default_config_path())
        if iface:
            params["interface"] = iface
        if frequency_hz:
            params["frequency_hz"] = int(frequency_hz, 0)
        if device_id:
            params["device_id"] = int(device_id, 0)
        if frame_id:
            params["frame_id"] = frame_id
        if wrench_topic:
            params["wrench_topic"] = wrench_topic
        if single and serial_port:
            params["serial_port"] = serial_port
        if single and can_channel:
            params["can_channel"] = can_channel
        ns = namespace if (single and namespace) else str(entry.get("namespace", ""))
        nodes.append(
            Node(
                package="paxini_driver",
                executable="driver",
                name="paxini_driver",
                namespace=ns,
                output="screen",
                parameters=[params],
            )
        )
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file", default_value=str(default_config_path())
            ),
            DeclareLaunchArgument("arm", default_value="all"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("interface", default_value=""),
            DeclareLaunchArgument("serial_port", default_value=""),
            DeclareLaunchArgument("can_channel", default_value=""),
            DeclareLaunchArgument("device_id", default_value=""),
            DeclareLaunchArgument("frequency_hz", default_value=""),
            DeclareLaunchArgument("frame_id", default_value=""),
            DeclareLaunchArgument("wrench_topic", default_value=""),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
