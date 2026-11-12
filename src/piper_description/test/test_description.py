import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Any
from unittest.mock import patch
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_path
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from urdf_parser_py.urdf import URDF
import xacro
import yaml

from piper_description.model_state import model_joint_states


SHARE = get_package_share_path('piper_description')
MODELS = sorted(path for path in (SHARE / 'urdf').glob('*') if path.is_file())


def _xacro_xml(path: Path) -> str:
    doc: Any = xacro.process_file(str(path))
    return doc.toxml()


@pytest.mark.parametrize('entry,model,config,rate', [
    ('piper_with_gripper/display_urdf.launch.py',
     'piper_description.urdf', 'piper_ctrl.rviz', 200.0),
    ('piper_with_gripper/display_xacro.launch.py',
     'piper_description.xacro', 'piper_ctrl.rviz', 200.0),
    ('piper_no_gripper/display_no_gripper_urdf.launch.py',
     'piper_no_gripper_description.urdf', 'piper_no_gripper.rviz', None),
    ('piper_no_gripper/display_no_gripper_xacro.launch.py',
     'piper_no_gripper_description.xacro', 'piper_no_gripper.rviz', None),
    ('piper_with_teach/display_with_teach_urdf.launch.py',
     'piper_description_with_teach.urdf', 'piper_ctrl.rviz', None),
])
@pytest.mark.parametrize('gui', ['true', 'false'])
def test_display_entries_preserve_defaults_gui_conditions_and_rates(
    entry, model, config, rate, gui,
):
    spec = importlib.util.spec_from_file_location('display_entry', SHARE / 'launch' / entry)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with patch('piper_description.display.Node', wraps=Node) as create_node:
        description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations == {
        'gui': 'true',
        'model': str(SHARE / 'urdf' / model),
        'rvizconfig': str(SHARE / 'rviz' / config),
    }
    context.launch_configurations['gui'] = gui
    nodes = [call.kwargs for call in create_node.call_args_list]
    assert [node['package'] for node in nodes
            if 'condition' not in node or node['condition'].evaluate(context)] == [
        'joint_state_publisher_gui' if gui == 'true' else 'joint_state_publisher',
        'robot_state_publisher', 'rviz2',
    ]
    gui_node = next(node for node in nodes if node['package'] == 'joint_state_publisher_gui')
    assert gui_node['parameters'] == ([] if rate is None else [{'rate': rate}])


@pytest.mark.parametrize('model', MODELS, ids=lambda model: model.name)
def test_models_parse_and_resolve_all_resources(model):
    text = _xacro_xml(model)
    parsed = URDF.from_xml_string(text)
    assert parsed.get_root() in ('base_link', 'world')
    assert {f'joint{index}' for index in range(1, 7)} <= set(parsed.joint_map)
    root = ET.fromstring(text)
    for mesh in root.findall('.//mesh'):
        filename = mesh.attrib['filename']
        if filename.startswith('package://piper_description/'):
            path = SHARE / filename.removeprefix('package://piper_description/')
        else:
            assert filename.startswith('file://')
            path = Path(filename.removeprefix('file://'))
        assert path.is_file(), filename
    for parameters in root.findall('.//plugin/parameters'):
        assert parameters.text and Path(parameters.text.strip()).is_file()
    assert 'piper_gazebo' not in text
    assert '/home/a/piper_ros' not in text


def test_gripper_with_wrench_stacks_measured_parts_and_keeps_finger_alignment():
    text = _xacro_xml(SHARE / 'urdf/paxini/gripper_with_wrench.xacro')
    robot = URDF.from_xml_string(text)
    assert robot.get_root() == 'arm_sensor_flange'
    assert set(robot.link_map) == {
        'arm_sensor_flange', 'wrench_sensor', 'sensor_gripper_flange',
        'gripper_base', 'link7', 'link8'}
    arm_joint = robot.joint_map['arm_flange_to_sensor']
    assert arm_joint.origin.xyz == [0, 0, 0.0075]
    assert arm_joint.origin.rpy == pytest.approx([0, 0, -math.pi / 4])
    assert robot.joint_map['sensor_to_gripper_flange'].origin.xyz == [0, 0, 0.024]
    assert robot.joint_map['sensor_to_gripper_flange'].origin.rpy == pytest.approx(
        [0, 0, math.pi / 2])
    assert robot.joint_map['flange_to_gripper_base'].origin.xyz == [0, 0, 0.0088]
    assert (robot.joint_map['sensor_to_gripper_flange'].origin.rpy[2]
            + arm_joint.origin.rpy[2]) == pytest.approx(math.pi / 4)
    mass = {link.name: link.inertial.mass for link in robot.links if link.inertial}
    assert mass['arm_sensor_flange'] == pytest.approx(0.242 / 9.81)
    assert mass['wrench_sensor'] == 0.18
    assert mass['sensor_gripper_flange'] == pytest.approx(0.168 / 9.81)
    assert mass['gripper_base'] == pytest.approx(0.45 - 0.356 / 9.81)
    assert mass['link7'] == mass['link8'] == 0.025

    previous = URDF.from_xml_string(
        _xacro_xml(SHARE / 'urdf/piper_description.urdf'))
    for name in ('joint7', 'joint8'):
        joint = robot.joint_map[name]
        old_joint = previous.joint_map[name]
        assert joint.origin.xyz[2] == pytest.approx(old_joint.origin.xyz[2] - 0.0105)
        assert joint.origin.rpy == old_joint.origin.rpy
        assert joint.axis == old_joint.axis
        assert joint.limit.lower == old_joint.limit.lower
        assert joint.limit.upper == old_joint.limit.upper

    expected_meshes = {
        'px6d75_arm_flange.stl', 'px6d75.stl', 'px6d75_gripper_flange.stl',
        'gripper_base_without_flange.STL', 'link7.STL', 'link8.STL',
    }
    meshes = ET.fromstring(text).findall('.//mesh')
    assert {mesh.attrib['filename'] for mesh in meshes} == {
        f'package://piper_description/meshes/{name}' for name in expected_meshes}


def test_piper_with_wrench_attaches_complete_terminal_to_link6():
    robot = URDF.from_xml_string(
        _xacro_xml(SHARE / 'urdf/piper_description_with_wrench.urdf'))
    arm = URDF.from_xml_string(
        _xacro_xml(SHARE / 'urdf/piper_description.urdf'))
    terminal = URDF.from_xml_string(
        _xacro_xml(SHARE / 'urdf/paxini/gripper_with_wrench.xacro'))
    assert robot.get_root() == 'base_link'
    assert set(robot.link_map) == (
        set(arm.link_map) - {'gripper_base', 'link7', 'link8'}) | set(terminal.link_map)
    assert set(robot.joint_map) == (
        (set(arm.joint_map) - {'joint6_to_gripper_base', 'joint7', 'joint8'})
        | set(terminal.joint_map) | {'joint6_to_arm_sensor_flange'})
    for index in range(1, 7):
        name = f'joint{index}'
        assert robot.joint_map[name].origin.xyz == arm.joint_map[name].origin.xyz
        if index < 6:
            assert robot.joint_map[name].origin.rpy == arm.joint_map[name].origin.rpy
        assert robot.joint_map[name].limit.velocity == arm.joint_map[name].limit.velocity
    joint6 = robot.joint_map['joint6']
    reference_roll = arm.joint_map['joint6'].origin.rpy[0]
    zero_offset = math.pi / 4
    assert joint6.origin.rpy == pytest.approx([
        math.atan2(math.sin(reference_roll) * math.cos(zero_offset), math.cos(reference_roll)),
        -math.asin(math.sin(reference_roll) * math.sin(zero_offset)),
        math.atan2(math.cos(reference_roll) * math.sin(zero_offset), math.cos(zero_offset)),
    ], abs=1e-12)
    assert joint6.axis == arm.joint_map['joint6'].axis
    assert robot.joint_map['joint6'].limit.lower == pytest.approx(math.radians(-175))
    assert robot.joint_map['joint6'].limit.upper == pytest.approx(math.radians(175))
    mount = robot.joint_map['joint6_to_arm_sensor_flange']
    assert mount.parent == 'link6'
    assert mount.child == 'arm_sensor_flange'
    assert mount.origin.xyz == [0, 0, 0]
    assert mount.origin.rpy == pytest.approx([0, 0, -math.pi / 2])
    assert robot.joint_map['joint7'].origin.xyz == terminal.joint_map['joint7'].origin.xyz
    assert robot.joint_map['joint8'].origin.xyz == terminal.joint_map['joint8'].origin.xyz


@pytest.mark.parametrize('model', sorted((SHARE / 'mujoco_model').glob('*.xml')),
                         ids=lambda model: model.name)
def test_mujoco_mesh_resources(model):
    root = ET.parse(model).getroot()
    assert root.tag == 'mujoco'
    for mesh in root.findall('.//asset/mesh'):
        assert (model.parent / mesh.attrib['file']).is_file()


def driver_sample(width=0.04) -> Any:
    source = JointState()
    source.header.stamp.sec = 123
    source.header.frame_id = 'base_link'
    source.name = [f'joint{index}' for index in range(1, 7)] + ['gripper']
    source.position = [0.1, 0.4, -0.5, 0.1, 0.2, 0.3, width]
    source.velocity = [1.0] * 6
    source.effort = [2.0] * 6
    return source


@pytest.mark.parametrize('width', [0.0, 0.04, 0.07])
def test_total_gripper_width_becomes_two_finger_positions(width):
    source = driver_sample(width)
    result = model_joint_states(source)
    assert result.name == [f'joint{index}' for index in range(1, 9)]
    assert result.position[:6] == source.position[:6]
    assert list(result.position[6:]) == [width / 2.0, -width / 2.0]
    assert result.header == source.header
    assert not result.velocity and not result.effort
    assert source.name[-1] == 'gripper'
    assert source.position[-1] == width


def test_feedback_is_mapped_by_name_not_array_order():
    source = driver_sample()
    expected = model_joint_states(source)
    source.name.reverse()
    source.position.reverse()
    assert model_joint_states(source) == expected


@pytest.mark.parametrize('config', sorted((SHARE / 'rviz').glob('*.rviz')),
                         ids=lambda config: config.name)
def test_rviz_can_receive_description_after_publisher_startup(config):
    displays = yaml.safe_load(config.read_text())['Visualization Manager']['Displays']
    model = next(display for display in displays
                 if display['Class'] == 'rviz_default_plugins/RobotModel')
    assert model['Description Topic']['Durability Policy'] == 'Transient Local'


@pytest.mark.parametrize('invalid', ['missing_joint', 'missing_gripper', 'length', 'nan'])
def test_invalid_feedback_is_not_filled_with_zero(invalid):
    source = driver_sample()
    if invalid == 'missing_joint':
        source.name[0] = 'unknown'
    elif invalid == 'missing_gripper':
        source.name[-1] = 'unknown'
    elif invalid == 'length':
        source.position.pop()
    else:
        source.position[0] = math.nan
    with pytest.raises(ValueError):
        model_joint_states(source)


def test_real_ros_follow_launch_converts_feedback_to_finger_tf():
    namespace = f'/description_test_{os.getpid()}'
    rclpy.init()
    observer = rclpy.create_node('description_observer', namespace=namespace)
    executor = SingleThreadedExecutor()
    executor.add_node(observer)
    publisher = observer.create_publisher(JointState, 'joint_states_feedback', 10)
    transforms = {}

    def receive(message):
        transforms.update((item.child_frame_id, item) for item in message.transforms)

    observer.create_subscription(TFMessage, 'tf', receive, 10)
    process = subprocess.Popen([
        'ros2', 'launch', 'piper_description', 'follow.launch.py',
        f'namespace:={namespace}', 'rviz:=false',
        f'model:={SHARE / "urdf/piper_description.urdf"}',
    ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and not {'link7', 'link8'} <= transforms.keys():
            assert process.poll() is None
            source = driver_sample()
            source.header.stamp = observer.get_clock().now().to_msg()
            publisher.publish(source)
            executor.spin_once(timeout_sec=0.05)
        assert {'link1', 'link6', 'link7', 'link8'} <= transforms.keys()
        left = transforms['link7'].transform.translation
        right = transforms['link8'].transform.translation
        assert left.y == pytest.approx(-0.02, abs=1e-6)
        assert right.y == pytest.approx(0.02, abs=1e-6)
        assert right.y - left.y == pytest.approx(0.04, abs=1e-6)
        assert observer.get_publishers_info_by_topic(f'{namespace}/command/mit') == []
        assert observer.get_publishers_info_by_topic(f'{namespace}/command/joints') == []
    finally:
        process.send_signal(signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=5)
            pytest.fail(f'Follow launch did not stop: {output}')
        finally:
            executor.shutdown()
            observer.destroy_node()
            rclpy.try_shutdown()
        print(output)
    assert process.returncode == 0
    assert 'process has died' not in output


@pytest.mark.parametrize('index,terminal', [
    (1, 'teach_base'), (2, 'teach_base'), (3, 'gripper_base'), (4, 'gripper_base'),
])
def test_follow_chooses_configured_end_effector_model(index, terminal):
    from piper_x_driver.configuration import default_config_path

    spec = importlib.util.spec_from_file_location(
        'follow_entry', SHARE / 'launch/follow.launch.py')
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({
        'namespace': f'arm_{index}', 'config_file': str(default_config_path()),
        'model': '', 'rvizconfig': str(SHARE / 'rviz/piper_ctrl.rviz'), 'rviz': 'false',
    })
    with patch.object(module, 'Node') as node:
        module._follow(context)
    description = node.call_args_list[1].kwargs['parameters'][0]['robot_description']
    root = ET.fromstring(description.evaluate(context))
    assert root.find(f'link[@name="{terminal}"]') is not None
