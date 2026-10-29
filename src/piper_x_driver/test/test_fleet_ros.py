import json
import math
import shutil
import time
from typing import Any

from piper_x_driver.configuration import default_config_path, FleetConfig
from piper_x_driver.gravity import gravity_torques
from piper_x_driver.node import DriverNode
from piper_x_interfaces.srv import ConfigureArm, DebugMit, Enable
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


def spin_until(executor, predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.005)
    assert predicate()


@pytest.mark.usefixtures('fake_arm')
@pytest.mark.parametrize('angle_deg', [-2.0, 195.0])
def test_real_ros_mit_debug_configuration_persistence_and_deadman(tmp_path, angle_deg):
    path = tmp_path / 'arms.yaml'
    shutil.copyfile(default_config_path(), path)
    rclpy.init()
    executor = SingleThreadedExecutor()
    driver = DriverNode(namespace='/arm_1', parameter_overrides=[
        Parameter('config_file', value=str(path))])
    driver.arm.position = (0.0, math.radians(angle_deg), -0.5, 0.0, 0.0, 0.0, 0.02)
    observer = Node('fleet_control_test')
    executor.add_node(driver)
    executor.add_node(observer)
    state = []

    def append_state(msg: String):
        state.append(json.loads(msg.data))

    observer.create_subscription(String, '/arm_1/control_state', append_state, 10)
    power = observer.create_client(Enable, '/arm_1/enable_srv')
    config = observer.create_client(ConfigureArm, '/arm_1/configure_arm')
    debug = observer.create_client(DebugMit, '/arm_1/debug_mit')

    def call(client, request) -> Any:
        future = client.call_async(request)
        spin_until(executor, future.done)
        return future.result()

    try:
        spin_until(executor, lambda: all(c.service_is_ready() for c in (power, config, debug)))
        spin_until(executor, lambda: state)
        assert [call_item[0] for call_item in driver.arm.calls[:3]] == ['power', 'mit', 'mode']
        assert state[-1]['configuration']['can_port'] == 'can0'
        assert state[-1]['configuration']['end_effector'] == 'teaching_handle'
        response = call(config, ConfigureArm.Request(end_effector='gripper', gravity_scale=0.5))
        assert not response.success
        assert FleetConfig(path).arm('arm_1').end_effector == 'teaching_handle'
        started = call(debug, DebugMit.Request(action='start'))
        assert started.success, started.message
        assert list(started.position) == list(driver.arm.position[:6])
        target = list(started.position)
        target[0] += 0.1
        response = call(debug, DebugMit.Request(
            action='target', session=started.session, position=target))
        assert response.success
        spin_until(executor, lambda: driver.control.mit_ready)
        profile = driver.fleet.effectors['teaching_handle']
        expected = gravity_torques(driver.arm.position[:6], profile.mass_kg, profile.com_m)
        assert driver.control.gravity == pytest.approx(expected)
        response = call(config, ConfigureArm.Request(
            end_effector='teaching_handle', gravity_scale=0.4))
        assert response.success, response.message
        assert driver.control.gravity_scale == 0.4
        saved = FleetConfig(path)
        assert saved.arm('arm_1').gravity_scale == 0.4
        assert saved.arm('arm_2').gravity_scale == 1
        spin_until(executor, lambda: 'heartbeat' in driver.last_error)
        assert all(driver.arm.enabled)
        assert call(debug, DebugMit.Request(action='stop', session=started.session)).success
        assert call(power, Enable.Request(enable_request=False)).enable_response
        assert not any(driver.arm.enabled)
        late = call(debug, DebugMit.Request(
            action='target', session=started.session, position=target))
        assert not late.success
        response = call(config, ConfigureArm.Request(end_effector='gripper', gravity_scale=0.6))
        assert response.success
        assert FleetConfig(path).arm('arm_1').end_effector == 'gripper'
    finally:
        executor.shutdown()
        driver.destroy_node()
        observer.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.usefixtures('fake_arm')
def test_invalid_or_unwritable_runtime_configuration_never_changes_active_model(
    tmp_path, monkeypatch,
):
    path = tmp_path / 'arms.yaml'
    shutil.copyfile(default_config_path(), path)
    rclpy.init()
    driver = DriverNode(namespace='/arm_3', parameter_overrides=[
        Parameter('config_file', value=str(path))])
    try:
        original = driver.control.effector
        before = path.read_bytes()
        response = driver._configure(
            ConfigureArm.Request(end_effector='gripper', gravity_scale=float('nan')),
            ConfigureArm.Response())
        assert not response.success
        assert driver.control.gravity_scale == 1
        assert path.read_bytes() == before

        def fail(*_):
            raise OSError('Configuration directory not writable')

        monkeypatch.setattr(driver.fleet, 'save_arm', fail)
        response = driver._configure(
            ConfigureArm.Request(end_effector='gripper', gravity_scale=0.5),
            ConfigureArm.Response())
        assert not response.success
        assert 'not writable' in response.message
        assert driver.control.effector == original
        assert driver.control.gravity_scale == 1
        assert path.read_bytes() == before
    finally:
        driver.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.usefixtures('fake_arm')
def test_all_four_drivers_use_configured_can_and_effector_without_startup_actions():
    rclpy.init()
    drivers = []
    try:
        drivers = [DriverNode(namespace=f'/arm_{index}') for index in range(1, 5)]
        assert [driver.arm.channel for driver in drivers] == ['can0', 'can1', 'can2', 'can3']
        assert [driver.arm_config.end_effector for driver in drivers] == [
            'teaching_handle', 'teaching_handle', 'gripper', 'gripper']
        assert all(driver.control.gravity_scale == 1 for driver in drivers)
        assert all(driver.control.mode == 'mit' and all(driver.arm.enabled) for driver in drivers)
    finally:
        for driver in drivers:
            driver.destroy_node()
        rclpy.try_shutdown()
