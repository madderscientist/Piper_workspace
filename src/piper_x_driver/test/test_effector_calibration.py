from concurrent.futures import Future
from dataclasses import replace
import json
import shutil
import threading
import time
from types import SimpleNamespace
from typing import Any

from can import CanError
from piper_x_driver.configuration import default_config_path
from piper_x_driver.node import DriverNode
from piper_x_interfaces.msg import MitCommand
from piper_x_interfaces.srv import CalibrateEffector, ConfigureArm, DebugMit, Enable, SetMode
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


def spin_until(executor, predicate, timeout=4):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.005)
    assert predicate()


@pytest.fixture
def calibration_setup(fake_arm, tmp_path):
    path = tmp_path / 'arms.yaml'
    shutil.copyfile(default_config_path(), path)
    rclpy.init()
    executor = SingleThreadedExecutor()
    driver = DriverNode(namespace='/arm_2', parameter_overrides=[
        Parameter('config_file', value=str(path))])
    assert isinstance(driver.arm, fake_arm)
    driver.control._clear_power_state()
    driver.arm._powered = False
    driver.arm._mode = 'standby'
    driver.arm.enabled = (False,) * 7
    driver.arm.calls.clear()
    driver.arm.position = (0.0, 0.5, -0.5, 0.0, 0.0, 0.0, -0.0108)
    driver.arm.status.update(ctrl_mode=2, teach_status=2, mode_feedback=1)
    observer = Node('effector_calibration_test')
    executor.add_node(driver)
    executor.add_node(observer)
    clients = {
        'calibrate': observer.create_client(CalibrateEffector, '/arm_2/calibrate_effector'),
        'power': observer.create_client(Enable, '/arm_2/enable_srv'),
        'mode': observer.create_client(SetMode, '/arm_2/set_mode'),
        'config': observer.create_client(ConfigureArm, '/arm_2/configure_arm'),
        'debug': observer.create_client(DebugMit, '/arm_2/debug_mit'),
    }
    states = []

    def append_state(msg: String):
        states.append(json.loads(msg.data))

    observer.create_subscription(String, '/arm_2/control_state', append_state, 10)

    def call(name, request) -> Any:
        future = clients[name].call_async(request)
        spin_until(executor, future.done)
        return future.result()

    try:
        spin_until(executor, lambda: all(client.service_is_ready() for client in clients.values()))
        yield driver, executor, observer, clients, states, call
    finally:
        executor.shutdown()
        driver.destroy_node()
        observer.destroy_node()
        rclpy.try_shutdown()


def test_closed_negative_opening_can_be_calibrated_without_enable_or_motion(calibration_setup):
    driver, executor, _, _, states, call = calibration_setup
    before = driver.fleet.path.read_bytes()
    response = call('calibrate', CalibrateEffector.Request(confirm_closed=True))
    assert response.success, response.message
    assert '0.000 mm' in response.message
    assert driver.arm.calls == [('calibrate_effector',)]
    assert driver.arm.position[6] == 0
    assert not any(driver.arm.enabled)
    assert driver.control.mode == 'standby'
    assert driver.fleet.path.read_bytes() == before
    spin_until(executor, lambda: states and not states[-1]['calibration_active'])
    assert driver.last_error == ''
    assert call('power', Enable.Request(enable_request=True)).enable_response
    assert driver.arm.calls[-1] == ('power', True, 0.0)


@pytest.mark.parametrize('condition', [
    'unconfirmed', 'joint_enabled', 'effector_enabled', 'stale', 'fault',
    'error_code', 'teaching', 'recording', 'remote', 'native_mit',
    'local_mit', 'local_joint', 'debug', 'preloading', 'enabling', 'calibrating',
])
def test_unsafe_calibration_is_rejected_without_can_commands(calibration_setup, condition):
    driver, _, _, _, _, call = calibration_setup
    arm = driver.arm
    if condition == 'joint_enabled':
        arm.enabled = (True,) + (False,) * 6
    elif condition == 'effector_enabled':
        arm.enabled = (False,) * 6 + (True,)
    elif condition == 'stale':
        sample = arm.snapshot()
        arm.snapshot = lambda: replace(sample, age=(1.001,) * 11)
    elif condition == 'fault':
        arm.status['arm_status'] = 1
    elif condition == 'error_code':
        arm.status['err_code'] = 1
    elif condition == 'teaching':
        driver.teaching_active = True
        driver.teaching_received = time.monotonic()
    elif condition == 'recording':
        arm.status['teach_status'] = 1
    elif condition == 'remote':
        arm.status['ctrl_mode'] = 6
    elif condition == 'native_mit':
        arm.status.update(ctrl_mode=1, mode_feedback=6)
    elif condition in ('local_mit', 'local_joint'):
        driver.control.mode = condition.removeprefix('local_')
        if condition == 'local_mit':
            assert driver._calibration_block_reason()
            assert arm.calls == []
            return
    elif condition == 'debug':
        driver.control.debug_session = 'other-page'
    elif condition == 'preloading':
        driver.control.mit_received = time.monotonic()
    elif condition == 'enabling':
        driver.power_request = True
    elif condition == 'calibrating':
        driver.calibration_active = True
    response = call(
        'calibrate', CalibrateEffector.Request(confirm_closed=condition != 'unconfirmed'))
    assert not response.success and response.message
    assert arm.calls == []


@pytest.mark.parametrize('failure', ['missing_ack', 'can_error', 'nonzero', 'old_zero'])
def test_calibration_failure_never_enables_or_retries(calibration_setup, monkeypatch, failure):
    driver, _, _, _, _, call = calibration_setup
    attempts = []
    snapshot = driver.arm.snapshot
    old_stamp = time.monotonic()

    def calibrate():
        attempts.append(1)
        if failure == 'missing_ack':
            raise RuntimeError('No fresh device acknowledgement')
        if failure == 'can_error':
            raise CanError('CAN transmission failed')
        if failure == 'old_zero':
            driver.arm.position = driver.arm.position[:6] + (0.0,)
            monkeypatch.setattr(driver.arm, 'snapshot', lambda: replace(
                snapshot(), age=(time.monotonic() - old_stamp,) * 11))

    monkeypatch.setattr(driver.arm, 'calibrate_effector', calibrate)
    response = call('calibrate', CalibrateEffector.Request(confirm_closed=True))
    assert not response.success, response.message
    assert 'unconfirmed' in response.message
    assert not driver.calibration_active
    assert not any(driver.arm.enabled)
    assert driver.arm.calls == []
    assert attempts == [1]


def test_waiting_for_sdk_ack_keeps_feedback_and_rejection_callbacks_responsive(
    calibration_setup, monkeypatch,
):
    driver, executor, observer, clients, states, call = calibration_setup
    entered, release = threading.Event(), threading.Event()
    original = driver.arm.calibrate_effector
    publisher = observer.create_publisher(MitCommand, '/arm_2/command/mit', 1)

    def calibrate():
        entered.set()
        assert release.wait(3)
        original()

    monkeypatch.setattr(driver.arm, 'calibrate_effector', calibrate)
    future = clients['calibrate'].call_async(CalibrateEffector.Request(confirm_closed=True))
    try:
        spin_until(executor, lambda: entered.is_set() and driver.calibration_active)
        count = len(states)
        spin_until(executor, lambda: len(states) >= count + 2)
        assert states[-1]['calibration_active']
        assert not call('power', Enable.Request(enable_request=True)).enable_response
        assert not call('mode', SetMode.Request(mode='standby')).success
        assert not call('config', ConfigureArm.Request(
            end_effector='gripper', gravity_scale=0.5)).success
        assert not call('debug', DebugMit.Request(action='start')).success
        assert not call('calibrate', CalibrateEffector.Request(confirm_closed=True)).success
        spin_until(executor, lambda: publisher.get_subscription_count() > 0)
        publisher.publish(MitCommand(position=list(driver.arm.position[:6])))
        spin_until(executor, lambda: 'commands rejected' in driver.last_error)
        assert driver.arm.calls == []
    finally:
        release.set()
    spin_until(executor, future.done)
    assert future.result().success, future.result().message
    assert driver.arm.calls == [('calibrate_effector',)]


def test_disable_remains_available_and_supersedes_pending_calibration(
    calibration_setup, monkeypatch,
):
    driver, executor, _, clients, _, call = calibration_setup
    entered, release = threading.Event(), threading.Event()
    original = driver.arm.calibrate_effector

    def calibrate():
        entered.set()
        assert release.wait(3)
        original()

    monkeypatch.setattr(driver.arm, 'calibrate_effector', calibrate)
    future = clients['calibrate'].call_async(CalibrateEffector.Request(confirm_closed=True))
    try:
        spin_until(executor, lambda: entered.is_set())
        response = call('power', Enable.Request(enable_request=False))
        assert response.enable_response
        assert driver.calibration_active
        assert driver.arm.calls == [('power', False, 0.0)]
    finally:
        release.set()
    spin_until(executor, future.done)
    assert not future.result().success
    assert 'superseded' in future.result().message
    assert not driver.calibration_active
    assert not any(driver.arm.enabled)


@pytest.mark.parametrize('width,success', [
    (0.0, True), (0.0001, True), (-0.000001, False), (0.000101, False),
])
def test_zero_readback_acceptance_bounds_are_exact(fake_arm, monkeypatch, width, success):
    arm = fake_arm()
    arm.position = arm.position[:6] + (width,)
    node: Any = SimpleNamespace(
        power_serial=0, calibration_confirm_after=0.0,
        _calibration_feedback=arm.snapshot)
    work, result = Future(), Future()
    work.set_result(None)
    monkeypatch.setattr('piper_x_driver.node.time.monotonic', lambda: 0.6)
    DriverNode._check_calibration(node, work, result, 0)
    assert result.done()
    if success:
        assert result.result() == width
    else:
        assert isinstance(result.exception(), ValueError)


def test_pre_ack_zero_feedback_does_not_confirm_calibration(fake_arm, monkeypatch):
    arm = fake_arm()
    arm.position = arm.position[:6] + (0.0,)
    sample = replace(arm.snapshot(), age=(0.05,) * 11)
    node: Any = SimpleNamespace(
        power_serial=0, calibration_confirm_after=0.98,
        _calibration_feedback=lambda: sample)
    work, result = Future(), Future()
    work.set_result(None)
    monkeypatch.setattr('piper_x_driver.node.time.monotonic', lambda: 1.0)
    DriverNode._check_calibration(node, work, result, 0)
    assert not result.done()
    node.calibration_confirm_after = 0.9
    DriverNode._check_calibration(node, work, result, 0)
    assert result.result() == 0.0
