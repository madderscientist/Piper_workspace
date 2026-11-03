import json
import time
from typing import Any
from unittest.mock import patch

from geometry_msgs.msg import WrenchStamped
from piper_x_driver.node import DriverNode
from piper_x_interfaces.msg import ArmStatus, JointCommand, MitCommand
from piper_x_interfaces.srv import Enable, SetMode
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


def spin_until(executor, predicate, timeout=4):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert predicate()


@pytest.mark.usefixtures('fake_arm')
def test_driver_real_ros_services_commands_and_telemetry():
    rclpy.init()
    driver = DriverNode()
    client = Node('driver_test')
    executor = SingleThreadedExecutor()
    executor.add_node(driver)
    executor.add_node(client)
    states = []
    wrenches = []
    ctrl_states = []
    subscription = client.create_subscription(ArmStatus, 'arm_status', states.append, 10)
    wrench_sub = client.create_subscription(WrenchStamped, 'end_wrench_stamped', wrenches.append, 10)
    ctrl_sub = client.create_subscription(
        String, 'control_state', lambda msg: ctrl_states.append(json.loads(msg.data)), 10,
    )
    power = client.create_client(Enable, 'enable_srv')
    mode = client.create_client(SetMode, 'set_mode')
    publisher = client.create_publisher(JointCommand, 'command/joints', 1)
    try:
        spin_until(executor, lambda: power.service_is_ready() and mode.service_is_ready())
        future = power.call_async(Enable.Request(enable_request=True))
        spin_until(executor, future.done)
        power_res = future.result()
        assert power_res is not None and power_res.enable_response
        future = mode.call_async(SetMode.Request(mode='joint'))
        spin_until(executor, future.done)
        mode_res = future.result()
        assert mode_res is not None and mode_res.success
        spin_until(executor, lambda: publisher.get_subscription_count() > 0)
        publisher.publish(JointCommand(
            position=[0.1, 0.5, -0.5, 0.0, 0.0, 0.0], speed_percent=10))
        spin_until(executor, lambda: driver.arm.position[0] == 0.1)
        spin_until(executor, lambda: states and all(states[-1].motor_enabled))
        spin_until(executor, lambda: bool(wrenches) and bool(ctrl_states))
        assert all(states[-1].motor_enable_valid)
        assert states[-1].gripper_enabled
        assert wrenches[-1].header.frame_id == 'gripper'
        assert len(ctrl_states[-1]['end_wrench']) == 6
        assert len(ctrl_states[-1]['end_force']) == 3
        assert len(ctrl_states[-1]['end_torque']) == 3
        assert len(ctrl_states[-1]['external_torque']) == 6
        assert len(ctrl_states[-1]['inertia_torque']) == 6
        assert driver.rate_hz == 200.0
        assert driver.telemetry_hz == 200.0
        rates = ctrl_states[-1]['rates']
        assert rates['target_hz'] == 200.0
        assert rates['telemetry_target_hz'] == 200.0
        for key in (
            'can_rx_hz', 'loop_hz', 'comp_hz', 'cmd_tx_hz', 'mit_tx_hz',
            'grip_tx_hz', 'ros_cmd_hz', 'joints_pub_hz', 'wrench_pub_hz',
            'pose_pub_hz', 'status_pub_hz', 'control_pub_hz',
        ):
            assert key in rates
    finally:
        client.destroy_subscription(subscription)
        client.destroy_subscription(wrench_sub)
        client.destroy_subscription(ctrl_sub)
        executor.shutdown()
        driver.destroy_node()
        client.destroy_node()
        rclpy.shutdown()


def test_driver_has_only_can_backend_and_never_acts_on_startup(fake_arm):
    from piper_x_driver.gravity import gravity_torques

    rclpy.init()
    driver = DriverNode()
    try:
        assert isinstance(driver.arm, fake_arm)
        assert driver.arm.channel == 'can0'
        assert not driver.has_parameter('mock')
        assert driver.control.mode == 'mit'
        assert all(driver.arm.enabled)
        assert [call[0] for call in driver.arm.calls] == ['power', 'mit', 'mode']
        mit_cmd = driver.arm.calls[1][1]
        assert mit_cmd.kp == driver.control.debug_kp
        assert mit_cmd.kd == driver.control.debug_kd
        profile = driver.fleet.effectors[driver.arm_config.end_effector]
        pose = (0.0, 0.5, -0.5, 0.0, 0.0, 0.0)
        expected_g = (
            gravity_torques(pose, profile.mass_kg, profile.com_m)
            if driver.control.gravity_model is None
            else driver.control.gravity_model.torques(pose, profile.mass_kg, profile.com_m)
        )
        assert mit_cmd.position == pytest.approx([
            q + g / kp for q, g, kp in zip(pose, expected_g, driver.control.debug_kp)
        ])
        assert 'simulated' not in ArmStatus.get_fields_and_field_types()
    finally:
        driver.destroy_node()
        assert driver.arm.calls[-1] == ('mode', 'drag')
        rclpy.try_shutdown()


@pytest.mark.parametrize('latest_serial', [1, 2])
def test_disable_confirmation_only_needs_seven_fresh_power_flags(latest_serial):
    from types import SimpleNamespace

    from piper_x_driver.backend import Feedback
    from rclpy.task import Future

    sample = Feedback(
        None, None, (False,) * 7, (-1.0,) * 3 + (0.0, -1.0) + (0.0,) * 6)
    node: Any = SimpleNamespace(
        power_serial=latest_serial, arm=SimpleNamespace(snapshot=lambda: sample),
        control=SimpleNamespace(mode='standby'))
    future = Future()
    DriverNode._check_power(node, False, time.monotonic() + 0.1, future, 1)
    assert not future.done()
    DriverNode._check_power(node, False, time.monotonic() - 0.1, future, 1)
    assert future.done() and future.result()


@pytest.mark.usefixtures('fake_arm')
@pytest.mark.parametrize('mode', ['standby', 'mit'])
def test_each_publish_reads_feedback_only_once(mode):
    rclpy.init()
    driver = DriverNode()
    try:
        if mode == 'mit':
            driver.control.power(True)
            driver.control.mit(MitCommand(position=list(driver.arm.position[:6])))
            driver.control.set_mode('mit')
        with patch.object(driver.arm, 'snapshot', wraps=driver.arm.snapshot) as snapshot:
            for _ in range(3):
                driver._publish()
            assert snapshot.call_count == 3
    finally:
        driver.destroy_node()
        rclpy.try_shutdown()
