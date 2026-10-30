from dataclasses import replace
import time

from piper_teaching.bilateral import ArmSample, TeachingFault
from piper_teaching.node import TeachingNode
from piper_x_driver.gravity import gravity_torques
from piper_x_driver.node import DriverNode
from piper_x_interfaces.srv import Enable, SetMode
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger


def spin_until(executor, predicate, timeout=6):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.005)
    assert predicate()


@pytest.mark.usefixtures('fake_arm')
@pytest.mark.parametrize('auto_start', [False, True])
def test_four_drivers_engage_auto_start_and_stop(auto_start):
    rclpy.init()
    executor = SingleThreadedExecutor()
    drivers = [DriverNode(namespace=f'/arm_{index}') for index in range(1, 5)]
    drivers[2].arm.position = (1.0, 0.5, -0.5, 0.0, 0.0, 0.0, 0.06)
    parameters = [
        Parameter('dry_run', value=False),
        Parameter('kp', value=[3.0] * 6),
        Parameter('kd', value=[0.5] * 6),
        Parameter('gripper_effort', value=0.3),
    ]
    if not auto_start:
        parameters.append(Parameter('auto_start', value=False))
    teaching = TeachingNode(parameter_overrides=parameters)
    for node in drivers + [teaching]:
        executor.add_node(node)
    try:
        spin_until(executor, lambda: all(item is not None for item in teaching.statuses))
        clients = teaching.power_clients + teaching.mode_clients
        spin_until(executor, lambda: all(client.service_is_ready() for client in clients))
        if not auto_start:
            response = teaching._start(Trigger.Request(), Trigger.Response())
            assert response.success, response.message
        spin_until(executor, lambda: teaching.phase == 'engaging')
        assert [driver.arm.position[6] for driver in drivers] == [0.02, 0.02, 0.06, 0.02]
        if not auto_start:
            response = teaching._stop_service(Trigger.Request(), Trigger.Response())
            assert response.success
            spin_until(executor, lambda: teaching.phase == 'stopped')
            assert all(driver.control.mode == 'mit' and all(driver.arm.enabled)
                       for driver in drivers)
            return
        spin_until(executor, lambda: (
            teaching.phase == 'active'
            and all(driver.arm.position[:6] == pytest.approx(goal)
                    for driver, goal in zip(drivers, teaching.coupling.engage_goals))
        ))
        assert not teaching.auto_start
        assert all(driver.control.mode == 'mit' for driver in drivers)
        assert all(all(driver.arm.enabled) for driver in drivers)
        assert all(any(call[0] == 'mit' for call in driver.arm.calls) for driver in drivers)
        future = teaching.power_clients[0].call_async(Enable.Request(enable_request=False))
        spin_until(executor, future.done)
        assert future.result().enable_response
        assert drivers[0].control.mode == 'standby' and not any(drivers[0].arm.enabled)
        spin_until(executor, lambda: teaching.phase == 'fault')
        assert all(driver.control.mode == 'mit' and all(driver.arm.enabled)
                   for driver in drivers[1:])
    finally:
        executor.shutdown(timeout_sec=1)
        teaching.destroy_node()
        for driver in drivers:
            driver.destroy_node()
        rclpy.shutdown()


@pytest.mark.usefixtures('fake_arm')
def test_master_slave_mode_controls_slaves_with_position_folded_gravity():
    rclpy.init()
    executor = SingleThreadedExecutor()
    drivers = [DriverNode(namespace=f'/arm_{index}') for index in range(1, 5)]
    kp = [5.0, 5.0, 5.0, 1.0, 0.7, 0.7]
    teaching = TeachingNode(parameter_overrides=[
        Parameter('mode', value='master_slave'),
        Parameter('dry_run', value=False),
        Parameter('kp', value=kp),
        Parameter('kd', value=[0.5, 0.8, 0.8, 0.2, 0.2, 0.2]),
        Parameter('gripper_effort', value=0.3),
    ])
    for node in drivers + [teaching]:
        executor.add_node(node)
    try:
        spin_until(executor, lambda: all(item is not None for item in teaching.statuses))
        mode_res = teaching._set_mode(SetMode.Request(mode='bilateral'), SetMode.Response())
        assert mode_res.success and teaching.config.mode == 'bilateral'
        mode_res = teaching._set_mode(SetMode.Request(mode='master_slave'), SetMode.Response())
        assert mode_res.success and teaching.config.mode == 'master_slave'
        spin_until(executor, lambda: teaching.phase == 'active')
        rejected = teaching._set_mode(SetMode.Request(mode='bilateral'), SetMode.Response())
        assert not rejected.success and teaching.config.mode == 'master_slave'
        master_1 = (0.2, 0.4, -0.4, -0.5, 0.1, -0.1, 0.05)
        master_2 = (-0.3, 0.6, -0.6, -0.4, -0.1, 0.2, 0.07)

        def drag_masters():
            drivers[0].arm.position = master_1
            drivers[1].arm.position = master_2
            return (
                drivers[2].arm.position == pytest.approx(master_1)
                and drivers[3].arm.position == pytest.approx(master_2)
            )

        spin_until(executor, drag_masters)
        # Moving slave arms must not pull master arms away from their own measured pose.
        drivers[2].arm.position = (0.8, 0.1, -0.1, 0.0, 0.0, 0.0, 0.01)
        for _ in range(15):
            executor.spin_once(timeout_sec=0.005)
        assert drivers[0].arm.position == pytest.approx(master_1)
        assert all(driver.control.mode == 'mit' for driver in drivers)
        for index, (driver, base_target) in enumerate(zip(
            drivers, (master_1[:6], master_2[:6], master_1[:6], master_2[:6]),
        )):
            from piper_x_driver.backend import MAX_GRAVITY_OFFSET

            profile = driver.fleet.effectors[driver.arm_config.end_effector]
            g_model = driver.control.gravity_model
            expected_g = (
                gravity_torques(base_target, profile.mass_kg, profile.com_m)
                if g_model is None
                else g_model.torques(base_target, profile.mass_kg, profile.com_m)
            )
            is_master = index in (0, 1)
            offsets = (0.0,) * 6 if is_master else tuple(
                max(-MAX_GRAVITY_OFFSET, min(MAX_GRAVITY_OFFSET, g / k))
                for g, k in zip(expected_g, kp)
            )
            expected_pos = [q + off for q, off in zip(base_target, offsets)]
            expected_torque = expected_g if is_master else tuple(
                g - off * k for g, off, k in zip(expected_g, offsets, kp)
            )
            spin_until(executor, lambda d=driver, g=expected_g, p=expected_pos, t=expected_torque: (
                d.arm.position[:6] == pytest.approx(base_target)
                and d.control.gravity == pytest.approx(g)
                and any(call[0] == 'mit' for call in d.arm.calls)
                and [call[1] for call in d.arm.calls if call[0] == 'mit'][-1].position
                == pytest.approx(p)
                and [call[1] for call in d.arm.calls if call[0] == 'mit'][-1].torque
                == pytest.approx(t)
            ))
            last_mit = [call[1] for call in driver.arm.calls if call[0] == 'mit'][-1]
            assert last_mit.position == pytest.approx(expected_pos)
            assert last_mit.torque == pytest.approx(expected_torque)
            assert last_mit.kp == pytest.approx((0.0,) * 6 if is_master else kp)
            expected_kd = (0.0,) * 6 if is_master else (0.5, 0.8, 0.8, 0.2, 0.2, 0.2)
            assert last_mit.kd == pytest.approx(expected_kd)
    finally:
        executor.shutdown(timeout_sec=1)
        teaching.destroy_node()
        for driver in drivers:
            driver.destroy_node()
        rclpy.shutdown()


@pytest.mark.usefixtures('fake_arm')
def test_preview_never_calls_power_mode_or_command_endpoints():
    rclpy.init()
    executor = SingleThreadedExecutor()
    drivers = [DriverNode(namespace=f'/arm_{index}') for index in range(1, 5)]
    for driver in drivers:
        driver.control._clear_power_state()
        driver.arm.enabled = (False,) * 7
        driver.arm.calls.clear()
    teaching = TeachingNode()
    for node in drivers + [teaching]:
        executor.add_node(node)
    try:
        spin_until(executor, lambda: all(item is not None for item in teaching.statuses))
        response = teaching._start(Trigger.Request(), Trigger.Response())
        assert not response.success
        teaching.stop('Preview stop')
        for _ in range(30):
            executor.spin_once(timeout_sec=0.005)
        assert all(driver.arm.calls == [] for driver in drivers)
    finally:
        executor.shutdown(timeout_sec=1)
        teaching.destroy_node()
        for driver in drivers:
            driver.destroy_node()
        rclpy.shutdown()


def test_enable_acknowledgement_waits_for_updated_enabled_telemetry(fake_arm, monkeypatch):
    rclpy.init()
    teaching = TeachingNode(parameter_overrides=[
        Parameter('dry_run', value=False),
        Parameter('kp', value=[3.0] * 6),
        Parameter('kd', value=[0.5] * 6),
        Parameter('gripper_effort', value=0.3),
    ])
    now = time.monotonic()
    sample = ArmSample(fake_arm().position, (now,) * 11, (False,) * 7)
    samples = [sample] * 4
    monkeypatch.setattr(teaching, 'samples', lambda _: tuple(samples))
    monkeypatch.setattr(teaching, '_results', lambda _: True)
    teaching.phase, teaching.deadline = 'enabling', now + 5.0
    try:
        teaching._tick()
        assert teaching.phase == 'enabling'
        assert not teaching.coupling.active
        samples[:] = [replace(sample, enabled=(True,) * 7)] * 4
        teaching._tick()
        assert teaching.phase == 'preloading'
        assert teaching.coupling.active
        targets = teaching.coupling.targets

        def expired(_):
            raise TeachingFault('expired ROS feedback')

        monkeypatch.setattr(teaching, 'samples', expired)
        teaching._tick()
        assert teaching.holding and teaching.phase == 'preloading'
        assert teaching.coupling.active and teaching.coupling.targets == targets
        monkeypatch.setattr(teaching, 'samples', lambda _: tuple(samples))
        teaching._tick()
        assert not teaching.holding and teaching.coupling.active
        teaching.power_clients = [
            type('Client', (), {
                'service_is_ready': lambda _: True,
                'call_async': lambda *_: None,
            })() for _ in range(4)
        ]
        teaching.mode_clients = teaching.power_clients
        samples[:] = [sample] * 4
        teaching.request_retry()
        teaching._tick()
        assert teaching.phase == 'enabling' and not teaching.retry_requested
    finally:
        teaching.destroy_node()
        rclpy.try_shutdown()
