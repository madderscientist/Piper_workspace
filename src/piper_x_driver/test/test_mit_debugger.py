from dataclasses import replace
import math
from unittest.mock import patch

from can import CanError
import pytest

from piper_x_driver.configuration import default_config_path, FleetConfig
from piper_x_driver.controller import Controller, MitTarget
from piper_x_driver.gravity import gravity_torques


def command(position):
    return MitTarget(tuple(position), (0.0,) * 6, (1.0,) * 6, (0.5,) * 6, (0.0,) * 6)


def setup_control(fake_arm, effector='gripper'):
    arm = fake_arm()
    clock = [0.0]
    profile = FleetConfig(default_config_path()).effectors[effector]
    control = Controller(arm, clock=lambda: clock[0], effector=profile)
    return arm, control, clock, profile


def test_gravity_is_only_calculated_after_external_mit_acknowledgement(fake_arm):
    arm, control, clock, _ = setup_control(fake_arm)
    with patch('piper_x_driver.controller.gravity_torques', wraps=gravity_torques) as gravity:
        control.watchdog()
        control.power(True)
        control.set_mode('joint')
        control.watchdog()
        control.set_mode('standby')
        control.mit(command(arm.position[:6]))
        control.set_mode('mit')
        assert gravity.call_count == 0
        arm.status['mode_feedback'] = 1
        clock[0] = 0.1
        control.watchdog()
        assert gravity.call_count == 0
        arm.status['mode_feedback'] = 6
        control.watchdog()
        assert gravity.call_count == 1
        control.power(False)
        control.watchdog()
        assert gravity.call_count == 1


@pytest.mark.parametrize('effector', ['teaching_handle', 'gripper'])
def test_default_gravity_added_exactly_once_and_live_scale_is_slewed(fake_arm, effector):
    from piper_x_driver.backend import MAX_GRAVITY_OFFSET

    arm, control, clock, profile = setup_control(fake_arm, effector)
    control.power(True)
    target = replace(command(arm.position[:6]), kp=(4.0,) * 6, torque=(0.1,) * 6)
    control.mit(target)
    control.set_mode('mit')
    expected = gravity_torques(arm.position[:6], profile.mass_kg, profile.com_m)
    control.watchdog()
    offsets = [
        max(-MAX_GRAVITY_OFFSET, min(MAX_GRAVITY_OFFSET, g / kp))
        for g, kp in zip(expected, target.kp)
    ]
    assert arm.calls[-1][1].position == pytest.approx([
        q + off for q, off in zip(target.position, offsets)
    ])
    assert arm.calls[-1][1].torque == pytest.approx([
        0.1 + (g - off * kp) for g, off, kp in zip(expected, offsets, target.kp)
    ])
    control.gravity_scale = 0.0
    for tick in range(1, 11):
        clock[0] = tick / 10
        control.mit(target)
        control.watchdog()
        assert control.gravity_scale_applied == pytest.approx(1 - tick / 10)
    assert arm.calls[-1][1].position == pytest.approx(target.position)
    assert arm.calls[-1][1].torque == pytest.approx((0.1,) * 6)
    control.gravity_scale = 1.0
    clock[0] = 2.0
    zero_kp = replace(target, kp=(0.0,) * 6)
    control.mit(zero_kp)
    control.watchdog()
    assert arm.calls[-1][1].position == pytest.approx(target.position)
    assert arm.calls[-1][1].torque == pytest.approx([0.1 + g for g in expected])
    zero_kd = replace(target, kd=(0.0,) * 6)
    control.mit(zero_kd)
    control.watchdog()
    assert arm.calls[-1][1].position == pytest.approx(target.position)
    assert arm.calls[-1][1].torque == pytest.approx([0.1 + g for g in expected])


def test_mit_debug_start_initializes_from_measurement_and_never_enables(fake_arm):
    arm, control, clock, profile = setup_control(fake_arm)
    with pytest.raises(ValueError, match='enabled'):
        control.debug_start()
    control.power(True)
    arm.calls.clear()
    session = control.debug_start()
    assert session
    assert control.debug_target == arm.position[:6]
    assert all(call[0] != 'power' for call in arm.calls)
    desired = (1.0,) + arm.position[1:6]
    control.debug_update(session, desired)
    clock[0] = 0.1
    expected = gravity_torques(arm.position[:6], profile.mass_kg, profile.com_m)
    control.watchdog()
    assert arm.calls[-1][1].position == pytest.approx([
        q + g / kp for q, g, kp in zip(desired, expected, control.debug_kp)
    ])
    assert arm.calls[-1][1].torque == (0.0,) * 6
    with pytest.raises(ValueError, match='owns'):
        control.mit(command(arm.position[:6]))
    with pytest.raises(ValueError, match='owns'):
        control.set_mode('joint')
    control.debug_stop(session)
    assert not control.debug_session
    assert control.mode == 'mit' and all(arm.enabled)
    clock[0] += 0.5
    control.watchdog()
    expected_after = gravity_torques(arm.position[:6], profile.mass_kg, profile.com_m)
    assert arm.calls[-1][1].position == pytest.approx([
        q + g / kp for q, g, kp in zip(arm.position[:6], expected_after, control.debug_kp)
    ])
    assert arm.calls[-1][1].velocity == (0.0,) * 6
    assert arm.calls[-1][1].torque == (0.0,) * 6
    assert control.mode == 'mit' and all(arm.enabled)


@pytest.mark.parametrize('teach_status', [2, 6])
def test_stopped_native_teaching_allows_explicit_mit_debug_start(fake_arm, teach_status):
    arm, control, _, _ = setup_control(fake_arm)
    control.power(True)
    arm.status.update(ctrl_mode=2, teach_status=teach_status)
    arm.calls.clear()
    session = control.debug_start()
    assert session
    assert control.debug_target == arm.position[:6]
    assert [call[0] for call in arm.calls] == ['mit', 'mode']
    assert arm.calls[-1] == ('mode', 'mit')
    assert not control.mit_ready


@pytest.mark.parametrize('angle_deg', [-2.0, 195.0])
def test_debug_session_holds_raw_measurement_and_sends_target_without_rate_limit(
    fake_arm, angle_deg,
):
    arm, control, clock, profile = setup_control(fake_arm)
    arm.position = (0.0, math.radians(angle_deg), -0.5, 0.0, 0.0, 0.0, 0.02)
    control.power(True)
    measured = arm.position[:6]
    session = control.debug_start()
    assert control.debug_target == measured
    assert arm.calls[-2][1].position == measured
    position = (0.5,) + measured[1:]
    control.debug_update(session, position)
    clock[0] = 0.1
    expected = gravity_torques(measured, profile.mass_kg, profile.com_m)
    control.watchdog()
    assert arm.calls[-1][1].position == pytest.approx([
        q + g / kp for q, g, kp in zip(position, expected, control.debug_kp)
    ])
    assert arm.calls[-1][1].torque == (0.0,) * 6
    assert control.debug_session == session


@pytest.mark.parametrize('now,expired', [(0.3, False), (0.300001, True)])
def test_slider_heartbeat_expiry_retains_output_and_accepts_fresh_input(fake_arm, now, expired):
    arm, control, clock, _ = setup_control(fake_arm)
    control.power(True)
    session = control.debug_start()
    for time in (0.1, 0.2):
        clock[0] = time
        control.watchdog()
    clock[0] = now
    if expired:
        initial = arm.mit_output
        with pytest.raises(ValueError, match='retaining last MIT output'):
            control.watchdog()
        assert control.mode == 'mit' and all(arm.enabled)
        assert arm.mit_output == initial
        control.debug_update(session, arm.position[:6])
        control.watchdog()
        assert not control.hold_reason and all(arm.enabled)
    else:
        control.watchdog()
        assert control.debug_session == session


def test_late_heartbeat_updates_existing_session_without_shutdown(fake_arm):
    arm, control, clock, _ = setup_control(fake_arm)
    control.power(True)
    session = control.debug_start()
    clock[0] = 0.301
    control.debug_update(session, arm.position[:6])
    control.watchdog()
    assert control.mode == 'mit' and all(arm.enabled)
    assert control.debug_session == session


@pytest.mark.parametrize('fault', ['mode', 'can'])
def test_debug_faults_retain_output_without_disabling(fake_arm, fault):
    arm, control, clock, _ = setup_control(fake_arm)
    control.power(True)
    control.debug_start()
    control.watchdog()
    initial = arm.mit_output
    clock[0] = 0.1
    if fault == 'mode':
        arm.status['ctrl_mode'] = 5
    else:
        def fail(_):
            raise CanError('Test bus failure')
        arm._write_mit = fail
    with pytest.raises(ValueError, match='no disable requested'):
        control.watchdog()
    clock[0] += 5.0
    with pytest.raises(ValueError, match='no disable requested'):
        control.watchdog()
    assert control.mode == 'mit' and all(arm.enabled)
    assert arm.mit_output == initial
    assert not any(call[0] == 'power' and not call[1] for call in arm.calls)


def test_foreign_session_and_invalid_values_do_not_refresh_heartbeat(fake_arm):
    arm, control, clock, _ = setup_control(fake_arm)
    control.power(True)
    session = control.debug_start()
    clock[0] = 0.1
    with pytest.raises(ValueError, match='belongs'):
        control.debug_update('foreign', arm.position[:6])
    with pytest.raises(ValueError, match='six'):
        control.debug_update(session, [0.0] * 5)
    assert control.debug_received == 0
