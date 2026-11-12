from dataclasses import replace
import math
from types import SimpleNamespace
from typing import Any
from unittest.mock import Mock, patch

import pytest

from piper_x_driver.backend import SdkArm
from piper_x_driver.controller import Controller


def target(**changes):
    fields = dict(
        position=[0.0, 0.5, -0.5, 0.0, 0.0, 0.0], velocity=[0.0] * 6,
        kp=[1.0] * 6, kd=[0.5] * 6, torque=[0.0] * 6,
    )
    fields.update(changes)
    return SimpleNamespace(**fields)


@pytest.mark.parametrize('width', [-0.0108, 0.0, 0.02, 0.1, 0.1041])
def test_enable_preserves_measured_width_without_clamping_or_mode_change(fake_arm, width):
    arm = fake_arm()
    arm.position = arm.position[:6] + (width,)
    control = Controller(arm)
    control.power(True)
    assert arm.calls == [('power', True, width)]
    assert control.mode == 'standby'


@pytest.mark.parametrize('acknowledged', [True, False])
def test_sdk_calibration_only_calls_the_bounded_zeroing_interface(acknowledged):
    adapter = offline_adapter()
    adapter.gripper = Mock()
    adapter.gripper.calibrate_gripper.return_value = acknowledged
    if acknowledged:
        adapter.calibrate_effector()
    else:
        with pytest.raises(RuntimeError, match='not acknowledge'):
            adapter.calibrate_effector()
    assert adapter.gripper.mock_calls == [
        ('calibrate_gripper', (), {'timeout': 1.0})]


def test_sdk_mit_request_uses_can_control_even_after_native_hold():
    from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config

    with patch('socket.socket', side_effect=AssertionError('Hardware forbidden')):
        adapter = offline_adapter()
        adapter.robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.PIPER_X, firmeware_version=PiperFW.V189, auto_connect=False))
        try:
            with patch.object(adapter.robot, '_send_msg') as send:
                adapter.mode('hold')
                adapter.mode('mit')
                assert send.call_count == 2
                assert send.call_args_list[0].args[0].grag_teach_ctrl == 2
                assert send.call_args_list[1].args[0].ctrl_mode == 1
        finally:
            adapter.robot.disconnect()


def test_no_implicit_modes_and_no_partial_targets(fake_arm):
    arm = fake_arm()
    control = Controller(arm)
    control.power(True)
    with pytest.raises(ValueError, match='matching mode'):
        control.joints(SimpleNamespace(position=target().position, speed_percent=10))
    control.set_mode('joint')
    with pytest.raises(ValueError, match='six'):
        control.joints(SimpleNamespace(position=[0.1], speed_percent=10))
    control.joints(SimpleNamespace(position=target().position, speed_percent=10))
    assert arm.calls[-1] == ('joints', target().position, 10)
    control.mit(target())
    control.set_mode('mit')
    new_pos = [0.2, 0.4, -0.4, 0.1, -0.1, 0.3]
    control.joints(SimpleNamespace(position=new_pos, speed_percent=0))
    assert control.mit_target is not None
    assert control.mit_target.position == tuple(new_pos)
    assert control.mit_target.kp == tuple(target().kp)
    assert control.mit_target.kd == tuple(target().kd)


@pytest.mark.parametrize('angle_deg', [-2.0, 195.0])
def test_joint_and_mit_commands_preserve_raw_j2_outside_model_workspace(fake_arm, angle_deg):
    arm = fake_arm()
    control = Controller(arm)
    control.power(True)
    position = list(arm.position[:6])
    position[1] = math.radians(angle_deg)
    control.set_mode('joint')
    control.joints(SimpleNamespace(position=position, speed_percent=10))
    assert arm.calls[-1] == ('joints', position, 10)
    assert control.feedback().position[1] == position[1]
    control.set_mode('standby')
    control.mit(target(position=position))
    control.set_mode('mit')
    assert arm.calls[-2][1].position == position
    assert arm.position[1] == position[1]


@pytest.mark.parametrize('angle_deg', [-2.0, 195.0])
def test_real_sdk_encodes_raw_j2_without_model_clipping(angle_deg):
    from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config

    with patch('socket.socket', side_effect=AssertionError('Hardware forbidden')):
        adapter = offline_adapter()
        adapter.robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.PIPER_X, firmeware_version=PiperFW.V189, auto_connect=False))
        try:
            assert not adapter.robot.get_joint_limits_enabled()
            parser = adapter.robot._parser
            with (patch.object(adapter.robot, '_send_msg'),
                  patch.object(adapter.robot, '_send_msgs')):
                with patch.object(parser, '_make_joint_ctrl_msgs',
                                  wraps=parser._make_joint_ctrl_msgs) as joint_message:
                    position = target().position
                    position[1] = math.radians(angle_deg)
                    adapter.joints(position, 10)
                    assert joint_message.call_args.args[0][1] == round(angle_deg * 1000)
                with patch.object(parser, '_make_joint_mit_ctrl_msg',
                                  wraps=parser._make_joint_mit_ctrl_msg) as mit_message:
                    adapter.mit(target(position=position))
                    encoded = mit_message.call_args_list[1].kwargs['p_des']
                    assert encoded * 25 / 65535 - 12.5 == pytest.approx(
                        position[1], abs=25 / 65535)
        finally:
            adapter.robot.disconnect()


@pytest.mark.parametrize('field', ['position', 'velocity', 'kp', 'kd', 'torque'])
def test_entire_mit_packet_validated_before_transmit(field, fake_arm):
    arm = fake_arm()
    control = Controller(arm)
    with pytest.raises(ValueError):
        control.mit(target(**{field: [0.0] * 5}))
    assert arm.calls == []


@pytest.mark.parametrize('trigger', ['explicit', 'input_timeout', 'feedback'])
def test_timeouts_and_exit_enter_gravity_drag_without_soft_stop(fake_arm, trigger):
    from piper_x_driver.gravity import gravity_torques

    now = [0.0]
    arm = fake_arm()
    control = Controller(arm, clock=lambda: now[0])
    control.power(True)
    with pytest.raises(ValueError, match='fresh'):
        control.set_mode('mit')
    command = target()
    control.mit(command)
    assert arm.calls == [('power', True, 0.02)]
    control.set_mode('mit')
    assert arm.calls[-2:] == [('mit', command), ('mode', 'mit')]
    control.watchdog()
    initial = arm.calls[-1][1]
    original_snapshot = arm.snapshot
    now[0] = 0.201
    if trigger == 'input_timeout':
        expected_g = gravity_torques(arm.position[:6])
        control.watchdog()
        output = arm.calls[-1][1]
        assert output.position == pytest.approx([
            q + g / kp for q, g, kp in zip(arm.position[:6], expected_g, control.debug_kp)
        ])
        assert output.kp == control.debug_kp
        assert output.kd == control.debug_kd
        assert output.velocity == (0.0,) * 6
        assert output.torque == (0.0,) * 6
        assert control.mode == 'mit' and all(arm.enabled)
    elif trigger == 'feedback':
        sample = arm.snapshot()
        arm.snapshot = lambda: replace(sample, age=(-1.0,) * 11, position=None)
        with pytest.raises(ValueError, match='retaining last MIT output'):
            control.watchdog()
        for elapsed in (1.0, 5.0, 60.0):
            now[0] = 0.201 + elapsed
            with pytest.raises(ValueError, match='retaining last MIT output'):
                control.watchdog()
            assert arm.calls[-1] == ('mit', initial)
            assert control.mode == 'mit' and all(arm.enabled)
        arm.snapshot = original_snapshot
        control.mit(command)
        control.watchdog()
        assert not control.hold_reason
    control.power(False)
    assert arm.calls[-1] == ('power', False, 0.0)
    assert not any(arm.enabled)
    assert control.mode == 'standby'


def test_bad_feedback_blocks_enable_but_never_blocks_disable(fake_arm):
    arm = fake_arm()
    control = Controller(arm)
    sample = arm.snapshot()
    arm.snapshot = lambda: replace(sample, age=(-1.0,) * 11)
    with pytest.raises(ValueError, match='fresh') as error:
        control.power(True)
    assert 'joint12(0x2A5)=missing' in str(error.value)
    control.power(False)
    assert arm.calls == [('power', False, 0.0)]


def test_actual_native_mode_is_not_external_joint_mode(fake_arm):
    arm = fake_arm()
    control = Controller(arm)
    control.power(True)
    control.set_mode('joint')
    arm.mode('hold')
    with pytest.raises(ValueError, match='matching'):
        control.joints(SimpleNamespace(position=target().position, speed_percent=10))


def test_sdk_snapshot_never_fills_absent_can_segments_with_zero():
    from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config
    from can import Message
    import time

    with patch('socket.socket', side_effect=AssertionError('Hardware forbidden')):
        adapter = offline_adapter()
        adapter.robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.PIPER_X, firmeware_version=PiperFW.V189, auto_connect=False))
        adapter.gripper = SimpleNamespace(get_gripper_status=lambda: None)
        adapter.robot._parser.parse_packet(Message(
            arbitration_id=0x2A7, data=bytes(8), timestamp=time.time(), is_extended_id=False))
        sample = adapter.snapshot()
        assert sample.position is None
        assert sample.age[0] == -1.0 and sample.age[2] >= 0
        adapter.robot.disconnect()


def test_complete_sdk_snapshot_uses_real_message_fields_and_si_values():
    from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config
    from pyAgxArm.protocols.can_protocol.drivers.effector.agx_gripper.default.parser import Parser
    from pyAgxArm.utiles.fps import FPSManager
    from can import Message
    import time

    with patch('socket.socket', side_effect=AssertionError('Hardware forbidden')):
        adapter = offline_adapter()
        adapter.robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.PIPER_X, firmeware_version=PiperFW.V189, auto_connect=False))
        gripper = Parser(FPSManager())
        stamp = time.time()
        gripper.parse_packet(Message(
            arbitration_id=0x2A8, data=bytes.fromhex('00004E2000000000'),
            timestamp=stamp, is_extended_id=False))
        adapter.gripper = SimpleNamespace(get_gripper_status=lambda: gripper.gripper)
        for id_ in (0x2A1, 0x2A5, 0x2A6, 0x2A7, 0x2A2, 0x2A3, 0x2A4,
                    *range(0x251, 0x257), *range(0x261, 0x267)):
            data = bytes.fromhex('03E8000000000000') if 0x251 <= id_ <= 0x256 else bytes(8)
            adapter.robot._parser.parse_packet(Message(
                arbitration_id=id_, data=data, timestamp=stamp, is_extended_id=False))

        def clock_during_receive():
            adapter.robot._parser.parse_packet(Message(
                arbitration_id=0x261, data=bytes(8),
                timestamp=stamp + 0.001, is_extended_id=False))
            return stamp

        with patch('piper_x_driver.backend.time', SimpleNamespace(time=clock_during_receive)):
            sample = adapter.snapshot()
        assert sample.position == pytest.approx((0.0,) * 6 + (0.02,))
        assert sample.velocity == (1.0,) * 6
        assert sample.torque == (0.0,) * 6
        assert sample.pose == (0.0,) * 6
        assert all(0 <= age < 0.2 for age in sample.age)
        adapter.robot.disconnect()


def test_sdk_disable_attempts_every_joint_and_gripper_after_can_failure():
    from can import CanError

    calls = []

    def disable(index):
        calls.append(index)
        if index == 2:
            raise CanError('J2 bus error')

    adapter = offline_adapter()
    adapter.robot = SimpleNamespace(disable=disable)
    adapter.gripper = SimpleNamespace(disable_gripper=lambda: calls.append('gripper'))
    with pytest.raises(CanError, match='J2'):
        adapter.power(False, 0.0)
    assert calls == [1, 2, 3, 4, 5, 6, 'gripper']


def test_partial_sdk_enable_is_rolled_back_without_closing_gripper():
    from can import CanError

    calls = []

    def enable(index):
        calls.append(('enable', index))
        if index == 3:
            raise CanError('Enable failed')

    adapter = offline_adapter()
    adapter.robot = SimpleNamespace(
        enable=enable, disable=lambda index: calls.append(('disable', index)))
    adapter.gripper = SimpleNamespace(
        disable_gripper=lambda: calls.append(('disable', 'gripper')))
    with pytest.raises(CanError, match='Enable failed'):
        adapter.power(True, 0.02)
    assert calls[:3] == [('enable', 1), ('enable', 2), ('enable', 3)]
    assert calls[3:] == [('disable', index) for index in range(1, 7)] + [
        ('disable', 'gripper')]


def test_hold_request_can_end_a_nonfault_recording_state(fake_arm):
    arm = fake_arm()
    control = Controller(arm)
    control.power(True)
    arm.status['arm_status'] = 11
    arm.status['teach_status'] = 1
    control.set_mode('hold')
    assert arm.status['teach_status'] == 2


def test_disable_changes_enable_flags_but_preserves_native_firmware_mode(fake_arm):
    arm = fake_arm()
    control = Controller(arm)
    control.power(True)
    arm.mode('hold')
    status = arm.status.copy()
    arm.calls.clear()
    control.power(False)
    assert arm.calls == [('power', False, 0.0)]
    assert not any(arm.enabled)
    assert arm.status == status


@pytest.mark.parametrize('fw_version', ['S-V1.9-0', 'S-V1.9-1'])
def test_sdk_waits_for_receive_fps_before_querying_firmware(fw_version):
    from unittest.mock import Mock

    robot = Mock()
    robot.get_fps.side_effect = [0, 0, 50]

    def firmware(**_):
        assert robot.get_fps.call_count == 3
        return {'software_version': fw_version}

    robot.get_firmware.side_effect = firmware
    with patch('piper_x_driver.backend.AgxArmFactory.create_arm', return_value=robot):
        with patch('piper_x_driver.backend.time.sleep') as sleep:
            adapter = SdkArm('can0')
    assert adapter.robot is robot
    robot.set_follower_mode.assert_called_once_with()
    assert robot.get_fps.call_count == 3
    assert sleep.call_count == 2
    robot.get_firmware.assert_called_once_with(timeout=2.0)
    robot.set_joint_limits_enabled.assert_called_once_with(False)
    robot.enable.assert_not_called()
    robot.disable.assert_not_called()
    robot.set_motion_mode.assert_not_called()
    adapter.close()


def test_sdk_retries_misaligned_firmware_segments_after_clearing_cache():
    from unittest.mock import Mock

    robot = Mock()
    robot.get_fps.return_value = 200
    robot.get_firmware.side_effect = [
        UnicodeDecodeError('utf-8', b'\x00\x00\x00\x00\xda\xf2\x00\x00', 4, 5, 'invalid'),
        {'software_version': 'S-V1.9-1'},
    ]
    with patch('piper_x_driver.backend.AgxArmFactory.create_arm', return_value=robot):
        with patch('piper_x_driver.backend.time.sleep'):
            adapter = SdkArm('can2')
    assert robot.get_firmware.call_count == 2
    assert robot._parser.firmware_info.msg.clear.call_count == 3
    adapter.close()


def test_sdk_missing_receive_stream_fails_and_disconnects():
    from unittest.mock import Mock

    robot = Mock()
    robot.get_fps.return_value = 0
    with patch('piper_x_driver.backend.AgxArmFactory.create_arm', return_value=robot):
        with patch('piper_x_driver.backend.time.monotonic', side_effect=[0.0, 2.1]):
            with pytest.raises(RuntimeError, match='receive stream'):
                SdkArm('can0')
    robot.get_firmware.assert_not_called()
    robot.disconnect.assert_called_once()


def offline_adapter() -> Any:
    adapter = SdkArm.__new__(SdkArm)
    adapter._initialize_shutdown()
    return adapter


def test_sdk_configures_rx_whitelist_and_bounded_socket_rcvbuf():
    import socket
    from piper_x_driver.backend import _SOCKETCAN_RCVBUF_REQUEST

    applied_filters = []
    sock_opts = []
    backlog = [object(), object(), None]
    recv_timeouts = []

    def fake_recv(timeout=None):
        recv_timeouts.append(timeout)
        return backlog.pop(0)

    recv_bus = SimpleNamespace(
        set_filters=lambda flt: applied_filters.append(flt),
        socket=SimpleNamespace(setsockopt=lambda *args: sock_opts.append(args)),
        recv=fake_recv,
    )
    adapter = offline_adapter()
    adapter.robot = SimpleNamespace(
        _ctx=SimpleNamespace(comm=SimpleNamespace(recv_bus=recv_bus)),
        _parser=SimpleNamespace(_rx_map={0x2A5: object(), 0x261: object(), 0x476: object()}),
    )
    adapter.gripper = SimpleNamespace(
        _parser=SimpleNamespace(_rx_map={0x2A8: object(), 0x476: object()}),
    )
    adapter._configure_rx_socket()
    assert applied_filters == [[
        {'can_id': cid, 'can_mask': 0x7FF, 'extended': False}
        for cid in (0x261, 0x2A5, 0x2A8, 0x476)
    ]]
    assert sock_opts == [(socket.SOL_SOCKET, socket.SO_RCVBUF, _SOCKETCAN_RCVBUF_REQUEST)]
    assert recv_timeouts == [0.0, 0.0, 0.0]


@pytest.mark.parametrize('entrypoint', ['power', 'close'])
def test_backend_mit_exit_and_close_enter_firmware_drag_mode(entrypoint):
    adapter = offline_adapter()
    events = []
    adapter.robot = SimpleNamespace(
        set_motion_mode=lambda _: None,
        move_mit=lambda *args: events.append(('mit', args)),
        set_leader_mode=lambda: events.append(('leader',)),
        restore_leader_drag_mode=lambda: events.append(('restore_drag',)),
        set_follower_mode=lambda: events.append(('follower',)),
        disable=lambda index: events.append(('disable', index)),
        disconnect=lambda: events.append(('disconnect',)),
    )
    adapter.gripper = SimpleNamespace(
        disable_gripper=lambda: events.append(('gripper',)))
    adapter.mode('mit')
    adapter.mit(target(torque=[0.1] * 6))
    if entrypoint == 'power':
        adapter.power(False, 0.0)
        assert events[-7:] == [('disable', i) for i in range(1, 7)] + [('gripper',)]
        assert adapter._mode == 'standby'
    else:
        adapter.close()
        assert events[-3:] == [('leader',), ('restore_drag',), ('disconnect',)]
        assert adapter._mode == 'drag'


def test_sdk_mit_scales_post_reduction_kp_kd_and_calibrates_gravity_offset():
    from piper_x_driver.backend import MAX_GRAVITY_OFFSET, MIT_GAIN_SCALE, MitTarget

    adapter = offline_adapter()
    sent = []
    adapter.robot = SimpleNamespace(
        move_mit=lambda *args: sent.append(args),
    )
    base = (0.0, 0.2, -0.4, -1.0, 0.0, 0.0)
    kp = (5.0, 5.0, 5.0, 1.0, 0.7, 0.7)
    kd = (0.5, 0.8, 0.8, 0.2, 0.2, 0.2)
    gravity = (0.0, 2.6, -4.0, 0.33, 0.0, 0.0)
    offset = tuple(g / k for g, k in zip(gravity, kp))
    position = tuple(q + off for q, off in zip(base, offset))
    adapter.mit(MitTarget(position, (0.0,) * 6, kp, kd, (0.0,) * 6, offset))
    assert len(sent) == 6
    for idx, (j, p_wire, v_wire, kp_wire, kd_wire, t_wire) in enumerate(sent):
        assert j == idx + 1
        assert v_wire == 0.0
        assert abs(p_wire - base[idx]) <= MAX_GRAVITY_OFFSET + 1e-12
        assert kd_wire == pytest.approx(kd[idx] / MIT_GAIN_SCALE[idx])
        kp_uint = int(kp_wire * 4095 / 500.0)
        kp_realized = MIT_GAIN_SCALE[idx] * (kp_uint * 500.0 / 4095.0)
        assert (p_wire - base[idx]) * kp_realized + t_wire == pytest.approx(gravity[idx])


def test_controller_applies_configured_friction_compensation_in_mit(fake_arm):
    from piper_x_driver.friction import ArmFrictionModel, JointFrictionModel, SpatialHarmonic

    arm = fake_arm()
    j1_model = JointFrictionModel(
        fc=0.3, fs=0.6, vs=0.15, fv=0.1, v0=0.005, vr=0.4,
        harmonics=(SpatialHarmonic(omega=36.0, a=0.12, b=-0.05, c=0.08, d=0.03),),
    )
    friction_model = ArmFrictionModel(arm_id='arm_1', joints={1: j1_model})
    control = Controller(arm, friction_model=friction_model)
    control.start_mit()
    # Stationary arm: friction compensation is strictly 0.0
    assert control.friction == (0.0,) * 6

    # Moving J1 at slow velocity: friction compensation is added to MIT torque
    base_snapshot = arm.snapshot
    vel = (0.05, 0.0, 0.0, 0.0, 0.0, 0.0)
    arm.snapshot = lambda: replace(base_snapshot(), velocity=vel)
    arm.status['ctrl_mode'] = 1
    arm.status['mode_feedback'] = 6
    control.watchdog()
    expected_f1 = j1_model.torque(arm.position[0], vel[0])
    assert expected_f1 > 0.4
    assert control.friction[0] == pytest.approx(expected_f1)
    assert control.output_torque[0] == pytest.approx(expected_f1)
    assert arm.calls[-1][1].torque[0] == pytest.approx(expected_f1)

    # Zeroing friction_scale disables friction compensation
    control.friction_scale = 0.0
    control.friction_scale_applied = 0.0
    control.watchdog()
    assert control.friction == (0.0,) * 6
    assert arm.calls[-1][1].torque[0] == pytest.approx(0.0)

    # Zero-gain drag (kp=0, kd=0) bounds zero-speed negative damping so motor velocity noise cannot self-excite
    control.friction_scale = 1.0
    control.friction_scale_applied = 1.0
    arm.snapshot = lambda: replace(base_snapshot(), velocity=(0.005, 0.0, 0.0, 0.0, 0.0, 0.0))
    control.mit(target(kp=[0.0] * 6, kd=[0.0] * 6))
    control.watchdog()
    assert 0.0 < control.friction[0] < 0.05


def test_stiction_breakaway_and_adaptive_stiffness_match_g1_laws(fake_arm):
    from piper_x_driver.controller import adaptive_stiffness_scale
    from piper_x_driver.friction import ArmFrictionModel, JointFrictionModel
    from piper_x_driver.gravity import ArmGravityModel, JointGravityModel, gravity_torques

    # 1. Adaptive stiffness law: intercept 1+scale at 0, half-extra at b, never <1, kd ~ sqrt(scale)
    assert adaptive_stiffness_scale(0.0, scale=2.0, b=0.05, power=1.0) == pytest.approx(3.0)
    assert adaptive_stiffness_scale(0.05, scale=2.0, b=0.05, power=1.0) == pytest.approx(2.0)
    assert adaptive_stiffness_scale(0.08, scale=2.0, b=0.08, power=3.0) == pytest.approx(2.0)
    assert adaptive_stiffness_scale(-0.08, scale=2.0, b=0.08, power=3.0) == pytest.approx(2.0)
    assert adaptive_stiffness_scale(100.0, scale=2.0, b=0.08, power=3.0) > 1.0
    assert adaptive_stiffness_scale(100.0, scale=2.0, b=0.08, power=3.0) == pytest.approx(1.0, abs=1e-4)

    # 2. Target velocity step estimator recovers 50 Hz staircase velocity at 200 Hz control rate
    now = [0.0]
    arm = fake_arm()
    j1_fric = JointFrictionModel(
        fc=0.2, fs=0.4, load_ratio=0.15, error_epsilon=0.05, velocity_epsilon=0.02,
    )
    g_model = ArmGravityModel(
        arm_id='arm_4',
        joints={1: JointGravityModel(scale=1.0, static_friction=0.4), 2: JointGravityModel(scale=1.0, static_friction=0.5)},
    )
    f_model = ArmFrictionModel(arm_id='arm_4', joints={1: j1_fric, 2: j1_fric})
    control = Controller(
        arm,
        clock=lambda: now[0],
        gravity_model=g_model,
        friction_model=f_model,
        adaptive_stiffness_scale=2.0,
        adaptive_stiffness_b=0.05,
        adaptive_stiffness_power=2.0,
        offset_ramp_s=2.0,
    )
    control.start_mit()
    arm.status['ctrl_mode'] = 1
    arm.status['mode_feedback'] = 6
    base_q = arm.position[:6]

    # Activation fade-in over offset_ramp_s=2.0s: starts at 1.0x, reaches 1.5x at 0.5s, 2.0x at 1.0s, and settles at 3.0x
    base_kp = (17.0, 16.0, 15.0, 4.0, 3.0, 3.0)
    base_kd = (6.0, 6.0, 3.0, 0.6, 0.4, 0.1)
    assert control.applied_kp == pytest.approx(base_kp)
    now[0] = 0.5
    control.mit(target(position=list(base_q), kp=list(base_kp), kd=list(base_kd)))
    control.watchdog()
    assert control.applied_kp == pytest.approx(tuple(1.5 * k for k in base_kp))
    now[0] = 1.0
    control.mit(target(position=list(base_q), kp=list(base_kp), kd=list(base_kd)))
    control.watchdog()
    assert control.applied_kp == pytest.approx(tuple(2.0 * k for k in base_kp))
    now[0] = 2.0
    for step in range(1, 20):
        now[0] = 2.0 + step * 0.005
        control.mit(target(position=list(base_q), kp=list(base_kp), kd=list(base_kd)))
        control.watchdog()
    sent = arm.calls[-1][1]
    assert control.friction == (0.0,) * 6
    assert sent.kp == pytest.approx(tuple(3.0 * k for k in base_kp))
    assert sent.kd == pytest.approx(tuple(math.sqrt(3.0) * d for d in base_kd))
    expected_g = g_model.torques(base_q)
    # Gravity offset divides by the adaptive kp' the motor actually receives
    assert sent.position == pytest.approx([
        q + g / (3.0 * k) for q, g, k in zip(base_q, expected_g, base_kp)
    ])

    # Crawling / stuck in stiction deadband (dq_meas == 0, error = +0.10 rad on J2):
    # gets breakaway push (load_ratio * |G2| + static_friction) * tanh(0.10 / 0.05),
    # and adaptive kp at |e| = 2*b with power=2 is 1 + 2 / (2^2 + 1) = 1.4x
    base_snapshot = arm.snapshot
    arm.snapshot = lambda: replace(base_snapshot(), position=(*base_q, 0.02), velocity=(0.0,) * 6)
    q_cmd = list(base_q)
    q_cmd[1] = base_q[1] + 0.10
    now[0] += 0.1  # > 50 ms hold decay so target_velocity contribution is separated
    control.mit(target(position=q_cmd, kp=list(base_kp), kd=list(base_kd)))
    control.watchdog()
    # Advance past hold decay so only position error drives stiction
    for _ in range(80):
        now[0] += 0.005
        control.mit(target(position=q_cmd, kp=list(base_kp), kd=list(base_kd)))
        control.watchdog()
    expected_scale_j2 = 1.0 + 2.0 / ((0.10 / 0.05) ** 2 + 1.0)
    assert control.applied_kp[1] == pytest.approx(base_kp[1] * expected_scale_j2)
    assert control.applied_kd[1] == pytest.approx(base_kd[1] * math.sqrt(expected_scale_j2))
    expected_stiction_j2 = (0.15 * abs(expected_g[1]) + 0.5) * math.tanh(0.10 / 0.05)
    assert control.friction[1] == pytest.approx(expected_stiction_j2, rel=1e-3)

    # Reversal cancellation: error = +0.05 rad (+1 drive) and target_velocity = -0.02 rad/s (-1 drive) cancel to 0
    q_rev = list(base_q)
    q_rev[0] = base_q[0] + 0.05
    now[0] += 0.005
    control.mit(target(position=q_rev, velocity=[-0.02, 0.0, 0.0, 0.0, 0.0, 0.0], kp=list(base_kp), kd=list(base_kd)))
    control.watchdog()
    assert control.friction[0] == pytest.approx(0.0, abs=1e-9)

    # Staircase target velocity recovery (50 Hz steps sampled at 200 Hz)
    control._reset_dynamics_state()
    q_step = list(base_q)
    vel_est = control._update_target_velocity(q_step, 0.005)
    speed = 0.25
    for tick in range(1, 401):
        if tick % 4 == 0:
            q_step[0] += speed * 0.02
        vel_est = control._update_target_velocity(q_step, 0.005)
    assert vel_est[0] == pytest.approx(speed, abs=0.005)


def test_controller_applies_nonperiodic_gripper_friction_in_active_and_free_drag(fake_arm):
    from piper_x_driver.friction import ArmFrictionModel, GripperFrictionModel

    arm = fake_arm()
    g_model = GripperFrictionModel(
        fc=0.40,
        fs=0.70,
        vs=0.015,
        fv=2.0,
        v0=0.002,
        vr=0.08,
        kp_equiv=100.0,
        w_grid=(0.005, 0.035, 0.065),
        bias_grid=(-0.20, -0.35, -0.55),
        modulation_grid=(0.15, -0.10, 0.20),
    )
    # Non-periodic spatial profile varies across stroke positions
    assert g_model.local_fc(0.005) != g_model.local_fc(0.035)
    assert g_model.bias_force(0.005) != g_model.bias_force(0.065)

    friction_model = ArmFrictionModel(arm_id='arm_1', gripper=g_model)
    control = Controller(arm, friction_model=friction_model)
    control.start_mit()
    arm.status['ctrl_mode'] = 1
    arm.status['mode_feedback'] = 6

    # 1. Slave / active control (effort > 0): preserves w_cmd without self-excitation and adds |f_comp| to effort
    base_snapshot = arm.snapshot
    arm.position = arm.position[:6] + (0.010,)
    arm.snapshot = lambda: replace(base_snapshot(), position=arm.position, gripper_velocity=0.05)
    control.gripper(SimpleNamespace(width=0.050, effort=1.5))
    last_call = arm.calls[-1]
    assert last_call[0] == 'grip'
    assert last_call[1] == pytest.approx(0.050)
    assert last_call[2] == pytest.approx(1.5 + abs(control.gripper_friction))
    assert control.gripper_friction > 0.0

    # 2. Master / passive drag (effort <= 0): velocity-indexed assist when moving, 0 when stationary
    arm.position = arm.position[:6] + (0.035,)
    arm.snapshot = lambda: replace(base_snapshot(), position=arm.position, gripper_velocity=0.02)
    control.gripper(SimpleNamespace(width=0.035, effort=0.0))
    assert arm.calls[-1][0] == 'grip'
    assert arm.calls[-1][2] > 0.0
    assert control.gripper_friction != 0.0

    # 3. Stationary (velocity == 0): drops to 0.0
    arm.snapshot = lambda: replace(base_snapshot(), position=arm.position, gripper_velocity=0.0)
    control.gripper(SimpleNamespace(width=0.035, effort=0.0))
    assert arm.calls[-1] == ('grip', 0.035, 0.0)
    assert control.gripper_friction == 0.0


def test_controller_applies_configured_gravity_model_in_mit(fake_arm):
    from piper_x_driver.gravity import ArmGravityModel, JointGravityModel, gravity_torques

    arm = fake_arm()
    g_model = ArmGravityModel(
        arm_id='arm_1',
        joints={2: JointGravityModel(scale=1.10, angle_offset=0.0, bias=-0.05, static_friction=0.7)},
    )
    control = Controller(arm, gravity_model=g_model)
    control.start_mit()
    cad = gravity_torques(arm.position[:6])
    assert control.gravity[1] == pytest.approx(1.10 * cad[1] - 0.05)




def test_negative_kd_in_mit_command_auto_derives_damping_in_driver(fake_arm, capsys):
    from piper_x_driver.gravity import damping_main, max_critical_kd

    arm = fake_arm()
    control = Controller(arm)
    control.power(True)
    kp = [60.0, 60.0, 60.0, 12.0, 8.0, 8.0]
    cmd = target(kp=kp, kd=[-1.0] * 6)
    control.mit(cmd)
    control.set_mode('mit')
    control.watchdog()
    sent = [call[1] for call in arm.calls if call[0] == 'mit'][-1]
    assert sent.kd == max_critical_kd(kp) == (9.25, 9.31, 4.51, 0.6, 0.49, 0.13)
    control.mit(target(kp=[-1.0] * 6, kd=[-1.0] * 6))
    control.watchdog()
    sent_auto = [call[1] for call in arm.calls if call[0] == 'mit'][-1]
    assert sent_auto.kp == control.kp
    assert sent_auto.kd == control.kd
    assert max_critical_kd((15.0, 15.0, 15.0, 3.0, 2.0, 2.0), 'gripper') == (
        4.56, 4.6, 2.22, 0.29, 0.24, 0.05)
    assert damping_main(['--kp', '60', '60', '60', '12', '8', '8']) == 0
    assert 'kd: [9.25, 9.31, 4.51, 0.6, 0.49, 0.13]' in capsys.readouterr().out


def test_end_effector_wrench_estimation_with_impedance_and_inertia(fake_arm):
    import numpy as np
    from piper_x_driver.gravity import (
        _kinematics,
        end_effector_wrench,
        geometric_jacobian,
        joint_inertias,
    )

    q = (0.2, 0.7, -0.9, 0.3, -0.4, 0.5)
    offset = (0.01, -0.02, 0.08)
    jac_base = geometric_jacobian(q, tool_offset_m=offset, frame='base_link')
    # Verify linear Jacobian columns against central finite differences of forward kinematics
    eps = 1e-6
    for idx in range(6):
        q_plus = (*q[:idx], q[idx] + eps, *q[idx + 1:])
        q_minus = (*q[:idx], q[idx] - eps, *q[idx + 1:])
        _, _, c_plus = _kinematics(q_plus, offset)
        _, _, c_minus = _kinematics(q_minus, offset)
        assert jac_base[:3, idx] == pytest.approx((c_plus[6] - c_minus[6]) / (2.0 * eps), abs=1e-6)

    # Verify full 6-axis local gripper wrench (3D force N + 3D torque Nm) round-trip through J(q)^T
    jac = geometric_jacobian(q, tool_offset_m=offset)
    true_wrench = (12.0, -8.5, 25.0, 0.8, -1.2, 0.45)
    tau_from_wrench = tuple(float(v) for v in (jac.T @ np.asarray(true_wrench)))
    assert end_effector_wrench(
        tau_from_wrench, q, tool_offset_m=offset, damping=0.0,
    ) == pytest.approx(true_wrench, abs=1e-6)
    assert end_effector_wrench(
        tau_from_wrench, q, tool_offset_m=offset, damping=1e-4,
    ) == pytest.approx(true_wrench, abs=1e-3)

    # Verify Controller estimates joint external torque (subtracting inertia) and 6D end wrench
    now = [0.0]
    arm = fake_arm()
    arm.position = (*q, 0.02)
    kp = (40.0, 40.0, 40.0, 10.0, 8.0, 6.0)
    kd = (4.0, 4.0, 3.0, 0.8, 0.6, 0.3)
    dq_meas = (0.1, -0.2, 0.15, 0.0, 0.05, -0.1)
    ddq = (1.5, -1.0, 0.8, -0.5, 0.4, -0.2)
    inertias = joint_inertias(q)
    expected_inertia = tuple(j_i * a_i for j_i, a_i in zip(inertias, ddq))
    jac_flange = geometric_jacobian(q)
    desired_tau_ext = tuple(float(v) for v in (jac_flange.T @ np.asarray(true_wrench)))
    base_snapshot = arm.snapshot
    arm.snapshot = lambda: replace(base_snapshot(), position=(*q, 0.02), velocity=dq_meas)
    control = Controller(arm, clock=lambda: now[0])
    control.start_mit()
    arm.status['ctrl_mode'] = 1
    arm.status['mode_feedback'] = 6
    # Next tick after dt=0.02s with velocity incremented by ddq * (0.04 + dt) so LPF outputs ddq
    dt = 0.02
    alpha = dt / (0.04 + dt)
    dq_next = tuple(dq_meas[i] + (ddq[i] / alpha) * dt for i in range(6))
    q_des_next = tuple(
        q[i] - (desired_tau_ext[i] - kd[i] * dq_next[i] - expected_inertia[i]) / kp[i]
        for i in range(6)
    )
    arm.snapshot = lambda: replace(base_snapshot(), position=(*q, 0.02), velocity=dq_next)
    now[0] = 0.02
    control.mit(target(position=list(q_des_next), velocity=[0.0] * 6, kp=list(kp), kd=list(kd)))
    control.watchdog()

    assert control.inertia_torque == pytest.approx(expected_inertia, rel=1e-6)
    assert control.external_torque == pytest.approx(desired_tau_ext, rel=1e-6)
    expected_wrench = end_effector_wrench(desired_tau_ext, q)
    assert control.end_wrench == pytest.approx(expected_wrench, rel=1e-6)
    assert control.end_force == pytest.approx(expected_wrench[:3], rel=1e-6)
    assert control.end_torque == pytest.approx(expected_wrench[3:], rel=1e-6)

    # Sub-4ms tick holds acceleration without finite-difference spike, and partial gravity_scale
    # does not leak uncompensated arm weight into external_torque
    now[0] = 0.021
    control.gravity_scale = 0.5
    control.gravity_scale_applied = 0.5
    from piper_x_driver.gravity import gravity_torques
    raw_g = gravity_torques(q)
    q_des_half_g = tuple(
        q_des_next[i] + 0.5 * raw_g[i] / kp[i]
        for i in range(6)
    )
    control.mit(target(position=list(q_des_half_g), velocity=[0.0] * 6, kp=list(kp), kd=list(kd)))
    control.watchdog()
    assert control.inertia_torque == pytest.approx(expected_inertia, rel=1e-6)
    assert control.external_torque == pytest.approx(desired_tau_ext, rel=1e-6)

    control.power(False)
    assert control.external_torque == (0.0,) * 6
    assert control.end_wrench == (0.0,) * 6
