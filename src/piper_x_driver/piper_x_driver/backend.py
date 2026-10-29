"""The only boundary that knows pyAgxArm and its pinned parser internals."""

from dataclasses import dataclass
import math
import time
from typing import Literal

from can import CanError
from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config
from pyAgxArm.protocols.can_protocol.msgs.piper.default import (
    ArmMsgLeaderFollowerModeConfig, ArmMsgMotionCtrl,
)


_JOINTS: tuple[Literal[1, 2, 3, 4, 5, 6], ...] = (1, 2, 3, 4, 5, 6)
_MOTION_MODES: dict[str, Literal['j', 'p', 'l', 'mit']] = {
    'joint': 'j', 'point': 'p', 'linear': 'l', 'mit': 'mit',
}
STATUS_FIELDS = (
    'ctrl_mode', 'arm_status', 'mode_feedback', 'teach_status',
    'motion_status', 'trajectory_num', 'err_code',
)
FEEDBACK_FIELDS = (
    'joint12(0x2A5)', 'joint34(0x2A6)', 'joint56(0x2A7)',
    'gripper(0x2A8)', 'arm_status(0x2A1)',
) + tuple(f'driver{i}(0x{0x260 + i:X})' for i in _JOINTS)
MIT_GAIN_SCALE = (24.75, 24.75, 28.41, 2.10, 1.36, 1.0)
_MIT_KP_MAX = 500.0
_MIT_KP_STEPS = 4095


@dataclass(frozen=True)
class MitTarget:
    position: tuple[float, ...]
    velocity: tuple[float, ...]
    kp: tuple[float, ...]
    kd: tuple[float, ...]
    torque: tuple[float, ...]
    gravity_offset: tuple[float, ...] = (0.0,) * 6


@dataclass(frozen=True)
class Feedback:
    position: tuple[float, ...] | None
    status: dict[str, int] | None
    enabled: tuple[bool, ...]
    age: tuple[float, ...]
    pose: tuple[float, ...] | None = None
    velocity: tuple[float, ...] = ()
    torque: tuple[float, ...] = ()


def fresh(ages, timeout=1.0):
    return all(0 <= age <= timeout for age in ages)


def feedback_problem(ages, timeout=1.0):
    if len(ages) != len(FEEDBACK_FIELDS):
        return f'expected 11 feedback segments, received {len(ages)}'
    problems = []
    for field, age in zip(FEEDBACK_FIELDS, ages):
        if age is None or age == -1:
            problems.append(f'{field}=missing')
        elif not math.isfinite(age) or not 0 <= age <= timeout:
            problems.append(f'{field} age={age * 1000:.3f} ms '
                            f'(required 0..{timeout * 1000:g} ms)')
    return '; '.join(problems)


class SdkArm:
    def __init__(self, channel):
        self._initialize_shutdown()
        config = create_agx_arm_config(
            robot=ArmModel.PIPER_X, firmeware_version=PiperFW.V189,
            interface='socketcan', channel=channel, auto_connect=False,
        )
        self.robot = AgxArmFactory.create_arm(config)
        try:
            self.gripper = self.robot.init_effector('agx_gripper')
            self.robot.connect()
            if not self.robot.is_connected():
                raise RuntimeError(f'{channel}: SDK did not connect')
            # Exit native Leader mode if left from a previous session so 200Hz CAN push resumes.
            self.robot.set_follower_mode()
            # The pinned SDK discards firmware segments while receive FPS is zero.
            deadline = time.monotonic() + 2.0
            while self.robot.get_fps() <= 0:
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'{channel}: SDK receive stream did not become ready')
                time.sleep(0.02)
            identity = None
            for attempt in range(4):
                identity = (
                    self.robot.get_firmware(timeout=2.0)
                    if attempt == 0 else
                    self.robot.get_firmware(timeout=0.5, min_interval=0.0)
                )
                if (identity is not None
                        and identity['software_version'].strip('\x00 ') == 'S-V1.9-0'):
                    break
                time.sleep(0.05)
            if identity is None or identity['software_version'].strip('\x00 ') != 'S-V1.9-0':
                raise RuntimeError(f'{channel}: expected S-V1.9-0, received {identity}')
            self.robot.set_auto_set_motion_mode_enabled(False)
            # Do not silently clamp raw angles to the SDK model's preset workspace.
            self.robot.set_joint_limits_enabled(False)
            if self._real_sdk():
                self.robot._ctx.set_tx_repeat_min_interval(0x151, 0.0)
                wait_until = time.monotonic() + 1.5
                while time.monotonic() < wait_until:
                    sample = self.snapshot()
                    if (sample.status is not None and sample.position is not None
                            and fresh(sample.age)):
                        if sample.status['teach_status'] == 1:
                            self.robot._send_msg(ArmMsgMotionCtrl(grag_teach_ctrl=2))
                            time.sleep(0.05)
                        break
                    time.sleep(0.02)
        except (RuntimeError, ValueError, OSError, CanError):
            self.robot.disconnect()
            raise

    def _real_sdk(self) -> bool:
        return hasattr(self, 'robot') and type(self.robot).__module__.startswith('pyAgxArm')

    def _initialize_shutdown(self):
        self._mode = 'standby'
        self._powered = False
        self.mit_output = None

    def _enter_drag(self):
        self.mode('drag')
        self._powered = False
        self.mit_output = None

    def retain_mit(self):
        if self.mit_output is None:
            raise RuntimeError('No transmitted MIT output available to retain')
        self._write_mit(self.mit_output)

    def snapshot(self):
        parser = self.robot._parser
        parts = [getattr(parser, name, None) for name in ('joint_12', 'joint_34', 'joint_56')]
        grip = self.gripper.get_gripper_status()
        status = self.robot.get_arm_status()
        drivers = [self.robot.get_driver_states(i) for i in _JOINTS]
        motors = [self.robot.get_motor_states(i) for i in _JOINTS]
        pose_parts = [
            getattr(parser, name, None) for name in
            ('end_pose_xy', 'end_pose_zrx', 'end_pose_ryrz')
        ]
        # SDK receive callbacks mutate cached timestamps in place.
        timestamps = [getattr(x, 'timestamp', None) for x in (*parts, grip, status, *drivers)]
        motor_timestamps = [getattr(x, 'timestamp', None) for x in motors]
        pose_timestamps = [getattr(x, 'timestamp', None) for x in pose_parts]
        now = time.time()
        ages = tuple(-1.0 if stamp is None else now - stamp for stamp in timestamps)
        # SDK aggregation can fill absent segments with zero; require every segment.
        joint = self.robot.get_joint_angles() if all(item is not None for item in parts) else None
        position = None
        if joint is not None and grip is not None:
            if grip.msg.mode != 'width':
                raise ValueError('This workspace requires width-mode gripper feedback')
            position = tuple(joint.msg) + (float(grip.msg.value),)
        enabled = tuple(
            item is not None and item.msg.foc_status.driver_enable_status for item in drivers
        ) + (grip is not None and grip.msg.foc_status.driver_enable_status,)
        fields = None if status is None else {
            key: int(getattr(status.msg, key)) for key in STATUS_FIELDS
        }
        velocity = torque = ()
        if all(stamp is not None and 0 <= now - stamp <= 1.0 for stamp in motor_timestamps):
            velocity = tuple(float(item.msg.velocity) for item in motors if item is not None)
            torque = tuple(float(item.msg.torque) for item in motors if item is not None)
        pose = None
        if all(stamp is not None and 0 <= now - stamp <= 1.0 for stamp in pose_timestamps):
            sample = self.robot.get_flange_pose()
            if sample is not None:
                pose = tuple(sample.msg)
        return Feedback(position, fields, enabled, ages, pose, velocity, torque)

    def calibrate_effector(self):
        if not self.gripper.calibrate_gripper(timeout=1.0):
            raise RuntimeError('Device did not acknowledge effector zero calibration; '
                               'result unknown, inspect feedback before any further operation')

    def power(self, enabled, width=0.0):
        if self._mode == 'drag' and hasattr(self, 'robot'):
            self.robot.set_follower_mode()
        self._write_power(enabled, width if enabled else 0.0)
        if self._real_sdk():
            deadline = time.monotonic() + 1.5
            while (not all(self.snapshot().enabled[:6]) if enabled
                   else any(self.snapshot().enabled)):
                if time.monotonic() >= deadline:
                    break
                time.sleep(0.02)
        self._powered = bool(enabled)
        if not enabled:
            self._mode = 'standby'
            self.mit_output = None

    def _write_power(self, enabled, *_):
        errors = []
        for index in _JOINTS:
            try:
                (self.robot.enable if enabled else self.robot.disable)(index)
            except CanError as error:
                errors.append(f'J{index}: {error}')
                if enabled:
                    break
        if not enabled or not errors:
            try:
                self.gripper.disable_gripper()
            except CanError as error:
                errors.append(f'gripper: {error}')
        if errors:
            if enabled:
                try:
                    self.power(False, 0.0)
                except CanError as error:
                    errors.append(f'rollback: {error}')
            raise CanError('; '.join(errors))

    def mode(self, name):
        self._write_mode(name)
        self._mode = name

    def _write_mode(self, name):
        if name == 'drag':
            if self._real_sdk():
                self.gripper.disable_gripper()
                self.robot._send_msg(ArmMsgLeaderFollowerModeConfig(
                    linkage_config=0xFA, linkage_offset=0x10))
                self.robot.restore_leader_drag_mode()
                self.gripper.disable_gripper()
            else:
                self.robot.set_leader_mode()
                self.robot.restore_leader_drag_mode()
            return
        if self._mode == 'drag':
            self.robot.set_follower_mode()
        if name == 'hold':
            self.robot._send_msg(ArmMsgMotionCtrl(grag_teach_ctrl=2))
        elif name == 'standby':
            self.robot._send_msg(self.robot._MSG_ModeCtrl(ctrl_mode=0))
        else:
            self.robot.set_motion_mode(_MOTION_MODES[name])

    def joints(self, position, speed):
        self.robot.set_speed_percent(speed)
        self.robot.move_j(list(position))

    def cartesian(self, pose, speed, mode):
        self.robot.set_speed_percent(speed)
        (self.robot.move_p if mode == 'point' else self.robot.move_l)(list(pose))

    def mit(self, command):
        output = MitTarget(
            tuple(command.position), tuple(command.velocity),
            tuple(command.kp), tuple(command.kd), tuple(command.torque),
            tuple(getattr(command, 'gravity_offset', (0.0,) * 6)),
        )
        self._write_mit(command)
        self.mit_output = output

    def _write_mit(self, command):
        offsets = getattr(command, 'gravity_offset', (0.0,) * 6)
        for index, joint in enumerate(_JOINTS):
            scale = MIT_GAIN_SCALE[index]
            kp = command.kp[index]
            kd = command.kd[index]
            position = command.position[index]
            offset = offsets[index] if index < len(offsets) else 0.0
            if kp <= 0.0:
                kp_wire = 0.0
            else:
                kp_uint = max(
                    1, min(_MIT_KP_STEPS, round((kp / scale) * _MIT_KP_STEPS / _MIT_KP_MAX)))
                kp_wire = (kp_uint + 0.25) * _MIT_KP_MAX / _MIT_KP_STEPS
                kp_realized = scale * (kp_uint * _MIT_KP_MAX / _MIT_KP_STEPS)
                if offset != 0.0:
                    position = max(
                        -12.5, min(12.5, position - offset + offset * (kp / kp_realized)))
            self.robot.move_mit(
                joint, position, command.velocity[index],
                kp_wire, max(0.0, min(5.0, kd / scale)), command.torque[index],
            )

    def grip(self, width, effort):
        if effort <= 0.0:
            self.gripper.disable_gripper()
        else:
            self.gripper.move_gripper_m(value=width, force=effort)

    def close(self):
        try:
            if self._mode != 'drag' and (self._mode == 'mit' or self._powered):
                self._enter_drag()
        finally:
            self._disconnect()

    def _disconnect(self):
        self.robot.disconnect()
