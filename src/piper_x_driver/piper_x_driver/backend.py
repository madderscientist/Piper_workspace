"""The only boundary that knows pyAgxArm and its pinned parser internals."""

from collections import deque
from dataclasses import dataclass
import math
import socket
import threading
import time
from typing import Literal

from can import CanError
from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config
from pyAgxArm.protocols.can_protocol.msgs.piper.default import (
    ArmMsgLeaderFollowerModeConfig, ArmMsgMotionCtrl, ArmMsgParamEnquiryAndConfig,
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
_MIT_KD_SPAN = 10.0
_MIT_KP_STEPS = 4095
MIT_KP_LSB = tuple(scale * _MIT_KP_MAX / _MIT_KP_STEPS for scale in MIT_GAIN_SCALE)
MIT_KD_LSB = tuple(scale * _MIT_KD_SPAN / _MIT_KP_STEPS for scale in MIT_GAIN_SCALE)
MAX_GRAVITY_OFFSET = 1.0
SUPPORTED_FIRMWARE = ('S-V1.9-0', 'S-V1.9-1')
# Linux doubles setsockopt(SO_RCVBUF, n) internally; 65536 -> 131072 bytes (~137 sk_buffs / ~60 ms FIFO cap).
_SOCKETCAN_RCVBUF_REQUEST = 65536


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
    gripper_velocity: float = 0.0
    gripper_force: float = 0.0


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
            if self._real_sdk():
                self.robot.connect(start_read_thread=False)
                if not self.robot.is_connected():
                    raise RuntimeError(f'{channel}: SDK did not connect')
                self._configure_rx_socket()
                self.robot._ctx.start_th()
            else:
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
                fw_cache = getattr(getattr(self.robot, '_parser', None), 'firmware_info', None)
                if fw_cache is not None and hasattr(fw_cache, 'msg'):
                    fw_cache.msg.clear()
                try:
                    identity = (
                        self.robot.get_firmware(timeout=2.0)
                        if attempt == 0 else
                        self.robot.get_firmware(timeout=0.5, min_interval=0.0)
                    )
                except UnicodeDecodeError:
                    identity = None
                    if fw_cache is not None and hasattr(fw_cache, 'msg'):
                        fw_cache.msg.clear()
                if (identity is not None
                        and identity['software_version'].strip('\x00 ') in SUPPORTED_FIRMWARE):
                    break
                time.sleep(0.05)
            if (identity is None
                    or identity['software_version'].strip('\x00 ') not in SUPPORTED_FIRMWARE):
                raise RuntimeError(
                    f'{channel}: expected {SUPPORTED_FIRMWARE}, received {identity}')
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
                self._grip_thread = threading.Thread(
                    target=self._grip_interp_loop,
                    name=f'grip-interp-{channel}',
                    daemon=True,
                )
                self._grip_thread.start()
                # Stop unused SDK background FPS/monitor threads and lock contention after startup.
                for dev in (self.robot, self.gripper):
                    ctx = getattr(dev, '_ctx', None)
                    fps_mgr = getattr(ctx, 'fps', None)
                    if fps_mgr is not None:
                        fps_mgr.increment = lambda *_: None
                        fps_mgr.stop()
                    mon_stop = getattr(ctx, '_monitor_stop_event', None)
                    if mon_stop is not None:
                        mon_stop.set()
        except (RuntimeError, ValueError, OSError, CanError):
            self.robot.disconnect()
            raise

    def _real_sdk(self) -> bool:
        return hasattr(self, 'robot') and type(self.robot).__module__.startswith('pyAgxArm')

    def _configure_rx_socket(self) -> None:
        comm = getattr(getattr(self.robot, '_ctx', None), 'comm', None)
        recv_bus = getattr(comm, 'recv_bus', None)
        if recv_bus is None:
            return
        allowed_ids = set(getattr(getattr(self.robot, '_parser', None), '_rx_map', ()))
        allowed_ids |= set(getattr(getattr(self.gripper, '_parser', None), '_rx_map', ()))
        if allowed_ids and hasattr(recv_bus, 'set_filters'):
            recv_bus.set_filters([
                {'can_id': int(cid), 'can_mask': 0x7FF, 'extended': False}
                for cid in sorted(allowed_ids)
            ])
        sock = getattr(recv_bus, 'socket', None)
        if sock is not None:
            sock.setsockopt(
                socket.SOL_SOCKET, socket.SO_RCVBUF, _SOCKETCAN_RCVBUF_REQUEST
            )
        if hasattr(recv_bus, 'recv'):
            while recv_bus.recv(timeout=0.0) is not None:
                pass

    def _initialize_shutdown(self):
        self._mode = 'standby'
        self._powered = False
        self.mit_output = None
        self._grip_history: deque[tuple[float, float]] = deque(maxlen=16)
        self._grip_vel: float = 0.0
        self._last_joint_rx_stamp: float | None = None
        self.can_rx_updates: int = 0
        self._grip_cond = threading.Condition()
        self._grip_stop = False
        self._grip_thread: threading.Thread | None = None
        self._grip_cmd_pos: float | None = None
        self._grip_start_pos: float = 0.0
        self._grip_target_pos: float = 0.0
        self._grip_target_effort: float = 0.0
        self._grip_interp_t0: float = 0.0
        self._grip_last_cmd_time: float = 0.0
        self._grip_last_tx_time: float = 0.0
        self._grip_interp_active: bool = False

    def _enter_drag(self):
        self.mode('drag')
        self._powered = False
        self.mit_output = None

    def leave_drag(self):
        if self._mode != 'drag':
            return
        if hasattr(self, 'robot'):
            self.robot.set_follower_mode()
        self._mode = 'standby'
        if self._real_sdk():
            deadline = time.monotonic() + 1.0
            while not fresh(self.snapshot().age):
                if time.monotonic() >= deadline:
                    break
                time.sleep(0.02)

    def retain_mit(self):
        if self.mit_output is None:
            raise RuntimeError('No transmitted MIT output available to retain')
        self._write_mit(self.mit_output)

    def snapshot(self):
        is_real = self._real_sdk()
        parser = self.robot._parser
        parts = (
            getattr(parser, 'joint_12', None),
            getattr(parser, 'joint_34', None),
            getattr(parser, 'joint_56', None),
        )
        g_parser = getattr(self.gripper, '_parser', None)
        grip = (
            getattr(g_parser, 'gripper', None)
            if g_parser is not None
            else self.gripper.get_gripper_status()
        )
        if is_real:
            status = getattr(parser, 'arm_status', None)
            drivers = (
                getattr(parser, 'driver_state_1', None),
                getattr(parser, 'driver_state_2', None),
                getattr(parser, 'driver_state_3', None),
                getattr(parser, 'driver_state_4', None),
                getattr(parser, 'driver_state_5', None),
                getattr(parser, 'driver_state_6', None),
            )
            motors = (
                getattr(parser, 'motor_state_1', None),
                getattr(parser, 'motor_state_2', None),
                getattr(parser, 'motor_state_3', None),
                getattr(parser, 'motor_state_4', None),
                getattr(parser, 'motor_state_5', None),
                getattr(parser, 'motor_state_6', None),
            )
        else:
            status = self.robot.get_arm_status()
            drivers = [self.robot.get_driver_states(i) for i in _JOINTS]
            motors = [self.robot.get_motor_states(i) for i in _JOINTS]
        pose_parts = (
            getattr(parser, 'end_pose_xy', None),
            getattr(parser, 'end_pose_zrx', None),
            getattr(parser, 'end_pose_ryrz', None),
        )
        # SDK receive callbacks mutate cached timestamps in place.
        timestamps = [getattr(x, 'timestamp', None) for x in (*parts, grip, status, *drivers)]
        j_stamp = timestamps[0]
        if j_stamp is not None and j_stamp != self._last_joint_rx_stamp:
            self._last_joint_rx_stamp = j_stamp
            self.can_rx_updates += 1
        motor_timestamps = [getattr(x, 'timestamp', None) for x in motors]
        pose_timestamps = [getattr(x, 'timestamp', None) for x in pose_parts]
        now = time.time()
        ages = tuple(-1.0 if stamp is None else now - stamp for stamp in timestamps)
        position = None
        j12_part, j34_part, j56_part = parts
        if j12_part is not None and j34_part is not None and j56_part is not None and grip is not None:
            if grip.msg.mode != 'width':
                raise ValueError('This workspace requires width-mode gripper feedback')
            if is_real and hasattr(j12_part.msg, 'joint_1'):
                j12, j34, j56 = j12_part.msg, j34_part.msg, j56_part.msg
                position = (
                    float(j12.joint_1), float(j12.joint_2),
                    float(j34.joint_3), float(j34.joint_4),
                    float(j56.joint_5), float(j56.joint_6),
                    float(grip.msg.value),
                )
            else:
                joint = self.robot.get_joint_angles()
                if joint is not None:
                    position = tuple(joint.msg) + (float(grip.msg.value),)
        elif self._mode == 'drag' and is_real:
            leader_parts = [
                getattr(parser, name, None)
                for name in ('leader_joint_12', 'leader_joint_34', 'leader_joint_56')
            ]
            leader_stamps = [getattr(x, 'timestamp', None) for x in leader_parts]
            if all(s is not None and 0 <= now - s <= 1.0 for s in leader_stamps):
                leader = self.robot.get_leader_joint_angles()
                if leader is not None:
                    width = 0.0 if grip is None else float(grip.msg.value)
                    position = tuple(leader.msg) + (width,)
                    ages = tuple(
                        now - s if s is not None else -1.0 for s in leader_stamps
                    ) + (0.0,) + ages[4:]
        enabled = tuple(
            item is not None and item.msg.foc_status.driver_enable_status for item in drivers
        ) + (grip is not None and grip.msg.foc_status.driver_enable_status,)
        st_msg = None if status is None else status.msg
        fields = None if st_msg is None else {
            'ctrl_mode': int(st_msg.ctrl_mode),
            'arm_status': int(st_msg.arm_status),
            'mode_feedback': int(st_msg.mode_feedback),
            'teach_status': int(st_msg.teach_status),
            'motion_status': int(st_msg.motion_status),
            'trajectory_num': int(st_msg.trajectory_num),
            'err_code': int(st_msg.err_code),
        }
        velocity = torque = ()
        if all(stamp is not None and 0 <= now - stamp <= 1.0 for stamp in motor_timestamps):
            velocity = tuple(float(item.msg.velocity) for item in motors if item is not None)
            torque = tuple(float(item.msg.torque) for item in motors if item is not None)
        pose = None
        pxy_part, pzrx_part, pryrz_part = pose_parts
        if all(stamp is not None and 0 <= now - stamp <= 1.0 for stamp in pose_timestamps):
            if (
                is_real
                and pxy_part is not None
                and pzrx_part is not None
                and pryrz_part is not None
                and hasattr(pxy_part.msg, 'X_axis')
            ):
                pxy, pzrx, pryrz = pxy_part.msg, pzrx_part.msg, pryrz_part.msg
                pose = (
                    float(pxy.X_axis), float(pxy.Y_axis), float(pzrx.Z_axis),
                    float(pzrx.RX_axis), float(pryrz.RY_axis), float(pryrz.RZ_axis),
                )
            else:
                sample = self.robot.get_flange_pose()
                if sample is not None:
                    pose = tuple(sample.msg)
        gripper_velocity = 0.0
        gripper_force = 0.0
        if grip is not None and 0 <= ages[3] <= 1.0:
            grip_pos = float(grip.msg.value)
            raw_force = getattr(grip.msg, 'force', 0.0)
            if raw_force is not None and math.isfinite(float(raw_force)):
                gripper_force = float(raw_force)
            grip_stamp = timestamps[3]
            if grip_stamp is not None:
                if not self._grip_history or grip_stamp > self._grip_history[-1][0]:
                    if self._grip_history and grip_stamp - self._grip_history[-1][0] > 0.5:
                        self._grip_history.clear()
                    self._grip_history.append((grip_stamp, grip_pos))
                    while (len(self._grip_history) > 2
                           and grip_stamp - self._grip_history[0][0] > 0.030):
                        self._grip_history.popleft()
                if len(self._grip_history) >= 4 and (
                    abs(self._grip_history[-1][1] - self._grip_history[-4][1]) < 5e-5
                ):
                    # Stationary for the most recent 4 CAN frames (~20 ms) -> zero lag stop
                    self._grip_vel = 0.0
                elif len(self._grip_history) >= 2:
                    t0_g, p0_g = self._grip_history[0]
                    t1_g, p1_g = self._grip_history[-1]
                    dt = t1_g - t0_g
                    self._grip_vel = (p1_g - p0_g) / dt if dt >= 1e-4 else 0.0
                else:
                    self._grip_vel = 0.0
            gripper_velocity = self._grip_vel
        return Feedback(
            position, fields, enabled, ages, pose, velocity, torque,
            gripper_velocity, gripper_force,
        )

    def calibrate_effector(self):
        if not self.gripper.calibrate_gripper(timeout=1.0):
            raise RuntimeError('Device did not acknowledge effector zero calibration; '
                               'result unknown, inspect feedback before any further operation')

    def power(self, enabled, width=0.0):
        self.leave_drag()
        self._write_power(enabled, width if enabled else 0.0)
        if self._real_sdk():
            deadline = time.monotonic() + 1.5
            next_retry = time.monotonic() + 0.12
            while (not all(self.snapshot().enabled[:6]) if enabled
                   else any(self.snapshot().enabled)):
                now = time.monotonic()
                if now >= deadline:
                    break
                if now >= next_retry:
                    self._write_power(enabled, width if enabled else 0.0)
                    next_retry = now + 0.12
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
                self._disable_gripper()
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
                # 切入固件内置零力拖动前，将固件末端负载档位设为满载（0x477: set_end_load=0x02）
                self.robot._send_msg(ArmMsgParamEnquiryAndConfig(
                    end_load_param_setting_effective=0xAE, set_end_load=0x02))
                self._disable_gripper()
                self.robot._send_msg(ArmMsgLeaderFollowerModeConfig(
                    linkage_config=0xFA, linkage_offset=0x10))
                self.robot.restore_leader_drag_mode()
                self._disable_gripper()
            else:
                self.robot.set_leader_mode()
                self.robot.restore_leader_drag_mode()
            return
        self.leave_drag()
        if name == 'hold':
            self.robot._send_msg(ArmMsgMotionCtrl(grag_teach_ctrl=2))
        elif name == 'standby':
            self.robot._send_msg(self.robot._MSG_ModeCtrl(ctrl_mode=0))
        else:
            mode_code = _MOTION_MODES[name]
            self.robot.set_motion_mode(mode_code)
            if self._mode == 'mit' and name != 'mit' and self._real_sdk():
                time.sleep(0.02)
                self.robot.set_motion_mode(mode_code)

    def joints(self, position, speed):
        self.robot.set_speed_percent(speed)
        if self._real_sdk():
            self.robot.set_motion_mode('j')
        self.robot.move_j(list(position))

    def cartesian(self, pose, speed, mode):
        self.robot.set_speed_percent(speed)
        if self._real_sdk():
            self.robot.set_motion_mode('p' if mode == 'point' else 'l')
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
        """将关节侧 MIT 指令转换为 CAN 线侧参数并校准 12 位 kp 量化误差。

        注意：
        - 上层传入的 `kp`、`kd` 均为减速器输出端（关节侧）物理量；
          底层驱动器直接按电机侧尺度计算 PD，放大倍数为 `scale = MIT_GAIN_SCALE[index]`。
        - 上层传入的 `torque`（含外部 t_cmd + 重力残差 + 摩擦力矩 f）已经是关节侧 Nm，
          SDK 的 `move_mit` 内部会自动做力矩常数换算。
        - 因为 CAN 帧中 `kp` 只有 12 bit（4095 步），量化后的实际关节刚度 `kp_realized`
          与理想 `kp` 存在微小偏差。这里将位置偏置重标定为 `wire_offset = offset * (kp / kp_realized)`，
          若 `wire_offset` 被限幅，则把差值 `offset * kp - wire_offset * kp_realized` 补回 `torque` (t_ff)，
          保证 `wire_offset * kp_realized + torque` 严格等于期望的总补偿力矩。
        """
        offsets = getattr(command, 'gravity_offset', (0.0,) * 6)
        for index, joint in enumerate(_JOINTS):
            scale = MIT_GAIN_SCALE[index]
            kp = command.kp[index]
            kd = command.kd[index]
            position = command.position[index]
            offset = offsets[index] if index < len(offsets) else 0.0
            # 还原未叠加重力位置偏置前的原始目标角度 base_pos
            base_pos = position - offset
            # torque 初始已包含：command.torque + 重力截断残差 residual + 摩擦力矩 friction
            torque = command.torque[index]
            raw_uint = round(kp / MIT_KP_LSB[index]) if kp > 0.0 else 0
            if raw_uint <= 0 or kd < MIT_KD_LSB[index]:
                # kp 或 kd 低于最小量化步长时，不通过位置偏置输出重力，将 offset * kp 退回 t_ff
                kp_wire = 0.0 if raw_uint <= 0 else (
                    (min(_MIT_KP_STEPS, raw_uint) + 0.25) * _MIT_KP_MAX / _MIT_KP_STEPS
                )
                position = max(-12.5, min(12.5, base_pos))
                torque += offset * kp
            else:
                # 计算 12 位无符号量化后的线侧 kp_wire 与实际生效的关节刚度 kp_realized
                kp_uint = min(_MIT_KP_STEPS, raw_uint)
                kp_wire = (kp_uint + 0.25) * _MIT_KP_MAX / _MIT_KP_STEPS
                kp_realized = scale * (kp_uint * _MIT_KP_MAX / _MIT_KP_STEPS)
                if offset != 0.0:
                    lo = max(-MAX_GRAVITY_OFFSET, -12.5 - base_pos)
                    hi = min(MAX_GRAVITY_OFFSET, -base_pos + 12.5)
                    # 按实际量化刚度 kp_realized 缩放位置偏置，未覆盖的残差补入 torque (t_ff)
                    wire_offset = max(lo, min(hi, offset * (kp / kp_realized)))
                    position = base_pos + wire_offset
                    torque += offset * kp - wire_offset * kp_realized
                else:
                    position = max(-12.5, min(12.5, position))
            self.robot.move_mit(
                joint, position, command.velocity[index],
                kp_wire, max(0.0, min(5.0, kd / scale)), torque,
            )

    def _disable_gripper(self):
        if hasattr(self, '_grip_cond'):
            with self._grip_cond:
                self._grip_interp_active = False
                self._grip_cmd_pos = None
                self.gripper.disable_gripper()
        else:
            self.gripper.disable_gripper()

    def _grip_interp_loop(self):
        while True:
            with self._grip_cond:
                while not self._grip_stop and not self._grip_interp_active:
                    self._grip_cond.wait()
                if self._grip_stop:
                    return
                now = time.monotonic()
                while not self._grip_stop and self._grip_interp_active:
                    wait_sec = 0.005 - (now - self._grip_last_tx_time)
                    if wait_sec <= 0.0005:
                        break
                    self._grip_cond.wait(timeout=wait_sec)
                    now = time.monotonic()
                if self._grip_stop or not self._grip_interp_active:
                    continue
                alpha = min(1.0, (max(0.0, now - self._grip_interp_t0) + 0.005) / 0.020)
                w_sub = self._grip_start_pos + alpha * (
                    self._grip_target_pos - self._grip_start_pos
                )
                self._grip_cmd_pos = w_sub
                self._grip_last_tx_time = now
                if alpha >= 1.0:
                    self._grip_interp_active = False
                try:
                    self.gripper.move_gripper_m(
                        value=w_sub, force=self._grip_target_effort
                    )
                except (RuntimeError, ValueError, OSError, CanError):
                    pass

    def grip(self, width, effort, interpolate=True):
        if effort <= 0.0:
            self._disable_gripper()
            return
        target_w, target_eff = float(width), float(effort)
        if getattr(self, '_grip_thread', None) is None or not self._real_sdk():
            self.gripper.move_gripper_m(value=target_w, force=target_eff)
            return
        with self._grip_cond:
            if not interpolate:
                self._grip_interp_active = False
                self._grip_cmd_pos = None
                self.gripper.move_gripper_m(value=target_w, force=target_eff)
                return
            now = time.monotonic()
            dt = now - self._grip_last_cmd_time
            self._grip_last_cmd_time = now
            start_w = (
                (float(self._grip_history[-1][1]) if self._grip_history else target_w)
                if (self._grip_cmd_pos is None or dt > 0.080)
                else self._grip_cmd_pos
            )
            self._grip_target_pos = target_w
            self._grip_target_effort = target_eff
            if abs(target_w - start_w) <= 0.0002 or (0.0 < dt <= 0.007):
                self._grip_cmd_pos = target_w
                self._grip_interp_active = False
                self._grip_last_tx_time = now
                self.gripper.move_gripper_m(value=target_w, force=target_eff)
            else:
                self._grip_start_pos = start_w
                self._grip_interp_t0 = now
                self._grip_interp_active = True
                if now - self._grip_last_tx_time >= 0.004:
                    w_step = start_w + 0.25 * (target_w - start_w)
                    self._grip_cmd_pos = w_step
                    self._grip_last_tx_time = now
                    self.gripper.move_gripper_m(value=w_step, force=target_eff)
                self._grip_cond.notify()

    def close(self):
        try:
            if self._mode != 'drag' and (self._mode == 'mit' or self._powered):
                self._enter_drag()
        finally:
            self._disconnect()

    def _disconnect(self):
        if hasattr(self, '_grip_cond'):
            with self._grip_cond:
                self._grip_stop = True
                self._grip_interp_active = False
                self._grip_cond.notify_all()
        grip_thread = getattr(self, '_grip_thread', None)
        if grip_thread is not None and grip_thread.is_alive():
            grip_thread.join(timeout=0.2)
        self._grip_thread = None
        self.robot.disconnect()
