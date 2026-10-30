"""Explicit command semantics, independent of ROS and CAN transport."""

import math
import secrets
import time

from can import CanError

from .backend import (
    feedback_problem, fresh, MAX_GRAVITY_OFFSET, MIT_KD_LSB, MIT_KP_LSB, MitTarget,
)
from .configuration import bounded
from .gravity import gravity_torques, max_critical_kd

MIT_POSITION_LIMIT = 12.5
LOWER = (-MIT_POSITION_LIMIT,) * 6
UPPER = (MIT_POSITION_LIMIT,) * 6
# Slider starting spans only; these are not command or feedback limits.
SLIDER_LOWER = tuple(map(math.radians, (-150, 0, -170, -89, -89, -170)))
SLIDER_UPPER = tuple(map(math.radians, (150, 180, 0, 89, 89, 170)))
MODES = ('standby', 'joint', 'point', 'linear', 'mit', 'drag', 'hold')
DEBUG_TIMEOUT = 0.3


def vector(values, name, lower, upper):
    if len(values) != 6 or any(
        not math.isfinite(value) or not lo <= value <= hi
        for value, lo, hi in zip(values, lower, upper)
    ):
        raise ValueError(f'{name}: six finite values within the documented bounds required')


class Controller:
    def __init__(self, arm, clock=time.monotonic, *, effector=None, gravity_scale=1.0,
                 gravity_model=None,
                 friction_scale=1.0, friction_model=None,
                 debug_kp=(15.0, 15.0, 15.0, 3.0, 2.0, 2.0),
                 debug_kd=None):
        self.arm = arm
        self.clock = clock
        self.mode = 'standby'
        self.mit_target = None
        self.mit_received = None
        self.effector = effector
        self.gravity_model = gravity_model
        self.friction_model = friction_model
        self.gravity_scale = bounded(gravity_scale, 'gravity_scale', 0, 1)
        self.gravity_scale_applied = self.gravity_scale
        self.friction_scale = bounded(friction_scale, 'friction_scale', 0, 1)
        self.friction_scale_applied = self.friction_scale
        self.gravity_updated = self.clock()
        self.debug_kp = tuple(debug_kp)
        self.debug_kd = max_critical_kd(self.debug_kp) if debug_kd is None else tuple(debug_kd)
        self.debug_session = ''
        self.debug_desired = ()
        self.debug_target = ()
        self.debug_received = 0.0
        self.mit_started = 0.0
        self.mit_ready = False
        self.gravity = (0.0,) * 6
        self.friction = (0.0,) * 6
        self.output_torque = (0.0,) * 6
        self.hold_reason = ''

    def feedback(self, sample=None):
        if sample is None:
            sample = self.arm.snapshot()
        if sample.status is None or not fresh(sample.age):
            details = feedback_problem(sample.age) or 'arm status missing'
            raise ValueError(
                f'Complete fresh joint/gripper/status/enable feedback required: {details}')
        if sample.position is None or len(sample.position) != 7 or len(sample.enabled) != 7:
            raise ValueError('Complete position feedback required')
        return sample

    def healthy(self, enabled=True, sample=None):
        sample = self.feedback(sample)
        assert sample.status is not None
        if sample.status['arm_status'] not in (0, 11, 12, 13) or sample.status['err_code']:
            raise ValueError(f'Firmware fault: {sample.status}')
        if enabled and not all(sample.enabled[:6]):
            raise ValueError('All six joints must be enabled')
        return sample

    def power(self, enabled):
        if enabled:
            if hasattr(self.arm, 'leave_drag'):
                self.arm.leave_drag()
            sample = self.healthy(enabled=False)
            assert sample.position is not None
            self.arm.power(True, sample.position[6])
        else:
            self.arm.power(False, 0.0)
            self._clear_power_state()

    def _clear_power_state(self):
        self.mode = 'standby'
        self.mit_target = None
        self.mit_received = None
        self.debug_session = ''
        self.gravity = self.friction = self.output_torque = (0.0,) * 6
        self.hold_reason = ''

    def start_mit(self):
        self.power(True)
        sample = self.healthy()
        assert sample.status is not None
        if sample.status['teach_status'] == 1:
            self.arm.mode('hold')
        command = self._gravity_command(sample)
        self.gravity_scale_applied = self.gravity_scale
        self.friction_scale_applied = self.friction_scale
        self.gravity_updated = self.clock()
        self.mit_started = self.clock()
        self.mit_ready = (
            sample.status['ctrl_mode'] == 1 and sample.status['mode_feedback'] == 6
        )
        self._send_mit(command, sample)
        self.arm.mode('mit')
        self.mode = 'mit'
        self.hold_reason = ''

    def set_mode(self, mode):
        if self.debug_session:
            raise ValueError('MIT debugger owns this arm; stop/disable it first')
        if mode not in MODES:
            raise ValueError(f'Unknown mode {mode}; expected {MODES}')
        if mode != 'drag' and hasattr(self.arm, 'leave_drag'):
            self.arm.leave_drag()
        if mode != 'standby':
            sample = self.healthy()
            if mode == 'mit':
                assert sample.status is not None
                if sample.status['teach_status'] == 1:
                    self.arm.mode('hold')
                if (self.mit_target is None or self.mit_received is None
                        or self.clock() - self.mit_received > 0.2):
                    raise ValueError('Publish a fresh six-joint MIT target before entering MIT')
                self.arm.mit(self.mit_target)
        self.arm.mode(mode)
        self.mode = mode
        self.hold_reason = ''
        if mode == 'mit':
            self.mit_started = self.clock()
            self.mit_ready = False
            self.gravity_scale_applied = self.gravity_scale
            self.friction_scale_applied = self.friction_scale
            self.gravity_updated = self.clock()
        else:
            self.gravity = self.friction = self.output_torque = (0.0,) * 6

    def require_mode(self, *modes):
        if self.mode not in modes:
            raise ValueError('Select the matching mode and wait for actual firmware feedback')
        sample = self.healthy()
        assert sample.status is not None
        expected = {'joint': 1, 'point': 0, 'linear': 2, 'mit': 6}
        if (sample.status['ctrl_mode'] != 1
                or sample.status['mode_feedback'] not in (expected[self.mode], 255)):
            raise ValueError('Select the matching mode and wait for actual firmware feedback')
        return sample

    def joints(self, command):
        if len(command.position) != 6:
            raise ValueError('position: six values required')
        if self.mode == 'mit':
            base = self.mit_target
            self.mit(MitTarget(
                tuple(command.position),
                tuple(base.velocity) if base is not None else (0.0,) * 6,
                tuple(base.kp) if base is not None else self.debug_kp,
                tuple(base.kd) if base is not None else self.debug_kd,
                tuple(base.torque) if base is not None else (0.0,) * 6,
            ))
            return
        self.require_mode('joint')
        self.arm.joints(command.position, command.speed_percent)

    def cartesian(self, command):
        if len(command.pose) != 6:
            raise ValueError('pose: six values required')
        self.require_mode('point', 'linear')
        self.arm.cartesian(command.pose, command.speed_percent, self.mode)

    def mit(self, command):
        if self.debug_session:
            raise ValueError('MIT debugger owns this arm; external MIT target rejected')
        for name in ('position', 'velocity', 'kp', 'kd', 'torque'):
            if len(getattr(command, name)) != 6:
                raise ValueError(f'{name}: six values required')
        self.mit_target = command
        self.mit_received = self.clock()

    def debug_start(self):
        sample = self.healthy()
        assert sample.position is not None
        if self.debug_session or self.mode not in ('standby', 'mit'):
            raise ValueError('Stop other controllers and select standby before MIT debugging')
        if self.mit_received is not None and self.clock() - self.mit_received <= 0.2:
            raise ValueError('Another MIT source is preloading targets')
        measured = sample.position[:6]
        self.mit(MitTarget(measured, (0.0,) * 6, (0.0,) * 6,
                           self.debug_kd, (0.0,) * 6))
        self.set_mode('mit')
        self.debug_session = secrets.token_urlsafe(24)
        self.debug_desired = self.debug_target = measured
        self.debug_received = self.clock()
        return self.debug_session

    def debug_update(self, session, position):
        if not self.debug_session or not secrets.compare_digest(session, self.debug_session):
            raise ValueError('MIT debugging session expired or belongs to another page')
        if len(position) != 6:
            raise ValueError('MIT position requires six values')
        self.debug_desired = tuple(position)
        self.debug_received = self.clock()

    def debug_stop(self, session):
        if not self.debug_session or not secrets.compare_digest(session, self.debug_session):
            raise ValueError('MIT debugging session expired; disable remains available')
        self.debug_session = ''
        self.debug_desired = self.debug_target = ()
        self.mit_target = None
        self.mit_received = None

    def _debug_command(self, now):
        if now - self.debug_received > DEBUG_TIMEOUT:
            raise ValueError('MIT slider heartbeat exceeded 0.3 s')
        self.debug_target = self.debug_desired
        return MitTarget(self.debug_target, (0.0,) * 6, self.debug_kp,
                         self.debug_kd, (0.0,) * 6)

    def _gravity_command(self, sample):
        assert sample.position is not None
        return MitTarget(
            sample.position[:6], (0.0,) * 6,
            self.debug_kp, self.debug_kd, (0.0,) * 6,
        )

    def _send_mit(self, command, sample):
        """合成重力补偿与摩擦补偿，并下发六轴 MIT 指令。

        电机驱动器底层 MIT 控制律：
            tau_motor = kp * (p_des - q_meas) + kd * (v_des - dq_meas) + t_ff

        本函数对各项力矩的叠加规则：
        1. 重力矩 `gravity` (g)：
           - 当关节有有效刚度与阻尼（kp、kd 均大于 CAN 最小量化步长）时，
             优先折算为目标位置偏置 `offset = clip(g / kp)` 叠加到 `p_des = q_cmd + offset`，
             使 kp * offset 产生重力支撑力矩；若 offset 被限幅（±MAX_GRAVITY_OFFSET 或 ±12.5 rad），
             未折算完的剩余重力 `residual = g - offset * kp` 补入前馈力矩 `t_ff`。
           - 当 kp 或 kd 为 0（如纯力矩透传 / 零刚度主臂）时，无法通过位置偏置产生力矩，
             此时 `offset = 0`，全部重力矩 `residual = g` 直接进入 `t_ff`。
        2. 摩擦力矩 `friction` (f)：
           - 根据当前实测位置 `sample.position[:6]` 与速度计算的摩擦+齿槽补偿力矩，
             不改变目标位置 `p_des`，直接叠加到前馈力矩 `t_ff` 上。
        3. 最终合成输出：
           - 发送位置：`positions[i] = command.position[i] + offset`
           - 发送前馈力矩：`torques[i] = command.torque[i] + residual + f`
        """
        assert sample.position is not None
        model = {} if self.effector is None else {
            'mass_kg': self.effector.mass_kg, 'com_m': self.effector.com_m}
        now = self.clock()
        dt = max(0.0, now - self.gravity_updated)
        # 重力补偿与摩擦补偿比例因子均按最大 1.0/s 斜率平滑过渡，避免突变跳变
        self.gravity_scale_applied += max(
            -dt, min(dt, self.gravity_scale - self.gravity_scale_applied))
        self.friction_scale_applied += max(
            -dt, min(dt, self.friction_scale - self.friction_scale_applied))
        self.gravity_updated = now
        # 1. 计算当前姿态下的六轴重力补偿力矩 g = gravity_scale_applied * g_model(q) (Nm)
        raw_gravity = (
            gravity_torques(sample.position[:6], **model)
            if self.gravity_model is None
            else self.gravity_model.torques(sample.position[:6], **model)
        )
        gravity = tuple(self.gravity_scale_applied * value for value in raw_gravity)
        # 2. 计算当前位置与速度下的六轴摩擦力补偿力矩 f = friction_scale_applied * f_model(q, dq) (Nm)
        velocity = (
            sample.velocity[:6]
            if len(sample.velocity) >= 6
            else tuple(command.velocity)
        )
        friction = (
            (0.0,) * 6
            if self.friction_model is None or self.friction_scale_applied == 0.0
            else tuple(
                self.friction_scale_applied * value
                for value in self.friction_model.torques(sample.position[:6], velocity)
            )
        )
        # 3. 逐轴合成目标位置 (q + offset) 与前馈力矩 t_ff (t_cmd + residual + f)
        offsets, positions, torques = [], [], []
        for idx, (q, g, f, kp, kd, t_cmd) in enumerate(zip(
            command.position, gravity, friction, command.kp, command.kd, command.torque,
        )):
            if kd < MIT_KD_LSB[idx] or round(kp / MIT_KP_LSB[idx]) <= 0:
                # 零刚度或零阻尼：重力无法折算为位置偏置，全部进入 t_ff
                offset = 0.0
                residual = g
            else:
                # 有刚度：重力优先折算为位置偏置 offset = g / kp，截断超出部分作为 residual 进入 t_ff
                lo = max(-MAX_GRAVITY_OFFSET, -MIT_POSITION_LIMIT - q)
                hi = min(MAX_GRAVITY_OFFSET, MIT_POSITION_LIMIT - q)
                raw = g / kp
                offset = max(lo, min(hi, raw))
                residual = 0.0 if offset == raw else g - offset * kp
            offsets.append(offset)
            positions.append(q + offset)
            # 前馈力矩 = 外部指令力矩 t_cmd + 重力截断残差 residual + 摩擦力矩 f
            torques.append(t_cmd + residual + f)
        output = MitTarget(
            tuple(positions), tuple(command.velocity),
            tuple(command.kp), tuple(command.kd), tuple(torques), tuple(offsets),
        )
        self.arm.mit(output)
        self.gravity, self.friction, self.output_torque = (
            gravity, friction, tuple(torques),
        )

    def gripper(self, command):
        self.healthy()
        self.arm.grip(command.width, command.effort)

    def watchdog(self):
        """Return one feedback snapshot after enforcing the MIT watchdog."""
        if self.mode == 'mit':
            try:
                sample = self.healthy()
                assert sample.status is not None and sample.position is not None
                now = self.clock()
                if self.debug_session:
                    command = self._debug_command(now)
                elif (self.mit_target is not None and self.mit_received is not None
                      and now - self.mit_received <= 0.2):
                    command = self.mit_target
                else:
                    if self.mit_target is not None:
                        self.mit_target = None
                        self.arm.grip(sample.position[6], 0.0)
                    command = self._gravity_command(sample)
                ready = sample.status['ctrl_mode'] == 1 and sample.status['mode_feedback'] == 6
                if not ready:
                    if sample.status['ctrl_mode'] in (0, 2):
                        if sample.status['teach_status'] == 1:
                            self.arm.mode('hold')
                            return sample
                        self.arm.mit(command)
                        self.arm.mode('mit')
                        return sample
                    if self.mit_ready or now - self.mit_started > 0.2:
                        raise ValueError('Arm did not acknowledge or left external MIT mode')
                    return sample
                self.mit_ready = True
                self._send_mit(command, sample)
                self.hold_reason = ''
            except (ValueError, RuntimeError, CanError) as error:
                self.hold_reason = str(error)
                try:
                    self.arm.retain_mit()
                except (ValueError, RuntimeError, CanError) as send_error:
                    raise ValueError(
                        f'{error}; last MIT output retained locally; retransmission failed: '
                        f'{send_error}; no disable requested') from send_error
                raise ValueError(
                    f'{error}; retaining last MIT output; no disable requested') from error
            return sample
        return self.arm.snapshot()
