"""Explicit command semantics, independent of ROS and CAN transport."""

from collections import deque
import math
import secrets
import time

from can import CanError

from .backend import (
    feedback_problem, fresh, MAX_GRAVITY_OFFSET, MIT_KD_LSB, MIT_KP_LSB, MitTarget,
)
from .configuration import bounded
from .gravity import (
    end_effector_wrench,
    gravity_torques,
    joint_inertias,
    max_critical_kd,
)

MIT_POSITION_LIMIT = 12.5
LOWER = (-MIT_POSITION_LIMIT,) * 6
UPPER = (MIT_POSITION_LIMIT,) * 6
# Slider starting spans only; these are not command or feedback limits.
SLIDER_LOWER = tuple(map(math.radians, (-150, 0, -170, -89, -89, -170)))
SLIDER_UPPER = tuple(map(math.radians, (150, 180, 0, 89, 89, 170)))
MODES = ('standby', 'joint', 'point', 'linear', 'mit', 'drag', 'hold')
DEBUG_TIMEOUT = 0.3
TARGET_STALE_SECONDS = 0.05


def _raised(value: float, power: float) -> float:
    if power == 1.0:
        return value
    if power == 2.0:
        return value * value
    if power == 3.0:
        return value * value * value
    return math.pow(value, power)


def adaptive_stiffness_scale(
    position_error: float,
    scale: float,
    b: float = 0.08,
    power: float = 3.0,
) -> float:
    """Raise joint stiffness near zero error while returning to 1.0 at large error:

        kp' / kp = 1 + scale / ((|e| / b)^power + 1)
    """
    if scale <= 0.0:
        return 1.0
    if b <= 0.0 or not math.isfinite(b):
        raise ValueError('adaptive_stiffness_b must be finite and positive')
    if power <= 0.0 or not math.isfinite(power):
        raise ValueError('adaptive_stiffness_power must be finite and positive')
    ratio = abs(float(position_error)) / float(b)
    return 1.0 + float(scale) / (_raised(ratio, float(power)) + 1.0)


class RateMeter:
    """Sliding-window event frequency tracker (Hz) over a monotonic clock window."""

    __slots__ = ('_stamps', '_window')

    def __init__(self, window: float = 1.0):
        self._stamps: deque[float] = deque(maxlen=1024)
        self._window = float(window)

    def mark(self, now: float) -> None:
        self._stamps.append(float(now))

    def hz(self, now: float) -> float:
        stamps = self._stamps
        cutoff = float(now) - self._window
        while stamps and stamps[0] < cutoff:
            stamps.popleft()
        n = len(stamps)
        if n < 2:
            return 0.0
        span = stamps[-1] - stamps[0]
        return round((n - 1) / span, 1) if span > 1e-6 else 0.0

    def clear(self) -> None:
        self._stamps.clear()


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
                 friction_error_epsilon=0.05, friction_velocity_epsilon=0.02,
                 target_velocity_cutoff_hz=5.0,
                 s_a=None,
                 adaptive_stiffness_scale=0.0, adaptive_stiffness_b=0.08,
                 adaptive_stiffness_power=3.0,
                 offset_ramp_s=2.0,
                 kp=None,
                 kd=None,
                 debug_kp=(17.0, 16.0, 15.0, 4.0, 3.0, 3.0),
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
        self.friction_error_epsilon = bounded(
            friction_error_epsilon, 'friction_error_epsilon', 0.0, 10.0,
        )
        self.friction_velocity_epsilon = bounded(
            friction_velocity_epsilon, 'friction_velocity_epsilon', 0.0, 100.0,
        )
        self.target_velocity_cutoff_hz = bounded(
            target_velocity_cutoff_hz, 'target_velocity_cutoff_hz', 0.0, 1000.0,
        )
        if self.target_velocity_cutoff_hz <= 0.0:
            raise ValueError('target_velocity_cutoff_hz must be finite and positive')
        raw_sa = adaptive_stiffness_scale if s_a is None else s_a
        self.adaptive_stiffness_scale = bounded(
            raw_sa, 's_a', 0.0, 20.0,
        )
        self.offset_ramp_s = bounded(offset_ramp_s, 'offset_ramp_s', 0.0, 30.0)
        self.adaptive_stiffness_scale_applied = (
            0.0 if self.offset_ramp_s > 0.0 else self.adaptive_stiffness_scale
        )
        self.adaptive_stiffness_b = bounded(
            adaptive_stiffness_b, 'adaptive_stiffness_b', 0.0, 10.0,
        )
        if self.adaptive_stiffness_b <= 0.0:
            raise ValueError('adaptive_stiffness_b must be finite and positive')
        self.adaptive_stiffness_power = bounded(
            adaptive_stiffness_power, 'adaptive_stiffness_power', 0.0, 20.0,
        )
        if self.adaptive_stiffness_power <= 0.0:
            raise ValueError('adaptive_stiffness_power must be finite and positive')
        self.gravity_updated = self.clock()
        self.kp = tuple(debug_kp if kp is None else kp)
        self.kd = self._resolve_kd(self.kp, debug_kd if kd is None else kd)
        self.debug_session = ''
        self.debug_desired = ()
        self.debug_target = ()
        self.debug_received = 0.0
        self.mit_started = 0.0
        self.mit_ready = False
        self.gravity = (0.0,) * 6
        self.friction = (0.0,) * 6
        self.output_torque = (0.0,) * 6
        self.applied_kp = self.kp
        self.applied_kd = self.kd
        self.target_velocity = (0.0,) * 6
        self._target_hold_time = [0.0] * 6
        self._previous_target = None
        self.joint_acceleration = (0.0,) * 6
        self.inertia_torque = (0.0,) * 6
        self.external_torque = (0.0,) * 6
        self.end_wrench = (0.0,) * 6
        self._last_mit_command = None
        self._prev_velocity = None
        self._vel_updated = None
        self.gripper_friction = 0.0
        self._gripper_assist_active = False
        self._gripper_received = None
        self.hold_reason = ''
        self._compute_wrench = True
        self.comp_rate = RateMeter()
        self.cmd_tx_rate = RateMeter()
        self.mit_tx_rate = RateMeter()
        self.grip_tx_rate = RateMeter()
        self.ros_cmd_rate = RateMeter()

    @property
    def end_force(self) -> tuple[float, float, float]:
        return (self.end_wrench[0], self.end_wrench[1], self.end_wrench[2])

    @property
    def end_torque(self) -> tuple[float, float, float]:
        return (self.end_wrench[3], self.end_wrench[4], self.end_wrench[5])

    @property
    def debug_kp(self) -> tuple[float, ...]:
        return self.kp

    @debug_kp.setter
    def debug_kp(self, value) -> None:
        self.kp = tuple(value)

    @property
    def debug_kd(self) -> tuple[float, ...]:
        return self.kd

    @debug_kd.setter
    def debug_kd(self, value) -> None:
        self.kd = tuple(value)

    @property
    def s_a(self) -> float:
        return self.adaptive_stiffness_scale

    @s_a.setter
    def s_a(self, value: float) -> None:
        self.adaptive_stiffness_scale = bounded(value, 's_a', 0.0, 20.0)

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

    def _reset_dynamics_state(self, *, clear_compensation=True):
        self.joint_acceleration = self.inertia_torque = (0.0,) * 6
        self.external_torque = self.end_wrench = (0.0,) * 6
        self._prev_velocity = None
        self._vel_updated = None
        self.target_velocity = (0.0,) * 6
        self._target_hold_time = [0.0] * 6
        self._previous_target = None
        if clear_compensation:
            self.gravity = self.friction = self.output_torque = (0.0,) * 6
            self._last_mit_command = None
            self.gripper_friction = 0.0
            self._gripper_assist_active = False

    def _clear_power_state(self):
        self.mode = 'standby'
        self.mit_target = None
        self.mit_received = None
        self._gripper_received = None
        self.debug_session = ''
        self._reset_dynamics_state(clear_compensation=True)
        self.hold_reason = ''
        self.comp_rate.clear()
        self.cmd_tx_rate.clear()
        self.mit_tx_rate.clear()
        self.grip_tx_rate.clear()

    def reconfigure(
        self,
        *,
        effector=None,
        gravity_scale=None,
        gravity_model=None,
        friction_scale=None,
        friction_model=None,
        friction_error_epsilon=None,
        friction_velocity_epsilon=None,
        target_velocity_cutoff_hz=None,
        s_a=None,
        adaptive_stiffness_scale=None,
        adaptive_stiffness_b=None,
        adaptive_stiffness_power=None,
        offset_ramp_s=None,
        update_models=False,
    ):
        if effector is not None or update_models:
            self.effector = effector
        if gravity_scale is not None:
            self.gravity_scale = bounded(gravity_scale, 'gravity_scale', 0, 1)
            if self.gravity_scale == 0.0:
                self.gravity_scale_applied = 0.0
        if friction_scale is not None:
            self.friction_scale = bounded(friction_scale, 'friction_scale', 0, 1)
            if self.friction_scale == 0.0:
                self.friction_scale_applied = 0.0
        if friction_error_epsilon is not None:
            self.friction_error_epsilon = bounded(
                friction_error_epsilon, 'friction_error_epsilon', 0.0, 10.0,
            )
        if friction_velocity_epsilon is not None:
            self.friction_velocity_epsilon = bounded(
                friction_velocity_epsilon, 'friction_velocity_epsilon', 0.0, 100.0,
            )
        if target_velocity_cutoff_hz is not None:
            cutoff = bounded(
                target_velocity_cutoff_hz, 'target_velocity_cutoff_hz', 0.0, 1000.0,
            )
            if cutoff <= 0.0:
                raise ValueError('target_velocity_cutoff_hz must be finite and positive')
            self.target_velocity_cutoff_hz = cutoff
        if s_a is not None:
            adaptive_stiffness_scale = s_a
        if adaptive_stiffness_scale is not None:
            self.adaptive_stiffness_scale = bounded(
                adaptive_stiffness_scale, 's_a', 0.0, 20.0,
            )
            if self.adaptive_stiffness_scale == 0.0:
                self.adaptive_stiffness_scale_applied = 0.0
        if adaptive_stiffness_b is not None:
            val_b = bounded(adaptive_stiffness_b, 'adaptive_stiffness_b', 0.0, 10.0)
            if val_b <= 0.0:
                raise ValueError('adaptive_stiffness_b must be finite and positive')
            self.adaptive_stiffness_b = val_b
        if adaptive_stiffness_power is not None:
            val_p = bounded(adaptive_stiffness_power, 'adaptive_stiffness_power', 0.0, 20.0)
            if val_p <= 0.0:
                raise ValueError('adaptive_stiffness_power must be finite and positive')
            self.adaptive_stiffness_power = val_p
        if offset_ramp_s is not None:
            self.offset_ramp_s = bounded(offset_ramp_s, 'offset_ramp_s', 0.0, 30.0)
        if update_models:
            self.gravity_model = gravity_model
            self.friction_model = friction_model

    def shutdown_to_drag(self) -> bool:
        if self.mode != 'drag' and (self.mode == 'mit' or getattr(self.arm, '_powered', False)):
            self.arm._enter_drag()
            self._clear_power_state()
            self.mode = 'drag'
            return True
        return False

    def start_mit(self):
        self.power(True)
        sample = self.healthy()
        assert sample.status is not None
        if sample.status['teach_status'] == 1:
            self.arm.mode('hold')
        command = self._gravity_command(sample)
        self.gravity_scale_applied = self.gravity_scale
        self.friction_scale_applied = self.friction_scale
        self.adaptive_stiffness_scale_applied = (
            0.0 if self.offset_ramp_s > 0.0 else self.adaptive_stiffness_scale
        )
        self.gravity_updated = self.clock()
        self._prev_velocity = None
        self._vel_updated = None
        self.target_velocity = (0.0,) * 6
        self._target_hold_time = [0.0] * 6
        self._previous_target = None
        self.mit_started = self.clock()
        self.mit_ready = (
            sample.status['ctrl_mode'] == 1 and sample.status['mode_feedback'] == 6
        )
        self._send_mit(command, sample)
        self.arm.mode('mit')
        self.mode = 'mit'
        self.hold_reason = ''

    def set_mode(self, mode, *, preload_mit=False):
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
                if preload_mit and (
                    self.mit_target is None
                    or self.mit_received is None
                    or self.clock() - self.mit_received > 0.2
                ):
                    self.mit(self._gravity_command(sample))
                if (self.mit_target is None or self.mit_received is None
                        or self.clock() - self.mit_received > 0.2):
                    raise ValueError('Publish a fresh six-joint MIT target before entering MIT')
                self.arm.mit(self.mit_target)
                now_tx = self.clock()
                self.mit_tx_rate.mark(now_tx)
                self.cmd_tx_rate.mark(now_tx)
        self.arm.mode(mode)
        self.mode = mode
        self.hold_reason = ''
        if mode == 'mit':
            self.mit_started = self.clock()
            self.mit_ready = False
            self.gravity_scale_applied = self.gravity_scale
            self.friction_scale_applied = self.friction_scale
            self.adaptive_stiffness_scale_applied = (
                0.0 if self.offset_ramp_s > 0.0 else self.adaptive_stiffness_scale
            )
            self.gravity_updated = self.clock()
            self._reset_dynamics_state(clear_compensation=False)
        else:
            self._reset_dynamics_state(clear_compensation=True)

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
                tuple(base.kp) if base is not None else self.kp,
                tuple(base.kd) if base is not None else self.kd,
                tuple(base.torque) if base is not None else (0.0,) * 6,
            ))
            return
        now = self.clock()
        self.ros_cmd_rate.mark(now)
        self.require_mode('joint')
        self.arm.joints(command.position, command.speed_percent)
        self.cmd_tx_rate.mark(now)

    def cartesian(self, command):
        if len(command.pose) != 6:
            raise ValueError('pose: six values required')
        now = self.clock()
        self.ros_cmd_rate.mark(now)
        self.require_mode('point', 'linear')
        self.arm.cartesian(command.pose, command.speed_percent, self.mode)
        self.cmd_tx_rate.mark(now)

    def _resolve_kd(self, kp, kd=None):
        if kd is not None and all(float(v) >= 0.0 for v in kd):
            return tuple(float(v) for v in kd)
        eff = 'teaching_handle' if self.effector is None else getattr(self.effector, 'model', 'teaching_handle')
        auto = max_critical_kd(kp, eff, self.gravity_model)
        return auto if kd is None else tuple(a if float(v) < 0 else float(v) for v, a in zip(kd, auto))

    def _resolve_command_gains(self, command):
        kp_vec = tuple(
            self.kp[idx] if float(v) < 0.0 else float(v)
            for idx, v in enumerate(command.kp)
        )
        kd_vec = (
            self.kd
            if all(float(k) < 0.0 for k in command.kp) and all(float(d) < 0.0 for d in command.kd)
            else self._resolve_kd(kp_vec, command.kd)
        )
        return kp_vec, kd_vec

    def mit(self, command):
        if self.debug_session:
            raise ValueError('MIT debugger owns this arm; external MIT target rejected')
        for name in ('position', 'velocity', 'kp', 'kd', 'torque'):
            if len(getattr(command, name)) != 6:
                raise ValueError(f'{name}: six values required')
        if any(float(v) < 0.0 for v in command.kp) or any(float(v) < 0.0 for v in command.kd):
            kp_resolved, kd_resolved = self._resolve_command_gains(command)
            command = MitTarget(
                tuple(command.position),
                tuple(command.velocity),
                kp_resolved,
                kd_resolved,
                tuple(command.torque),
                tuple(getattr(command, 'gravity_offset', (0.0,) * 6)),
            )
        self.mit_target = command
        now = self.clock()
        self.mit_received = now
        self.ros_cmd_rate.mark(now)

    def debug_start(self):
        sample = self.healthy()
        assert sample.position is not None
        now = self.clock()
        active_debug = bool(self.debug_session and now - self.debug_received <= DEBUG_TIMEOUT)
        if active_debug or self.mode not in ('standby', 'mit'):
            raise ValueError('Stop other controllers and select standby before MIT debugging')
        if self.mit_received is not None and now - self.mit_received <= 0.2:
            raise ValueError('Another MIT source is preloading targets')
        measured = sample.position[:6]
        if not (self.mode == 'mit' and self.mit_ready):
            self.mit(MitTarget(measured, (0.0,) * 6, (0.0,) * 6,
                               self.kd, (0.0,) * 6))
            if self.ros_cmd_rate._stamps:
                self.ros_cmd_rate._stamps.pop()
            self.set_mode('mit')
        self.debug_session = secrets.token_urlsafe(24)
        self.debug_desired = self.debug_target = measured
        self.debug_received = now
        if self.mode == 'mit' and self.mit_ready:
            self._send_mit(self._debug_command(now), sample)
        return self.debug_session

    def debug_update(self, session, position):
        if not self.debug_session or not secrets.compare_digest(session, self.debug_session):
            raise ValueError('MIT debugging session expired or belongs to another page')
        if len(position) != 6:
            raise ValueError('MIT position requires six values')
        self.debug_desired = self.debug_target = tuple(position)
        now = self.clock()
        self.debug_received = now
        self.ros_cmd_rate.mark(now)

    def debug_stop(self, session):
        expired = bool(self.debug_session and self.clock() - self.debug_received > DEBUG_TIMEOUT)
        if not self.debug_session or (
            not expired and not secrets.compare_digest(session, self.debug_session)
        ):
            raise ValueError('MIT debugging session expired; disable remains available')
        self.debug_session = ''
        self.debug_desired = self.debug_target = ()
        self.mit_target = None
        self.mit_received = None

    def _debug_command(self, now):
        if now - self.debug_received > DEBUG_TIMEOUT:
            raise ValueError('MIT slider heartbeat exceeded 0.3 s')
        self.debug_target = self.debug_desired
        return MitTarget(self.debug_target, (0.0,) * 6, self.kp,
                         self.kd, (0.0,) * 6)

    def _gravity_command(self, sample):
        assert sample.position is not None
        return MitTarget(
            sample.position[:6], (0.0,) * 6,
            self.kp, self.kd, (0.0,) * 6,
        )

    def reset_wrench_estimation(self):
        self._prev_velocity = None
        self._vel_updated = None
        self.joint_acceleration = (0.0,) * 6
        self.inertia_torque = (0.0,) * 6
        self.external_torque = (0.0,) * 6
        self.end_wrench = (0.0,) * 6

    def _update_acceleration(self, sample, command, now: float) -> tuple[float, ...]:
        has_measured_velocity = len(sample.velocity) >= 6
        velocity = (
            sample.velocity[:6]
            if has_measured_velocity
            else tuple(command.velocity)
        )
        raw_accel = getattr(sample, 'acceleration', ())
        if len(raw_accel) >= 6:
            accel = tuple(float(a) for a in raw_accel[:6])
        elif has_measured_velocity:
            vel_now = tuple(float(v) for v in sample.velocity[:6])
            if self._prev_velocity is None or self._vel_updated is None:
                accel = (0.0,) * 6
                self._prev_velocity = vel_now
                self._vel_updated = now
            else:
                dt_vel = now - self._vel_updated
                if 0.0 <= dt_vel < 0.0015 or (
                    vel_now == self._prev_velocity and 0.0 <= dt_vel < 0.0045
                ):
                    accel = self.joint_acceleration
                elif 0.0015 <= dt_vel <= 0.2:
                    alpha = dt_vel / (0.04 + dt_vel)
                    accel = tuple(
                        (1.0 - alpha) * a_prev + alpha * ((v - v_prev) / dt_vel)
                        for a_prev, v, v_prev in zip(
                            self.joint_acceleration, vel_now, self._prev_velocity,
                        )
                    )
                    self._prev_velocity = vel_now
                    self._vel_updated = now
                else:
                    accel = (0.0,) * 6
                    self._prev_velocity = vel_now
                    self._vel_updated = now
        else:
            accel = (0.0,) * 6
            self._prev_velocity = None
            self._vel_updated = None
        self.joint_acceleration = accel
        return velocity

    def _update_target_velocity(self, target_position, elapsed: float) -> tuple[float, ...]:
        """Estimate commanded target velocity across setpoint steps (like G1 GravityFeedforward)."""
        target = tuple(float(v) for v in target_position[:6])
        if self._previous_target is None or len(self._previous_target) != 6:
            self._previous_target = target
            self.target_velocity = (0.0,) * 6
            self._target_hold_time = [0.0] * 6
            return self.target_velocity
        if elapsed <= 0.0:
            return self.target_velocity
        alpha = 1.0 - math.exp(-2.0 * math.pi * self.target_velocity_cutoff_hz * elapsed)
        vel = list(self.target_velocity)
        prev = list(self._previous_target)
        for idx in range(6):
            self._target_hold_time[idx] += elapsed
            if target[idx] != prev[idx]:
                raw = (target[idx] - prev[idx]) / self._target_hold_time[idx]
                vel[idx] += alpha * (raw - vel[idx])
                prev[idx] = target[idx]
                self._target_hold_time[idx] = 0.0
            elif self._target_hold_time[idx] > TARGET_STALE_SECONDS:
                vel[idx] -= alpha * vel[idx]
        self._previous_target = tuple(prev)
        self.target_velocity = tuple(vel)
        return self.target_velocity

    def _resolve_adaptive_gains(
        self,
        base_kp: tuple[float, ...],
        base_kd: tuple[float, ...],
        position_errors: tuple[float, ...],
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """Compute error-adaptive (kp', kd') from nominal gains without compounding."""
        amount = self.adaptive_stiffness_scale_applied
        if amount <= 0.0:
            return tuple(float(k) for k in base_kp), tuple(float(d) for d in base_kd)
        kp_out = []
        kd_out = []
        for kp_val, kd_val, err in zip(base_kp, base_kd, position_errors):
            kp_f = float(kp_val)
            kd_f = float(kd_val)
            if kp_f <= 0.0 or kd_f <= 0.0:
                kp_out.append(kp_f)
                kd_out.append(kd_f)
                continue
            mult = adaptive_stiffness_scale(
                err, amount, self.adaptive_stiffness_b, self.adaptive_stiffness_power,
            )
            kp_new = min(500.0, kp_f * mult)
            scale_real = kp_new / kp_f
            kd_new = min(100.0, kd_f * math.sqrt(scale_real))
            kp_out.append(kp_new)
            kd_out.append(kd_new)
        return tuple(kp_out), tuple(kd_out)

    def _update_wrench_estimate(
        self,
        command,
        sample,
        resolved_kd=None,
        *,
        resolved_kp=None,
        velocity=None,
        raw_gravity=None,
        gravity=None,
        raw_friction=None,
        friction=None,
    ):
        assert sample.position is not None
        if velocity is None:
            velocity = (
                sample.velocity[:6]
                if len(sample.velocity) >= 6
                else tuple(command.velocity)
            )
        if resolved_kp is None:
            resolved_kp = tuple(command.kp)
        if resolved_kd is None:
            resolved_kd = self._resolve_kd(resolved_kp, command.kd)
        real_sdk = getattr(self.arm, '_real_sdk', lambda: False)()
        ext_torques = []
        for idx, (q_des, q_meas, dq_des, dq_meas, kp, kd, t_cmd, t_iner) in enumerate(zip(
            command.position,
            sample.position[:6],
            command.velocity,
            velocity,
            resolved_kp,
            resolved_kd,
            command.torque,
            self.inertia_torque,
        )):
            raw_uint = round(kp / MIT_KP_LSB[idx]) if kp > 0.0 else 0
            if raw_uint <= 0:
                kp_eff = 0.0
            elif real_sdk:
                kp_eff = min(4095, raw_uint) * MIT_KP_LSB[idx]
            else:
                kp_eff = kp
            uncomp_gravity = (
                0.0 if raw_gravity is None or gravity is None else (raw_gravity[idx] - gravity[idx])
            )
            uncomp_friction = (
                0.0 if raw_friction is None or friction is None else (raw_friction[idx] - friction[idx])
            )
            ext_torques.append(float(
                kp_eff * (q_meas - q_des)
                + kd * (dq_meas - dq_des)
                - t_cmd
                + t_iner
                + uncomp_gravity
                + uncomp_friction
            ))
        self.external_torque = tuple(ext_torques)
        self.end_wrench = end_effector_wrench(
            self.external_torque,
            sample.position[:6],
            gravity_model=self.gravity_model,
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
        if self.offset_ramp_s <= 0.0:
            self.adaptive_stiffness_scale_applied = self.adaptive_stiffness_scale
        elif self.mit_ready:
            sa_step = dt * max(1.0, abs(self.adaptive_stiffness_scale)) / self.offset_ramp_s
            self.adaptive_stiffness_scale_applied += max(
                -sa_step,
                min(sa_step, self.adaptive_stiffness_scale - self.adaptive_stiffness_scale_applied),
            )
        self.gravity_updated = now
        # 0. 计算关节位置跟踪误差 e = q_target - q_meas 与目标速度估计 dq_target，并先于补偿求得自适应增益 (kp', kd')
        position_errors = tuple(
            float(q_des) - float(q_meas)
            for q_des, q_meas in zip(command.position[:6], sample.position[:6])
        )
        step_target_vel = self._update_target_velocity(command.position[:6], dt)
        target_velocities = tuple(
            float(v_cmd) if abs(float(v_cmd)) > 1e-9 else float(v_step)
            for v_cmd, v_step in zip(command.velocity[:6], step_target_vel)
        )
        base_kd = self._resolve_kd(command.kp, command.kd)
        effective_kp, resolved_kd = self._resolve_adaptive_gains(
            tuple(command.kp), base_kd, position_errors,
        )
        self.applied_kp, self.applied_kd = effective_kp, resolved_kd
        # 1. 计算当前姿态下的六轴重力补偿力矩 g = gravity_scale_applied * g_model(q) (Nm)
        raw_gravity = (
            gravity_torques(sample.position[:6], **model)
            if self.gravity_model is None
            else self.gravity_model.torques(sample.position[:6], **model)
        )
        gravity = tuple(self.gravity_scale_applied * value for value in raw_gravity)
        # 2. 计算当前位置与速度下的六轴摩擦力补偿力矩 f = friction_scale_applied * f_model(q, dq) (Nm)
        compute_wrench = self._compute_wrench
        if compute_wrench:
            velocity = self._update_acceleration(sample, command, now)
        else:
            velocity = (
                sample.velocity[:6]
                if len(sample.velocity) >= 6
                else tuple(command.velocity)
            )
        static_frictions = (
            (0.0,) * 6
            if self.gravity_model is None
            else tuple(self.gravity_model.joint(i).static_friction for i in range(1, 7))
        )
        active_impedance = tuple(
            kd >= MIT_KD_LSB[idx] and round(kp / MIT_KP_LSB[idx]) > 0
            for idx, (kp, kd) in enumerate(zip(effective_kp, resolved_kd))
        )
        stiction_errors = tuple(
            err if active else 0.0
            for err, active in zip(position_errors, active_impedance)
        )
        stiction_target_vels = tuple(
            t_vel if active else 0.0
            for t_vel, active in zip(target_velocities, active_impedance)
        )
        raw_friction = (
            (0.0,) * 6
            if self.friction_model is None
            else tuple(self.friction_model.torques(
                sample.position[:6],
                velocity,
                position_errors=stiction_errors,
                target_velocities=stiction_target_vels,
                gravity_torques=raw_gravity,
                static_frictions=static_frictions,
                error_epsilon=self.friction_error_epsilon,
                velocity_epsilon=self.friction_velocity_epsilon,
            ))
        )
        friction = (
            (0.0,) * 6
            if self.friction_scale_applied == 0.0
            else tuple(
                self.friction_scale_applied * value
                for value in raw_friction
            )
        )
        # 3. 逐轴合成目标位置 (q + offset) 与前馈力矩 t_ff (t_cmd + residual + f)
        offsets, positions, torques = [], [], []
        for idx, (q, g, f, kp, kd, t_cmd) in enumerate(zip(
            command.position, gravity, friction, effective_kp, resolved_kd, command.torque,
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
            tuple(effective_kp), tuple(resolved_kd), tuple(torques), tuple(offsets),
        )
        self.arm.mit(output)
        self.comp_rate.mark(now)
        self.mit_tx_rate.mark(now)
        self.cmd_tx_rate.mark(now)
        self._last_mit_command = command
        self.gravity, self.friction, self.output_torque = (
            gravity, friction, tuple(torques),
        )
        # 4. 计算六轴关节受载力矩（阻抗恢复力矩扣除惯性力矩）与末端六轴力螺旋 (fx, fy, fz, tx, ty, tz)
        if compute_wrench:
            if any(a != 0.0 for a in self.joint_acceleration):
                eff_name = (
                    'teaching_handle'
                    if self.effector is None
                    else getattr(self.effector, 'model', 'teaching_handle')
                )
                inertias = (
                    joint_inertias(sample.position[:6], end_effector=eff_name, **model)
                    if self.gravity_model is None
                    else self.gravity_model.inertias(
                        sample.position[:6], end_effector=eff_name, **model
                    )
                )
                self.inertia_torque = tuple(
                    float(j_val * a_val) for j_val, a_val in zip(inertias, self.joint_acceleration)
                )
            else:
                self.inertia_torque = (0.0,) * 6
            self._update_wrench_estimate(
                command,
                sample,
                resolved_kd,
                resolved_kp=effective_kp,
                velocity=velocity,
                raw_gravity=raw_gravity,
                gravity=gravity,
                raw_friction=raw_friction,
                friction=friction,
            )
        external_grip_active = (
            self._gripper_received is not None
            and 0.0 <= now - self._gripper_received <= 0.2
        )
        if not external_grip_active:
            if self.mit_target is None:
                self._apply_gripper(float(sample.position[6]), 0.0, sample, explicit=False)
            elif self._gripper_assist_active:
                self.arm.grip(float(sample.position[6]), 0.0)
                self.gripper_friction = 0.0
                self._gripper_assist_active = False

    def _apply_gripper(self, width_cmd, effort_cmd, sample, explicit=True):
        width, effort = float(width_cmd), float(effort_cmd)
        was_assist = self._gripper_assist_active
        g_model = None if self.friction_model is None else self.friction_model.gripper
        if (g_model is not None and self.friction_scale_applied > 0.0
                and sample.position is not None):
            width, effort, self.gripper_friction = g_model.compensate_command(
                width, effort, float(sample.position[6]),
                float(getattr(sample, 'gripper_velocity', 0.0)),
                scale=self.friction_scale_applied,
            )
            self._gripper_assist_active = (effort_cmd <= 0.0 and effort > 0.0)
        else:
            self.gripper_friction = 0.0
            self._gripper_assist_active = False
        if effort <= 0.0 and not was_assist and (
            not explicit
            or (getattr(self.arm, '_real_sdk', lambda: False)()
                and len(sample.enabled) >= 7 and not sample.enabled[6])
        ):
            return
        if self._gripper_assist_active:
            try:
                self.arm.grip(width, effort, interpolate=False)
                self.grip_tx_rate.mark(self.clock())
                return
            except TypeError:
                pass
        self.arm.grip(width, effort)
        self.grip_tx_rate.mark(self.clock())

    def gripper(self, command):
        sample = self.healthy(enabled=False)
        self._gripper_received = self.clock()
        self._apply_gripper(command.width, command.effort, sample)

    def watchdog(self, *, compute_wrench=True):
        """Return one feedback snapshot after enforcing the MIT watchdog."""
        if self.mode == 'mit':
            sample = None
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
                        self.mit_tx_rate.mark(now)
                        self.cmd_tx_rate.mark(now)
                        self.arm.mode('mit')
                        return sample
                    if self.mit_ready or now - self.mit_started > 0.2:
                        raise ValueError('Arm did not acknowledge or left external MIT mode')
                    return sample
                self.mit_ready = True
                self._compute_wrench = bool(compute_wrench)
                try:
                    self._send_mit(command, sample)
                finally:
                    self._compute_wrench = True
                self.hold_reason = ''
            except (ValueError, RuntimeError, CanError) as error:
                self.hold_reason = str(error)
                try:
                    self.arm.retain_mit()
                    now_ret = self.clock()
                    self.mit_tx_rate.mark(now_ret)
                    self.cmd_tx_rate.mark(now_ret)
                    if (
                        compute_wrench
                        and sample is not None
                        and sample.position is not None
                        and self._last_mit_command is not None
                    ):
                        self.inertia_torque = (0.0,) * 6
                        model = {} if self.effector is None else {
                            'mass_kg': self.effector.mass_kg, 'com_m': self.effector.com_m}
                        raw_g = (
                            gravity_torques(sample.position[:6], **model)
                            if self.gravity_model is None
                            else self.gravity_model.torques(sample.position[:6], **model)
                        )
                        self._update_wrench_estimate(
                            self._last_mit_command,
                            sample,
                            self.applied_kd,
                            resolved_kp=self.applied_kp,
                            raw_gravity=raw_g,
                            gravity=self.gravity,
                        )
                except (ValueError, RuntimeError, CanError) as send_error:
                    raise ValueError(
                        f'{error}; last MIT output retained locally; retransmission failed: '
                        f'{send_error}; no disable requested') from send_error
                raise ValueError(
                    f'{error}; retaining last MIT output; no disable requested') from error
            return sample
        return self.arm.snapshot()
