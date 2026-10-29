"""Explicit command semantics, independent of ROS and CAN transport."""

import math
import secrets
import time

from can import CanError

from .backend import feedback_problem, fresh, MitTarget
from .configuration import bounded
from .gravity import gravity_torques

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
                 debug_kp=(15.0, 15.0, 15.0, 3.0, 2.0, 2.0),
                 debug_kd=(4.62, 4.65, 2.25, 0.30, 0.24, 0.06)):
        self.arm = arm
        self.clock = clock
        self.mode = 'standby'
        self.mit_target = None
        self.mit_received = None
        self.effector = effector
        self.gravity_scale = bounded(gravity_scale, 'gravity_scale', 0, 1)
        self.gravity_scale_applied = self.gravity_scale
        self.gravity_updated = self.clock()
        self.debug_kp, self.debug_kd = tuple(debug_kp), tuple(debug_kd)
        self.debug_session = ''
        self.debug_desired = ()
        self.debug_target = ()
        self.debug_received = 0.0
        self.mit_started = 0.0
        self.mit_ready = False
        self.gravity = (0.0,) * 6
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
        self.gravity = self.output_torque = (0.0,) * 6
        self.hold_reason = ''

    def start_mit(self):
        self.power(True)
        sample = self.healthy()
        assert sample.status is not None
        if sample.status['teach_status'] == 1:
            self.arm.mode('hold')
        command = self._gravity_command(sample)
        self.gravity_scale_applied = self.gravity_scale
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
            self.gravity_updated = self.clock()
        else:
            self.gravity = self.output_torque = (0.0,) * 6

    def require_mode(self, *modes):
        sample = self.healthy()
        assert sample.status is not None
        expected = {'joint': 1, 'point': 0, 'linear': 2, 'mit': 6}
        if (self.mode not in modes or sample.status['ctrl_mode'] != 1
                or sample.status['mode_feedback'] != expected[self.mode]):
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
        assert sample.position is not None
        model = {} if self.effector is None else {
            'mass_kg': self.effector.mass_kg, 'com_m': self.effector.com_m}
        now = self.clock()
        dt = max(0.0, now - self.gravity_updated)
        self.gravity_scale_applied += max(
            -dt, min(dt, self.gravity_scale - self.gravity_scale_applied))
        self.gravity_updated = now
        gravity = tuple(self.gravity_scale_applied * value for value in
                        gravity_torques(sample.position[:6], **model))
        offset = tuple(
            0.0 if kp == 0 else value / kp
            for value, kp in zip(gravity, command.kp)
        )
        position = tuple(a + b for a, b in zip(command.position, offset))
        torque = tuple(command.torque)
        output = MitTarget(position, tuple(command.velocity),
                           tuple(command.kp), tuple(command.kd), torque, offset)
        self.arm.mit(output)
        self.gravity, self.output_torque = gravity, torque

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
