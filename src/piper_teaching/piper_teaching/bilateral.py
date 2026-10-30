"""Hardware-independent reciprocal position references and activation interlocks."""

from dataclasses import dataclass
import math

from piper_x_driver.backend import feedback_problem


JOINT_NAMES = tuple(f'joint{i}' for i in range(1, 7)) + ('gripper',)
_PEERS = {
    'bilateral': (2, 3, 0, 1),
    'master_slave': (0, 1, 0, 1),
}
TEACHING_MODES = tuple(_PEERS)
MASTER_INDICES = (0, 1)


class TeachingFault(RuntimeError):
    """A failed interlock that must stop coupling, never silently resume it."""


def peers_for_mode(mode: str) -> tuple[int, ...]:
    if mode not in _PEERS:
        raise ValueError(f'mode must be one of {TEACHING_MODES}')
    return _PEERS[mode]


@dataclass(frozen=True)
class TeachingConfig:
    mode: str = 'bilateral'
    kp: tuple[float, ...] = (0.0,) * 6
    kd: tuple[float, ...] = (-1.0,) * 6
    gripper_effort: float = 0.0
    feedback_timeout: float = 1.0
    engage_speed_deg_s: float = 24.0

    def __post_init__(self) -> None:
        if self.mode not in TEACHING_MODES:
            raise ValueError(f'mode must be one of {TEACHING_MODES}')
        if len(self.kp) != 6 or any(not math.isfinite(v) or not 0 <= v <= 500 for v in self.kp):
            raise ValueError('kp requires six finite values in [0, 500]')
        if len(self.kd) != 6 or any(not math.isfinite(v) or not -100 <= v <= 100 for v in self.kd):
            raise ValueError('kd requires six finite values in [-100, 100] (<0 selects driver auto-damping)')
        if not math.isfinite(self.gripper_effort) or not 0 <= self.gripper_effort <= 5:
            raise ValueError('gripper_effort requires a finite SDK value in [0, 5]')
        for name in ('feedback_timeout', 'engage_speed_deg_s'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.feedback_timeout > 2.0:
            raise ValueError('feedback_timeout must be <= 2.0s')

    def require_gains(self) -> None:
        if any(v <= 0 for v in self.kp) or any(v == 0 for v in self.kd) or self.gripper_effort <= 0:
            raise TeachingFault(
                'Set six positive kp values, non-zero kd values (<0 for driver auto-damping), '
                'and positive gripper_effort explicitly')


@dataclass(frozen=True)
class ArmSample:
    position: tuple[float, ...]
    received: tuple[float | None, ...]
    enabled: tuple[bool, ...]
    fault: str = ''
    mode: int = 0
    velocity: tuple[float, ...] = (0.0,) * 6


def measured_references(samples: tuple[ArmSample, ...], config: TeachingConfig,
                        now: float) -> tuple[tuple[float, ...], ...]:
    """Expose fresh measurements."""
    if len(samples) != 4:
        raise TeachingFault('Exactly four arm samples are required')
    peers = peers_for_mode(config.mode)
    for index, sample in enumerate(samples, 1):
        problem = feedback_problem(
            tuple(None if stamp is None else now - stamp for stamp in sample.received),
            config.feedback_timeout)
        if problem:
            raise TeachingFault(f'arm_{index}: incomplete or expired CAN feedback: {problem}')
        if sample.fault:
            raise TeachingFault(f'arm_{index}: {sample.fault}')
        if len(sample.position) != 7:
            raise TeachingFault(f'arm_{index}: invalid joint/gripper position')
    return tuple(samples[peer].position for peer in peers)


def references(samples: tuple[ArmSample, ...], config: TeachingConfig,
               now: float, require_enabled: bool = False) -> tuple[tuple[float, ...], ...]:
    """Validate fresh feedback and enable flags before returning reciprocal references."""
    refs = measured_references(samples, config, now)
    for index, sample in enumerate(samples, 1):
        if require_enabled and (len(sample.enabled) != 7 or not all(sample.enabled[:6])):
            raise TeachingFault(f'arm_{index}: all six motors must already be enabled')
    return refs


class BilateralCoupling:
    """Pause on unavailable feedback without altering the last sent targets."""

    def __init__(self, config: TeachingConfig) -> None:
        self.config = config
        self.active = False
        self.reason = 'Not started'
        self.targets: tuple[tuple[float, ...], ...] = ()
        self.velocities: tuple[tuple[float, ...], ...] = ((0.0,) * 6,) * 4
        self._last_tick = 0.0
        self.engage_goals: tuple[tuple[float, ...], ...] = ()
        self.engaged = False
        self.engage_error_deg: float | None = None

    def start(self, samples: tuple[ArmSample, ...], now: float) -> None:
        if self.active:
            raise TeachingFault('Coupling is already started')
        self.check_start(samples, now)
        self.targets = tuple(sample.position for sample in samples)
        pair_goals = (
            (samples[0].position[:6], samples[1].position[:6])
            if self.config.mode == 'master_slave' else
            tuple(
                tuple(
                    0.5 * (a + b)
                    for a, b in zip(samples[i].position[:6], samples[i + 2].position[:6])
                )
                for i in MASTER_INDICES
            )
        )
        self.engage_goals = pair_goals * 2
        self.velocities = ((0.0,) * 6,) * 4
        self._last_tick = now
        self.engaged = False
        self.engage_error_deg = None
        self.active = True
        self.reason = 'Waiting for all four MOVE_M mode acknowledgements'

    def check_start(self, samples: tuple[ArmSample, ...], now: float,
                    require_enabled: bool = True) -> None:
        references(samples, self.config, now, require_enabled=require_enabled)
        self.config.require_gains()

    def stop(self, reason: str) -> None:
        self.active = False
        self.reason = reason

    def pause(self, reason: str, now: float) -> None:
        self._last_tick = now
        self.reason = reason

    def step(self, samples: tuple[ArmSample, ...], now: float,
             modes_ready: bool) -> tuple[tuple[tuple[float, ...], ...], tuple[float, ...]]:
        if not self.active:
            raise TeachingFault('Coupling is stopped; explicit start is required')
        try:
            dt = now - self._last_tick
            if not 0 < dt <= self.config.feedback_timeout:
                raise TeachingFault('Control-loop deadline missed')
            refs = references(samples, self.config, now, require_enabled=True)
            if modes_ready:
                if not self.engaged:
                    goals = self.engage_goals
                    max_step = math.radians(self.config.engage_speed_deg_s) * dt
                    self.targets = tuple(
                        tuple(
                            old_q + max(-max_step, min(max_step, goal_q - old_q))
                            for old_q, goal_q in zip(old[:6], goal)
                        ) + (old[6],)
                        for old, goal in zip(self.targets, goals)
                    )
                    self.velocities = ((0.0,) * 6,) * 4
                    self.engage_error_deg = math.degrees(max(
                        abs(q - goal_q)
                        for sample, goal in zip(samples, goals)
                        for q, goal_q in zip(sample.position[:6], goal)
                    ))
                    self.engaged = all(
                        abs(q - goal_q) <= 1e-9
                        for target, goal in zip(self.targets, goals)
                        for q, goal_q in zip(target[:6], goal)
                    )
                else:
                    self.targets = refs
                    self.engage_error_deg = 0.0
                    peers = peers_for_mode(self.config.mode)
                    self.velocities = tuple(
                        (0.0,) * 6
                        if self.config.mode == 'master_slave' and arm in MASTER_INDICES
                        else tuple(
                            max(-45.0, min(45.0, float(v))) for v in samples[peer].velocity[:6]
                        )
                        for arm, peer in enumerate(peers)
                    )
                self.reason = 'Coupling active' if self.engaged else (
                    f'Engaging pair alignment; max joint error {self.engage_error_deg:.1f} deg')
            else:
                self.velocities = ((0.0,) * 6,) * 4
            self._last_tick = now
            return self.targets, self.config.kp
        except TeachingFault as exc:
            self.pause(str(exc), now)
            raise
