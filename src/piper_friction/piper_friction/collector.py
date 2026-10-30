"""ROS 2 client node for executing multi-speed forward/reverse MIT sweeps and logging data."""

from dataclasses import dataclass
import json
import math
import time
from pathlib import Path
from typing import Sequence

from piper_x_driver.gravity import ArmGravityModel, gravity_torques, load_gravity_model, max_critical_kd
from piper_x_interfaces.msg import ArmStatus, MitCommand
from piper_x_interfaces.srv import ConfigureArm, Enable, SetMode
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .identification import SweepSample

JOINT_NAMES = tuple(f'joint{i}' for i in range(1, 7)) + ('gripper',)
DEFAULT_SWEEP_SPEEDS = (0.03, 0.06, 0.12, 0.25, 0.5, 1.0, 1.5)
DEFAULT_SWEEP_KP = (30.0, 30.0, 30.0, 8.0, 6.0, 6.0)
SAFE_ANCHOR_POSE = (0.0, 0.90, -1.50, 0.0, 0.0, 0.0)


def resolve_safe_anchor(initial_position: Sequence[float], joint: int) -> tuple[float, ...]:
    """Return safe raised 6-joint anchor pose, keeping J1 at current yaw when testing J2..J6."""
    anchor = list(SAFE_ANCHOR_POSE)
    if len(initial_position) >= 6 and joint != 1:
        anchor[0] = float(initial_position[0])
    return tuple(anchor)


@dataclass(frozen=True)
class SweepConfig:
    """Configuration for a single-joint multi-speed forward/reverse sweep."""

    namespace: str = '/arm_1'
    joint: int = 1
    q_min: float = -0.5
    q_max: float = 0.5
    speeds: tuple[float, ...] = DEFAULT_SWEEP_SPEEDS
    repeats: int = 1
    kp: tuple[float, ...] = DEFAULT_SWEEP_KP
    kd: tuple[float, ...] | None = None
    approach_speed: float = 0.4
    settle_time: float = 0.4
    rate_hz: float = 100.0
    use_safe_anchor: bool = True

    def __post_init__(self) -> None:
        if not 1 <= self.joint <= 6:
            raise ValueError(f'joint must be in 1..6, got {self.joint}')
        if not math.isfinite(self.q_min) or not math.isfinite(self.q_max) or self.q_max <= self.q_min:
            raise ValueError('q_min and q_max must be finite with q_max > q_min')
        if not self.speeds or any(not math.isfinite(s) or s <= 0 for s in self.speeds):
            raise ValueError('speeds must be a non-empty sequence of positive finite values')
        if self.repeats < 1:
            raise ValueError('repeats must be >= 1')
        if len(self.kp) != 6 or any(not math.isfinite(v) or v <= 0 for v in self.kp):
            raise ValueError('kp requires six positive finite values')
        if self.kd is None:
            object.__setattr__(self, 'kd', max_critical_kd(self.kp))
        elif len(self.kd) != 6 or any(not math.isfinite(v) or v < 0 for v in self.kd):
            raise ValueError('kd requires six non-negative finite values')
        if self.approach_speed <= 0 or not math.isfinite(self.approach_speed):
            raise ValueError('approach_speed must be finite and positive')
        if self.settle_time < 0 or not math.isfinite(self.settle_time):
            raise ValueError('settle_time must be finite and non-negative')
        if self.rate_hz <= 0 or not math.isfinite(self.rate_hz):
            raise ValueError('rate_hz must be finite and positive')


@dataclass(frozen=True)
class _TrajectorySegment:
    kind: str
    q_start: float
    q_end: float
    duration: float
    dq_des: float


def build_sweep_schedule(
    q_initial: float,
    config: SweepConfig,
    *,
    anchor_travel: float = 0.0,
    q_return: float | None = None,
) -> list[_TrajectorySegment]:
    """Build the sequence of approach, settle, forward/reverse sweep, and return segments."""
    segments: list[_TrajectorySegment] = []
    approach_dist = max(abs(config.q_min - q_initial), float(anchor_travel))
    if approach_dist > 1e-6:
        segments.append(_TrajectorySegment(
            kind='approach',
            q_start=q_initial,
            q_end=config.q_min,
            duration=approach_dist / config.approach_speed,
            dq_des=0.0,
        ))
    span = config.q_max - config.q_min
    for speed in config.speeds:
        for _ in range(config.repeats):
            if config.settle_time > 0:
                segments.append(_TrajectorySegment(
                    kind='settle',
                    q_start=config.q_min,
                    q_end=config.q_min,
                    duration=config.settle_time,
                    dq_des=0.0,
                ))
            segments.append(_TrajectorySegment(
                kind='sweep',
                q_start=config.q_min,
                q_end=config.q_max,
                duration=span / speed,
                dq_des=float(speed),
            ))
            if config.settle_time > 0:
                segments.append(_TrajectorySegment(
                    kind='settle',
                    q_start=config.q_max,
                    q_end=config.q_max,
                    duration=config.settle_time,
                    dq_des=0.0,
                ))
            segments.append(_TrajectorySegment(
                kind='sweep',
                q_start=config.q_max,
                q_end=config.q_min,
                duration=span / speed,
                dq_des=-float(speed),
            ))
    target_return = q_initial if q_return is None else float(q_return)
    return_dist = abs(target_return - config.q_min)
    if return_dist > 1e-6:
        segments.append(_TrajectorySegment(
            kind='return',
            q_start=config.q_min,
            q_end=target_return,
            duration=return_dist / config.approach_speed,
            dq_des=0.0,
        ))
    return segments


def sample_velocity_confidence(
    vel_window: Sequence[tuple[float, float]],
    dq_des: float,
    vel_tol: float = 0.08,
    max_vel_tol: float = 0.25,
) -> float:
    """Return confidence in [0.0, 1.0]: 1.0 in steady zone, cosine-tapered for small accel, 0.0 for large accel."""
    if is_velocity_stabilized(vel_window, dq_des, vel_tol=vel_tol):
        return 1.0
    speed = abs(dq_des)
    if speed <= 1e-6 or len(vel_window) < 3:
        return 0.0
    sign = 1.0 if dq_des > 0 else -1.0
    recent = vel_window[-min(5, len(vel_window)):]
    v_mean = sum(sign * v for _, v in recent) / len(recent)
    dt = recent[-1][0] - recent[0][0]
    if dt <= 1e-5:
        return 0.0
    ddq = abs(sign * recent[-1][1] - sign * recent[0][1]) / dt
    alpha_steady = max(0.4, 0.50 * speed)
    alpha_max = max(1.2, 1.50 * speed)
    vel_err = abs(v_mean - speed)
    # Hard reject if acceleration or velocity deviation exceeds the small-expansion bounds
    if ddq >= alpha_max or vel_err >= max_vel_tol * speed:
        return 0.0
    u_acc = max(0.0, ddq - alpha_steady) / max(1e-6, alpha_max - alpha_steady)
    u_vel = max(0.0, vel_err - vel_tol * speed) / max(1e-6, (max_vel_tol - vel_tol) * speed)
    u = math.hypot(u_acc, u_vel)
    if u >= 1.0:
        return 0.0
    return float(0.5 * (1.0 + math.cos(math.pi * u)))


def is_velocity_stabilized(
    vel_window: Sequence[tuple[float, float]],
    dq_des: float,
    vel_tol: float = 0.08,
) -> bool:
    """Return True once measured joint velocity has converged to dq_des with low acceleration."""
    speed = abs(dq_des)
    if speed <= 1e-6 or not vel_window:
        return False
    sign = 1.0 if dq_des > 0 else -1.0
    v_cur = sign * vel_window[-1][1]
    if abs(v_cur - speed) <= 1e-5:
        return True
    if speed <= 0.15:
        elapsed = vel_window[-1][0] - vel_window[0][0]
        return elapsed >= 0.05 and v_cur >= 0.0
    if len(vel_window) < 3:
        return False
    if all(abs(v) <= 1e-6 for _, v in vel_window):
        return True
    recent = vel_window[-min(5, len(vel_window)):]
    v_mean = sum(sign * v for _, v in recent) / len(recent)
    dt = recent[-1][0] - recent[0][0]
    if dt <= 1e-5:
        return False
    ddq = abs(sign * recent[-1][1] - sign * recent[0][1]) / dt
    in_band = (1.0 - vel_tol) * speed <= v_mean <= (1.0 + vel_tol) * speed
    low_acc = ddq <= max(0.5, 0.65 * speed)
    return in_band and low_acc


class SweepCollectorNode(Node):
    """ROS 2 client node that drives one joint through multi-speed sweeps and records samples."""

    def __init__(self, config: SweepConfig, **kwargs):
        super().__init__('piper_friction_collector', **kwargs)
        self.config = config
        prefix = config.namespace.rstrip('/')
        self.phase = 'waiting_feedback'
        self.error = ''
        self.completed = False

        self.latest_position: tuple[float, ...] | None = None
        self.latest_velocity: tuple[float, ...] = (0.0,) * 6
        self.latest_torque: tuple[float, ...] = (0.0,) * 6
        self.latest_status: ArmStatus | None = None
        self.effector_kwargs: dict[str, object] = {}
        self.arm_end_effector: str = 'teaching_handle'
        self.arm_gravity_scale: float = 1.0
        self.applied_gravity_torques: tuple[float, ...] | None = None
        self.gravity_model: ArmGravityModel | None = None
        self._loaded_gravity_path: str = ''

        self.initial_position: tuple[float, ...] | None = None
        self.anchor_position: tuple[float, ...] | None = None
        self.schedule: list[_TrajectorySegment] = []
        self.segment_index = 0
        self.segment_started = 0.0
        self.sweep_started = 0.0
        self.current_q_des = 0.0
        self.current_dq_des = 0.0
        self._pending_future = None
        self._segment_velocities: list[tuple[float, float]] = []
        self.samples: list[SweepSample] = []

        self.create_subscription(JointState, f'{prefix}/joint_states_feedback', self._on_joints, 10)
        self.create_subscription(ArmStatus, f'{prefix}/arm_status', self._on_status, 10)
        self.create_subscription(String, f'{prefix}/control_state', self._on_control_state, 10)
        self.mit_pub = self.create_publisher(MitCommand, f'{prefix}/command/mit', 1)
        self.config_client = self.create_client(ConfigureArm, f'{prefix}/configure_arm')
        self.enable_client = self.create_client(Enable, f'{prefix}/enable_srv')
        self.mode_client = self.create_client(SetMode, f'{prefix}/set_mode')
        self.create_timer(1.0 / config.rate_hz, self._tick)

    def _on_joints(self, message: JointState) -> None:
        names = list(message.name)
        if len(names) != 7 or set(names) != set(JOINT_NAMES) or len(message.position) != 7:
            return
        order = [names.index(name) for name in JOINT_NAMES]
        self.latest_position = tuple(float(message.position[i]) for i in order)
        if len(message.velocity) >= 6:
            self.latest_velocity = tuple(float(message.velocity[i]) for i in order[:6])
        if len(message.effort) >= 6:
            self.latest_torque = tuple(float(message.effort[i]) for i in order[:6])

        if self.phase == 'running' and self.anchor_position is not None:
            now = time.monotonic()
            j_idx = self.config.joint - 1
            if abs(self.current_dq_des) > 1e-6:
                self._segment_velocities.append((now, self.latest_velocity[j_idx]))
                conf = sample_velocity_confidence(self._segment_velocities, self.current_dq_des)
                if conf > 0.0:
                    if (self.applied_gravity_torques is not None
                            and len(self.applied_gravity_torques) >= 6):
                        g_val = float(self.applied_gravity_torques[j_idx])
                    else:
                        g_torques = (
                            gravity_torques(self.latest_position[:6], **self.effector_kwargs)
                            if self.gravity_model is None
                            else self.gravity_model.torques(self.latest_position[:6], **self.effector_kwargs)
                        )
                        g_val = float(self.arm_gravity_scale * g_torques[j_idx])
                    self.samples.append(SweepSample(
                        t=now - self.sweep_started,
                        joint=self.config.joint,
                        q=self.latest_position[j_idx],
                        dq=self.latest_velocity[j_idx],
                        tau_meas=self.latest_torque[j_idx],
                        q_des=self.current_q_des,
                        dq_des=self.current_dq_des,
                        tau_gravity=g_val,
                        weight=conf,
                    ))

    def _on_status(self, message: ArmStatus) -> None:
        self.latest_status = message

    def _on_control_state(self, message: String) -> None:
        try:
            state = json.loads(message.data)
            cfg = state.get('configuration', {})
            if isinstance(cfg.get('end_effector'), str) and cfg['end_effector']:
                self.arm_end_effector = cfg['end_effector']
            if isinstance(cfg.get('gravity_scale'), (int, float)):
                self.arm_gravity_scale = float(cfg['gravity_scale'])
            if isinstance(state.get('gravity_scale_applied'), (int, float)):
                self.arm_gravity_scale = float(state['gravity_scale_applied'])
            g_vec = state.get('gravity_torque')
            if isinstance(g_vec, list) and len(g_vec) >= 6:
                self.applied_gravity_torques = tuple(float(x) for x in g_vec[:6])
            if isinstance(cfg.get('gravity_config'), str):
                g_cfg = cfg['gravity_config'].strip()
                if g_cfg and g_cfg != self._loaded_gravity_path and Path(g_cfg).is_file():
                    self.gravity_model = load_gravity_model(g_cfg)
                    self._loaded_gravity_path = g_cfg
            eff = cfg.get('effector', {})
            if 'mass_kg' in eff and 'com_m' in eff:
                self.effector_kwargs = {
                    'mass_kg': float(eff['mass_kg']),
                    'com_m': tuple(float(x) for x in eff['com_m']),
                }
        except (ValueError, TypeError, KeyError):
            pass

    def _publish_mit(self, q_joint: float, dq_joint: float, anchor_blend: float = 1.0) -> None:
        assert self.anchor_position is not None
        assert self.config.kd is not None
        if self.initial_position is not None and anchor_blend < 1.0:
            pos = [
                float(self.initial_position[i] + anchor_blend * (self.anchor_position[i] - self.initial_position[i]))
                for i in range(6)
            ]
        else:
            pos = list(self.anchor_position[:6])
        vel = [0.0] * 6
        j_idx = self.config.joint - 1
        pos[j_idx] = float(q_joint)
        vel[j_idx] = float(dq_joint)
        msg = MitCommand()
        msg.position = pos
        msg.velocity = vel
        msg.kp = list(self.config.kp)
        msg.kd = list(self.config.kd)
        msg.torque = [0.0] * 6
        self.mit_pub.publish(msg)

    def _fail(self, reason: str) -> None:
        self.error = reason
        self.phase = 'failed'
        self.get_logger().error(reason)

    def _tick(self) -> None:
        if self.phase in ('completed', 'failed'):
            return
        if self.latest_position is None or self.latest_status is None:
            return
        if self.latest_status.err_code or self.latest_status.arm_status not in (0, 11, 12, 13):
            self._fail(
                f'Arm fault reported: arm_status={self.latest_status.arm_status}, '
                f'err_code={self.latest_status.err_code}'
            )
            return

        if self.phase == 'waiting_feedback':
            self.initial_position = self.latest_position[:6]
            self.anchor_position = (
                resolve_safe_anchor(self.initial_position, self.config.joint)
                if self.config.use_safe_anchor
                else self.initial_position
            )
            self.current_q_des = self.initial_position[self.config.joint - 1]
            self.current_dq_des = 0.0
            # 进入摩擦力标定程序时，先将驱动的 friction_scale 设为 0.0，防止已标定的摩擦力影响采样
            if self.config_client.service_is_ready():
                self._pending_future = self.config_client.call_async(
                    ConfigureArm.Request(
                        end_effector=self.arm_end_effector,
                        gravity_scale=self.arm_gravity_scale,
                        friction_scale=0.0,
                    )
                )
                self.phase = 'zeroing_friction'
                return
            self._check_enable_or_mit()
            return

        if self.phase == 'zeroing_friction':
            assert self._pending_future is not None
            if not self._pending_future.done():
                return
            res = self._pending_future.result()
            if res is None or not res.success:
                self._fail(
                    f'Failed to zero friction_scale: {getattr(res, "message", "no response")}'
                )
                return
            self._check_enable_or_mit()
            return

        if self.phase == 'enabling':
            assert self._pending_future is not None
            if not self._pending_future.done():
                return
            res = self._pending_future.result()
            if res is None or not res.enable_response:
                self._fail(f'Failed to enable arm: {getattr(res, "message", "no response")}')
                return
            self._enter_mit_phase()
            return

        if self.phase == 'setting_mode':
            assert self.anchor_position is not None
            self._publish_mit(self.current_q_des, 0.0, anchor_blend=0.0)
            assert self._pending_future is not None
            if not self._pending_future.done():
                return
            res = self._pending_future.result()
            if res is None or not res.success:
                self._fail(f'Failed to enter MIT mode: {getattr(res, "message", "no response")}')
                return
            self._start_schedule()
            return

        if self.phase == 'running':
            now = time.monotonic()
            while self.segment_index < len(self.schedule):
                seg = self.schedule[self.segment_index]
                elapsed = now - self.segment_started
                if elapsed <= seg.duration:
                    ratio = elapsed / seg.duration if seg.duration > 0 else 1.0
                    interp = (
                        0.5 * (1.0 - math.cos(math.pi * ratio))
                        if seg.kind in ('approach', 'return')
                        else ratio
                    )
                    self.current_q_des = seg.q_start + interp * (seg.q_end - seg.q_start)
                    self.current_dq_des = seg.dq_des
                    blend = interp if (self.segment_index == 0 and seg.kind == 'approach') else 1.0
                    self._publish_mit(self.current_q_des, self.current_dq_des, anchor_blend=blend)
                    return
                self.segment_index += 1
                self.segment_started = now
                self._segment_velocities = []

            assert self.anchor_position is not None
            self.current_q_des = self.anchor_position[self.config.joint - 1]
            self.current_dq_des = 0.0
            self._segment_velocities = []
            self._publish_mit(self.current_q_des, 0.0)
            self.phase = 'completed'
            self.completed = True

    def _check_enable_or_mit(self) -> None:
        assert self.latest_status is not None
        if not all(self.latest_status.motor_enabled):
            if not self.enable_client.service_is_ready():
                return
            self._pending_future = self.enable_client.call_async(
                Enable.Request(enable_request=True)
            )
            self.phase = 'enabling'
            return
        self._enter_mit_phase()

    def _enter_mit_phase(self) -> None:
        assert self.anchor_position is not None
        assert self.latest_status is not None
        self._publish_mit(self.current_q_des, 0.0, anchor_blend=0.0)
        if self.latest_status.ctrl_mode == 1 and self.latest_status.mode_feedback == 6:
            self._start_schedule()
            return
        if not self.mode_client.service_is_ready():
            return
        self._pending_future = self.mode_client.call_async(SetMode.Request(mode='mit'))
        self.phase = 'setting_mode'

    def _start_schedule(self) -> None:
        assert self.anchor_position is not None
        init_p = self.initial_position or self.anchor_position
        j_idx = self.config.joint - 1
        q_init = init_p[j_idx]
        q_ret = self.anchor_position[j_idx]
        anchor_travel = max(
            (abs(self.anchor_position[i] - init_p[i]) for i in range(6) if i != j_idx),
            default=0.0,
        )
        self.schedule = build_sweep_schedule(
            q_init, self.config, anchor_travel=anchor_travel, q_return=q_ret,
        )
        self.segment_index = 0
        self.sweep_started = time.monotonic()
        self.segment_started = self.sweep_started
        self._segment_velocities = []
        self.phase = 'running'
