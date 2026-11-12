"""ROS 2 client node for executing multi-speed forward/reverse MIT sweeps and logging data."""

from dataclasses import dataclass
import json
import math
import time
from pathlib import Path
from typing import Any, Sequence

from piper_x_driver.gravity import ArmGravityModel, gravity_torques, load_gravity_model, max_critical_kd
from piper_x_interfaces.msg import ArmStatus, GripperCommand, MitCommand
from piper_x_interfaces.srv import ConfigureArm, Enable, SetMode
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .identification import (
    SweepSample,
    V0_BURST_GROUPS,
    V0_CANDIDATES,
    V0_NOISE_DURATION_S,
    V0_OBS_START_S,
    V0_STEP_DURATION_S,
    compute_resting_v_noise,
    is_v0_step_oscillating,
    v0_closed_loop_step_torque,
    v0_excitation_amplitude,
)
from .model import ArmFrictionModel

JOINT_NAMES = tuple(f'joint{i}' for i in range(1, 7)) + ('gripper',)
DEFAULT_SWEEP_SPEEDS = (0.03, 0.06, 0.12, 0.25, 0.5, 1.0, 1.5)
DEFAULT_GRIPPER_SPEEDS = (0.005, 0.01, 0.02, 0.04, 0.08)
DEFAULT_SWEEP_KP = (30.0, 30.0, 30.0, 8.0, 6.0, 6.0)
DEFAULT_GRIPPER_EFFORT = 2.0
SAFE_ANCHOR_POSE = (0.0, 0.90, -1.50, 0.0, 0.0, 0.0)


def resolve_safe_anchor(initial_position: Sequence[float], joint: int) -> tuple[float, ...]:
    """Return safe raised 6-joint anchor pose, keeping J1 at current yaw when testing J2..J7."""
    anchor = list(SAFE_ANCHOR_POSE)
    if len(initial_position) >= 6 and joint != 1:
        anchor[0] = float(initial_position[0])
    return tuple(anchor)


@dataclass(frozen=True)
class SweepConfig:
    """Configuration for a single-joint (1..6) or gripper (7) multi-speed sweep."""

    namespace: str = '/arm_1'
    joint: int = 1
    q_min: float = -0.5
    q_max: float = 0.5
    speeds: tuple[float, ...] = DEFAULT_SWEEP_SPEEDS
    repeats: int = 1
    kp: tuple[float, ...] = DEFAULT_SWEEP_KP
    kd: tuple[float, ...] | None = None
    gripper_effort: float = DEFAULT_GRIPPER_EFFORT
    approach_speed: float = 0.4
    settle_time: float = 0.4
    rate_hz: float = 100.0
    use_safe_anchor: bool = True

    def __post_init__(self) -> None:
        if not 1 <= self.joint <= 7:
            raise ValueError(f'joint must be in 1..7 (7 for gripper), got {self.joint}')
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
        if not math.isfinite(self.gripper_effort) or not 0 < self.gripper_effort <= 5.0:
            raise ValueError('gripper_effort must be in (0, 5.0]')
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


def _recent_velocity_window(
    vel_window: Sequence[tuple[float, float]],
    min_span: float = 0.08,
    min_samples: int = 5,
) -> Sequence[tuple[float, float]]:
    """Select trailing velocity samples spanning at least min_span s (and >= min_samples)."""
    n = len(vel_window)
    if n <= min_samples:
        return vel_window
    t_end = vel_window[-1][0]
    idx = n - min_samples
    while idx > 0 and (t_end - vel_window[idx][0]) < min_span:
        idx -= 1
    return vel_window[idx:]


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
    recent = _recent_velocity_window(vel_window)
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
    recent = _recent_velocity_window(vel_window)
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
        self.latest_gripper_velocity: float = 0.0
        self.latest_gripper_force: float = 0.0
        self._has_ctrl_grip_vel: bool = False
        self._prev_grip_sample: tuple[float, float] | None = None
        self.latest_status: ArmStatus | None = None
        self.effector_kwargs: dict[str, Any] = {}
        self.arm_end_effector: str = 'teaching_handle'
        self.arm_gravity_scale: float = 1.0
        self.arm_friction_scale: float = 0.85
        self._saved_friction_scale: float | None = None
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

        realtime_sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            JointState, f'{prefix}/joint_states_feedback', self._on_joints, realtime_sub_qos)
        self.create_subscription(
            ArmStatus, f'{prefix}/arm_status', self._on_status, realtime_sub_qos)
        self.create_subscription(
            String, f'{prefix}/control_state', self._on_control_state, realtime_sub_qos)
        self.mit_pub = self.create_publisher(MitCommand, f'{prefix}/command/mit', 10)
        self.gripper_pub = self.create_publisher(GripperCommand, f'{prefix}/command/gripper', 10)
        self.config_client = self.create_client(ConfigureArm, f'{prefix}/configure_arm')
        self.enable_client = self.create_client(Enable, f'{prefix}/enable_srv')
        self.mode_client = self.create_client(SetMode, f'{prefix}/set_mode')
        self.create_timer(1.0 / config.rate_hz, self._tick)

    def _on_joints(self, message: JointState) -> None:
        names = list(message.name)
        if len(names) != 7 or set(names) != set(JOINT_NAMES) or len(message.position) != 7:
            return
        order = [names.index(name) for name in JOINT_NAMES]
        now = time.monotonic()
        self.latest_position = tuple(float(message.position[i]) for i in order)
        if len(message.velocity) >= 6:
            self.latest_velocity = tuple(float(message.velocity[i]) for i in order[:6])
        if len(message.velocity) >= 7:
            self.latest_gripper_velocity = float(message.velocity[order[6]])
        elif not self._has_ctrl_grip_vel and self._prev_grip_sample is not None:
            dt = now - self._prev_grip_sample[0]
            if 1e-4 <= dt <= 0.5:
                self.latest_gripper_velocity = (
                    self.latest_position[6] - self._prev_grip_sample[1]
                ) / dt
        self._prev_grip_sample = (now, self.latest_position[6])
        if len(message.effort) >= 6:
            self.latest_torque = tuple(float(message.effort[i]) for i in order[:6])
        if len(message.effort) >= 7:
            self.latest_gripper_force = float(message.effort[order[6]])

        if self.phase == 'running' and self.anchor_position is not None:
            j_idx = self.config.joint - 1
            if abs(self.current_dq_des) > 1e-6:
                if self.config.joint == 7:
                    dq_meas = self.latest_gripper_velocity
                    tau_meas = self.latest_gripper_force
                else:
                    dq_meas = self.latest_velocity[j_idx]
                    tau_meas = self.latest_torque[j_idx]
                self._segment_velocities.append((now, dq_meas))
                conf = sample_velocity_confidence(self._segment_velocities, self.current_dq_des)
                if conf > 0.0:
                    if self.config.joint == 7:
                        g_val = 0.0
                    elif (self.applied_gravity_torques is not None
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
                        dq=dq_meas,
                        tau_meas=tau_meas,
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
            if isinstance(cfg.get('friction_scale'), (int, float)):
                self.arm_friction_scale = float(cfg['friction_scale'])
            if isinstance(state.get('gravity_scale_applied'), (int, float)):
                self.arm_gravity_scale = float(state['gravity_scale_applied'])
            g_vec = state.get('gravity_torque')
            if isinstance(g_vec, list) and len(g_vec) >= 6:
                self.applied_gravity_torques = tuple(float(x) for x in g_vec[:6])
            if isinstance(state.get('gripper_velocity'), (int, float)):
                self.latest_gripper_velocity = float(state['gripper_velocity'])
                self._has_ctrl_grip_vel = True
            if isinstance(state.get('gripper_force'), (int, float)):
                self.latest_gripper_force = float(state['gripper_force'])
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
        if self.config.joint == 7:
            self.gripper_pub.publish(GripperCommand(
                width=max(0.0, min(0.1, float(q_joint))),
                effort=float(self.config.gripper_effort),
            ))
            return
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

    def _restore_friction_scale(self):
        if (self._saved_friction_scale is not None
                and self.config_client.service_is_ready()):
            fut = self.config_client.call_async(
                ConfigureArm.Request(
                    end_effector=self.arm_end_effector,
                    gravity_scale=self.arm_gravity_scale,
                    friction_scale=-4.0 - self._saved_friction_scale,
                )
            )
            self._saved_friction_scale = None
            return fut
        return None

    def _fail(self, reason: str) -> None:
        self._restore_friction_scale()
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
            self.initial_position = self.latest_position[:7]
            base_anchor = (
                resolve_safe_anchor(self.initial_position[:6], self.config.joint)
                if self.config.use_safe_anchor
                else self.initial_position[:6]
            )
            self.anchor_position = base_anchor + (self.initial_position[6],)
            self.current_q_des = self.initial_position[self.config.joint - 1]
            self.current_dq_des = 0.0
            # 进入摩擦力标定程序时，仅在驱动内存中临时将 friction_scale 设为 0.0（不改写 arms.yaml）
            if self.config_client.service_is_ready():
                self._saved_friction_scale = self.arm_friction_scale
                self._pending_future = self.config_client.call_async(
                    ConfigureArm.Request(
                        end_effector=self.arm_end_effector,
                        gravity_scale=self.arm_gravity_scale,
                        friction_scale=-2.0,
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
            if self.config.joint == 7:
                self.gripper_pub.publish(GripperCommand(width=float(self.current_q_des), effort=0.0))
            restore_fut = self._restore_friction_scale()
            if restore_fut is not None:
                self._pending_future = restore_fut
                self.phase = 'restoring_friction'
                return
            self.phase = 'completed'
            self.completed = True
            return

        if self.phase == 'restoring_friction':
            assert self.anchor_position is not None
            self._publish_mit(self.current_q_des, 0.0)
            if self._pending_future is not None and not self._pending_future.done():
                return
            self.phase = 'completed'
            self.completed = True

    def _check_enable_or_mit(self) -> None:
        assert self.latest_status is not None
        if self.config.joint == 7:
            self._start_schedule()
            return
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
        anchor_travel = (
            0.0
            if self.config.joint == 7
            else max(
                (abs(self.anchor_position[i] - init_p[i]) for i in range(6) if i != j_idx),
                default=0.0,
            )
        )
        self.schedule = build_sweep_schedule(
            q_init, self.config, anchor_travel=anchor_travel, q_return=q_ret,
        )
        self.segment_index = 0
        self.sweep_started = time.monotonic()
        self.segment_started = self.sweep_started
        self._segment_velocities = []
        self.phase = 'running'


@dataclass(frozen=True)
class V0ProbeConfig:
    """Configuration for closed-loop breakaway ping + critical v0 search."""

    namespace: str = '/arm_1'
    joints: tuple[int, ...] = (1, 2, 3, 4, 5, 6)
    noise_duration: float = V0_NOISE_DURATION_S
    step_duration: float = V0_STEP_DURATION_S
    candidates: tuple[float, ...] = V0_CANDIDATES
    max_drift_rad: float = 0.045
    rate_hz: float = 200.0

    def __post_init__(self) -> None:
        if not self.joints or any(j not in range(1, 7) for j in self.joints):
            raise ValueError(f'joints must be a non-empty subset of 1..6, got {self.joints}')


class V0CollectorNode(Node):
    """ROS 2 client node that finds each joint's critical v0 via closed-loop ping + ringdown descent."""

    def __init__(
        self,
        config: V0ProbeConfig,
        arm_model: ArmFrictionModel,
        static_frictions: Sequence[float] = (0.0,) * 6,
        friction_scale: float = 0.88,
        **kwargs,
    ):
        super().__init__('piper_friction_v0_collector', **kwargs)
        self.config = config
        self.arm_model = arm_model
        self.target_friction_scale = float(friction_scale)
        self.amplitudes = {
            j: v0_excitation_amplitude(
                arm_model.joint(j),
                static_frictions[j - 1] if j - 1 < len(static_frictions) else 0.0,
            )
            for j in config.joints
            if j in arm_model.joints
        }
        self.burst_groups = tuple(
            tuple(j for j in grp if j in self.amplitudes)
            for grp in V0_BURST_GROUPS
        )
        self.burst_groups = tuple(grp for grp in self.burst_groups if grp)

        prefix = config.namespace.rstrip('/')
        self.phase = 'waiting_feedback'
        self.error = ''
        self.completed = False
        self.latest_position: tuple[float, ...] | None = None
        self.latest_velocity: tuple[float, ...] = (0.0,) * 6
        self.latest_status: ArmStatus | None = None
        self.arm_end_effector = 'teaching_handle'
        self.arm_gravity_scale = 1.0
        self.arm_friction_scale = float(friction_scale)
        self._saved_friction_scale: float | None = None
        self.q0: tuple[float, ...] | None = None
        self._pending_future = None
        self.phase_started = 0.0
        self.burst_index = 0
        self.cand_index = 0
        self.current_torques: tuple[float, ...] = (0.0,) * 6
        self.noise_samples: list[tuple[float, tuple[float, ...]]] = []
        self.v_noise_by_joint: dict[int, float] = {j: 0.001 for j in self.amplitudes}
        self.triggered_joints: set[int] = set()
        self._step_qs: dict[int, list[float]] = {j: [] for j in self.amplitudes}
        self._step_obs_vels: dict[int, list[float]] = {j: [] for j in self.amplitudes}
        self.search_records: dict[int, list[tuple[float, list[float], list[float]]]] = {
            j: [] for j in self.amplitudes
        }

        realtime_sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            JointState, f'{prefix}/joint_states_feedback', self._on_joints, realtime_sub_qos,
        )
        self.create_subscription(
            ArmStatus, f'{prefix}/arm_status', self._on_status, realtime_sub_qos,
        )
        self.create_subscription(
            String, f'{prefix}/control_state', self._on_control_state, realtime_sub_qos,
        )
        self.mit_pub = self.create_publisher(MitCommand, f'{prefix}/command/mit', 10)
        self.config_client = self.create_client(ConfigureArm, f'{prefix}/configure_arm')
        self.enable_client = self.create_client(Enable, f'{prefix}/enable_srv')
        self.mode_client = self.create_client(SetMode, f'{prefix}/set_mode')
        self.create_timer(1.0 / config.rate_hz, self._tick)

    def _on_joints(self, message: JointState) -> None:
        names = tuple(message.name)
        if len(message.position) != 7:
            return
        if names == JOINT_NAMES:
            order = range(7)
        else:
            if len(names) != 7 or set(names) != set(JOINT_NAMES):
                return
            order = [names.index(name) for name in JOINT_NAMES]
        now = time.monotonic()
        self.latest_position = tuple(float(message.position[i]) for i in order)
        if len(message.velocity) >= 6:
            self.latest_velocity = tuple(float(message.velocity[i]) for i in order[:6])

        if self.phase == 'measuring_noise':
            elapsed = now - self.phase_started
            if elapsed >= 0.10:
                self.noise_samples.append((elapsed - 0.10, self.latest_velocity))
        elif self.phase == 'running_burst' and self.burst_index < len(self.burst_groups):
            elapsed = now - self.phase_started
            if elapsed <= self.config.step_duration and self.cand_index < len(self.config.candidates):
                cand = self.config.candidates[self.cand_index]
                tau = [0.0] * 6
                for j in self.burst_groups[self.burst_index]:
                    if j in self.triggered_joints:
                        continue
                    idx = j - 1
                    q_j = self.latest_position[idx]
                    dq_j = self.latest_velocity[idx]
                    self._step_qs[j].append(q_j)
                    if elapsed >= V0_OBS_START_S:
                        self._step_obs_vels[j].append(dq_j)
                    tau[idx] = v0_closed_loop_step_torque(
                        elapsed, q_j, dq_j, self.arm_model.joint(j),
                        cand, self.amplitudes[j], self.target_friction_scale,
                    )
                self._publish_zero_gain_mit(tau)

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
            if isinstance(cfg.get('friction_scale'), (int, float)):
                self.arm_friction_scale = float(cfg['friction_scale'])
        except (ValueError, TypeError, KeyError):
            pass

    def _publish_zero_gain_mit(self, torques: Sequence[float]) -> None:
        assert self.q0 is not None
        cur = self.latest_position[:6] if self.latest_position is not None else self.q0
        kp = [0.0] * 6
        kd = [0.0] * 6
        tau = [float(t) for t in torques[:6]]
        for i in range(6):
            if abs(cur[i] - self.q0[i]) > self.config.max_drift_rad:
                tau[i] = 0.0
                kp[i] = 8.0
                kd[i] = 1.0
        msg = MitCommand()
        msg.position = list(self.q0)
        msg.velocity = [0.0] * 6
        msg.kp = kp
        msg.kd = kd
        msg.torque = tau
        self.current_torques = tuple(tau)
        self.mit_pub.publish(msg)

    def _restore_scale(self):
        if self._saved_friction_scale is not None and self.config_client.service_is_ready():
            fut = self.config_client.call_async(
                ConfigureArm.Request(
                    end_effector=self.arm_end_effector,
                    gravity_scale=1.0,
                    friction_scale=-4.0 - self._saved_friction_scale,
                )
            )
            self._saved_friction_scale = None
            return fut
        return None

    def _fail(self, reason: str) -> None:
        self._restore_scale()
        self.error = reason
        self.phase = 'failed'
        self.get_logger().error(reason)

    def _enter_mit_or_noise(self) -> None:
        assert self.latest_status is not None
        self._publish_zero_gain_mit((0.0,) * 6)
        if self.latest_status.ctrl_mode == 1 and self.latest_status.mode_feedback == 6:
            self.phase = 'measuring_noise'
            self.phase_started = time.monotonic()
            return
        if not self.mode_client.service_is_ready():
            return
        self._pending_future = self.mode_client.call_async(SetMode.Request(mode='mit'))
        self.phase = 'setting_mode'

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
            self.q0 = self.latest_position[:6]
            if self.config_client.service_is_ready():
                self._saved_friction_scale = self.arm_friction_scale
                self._pending_future = self.config_client.call_async(
                    ConfigureArm.Request(
                        end_effector=self.arm_end_effector,
                        gravity_scale=1.0,
                        friction_scale=-2.0,
                    )
                )
                self.phase = 'zeroing_friction'
                return
            if not all(self.latest_status.motor_enabled):
                if self.enable_client.service_is_ready():
                    self._pending_future = self.enable_client.call_async(
                        Enable.Request(enable_request=True),
                    )
                    self.phase = 'enabling'
                return
            self._enter_mit_or_noise()
            return

        if self.phase == 'zeroing_friction':
            assert self._pending_future is not None
            if not self._pending_future.done():
                return
            res = self._pending_future.result()
            if res is None or not res.success:
                self._fail(f'Failed to zero friction_scale: {getattr(res, "message", "no response")}')
                return
            if not all(self.latest_status.motor_enabled):
                if not self.enable_client.service_is_ready():
                    return
                self._pending_future = self.enable_client.call_async(
                    Enable.Request(enable_request=True),
                )
                self.phase = 'enabling'
                return
            self._enter_mit_or_noise()
            return

        if self.phase == 'enabling':
            assert self._pending_future is not None
            if not self._pending_future.done():
                return
            res = self._pending_future.result()
            if res is None or not res.enable_response:
                self._fail(f'Failed to enable arm: {getattr(res, "message", "no response")}')
                return
            self._enter_mit_or_noise()
            return

        if self.phase == 'setting_mode':
            self._publish_zero_gain_mit((0.0,) * 6)
            assert self._pending_future is not None
            if not self._pending_future.done():
                return
            res = self._pending_future.result()
            if res is None or not res.success:
                self._fail(f'Failed to enter MIT mode: {getattr(res, "message", "no response")}')
                return
            self.phase = 'measuring_noise'
            self.phase_started = time.monotonic()
            return

        if self.phase == 'measuring_noise':
            self._publish_zero_gain_mit((0.0,) * 6)
            if time.monotonic() - self.phase_started >= (0.10 + self.config.noise_duration):
                for j in self.amplitudes:
                    self.v_noise_by_joint[j] = compute_resting_v_noise(
                        [vel[j - 1] for _, vel in self.noise_samples if len(vel) >= j],
                    )
                self.burst_index = 0
                self.cand_index = 0
                self._reset_step_buffers()
                if self.burst_groups:
                    self.phase = 'running_burst'
                    self.phase_started = time.monotonic()
                else:
                    fut = self._restore_scale()
                    if fut is not None:
                        self._pending_future = fut
                        self.phase = 'restoring_friction'
                    else:
                        self.phase = 'completed'
                        self.completed = True
            return

        if self.phase == 'running_burst':
            now = time.monotonic()
            elapsed = now - self.phase_started
            active_group = [
                j for j in self.burst_groups[self.burst_index] if j not in self.triggered_joints
            ]
            if active_group and elapsed <= self.config.step_duration:
                cand = self.config.candidates[self.cand_index]
                tau = [0.0] * 6
                for j in active_group:
                    idx = j - 1
                    tau[idx] = v0_closed_loop_step_torque(
                        elapsed,
                        self.latest_position[idx],
                        self.latest_velocity[idx],
                        self.arm_model.joint(j),
                        cand,
                        self.amplitudes[j],
                        self.target_friction_scale,
                    )
                self._publish_zero_gain_mit(tau)
                return
            self._publish_zero_gain_mit((0.0,) * 6)
            if active_group:
                cand = self.config.candidates[self.cand_index]
                for j in active_group:
                    qs_j = list(self._step_qs[j])
                    obs_j = list(self._step_obs_vels[j])
                    self.search_records[j].append((cand, qs_j, obs_j))
                    unstable, _, _ = is_v0_step_oscillating(obs_j, self.v_noise_by_joint[j])
                    if unstable:
                        self.triggered_joints.add(j)
            self._reset_step_buffers()
            remaining_in_group = [
                j for j in self.burst_groups[self.burst_index] if j not in self.triggered_joints
            ]
            if remaining_in_group and self.cand_index + 1 < len(self.config.candidates):
                self.cand_index += 1
                self.phase_started = now
                return
            if self.burst_index + 1 < len(self.burst_groups):
                self.burst_index += 1
                self.cand_index = 0
                self.phase_started = now
                return
            fut = self._restore_scale()
            if fut is not None:
                self._pending_future = fut
                self.phase = 'restoring_friction'
                return
            self.phase = 'completed'
            self.completed = True
            return

        if self.phase == 'restoring_friction':
            self._publish_zero_gain_mit((0.0,) * 6)
            if self._pending_future is not None and not self._pending_future.done():
                return
            self.phase = 'completed'
            self.completed = True

    def _reset_step_buffers(self) -> None:
        for j in self.amplitudes:
            self._step_qs[j] = []
            self._step_obs_vels[j] = []
