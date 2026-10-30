"""ROS 2 client node for executing bidirectional quasi-static gravity & static friction probes."""

from dataclasses import dataclass
import json
import math
import time
from pathlib import Path
from typing import Sequence

import numpy as np
from piper_x_driver.backend import MIT_GAIN_SCALE, MIT_KD_LSB, MIT_KP_LSB
from piper_x_driver.gravity import ArmGravityModel, gravity_torques, load_gravity_model, max_critical_kd
from piper_x_interfaces.msg import ArmStatus, MitCommand
from piper_x_interfaces.srv import ConfigureArm, Enable, SetMode
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .identification import ProbeSample

JOINT_NAMES = tuple(f'joint{i}' for i in range(1, 7)) + ('gripper',)
DEFAULT_PROBE_KP = (30.0, 30.0, 30.0, 10.0, 8.0, 6.0)
SAFE_ANCHOR_POSE = (0.0, 0.90, -1.50, 0.0, 0.0, 0.0)


def resolve_safe_anchor(initial_position: Sequence[float], joint: int) -> tuple[float, ...]:
    """Return safe raised 6-joint anchor pose, keeping J1 at current yaw when testing J2..J6."""
    anchor = list(SAFE_ANCHOR_POSE)
    if len(initial_position) >= 6 and joint != 1:
        anchor[0] = float(initial_position[0])
    return tuple(anchor)


def realized_gains(joint: int, kp: float, kd: float) -> tuple[float, float]:
    """Return actual quantized (kp_realized, kd_realized) in joint-side Nm/rad and Nm/(rad/s)."""
    idx = joint - 1
    scale = MIT_GAIN_SCALE[idx]
    raw_uint = round(kp / MIT_KP_LSB[idx]) if kp > 0.0 else 0
    if raw_uint <= 0 or kd < MIT_KD_LSB[idx]:
        return 0.0, 0.0
    kp_uint = min(4095, raw_uint)
    kp_real = scale * (kp_uint * 500.0 / 4095.0)
    kd_real = scale * max(0.0, min(5.0, kd / scale))
    return float(kp_real), float(kd_real)


@dataclass(frozen=True)
class ProbeConfig:
    """Configuration for a single-joint bidirectional quasi-static staircase probe."""

    namespace: str = '/arm_1'
    joint: int = 2
    q_min: float = 0.55
    q_max: float = 1.20
    num_points: int = 9
    pre_travel: float = 0.04
    approach_speed: float = 0.05
    settle_time: float = 0.60
    kp: tuple[float, ...] = DEFAULT_PROBE_KP
    kd: tuple[float, ...] | None = None
    rate_hz: float = 100.0
    use_safe_anchor: bool = True

    def __post_init__(self) -> None:
        if not 1 <= self.joint <= 6:
            raise ValueError(f'joint must be in 1..6, got {self.joint}')
        if not math.isfinite(self.q_min) or not math.isfinite(self.q_max) or self.q_max <= self.q_min:
            raise ValueError('q_min and q_max must be finite with q_max > q_min')
        if self.num_points < 3:
            raise ValueError('num_points must be >= 3')
        if self.pre_travel < 0 or not math.isfinite(self.pre_travel):
            raise ValueError('pre_travel must be finite and non-negative')
        if self.approach_speed <= 0 or not math.isfinite(self.approach_speed):
            raise ValueError('approach_speed must be finite and positive')
        if self.settle_time <= 0 or not math.isfinite(self.settle_time):
            raise ValueError('settle_time must be finite and positive')
        if len(self.kp) != 6 or any(not math.isfinite(v) or v <= 0 for v in self.kp):
            raise ValueError('kp requires six positive finite values')
        if self.kd is None:
            object.__setattr__(self, 'kd', max_critical_kd(self.kp))
        elif len(self.kd) != 6 or any(not math.isfinite(v) or v < 0 for v in self.kd):
            raise ValueError('kd requires six non-negative finite values')
        if self.rate_hz <= 0 or not math.isfinite(self.rate_hz):
            raise ValueError('rate_hz must be finite and positive')


@dataclass(frozen=True)
class _ProbeSegment:
    kind: str
    direction: int
    q_start: float
    q_end: float
    duration: float
    dq_des: float


def build_probe_schedule(
    q_initial: float,
    config: ProbeConfig,
    *,
    anchor_travel: float = 0.0,
    q_return: float | None = None,
) -> list[_ProbeSegment]:
    """Build ascending (+1) and descending (-1) quasi-static staircase approach & hold segments."""
    waypoints = [float(x) for x in np.linspace(config.q_min, config.q_max, config.num_points)]
    segments: list[_ProbeSegment] = []

    # 1. Move to safe anchor pose and lower pre-travel start point (q_min - pre_travel)
    lower_start = waypoints[0] - config.pre_travel
    dist_init = max(abs(lower_start - q_initial), float(anchor_travel))
    if dist_init > 1e-6:
        segments.append(_ProbeSegment(
            kind='positioning',
            direction=0,
            q_start=q_initial,
            q_end=lower_start,
            duration=dist_init / config.approach_speed,
            dq_des=0.0,
        ))
        segments.append(_ProbeSegment(
            kind='settle',
            direction=0,
            q_start=lower_start,
            q_end=lower_start,
            duration=config.settle_time,
            dq_des=0.0,
        ))

    # 2. Ascending staircase (+1 approach into each waypoint from below)
    cur_q = lower_start
    for wp in waypoints:
        step = abs(wp - cur_q)
        if step > 1e-6:
            segments.append(_ProbeSegment(
                kind='approach',
                direction=+1,
                q_start=cur_q,
                q_end=wp,
                duration=step / config.approach_speed,
                dq_des=0.0,
            ))
        segments.append(_ProbeSegment(
            kind='measure',
            direction=+1,
            q_start=wp,
            q_end=wp,
            duration=config.settle_time,
            dq_des=0.0,
        ))
        cur_q = wp

    # 3. Move to upper pre-travel turnaround point (q_max + pre_travel)
    upper_start = waypoints[-1] + config.pre_travel
    if config.pre_travel > 1e-6:
        segments.append(_ProbeSegment(
            kind='positioning',
            direction=0,
            q_start=cur_q,
            q_end=upper_start,
            duration=config.pre_travel / config.approach_speed,
            dq_des=0.0,
        ))
        segments.append(_ProbeSegment(
            kind='settle',
            direction=0,
            q_start=upper_start,
            q_end=upper_start,
            duration=config.settle_time,
            dq_des=0.0,
        ))
        cur_q = upper_start

    # 4. Descending staircase (-1 approach into each waypoint from above)
    for wp in reversed(waypoints):
        step = abs(cur_q - wp)
        if step > 1e-6:
            segments.append(_ProbeSegment(
                kind='approach',
                direction=-1,
                q_start=cur_q,
                q_end=wp,
                duration=step / config.approach_speed,
                dq_des=0.0,
            ))
        segments.append(_ProbeSegment(
            kind='measure',
            direction=-1,
            q_start=wp,
            q_end=wp,
            duration=config.settle_time,
            dq_des=0.0,
        ))
        cur_q = wp

    # 5. Return to safe anchor angle (not raw folded q_initial)
    target_return = q_initial if q_return is None else float(q_return)
    ret_dist = abs(target_return - cur_q)
    if ret_dist > 1e-6:
        segments.append(_ProbeSegment(
            kind='return',
            direction=0,
            q_start=cur_q,
            q_end=target_return,
            duration=ret_dist / config.approach_speed,
            dq_des=0.0,
        ))
    return segments


class ProbeCollectorNode(Node):
    """ROS 2 client node that executes bidirectional quasi-static probes with compensation off."""

    def __init__(self, config: ProbeConfig, **kwargs):
        super().__init__('piper_gravity_collector', **kwargs)
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
        self.saved_gravity_scale: float = 1.0
        self.saved_friction_scale: float = 0.5
        self.applied_gravity_scale: float = 1.0
        self.applied_gravity_torques: tuple[float, ...] | None = None
        self.gravity_model: ArmGravityModel | None = None
        self._loaded_gravity_path: str = ''

        self.initial_position: tuple[float, ...] | None = None
        self.anchor_position: tuple[float, ...] | None = None
        self.schedule: list[_ProbeSegment] = []
        self.segment_index = 0
        self.segment_started = 0.0
        self.probe_started = 0.0
        self.current_q_des = 0.0
        self.current_dq_des = 0.0
        self.current_direction = 0
        self.current_kind = ''
        self._pending_future = None
        self.samples: list[ProbeSample] = []

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

        if (self.phase == 'running'
                and self.anchor_position is not None
                and self.current_kind == 'measure'
                and self.current_direction in (-1, 1)):
            now = time.monotonic()
            seg = self.schedule[self.segment_index]
            elapsed = now - self.segment_started
            j_idx = self.config.joint - 1
            dq_val = self.latest_velocity[j_idx]
            # Wait for the second half of the settle hold and verify velocity is near zero
            if elapsed >= 0.4 * seg.duration and abs(dq_val) <= 0.03:
                assert self.config.kd is not None
                kp_real, kd_real = realized_gains(
                    self.config.joint,
                    self.config.kp[j_idx],
                    self.config.kd[j_idx],
                )
                q_val = self.latest_position[j_idx]
                g_torques = gravity_torques(self.latest_position[:6], **self.effector_kwargs)
                if (self.applied_gravity_torques is not None
                        and len(self.applied_gravity_torques) >= 6):
                    g_applied = float(self.applied_gravity_torques[j_idx])
                else:
                    g_active = (
                        g_torques[j_idx]
                        if self.gravity_model is None
                        else self.gravity_model.torques(self.latest_position[:6], **self.effector_kwargs)[j_idx]
                    )
                    g_applied = self.applied_gravity_scale * float(g_active)
                tau_kp = g_applied + kp_real * (self.current_q_des - q_val) - kd_real * dq_val
                self.samples.append(ProbeSample(
                    t=now - self.probe_started,
                    joint=self.config.joint,
                    direction=self.current_direction,
                    q_target=self.current_q_des,
                    q=q_val,
                    dq=dq_val,
                    tau_meas=self.latest_torque[j_idx],
                    tau_kp=float(tau_kp),
                    tau_gravity=float(g_torques[j_idx]),
                    q_full=self.latest_position[:6],
                ))

    def _on_status(self, message: ArmStatus) -> None:
        self.latest_status = message

    def _on_control_state(self, message: String) -> None:
        try:
            state = json.loads(message.data)
            cfg = state.get('configuration', {})
            if isinstance(cfg.get('end_effector'), str) and cfg['end_effector']:
                self.arm_end_effector = cfg['end_effector']
            if self.phase == 'waiting_feedback':
                if isinstance(cfg.get('gravity_scale'), (int, float)):
                    self.saved_gravity_scale = float(cfg['gravity_scale'])
                if isinstance(cfg.get('friction_scale'), (int, float)):
                    self.saved_friction_scale = float(cfg['friction_scale'])
            if isinstance(state.get('gravity_scale_applied'), (int, float)):
                self.applied_gravity_scale = float(state['gravity_scale_applied'])
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

        # 标定前保持重力补偿开启（gravity_scale=1.0 防止机械臂失重下垂），仅将 friction_scale 设为 0.0
        if self.phase == 'waiting_feedback':
            self.initial_position = self.latest_position[:6]
            self.anchor_position = (
                resolve_safe_anchor(self.initial_position, self.config.joint)
                if self.config.use_safe_anchor
                else self.initial_position
            )
            self.current_q_des = self.initial_position[self.config.joint - 1]
            self.current_dq_des = 0.0
            if self.config_client.service_is_ready():
                self._pending_future = self.config_client.call_async(
                    ConfigureArm.Request(
                        end_effector=self.arm_end_effector,
                        gravity_scale=1.0,
                        friction_scale=0.0,
                    )
                )
                self.phase = 'zeroing_compensation'
                return
            self._check_enable_or_mit()
            return

        if self.phase == 'zeroing_compensation':
            assert self._pending_future is not None
            if not self._pending_future.done():
                return
            res = self._pending_future.result()
            if res is None or not res.success:
                self._fail(
                    f'Failed to zero compensation: {getattr(res, "message", "no response")}'
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
                    # Use cosine S-curve easing so velocity starts and ends smoothly at 0 without inertial overshoot
                    smooth = 0.5 * (1.0 - math.cos(math.pi * ratio))
                    self.current_q_des = seg.q_start + smooth * (seg.q_end - seg.q_start)
                    self.current_dq_des = 0.0
                    self.current_direction = seg.direction
                    self.current_kind = seg.kind
                    blend = smooth if (self.segment_index == 0 and seg.kind == 'positioning') else 1.0
                    self._publish_mit(self.current_q_des, 0.0, anchor_blend=blend)
                    return
                self.segment_index += 1
                self.segment_started = now

            assert self.anchor_position is not None
            self.current_q_des = self.anchor_position[self.config.joint - 1]
            self.current_dq_des = 0.0
            self.current_direction = 0
            self.current_kind = ''
            self._publish_mit(self.current_q_des, 0.0)
            if self.config_client.service_is_ready():
                self._pending_future = self.config_client.call_async(
                    ConfigureArm.Request(
                        end_effector=self.arm_end_effector,
                        gravity_scale=self.saved_gravity_scale,
                        friction_scale=self.saved_friction_scale,
                    )
                )
                self.phase = 'restoring_compensation'
                return
            self.phase = 'completed'
            self.completed = True
            return

        if self.phase == 'restoring_compensation':
            assert self.anchor_position is not None
            self._publish_mit(self.current_q_des, 0.0)
            assert self._pending_future is not None
            if not self._pending_future.done():
                return
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
        self.schedule = build_probe_schedule(
            q_init, self.config, anchor_travel=anchor_travel, q_return=q_ret,
        )
        self.segment_index = 0
        self.probe_started = time.monotonic()
        self.segment_started = self.probe_started
        self.phase = 'running'
