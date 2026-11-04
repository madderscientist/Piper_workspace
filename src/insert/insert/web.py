"""ROS 2 + HTTP Dashboard for InsertAnything policy deployment on PiPER-X (CAN4 / arm_4)."""

import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading
import time
from typing import Any, Sequence
from urllib.parse import urlsplit

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import WrenchStamped
import numpy as np
from piper_x_driver.configuration import ArmConfig, default_config_path, FleetConfig
from piper_x_driver.controller import MIT_POSITION_LIMIT, RateMeter
from piper_x_driver.gravity import ArmGravityModel, max_critical_kd
from piper_x_interfaces.msg import ArmStatus, GripperCommand, MitCommand
from piper_x_interfaces.srv import Enable, SetMode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .policy import (
    DEFAULT_TIP_LENGTH_M,
    POLICY_PERIOD_S,
    SERVO_LAG_TAU_S,
    GoalPose,
    InsertAnythingPolicy,
    PolicyStepResult,
    WrenchFilter,
    equivalent_filter_tau,
    flange_wrench_to_tip_wrench,
    forward_tip_pose,
    normalize_tip_offset,
    quat_rotvec,
    quat_to_rpy_deg,
    relative_pose_in_hole,
)

JOINT_NAMES = tuple(f'joint{i}' for i in range(1, 7)) + ('gripper',)
DEFAULT_POLICY_KP = (45.0, 45.0, 45.0, 10.0, 6.0, 4.0)
DEFAULT_OPEN_WIDTH_M = 0.050
DEFAULT_CLOSE_WIDTH_M = 0.000
DEFAULT_GRIP_EFFORT = 2.0


def web_root_path() -> Path:
    try:
        share = Path(get_package_share_directory('insert')) / 'web'
        if (share / 'index.html').is_file():
            return share
    except PackageNotFoundError:
        pass
    return Path(__file__).resolve().parent.parent / 'web'


def resolve_target_arm(
    fleet: FleetConfig | None, arm_selector: str = 'can4', namespace_override: str = '',
) -> tuple[str, str, str, ArmGravityModel | None]:
    selector = (arm_selector or 'can4').strip()
    ns_clean = ('/' + namespace_override.strip('/')) if namespace_override.strip('/') else ''
    if fleet is not None and fleet.arms:
        selected: ArmConfig | None = None
        if ns_clean:
            selected = next((a for a in fleet.arms if a.namespace == ns_clean), None)
        if selected is None:
            selected = next(
                (a for a in fleet.arms if selector in (a.id, a.can_port, a.namespace, a.namespace.lstrip('/'))),
                None,
            )
        if selected is None and selector.lower() in ('can4', '4', 'arm4', 'arm_4'):
            selected = next((a for a in fleet.arms if a.id == 'arm_4'), fleet.arms[-1])
        if selected is None:
            selected = fleet.arms[-1]
        return selected.id, (ns_clean or selected.namespace), selected.can_port, fleet.gravity_model(selected)
    return 'arm_4', (ns_clean or '/arm_4'), 'can4', None


class InsertDashboardNode(Node):
    """ROS 2 node managing zero-force drag, gripper switch, goal recording, and 50 Hz ONNX policy."""

    def __init__(
        self, *, config_file: str | Path | None = None, arm_selector: str = 'can4',
        namespace_override: str = '', onnx_path: str | Path | None = None,
        tip_length_m: float = DEFAULT_TIP_LENGTH_M,
        tip_offset_m: Sequence[float] | None = None,
    ) -> None:
        super().__init__('insert_dashboard')
        self.lock = threading.RLock()
        try:
            self.fleet: FleetConfig | None = FleetConfig(Path(config_file) if config_file else default_config_path())
        except Exception:
            self.fleet = None

        self.arm_id, self.arm_namespace, self.can_port, self.gravity_model = resolve_target_arm(
            self.fleet, arm_selector=arm_selector, namespace_override=namespace_override,
        )
        self.can_label = 'can4' if arm_selector.lower() == 'can4' else self.can_port
        self.tip_offset_m = normalize_tip_offset(tip_offset_m, tip_length_m)
        self.tip_length_m = float(self.tip_offset_m[2])
        self.policy = InsertAnythingPolicy(onnx_path=onnx_path)
        self.wrench_filter = WrenchFilter()
        self.policy_kp: tuple[float, ...] = DEFAULT_POLICY_KP
        self.policy_kd: tuple[float, ...] = max_critical_kd(self.policy_kp, 'gripper', self.gravity_model)

        self.mode = 'idle'
        self.goal: GoalPose | None = None
        self.hold_joints: tuple[float, ...] | None = None
        self.last_cmd_joints: tuple[float, ...] | None = None
        self.servo_expected_joints: np.ndarray | None = None
        self._servo_updated_at: float | None = None
        self.last_step_result: PolicyStepResult | None = None
        self.policy_started_at: float | None = None
        self.reached_goal = False
        self.reached_goal_steps = 0
        self.last_error = ''
        self.status_message = '就绪：可先零力拖动将插头插入孔中，点击“记录孔中位置”'

        self.gripper_state = 'idle'
        self.gripper_open_width_m = DEFAULT_OPEN_WIDTH_M
        self.gripper_close_width_m = DEFAULT_CLOSE_WIDTH_M
        self.gripper_effort = DEFAULT_GRIP_EFFORT
        self.gripper_cmd_width_m = DEFAULT_OPEN_WIDTH_M
        self.gripper_cmd_effort = 0.0

        self.joint_positions: tuple[float, ...] | None = None
        self.joint_velocities: tuple[float, ...] = (0.0,) * 6
        self.joint_efforts: tuple[float, ...] = (0.0,) * 6
        self.gripper_width_m: float | None = None
        self.joints_received_at: float | None = None
        self.arm_status: ArmStatus | None = None
        self.status_received_at: float | None = None
        self.control_state: dict[str, Any] = {}
        self.control_received_at: float | None = None
        self.wrench_received_at: float | None = None
        self.policy_rate = RateMeter()
        self.history: deque[dict[str, Any]] = deque(maxlen=120)
        self.logs: deque[dict[str, Any]] = deque(maxlen=50)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        ns = self.arm_namespace
        self.create_subscription(JointState, f'{ns}/joint_states_feedback', self._on_joints, qos)
        self.create_subscription(WrenchStamped, f'{ns}/end_wrench_stamped', self._on_wrench, qos)
        self.create_subscription(ArmStatus, f'{ns}/arm_status', self._on_status, qos)
        self.create_subscription(String, f'{ns}/control_state', self._on_control_state, qos)

        self.mit_pub = self.create_publisher(MitCommand, f'{ns}/command/mit', 10)
        self.gripper_pub = self.create_publisher(GripperCommand, f'{ns}/command/gripper', 10)
        self.enable_client = self.create_client(Enable, f'{ns}/enable_srv')
        self.mode_client = self.create_client(SetMode, f'{ns}/set_mode')

        self._mit_msg = MitCommand()
        self._grip_msg = GripperCommand()
        self._timer = self.create_timer(POLICY_PERIOD_S, self._tick)
        self._log('INFO', f'已绑定 {self.can_label} ({self.arm_namespace})，加载模型 {self.policy.onnx_path.name}')

    def _log(self, level: str, text: str) -> None:
        with self.lock:
            self.logs.append({'time': time.strftime('%H:%M:%S'), 'level': level, 'msg': str(text)})

    def _on_joints(self, msg: JointState) -> None:
        now = time.monotonic()
        names = tuple(msg.name)
        if names == JOINT_NAMES and len(msg.position) >= 7:
            order, g_idx = range(6), 6
        else:
            idx_map = {name: i for i, name in enumerate(names)}
            if not all(f'joint{i}' in idx_map for i in range(1, 7)):
                return
            order = [idx_map[f'joint{i}'] for i in range(1, 7)]
            g_idx = idx_map.get('gripper')
        pos6 = tuple(float(msg.position[i]) for i in order)
        if not all(math.isfinite(v) for v in pos6):
            return
        grip = float(msg.position[g_idx]) if g_idx is not None and g_idx < len(msg.position) else 0.0
        vel6 = tuple(float(msg.velocity[i]) for i in order) if all(i < len(msg.velocity) for i in order) else (0.0,) * 6
        eff6 = tuple(float(msg.effort[i]) for i in order) if all(i < len(msg.effort) for i in order) else (0.0,) * 6
        with self.lock:
            self.joint_positions = pos6
            self.gripper_width_m = grip if math.isfinite(grip) else self.gripper_width_m
            self.joint_velocities = vel6 if all(math.isfinite(v) for v in vel6) else (0.0,) * 6
            self.joint_efforts = eff6 if all(math.isfinite(v) for v in eff6) else (0.0,) * 6
            self.joints_received_at = now

    def _update_servo_expected(self, now: float) -> None:
        if self.joint_positions is None:
            return
        if self.servo_expected_joints is None or self.last_cmd_joints is None or self.mode != 'policy':
            self.servo_expected_joints = np.array(self.joint_positions[:6], dtype=np.float64)
            self._servo_updated_at = now
            return
        dt = 0.0 if self._servo_updated_at is None else max(0.0, now - self._servo_updated_at)
        self._servo_updated_at = now
        if 0.0 < dt <= 0.5:
            alpha = 1.0 - math.exp(-dt / SERVO_LAG_TAU_S)
            self.servo_expected_joints += alpha * (np.asarray(self.last_cmd_joints[:6]) - self.servo_expected_joints)

    def _ingest_flange_wrench(self, raw_flange_wrench: Sequence[float], now: float) -> None:
        if self.joint_positions is not None:
            self._update_servo_expected(now)
            in_policy = self.mode == 'policy'
            tip_w_g = flange_wrench_to_tip_wrench(
                raw_flange_wrench,
                self.joint_positions,
                tip_offset_m=self.tip_offset_m,
                gravity_model=self.gravity_model,
                q_cmd=self.last_cmd_joints if in_policy else None,
                q_servo=self.servo_expected_joints if in_policy else None,
                kp=self.policy_kp if in_policy else None,
            )
            self.wrench_filter.update(tip_w_g, timestamp=now, already_at_tip=True)
        else:
            self.wrench_filter.update(raw_flange_wrench, timestamp=now, already_at_tip=False)

    def _on_wrench(self, msg: WrenchStamped) -> None:
        now = time.monotonic()
        f, t = msg.wrench.force, msg.wrench.torque
        raw = (float(f.x), float(f.y), float(f.z), float(t.x), float(t.y), float(t.z))
        if all(math.isfinite(v) for v in raw):
            with self.lock:
                self._ingest_flange_wrench(raw, now)
                self.wrench_received_at = now

    def _on_status(self, msg: ArmStatus) -> None:
        with self.lock:
            self.arm_status = msg
            self.status_received_at = time.monotonic()

    def _on_control_state(self, msg: String) -> None:
        now = time.monotonic()
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(data, dict):
            return
        with self.lock:
            self.control_state = data
            self.control_received_at = now
            if self.wrench_received_at is None or (now - self.wrench_received_at) > 0.05:
                ew = data.get('end_wrench')
                if isinstance(ew, list) and len(ew) == 6 and all(
                    isinstance(x, (int, float)) and math.isfinite(float(x)) for x in ew
                ):
                    self._ingest_flange_wrench([float(x) for x in ew], now)

    def _publish_mit(
        self, position: Sequence[float], kp: Sequence[float], kd: Sequence[float],
        velocity: Sequence[float] = (0.0,) * 6, torque: Sequence[float] = (0.0,) * 6,
    ) -> None:
        msg = self._mit_msg
        msg.position = [max(-MIT_POSITION_LIMIT, min(MIT_POSITION_LIMIT, float(v))) for v in position[:6]]
        msg.velocity = [float(v) for v in velocity[:6]]
        msg.kp = [float(v) for v in kp[:6]]
        msg.kd = [float(v) for v in kd[:6]]
        msg.torque = [float(v) for v in torque[:6]]
        self.mit_pub.publish(msg)

    def _publish_gripper(self, width_m: float, effort: float) -> None:
        msg = self._grip_msg
        msg.width = max(0.0, min(0.1, float(width_m)))
        msg.effort = max(0.0, min(5.0, float(effort)))
        self.gripper_pub.publish(msg)

    def _ensure_driver_mit_mode(self, q_preload: Sequence[float]) -> None:
        if str(self.control_state.get('mode', '')) != 'mit':
            self._publish_mit(q_preload, (0.0,) * 6, (0.0,) * 6)
            if self.mode_client.service_is_ready():
                self.mode_client.call_async(SetMode.Request(mode='mit'))

    def _tick(self) -> None:
        now = time.monotonic()
        with self.lock:
            if self.gripper_state in ('open', 'closed') and self.gripper_cmd_effort > 0.0:
                self._publish_gripper(self.gripper_cmd_width_m, self.gripper_cmd_effort)
            if self.joint_positions is None:
                return
            q_meas = self.joint_positions

            if self.mode == 'drag':
                self.last_cmd_joints = q_meas
                self.servo_expected_joints = np.array(q_meas[:6], dtype=np.float64)
                self._ensure_driver_mit_mode(q_meas)
                self._publish_mit(q_meas, (0.0,) * 6, (0.0,) * 6)
                return

            if self.mode == 'hold':
                target_q = self.hold_joints if self.hold_joints is not None else q_meas
                self.last_cmd_joints = target_q
                self._ensure_driver_mit_mode(target_q)
                self._publish_mit(target_q, self.policy_kp, self.policy_kd)
                return

            if self.mode == 'policy':
                if self.goal is None:
                    self.mode = 'drag'
                    self.status_message = '未记录孔中目标位置，已退回零力拖动'
                    return
                try:
                    self._ensure_driver_mit_mode(q_meas)
                    wrench_t = self.wrench_filter.tip_wrench(tip_offset_m=self.tip_offset_m)
                    step_res = self.policy.step(
                        q_meas,
                        self.goal,
                        wrench_t,
                        tip_offset_m=self.tip_offset_m,
                        gravity_model=self.gravity_model,
                    )
                    self.last_cmd_joints = step_res.q_cmd
                    self._publish_mit(step_res.q_cmd, self.policy_kp, self.policy_kd)
                    self.policy_rate.mark(now)
                    self.last_step_result = step_res
                    self.last_error = ''

                    xy_err_m = float(np.linalg.norm(step_res.p_ht[:2]))
                    z_ht_m = float(step_res.p_ht[2])
                    rot_err_deg = math.degrees(float(np.linalg.norm(quat_rotvec(step_res.q_ht))))
                    self.reached_goal_steps = (
                        self.reached_goal_steps + 1 if (z_ht_m <= 0.0002 and xy_err_m <= 0.003) else 0
                    )
                    if self.reached_goal_steps >= 5:
                        self.reached_goal = True

                    self.history.append({
                        'step': step_res.step,
                        'p_ht_mm': [float(v * 1000.0) for v in step_res.p_ht],
                        'xy_err_mm': xy_err_m * 1000.0,
                        'rot_err_deg': rot_err_deg,
                        'force_norm_n': float(np.linalg.norm(step_res.wrench_t[:3])),
                        'moment_norm_nm': float(np.linalg.norm(step_res.wrench_t[3:6])),
                    })
                except Exception as exc:
                    self.last_error = str(exc)
                    self.status_message = f'策略执行异常：{exc}'
                    self._log('ERROR', self.status_message)
                    self.hold_joints = q_meas
                    self.mode = 'hold'

    def set_power(self, enable: bool, timeout: float = 3.5) -> dict[str, Any]:
        if not self.enable_client.service_is_ready():
            raise RuntimeError(f'驱动使能服务 {self.arm_namespace}/enable_srv 未就绪')
        with self.lock:
            if not enable:
                self.mode = 'idle'
                self.gripper_state = 'idle'
                self.gripper_cmd_effort = 0.0
        future = self.enable_client.call_async(Enable.Request(enable_request=bool(enable)))
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError('等待驱动使能响应超时')
        resp = future.result()
        if not resp.enable_response:
            raise RuntimeError(resp.message or '使能/失能请求未通过')
        with self.lock:
            if enable and self.mode == 'idle':
                self.mode = 'drag'
            self.status_message = f'已{"使能" if enable else "失能"}机械臂：{resp.message}'
        self._log('INFO', self.status_message)
        return {'ok': True, 'enabled': bool(enable), 'message': self.status_message}

    def set_gripper(
        self, action: str, *, width_m: float | None = None, effort: float | None = None,
    ) -> dict[str, Any]:
        act = action.strip().lower()
        with self.lock:
            if effort is not None:
                self.gripper_effort = max(0.0, min(5.0, float(effort)))
            if act == 'open':
                if width_m is not None:
                    self.gripper_open_width_m = max(0.0, min(0.1, float(width_m)))
                self.gripper_cmd_width_m = self.gripper_open_width_m
                self.gripper_cmd_effort = max(0.5, self.gripper_effort)
                self.gripper_state = 'open'
                label = f'已打开夹爪（目标开度 {self.gripper_cmd_width_m * 1000:.1f} mm，力度 {self.gripper_cmd_effort:.1f}）'
            elif act == 'close':
                if width_m is not None:
                    self.gripper_close_width_m = max(0.0, min(0.1, float(width_m)))
                self.gripper_cmd_width_m = self.gripper_close_width_m
                self.gripper_cmd_effort = max(0.5, self.gripper_effort)
                self.gripper_state = 'closed'
                label = f'已闭合夹爪夹紧插头（目标开度 {self.gripper_cmd_width_m * 1000:.1f} mm，持续保持力度 {self.gripper_cmd_effort:.1f}）'
            elif act == 'release':
                self.gripper_cmd_width_m = float(
                    self.gripper_width_m if self.gripper_width_m is not None else self.gripper_cmd_width_m
                )
                self.gripper_cmd_effort = 0.0
                self.gripper_state = 'released'
                label = '已释放夹爪（零力）'
            elif act == 'set':
                w = self.gripper_cmd_width_m if width_m is None else max(0.0, min(0.1, float(width_m)))
                eff = self.gripper_effort if effort is None else max(0.0, min(5.0, float(effort)))
                self.gripper_cmd_width_m, self.gripper_cmd_effort = w, eff
                self.gripper_state = 'released' if eff <= 0.0 else ('closed' if w <= 0.015 else 'open')
                label = f'已设置夹爪（开度 {w * 1000:.1f} mm，力度 {eff:.1f}）'
            else:
                raise ValueError(f'未知的夹爪操作: {action}')
            self._publish_gripper(self.gripper_cmd_width_m, self.gripper_cmd_effort)
            self.status_message = label
        self._log('INFO', label)
        return {
            'ok': True,
            'gripper_state': self.gripper_state,
            'width_m': self.gripper_cmd_width_m,
            'effort': self.gripper_cmd_effort,
            'message': label,
        }

    def enter_drag(self) -> dict[str, Any]:
        with self.lock:
            self.mode = 'drag'
            self.hold_joints = None
            if self.joint_positions is not None:
                self._ensure_driver_mit_mode(self.joint_positions)
                self._publish_mit(self.joint_positions, (0.0,) * 6, (0.0,) * 6)
            self.status_message = '已进入零力拖动模式：可手拖机械臂将插头插入孔中，夹爪保持当前状态'
        self._log('INFO', self.status_message)
        return {'ok': True, 'mode': self.mode, 'message': self.status_message}

    def _build_goal(self, joints: tuple[float, ...], recorded_at: float) -> GoalPose:
        p_bh, q_bh = forward_tip_pose(joints, tip_offset_m=self.tip_offset_m, gravity_model=self.gravity_model)
        return GoalPose(
            joints=joints,
            p_bh=(float(p_bh[0]), float(p_bh[1]), float(p_bh[2])),
            q_bh=(float(q_bh[0]), float(q_bh[1]), float(q_bh[2]), float(q_bh[3])),
            tip_length_m=self.tip_length_m,
            recorded_at=recorded_at,
            tip_offset_m=self.tip_offset_m,
        )

    def record_goal(self, tip_length_m: float | None = None, *, tip_offset_m: Sequence[float] | None = None) -> dict[str, Any]:
        with self.lock:
            if self.joint_positions is None:
                raise RuntimeError(f'尚未收到 {self.arm_namespace}/joint_states_feedback 关节反馈，无法记录位置')
            if tip_offset_m is not None or tip_length_m is not None:
                self.tip_offset_m = normalize_tip_offset(tip_offset_m, tip_length_m)
                self.tip_length_m = float(self.tip_offset_m[2])
            q_now = tuple(float(v) for v in self.joint_positions[:6])
            self.goal = self._build_goal(q_now, time.time())
            self.policy.reset()
            self.last_step_result = None
            self.reached_goal = False
            self.reached_goal_steps = 0
            self.history.clear()
            pos_mm = [v * 1000.0 for v in self.goal.p_bh]
            self.status_message = (
                f'已记录孔中目标位置：XYZ = [{pos_mm[0]:.1f}, {pos_mm[1]:.1f}, {pos_mm[2]:.1f}] mm；'
                '现在可将末端移出孔外，然后点击“执行策略”'
            )
            goal_dict = self.goal.to_dict()
        self._log('INFO', self.status_message)
        return {'ok': True, 'goal': goal_dict, 'message': self.status_message}

    def clear_goal(self) -> dict[str, Any]:
        with self.lock:
            if self.mode == 'policy':
                self.mode = 'drag'
            self.goal = None
            self.last_step_result = None
            self.reached_goal = False
            self.reached_goal_steps = 0
            self.history.clear()
            self.status_message = '已清除记录的孔中位置'
        self._log('INFO', self.status_message)
        return {'ok': True, 'message': self.status_message}

    def tare_wrench(self) -> dict[str, Any]:
        with self.lock:
            bias = self.wrench_filter.tare()
            self.status_message = (
                f'已完成末端力觉标零：bias = [{bias[0]:.2f}, {bias[1]:.2f}, {bias[2]:.2f}] N, '
                f'[{bias[3]:.3f}, {bias[4]:.3f}, {bias[5]:.3f}] N·m'
            )
        self._log('INFO', self.status_message)
        return {'ok': True, 'bias': [float(v) for v in bias], 'message': self.status_message}

    def start_policy(self) -> dict[str, Any]:
        with self.lock:
            if self.goal is None:
                raise RuntimeError('请先将插头放入孔中并点击“记录孔中位置”')
            if self.joint_positions is None:
                raise RuntimeError(f'尚未收到 {self.arm_namespace}/joint_states_feedback 关节反馈')
            self.policy.reset()
            self.wrench_filter.reset()
            self.last_cmd_joints = tuple(float(v) for v in self.joint_positions[:6])
            self.servo_expected_joints = np.array(self.joint_positions[:6], dtype=np.float64)
            self._servo_updated_at = time.monotonic()
            self.last_step_result = None
            self.policy_started_at = time.monotonic()
            self.reached_goal = False
            self.reached_goal_steps = 0
            self.last_error = ''
            self.history.clear()
            self._ensure_driver_mit_mode(self.joint_positions)
            self.mode = 'policy'
            self.status_message = '正在执行 InsertAnything 插入策略（50 Hz 闭环控制中）'
        self._log('INFO', self.status_message)
        return {'ok': True, 'mode': self.mode, 'message': self.status_message}

    def stop_policy(self, next_mode: str = 'drag') -> dict[str, Any]:
        target_mode = next_mode.strip().lower() if next_mode else 'drag'
        if target_mode not in ('drag', 'hold', 'idle'):
            raise ValueError('stop mode must be drag, hold, or idle')
        with self.lock:
            self.hold_joints = (
                tuple(float(v) for v in self.joint_positions[:6])
                if target_mode == 'hold' and self.joint_positions is not None
                else None
            )
            self.mode = target_mode
            label = {
                'drag': '已停止策略并切回零力拖动模式',
                'hold': '已停止策略并保持当前位置',
                'idle': '已停止策略',
            }[target_mode]
            self.status_message = label
        self._log('INFO', label)
        return {'ok': True, 'mode': self.mode, 'message': label}

    def update_parameters(self, body: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            new_offset: tuple[float, float, float] | None = None
            if body.get('tip_offset_m') is not None:
                new_offset = normalize_tip_offset(body['tip_offset_m'])
            elif body.get('tip_offset_cm') is not None:
                cm = body['tip_offset_cm']
                if not isinstance(cm, list) or len(cm) != 3:
                    raise ValueError('tip_offset_cm requires [x_cm, y_cm, z_cm]')
                new_offset = normalize_tip_offset([float(v) / 100.0 for v in cm])
            elif body.get('tip_length_m') is not None:
                new_offset = (self.tip_offset_m[0], self.tip_offset_m[1], float(body['tip_length_m']))

            if new_offset is not None:
                self.tip_offset_m = new_offset
                self.tip_length_m = float(new_offset[2])
                if self.goal is not None:
                    self.goal = self._build_goal(self.goal.joints, self.goal.recorded_at)
            if body.get('filter_tau_s') is not None:
                tau_val = float(body['filter_tau_s'])
                if not math.isfinite(tau_val) or tau_val <= 0.0:
                    raise ValueError('filter_tau_s must be positive')
                self.wrench_filter.tau = tau_val
            elif body.get('tau_1khz_s') is not None:
                tau_1k = float(body['tau_1khz_s'])
                self.wrench_filter.tau_1khz = tau_1k
                self.wrench_filter.tau = equivalent_filter_tau(tau_1k)
            if 'kp' in body:
                kp_raw = body['kp']
                if not isinstance(kp_raw, list) or len(kp_raw) != 6:
                    raise ValueError('kp requires 6 values')
                self.policy_kp = tuple(max(0.0, min(500.0, float(v))) for v in kp_raw)
                if body.get('kd') is None:
                    self.policy_kd = max_critical_kd(self.policy_kp, 'gripper', self.gravity_model)
            if body.get('kd') is not None:
                kd_raw = body['kd']
                if not isinstance(kd_raw, list) or len(kd_raw) != 6:
                    raise ValueError('kd requires 6 values')
                auto_kd = max_critical_kd(self.policy_kp, 'gripper', self.gravity_model)
                self.policy_kd = tuple(
                    auto if float(v) < 0.0 else max(0.0, min(100.0, float(v)))
                    for v, auto in zip(kd_raw, auto_kd)
                )
            ox_cm, oy_cm, oz_cm = (v * 100.0 for v in self.tip_offset_m)
            self.status_message = (
                f'参数已更新：作用点偏移 = [{ox_cm:+.1f}, {oy_cm:+.1f}, {oz_cm:+.1f}] cm, '
                f'200Hz 等效滤波 τ = {self.wrench_filter.tau * 1000:.2f} ms'
            )
        self._log('INFO', self.status_message)
        return {
            'ok': True,
            'tip_length_m': self.tip_length_m,
            'tip_offset_m': list(self.tip_offset_m),
            'tip_offset_cm': [v * 100.0 for v in self.tip_offset_m],
            'filter_tau_s': self.wrench_filter.tau,
            'kp': list(self.policy_kp),
            'kd': list(self.policy_kd),
            'message': self.status_message,
        }

    def status_payload(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            joints_fresh = self.joints_received_at is not None and (now - self.joints_received_at) <= 2.0
            status_fresh = self.status_received_at is not None and (now - self.status_received_at) <= 2.0
            control_fresh = self.control_received_at is not None and (now - self.control_received_at) <= 2.0
            online = bool(joints_fresh or status_fresh or control_fresh)

            enabled_flags = list(self.arm_status.motor_enabled) if self.arm_status is not None else [False] * 6
            power_state = (
                'enabled' if (status_fresh and all(enabled_flags)) else
                'mixed' if (status_fresh and any(enabled_flags)) else
                'disabled' if status_fresh else 'unknown'
            )

            tip_pose_dict = None
            hole_rel_dict = None
            if self.joint_positions is not None:
                p_bt, q_bt = forward_tip_pose(
                    self.joint_positions, tip_offset_m=self.tip_offset_m, gravity_model=self.gravity_model,
                )
                tip_pose_dict = {
                    'position_m': [float(v) for v in p_bt],
                    'position_mm': [float(v * 1000.0) for v in p_bt],
                    'quaternion_wxyz': [float(v) for v in q_bt],
                    'rpy_deg': list(quat_to_rpy_deg(q_bt)),
                }
                if self.goal is not None:
                    p_ht, q_ht = relative_pose_in_hole(p_bt, q_bt, self.goal.p_bh, self.goal.q_bh)
                    rot_vec = quat_rotvec(q_ht)
                    hole_rel_dict = {
                        'p_ht_m': [float(v) for v in p_ht],
                        'p_ht_mm': [float(v * 1000.0) for v in p_ht],
                        'q_ht_wxyz': [float(v) for v in q_ht],
                        'rotvec_deg': [math.degrees(float(v)) for v in rot_vec],
                        'xy_distance_mm': float(np.linalg.norm(p_ht[:2]) * 1000.0),
                        'total_distance_mm': float(np.linalg.norm(p_ht) * 1000.0),
                        'angle_error_deg': math.degrees(float(np.linalg.norm(rot_vec))),
                    }

            wrench_t = self.wrench_filter.tip_wrench(tip_offset_m=self.tip_offset_m)
            wrench_s = self.wrench_filter.filtered_s
            sr = self.last_step_result
            step_dict = None if sr is None else {
                'step': int(sr.step),
                'actions': [float(v) for v in sr.actions],
                'dp_h_mm': [float(v * self.policy.translation_limit_m * 1000.0) for v in sr.actions[:3]],
                'dr_h_deg': [math.degrees(float(v * self.policy.rotation_limit_rad)) for v in sr.actions[3:6]],
                'p_target_h_mm': [float(v * 1000.0) for v in sr.p_target_h],
                'q_cmd_deg': [math.degrees(float(v)) for v in sr.q_cmd],
                'latency_ms': float(sr.latency_ms),
                'task_fraction': float(sr.task_fraction),
            }
            elapsed_s = (
                float(now - self.policy_started_at)
                if self.mode == 'policy' and self.policy_started_at is not None else 0.0
            )

            return {
                'arm': {
                    'id': self.arm_id,
                    'namespace': self.arm_namespace,
                    'can_port': self.can_port,
                    'can_label': self.can_label,
                    'online': online,
                    'joints_fresh': joints_fresh,
                    'status_fresh': status_fresh,
                    'control_fresh': control_fresh,
                    'power_state': power_state,
                    'motor_enabled': enabled_flags,
                    'gripper_enabled': bool(self.arm_status.gripper_enabled) if self.arm_status else False,
                    'driver_mode': str(self.control_state.get('mode', 'unknown')),
                    'mit_ready': bool(self.control_state.get('mit_ready', False)),
                    'joints_rad': list(self.joint_positions) if self.joint_positions else None,
                    'joints_deg': [math.degrees(v) for v in self.joint_positions] if self.joint_positions else None,
                    'velocities_rad_s': list(self.joint_velocities),
                    'efforts_nm': list(self.joint_efforts),
                    'gripper_width_m': self.gripper_width_m,
                    'gripper_width_mm': self.gripper_width_m * 1000.0 if self.gripper_width_m is not None else None,
                    'tip_pose': tip_pose_dict,
                    'hole_relative': hole_rel_dict,
                    'rates': self.control_state.get('rates', {}),
                    'driver_error': str(self.control_state.get('last_error', '')),
                },
                'gripper': {
                    'state': self.gripper_state,
                    'open_width_m': self.gripper_open_width_m,
                    'close_width_m': self.gripper_close_width_m,
                    'effort': self.gripper_effort,
                    'cmd_width_m': self.gripper_cmd_width_m,
                    'cmd_effort': self.gripper_cmd_effort,
                },
                'wrench': {
                    'sensor_s': [float(v) for v in wrench_s],
                    'tip_t': [float(v) for v in wrench_t],
                    'bias_s': [float(v) for v in self.wrench_filter.bias_s],
                    'force_norm_n': float(np.linalg.norm(wrench_t[:3])),
                    'moment_norm_nm': float(np.linalg.norm(wrench_t[3:6])),
                    'filter_tau_ms': round(self.wrench_filter.tau * 1000.0, 2),
                    'filter_tau_1khz_ms': round(self.wrench_filter.tau_1khz * 1000.0, 2),
                },
                'policy': {
                    'mode': self.mode,
                    'onnx_path': str(self.policy.onnx_path),
                    'steps': int(self.policy.steps),
                    'elapsed_s': round(elapsed_s, 2),
                    'rate_hz': self.policy_rate.hz(now),
                    'reached_goal': self.reached_goal,
                    'reached_goal_steps': self.reached_goal_steps,
                    'tip_length_m': self.tip_length_m,
                    'tip_offset_m': list(self.tip_offset_m),
                    'tip_offset_cm': [v * 100.0 for v in self.tip_offset_m],
                    'kp': list(self.policy_kp),
                    'kd': list(self.policy_kd),
                    'translation_scale_mm': self.policy.translation_limit_m * 1000.0,
                    'rotation_scale_deg': math.degrees(self.policy.rotation_limit_rad),
                    'last_step': step_dict,
                    'last_error': self.last_error,
                    'status_message': self.status_message,
                },
                'goal': self.goal.to_dict() if self.goal is not None else None,
                'history': list(self.history),
                'logs': list(self.logs),
            }


class _InsertDashboardHandler(BaseHTTPRequestHandler):
    node: InsertDashboardNode
    static_dir: Path

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get('Content-Length', '0'))
        raw = self.rfile.read(length).decode('utf-8') if length > 0 else '{}'
        data = json.loads(raw or '{}')
        if not isinstance(data, dict):
            raise ValueError('Expected JSON object body')
        return data

    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if path == '/api/status':
            self._send_json(200, self.node.status_payload())
            return
        rel_path = Path('index.html' if path in ('', '/') else path.lstrip('/'))
        if '..' in rel_path.parts or not (self.static_dir / rel_path).is_file():
            self._send_json(404, {'ok': False, 'error': f'Not found: {path}'})
            return
        file_path = self.static_dir / rel_path
        mime = {
            '.html': 'text/html; charset=utf-8',
            '.css': 'text/css; charset=utf-8',
            '.js': 'application/javascript; charset=utf-8',
        }.get(file_path.suffix, 'application/octet-stream')
        data = file_path.read_bytes()
        self.send_response(200)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self) -> None:
        path = urlsplit(self.path).path
        try:
            body = self._read_json()
            if path == '/api/power':
                res = self.node.set_power(bool(body.get('enable', True)))
            elif path == '/api/gripper':
                w, e = body.get('width_m'), body.get('effort')
                res = self.node.set_gripper(
                    str(body.get('action', 'open')),
                    width_m=None if w is None else float(w),
                    effort=None if e is None else float(e),
                )
            elif path == '/api/drag':
                res = self.node.enter_drag()
            elif path in ('/api/goal/record', '/api/record_goal'):
                tip_len = body.get('tip_length_m')
                tip_off = body.get('tip_offset_m')
                if tip_off is None and body.get('tip_offset_cm') is not None:
                    tip_off = [float(v) / 100.0 for v in body['tip_offset_cm']]
                res = self.node.record_goal(
                    tip_length_m=None if tip_len is None else float(tip_len),
                    tip_offset_m=tip_off,
                )
            elif path in ('/api/goal/clear', '/api/clear_goal'):
                res = self.node.clear_goal()
            elif path == '/api/wrench/tare':
                res = self.node.tare_wrench()
            elif path == '/api/policy/start':
                res = self.node.start_policy()
            elif path == '/api/policy/stop':
                res = self.node.stop_policy(str(body.get('mode', 'drag')))
            elif path == '/api/parameters':
                res = self.node.update_parameters(body)
            else:
                self._send_json(404, {'ok': False, 'error': f'Unknown endpoint: {path}'})
                return
            self._send_json(200, res)
        except Exception as error:
            self._send_json(400, {'ok': False, 'error': str(error)})


def create_http_server(
    node: InsertDashboardNode,
    host: str = '127.0.0.1',
    port: int = 8092,
) -> ThreadingHTTPServer:
    handler = type(
        'BoundInsertDashboardHandler',
        (_InsertDashboardHandler,),
        {'node': node, 'static_dir': web_root_path()},
    )
    return ThreadingHTTPServer((host, port), handler)


def web_main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Launch the InsertAnything ONNX Policy Dashboard for PiPER-X (CAN4 / arm_4).'
    )
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8092)
    parser.add_argument('--config', default=None)
    parser.add_argument('--arm', default='can4')
    parser.add_argument('--namespace', default='')
    parser.add_argument('--onnx', default=None)
    parser.add_argument('--tip-length', type=float, default=DEFAULT_TIP_LENGTH_M)
    clean_argv = remove_ros_args(list(argv) if argv is not None else None)
    args = parser.parse_args(clean_argv[1:] if argv is None else clean_argv)

    rclpy.init(args=list(argv) if argv is not None else None)
    node = InsertDashboardNode(
        config_file=args.config,
        arm_selector=args.arm,
        namespace_override=args.namespace,
        onnx_path=args.onnx,
        tip_length_m=args.tip_length,
    )
    server = create_http_server(node, host=args.host, port=args.port)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(
        f'InsertAnything Dashboard ({node.can_label} -> {node.arm_namespace}) '
        f'listening on http://{args.host}:{server.server_port}/'
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(web_main())
