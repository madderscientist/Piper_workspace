"""Four-arm telemetry, persisted configuration and explicit MIT debugging over ROS."""

import argparse
from collections import deque
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
import math
from pathlib import Path
import re
import secrets
import sys
import threading
import time
from typing import Any, TypedDict
from urllib.parse import urlsplit
import yaml

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from piper_x_interfaces.msg import (
    ArmStatus, CartesianCommand, GripperCommand, JointCommand, MitCommand,
)
from piper_x_interfaces.srv import CalibrateEffector, ConfigureArm, DebugMit, Enable, SetMode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from piper_x_driver.configuration import default_config_path, FleetConfig

from .control import ArmController, drain_client, TeachingController


DEFAULT_NAMESPACES = tuple(f'/arm_{i}' for i in range(1, 5))
STREAM_TOPICS = {
    'joints': 'joint_states_feedback',
    'status': 'arm_status',
    'pose': 'end_pose_stamped',
}
STALE_SECONDS = 2.0
LOGGER = logging.getLogger('state_dashboard.http')
_CAMERA_CALIB_CACHE: dict[str, Any] = {'mtime': None, 'path': None, 'data': {}}


def default_camera_calibration_path() -> Path | None:
    local = Path(__file__).resolve().parents[2] / 'camera_calibration/config/calibration.yaml'
    if local.is_file():
        return local
    try:
        share = Path(get_package_share_directory('camera_calibration')) / 'config/calibration.yaml'
        if share.is_file():
            return share
    except Exception:
        pass
    return local if local.parent.is_dir() else None


def load_camera_calibrations(calib_path: Path | None = None) -> dict[str, dict[str, Any]]:
    """Load per-arm camera intrinsic and extrinsic calibration from camera_calibration YAML."""
    path = calib_path or default_camera_calibration_path()
    if path is None or not path.is_file():
        return {}
    try:
        mtime = path.stat().st_mtime
    except OSError:
        return {}
    if _CAMERA_CALIB_CACHE['path'] == str(path) and _CAMERA_CALIB_CACHE['mtime'] == mtime:
        return _CAMERA_CALIB_CACHE['data']
    try:
        raw = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except Exception:
        return {}
    intrinsics = raw.get('intrinsics') or {}
    extrinsics = raw.get('extrinsics') or {}
    by_arm: dict[str, dict[str, Any]] = {}
    for cam_name, ext in extrinsics.items():
        if not isinstance(ext, dict):
            continue
        arm_key = str(ext.get('arm') or '').strip().lstrip('/')
        if not arm_key and cam_name.startswith('camera_arm_'):
            arm_key = cam_name[len('camera_'):]
        if not arm_key:
            continue
        intr_list = intrinsics.get(cam_name) or []
        by_arm[f'/{arm_key}'] = {
            'camera': cam_name,
            'parent': ext.get('parent', 'gripper_base_top'),
            'reference_point': ext.get('reference_point', 'gripper_base_top'),
            'translation': [float(v) for v in (ext.get('translation') or [0.0, 0.0, 0.0])[:3]],
            'rotation': [float(v) for v in (ext.get('rotation') or [0.0, 0.0, 0.0, 1.0])[:4]],
            'rpy': [float(v) for v in (ext.get('rpy') or [0.0, 0.0, 0.0])[:3]],
            'mesh_origins': ext.get('mesh_origins') or {},
            'method': ext.get('method', ''),
            'samples': ext.get('samples', 0),
            'residual_mm': ext.get('residual_mm'),
            'cross_check': ext.get('cross_check'),
            'board_pose': ext.get('board_pose'),
            'stamp': ext.get('stamp', ''),
            'intrinsics': [
                {
                    'width': int(item['width']),
                    'height': int(item['height']),
                    'camera_matrix': [float(v) for v in item['camera_matrix']],
                    'distortion_coefficients': [float(v) for v in item['distortion_coefficients']],
                    'rms': item.get('rms'),
                    'fov_deg': item.get('fov_deg'),
                    'board': item.get('board'),
                    'stamp': item.get('stamp', ''),
                }
                for item in intr_list if isinstance(item, dict) and 'camera_matrix' in item
            ],
        }
    _CAMERA_CALIB_CACHE.update({'path': str(path), 'mtime': mtime, 'data': by_arm})
    return by_arm


def _reject_nonfinite(constant: str) -> Any:
    raise ValueError(f'Invalid JSON constant: {constant}')


def normalize_namespaces(values: list[str]) -> list[str]:
    """Validate four distinct, non-root ROS namespaces."""
    namespaces = ['/' + value.strip('/') for value in values]
    if len(namespaces) != 4 or len(set(namespaces)) != 4:
        raise ValueError('Exactly four distinct arm namespaces are required.')
    for namespace in namespaces:
        if not re.fullmatch(r'(?:/[A-Za-z_][A-Za-z0-9_]*)+', namespace):
            raise ValueError(f'Invalid arm namespace: {namespace}')
    return namespaces


def finite(value: float) -> float:
    """Reject invalid numbers rather than emitting non-standard JSON."""
    result = float(value)
    if not math.isfinite(result):
        raise ValueError('Telemetry contains a non-finite number.')
    return result


def joint_payload(message: JointState) -> dict[str, Any]:
    """Extract joint feedback by name, including optional trailing gripper data."""
    names = list(message.name)
    if len(set(names)) != len(names):
        raise ValueError('JointState contains duplicate joint names.')
    arrays = (message.position, message.velocity, message.effort)
    if any(len(values) > len(names) for values in arrays):
        raise ValueError('JointState has more values than joint names.')
    for values in arrays:
        for value in values:
            finite(value)
    indices = {}
    gripper_index = None
    for index, name in enumerate(names):
        short_name = name.rsplit('/', 1)[-1]
        match = re.fullmatch(r'joint_?([1-6])', short_name)
        if match:
            joint = int(match.group(1)) - 1
            if joint in indices:
                raise ValueError('Multiple names identify the same Piper joint.')
            indices[joint] = index
        elif short_name in ('gripper', 'joint7', 'joint_7'):
            gripper_index = index
    if not indices:
        raise ValueError('No Piper joint1..joint6 names found in JointState.')

    def values_for(
        values: Sequence[float], degrees: bool = False,
    ) -> list[float | None]:
        result = []
        for joint in range(6):
            index = indices.get(joint)
            value = finite(values[index]) if index is not None and index < len(values) else None
            result.append(math.degrees(value) if degrees and value is not None else value)
        return result

    gripper = None
    if gripper_index is not None and gripper_index < len(message.position):
        gripper = finite(message.position[gripper_index])
    return {
        'position_deg': values_for(message.position, degrees=True),
        'velocity_rad_s': values_for(message.velocity),
        'effort_nm': values_for(message.effort),
        'gripper_m': gripper,
    }


def status_payload(message: ArmStatus) -> dict[str, Any]:
    """Preserve raw protocol codes and explicit per-joint error flags."""
    result: dict[str, Any] = {
        key: int(getattr(message, key))
        for key in (
            'ctrl_mode', 'arm_status', 'mode_feedback', 'teach_status',
            'motion_status', 'trajectory_num', 'err_code',
        )
    }
    result['angle_limits'] = [
        bool(getattr(message, f'joint_{i}_angle_limit')) for i in range(1, 7)
    ]
    result['communication_errors'] = [
        bool(getattr(message, f'communication_status_joint_{i}'))
        for i in range(1, 7)
    ]
    enabled = list(message.motor_enabled)
    valid = list(message.motor_enable_valid)
    result['motor_enabled'] = enabled
    result['motor_enable_valid'] = valid
    result['gripper_enabled'] = message.gripper_enabled
    result['gripper_valid'] = message.gripper_valid
    result['teaching_active'] = message.teaching_active
    result['feedback_age'] = [finite(age) for age in message.feedback_age]
    result['last_error'] = str(message.last_error)
    result['power_state'] = (
        'unknown' if not all(valid) else
        'enabled' if all(enabled) else
        'mixed' if any(enabled) else 'disabled'
    )
    return result


def pose_payload(message: PoseStamped) -> dict[str, Any]:
    """Convert ROS metres and quaternion orientation into display units."""
    pose = message.pose
    position = [finite(value) for value in (pose.position.x, pose.position.y, pose.position.z)]
    quaternion = [
        finite(value) for value in
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    ]
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-12:
        raise ValueError('End pose has a zero-length quaternion.')
    x, y, z, w = [value / norm for value in quaternion]
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return {
        'position_mm': [value * 1000 for value in position],
        'rpy_deg': [math.degrees(value) for value in (roll, pitch, yaw)],
        'frame_id': message.header.frame_id,
    }


@dataclass
class StreamState:
    """A last valid sample and independent freshness information."""

    payload: dict[str, Any] | None = None
    received_at: float | None = None
    count: int = 0
    error: str | None = None
    samples: deque[float] = field(default_factory=lambda: deque(maxlen=2048))


class StreamSnapshot(TypedDict):
    """Public per-topic freshness metadata."""

    topic: str
    age_ms: int | None
    fresh: bool
    hz: float
    messages: int
    error: str | None


class TelemetryStore:
    """Synchronize ROS callbacks and HTTP readers without exposing mutable state."""

    def __init__(
        self, namespaces: list[str], clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.namespaces = normalize_namespaces(namespaces)
        self._clock = clock
        self._lock = threading.Lock()
        self._arms = [
            {name: StreamState() for name in STREAM_TOPICS} for _ in namespaces
        ]
        self._controls = [StreamState() for _ in namespaces]
        self._teaching = StreamState()
        self._logs: deque[dict[str, Any]] = deque(maxlen=60)

    def _append_log(self, level: str, node_name: str, text: str) -> None:
        self._logs.append({
            'time': time.strftime('%H:%M:%S'),
            'level': level,
            'node': node_name,
            'msg': text,
        })

    def add_log(self, level: str, node_name: str, text: str) -> None:
        with self._lock:
            self._append_log(level, node_name, text)

    def update_teaching(self, payload: dict[str, Any]) -> None:
        with self._lock:
            self._teaching.payload = payload
            self._teaching.received_at = self._clock()

    def update_control(self, index: int, payload: dict[str, Any]) -> None:
        with self._lock:
            prev_err = (
                self._controls[index].payload.get('last_error')
                if self._controls[index].payload else None
            )
            new_err = payload.get('last_error')
            self._controls[index].payload = payload
            self._controls[index].received_at = self._clock()
            if new_err and new_err != prev_err:
                self._append_log('ERROR', f'{self.namespaces[index]}.driver', str(new_err))

    def reject_control(self, index: int, error: str) -> None:
        with self._lock:
            self._controls[index].error = error
            self._controls[index].received_at = None

    def update(self, index: int, stream: str, payload: dict[str, Any]) -> None:
        with self._lock:
            now = self._clock()
            state = self._arms[index][stream]
            state.payload = payload
            state.received_at = now
            state.count += 1
            state.error = None
            state.samples.append(now)

    def reject(self, index: int, stream: str, error: str) -> bool:
        """Record a visible error and report whether its message changed."""
        with self._lock:
            state = self._arms[index][stream]
            changed = state.error != error
            state.error = error
            return changed

    def enable_block_reason(self, index: int) -> str | None:
        """Require fresh, fault-free feedback before accepting an enable request."""
        now = self._clock()
        with self._lock:
            states = self._arms[index]
            if any(
                state.received_at is None or now - state.received_at > STALE_SECONDS
                or state.error is not None for state in states.values()
            ):
                return '遥测缺失、过期或无效，不能使能。'
            status = states['status'].payload
            if status is None or status.get('arm_status') != 0 or status.get('err_code') != 0:
                return '机械臂状态异常，请先检查急停和故障。'
            if any(status.get('angle_limits', [])) or any(status.get('communication_errors', [])):
                return '关节限位或通信异常，不能使能。'
            if status.get('power_state') != 'disabled':
                return '仅在六个关节均确认失能时允许使能；其他状态请先失能。'
        return None

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            now = self._clock()
            arms = []
            for index, states in enumerate(self._arms):
                streams: dict[str, StreamSnapshot] = {}
                for name, state in states.items():
                    age = None if state.received_at is None else max(0, now - state.received_at)
                    while state.samples and now - state.samples[0] > STALE_SECONDS:
                        state.samples.popleft()
                    samples = state.samples
                    span = samples[-1] - samples[0] if len(samples) > 1 else 0
                    streams[name] = {
                        'topic': self.namespaces[index] + '/' + STREAM_TOPICS[name],
                        'age_ms': None if age is None else round(age * 1000),
                        'fresh': age is not None and age <= STALE_SECONDS and state.error is None,
                        'hz': round((len(samples) - 1) / span, 1) if span > 0 else 0.0,
                        'messages': state.count,
                        'error': state.error,
                    }
                fresh = sum(stream['fresh'] for stream in streams.values())
                received = any(state.received_at is not None for state in states.values())
                if fresh == len(STREAM_TOPICS):
                    connection = 'live'
                elif fresh:
                    connection = 'partial'
                elif received:
                    connection = 'stale'
                else:
                    connection = 'waiting'
                control = self._controls[index]
                control_at = control.received_at
                cam_calibs = load_camera_calibrations()
                arms.append({
                    'id': index + 1,
                    'namespace': self.namespaces[index],
                    'connection': connection,
                    'streams': streams,
                    **{name: state.payload for name, state in states.items()},
                    'control': control.payload,
                    'control_fresh': control_at is not None and 0 <= now - control_at <= 0.3,
                    'camera_calibration': cam_calibs.get(self.namespaces[index]),
                })
            teaching_at = self._teaching.received_at
            # JSON round-tripping also prevents callbacks from mutating a response.
            return json.loads(json.dumps({
                'arms': arms,
                'teaching': self._teaching.payload,
                'teaching_fresh': (
                    teaching_at is not None and 0 <= now - teaching_at <= STALE_SECONDS
                ),
                'logs': list(self._logs),
                'server_time': time.time(),
                'stale_after_ms': int(STALE_SECONDS * 1000),
                'read_only': True,
            }, allow_nan=False))


@dataclass
class EnableState:
    """Last command outcome, not a claim about ongoing motor enable feedback."""

    pending: bool = False
    outcome: str = 'idle'
    message: str = '尚未通过页面发送请求。'
    requested_at: float | None = None


class EnableController:
    """Dispatch explicit requests once and wait for the driver's actual response."""

    def __init__(
        self, store: TelemetryStore, clients: Sequence[Any], timeout: float = 10.0,
    ) -> None:
        if len(clients) != len(store.namespaces):
            raise ValueError('One enable client per arm is required.')
        self.store = store
        self.clients = clients
        self.timeout = timeout
        self._lock = threading.Lock()
        self._states = [
            {'enable': EnableState(), 'disable': EnableState()} for _ in clients
        ]
        self._ready_cache: list[bool] = [False] * len(clients)
        self._ready_checked_at: float = -1.0

    def snapshot(self, enable: bool = True) -> list[dict[str, object]]:
        action = 'enable' if enable else 'disable'
        with self._lock:
            now = time.monotonic()
            if (self._ready_checked_at < 0.0
                    or now - self._ready_checked_at > 0.25
                    or not all(self._ready_cache)):
                self._ready_cache = [client.service_is_ready() for client in self.clients]
                self._ready_checked_at = now
            return [
                {
                    'service': namespace + '/enable_srv',
                    'available': ready,
                    'blocked_reason': self.store.enable_block_reason(index) if enable else None,
                    'pending': state.pending,
                    'outcome': state.outcome,
                    'message': state.message,
                    'requested_at': state.requested_at,
                }
                for index, (namespace, ready, states) in enumerate(
                    zip(self.store.namespaces, self._ready_cache, self._states))
                for state in [states[action]]
            ]

    def _error(self, index: int, status: int, message: str) -> tuple[int, dict[str, object]]:
        LOGGER.warning('%s: %s', self.store.namespaces[index], message)
        return status, {'success': False, 'arm_id': index + 1, 'message': message}

    def enable(self, index: int) -> tuple[int, dict[str, object]]:
        """Enable only with fresh, fault-free feedback and no pending power request."""
        return self._request(index, enable=True)

    def disable(self, index: int) -> tuple[int, dict[str, object]]:
        """Permit disabling with stale/fault feedback, even behind pending enabling."""
        return self._request(index, enable=False)

    def _request(self, index: int, enable: bool) -> tuple[int, dict[str, object]]:
        action = 'enable' if enable else 'disable'
        label = '使能' if enable else '失能'
        done = threading.Event()
        with self._lock:
            states = self._states[index]
            if states[action].pending or (enable and states['disable'].pending):
                return self._error(index, 409, '该机械臂已有相关请求未完成，请勿重复提交。')
            reason = self.store.enable_block_reason(index) if enable else None
            if reason:
                return self._error(index, 409, reason)
            client = self.clients[index]
            if not client.service_is_ready():
                return self._error(index, 503, '使能/失能服务不可用，请检查对应 ROS 驱动。')
            state = EnableState(
                pending=True, outcome='pending',
                message=f'正在等待驱动确认{label}……', requested_at=time.time())
            states[action] = state
            try:
                future = client.call_async(Enable.Request(enable_request=enable))
            except RuntimeError as error:
                state.pending = False
                state.outcome = 'failed'
                state.message = f'ROS 请求发送失败：{error}'
                return self._error(index, 503, state.message)
        LOGGER.info('Explicit %s requested: %s/enable_srv', action, self.store.namespaces[index])
        future.add_done_callback(partial(self._finish, index, state, label, done))
        done.wait(self.timeout)
        with self._lock:
            if state.pending:
                state.outcome = 'unknown'
                state.message = f'{label}请求超时，结果未知；驱动可能仍在执行，请勿重复操作。'
                return self._error(index, 504, state.message)
            success = state.outcome == 'succeeded'
            return (200 if success else 502), {
                'success': success, 'arm_id': index + 1, 'action': action,
                'outcome': state.outcome, 'message': state.message,
            }

    def _finish(
        self, index: int, state: EnableState, label: str,
        done: threading.Event, future: Future,
    ) -> None:
        response = None if future.cancelled() or future.exception() else future.result()
        with self._lock:
            state.pending = False
            if response is not None and response.enable_response:
                state.outcome = 'succeeded'
                state.message = f'驱动已确认本次{label}成功；这不是持续的电机使能状态反馈。'
                LOGGER.info('Driver confirmed %s: %s', label, self.store.namespaces[index])
            else:
                state.outcome = 'failed'
                state.message = f'驱动未确认{label}成功，请检查设备、急停和驱动日志。'
                if not future.cancelled() and future.exception():
                    state.message = f'ROS 服务调用失败：{future.exception()}'
                elif response is not None and response.message:
                    state.message = f'驱动拒绝{label}：{response.message}'
                LOGGER.warning('%s: %s', self.store.namespaces[index], state.message)
        done.set()


class DashboardNode(Node):
    """Subscribe to feedback and call per-arm driver services without opening CAN."""

    def __init__(self, namespaces: list[str]) -> None:
        super().__init__('state_dashboard')
        self.store = TelemetryStore(namespaces)
        message_types = {'joints': JointState, 'status': ArmStatus, 'pose': PoseStamped}
        feedback_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=25,
        )
        self._feedback_subscriptions = [
            self.create_subscription(
                message_types[stream], namespace + '/' + topic,
                partial(self._receive, index, stream), feedback_qos,
            )
            for index, namespace in enumerate(self.store.namespaces)
            for stream, topic in STREAM_TOPICS.items()
        ]
        self.enable_controller = EnableController(self.store, [
            self.create_client(Enable, namespace + '/enable_srv')
            for namespace in self.store.namespaces
        ])
        self._command_pubs: dict[tuple[int, str], Any] = {}
        self.arm_controller = ArmController(
            [self.create_client(ConfigureArm, ns + '/configure_arm') for ns in namespaces],
            [self.create_client(DebugMit, ns + '/debug_mit') for ns in namespaces],
            [self.create_client(CalibrateEffector, ns + '/calibrate_effector')
             for ns in namespaces],
            [self.create_client(SetMode, ns + '/set_mode') for ns in namespaces],
            self._get_command_publisher,
            log_callback=self.store.add_log,
        )
        self.teaching_controller = TeachingController(
            self.create_client(Trigger, '/bilateral_teaching/start'),
            self.create_client(Trigger, '/bilateral_teaching/stop'),
            self.create_client(SetMode, '/bilateral_teaching/set_mode'),
        )
        self._control_subscriptions = [
            self.create_subscription(
                String, namespace + '/control_state', partial(self._control_state, index), 10)
            for index, namespace in enumerate(namespaces)
        ]
        self._teaching_subscription = self.create_subscription(
            String, '/bilateral_teaching/state', self._teaching_state, 10)

    def _get_command_publisher(self, index: int, kind: str) -> Any:
        key = (index, kind)
        if key not in self._command_pubs:
            spec = {
                'joints': (JointCommand, '/command/joints'),
                'cartesian': (CartesianCommand, '/command/cartesian'),
                'gripper': (GripperCommand, '/command/gripper'),
                'mit': (MitCommand, '/command/mit'),
            }[kind]
            pub = self.create_publisher(
                spec[0], self.store.namespaces[index] + spec[1], 10)
            deadline = time.monotonic() + 0.15
            while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
                time.sleep(0.005)
            self._command_pubs[key] = pub
        return self._command_pubs[key]

    def _teaching_state(self, message: String) -> None:
        try:
            payload = json.loads(message.data, parse_constant=_reject_nonfinite)
            if not isinstance(payload, dict):
                raise ValueError('Invalid teaching state')
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        self.store.update_teaching(payload)

    def _control_state(self, index: int, message: String) -> None:
        try:
            payload = json.loads(message.data, parse_constant=_reject_nonfinite)
            if (not isinstance(payload, dict) or not isinstance(payload.get('configuration'), dict)
                    or not isinstance(payload.get('debug_active'), bool)):
                raise ValueError('Invalid driver control state')
        except ValueError as error:
            self.store.reject_control(index, str(error))
            self.get_logger().error(str(error))
            return
        self.store.update_control(index, payload)

    def _receive(
        self, index: int, stream: str, message: JointState | ArmStatus | PoseStamped,
    ) -> None:
        try:
            if isinstance(message, JointState):
                payload = joint_payload(message)
            elif isinstance(message, ArmStatus):
                payload = status_payload(message)
            elif isinstance(message, PoseStamped):
                payload = pose_payload(message)
            else:
                raise TypeError('Unexpected telemetry message type.')
        except ValueError as error:
            if self.store.reject(index, stream, str(error)):
                self.get_logger().warning(f'{self.store.namespaces[index]} / {stream}: {error}')
            return
        self.store.update(index, stream, payload)


class DashboardServer(ThreadingHTTPServer):
    """Serve local assets, telemetry and token-protected per-arm control routes."""

    daemon_threads = True

    def __init__(
        self, address: tuple[str, int], store: TelemetryStore, assets: Path,
        enable_controller: EnableController | None = None,
        arm_controller: ArmController | None = None,
        teaching_controller: TeachingController | None = None,
        meshes: Path | None = None,
    ) -> None:
        self.store = store
        self.assets = assets
        self.enable_controller = enable_controller
        self.arm_controller = arm_controller
        self.teaching_controller = teaching_controller
        if meshes is None:
            try:
                meshes = Path(get_package_share_directory('piper_description')) / 'meshes'
            except Exception:
                meshes = None
        self.meshes = meshes
        self.control_token = secrets.token_urlsafe(32)
        for filename in ('index.html', 'app.js', 'controls.js', 'styles.css'):
            if not (assets / filename).is_file():
                raise FileNotFoundError(f'Dashboard asset not installed: {filename}')
        super().__init__(address, DashboardHandler)

    def serve_forever(self, poll_interval: float = 0.05) -> None:
        super().serve_forever(poll_interval=poll_interval)


class DashboardHandler(BaseHTTPRequestHandler):
    """Constrain writes to explicit, same-origin per-arm control requests."""

    server: DashboardServer

    def do_GET(self) -> None:
        self._respond(head=False)

    def do_HEAD(self) -> None:
        self._respond(head=True)

    def do_POST(self) -> None:
        arm_match = re.fullmatch(
            r'/api/arms/([1-4])/(enable|disable|configure|calibrate|mode|joints|cartesian|gripper'
            r'|mit/(?:start|target|stop|command))',
            self.path)
        teaching_match = re.fullmatch(r'/api/teaching/(start|stop|mode)', self.path)
        if arm_match is None and teaching_match is None:
            self._json_response(404, {'message': '未知操作。'})
            return
        if teaching_match is not None:
            if self.server.teaching_controller is None:
                self._json_response(403, {'message': '当前服务不支持示教控制。'})
                return
            power = False
        else:
            assert arm_match is not None
            action = arm_match.group(2)
            power = action in ('enable', 'disable')
            if ((power and self.server.enable_controller is None)
                    or (not power and self.server.arm_controller is None)):
                self._json_response(403, {'message': '当前服务仅支持监控。'})
                return
        token = self.headers.get('X-Piper-Control-Token', '')
        origin = self.headers.get('Origin')
        try:
            parsed_origin = None if origin is None else urlsplit(origin)
        except ValueError:
            self._json_response(403, {'message': '无效的请求来源。'})
            return
        if (
            not secrets.compare_digest(token.encode('utf-8'), self.server.control_token.encode())
            or self.headers.get('Sec-Fetch-Site') == 'cross-site'
            or (parsed_origin is not None and (
                parsed_origin.scheme not in ('http', 'https')
                or parsed_origin.netloc != self.headers.get('Host')
            ))
        ):
            self._json_response(403, {'message': '控制请求校验失败，请刷新本地页面。'})
            return
        if (
            self.headers.get_content_type() != 'application/json'
            or self.headers.get('Transfer-Encoding') is not None
        ):
            self._json_response(400, {'message': '请求必须是小型 JSON 对象。'})
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
        except ValueError:
            self._json_response(400, {'message': '无效的请求长度。'})
            return
        if not 1 <= length <= (128 if power else 1024):
            self._json_response(400, {'message': '无效的请求长度。'})
            return
        self.connection.settimeout(3)
        try:
            body = json.loads(self.rfile.read(length))
        except (ValueError, TimeoutError) as error:
            LOGGER.warning('Invalid arm power request body: %s', error)
            self._json_response(400, {'message': '无效的 JSON 请求。'})
            return
        if power and body != {}:
            self._json_response(400, {'message': '使能/失能请求不接受额外参数。'})
            return
        if teaching_match is not None:
            assert self.server.teaching_controller is not None
            status, result = self.server.teaching_controller.request(
                teaching_match.group(1), body)
        elif power:
            assert arm_match is not None
            controller = self.server.enable_controller
            assert controller is not None
            operation = controller.enable if action == 'enable' else controller.disable
            status, result = operation(int(arm_match.group(1)) - 1)
        else:
            assert arm_match is not None and self.server.arm_controller is not None
            status, result = self.server.arm_controller.request(
                int(arm_match.group(1)) - 1, action, body)
        self._json_response(status, result)

    def _json_response(self, status: int, payload: dict[str, object]) -> None:
        self._send(
            status, json.dumps(payload, allow_nan=False).encode('utf-8'),
            'application/json; charset=utf-8', head=False)

    def _respond(self, head: bool) -> None:
        path = urlsplit(self.path).path
        assets = {
            '/': ('index.html', 'text/html; charset=utf-8'),
            '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
            '/controls.js': ('controls.js', 'text/javascript; charset=utf-8'),
            '/mesh.js': ('mesh.js', 'text/javascript; charset=utf-8'),
            '/styles.css': ('styles.css', 'text/css; charset=utf-8'),
        }
        mesh_match = re.fullmatch(r'/meshes/([A-Za-z0-9_]+\.(?:STL|stl))', path)
        if path == '/api/status':
            snapshot = self.server.store.snapshot()
            if (self.server.enable_controller is not None
                    or self.server.arm_controller is not None
                    or self.server.teaching_controller is not None):
                snapshot['read_only'] = False
                snapshot['control_token'] = self.server.control_token
            if self.server.enable_controller is not None:
                for arm, enable, disable in zip(
                    snapshot['arms'], self.server.enable_controller.snapshot(),
                    self.server.enable_controller.snapshot(enable=False),
                ):
                    arm['enable'] = enable
                    arm['disable'] = disable
            body = json.dumps(snapshot, allow_nan=False).encode('utf-8')
            content_type = 'application/json; charset=utf-8'
        elif path in assets and (self.server.assets / assets[path][0]).is_file():
            filename, content_type = assets[path]
            body = (self.server.assets / filename).read_bytes()
        elif (mesh_match is not None and self.server.meshes is not None
              and (self.server.meshes / mesh_match.group(1)).is_file()):
            body = (self.server.meshes / mesh_match.group(1)).read_bytes()
            content_type = 'application/octet-stream'
        else:
            self.send_error(404, 'Not found')
            return
        self._send(200, body, content_type, head)

    def _send(self, status: int, body: bytes, content_type: str, head: bool) -> None:
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header(
            'Content-Security-Policy',
            "default-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; "
            "object-src 'none'; frame-ancestors 'none'; base-uri 'none'",
        )
        self.end_headers()
        if not head:
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                LOGGER.debug('Dashboard client disconnected during a response.')

    def log_request(self, code: int | str = '-', size: int | str = '-') -> None:
        if isinstance(code, int) and code >= 400:
            super().log_request(code, size)


def main(args: list[str] | None = None) -> None:
    """Run the dashboard without starting drivers or opening CAN interfaces."""
    argv = sys.argv if args is None else ['state_dashboard', *args]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8089)
    parser.add_argument('--config', default=str(default_config_path()))
    parser.add_argument('--namespaces', nargs=4)
    options = parser.parse_args(remove_ros_args(argv)[1:])
    try:
        fleet = FleetConfig(options.config)
        configured_namespaces = [arm.namespace for arm in fleet.arms]
        namespaces = normalize_namespaces(options.namespaces or configured_namespaces)
        if namespaces != configured_namespaces:
            raise ValueError('--namespaces must match the fleet configuration')
    except (ValueError, OSError, yaml.YAMLError) as error:
        parser.error(str(error))
    if not 1 <= options.port <= 65535:
        parser.error('--port must be between 1 and 65535.')
    rclpy.init(args=argv)
    node = DashboardNode(namespaces)
    try:
        local_assets = Path(__file__).resolve().parent.parent / 'dashboard'
        assets = (
            local_assets if local_assets.is_dir()
            else Path(get_package_share_directory('state_dashboard')) / 'dashboard'
        )
        with DashboardServer(
            (options.host, options.port), node.store, assets, node.enable_controller,
            node.arm_controller, node.teaching_controller,
        ) as server:
            worker = threading.Thread(target=server.serve_forever, daemon=True)
            worker.start()
            node.get_logger().info(
                f'Dashboard with explicit configuration/MIT controls: '
                f'http://{options.host}:{server.server_port}')
            try:
                subs = list(node.subscriptions)
                clients = list(node.clients)
                while rclpy.ok():
                    for sub in subs:
                        while True:
                            with sub.handle:
                                msg_info = sub.handle.take_message(sub.msg_type, sub.raw)
                            if msg_info is None:
                                break
                            sub.callback(msg_info[0])
                    for client in clients:
                        if client._pending_requests:
                            drain_client(client)
                    time.sleep(0.005)
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            finally:
                server.shutdown()
                worker.join(timeout=5)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
