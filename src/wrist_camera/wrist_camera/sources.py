"""单臂标定数据源：纯 Python RTSP 腕部相机直连取图与切档位，以及机械臂静止判据与正运动学。"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

import numpy as np

from wrist_camera import transforms
from wrist_camera.rtsp import RtspReader, ffmpeg_command, fit_size, probe_size
from wrist_camera.transforms import (
    GRIPPER_BASE_WITHOUT_FLANGE_TOP_Z_M,
    GRIPPER_BASE_WITH_FLANGE_TOP_Z_M,
    GRIPPER_FLANGE_THICKNESS_M,
    GRIPPER_WITH_WRENCH_TOP_Z_M,
    load_arm_angle_offsets,
    piper_fk,
)

ALLOWED_ARMS = ('arm_3', 'arm_4')
_ARM_ALIASES = {
    'arm_3': 'arm_3', 'arm3': 'arm_3', '/arm_3': 'arm_3', '3': 'arm_3',
    'arm_4': 'arm_4', 'arm4': 'arm_4', '/arm_4': 'arm_4', '4': 'arm_4',
}
_JOINT_NAMES_6 = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

__all__ = [
    'ALLOWED_ARMS',
    'CameraSource',
    'Frames',
    'GRIPPER_BASE_WITHOUT_FLANGE_TOP_Z_M',
    'GRIPPER_BASE_WITH_FLANGE_TOP_Z_M',
    'GRIPPER_FLANGE_THICKNESS_M',
    'GRIPPER_WITH_WRENCH_TOP_Z_M',
    'MotionGate',
    'load_arm_angle_offsets',
    'normalize_arm_id',
    'piper_fk',
    'select_arm_camera',
]


def normalize_arm_id(arm: str) -> str:
    raw = str(arm or '').strip().lower()
    if not raw:
        raise ValueError('必须指定机械臂（仅支持 arm_3 或 arm_4）')
    canonical = _ARM_ALIASES.get(raw)
    if canonical is None:
        raise ValueError(f'只有 arm_3 和 arm_4 有腕部相机，不支持 {arm!r}')
    return canonical


def _match_camera(cameras: dict, selector: str) -> tuple[str | None, dict | None]:
    low = selector.strip().lower().strip('/')
    for name, cfg in cameras.items():
        aliases = [str(a).lower().strip('/') for a in (cfg.get('aliases') or [])]
        candidates = {
            name.lower(),
            str(cfg.get('arm', '')).lower().strip('/'),
            str(cfg.get('frame', '')).lower(),
            str(cfg.get('ip', '')).lower(),
            *aliases,
        }
        if low in candidates:
            return name, dict(cfg)
    return None, None


def select_arm_camera(cameras_config: dict, arm: str = '', camera: str = '') -> tuple[str, str, dict]:
    """解析并校验单臂与对应腕部相机（支持只传 arm、只传 camera、或同时传两者并校验匹配）。"""
    arm_raw = str(arm or '').strip()
    cam_raw = str(camera or '').strip()
    if not arm_raw and not cam_raw:
        raise ValueError('启动标定需要指定 --arm（arm_3 或 arm_4）或 --camera')
    cameras = (cameras_config or {}).get('cameras', {}) or {}
    if not cameras:
        raise ValueError('cameras.yaml 中没有配置任何相机')

    if arm_raw and not cam_raw:
        arm_id = normalize_arm_id(arm_raw)
        matched_name, matched_cfg = _match_camera(cameras, arm_id)
        if matched_name is None or matched_cfg is None:
            raise ValueError(f'未找到 {arm_id} 对应的腕部相机配置')
    elif cam_raw and not arm_raw:
        matched_name, matched_cfg = _match_camera(cameras, cam_raw)
        if matched_name is None or matched_cfg is None:
            raise ValueError(f'未找到相机 {camera!r}，可选相机：{list(cameras)}')
        arm_id = normalize_arm_id(matched_cfg.get('arm', ''))
    else:
        arm_id = normalize_arm_id(arm_raw)
        matched_name, matched_cfg = _match_camera(cameras, cam_raw)
        if matched_name is None or matched_cfg is None:
            raise ValueError(f'未找到相机 {camera!r}，可选相机：{list(cameras)}')
        cfg_arm = matched_cfg.get('arm')
        if cfg_arm and normalize_arm_id(cfg_arm) != arm_id:
            raise ValueError(
                f'指定的机械臂 {arm_id} 与相机 {matched_name}（归属 {cfg_arm}）不匹配')

    matched_cfg.setdefault('arm', arm_id)
    matched_cfg.setdefault('arm_namespace', f'/{arm_id}')
    return arm_id, matched_name, matched_cfg


def _known_native_size(url: str) -> tuple[int, int] | None:
    tail = url.rstrip('/').rsplit('/', 1)[-1].lower()
    if tail == 'stream1':
        return (720, 480)
    if tail == 'stream0':
        return (1920, 1080)
    return None


class CameraSource:
    """一路腕部相机：直接拉取 RTSP 码流缓存最新一帧，并支持在线切换分辨率档位。"""

    def __init__(
        self,
        name: str,
        config: dict,
        *,
        auto_start: bool = True,
        poll_period_s: float = 1.0,
        stale_timeout_s: float = 5.0,
        reader_cls: Any = RtspReader,
        probe_fn=probe_size,
    ) -> None:
        self.name = name
        self.config = config
        self.label = config.get('label', name)
        self.role = config.get('role', 'target')
        self.frame = config.get('frame', name)
        self.parent_frame = config.get('parent_frame', 'gripper_base_top')
        self.ip = config.get('ip', '')
        self.arm = config.get('arm', '')
        self.profiles = [dict(p) for p in (config.get('profiles') or [])]
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._frame = None                   # (seq, bgr, stamp)
        self._seq = 0
        self._poll_period = poll_period_s
        self._stale_timeout = stale_timeout_s
        self._reader_cls = reader_cls
        self._probe_fn = probe_fn
        self._reader: RtspReader | None = None
        self._applied: tuple[str, tuple[int, int]] | None = None
        self._error: str | None = None
        self._wake = threading.Event()
        self._closed = threading.Event()
        self._supervisor: threading.Thread | None = None

        default_profile = self.profiles[0] if self.profiles else None
        if default_profile is None and (config.get('url') or self.ip):
            default_profile = {
                'width': int(config.get('width') or 0),
                'height': int(config.get('height') or 0),
                'url': str(config.get('url') or f'rtsp://admin:123456@{self.ip}/stream1'),
            }
        self._target_profile: dict | None = dict(default_profile) if default_profile else None
        if auto_start and self._target_profile and self._target_profile.get('url'):
            self._start_supervisor()

    @property
    def switchable(self) -> bool:
        return len(self.profiles) > 1

    def _start_supervisor(self) -> None:
        if self._supervisor is None and not self._closed.is_set():
            self._supervisor = threading.Thread(
                target=self._supervise, daemon=True, name=f'rtsp-{self.name}')
            self._supervisor.start()

    def _supervise(self) -> None:
        while not self._closed.is_set():
            self._wake.clear()
            try:
                self._step()
            except Exception as exc:
                self._error = str(exc)
            if self._closed.is_set():
                break
            self._wake.wait(self._poll_period)

    def _step(self) -> None:
        with self._lock:
            target = dict(self._target_profile) if self._target_profile else None
        if target is None or not target.get('url'):
            self._close_reader()
            return
        url = str(target['url']).strip()
        desired_size = (int(target.get('width') or 0), int(target.get('height') or 0))
        reader = self._reader
        if reader is None:
            self._open_reader(url, desired_size)
        elif not reader.alive() or reader.frame_age() > self._stale_timeout:
            self._close_reader()
            self._open_reader(url, desired_size)
        elif (
            self._applied is None
            or self._applied[0] != url
            or (desired_size != (0, 0) and self._applied[1] != desired_size)
        ):
            self._close_reader()
            self._open_reader(url, desired_size)

    def _open_reader(self, url: str, desired_size: tuple[int, int]) -> None:
        if self._closed.is_set():
            return
        target_w, target_h = desired_size
        native = _known_native_size(url)
        if target_w > 0 and target_h > 0:
            size = (target_w, target_h)
            scale = None if native in (None, size) else size
        else:
            if native is None:
                native = self._probe_fn(url)
            if native is None:
                self._error = f'ffprobe 读不到分辨率：{url}'
                return
            size = fit_size(native, target_w, target_h)
            scale = None if size == native else size
        try:
            reader = self._reader_cls(
                ffmpeg_command(url, scale=scale), size[0], size[1], self._on_raw_frame)
        except OSError as exc:
            self._error = f'启动 ffmpeg 失败：{exc}'
            return
        if self._closed.is_set():
            reader.stop()
            return
        self._reader = reader
        self._applied = (url, size)

    def _close_reader(self) -> None:
        reader = self._reader
        self._reader = None
        self._applied = None
        if reader is not None:
            reader.stop()

    def _on_raw_frame(self, raw: bytes, width: int, height: int) -> None:
        try:
            image = np.frombuffer(raw, np.uint8).reshape(height, width, 3)
        except ValueError:
            return
        self._error = None
        self.push_bgr(image, copy=False)

    def push_bgr(self, image: np.ndarray, *, copy: bool = False) -> None:
        with self._cond:
            self._seq += 1
            self._frame = (self._seq, image.copy() if copy else image, time.monotonic())
            self._cond.notify_all()

    def wait_frame(self, after_seq: int, timeout: float = 1.0):
        with self._cond:
            if self._frame is None or self._frame[0] <= after_seq:
                self._cond.wait(timeout)
            frame = self._frame
        if frame is None or frame[0] <= after_seq:
            return None
        return frame

    def latest(self):
        with self._lock:
            return self._frame

    def grab(self, timeout: float = 3.0):
        with self._lock:
            start = self._seq
        frame = self.wait_frame(start, timeout=timeout)
        return None if frame is None else frame[1]

    def status(self) -> dict:
        with self._lock:
            url = str((self._target_profile or {}).get('url') or '')
        base = {'name': self.name, 'label': self.label, 'role': self.role,
                'arm': self.arm, 'ip': self.ip, 'url': url}
        frame = self.latest()
        if frame is None:
            return {**base, 'online': False, 'message': self._error or '还没收到图'}
        height, width = frame[1].shape[:2]
        age = time.monotonic() - frame[2]
        online = age < 3.0
        extra = {} if online else {'message': self._error or f'最近一帧已过期 ({age:.1f} s)'}
        return {**base, 'online': online, 'width': width, 'height': height,
                'age_s': round(age, 2), **extra}

    def find_profile(self, width: int, height: int) -> dict | None:
        for profile in self.profiles:
            if (profile['width'], profile['height']) == (width, height):
                return profile
        return None

    def apply_profile(self, width: int, height: int, timeout: float = 25.0) -> dict:
        profile = self.find_profile(width, height)
        if profile is None:
            raise ValueError(f'{self.label} 没有 {width}x{height} 这个档位')
        with self._lock:
            self._target_profile = dict(profile)
            seq = self._seq
        self._start_supervisor()
        self._wake.set()

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            frame = self.wait_frame(seq, timeout=min(1.0, max(0.05, remaining)))
            if frame is None:
                continue
            seq, image, _ = frame
            if image.shape[1] == width and image.shape[0] == height:
                return {'width': width, 'height': height}
        raise RuntimeError(f'{self.label} 切到 {width}x{height} 后 {timeout:.0f} s 内没等到对应尺寸的图')

    def close(self) -> None:
        self._closed.set()
        self._wake.set()
        if self._supervisor is not None and self._supervisor is not threading.current_thread():
            self._supervisor.join(timeout=3.0)
        self._close_reader()
        with self._cond:
            self._cond.notify_all()


class MotionGate:
    """所选机械臂静止判据与实时六轴关节角缓存（外参标定时按需在独立后台上下文监听驱动关节反馈）。"""

    def __init__(
        self,
        config: dict,
        default_topic: str = '/joint_states',
        *,
        subscribe_ros: bool = True,
    ) -> None:
        self.max_speed = float(config.get('max_joint_speed', 0.1))
        self.settle = float(config.get('settle_seconds', 0.5))
        self.topic = str(config.get('joint_states_topic') or default_topic)
        self._lock = threading.Lock()
        self._speed: float | None = None
        self._still_since: float | None = None
        self._stamp = 0.0
        self._previous: tuple[np.ndarray, float] | None = None
        self._joints: tuple[float, ...] | None = None
        self._stop = threading.Event()
        self._ros_thread: threading.Thread | None = None
        if subscribe_ros:
            self._ros_thread = threading.Thread(
                target=self._spin_joint_listener, daemon=True, name='calib-joint-sub')
            self._ros_thread.start()

    def _spin_joint_listener(self) -> None:
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import JointState
        except ImportError:
            return

        ctx = Context()
        rclpy.init(context=ctx)
        node = Node('wrist_camera_joint_listener', context=ctx)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        node.create_subscription(JointState, self.topic, self._on_joints_msg, qos)
        executor = SingleThreadedExecutor(context=ctx)
        executor.add_node(node)
        try:
            while not self._stop.is_set() and ctx.ok():
                executor.spin_once(timeout_sec=0.1)
        except Exception:
            pass
        finally:
            executor.remove_node(node)
            node.destroy_node()
            if ctx.ok():
                rclpy.shutdown(context=ctx)

    def _extract_six_joints(self, names: list[str], pos, vel) -> tuple[np.ndarray, np.ndarray | None]:
        if names[:6] == _JOINT_NAMES_6 and len(pos) >= 6:
            positions = np.array([float(pos[i]) for i in range(6)], dtype=float)
            velocities = (
                np.array([float(vel[i]) for i in range(6)], dtype=float)
                if len(vel) >= 6 else None
            )
            return positions, velocities
        if names and all(f'joint{i}' in names for i in range(1, 7)):
            order = [names.index(f'joint{i}') for i in range(1, 7)]
            positions = np.array([float(pos[i]) for i in order], dtype=float)
            velocities = (
                np.array([float(vel[i]) for i in order], dtype=float)
                if all(i < len(vel) for i in order) and len(vel) >= 6 else None
            )
            return positions, velocities
        positions = np.asarray(pos[:6] if len(pos) >= 6 else pos, dtype=float)
        velocities = np.asarray(vel[:6] if len(vel) >= 6 else vel, dtype=float) if vel else None
        return positions, velocities

    def _on_joints_msg(self, message) -> None:
        positions, velocities = self._extract_six_joints(
            list(message.name), message.position, message.velocity)
        self.update_joints(positions, velocities)

    def update_joints(self, positions, velocities=None) -> None:
        now = time.monotonic()
        positions = np.asarray(positions, dtype=float)
        if velocities is not None and len(velocities) > 0:
            speed = float(np.max(np.abs(velocities)))
        else:
            speed = 0.0
            if self._previous is not None:
                previous, stamp = self._previous
                span = now - stamp
                if span > 1e-3 and previous.shape == positions.shape and len(positions) > 0:
                    speed = float(np.max(np.abs(positions - previous)) / span)
            self._previous = (positions, now)
        with self._lock:
            self._speed = speed
            self._stamp = now
            if len(positions) >= 6 and np.all(np.isfinite(positions[:6])):
                self._joints = tuple(float(v) for v in positions[:6])
            if speed <= self.max_speed:
                if self._still_since is None:
                    self._still_since = now
            else:
                self._still_since = None

    def latest_joints(self, max_age: float = 2.0) -> tuple[float, ...] | None:
        with self._lock:
            joints, stamp = self._joints, self._stamp
        if joints is None or (time.monotonic() - stamp) > max_age:
            return None
        return joints

    def state(self) -> dict:
        with self._lock:
            speed, since, stamp = self._speed, self._still_since, self._stamp
        if speed is None:
            return {'ok': False, 'topic': self.topic, 'reason': f'收不到 {self.topic}'}
        if time.monotonic() - stamp > 2.0:
            return {'ok': False, 'topic': self.topic,
                    'reason': f'{self.topic} 已经 2 秒没更新', 'speed': speed}
        if since is None:
            return {'ok': False, 'topic': self.topic, 'speed': round(speed, 5),
                    'reason': f'手臂在动（{speed:.3f} rad/s > {self.max_speed}）'}
        held = time.monotonic() - since
        if held < self.settle:
            return {'ok': False, 'topic': self.topic, 'speed': round(speed, 5),
                    'reason': f'刚停稳 {held:.2f} s，再等 {self.settle - held:.2f} s'}
        return {'ok': True, 'topic': self.topic, 'speed': round(speed, 5), 'still_s': round(held, 2)}

    def close(self) -> None:
        self._stop.set()
        if self._ros_thread is not None:
            self._ros_thread.join(timeout=2.0)


class Frames:
    """机械臂末端位姿查询：由六轴关节角正运动学计算 base_link -> gripper_base_top。"""

    def __init__(
        self,
        arm_id: str = 'arm_3',
        link6_to_top_z_m: float = GRIPPER_WITH_WRENCH_TOP_Z_M,
    ) -> None:
        self.arm_id = arm_id
        self.link6_to_top_z_m = float(link6_to_top_z_m)
        self.angle_offsets = load_arm_angle_offsets(arm_id)

    def _offset_for_child(self, child: str) -> float:
        return self.link6_to_top_z_m if child == 'gripper_base_top' else 0.0

    def lookup(self, parent: str, child: str, gate: MotionGate | None = None) -> np.ndarray:
        dz = self._offset_for_child(child)
        if gate is not None:
            joints = gate.latest_joints()
            if joints is not None:
                return piper_fk(joints, self.angle_offsets, tool_offset_z_m=dz)
        raise RuntimeError(f'尚未收到新鲜的 {self.arm_id} 关节反馈，无法计算 {parent} -> {child}')

    def status(self, parent: str, child: str, gate: MotionGate | None = None) -> dict:
        dz = self._offset_for_child(child)
        if gate is not None:
            joints = gate.latest_joints()
            if joints is not None:
                pose = piper_fk(joints, self.angle_offsets, tool_offset_z_m=dz)
                info = transforms.matrix_to_dict(pose)
                return {
                    'frame': child, 'ok': True, 'source': 'fk',
                    'link6_to_top_z_mm': round(dz * 1000.0, 2),
                    'xyz': [round(v, 4) for v in info['translation']],
                    'rpy_deg': [round(math.degrees(v), 2) for v in info['rpy']],
                }
        return {'frame': child, 'ok': False, 'source': 'none',
                'link6_to_top_z_mm': round(dz * 1000.0, 2)}
