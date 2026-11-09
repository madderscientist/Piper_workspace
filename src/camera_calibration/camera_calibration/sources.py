"""ROS 侧的单臂采集：单路腕部相机图像、机械臂关节状态/正运动学与 TF、以及运行时切分辨率。"""

from __future__ import annotations

import math
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener
import yaml

from camera_calibration import transforms

ALLOWED_ARMS = ('arm_3', 'arm_4')
GRIPPER_WITH_WRENCH_TOP_Z_M = 0.0928
GRIPPER_BASE_WITH_FLANGE_TOP_Z_M = 0.063
GRIPPER_BASE_WITHOUT_FLANGE_TOP_Z_M = 0.0525
GRIPPER_FLANGE_THICKNESS_M = 0.0105
_ARM_ALIASES = {
    'arm_3': 'arm_3', 'arm3': 'arm_3', '/arm_3': 'arm_3', '3': 'arm_3',
    'arm_4': 'arm_4', 'arm4': 'arm_4', '/arm_4': 'arm_4', '4': 'arm_4',
}

IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
JOINT_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=5)

# PiPER-X 6 轴正运动学常量（与 piper_x_driver.gravity._fast_fk 完全一致；joint6_to_gripper_base 为单位变换）
_PIPER_XYZ = (
    (0.0, 0.0, 0.123),
    (0.0, 0.0, 0.0),
    (0.28503, 0.0, 0.0),
    (0.27364, 0.0, 0.0),
    (0.07466, 0.0, 0.0),
    (0.0, -0.035, 0.0),
)
_PIPER_RPY = (
    (0.0, 0.0, 3.1415926),
    (-1.5707963, -3.005806, -3.1415926),
    (0.0, 0.0, 2.8380798),
    (0.0, 0.0, 0.0806342),
    (-1.5707963, 1.5707963, 0.0),
    (1.5707963, 0.0, 0.0),
)


def _euler_xyz_rot(rpy: tuple[float, float, float]) -> tuple[tuple[float, ...], ...]:
    rx, ry, rz = rpy
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return (
        (cy * cz, cz * sx * sy - cx * sz, cx * cz * sy + sx * sz),
        (cy * sz, cx * cz + sx * sy * sz, cx * sy * sz - cz * sx),
        (-sy, cy * sx, cx * cy),
    )


_PIPER_ROT = tuple(_euler_xyz_rot(r) for r in _PIPER_RPY)


def piper_fk(position, angle_offsets=None, tool_offset_z_m: float = 0.0) -> np.ndarray:
    """由六轴关节角计算 base_link -> 末端参考点（默认 link6，或沿 +Z 偏移 tool_offset_z_m 到夹爪最高点）的 4x4 变换。"""
    if len(position) < 6:
        raise ValueError('需要 6 个关节角度')
    r00, r01, r02 = 1.0, 0.0, 0.0
    r10, r11, r12 = 0.0, 1.0, 0.0
    r20, r21, r22 = 0.0, 0.0, 1.0
    tx, ty, tz = 0.0, 0.0, 0.0
    for i in range(6):
        offset = float(angle_offsets[i]) if angle_offsets is not None and i < len(angle_offsets) else 0.0
        q = float(position[i]) + offset
        ox, oy, oz = _PIPER_XYZ[i]
        tx += r00 * ox + r01 * oy + r02 * oz
        ty += r10 * ox + r11 * oy + r12 * oz
        tz += r20 * ox + r21 * oy + r22 * oz
        m0, m1, m2 = _PIPER_ROT[i]
        n00 = r00 * m0[0] + r01 * m1[0] + r02 * m2[0]
        n01 = r00 * m0[1] + r01 * m1[1] + r02 * m2[1]
        n02 = r00 * m0[2] + r01 * m1[2] + r02 * m2[2]
        n10 = r10 * m0[0] + r11 * m1[0] + r12 * m2[0]
        n11 = r10 * m0[1] + r11 * m1[1] + r12 * m2[1]
        n12 = r10 * m0[2] + r11 * m1[2] + r12 * m2[2]
        n20 = r20 * m0[0] + r21 * m1[0] + r22 * m2[0]
        n21 = r20 * m0[1] + r21 * m1[1] + r22 * m2[1]
        n22 = r20 * m0[2] + r21 * m1[2] + r22 * m2[2]
        c, s = math.cos(q), math.sin(q)
        r00, r01, r02 = n00 * c + n01 * s, n01 * c - n00 * s, n02
        r10, r11, r12 = n10 * c + n11 * s, n11 * c - n10 * s, n12
        r20, r21, r22 = n20 * c + n21 * s, n21 * c - n20 * s, n22
    rot = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]], dtype=float)
    dz = float(tool_offset_z_m)
    return transforms.rt_to_matrix(rot, [tx + r02 * dz, ty + r12 * dz, tz + r22 * dz])


def load_arm_angle_offsets(arm_id: str) -> tuple[float, ...]:
    """尝试加载对应机械臂标定过的重力零位偏置 angle_offset（不存在则返回全 0）。"""
    candidates = [
        Path(f'/home/a/piper_x_ws/src/piper_gravity/config/{arm_id}_gravity.yaml'),
    ]
    for path in candidates:
        if path.is_file():
            try:
                data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
                joints = data.get('joints', {}) or {}
                return tuple(
                    float((joints.get(f'joint{i}') or joints.get(i) or {}).get('angle_offset', 0.0))
                    for i in range(1, 7)
                )
            except Exception:
                pass
    return (0.0,) * 6


def normalize_arm_id(arm: str) -> str:
    raw = str(arm or '').strip().lower()
    if not raw:
        raise ValueError('启动标定 dashboard 必须指定 arm（仅支持 arm_3 或 arm_4）')
    canonical = _ARM_ALIASES.get(raw)
    if canonical is None:
        raise ValueError(f'只有 arm_3 和 arm_4 有腕部相机，不支持 {arm!r}')
    return canonical


def select_arm_camera(cameras_config: dict, arm: str, camera: str) -> tuple[str, str, dict]:
    """校验并选出单臂与对应腕部相机配置，拒绝未指定、非 arm_3/arm_4 或臂与相机不匹配的启动请求。"""
    arm_raw = str(arm or '').strip()
    cam_raw = str(camera or '').strip()
    if not arm_raw or not cam_raw:
        raise ValueError(
            '启动标定 dashboard 需要同时指定 arm 和 camera（例如 arm:=arm_3 camera:=camera_arm_3）')
    arm_id = normalize_arm_id(arm_raw)
    cameras = (cameras_config or {}).get('cameras', {}) or {}
    if not cameras:
        raise ValueError('cameras.yaml 中没有配置任何相机')

    cam_low = cam_raw.lower().strip('/')
    matched_name = None
    matched_cfg = None
    for name, cfg in cameras.items():
        aliases = [str(a).lower().strip('/') for a in (cfg.get('aliases') or [])]
        candidates = {
            name.lower(),
            str(cfg.get('frame', '')).lower(),
            str(cfg.get('ip', '')).lower(),
            str(cfg.get('topic', '')).lower().strip('/'),
            str((cfg.get('switch') or {}).get('node', '')).lower().strip('/'),
            *aliases,
        }
        if cam_low in candidates:
            matched_name = name
            matched_cfg = dict(cfg)
            break

    if matched_name is None or matched_cfg is None:
        raise ValueError(f'未找到相机 {camera!r}，可选相机：{list(cameras)}')

    cfg_arm = matched_cfg.get('arm')
    if cfg_arm and normalize_arm_id(cfg_arm) != arm_id:
        raise ValueError(
            f'指定的机械臂 {arm_id} 与相机 {matched_name}（归属 {cfg_arm}）不匹配')
    matched_cfg.setdefault('arm', arm_id)
    matched_cfg.setdefault('arm_namespace', f'/{arm_id}')
    return arm_id, matched_name, matched_cfg


def image_to_bgr(message: Image) -> np.ndarray:
    """按 encoding 转 BGR。"""
    encoding = message.encoding.lower()
    raw = np.frombuffer(message.data, np.uint8)
    height, width = message.height, message.width
    if encoding in ('bgr8', 'rgb8'):
        rows = raw.reshape(height, message.step)[:, :width * 3]
        image = rows.reshape(height, width, 3)
        return image if encoding == 'bgr8' else cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == 'mono8':
        rows = raw.reshape(height, message.step)[:, :width]
        return cv2.cvtColor(rows.reshape(height, width), cv2.COLOR_GRAY2BGR)
    if encoding in ('yuyv', 'yuv422_yuy2'):
        rows = raw.reshape(height, message.step)[:, :width * 2]
        return cv2.cvtColor(rows.reshape(height, width, 2), cv2.COLOR_YUV2BGR_YUYV)
    raise ValueError(f'不认识的图像编码：{message.encoding}')


class CameraSource:
    """一路相机：缓存最新一帧，并负责把它切到指定档位。"""

    def __init__(self, node, name: str, config: dict) -> None:
        self.node = node
        self.name = name
        self.config = config
        self.label = config.get('label', name)
        self.role = config.get('role', 'target')
        self.frame = config.get('frame', name)
        self.parent_frame = config.get('parent_frame', 'gripper_base')
        self.ip = config.get('ip', '')
        self.arm = config.get('arm', '')
        self.profiles = config.get('profiles', [])
        self._lock = threading.Lock()
        self._frame = None                   # (seq, bgr, stamp)
        self._seq = 0
        self._info = None
        self._client = None

        node.create_subscription(Image, config['topic'], self._on_image, IMAGE_QOS)
        if config.get('info_topic'):
            node.create_subscription(CameraInfo, config['info_topic'], self._on_info, 10)
        switch = config.get('switch', {})
        if switch.get('kind') in ('camera_node', 'realsense'):
            self._client = node.create_client(
                SetParameters, f"{switch['node'].rstrip('/')}/set_parameters",
                callback_group=node.callbacks)

    def _on_image(self, message: Image) -> None:
        try:
            image = image_to_bgr(message)
        except ValueError as error:
            self.node.get_logger().warn(f'{self.name}: {error}', once=True)
            return
        with self._lock:
            self._seq += 1
            self._frame = (self._seq, image.copy(), time.monotonic())

    def _on_info(self, message: CameraInfo) -> None:
        self._info = {
            'width': message.width, 'height': message.height,
            'camera_matrix': [float(v) for v in message.k],
            'distortion_model': message.distortion_model or 'plumb_bob',
            'distortion_coefficients': [float(v) for v in message.d][:5] or [0.0] * 5,
        }

    @property
    def factory_info(self) -> dict | None:
        return self._info

    def latest(self):
        with self._lock:
            return self._frame

    def grab(self, timeout: float = 3.0):
        with self._lock:
            start = self._seq
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                frame = self._frame
            if frame is not None and frame[0] > start:
                return frame[1]
            time.sleep(0.02)
        return None

    def status(self) -> dict:
        base = {'name': self.name, 'label': self.label, 'role': self.role,
                'arm': self.arm, 'ip': self.ip, 'topic': self.config.get('topic', '')}
        frame = self.latest()
        if frame is None:
            return {**base, 'online': False, 'message': '还没收到图'}
        height, width = frame[1].shape[:2]
        age = time.monotonic() - frame[2]
        return {**base, 'online': age < 3.0, 'width': width, 'height': height,
                'age_s': round(age, 2)}

    def find_profile(self, width: int, height: int) -> dict | None:
        for profile in self.profiles:
            if (profile['width'], profile['height']) == (width, height):
                return profile
        return None

    def apply_profile(self, width: int, height: int, timeout: float = 25.0) -> dict:
        profile = self.find_profile(width, height)
        if profile is None:
            raise ValueError(f'{self.label} 没有 {width}x{height} 这个档位')
        if self._client is None:
            raise RuntimeError(f'{self.label} 不支持在线切档位，请改 launch 参数后重启')

        if self.config['switch']['kind'] == 'camera_node':
            self._call([_param('rtsp_url', profile['url']),
                        _param('image_width', 0), _param('image_height', 0)],
                       timeout=10.0)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.grab(timeout=1.0)
            if frame is not None and frame.shape[1] == width and frame.shape[0] == height:
                return {'width': width, 'height': height}
        raise RuntimeError(f'{self.label} 切到 {width}x{height} 后 {timeout:.0f} s 内没等到对应尺寸的图')

    def _call(self, parameters, timeout: float) -> None:
        if not self._client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"连不上 {self.config['switch']['node']} 的参数服务")
        request = SetParameters.Request()
        request.parameters = parameters
        future = self._client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError('设置参数超时')
        for result in future.result().results:
            if not result.successful:
                raise RuntimeError(f'设置参数被拒绝：{result.reason}')


class MotionGate:
    """所选机械臂静止判据与实时六轴关节角缓存。"""

    def __init__(self, node, config: dict, default_topic: str = '/joint_states') -> None:
        self.max_speed = float(config.get('max_joint_speed', 0.1))
        self.settle = float(config.get('settle_seconds', 0.5))
        self.topic = str(config.get('joint_states_topic') or default_topic)
        self._lock = threading.Lock()
        self._speed = None
        self._still_since = None
        self._stamp = 0.0
        self._previous = None
        self._joints: tuple[float, ...] | None = None
        node.create_subscription(JointState, self.topic, self._on_joints, JOINT_QOS)
        if self.topic != '/joint_states':
            node.create_subscription(JointState, '/joint_states', self._on_joints, 10)

    def _extract_six_joints(self, message: JointState) -> tuple[np.ndarray, np.ndarray | None]:
        names = list(message.name)
        pos = message.position
        vel = message.velocity
        if names and all(f'joint{i}' in names for i in range(1, 7)):
            order = [names.index(f'joint{i}') for i in range(1, 7)]
            positions = np.array([float(pos[i]) for i in order], dtype=float)
            velocities = (
                np.array([float(vel[i]) for i in order], dtype=float)
                if all(i < len(vel) for i in order) and len(vel) >= 6
                else None
            )
            return positions, velocities
        positions = np.asarray(pos[:6] if len(pos) >= 6 else pos, dtype=float)
        velocities = np.asarray(vel[:6] if len(vel) >= 6 else vel, dtype=float) if vel else None
        return positions, velocities

    def _on_joints(self, message: JointState) -> None:
        now = time.monotonic()
        positions, velocities = self._extract_six_joints(message)
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


class Frames:
    """机械臂末端位姿查询：计算 base_link -> gripper_base_top（夹爪底座 Mesh 最高点中心）。"""

    def __init__(self, node, arm_id: str = 'arm_3',
                 link6_to_top_z_m: float = GRIPPER_WITH_WRENCH_TOP_Z_M) -> None:
        self.node = node
        self.arm_id = arm_id
        self.link6_to_top_z_m = float(link6_to_top_z_m)
        self.angle_offsets = load_arm_angle_offsets(arm_id)
        self.buffer = Buffer()
        self._listener = TransformListener(self.buffer, node)

    def _offset_for_child(self, child: str) -> float:
        return self.link6_to_top_z_m if child == 'gripper_base_top' else 0.0

    def lookup(self, parent: str, child: str, timeout: float = 1.0,
               gate: MotionGate | None = None) -> np.ndarray:
        dz = self._offset_for_child(child)
        if gate is not None:
            joints = gate.latest_joints()
            if joints is not None:
                return piper_fk(joints, self.angle_offsets, tool_offset_z_m=dz)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.buffer.can_transform(parent, child, rclpy.time.Time()):
                break
            if child == 'gripper_base_top' and self.buffer.can_transform(parent, 'gripper_base', rclpy.time.Time()):
                break
            time.sleep(0.02)
        if self.buffer.can_transform(parent, child, rclpy.time.Time()):
            stamped = self.buffer.lookup_transform(parent, child, rclpy.time.Time())
            rotation = stamped.transform.rotation
            translation = stamped.transform.translation
            return transforms.rt_to_matrix(
                transforms.quat_to_matrix([rotation.x, rotation.y, rotation.z, rotation.w]),
                [translation.x, translation.y, translation.z])
        if child == 'gripper_base_top' and self.buffer.can_transform(parent, 'gripper_base', rclpy.time.Time()):
            stamped = self.buffer.lookup_transform(parent, 'gripper_base', rclpy.time.Time())
            rotation = stamped.transform.rotation
            translation = stamped.transform.translation
            base_grip = transforms.rt_to_matrix(
                transforms.quat_to_matrix([rotation.x, rotation.y, rotation.z, rotation.w]),
                [translation.x, translation.y, translation.z])
            return base_grip @ transforms.rt_to_matrix(np.eye(3), [0.0, 0.0, dz])
        raise RuntimeError(f'收不到关节反馈且查不到 TF {parent} -> {child}')

    def available(self, parent: str, child: str, gate: MotionGate | None = None) -> bool:
        if gate is not None and gate.latest_joints() is not None:
            return True
        if self.buffer.can_transform(parent, child, rclpy.time.Time()):
            return True
        if child == 'gripper_base_top' and self.buffer.can_transform(parent, 'gripper_base', rclpy.time.Time()):
            return True
        return False

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
        tf_ok = self.available(parent, child, gate=None)
        return {'frame': child, 'ok': tf_ok, 'source': 'tf' if tf_ok else 'none',
                'link6_to_top_z_mm': round(dz * 1000.0, 2)}


def _param(name: str, value) -> Parameter:
    if isinstance(value, bool):
        inner = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    elif isinstance(value, int):
        inner = ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                               integer_value=value)
    else:
        inner = ParameterValue(type=ParameterType.PARAMETER_STRING,
                               string_value=str(value))
    return Parameter(name=name, value=inner)
