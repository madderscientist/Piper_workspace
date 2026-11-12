"""4x4 齐次变换与 PiPER-X 六轴正运动学（纯 numpy + math，零 ROS / cv2 / scipy 依赖）。"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

GRIPPER_WITH_WRENCH_TOP_Z_M = 0.0928
GRIPPER_BASE_WITH_FLANGE_TOP_Z_M = 0.063
GRIPPER_BASE_WITHOUT_FLANGE_TOP_Z_M = 0.0525
GRIPPER_FLANGE_THICKNESS_M = 0.0105

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


def rodrigues(rvec) -> np.ndarray:
    """3 维旋转向量 -> 3x3 旋转矩阵（避免顶层引 cv2）。"""
    v = np.asarray(rvec, float).reshape(3)
    theta = float(np.linalg.norm(v))
    if theta < 1e-12:
        return np.eye(3) + np.array([
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ])
    kx, ky, kz = v / theta
    k_mat = np.array([
        [0.0, -kz, ky],
        [kz, 0.0, -kx],
        [-ky, kx, 0.0],
    ])
    return np.eye(3) + math.sin(theta) * k_mat + (1.0 - math.cos(theta)) * (k_mat @ k_mat)


def rt_to_matrix(rotation, translation) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = np.asarray(rotation, float).reshape(3, 3)
    matrix[:3, 3] = np.asarray(translation, float).reshape(3)
    return matrix


def rvec_tvec_to_matrix(rvec, tvec) -> np.ndarray:
    return rt_to_matrix(rodrigues(rvec), np.asarray(tvec, float).reshape(3))


def invert(matrix) -> np.ndarray:
    matrix = np.asarray(matrix, float)
    rotation = matrix[:3, :3]
    out = np.eye(4)
    out[:3, :3] = rotation.T
    out[:3, 3] = -rotation.T @ matrix[:3, 3]
    return out


def quat_to_matrix(quat) -> np.ndarray:
    """四元数按 [x, y, z, w] 顺序转 3x3 旋转矩阵。"""
    x, y, z, w = (float(v) for v in quat)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError('四元数模长为零')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(matrix) -> np.ndarray:
    rotation = np.asarray(matrix, float)[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
    quat = np.array([x, y, z, w])
    return quat / np.linalg.norm(quat)


def matrix_to_rpy(matrix) -> np.ndarray:
    rotation = np.asarray(matrix, float)[:3, :3]
    sy = max(-1.0, min(1.0, -rotation[2, 0]))
    pitch = math.asin(sy)
    if abs(sy) > 1.0 - 1e-9:
        return np.array([0.0, pitch, math.atan2(-rotation[0, 1], rotation[1, 1])])
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return np.array([roll, pitch, yaw])


def average_transforms(matrices) -> np.ndarray:
    matrices = [np.asarray(m, float) for m in matrices]
    if not matrices:
        raise ValueError('没有样本可平均')
    quats = np.stack([matrix_to_quat(m) for m in matrices])
    _, vectors = np.linalg.eigh(quats.T @ quats)
    mean_quat = vectors[:, -1]
    if mean_quat[3] < 0:
        mean_quat = -mean_quat
    translation = np.mean([m[:3, 3] for m in matrices], axis=0)
    return rt_to_matrix(quat_to_matrix(mean_quat), translation)


def transform_delta(a, b) -> tuple[float, float]:
    delta = invert(np.asarray(a, float)) @ np.asarray(b, float)
    cos = (float(np.trace(delta[:3, :3])) - 1.0) / 2.0
    angle = math.degrees(math.acos(max(-1.0, min(1.0, cos))))
    return angle, float(np.linalg.norm(delta[:3, 3]))


def perturb(matrix, delta) -> np.ndarray:
    delta = np.asarray(delta, float)
    step = rt_to_matrix(rodrigues(delta[:3]), delta[3:])
    return np.asarray(matrix, float) @ step


def log_delta(matrix) -> np.ndarray:
    matrix = np.asarray(matrix, float)
    rot = matrix[:3, :3]
    v = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]], dtype=float)
    s2 = float(np.linalg.norm(v))
    c2 = float(np.trace(rot)) - 1.0
    if s2 < 1e-7 and c2 >= 0.0:
        rvec = 0.5 * v
    elif s2 > 1e-12:
        theta = math.atan2(0.5 * s2, 0.5 * c2)
        rvec = (theta / s2) * v
    else:
        # 接近 180° 旋转时从 R + I 的列提取旋转轴
        plus = rot + np.eye(3)
        col = plus[:, int(np.argmax(np.linalg.norm(plus, axis=0)))]
        n = float(np.linalg.norm(col))
        rvec = (math.pi / n) * col if n > 1e-12 else np.zeros(3)
    return np.concatenate([rvec, matrix[:3, 3]])


def deviation(deltas) -> dict:
    angles = [d[0] for d in deltas]
    distances = [d[1] for d in deltas]
    return {
        'count': len(deltas),
        'angle_rms_deg': float(np.sqrt(np.mean(np.square(angles)))) if angles else 0.0,
        'angle_max_deg': float(max(angles)) if angles else 0.0,
        'trans_rms_mm': float(np.sqrt(np.mean(np.square(distances))) * 1e3) if distances else 0.0,
        'trans_max_mm': float(max(distances) * 1e3) if distances else 0.0,
    }


def spread(matrices, mean=None) -> dict:
    matrices = [np.asarray(m, float) for m in matrices]
    reference = average_transforms(matrices) if mean is None else np.asarray(mean, float)
    deltas = [transform_delta(reference, m) for m in matrices]
    return {**deviation(deltas),
            'per_sample': [{'angle_deg': a, 'trans_mm': d * 1e3} for a, d in deltas]}


def matrix_to_dict(matrix) -> dict:
    matrix = np.asarray(matrix, float)
    quat = matrix_to_quat(matrix)
    rpy = matrix_to_rpy(matrix)
    return {
        'translation': [float(v) for v in matrix[:3, 3]],
        'rotation': [float(v) for v in quat],
        'rpy': [float(v) for v in rpy],
    }


def matrix_from_dict(data) -> np.ndarray:
    return rt_to_matrix(quat_to_matrix(data['rotation']), data['translation'])


def piper_fk(position, angle_offsets=None, tool_offset_z_m: float = 0.0) -> np.ndarray:
    """由六轴关节角计算 base_link -> 末端参考点（默认 link6，或沿 +Z 偏移到夹爪最高点）的 4x4 变换。"""
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
    return rt_to_matrix(rot, [tx + r02 * dz, ty + r12 * dz, tz + r22 * dz])


def load_arm_angle_offsets(arm_id: str) -> tuple[float, ...]:
    """加载对应机械臂标定过的重力零位偏置 angle_offset（不存在则返回全 0）。"""
    arm_clean = str(arm_id or '').strip().lstrip('/')
    pkg_src = Path(__file__).resolve().parents[2] / 'piper_gravity' / 'config' / f'{arm_clean}_gravity.yaml'
    candidates = [pkg_src, Path(f'/home/a/piper_x_ws/src/piper_gravity/config/{arm_clean}_gravity.yaml')]
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
