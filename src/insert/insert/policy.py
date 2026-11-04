"""InsertAnything ONNX policy inference, frame transforms, wrench filtering, and tip IK."""

from dataclasses import dataclass
import math
from pathlib import Path
import time
from typing import Sequence

import numpy as np
import onnxruntime as ort

from piper_x_driver.controller import MIT_POSITION_LIMIT
from piper_x_driver.gravity import ArmGravityModel, _fast_fk, end_effector_wrench, geometric_jacobian

POLICY_PERIOD_S = 0.02
ACTION_TRANSLATION_SCALE_M = 0.001
ACTION_ROTATION_SCALE_RAD = math.radians(0.5)

MAX_TIP_LEAD_XY_M = 0.012
MAX_TIP_LEAD_Z_M = 0.012
MAX_TIP_LEAD_RAD = math.radians(4.0)
MAX_GOAL_TILT_RAD = math.radians(5.0)
MAX_JOINT_STEP_RAD = 0.04
SERVO_LAG_TAU_S = 0.06

TRAIN_WRENCH_DT_S = 0.001
TRAIN_WRENCH_FILTER_TAU_S = 0.015
DRIVER_WRENCH_DT_S = 0.005


def equivalent_filter_tau(
    tau_ref: float = TRAIN_WRENCH_FILTER_TAU_S, dt_ref: float = TRAIN_WRENCH_DT_S, dt_target: float = DRIVER_WRENCH_DT_S,
) -> float:
    if tau_ref <= 0.0 or dt_ref <= 0.0 or dt_target <= 0.0:
        raise ValueError('tau_ref, dt_ref, and dt_target must be positive')
    alpha_target = (float(dt_target) / float(dt_ref)) * (1.0 - math.exp(-float(dt_ref) / float(tau_ref)))
    return float(dt_target) * 0.25 if alpha_target >= 1.0 else -float(dt_target) / math.log(1.0 - alpha_target)


WRENCH_FILTER_TAU_S = equivalent_filter_tau()
DEFAULT_TIP_LENGTH_M = 0.195
DEFAULT_TIP_OFFSET_M: tuple[float, float, float] = (0.0, 0.0, DEFAULT_TIP_LENGTH_M)

_Q_T_FROM_GRIPPER = np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float64)
_R_LINK6_TO_T = np.diag([-1.0, 1.0, -1.0])
_EYE3 = np.eye(3, dtype=np.float64)
_EYE6 = np.eye(6, dtype=np.float64)


def default_onnx_path() -> Path:
    here = Path(__file__).resolve()
    local = here.parent.parent / 'actor.onnx'
    if local.is_file():
        return local
    for parent in here.parents:
        candidate = parent / 'src' / 'insert' / 'actor.onnx'
        if candidate.is_file():
            return candidate
    return local


def canonical_quat(q: Sequence[float] | np.ndarray) -> np.ndarray:
    arr = np.asarray(q, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(arr))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid or zero-norm quaternion')
    unit = arr / norm
    for comp in unit:
        if abs(float(comp)) > 1e-8:
            return -unit if comp < 0.0 else unit
    return unit


def quat_inv(q: Sequence[float] | np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(q, dtype=np.float64).reshape(4)
    return np.array([w, -x, -y, -z], dtype=np.float64)


def quat_mul(a: Sequence[float] | np.ndarray, b: Sequence[float] | np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.asarray(a, dtype=np.float64).reshape(4)
    bw, bx, by, bz = np.asarray(b, dtype=np.float64).reshape(4)
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=np.float64)


def rotate(q: Sequence[float] | np.ndarray, v: Sequence[float] | np.ndarray) -> np.ndarray:
    q_arr = np.asarray(q, dtype=np.float64).reshape(4)
    v_arr = np.asarray(v, dtype=np.float64).reshape(3)
    qv = q_arr[1:4]
    uv = np.cross(qv, v_arr)
    return v_arr + 2.0 * (q_arr[0] * uv + np.cross(qv, uv))


def rotvec_quat(v: Sequence[float] | np.ndarray) -> np.ndarray:
    v_arr = np.asarray(v, dtype=np.float64).reshape(3)
    angle = float(np.linalg.norm(v_arr))
    scale = 0.5 * float(np.sinc(angle / (2.0 * math.pi)))
    return np.array([math.cos(0.5 * angle), *(v_arr * scale)], dtype=np.float64)


def quat_rotvec(q: Sequence[float] | np.ndarray) -> np.ndarray:
    q_can = canonical_quat(q)
    qv = q_can[1:4]
    sine = float(np.linalg.norm(qv))
    angle = 2.0 * math.atan2(sine, float(q_can[0]))
    return qv * (angle / max(sine, 1e-12) if sine > 1e-7 else 2.0)


def quat_to_mat(q: Sequence[float] | np.ndarray) -> np.ndarray:
    w, x, y, z = canonical_quat(q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


def orthonormalize_rot(rot: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(np.asarray(rot, dtype=np.float64).reshape(3, 3))
    r = u @ vt
    if np.linalg.det(r) < 0.0:
        u[:, -1] = -u[:, -1]
        r = u @ vt
    return r


def _mat_to_quat_raw(r: np.ndarray) -> np.ndarray:
    tr = float(r[0, 0] + r[1, 1] + r[2, 2])
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = (0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        q = ((r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s)
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        q = ((r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s)
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        q = ((r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s)
    return canonical_quat(q)


def mat_to_quat(rot: np.ndarray) -> np.ndarray:
    return _mat_to_quat_raw(orthonormalize_rot(rot))


def quat_to_rpy_deg(q: Sequence[float] | np.ndarray) -> tuple[float, float, float]:
    w, x, y, z = canonical_quat(q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def transform_wrench(
    force: Sequence[float] | np.ndarray, moment: Sequence[float] | np.ndarray,
    q_target_source: Sequence[float] | np.ndarray, source_to_target_in_target: Sequence[float] | np.ndarray,
) -> np.ndarray:
    f = rotate(q_target_source, force)
    m = rotate(q_target_source, moment)
    return np.concatenate([f, m - np.cross(np.asarray(source_to_target_in_target, dtype=np.float64).reshape(3), f)])


def actor_observation(
    tip_pos_h: Sequence[float] | np.ndarray, q_ht: Sequence[float] | np.ndarray,
    target_pos_h: Sequence[float] | np.ndarray, target_q_h: Sequence[float] | np.ndarray,
    wrench_t: Sequence[float] | np.ndarray,
) -> np.ndarray:
    p_ht = np.asarray(tip_pos_h, dtype=np.float64).reshape(3)
    p_goal = np.asarray(target_pos_h, dtype=np.float64).reshape(3)
    rel_quat = canonical_quat(quat_mul(quat_inv(canonical_quat(q_ht)), canonical_quat(target_q_h)))
    obs = np.empty((1, 13), dtype=np.float32)
    obs[0, 0:3] = p_goal - p_ht
    obs[0, 3:7] = rel_quat
    obs[0, 7:13] = np.asarray(wrench_t, dtype=np.float64).reshape(6)
    return obs


def action_target(
    tip_pos_h: Sequence[float] | np.ndarray, q_ht: Sequence[float] | np.ndarray, action: Sequence[float] | np.ndarray,
    translation_limit_m: float = ACTION_TRANSLATION_SCALE_M, rotation_limit_rad: float = ACTION_ROTATION_SCALE_RAD,
) -> tuple[np.ndarray, np.ndarray]:
    p_ht = np.asarray(tip_pos_h, dtype=np.float64).reshape(3)
    q_curr = canonical_quat(q_ht)
    act = np.asarray(action, dtype=np.float64).reshape(6)
    return (
        p_ht + act[:3] * float(translation_limit_m),
        canonical_quat(quat_mul(rotvec_quat(act[3:6] * float(rotation_limit_rad)), q_curr)),
    )


def normalize_tip_offset(
    tip_offset_m: Sequence[float] | np.ndarray | None = None, tip_length_m: float | None = None,
) -> tuple[float, float, float]:
    if tip_offset_m is not None:
        arr = np.asarray(tip_offset_m, dtype=np.float64).reshape(3)
        if not np.isfinite(arr).all():
            raise ValueError('tip_offset_m must contain finite numbers')
        return (float(arr[0]), float(arr[1]), float(arr[2]))
    if tip_length_m is not None:
        val = float(tip_length_m)
        if not math.isfinite(val):
            raise ValueError('tip_length_m must be finite')
        return (0.0, 0.0, val)
    return DEFAULT_TIP_OFFSET_M


def _gripper_offset_to_link6(offset_gripper: tuple[float, float, float]) -> tuple[float, float, float]:
    gx, gy, gz = offset_gripper
    return (-float(gx), -float(gy), float(gz))


def forward_tip_pose(
    position: Sequence[float], tip_length_m: float | None = None,
    gravity_model: ArmGravityModel | None = None, *, tip_offset_m: Sequence[float] | np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    if len(position) < 6:
        raise ValueError('Six joint positions are required')
    offset_link6 = _gripper_offset_to_link6(normalize_tip_offset(tip_offset_m, tip_length_m))
    q_eff = (
        tuple(float(position[i]) for i in range(6))
        if gravity_model is None
        else tuple(float(position[i]) + gravity_model.joint(i + 1).angle_offset for i in range(6))
    )
    _, _, centers, rots = _fast_fk(q_eff, offset_link6)
    return np.asarray(centers[6], dtype=np.float64), mat_to_quat(np.asarray(rots[5], dtype=np.float64) @ _R_LINK6_TO_T)


def relative_pose_in_hole(
    p_bt: Sequence[float] | np.ndarray, q_bt: Sequence[float] | np.ndarray,
    p_bh: Sequence[float] | np.ndarray, q_bh: Sequence[float] | np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    q_hb = quat_inv(canonical_quat(q_bh))
    return (
        rotate(q_hb, np.asarray(p_bt, dtype=np.float64) - np.asarray(p_bh, dtype=np.float64)),
        canonical_quat(quat_mul(q_hb, canonical_quat(q_bt))),
    )


def hole_target_to_base(
    p_target_h: Sequence[float] | np.ndarray, q_target_ht: Sequence[float] | np.ndarray,
    p_bh: Sequence[float] | np.ndarray, q_bh: Sequence[float] | np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    q_bh_can = canonical_quat(q_bh)
    return (
        np.asarray(p_bh, dtype=np.float64) + rotate(q_bh_can, p_target_h),
        canonical_quat(quat_mul(q_bh_can, canonical_quat(q_target_ht))),
    )


def solve_tip_ik(
    q_init: Sequence[float], p_target_b: Sequence[float] | np.ndarray, q_target_bt: Sequence[float] | np.ndarray,
    *, tip_length_m: float | None = None, tip_offset_m: Sequence[float] | np.ndarray | None = None,
    gravity_model: ArmGravityModel | None = None, damping: float = 1e-3, max_iters: int = 20,
) -> tuple[float, ...]:
    if len(q_init) < 6:
        raise ValueError('Six initial joint positions are required')
    q = np.array([float(q_init[i]) for i in range(6)], dtype=np.float64)
    p_des = np.asarray(p_target_b, dtype=np.float64).reshape(3)
    q_des = canonical_quat(q_target_bt)
    tool_offset = _gripper_offset_to_link6(normalize_tip_offset(tip_offset_m, tip_length_m))
    offsets = (
        (0.0,) * 6
        if gravity_model is None
        else tuple(gravity_model.joint(i + 1).angle_offset for i in range(6))
    )
    lam2_eye = (float(damping) ** 2) * _EYE6
    jac = np.empty((6, 6), dtype=np.float64)
    err6 = np.empty(6, dtype=np.float64)

    for _ in range(max(1, int(max_iters))):
        origins, axes, centers, rots = _fast_fk(tuple(float(q[i]) + offsets[i] for i in range(6)), tool_offset)
        ex, ey, ez = centers[6]
        ep0, ep1, ep2 = p_des[0] - ex, p_des[1] - ey, p_des[2] - ez
        q_curr = _mat_to_quat_raw(np.asarray(rots[5], dtype=np.float64) @ _R_LINK6_TO_T)
        er = quat_rotvec(quat_mul(q_des, quat_inv(q_curr)))
        if (ep0 * ep0 + ep1 * ep1 + ep2 * ep2) < 1e-14 and float(er @ er) < 1e-14:
            break
        err6[0], err6[1], err6[2] = ep0, ep1, ep2
        err6[3:6] = er
        for i in range(6):
            zx, zy, zz = axes[i]
            ox, oy, oz = origins[i]
            rx, ry, rz = ex - ox, ey - oy, ez - oz
            jac[0, i] = zy * rz - zz * ry
            jac[1, i] = zz * rx - zx * rz
            jac[2, i] = zx * ry - zy * rx
            jac[3, i] = zx
            jac[4, i] = zy
            jac[5, i] = zz
        q = np.clip(q + jac.T @ np.linalg.solve(jac @ jac.T + lam2_eye, err6), -MIT_POSITION_LIMIT, MIT_POSITION_LIMIT)

    return tuple(float(v) for v in q)


def flange_wrench_to_tip_wrench(
    flange_wrench_s: Sequence[float] | np.ndarray, position: Sequence[float],
    *, tip_length_m: float | None = None, tip_offset_m: Sequence[float] | np.ndarray | None = None,
    gravity_model: ArmGravityModel | None = None, q_cmd: Sequence[float] | np.ndarray | None = None,
    q_servo: Sequence[float] | np.ndarray | None = None, kp: Sequence[float] | np.ndarray | None = None,
    damping: float = 1e-2,
) -> np.ndarray:
    w0 = np.asarray(flange_wrench_s, dtype=np.float64).reshape(6)
    if float(w0 @ w0) < 1e-24:
        return np.zeros(6, dtype=np.float64)
    offset_link6 = _gripper_offset_to_link6(normalize_tip_offset(tip_offset_m, tip_length_m))
    j0 = (
        geometric_jacobian(position[:6], tool_offset_m=(0.0, 0.0, 0.0), frame='gripper')
        if gravity_model is None
        else gravity_model.jacobian(position[:6], tool_offset_m=(0.0, 0.0, 0.0), frame='gripper')
    )
    jv, jw = j0[:3, :], j0[3:6, :]
    lam2 = float(damping) * float(damping)
    pv = np.linalg.solve(jv @ jv.T + lam2 * _EYE3, jv)
    pw = np.linalg.solve(jw @ jw.T + max(lam2, 1e-2) * _EYE3, jw @ (_EYE6 - jv.T @ pv))
    tau_ext = np.linalg.solve(np.vstack([pv, pw]), w0)
    if q_cmd is not None and q_servo is not None and kp is not None:
        tau_ext = tau_ext - np.asarray(kp, dtype=np.float64) * (
            np.asarray(q_servo, dtype=np.float64) - np.asarray(q_cmd, dtype=np.float64)
        )
    return np.asarray(
        end_effector_wrench(
            tuple(float(v) for v in tau_ext),
            position[:6], gravity_model=gravity_model, tool_offset_m=offset_link6, damping=damping, frame='gripper',
        ),
        dtype=np.float64,
    )


class WrenchFilter:
    def __init__(self, tau: float = WRENCH_FILTER_TAU_S, *, tau_1khz: float = TRAIN_WRENCH_FILTER_TAU_S) -> None:
        if tau <= 0.0 or not math.isfinite(tau):
            raise ValueError('tau must be finite and positive')
        self.tau = float(tau)
        self.tau_1khz = float(tau_1khz)
        self.bias_s = np.zeros(6, dtype=np.float64)
        self.raw_s = np.zeros(6, dtype=np.float64)
        self.filtered_s = np.zeros(6, dtype=np.float64)
        self._last_stamp: float | None = None
        self._initialized = False
        self._already_at_tip = False

    def reset(self) -> None:
        self.filtered_s.fill(0.0)
        self._last_stamp = None
        self._initialized = False

    def tare(self, raw_wrench_s: Sequence[float] | np.ndarray | None = None) -> np.ndarray:
        if raw_wrench_s is not None:
            arr = np.asarray(raw_wrench_s, dtype=np.float64).reshape(6)
            if not np.isfinite(arr).all():
                raise ValueError('Cannot tare with non-finite wrench')
            self.raw_s = arr.copy()
        self.bias_s = self.raw_s.copy()
        self.reset()
        return self.bias_s.copy()

    def clear_tare(self) -> None:
        self.bias_s.fill(0.0)
        self.reset()

    def update(
        self, raw_wrench_s: Sequence[float] | np.ndarray, timestamp: float | None = None,
        dt: float | None = None, *, already_at_tip: bool = False,
    ) -> np.ndarray:
        arr = np.asarray(raw_wrench_s, dtype=np.float64).reshape(6)
        if not np.isfinite(arr).all():
            raise ValueError('Wrench contains non-finite values')
        self.raw_s = arr.copy()
        self._already_at_tip = bool(already_at_tip)
        compensated = arr - self.bias_s
        if dt is None and timestamp is not None:
            if self._last_stamp is not None:
                dt = float(timestamp) - self._last_stamp
            self._last_stamp = float(timestamp)
        if not self._initialized or dt is None or dt <= 0.0 or dt > 0.5:
            self.filtered_s = compensated.copy()
            self._initialized = True
        else:
            self.filtered_s += (1.0 - math.exp(-float(dt) / self.tau)) * (compensated - self.filtered_s)
        return self.filtered_s.copy()

    def tip_wrench(
        self, tip_length_m: float | None = None, *, tip_offset_m: Sequence[float] | np.ndarray | None = None,
    ) -> np.ndarray:
        if self._already_at_tip:
            f = self.filtered_s
            return np.array([f[0], -f[1], -f[2], f[3], -f[4], -f[5]], dtype=np.float64)
        offset_gripper = normalize_tip_offset(tip_offset_m, tip_length_m)
        return transform_wrench(
            self.filtered_s[:3], self.filtered_s[3:6], _Q_T_FROM_GRIPPER, rotate(_Q_T_FROM_GRIPPER, offset_gripper),
        )


@dataclass(frozen=True)
class GoalPose:
    joints: tuple[float, ...]
    p_bh: tuple[float, float, float]
    q_bh: tuple[float, float, float, float]
    tip_length_m: float
    recorded_at: float
    tip_offset_m: tuple[float, float, float] = DEFAULT_TIP_OFFSET_M

    def to_dict(self) -> dict:
        return {
            'joints_rad': list(self.joints),
            'joints_deg': [math.degrees(v) for v in self.joints],
            'position_m': list(self.p_bh),
            'position_mm': [v * 1000.0 for v in self.p_bh],
            'quaternion_wxyz': list(self.q_bh),
            'rpy_deg': list(quat_to_rpy_deg(self.q_bh)),
            'tip_length_m': float(self.tip_length_m),
            'tip_offset_m': list(self.tip_offset_m),
            'tip_offset_cm': [v * 100.0 for v in self.tip_offset_m],
            'recorded_at': float(self.recorded_at),
        }


@dataclass(frozen=True)
class PolicyStepResult:
    step: int
    obs: np.ndarray
    actions: np.ndarray
    p_bt: np.ndarray
    q_bt: np.ndarray
    p_ht: np.ndarray
    q_ht: np.ndarray
    wrench_t: np.ndarray
    p_target_h: np.ndarray
    q_target_ht: np.ndarray
    p_target_b: np.ndarray
    q_target_bt: np.ndarray
    q_cmd: tuple[float, ...]
    latency_ms: float
    task_fraction: float = 1.0


class InsertAnythingPolicy:
    def __init__(
        self,
        onnx_path: str | Path | None = None,
        *,
        translation_limit_m: float = ACTION_TRANSLATION_SCALE_M,
        rotation_limit_rad: float = ACTION_ROTATION_SCALE_RAD,
        max_tip_lead_xy_m: float = MAX_TIP_LEAD_XY_M,
        max_tip_lead_z_m: float = MAX_TIP_LEAD_Z_M,
        max_tip_lead_rad: float = MAX_TIP_LEAD_RAD,
        max_goal_tilt_rad: float = MAX_GOAL_TILT_RAD,
        max_joint_step_rad: float = MAX_JOINT_STEP_RAD,
    ) -> None:
        self.onnx_path = (Path(onnx_path) if onnx_path is not None else default_onnx_path()).resolve(strict=True)
        self.translation_limit_m = float(translation_limit_m)
        self.rotation_limit_rad = float(rotation_limit_rad)
        self.max_tip_lead_xy_m = float(max_tip_lead_xy_m)
        self.max_tip_lead_z_m = float(max_tip_lead_z_m)
        self.max_tip_lead_rad = float(max_tip_lead_rad)
        self.max_goal_tilt_rad = float(max_goal_tilt_rad)
        self.max_joint_step_rad = float(max_joint_step_rad)

        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(
            str(self.onnx_path), sess_options=options, providers=['CPUExecutionProvider'],
        )
        self.hidden = np.zeros((2, 1, 128), dtype=np.float32)
        self.steps = 0
        self.cmd_p_h: np.ndarray | None = None
        self.cmd_q_ht: np.ndarray | None = None
        self.cmd_joints: tuple[float, ...] | None = None

    def reset(self) -> None:
        self.hidden.fill(0.0)
        self.steps = 0
        self.cmd_p_h = None
        self.cmd_q_ht = None
        self.cmd_joints = None

    def infer(self, obs: np.ndarray) -> tuple[np.ndarray, float]:
        obs_arr = np.ascontiguousarray(obs, dtype=np.float32)
        if obs_arr.shape != (1, 13) or not np.isfinite(obs_arr).all():
            raise ValueError(f'Invalid observation shape or values: {obs_arr.shape}')
        t0 = time.perf_counter_ns()
        actions, next_hidden = self.session.run(['actions', 'h_out'], {'obs': obs_arr, 'h_in': self.hidden})
        latency_ms = (time.perf_counter_ns() - t0) / 1e6
        if not np.isfinite(actions).all() or not np.isfinite(next_hidden).all():
            raise RuntimeError('Non-finite policy output or hidden state')
        self.hidden = np.ascontiguousarray(next_hidden, dtype=np.float32)
        self.steps += 1
        return np.asarray(actions[0], dtype=np.float64), float(latency_ms)

    def step(
        self, q_meas: Sequence[float], goal: GoalPose, wrench_t: Sequence[float] | np.ndarray,
        *, tip_length_m: float | None = None, tip_offset_m: Sequence[float] | np.ndarray | None = None,
        gravity_model: ArmGravityModel | None = None, goal_pos_h: Sequence[float] = (0.0, 0.0, 0.0),
        goal_quat_h: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
        accumulate_target: bool = True,
    ) -> PolicyStepResult:
        offset_gripper = (
            normalize_tip_offset(tip_offset_m, tip_length_m)
            if (tip_offset_m is not None or tip_length_m is not None)
            else goal.tip_offset_m
        )
        p_bt, q_bt = forward_tip_pose(q_meas, tip_offset_m=offset_gripper, gravity_model=gravity_model)
        p_ht, q_ht = relative_pose_in_hole(p_bt, q_bt, goal.p_bh, goal.q_bh)
        w_t = np.asarray(wrench_t, dtype=np.float64).reshape(6)

        obs = actor_observation(p_ht, q_ht, goal_pos_h, goal_quat_h, w_t)
        actions, latency_ms = self.infer(obs)

        base_p_h = p_ht.copy() if (not accumulate_target or self.cmd_p_h is None) else self.cmd_p_h.copy()
        base_q_ht = q_ht.copy() if (not accumulate_target or self.cmd_q_ht is None) else self.cmd_q_ht.copy()

        p_cand_h, q_cand_ht = action_target(
            base_p_h, base_q_ht, actions,
            translation_limit_m=self.translation_limit_m,
            rotation_limit_rad=self.rotation_limit_rad,
        )

        if accumulate_target:
            lead_p = p_cand_h - p_ht
            xy_norm = float(np.linalg.norm(lead_p[:2]))
            if self.max_tip_lead_xy_m > 0.0 and xy_norm > self.max_tip_lead_xy_m:
                lead_p[:2] *= self.max_tip_lead_xy_m / xy_norm
            if self.max_tip_lead_z_m > 0.0:
                lead_p[2] = float(np.clip(lead_p[2], -self.max_tip_lead_z_m, self.max_tip_lead_z_m))
            p_cand_h = p_ht + lead_p

            lead_r = quat_rotvec(quat_mul(q_cand_ht, quat_inv(q_ht)))
            r_norm = float(np.linalg.norm(lead_r))
            if self.max_tip_lead_rad > 0.0 and r_norm > self.max_tip_lead_rad:
                lead_r *= self.max_tip_lead_rad / r_norm
            q_cand_ht = canonical_quat(quat_mul(rotvec_quat(lead_r), q_ht))

            goal_q_can = canonical_quat(goal_quat_h)
            tilt_r = quat_rotvec(quat_mul(q_cand_ht, quat_inv(goal_q_can)))
            tilt_norm = float(np.linalg.norm(tilt_r))
            if self.max_goal_tilt_rad > 0.0 and tilt_norm > self.max_goal_tilt_rad:
                q_cand_ht = canonical_quat(quat_mul(rotvec_quat(tilt_r * (self.max_goal_tilt_rad / tilt_norm)), goal_q_can))

        q_seed = tuple(float(v) for v in q_meas[:6]) if self.cmd_joints is None else self.cmd_joints
        p_target_b, q_target_bt = hole_target_to_base(p_cand_h, q_cand_ht, goal.p_bh, goal.q_bh)
        q_sol = solve_tip_ik(q_seed, p_target_b, q_target_bt, tip_offset_m=offset_gripper, gravity_model=gravity_model)

        task_fraction = 1.0
        if accumulate_target and self.max_joint_step_rad > 0.0:
            max_dq = float(np.max(np.abs(np.asarray(q_sol) - np.asarray(q_seed))))
            if max_dq > self.max_joint_step_rad:
                task_fraction = float(self.max_joint_step_rad / max_dq)
                p_target_h = base_p_h + task_fraction * (p_cand_h - base_p_h)
                dr_step = quat_rotvec(quat_mul(q_cand_ht, quat_inv(base_q_ht))) * task_fraction
                q_target_ht = canonical_quat(quat_mul(rotvec_quat(dr_step), base_q_ht))
                p_target_b, q_target_bt = hole_target_to_base(p_target_h, q_target_ht, goal.p_bh, goal.q_bh)
                q_cmd = solve_tip_ik(q_seed, p_target_b, q_target_bt, tip_offset_m=offset_gripper, gravity_model=gravity_model)
            else:
                p_target_h, q_target_ht, q_cmd = p_cand_h, q_cand_ht, q_sol
        else:
            p_target_h, q_target_ht, q_cmd = p_cand_h, q_cand_ht, q_sol

        self.cmd_p_h = p_target_h.copy()
        self.cmd_q_ht = q_target_ht.copy()
        self.cmd_joints = q_cmd

        return PolicyStepResult(
            step=self.steps,
            obs=obs[0],
            actions=actions,
            p_bt=p_bt,
            q_bt=q_bt,
            p_ht=p_ht,
            q_ht=q_ht,
            wrench_t=w_t,
            p_target_h=p_target_h,
            q_target_ht=q_target_ht,
            p_target_b=p_target_b,
            q_target_bt=q_target_bt,
            q_cmd=q_cmd,
            latency_ms=latency_ms,
            task_fraction=task_fraction,
        )
