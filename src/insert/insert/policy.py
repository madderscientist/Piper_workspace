"""InsertAnything ONNX policy inference, frame transforms, wrench filtering, and tip IK."""

from dataclasses import dataclass
import math
from pathlib import Path
import time
from typing import Sequence

import numpy as np
import onnxruntime as ort

from piper_x_driver.gravity import (
    ArmGravityModel,
    end_effector_wrench,
    forward_kinematics,
    geometric_jacobian,
    inverse_kinematics,
)

POLICY_PERIOD_S = 0.02
ACTION_TRANSLATION_SCALE_M = 0.001
ACTION_ROTATION_SCALE_RAD = math.radians(0.5)

JOINT_LOWER = np.array([-2.6179938779914944, 0.0, -2.9670597283903604, -1.5533430342749532, -1.5533430342749532, -2.9670597283903604], dtype=np.float64)
JOINT_UPPER = np.array([2.6179938779914944, 3.141592653589793, 0.0, 1.5533430342749532, 1.5533430342749532, 2.9670597283903604], dtype=np.float64)

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
    """Exact exponential discretization preserves the physical time constant."""
    if any(not math.isfinite(v) or v <= 0.0 for v in (tau_ref, dt_ref, dt_target)):
        raise ValueError('tau_ref, dt_ref, and dt_target must be positive')
    return float(tau_ref)


WRENCH_FILTER_TAU_S = equivalent_filter_tau()
DEFAULT_TIP_LENGTH_M = 0.195
DEFAULT_TIP_OFFSET_M: tuple[float, float, float] = (0.0, 0.0, DEFAULT_TIP_LENGTH_M)
# PX6D-75A measurement center along +Z_link6/+Z_gripper: 7.5 mm arm flange + 12.0 mm half sensor height
PAXINI_SENSOR_OFFSET_M: tuple[float, float, float] = (0.0, 0.0, 0.0195)

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
    if q_can[0] < 0.0:
        q_can = -q_can
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
    position: Sequence[float] | np.ndarray, tip_length_m: float | None = None,
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
    t_base_tip = forward_kinematics(q_eff, offset_link6)
    return t_base_tip[:3, 3].copy(), mat_to_quat(t_base_tip[:3, :3] @ _R_LINK6_TO_T)


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


def _inv_sym3(a: np.ndarray, lam2: float) -> np.ndarray:
    a00 = float(a[0, 0]) + lam2
    a01 = float(a[0, 1])
    a02 = float(a[0, 2])
    a11 = float(a[1, 1]) + lam2
    a12 = float(a[1, 2])
    a22 = float(a[2, 2]) + lam2
    c00 = a11 * a22 - a12 * a12
    c01 = a02 * a12 - a01 * a22
    c02 = a01 * a12 - a02 * a11
    c11 = a00 * a22 - a02 * a02
    c12 = a01 * a02 - a00 * a12
    c22 = a00 * a11 - a01 * a01
    inv_det = 1.0 / (a00 * c00 + a01 * c01 + a02 * c02)
    return np.array([
        [c00 * inv_det, c01 * inv_det, c02 * inv_det],
        [c01 * inv_det, c11 * inv_det, c12 * inv_det],
        [c02 * inv_det, c12 * inv_det, c22 * inv_det],
    ], dtype=np.float64)


def solve_tip_ik(
    q_init: Sequence[float] | np.ndarray, p_target_b: Sequence[float] | np.ndarray, q_target_bt: Sequence[float] | np.ndarray,
    *, tip_length_m: float | None = None, tip_offset_m: Sequence[float] | np.ndarray | None = None,
    gravity_model: ArmGravityModel | None = None, damping: float = 1e-3, max_iters: int = 20,
) -> tuple[float, ...]:
    if len(q_init) < 6:
        raise ValueError('Six initial joint positions are required')
    target_link6 = np.eye(4, dtype=np.float64)
    target_link6[:3, 3] = np.asarray(p_target_b, dtype=np.float64).reshape(3)
    target_link6[:3, :3] = quat_to_mat(q_target_bt) @ _R_LINK6_TO_T
    tool_offset = _gripper_offset_to_link6(normalize_tip_offset(tip_offset_m, tip_length_m))
    solver = inverse_kinematics if gravity_model is None else gravity_model.inverse_kinematics
    res = solver(
        target_link6,
        q_init[:6],
        tool_offset_m=tool_offset,
        lower_limits=JOINT_LOWER,
        upper_limits=JOINT_UPPER,
        damping=damping,
        max_iters=max_iters,
        pos_tol_m=1e-7,
        rot_tol_rad=1e-7,
    )
    assert isinstance(res, tuple)
    return res


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
    lam2 = max(float(damping) * float(damping), 0.0)
    jjt = j0 @ j0.T
    b0 = (jjt + lam2 * _EYE6) @ w0
    reg2 = max(lam2 * 1e-2, 1e-6)
    tau_ext = j0.T @ np.linalg.solve(jjt + reg2 * _EYE6, b0)
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
    def __init__(
        self,
        tau: float | None = None,
        *,
        tau_1khz: float = TRAIN_WRENCH_FILTER_TAU_S,
        sensor_offset_m: Sequence[float] | np.ndarray = (0.0, 0.0, 0.0),
    ) -> None:
        tau = tau_1khz if tau is None else tau
        if tau <= 0.0 or not math.isfinite(tau):
            raise ValueError('tau must be finite and positive')
        sensor_off = np.asarray(sensor_offset_m, dtype=np.float64).reshape(3)
        if not np.isfinite(sensor_off).all():
            raise ValueError('sensor_offset_m must contain finite numbers')
        self.tau = float(tau)
        self.sensor_offset_m: tuple[float, float, float] = (
            float(sensor_off[0]), float(sensor_off[1]), float(sensor_off[2]),
        )
        self.sample_dt: float | None = None
        self.alpha: float | None = None
        self.bias_s = np.zeros(6, dtype=np.float64)
        self.raw_s = np.zeros(6, dtype=np.float64)
        self.filtered_s = np.zeros(6, dtype=np.float64)
        self._last_stamp: float | None = None
        self._initialized = False
        self._already_at_tip = False

    @property
    def tau_1khz(self) -> float:
        return self.tau

    def reset(self) -> None:
        self.filtered_s.fill(0.0)
        self._last_stamp = None
        self._initialized = False
        self.sample_dt = None
        self.alpha = None

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
        at_tip = bool(already_at_tip)
        if self._initialized and at_tip != self._already_at_tip:
            self.clear_tare()
        self.raw_s = arr.copy()
        self._already_at_tip = at_tip
        compensated = arr - self.bias_s
        if timestamp is not None:
            if dt is None and self._last_stamp is not None:
                dt = float(timestamp) - self._last_stamp
            self._last_stamp = float(timestamp)
        self.sample_dt = float(dt) if dt is not None and math.isfinite(dt) and dt > 0.0 else None
        self.alpha = -math.expm1(-self.sample_dt / self.tau) if self.sample_dt is not None else None
        if not self._initialized or self.alpha is None:
            self.filtered_s = compensated.copy()
            self._initialized = True
        else:
            self.filtered_s += self.alpha * (compensated - self.filtered_s)
        return self.filtered_s.copy()

    def tip_wrench(
        self, tip_length_m: float | None = None, *, tip_offset_m: Sequence[float] | np.ndarray | None = None,
    ) -> np.ndarray:
        if self._already_at_tip:
            f = self.filtered_s
            return np.array([f[0], -f[1], -f[2], f[3], -f[4], -f[5]], dtype=np.float64)
        offset_gripper = normalize_tip_offset(tip_offset_m, tip_length_m)
        lever_gripper = (
            offset_gripper[0] - self.sensor_offset_m[0],
            offset_gripper[1] - self.sensor_offset_m[1],
            offset_gripper[2] - self.sensor_offset_m[2],
        )
        return transform_wrench(
            self.filtered_s[:3], self.filtered_s[3:6], _Q_T_FROM_GRIPPER, rotate(_Q_T_FROM_GRIPPER, lever_gripper),
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
    compute_ms: float = 0.0
    v_cmd: tuple[float, ...] = (0.0,) * 6


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
        actions_arr = np.asarray(actions, dtype=np.float64)
        hidden_arr = np.ascontiguousarray(next_hidden, dtype=np.float32)
        if not np.isfinite(actions_arr).all() or not np.isfinite(hidden_arr).all():
            raise RuntimeError('Non-finite policy output or hidden state')
        self.hidden = hidden_arr
        self.steps += 1
        return actions_arr[0], float(latency_ms)

    def step(
        self, q_meas: Sequence[float] | np.ndarray, goal: GoalPose, wrench_t: Sequence[float] | np.ndarray,
        *, tip_length_m: float | None = None, tip_offset_m: Sequence[float] | np.ndarray | None = None,
        gravity_model: ArmGravityModel | None = None, goal_pos_h: Sequence[float] = (0.0, 0.0, 0.0),
        goal_quat_h: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
        accumulate_target: bool = False,
    ) -> PolicyStepResult:
        start = time.perf_counter_ns()
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

        q_seed = (
            tuple(float(v) for v in q_meas[:6])
            if (not accumulate_target or self.cmd_joints is None)
            else self.cmd_joints
        )
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
            compute_ms=(time.perf_counter_ns() - start) / 1e6,
        )


class RuleBasedInsertPolicy:
    """Closed-loop rule-based insertion: (1) PI-align XY & orientation at safe Z, (2) force-regulated vertical press + micro-spiral if rim-blocked."""

    def __init__(
        self,
        *,
        align_xy_speed_m_s: float = 0.020,
        align_z_speed_m_s: float = 0.020,
        align_rot_speed_rad_s: float = math.radians(20.0),
        press_z_speed_m_s: float = 0.020,
        min_align_z_m: float = 0.015,
        press_overdrive_z_m: float = -0.006,
        align_xy_tol_m: float = 0.0010,
        align_ang_tol_rad: float = math.radians(1.0),
        align_settle_steps: int = 25,
        ki_xy: float = 1.8,
        ki_rot: float = 1.5,
        max_int_xy_m: float = 0.007,
        max_int_rot_rad: float = math.radians(4.0),
        rim_contact_force_n: float = 1.8,
        max_press_lead_z_m: float = 0.012,
        max_press_force_n: float = 15.0,
        spiral_speed_m_s: float = 0.005,
        spiral_pitch_m: float = 0.0005,
        spiral_max_radius_m: float = 0.0035,
        spiral_kp_xy: float = 1.0,
        spiral_press_force_n: float = 2.2,
    ) -> None:
        self.align_xy_speed_m_s = float(align_xy_speed_m_s)
        self.align_z_speed_m_s = float(align_z_speed_m_s)
        self.align_rot_speed_rad_s = float(align_rot_speed_rad_s)
        self.press_z_speed_m_s = float(press_z_speed_m_s)
        self.min_align_z_m = float(min_align_z_m)
        self.press_overdrive_z_m = float(press_overdrive_z_m)
        self.align_xy_tol_m = float(align_xy_tol_m)
        self.align_ang_tol_rad = float(align_ang_tol_rad)
        self.align_settle_steps = int(align_settle_steps)
        self.ki_xy = float(ki_xy)
        self.ki_rot = float(ki_rot)
        self.max_int_xy_m = float(max_int_xy_m)
        self.max_int_rot_rad = float(max_int_rot_rad)
        self.rim_contact_force_n = float(rim_contact_force_n)
        self.max_press_lead_z_m = float(max_press_lead_z_m)
        self.max_press_force_n = float(max_press_force_n)
        self.spiral_speed_m_s = float(spiral_speed_m_s)
        self.spiral_pitch_m = float(spiral_pitch_m)
        self.spiral_max_radius_m = float(spiral_max_radius_m)
        self.spiral_kp_xy = float(spiral_kp_xy)
        self.spiral_press_force_n = float(spiral_press_force_n)
        self.reset()

    def reset(self) -> None:
        self.steps = 0
        self.stage = 'align'
        self.align_z_m = self.min_align_z_m
        self.align_cmd_done_steps = 0
        self.align_in_tol_steps = 0
        self.stall_steps = 0
        self.spiral_phase = 0.0
        self.best_z_m = 1.0
        self.rim_z_m: float | None = None
        self.entry_xy: np.ndarray | None = None
        self.int_xy = np.zeros(2, dtype=np.float64)
        self.int_rot = np.zeros(3, dtype=np.float64)
        self.cmd_p_h: np.ndarray | None = None
        self.cmd_q_ht: np.ndarray | None = None
        self.cmd_joints: tuple[float, ...] | None = None
        self.cmd_vel: np.ndarray = np.zeros(6, dtype=np.float64)

    def step(
        self, q_meas: Sequence[float] | np.ndarray, goal: GoalPose, wrench_t: Sequence[float] | np.ndarray,
        *, tip_length_m: float | None = None, tip_offset_m: Sequence[float] | np.ndarray | None = None,
        gravity_model: ArmGravityModel | None = None, goal_pos_h: Sequence[float] = (0.0, 0.0, 0.0),
        goal_quat_h: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
        dt: float = POLICY_PERIOD_S,
    ) -> PolicyStepResult:
        start = time.perf_counter_ns()
        offset_gripper = (
            normalize_tip_offset(tip_offset_m, tip_length_m)
            if (tip_offset_m is not None or tip_length_m is not None)
            else goal.tip_offset_m
        )
        p_bt, q_bt = forward_tip_pose(q_meas, tip_offset_m=offset_gripper, gravity_model=gravity_model)
        p_ht, q_ht = relative_pose_in_hole(p_bt, q_bt, goal.p_bh, goal.q_bh)
        w_t = np.asarray(wrench_t, dtype=np.float64).reshape(6)
        obs = actor_observation(p_ht, q_ht, goal_pos_h, goal_quat_h, w_t)
        goal_p = np.asarray(goal_pos_h, dtype=np.float64).reshape(3)
        goal_q = canonical_quat(goal_quat_h)

        if self.cmd_p_h is None or self.cmd_q_ht is None:
            self.cmd_p_h = p_ht.copy()
            self.cmd_q_ht = q_ht.copy()
            self.best_z_m = float(p_ht[2])
            xy_init = float(np.linalg.norm(p_ht[:2] - goal_p[:2]))
            ang_init = float(np.linalg.norm(quat_rotvec(quat_mul(q_ht, quat_inv(goal_q)))))
            if xy_init <= self.align_xy_tol_m and ang_init <= self.align_ang_tol_rad:
                self.align_z_m = float(p_ht[2])
                self.stage = 'press'
            else:
                self.align_z_m = max(float(p_ht[2]), self.min_align_z_m)
                self.stage = 'align'

        self.steps += 1
        ref_xy = goal_p[:2].copy()
        kp_xy_eff = 1.0
        ki_xy_eff = self.ki_xy
        v_des_h = np.zeros(3, dtype=np.float64)
        if self.stage == 'align':
            # Move XY toward goal_p[:2] and Z toward align_z_m while rotating toward goal_q
            err_xy = goal_p[:2] - self.cmd_p_h[:2]
            dist_xy = float(np.linalg.norm(err_xy))
            max_xy = self.align_xy_speed_m_s * dt
            if dist_xy <= max_xy:
                self.cmd_p_h[:2] = goal_p[:2]
            else:
                self.cmd_p_h[:2] += err_xy * (max_xy / dist_xy)
                v_des_h[:2] = err_xy * (self.align_xy_speed_m_s / dist_xy)

            err_z = self.align_z_m - float(self.cmd_p_h[2])
            max_z = self.align_z_speed_m_s * dt
            if abs(err_z) <= max_z:
                self.cmd_p_h[2] = self.align_z_m
            else:
                self.cmd_p_h[2] += math.copysign(max_z, err_z)
                v_des_h[2] = math.copysign(self.align_z_speed_m_s, err_z)

            err_r = quat_rotvec(quat_mul(goal_q, quat_inv(self.cmd_q_ht)))
            dist_r = float(np.linalg.norm(err_r))
            max_r = self.align_rot_speed_rad_s * dt
            if dist_r <= max_r:
                self.cmd_q_ht = goal_q.copy()
            else:
                self.cmd_q_ht = canonical_quat(quat_mul(rotvec_quat(err_r * (max_r / dist_r)), self.cmd_q_ht))

            cmd_xy_done = float(np.linalg.norm(self.cmd_p_h[:2] - goal_p[:2])) <= 1e-5
            cmd_z_done = abs(float(self.cmd_p_h[2]) - self.align_z_m) <= 1e-5
            cmd_r_done = float(np.linalg.norm(quat_rotvec(quat_mul(goal_q, quat_inv(self.cmd_q_ht))))) <= 1e-4
            if cmd_xy_done and cmd_z_done and cmd_r_done:
                self.align_cmd_done_steps += 1
            meas_xy_err = float(np.linalg.norm(p_ht[:2] - goal_p[:2]))
            meas_ang_err = float(np.linalg.norm(quat_rotvec(quat_mul(q_ht, quat_inv(goal_q)))))
            if meas_xy_err <= self.align_xy_tol_m and meas_ang_err <= self.align_ang_tol_rad:
                self.align_in_tol_steps += 1
            else:
                self.align_in_tol_steps = 0
            if (cmd_xy_done and cmd_z_done and cmd_r_done) and (
                self.align_in_tol_steps >= 5 or self.align_cmd_done_steps >= self.align_settle_steps
            ):
                self.stage = 'press'
                self.cmd_p_h[:2] = goal_p[:2]
                self.cmd_q_ht = goal_q.copy()
                self.best_z_m = float(p_ht[2])
                self.rim_z_m = None
                self.entry_xy = None
        else:
            # Stage 2 ('press'): lock nominal XY and orientation, lower Z with force-regulated lead
            self.cmd_p_h[:2] = goal_p[:2]
            self.cmd_q_ht = goal_q.copy()
            target_z = float(goal_p[2]) + self.press_overdrive_z_m
            fz = float(w_t[2])
            z_curr = float(p_ht[2])
            dz = self.press_z_speed_m_s * dt
            # Require sustained upward contact force AND arrested descent before latching rim height
            rim_blocked = fz >= self.rim_contact_force_n and (z_curr - float(self.cmd_p_h[2])) >= 0.0015
            if z_curr < self.best_z_m - 0.0004:
                self.best_z_m = z_curr
                self.stall_steps = 0
            elif z_curr > 0.0015 and rim_blocked:
                self.stall_steps += 1
            if self.rim_z_m is None and self.stall_steps >= 10 and z_curr > 0.0015:
                self.rim_z_m = z_curr

            on_rim = self.rim_z_m is None or z_curr > self.rim_z_m - 0.0015
            spiraling = self.stall_steps >= 10 and z_curr > 0.0015 and on_rim
            if not on_rim and self.spiral_phase > 0.0:
                if self.entry_xy is None:
                    self.entry_xy = p_ht[:2].copy()
                self.int_xy *= 0.85
                ref_xy = self.entry_xy.copy()
            elif not spiraling and self.spiral_phase > 0.0:
                # Hold current XY while dropping into the hole mouth after spiral catch
                self.int_xy *= 0.9
                ref_xy = p_ht[:2].copy()
            elif spiraling:
                # Lag-gated Archimedes spiral: wait for physical tip to break stiction before sweeping phase
                radius = min(
                    self.spiral_max_radius_m,
                    0.0005 + self.spiral_pitch_m * (self.spiral_phase / (2.0 * math.pi)),
                )
                ref_xy = goal_p[:2] + np.array([
                    radius * math.cos(self.spiral_phase),
                    radius * math.sin(self.spiral_phase),
                ], dtype=np.float64)
                xy_lag = float(np.linalg.norm(ref_xy - p_ht[:2]))
                speed_scale = 1.0 / (1.0 + (xy_lag / 0.0008) ** 2)
                self.spiral_phase += (self.spiral_speed_m_s * dt * speed_scale) / max(radius, 0.0006)
                radius = min(
                    self.spiral_max_radius_m,
                    0.0005 + self.spiral_pitch_m * (self.spiral_phase / (2.0 * math.pi)),
                )
                ref_xy = goal_p[:2] + np.array([
                    radius * math.cos(self.spiral_phase),
                    radius * math.sin(self.spiral_phase),
                ], dtype=np.float64)
                kp_xy_eff = self.spiral_kp_xy
                ki_xy_eff = self.ki_xy
                # Regulate axial contact force around spiral_press_force_n without retracting into Z stiction deadband
                min_cmd_z = max(target_z, z_curr - self.max_press_lead_z_m)
                if fz < self.spiral_press_force_n:
                    self.cmd_p_h[2] = max(min_cmd_z, float(self.cmd_p_h[2]) - 0.5 * dz)
                elif fz > self.spiral_press_force_n + 1.2:
                    self.cmd_p_h[2] = min(z_curr, float(self.cmd_p_h[2]) + 0.3 * dz)

            if not spiraling and fz < self.max_press_force_n:
                if fz >= 1.0:
                    min_cmd_z = max(target_z, z_curr - self.max_press_lead_z_m)
                else:
                    # Uninterrupted linear Z ramp in free air so position target never stalls on Z stiction
                    min_cmd_z = target_z
                next_z = max(min_cmd_z, float(self.cmd_p_h[2]) - dz)
                if next_z < float(self.cmd_p_h[2]) - 1e-9 and fz < self.rim_contact_force_n:
                    v_des_h[2] = -self.press_z_speed_m_s
                self.cmd_p_h[2] = next_z

        # Closed-loop Cartesian integral action to overcome residual joint stiction in XY and orientation
        track_err_xy = (self.cmd_p_h[:2] if self.stage == 'align' else ref_xy) - p_ht[:2]
        self.int_xy += ki_xy_eff * track_err_xy * dt
        int_xy_norm = float(np.linalg.norm(self.int_xy))
        if int_xy_norm > self.max_int_xy_m > 0.0:
            self.int_xy *= self.max_int_xy_m / int_xy_norm

        track_err_r = quat_rotvec(quat_mul(self.cmd_q_ht, quat_inv(q_ht)))
        self.int_rot += self.ki_rot * track_err_r * dt
        int_r_norm = float(np.linalg.norm(self.int_rot))
        if int_r_norm > self.max_int_rot_rad > 0.0:
            self.int_rot *= self.max_int_rot_rad / int_r_norm

        p_target_h = self.cmd_p_h.copy()
        if self.stage == 'align':
            p_target_h[:2] = self.cmd_p_h[:2] + self.int_xy
        else:
            p_target_h[:2] = ref_xy + (kp_xy_eff - 1.0) * track_err_xy + self.int_xy
        q_target_ht = canonical_quat(quat_mul(rotvec_quat(self.int_rot), self.cmd_q_ht))
        dp_h = p_target_h - p_ht
        dr_h = quat_rotvec(quat_mul(q_target_ht, quat_inv(q_ht)))
        actions = np.concatenate([
            dp_h / ACTION_TRANSLATION_SCALE_M,
            dr_h / ACTION_ROTATION_SCALE_RAD,
        ])
        p_target_b, q_target_bt = hole_target_to_base(p_target_h, q_target_ht, goal.p_bh, goal.q_bh)
        prev_cmd = self.cmd_joints
        q_seed = tuple(float(v) for v in q_meas[:6]) if prev_cmd is None else prev_cmd
        q_cmd = solve_tip_ik(
            q_seed, p_target_b, q_target_bt,
            tip_offset_m=offset_gripper, gravity_model=gravity_model, max_iters=10,
        )
        if float(v_des_h @ v_des_h) < 1e-12:
            self.cmd_vel.fill(0.0)
        else:
            # Analytical 6-DOF pure-translation Jacobian velocity feedforward (omega_tip = 0)
            v_des_b = rotate(goal.q_bh, v_des_h)
            tool_off = _gripper_offset_to_link6(offset_gripper)
            jac_b = (
                geometric_jacobian(q_cmd, tool_offset_m=tool_off, frame='base_link')
                if gravity_model is None
                else gravity_model.jacobian(q_cmd, tool_offset_m=tool_off, frame='base_link')
            )
            jv, jw = jac_b[:3, :], jac_b[3:6, :]
            pv = jv.T @ _inv_sym3(jv @ jv.T, 1e-4)
            dq_p = pv @ v_des_b
            jw_null = jw - (jw @ pv) @ jv
            dq_r = jw_null.T @ (_inv_sym3(jw_null @ jw_null.T, 1e-3) @ (-jw @ dq_p))
            self.cmd_vel = dq_p + (dq_r - pv @ (jv @ dq_r))
        self.cmd_joints = q_cmd
        compute_ms = (time.perf_counter_ns() - start) / 1e6
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
            latency_ms=compute_ms,
            task_fraction=1.0,
            compute_ms=compute_ms,
            v_cmd=tuple(float(v) for v in self.cmd_vel),
        )
