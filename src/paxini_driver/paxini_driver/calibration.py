"""PiPER-X 上帕西尼 PX6D 六维力传感器的多姿态常数零偏与下游工具重力标定求解。"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from functools import lru_cache
from typing import Any, Optional, Sequence, Tuple

import numpy as np
from piper_x_driver.gravity import link6_rotation_in_base

GRAVITY_ACCEL = 9.81


class WrenchStabilityWindow:
    """在滑动时间窗口内检查六通道原始力的峰峰值波动，判定机械臂到位后读数是否已稳定。"""

    def __init__(self, duration_sec: float, ranges: Sequence[float]) -> None:
        self.duration_sec = float(duration_sec)
        self.ranges = np.asarray(ranges, dtype=float)
        self.samples = deque()

    def reset(self) -> None:
        self.samples.clear()

    def update(self, wrench: Sequence[float], now: float) -> None:
        if self.samples and now - self.samples[-1][0] > self.duration_sec:
            self.reset()
        self.samples.append((now, tuple(wrench[:6])))
        cutoff = now - self.duration_sec
        # 保留窗口左边界处或稍早的一帧样本，保证时间跨度完整覆盖 duration_sec。
        while len(self.samples) > 2 and self.samples[1][0] <= cutoff:
            self.samples.popleft()

    def is_stable(self, now: float) -> bool:
        if len(self.samples) < 2:
            return False
        first, last = self.samples[0][0], self.samples[-1][0]
        if last - first + 1e-9 < self.duration_sec or now - last > self.duration_sec:
            return False
        positions = np.asarray([sample[1] for sample in self.samples])
        return bool(np.all(np.ptp(positions, axis=0) <= self.ranges + 1e-12))


# 实测五姿态重力数据验证表明：传感器通道方向与 SDK 兼容的 link6 坐标轴一致；
# 显示用 URDF 关节链与 SDK 关节反馈定义不同，不可用于此处重力投影。
def sensor_rotation_in_base(q: Sequence[float] | np.ndarray) -> np.ndarray:
    """按共享的 SDK 兼容正运动学返回基座系下传感器测量坐标系的旋转矩阵。"""
    return link6_rotation_in_base(q)


@lru_cache(maxsize=256)
def _gravity_direction(q: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(float(v) for v in -sensor_rotation_in_base(q)[2, :])


def gravity_unit_in_sensor(q: Sequence[float] | np.ndarray) -> np.ndarray:
    """返回传感器坐标系下的单位向下重力方向向量 g_hat_S = R_BS(q)^T [0, 0, -1]^T。"""
    return np.asarray(_gravity_direction(tuple(float(v) for v in q[:6])))


def _skew(v: np.ndarray) -> np.ndarray:
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    return np.array(
        [
            [0.0, -vz, vy],
            [vz, 0.0, -vx],
            [-vy, vx, 0.0],
        ],
        dtype=float,
    )


def tool_gravity_wrench(
    q: Sequence[float],
    tool_mass_kg: float,
    tool_com_m: Sequence[float],
) -> Tuple[float, float, float, float, float, float]:
    """返回传感器下游工具（夹爪等）在当前姿态 q 下对传感器参考点产生的六维重力与重力矩。"""
    mass = float(tool_mass_kg)
    if mass <= 0.0 or len(q) < 6:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    gx, gy, gz = _gravity_direction(tuple(float(v) for v in q[:6]))
    scale = mass * GRAVITY_ACCEL
    fx, fy, fz = scale * gx, scale * gy, scale * gz
    cx, cy, cz = float(tool_com_m[0]), float(tool_com_m[1]), float(tool_com_m[2])
    return (
        fx,
        fy,
        fz,
        cy * fz - cz * fy,
        cz * fx - cx * fz,
        cx * fy - cy * fx,
    )


@dataclass(frozen=True)
class CalibrationResult:
    """多姿态最小二乘解算得到的传感器常数零偏、下游工具质量/质心及可辨识性指标。"""

    tare_offset: Tuple[float, float, float, float, float, float]
    tool_mass_kg: float
    tool_com_m: Tuple[float, float, float]
    num_poses: int
    force_rms_n: float
    torque_rms_nm: float
    force_rank: int
    torque_rank: int

    def to_dict(self) -> dict[str, Any]:
        return {
            "tare_offset": [round(float(x), 6) for x in self.tare_offset],
            "tool_mass_kg": round(float(self.tool_mass_kg), 6),
            "tool_com_m": [round(float(x), 6) for x in self.tool_com_m],
            "num_poses": int(self.num_poses),
            "force_rms_n": round(float(self.force_rms_n), 4),
            "torque_rms_nm": round(float(self.torque_rms_nm), 5),
            "force_rank": self.force_rank,
            "torque_rank": self.torque_rank,
            "mass_identifiable": self.force_rank == 4,
            "com_identifiable": self.tool_mass_kg > 0.0 and self.torque_rank == 6,
        }


def solve_wrench_calibration(
    poses: Sequence[Sequence[float] | np.ndarray] | np.ndarray,
    wrenches: Sequence[Sequence[float] | np.ndarray] | np.ndarray,
    *,
    gravity_directions: Optional[Sequence[Sequence[float] | np.ndarray] | np.ndarray] = None,
) -> CalibrationResult:
    """联合拟合 F = b_F + W*d 与 M = b_M - [d]_x * p（W = m*9.81，p = W*c），分离常数零偏与工具重力。

    优先使用每个采样窗口内逐帧计算再取平均的重力方向 d，因为非线性旋转满足 R(mean(q)) != mean(R(q))；
    同时输出设计矩阵秩，明确报告工具质量与质心在所给姿态集合下是否可辨识。
    """
    n = len(poses)
    if n == 0 or n != len(wrenches):
        raise ValueError(
            "poses and wrenches must be non-empty and have the same length"
        )
    observed = np.asarray(wrenches, dtype=float).reshape(n, 6)
    g = np.asarray(
        gravity_directions
        if gravity_directions is not None
        else [gravity_unit_in_sensor(q) for q in poses],
        dtype=float,
    ).reshape(n, 3)
    if not np.isfinite(observed).all() or not np.isfinite(g).all():
        raise ValueError("Calibration measurements must be finite")
    force, torque = observed[:, :3], observed[:, 3:]
    a_f = np.vstack([np.column_stack((np.eye(3), direction)) for direction in g])
    force_rank = int(np.linalg.matrix_rank(a_f))
    if force_rank == 4:
        x_f = np.linalg.lstsq(a_f, force.reshape(-1), rcond=None)[0]
        weight = max(0.0, float(x_f[3]))
        # 带物理约束 W >= 0 的精确最小二乘解：若截断到 W=0，最优零偏退化为观测均值。
        f_bias = force.mean(axis=0) - weight * g.mean(axis=0)
    else:
        # 姿态重力方向缺乏变化时无法分离常数零偏与工具质量，取等价解族中的 W=0 并标记不可辨识。
        weight = 0.0
        f_bias = force.mean(axis=0)
    if weight > 0.0:
        a_t = np.vstack(
            [np.column_stack((np.eye(3), -_skew(direction))) for direction in g]
        )
        x_t, _, torque_rank, _ = np.linalg.lstsq(a_t, torque.reshape(-1), rcond=None)
        t_bias, moment = x_t[:3], x_t[3:]
        com = moment / weight
    else:
        torque_rank = 3
        t_bias = torque.mean(axis=0)
        com = np.zeros(3)
    force_error = force - (f_bias + weight * g)
    torque_error = torque - (t_bias + np.cross(com, weight * g))
    tare_6 = (
        float(f_bias[0]),
        float(f_bias[1]),
        float(f_bias[2]),
        float(t_bias[0]),
        float(t_bias[1]),
        float(t_bias[2]),
    )
    com_3 = (float(com[0]), float(com[1]), float(com[2]))
    return CalibrationResult(
        tare_offset=tare_6,
        tool_mass_kg=weight / GRAVITY_ACCEL,
        tool_com_m=com_3,
        num_poses=n,
        force_rms_n=float(np.sqrt(np.mean(force_error**2))),
        torque_rms_nm=float(np.sqrt(np.mean(torque_error**2))),
        force_rank=force_rank,
        torque_rank=int(torque_rank),
    )
