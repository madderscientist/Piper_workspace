"""Multi-pose zero-bias and tool-gravity calibration for PaXini PX6D sensors on PiPER-X."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from functools import lru_cache
from typing import Any, Optional, Sequence, Tuple

import numpy as np
from piper_x_driver.gravity import link6_rotation_in_base

from .protocol import WrenchSample

GRAVITY_ACCEL = 9.81


class WrenchStabilityWindow:
    """Check raw wrench peak-to-peak ranges over a rolling time window."""

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
        # Keep one sample at or just before the window boundary for full coverage.
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


# Real five-pose arm3 gravity data matches the SDK link6 axes on this assembly.
# The display URDF joint chain is different and must not be used with SDK joint feedback.
def sensor_rotation_in_base(q: Sequence[float] | np.ndarray) -> np.ndarray:
    """Return the sensor channel rotation using the shared SDK-compatible kinematics."""
    return link6_rotation_in_base(q)


@lru_cache(maxsize=256)
def _gravity_direction(q: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(float(v) for v in -sensor_rotation_in_base(q)[2, :])


def gravity_unit_in_sensor(q: Sequence[float] | np.ndarray) -> np.ndarray:
    """Return unit downward gravity direction g_hat_S = R_BS(q)^T [0, 0, -1]^T in wrench_sensor frame."""
    # Key on all measured angles and return independent storage to callers.
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
    """Return 6-axis gravity wrench (fx, fy, fz, mx, my, mz) of downstream tool in sensor frame."""
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


def compensate_wrench(
    raw: WrenchSample,
    tare_offset: Sequence[float],
    q: Optional[Sequence[float]] = None,
    tool_mass_kg: float = 0.0,
    tool_com_m: Sequence[float] = (0.0, 0.0, 0.0),
) -> WrenchSample:
    """Subtract calibrated zero-bias and (when joint angles q are available) tool gravity wrench."""
    if q is not None and len(q) >= 6 and tool_mass_kg > 0.0:
        gw = tool_gravity_wrench(q, tool_mass_kg, tool_com_m)
        return WrenchSample(
            raw.fx - (tare_offset[0] + gw[0]),
            raw.fy - (tare_offset[1] + gw[1]),
            raw.fz - (tare_offset[2] + gw[2]),
            raw.mx - (tare_offset[3] + gw[3]),
            raw.my - (tare_offset[4] + gw[4]),
            raw.mz - (tare_offset[5] + gw[5]),
        )
    return raw.minus(tare_offset)


@dataclass(frozen=True)
class CalibrationResult:
    """Solved sensor zero-bias and downstream tool gravity model."""

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
    """Fit f=b_f+W*g and t=b_t-[g]_x*p, with W=m*9.81 and p=W*CoM.

    Use mean gravity directions over each sample window when available; R(mean(q))
    generally differs from mean(R(q)). Rank diagnostics expose unobservable parameters.
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
        # This is the exact constrained LS solution for W >= 0: at W=0,
        # the optimal bias is the observed mean, not the unconstrained intercept.
        f_bias = force.mean(axis=0) - weight * g.mean(axis=0)
    else:
        # Constant orientation cannot separate mass and bias. Choose W=0 from
        # the equivalent solution family and report mass_identifiable=False.
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
