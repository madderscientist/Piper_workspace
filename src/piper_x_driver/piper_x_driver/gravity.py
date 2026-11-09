"""Kinematics, gravity/inertia models, and 6-axis wrench estimation for an upright PiPER-X.

Inertial parameters from agilexrobotics/agx_arm_urdf at
680fa925cd00db7e2628563d2edc5540963045de (piper_x_description.urdf and
piper_x_with_gripper_description.xacro). Finger moments cancel by symmetry.
Provides fast scalar forward kinematics, static gravity torques, pose-dependent
joint inertias, critical damping bounds, geometric Jacobians, and damped wrench solvers.

Model parameter license:
MIT License
Copyright (c) 2026 aalicecc

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

from dataclasses import dataclass, field, replace
from functools import lru_cache
import math
from pathlib import Path

import numpy as np
import yaml

from .terminal import RigidBody, terminal_body, terminal_for_model, terminal_profiles


def _euler_xyz_matrix(rpy: tuple[float, float, float]) -> np.ndarray:
    rx, ry, rz = rpy
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return np.array([
        [cy * cz, cz * sx * sy - cx * sz, cx * cz * sy + sx * sz],
        [cy * sz, cx * cz + sx * sy * sz, cx * sy * sz - cz * sx],
        [-sy, cy * sx, cx * cy],
    ])


_XYZ = np.array([
    (0, 0, 0.123), (0, 0, 0), (0.28503, 0, 0),
    (0.27364, 0, 0), (0.07466, 0, 0), (0, -0.035, 0),
])
_RPY = [
    (0, 0, 3.1415926), (-1.5707963, -3.005806, -3.1415926),
    (0, 0, 2.8380798), (0, 0, 0.0806342),
    (-1.5707963, 1.5707963, 0), (1.5707963, 0, 0),
]
_ROTATIONS = np.array([_euler_xyz_matrix(r) for r in _RPY])
_DEFAULT_TERMINAL = terminal_body('gripper')
_MASS = np.array([0.71, 1.16, 0.67, 0.39, 0.35, 0.01655408133, _DEFAULT_TERMINAL.mass_kg])
_COM = np.array([
    (0.00053, -0.00039, -0.00315), (0.21856, -0.00859, 0.00227),
    (0.2014, -0.00418, -0.00141), (0.059, -0.05831, 0.002),
    (0.00005, -0.01282, -0.00429), (0.00134940815, 0, -0.00344975303),
    _DEFAULT_TERMINAL.com_m,
])
_DESCENDANT = np.arange(7)[:, None] >= np.arange(6)[None, :]
_GRAVITY = np.array([0.0, 0.0, -9.81])
_LINK_INERTIAS = np.array([
    [[0.00050027, 7.8e-07, -6.26e-06], [7.8e-07, 0.00040879, -4.62e-06], [-6.26e-06, -4.62e-06, 0.00045802]],
    [[0.00111873, 0.00182162, 0.00019185], [0.00182162, 0.06763292, 7.71e-06], [0.00019185, 7.71e-06, 0.067559]],
    [[0.00080618, -0.00263276, -1.58e-06], [-0.00263276, 0.01313367, 5.5e-07], [-1.58e-06, 5.5e-07, 0.01363321]],
    [[0.00014613, 6.7e-07, 4.08e-06], [6.7e-07, 0.00017844, 6.2e-07], [4.08e-06, 6.2e-07, 0.00018914]],
    [[0.00171172, -9.89e-06, -1e-07], [-9.89e-06, 0.00017936, 2.42e-06], [-1e-07, 2.42e-06, 0.00172318]],
    [[0.00138626, -2.02e-06, -3e-08], [-2.02e-06, 0.00152112, 2.75e-06], [-3e-08, 2.75e-06, 0.00031152]],
])
_EFFECTOR_INERTIAS = {
    alias: terminal_body(key).inertia
    for key, profile in terminal_profiles().items()
    for alias in (key, profile['model'])
}
_MAX_INERTIA = {
    alias: terminal_for_model(key).max_joint_inertia_kg_m2
    for key, profile in terminal_profiles().items()
    for alias in (key, profile['model'])
}
_XYZ_T = tuple(tuple(map(float, row)) for row in _XYZ)
_ROT_T = tuple(tuple(tuple(map(float, row)) for row in mat) for mat in _ROTATIONS)
_COM_T = tuple(tuple(map(float, row)) for row in _COM)
_MASS_T = tuple(map(float, _MASS))
_LINK_I_T = tuple(tuple(tuple(map(float, row)) for row in mat) for mat in _LINK_INERTIAS)
_EFF_I_T = {
    k: tuple(tuple(map(float, row)) for row in mat)
    for k, mat in _EFFECTOR_INERTIAS.items()
}


def max_critical_kd(
    kp: tuple[float, ...] | list[float],
    end_effector: str = 'teaching_handle',
    gravity_model: 'ArmGravityModel | None' = None,
) -> tuple[float, ...]:
    """Return maximum critical damping kd (Nm/(rad/s)) across all URDF poses for kp."""
    if gravity_model is not None and gravity_model.effector_model:
        end_effector = gravity_model.effector_model
    inertias = _MAX_INERTIA.get(end_effector, _MAX_INERTIA['teaching_handle'])
    scales = (
        (1.0,) * 6
        if gravity_model is None
        else tuple(gravity_model.joint(i).scale or 1.0 for i in range(1, 7))
    )
    terminal = terminal_for_model(end_effector)
    if (terminal is not None and terminal.payload_link is not None
            and gravity_model is not None and gravity_model.added_mass_kg):
        shape = terminal.components[terminal.payload_link]
        unit = shape.inertia / shape.mass_kg
        radius = float(np.linalg.norm(shape.com_m))
        unit_max = float(np.linalg.eigvalsh(unit).max())
        increment = [
            (sum(float(np.linalg.norm(v)) for v in _XYZ[j + 1:]) + radius) ** 2 + unit_max
            for j in range(6)
        ]
        increment[5] = unit[2, 2] + shape.com_m[0] ** 2 + shape.com_m[1] ** 2
        inertias = tuple(v + gravity_model.added_mass_kg * d for v, d in zip(inertias, increment))
    return tuple(
        round(math.sqrt(2.0 * (scale * inertia) * max(0.0, float(k))), 2)
        for scale, inertia, k in zip(scales, inertias, kp)
    )


def link6_rotation_in_base(position: tuple[float, ...] | list[float]) -> np.ndarray:
    """Return the SDK-compatible link6 rotation for measured joint angles (rad)."""
    if len(position) < 6:
        raise ValueError('Six joint positions are required')
    return np.asarray(_fast_fk(position[:6], (0.0, 0.0, 0.0))[3][5], dtype=float)


def _kinematics(position: tuple[float, ...], effector_com=None):
    origins, axes, centers, _ = _fast_fk(position, effector_com)
    return np.asarray(origins, dtype=float), np.asarray(axes, dtype=float), np.asarray(centers, dtype=float)


def _fast_fk(position, effector_com=None):
    r00, r01, r02 = 1.0, 0.0, 0.0
    r10, r11, r12 = 0.0, 1.0, 0.0
    r20, r21, r22 = 0.0, 0.0, 1.0
    tx, ty, tz = 0.0, 0.0, 0.0
    origins = []
    axes = []
    centers = []
    rots = []
    for i in range(6):
        q = float(position[i])
        ox, oy, oz = _XYZ_T[i]
        tx += r00 * ox + r01 * oy + r02 * oz
        ty += r10 * ox + r11 * oy + r12 * oz
        tz += r20 * ox + r21 * oy + r22 * oz
        m0, m1, m2 = _ROT_T[i]
        n00 = r00 * m0[0] + r01 * m1[0] + r02 * m2[0]
        n01 = r00 * m0[1] + r01 * m1[1] + r02 * m2[1]
        n02 = r00 * m0[2] + r01 * m1[2] + r02 * m2[2]
        n10 = r10 * m0[0] + r11 * m1[0] + r12 * m2[0]
        n11 = r10 * m0[1] + r11 * m1[1] + r12 * m2[1]
        n12 = r10 * m0[2] + r11 * m1[2] + r12 * m2[2]
        n20 = r20 * m0[0] + r21 * m1[0] + r22 * m2[0]
        n21 = r20 * m0[1] + r21 * m1[1] + r22 * m2[1]
        n22 = r20 * m0[2] + r21 * m1[2] + r22 * m2[2]
        origins.append((tx, ty, tz))
        axes.append((n02, n12, n22))
        c, s = math.cos(q), math.sin(q)
        r00, r01, r02 = n00 * c + n01 * s, n01 * c - n00 * s, n02
        r10, r11, r12 = n10 * c + n11 * s, n11 * c - n10 * s, n12
        r20, r21, r22 = n20 * c + n21 * s, n21 * c - n20 * s, n22
        cx, cy, cz = _COM_T[i]
        centers.append((
            tx + r00 * cx + r01 * cy + r02 * cz,
            ty + r10 * cx + r11 * cy + r12 * cz,
            tz + r20 * cx + r21 * cy + r22 * cz,
        ))
        rots.append(((r00, r01, r02), (r10, r11, r12), (r20, r21, r22)))
    ec = _COM_T[6] if effector_com is None else effector_com
    cx, cy, cz = float(ec[0]), float(ec[1]), float(ec[2])
    centers.append((
        tx + r00 * cx + r01 * cy + r02 * cz,
        ty + r10 * cx + r11 * cy + r12 * cz,
        tz + r20 * cx + r21 * cy + r22 * cz,
    ))
    rots.append(rots[5])
    return origins, axes, centers, rots


def joint_inertias(
    position: tuple[float, ...] | list[float],
    end_effector: str = 'teaching_handle',
    mass_kg: float | None = None,
    com_m: tuple[float, ...] | None = None,
    scales: tuple[float, ...] = (1.0,) * 6,
    inertia_tensor=None,
) -> tuple[float, ...]:
    """Return pose-dependent diagonal joint-space inertias J_j(q) (kg*m^2) from URDF 3x3 tensors.

    When `scales` is provided (e.g. from gravity calibration), each joint's effective inertia
    is scaled proportionally without altering the 3x3 inertia tensor's geometric shape.
    """
    terminal = terminal_for_model(end_effector)
    nominal = terminal_body(end_effector) if terminal is not None and terminal.payload_link else None
    if mass_kg is None:
        mass_kg = float(_MASS[6]) if nominal is None else nominal.mass_kg
    if com_m is None:
        com_m = tuple(_COM[6]) if nominal is None else nominal.com_m
    eff_t = (_EFF_I_T.get(end_effector, _EFF_I_T['teaching_handle'])
             if inertia_tensor is None else np.asarray(inertia_tensor, dtype=float).tolist())
    return _joint_inertias_cached(
        tuple(float(v) for v in position[:6]), float(mass_kg), tuple(float(v) for v in com_m),
        tuple(float(v) for v in scales), tuple(tuple(float(v) for v in row) for row in eff_t),
    )


@lru_cache(maxsize=256)
def _joint_inertias_cached(position, mass_kg, com_m, scales, eff_t):
    origins, axes, centers, rots = _fast_fk(position, com_m)
    masses = (*_MASS_T[:6], mass_kg)
    tensors = (*_LINK_I_T, eff_t)
    out: list[float] = []
    for j in range(6):
        zx, zy, zz = axes[j]
        ox, oy, oz = origins[j]
        total_j = 0.0
        for k in range(j, 7):
            ckx, cky, ckz = centers[k]
            rx, ry, rz = ckx - ox, cky - oy, ckz - oz
            vx = zy * rz - zz * ry
            vy = zz * rx - zx * rz
            vz = zx * ry - zy * rx
            rk0, rk1, rk2 = rots[k]
            lx = rk0[0] * zx + rk1[0] * zy + rk2[0] * zz
            ly = rk0[1] * zx + rk1[1] * zy + rk2[1] * zz
            lz = rk0[2] * zx + rk1[2] * zy + rk2[2] * zz
            t0, t1, t2 = tensors[k]
            rot_part = (
                lx * (t0[0] * lx + t0[1] * ly + t0[2] * lz)
                + ly * (t1[0] * lx + t1[1] * ly + t1[2] * lz)
                + lz * (t2[0] * lx + t2[1] * ly + t2[2] * lz)
            )
            total_j += masses[k] * (vx * vx + vy * vy + vz * vz) + rot_part
        out.append(float(scales[j] * total_j))
    return tuple(out)


def gravity_torques(
    position: tuple[float, ...], mass_kg: float = float(_MASS[6]),
    com_m: tuple[float, ...] = tuple(_COM[6]),
) -> tuple[float, ...]:
    """Return joint-side Nm opposing gravity, not motor-side current/torque."""
    return _gravity_torques_cached(
        tuple(float(v) for v in position[:6]), float(mass_kg), tuple(float(v) for v in com_m),
    )


@lru_cache(maxsize=256)
def _gravity_torques_cached(position, mass_kg, com_m):
    origins, axes, centers, _ = _fast_fk(position, com_m)
    masses = (*_MASS_T[:6], float(mass_kg))
    out = [0.0] * 6
    acc_m = masses[6]
    acc_mx = acc_m * centers[6][0]
    acc_my = acc_m * centers[6][1]
    for j in range(5, -1, -1):
        mj = masses[j]
        acc_m += mj
        acc_mx += mj * centers[j][0]
        acc_my += mj * centers[j][1]
        arm_x = acc_mx - acc_m * origins[j][0]
        arm_y = acc_my - acc_m * origins[j][1]
        ax0, ax1, _ = axes[j]
        out[j] = float(9.81 * (arm_y * ax0 - arm_x * ax1))
    return tuple(out)


def geometric_jacobian(
    position: tuple[float, ...] | list[float],
    tool_offset_m: tuple[float, ...] | list[float] = (0.0, 0.0, 0.0),
    *,
    frame: str = 'gripper',
) -> np.ndarray:
    """Return the 6x6 geometric Jacobian J(q) in 'gripper' local frame (+x left, +y up, +z forward) or 'base_link'."""
    if len(position) < 6:
        raise ValueError('Six joint positions are required')
    if frame not in ('gripper', 'base_link'):
        raise ValueError("frame must be 'gripper' or 'base_link'")
    origins, axes, centers, rots = _fast_fk(position[:6], tool_offset_m)
    ex, ey, ez = centers[6]
    jacobian = np.empty((6, 6), dtype=float)
    r0, r1, r2 = rots[5]
    for i in range(6):
        zx, zy, zz = axes[i]
        ox, oy, oz = origins[i]
        rx, ry, rz = ex - ox, ey - oy, ez - oz
        vx = zy * rz - zz * ry
        vy = zz * rx - zx * rz
        vz = zx * ry - zy * rx
        if frame == 'gripper':
            # Gripper local frame: +x = -col0 (left), +y = -col1 (up), +z = +col2 (forward)
            jacobian[0, i] = -(r0[0] * vx + r1[0] * vy + r2[0] * vz)
            jacobian[1, i] = -(r0[1] * vx + r1[1] * vy + r2[1] * vz)
            jacobian[2, i] = r0[2] * vx + r1[2] * vy + r2[2] * vz
            jacobian[3, i] = -(r0[0] * zx + r1[0] * zy + r2[0] * zz)
            jacobian[4, i] = -(r0[1] * zx + r1[1] * zy + r2[1] * zz)
            jacobian[5, i] = r0[2] * zx + r1[2] * zy + r2[2] * zz
        else:
            jacobian[0, i] = vx
            jacobian[1, i] = vy
            jacobian[2, i] = vz
            jacobian[3, i] = zx
            jacobian[4, i] = zy
            jacobian[5, i] = zz
    return jacobian


def estimate_joint_torques(
    position: tuple[float, ...] | list[float],
    velocity: tuple[float, ...] | list[float],
    target_position: tuple[float, ...] | list[float],
    target_velocity: tuple[float, ...] | list[float],
    kp: tuple[float, ...] | list[float],
    kd: tuple[float, ...] | list[float],
    torque_cmd: tuple[float, ...] | list[float] = (0.0,) * 6,
    acceleration: tuple[float, ...] | list[float] = (0.0,) * 6,
    *,
    end_effector: str = 'teaching_handle',
    mass_kg: float | None = None,
    com_m: tuple[float, ...] | None = None,
    gravity_model: 'ArmGravityModel | None' = None,
) -> tuple[float, ...]:
    """Estimate 6-joint load/external torques (Nm) from MIT impedance error minus inertial torque."""
    for name, values in (
        ('position', position),
        ('velocity', velocity),
        ('target_position', target_position),
        ('target_velocity', target_velocity),
        ('kp', kp),
        ('kd', kd),
        ('torque_cmd', torque_cmd),
        ('acceleration', acceleration),
    ):
        if len(values) < 6:
            raise ValueError(f'{name} requires six values')
    inertias = (
        joint_inertias(position[:6], end_effector=end_effector, mass_kg=mass_kg, com_m=com_m)
        if gravity_model is None
        else gravity_model.inertias(
            position[:6], end_effector=end_effector, mass_kg=mass_kg, com_m=com_m,
        )
    )
    return tuple(
        float(
            float(kp[i]) * (float(position[i]) - float(target_position[i]))
            + float(kd[i]) * (float(velocity[i]) - float(target_velocity[i]))
            - float(torque_cmd[i])
            + inertias[i] * float(acceleration[i])
        )
        for i in range(6)
    )


def end_effector_wrench(
    joint_torques: tuple[float, ...] | list[float],
    position: tuple[float, ...] | list[float],
    *,
    gravity_model: 'ArmGravityModel | None' = None,
    tool_offset_m: tuple[float, ...] | list[float] = (0.0, 0.0, 0.0),
    damping: float = 1e-2,
    frame: str = 'gripper',
) -> tuple[float, ...]:
    """Map 6-joint torques (Nm) to 6-axis end-effector wrench (fx, fy, fz, tx, ty, tz) in gripper or base_link frame.

    Returns (fx, fy, fz, tx, ty, tz) where (fx, fy, fz) are forces in N and
    (tx, ty, tz) are torques in Nm, solved via damped transpose pseudo-inverse:
        F = (J * J^T + damping^2 * I)^(-1) * J * tau
    """
    if len(joint_torques) < 6 or len(position) < 6:
        raise ValueError('Six joint torques and six joint positions are required')
    if (
        isinstance(damping, bool)
        or not isinstance(damping, (int, float))
        or not math.isfinite(damping)
        or damping < 0.0
    ):
        raise ValueError('damping must be finite and non-negative')
    if frame not in ('gripper', 'base_link'):
        raise ValueError("frame must be 'gripper' or 'base_link'")
    t0, t1, t2, t3, t4, t5 = (
        float(joint_torques[0]), float(joint_torques[1]), float(joint_torques[2]),
        float(joint_torques[3]), float(joint_torques[4]), float(joint_torques[5]),
    )
    if not (t0 or t1 or t2 or t3 or t4 or t5):
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    if damping > 0.0:
        q_eff = (
            position[:6]
            if gravity_model is None
            else tuple(float(position[i]) + gravity_model.joint(i + 1).angle_offset for i in range(6))
        )
        origins, axes, centers, rots = _fast_fk(q_eff, tool_offset_m)
        return _solve_damped_wrench(
            (t0, t1, t2, t3, t4, t5), origins, axes, centers[6], rots[5], float(damping), frame,
        )
    tau = np.asarray(joint_torques[:6], dtype=float)
    j = (
        geometric_jacobian(position[:6], tool_offset_m=tool_offset_m, frame=frame)
        if gravity_model is None
        else gravity_model.jacobian(position[:6], tool_offset_m=tool_offset_m, frame=frame)
    )
    wrench, *_ = np.linalg.lstsq(j.T, tau, rcond=None)
    return tuple(float(v) for v in wrench)


def _solve_damped_wrench(
    tau: tuple[float, float, float, float, float, float],
    origins: list[tuple[float, float, float]],
    axes: list[tuple[float, float, float]],
    tip: tuple[float, float, float],
    rot5: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    damping: float,
    frame: str,
) -> tuple[float, ...]:
    t0, t1, t2, t3, t4, t5 = tau
    ex, ey, ez = tip
    r0, r1, r2 = rot5
    r00, r01, r02 = r0
    r10, r11, r12 = r1
    r20, r21, r22 = r2
    j0 = [0.0] * 6
    j1 = [0.0] * 6
    j2 = [0.0] * 6
    j3 = [0.0] * 6
    j4 = [0.0] * 6
    j5 = [0.0] * 6
    use_gripper = frame == 'gripper'
    for i in range(6):
        zx, zy, zz = axes[i]
        ox, oy, oz = origins[i]
        rx, ry, rz = ex - ox, ey - oy, ez - oz
        vx = zy * rz - zz * ry
        vy = zz * rx - zx * rz
        vz = zx * ry - zy * rx
        if use_gripper:
            j0[i] = -(r00 * vx + r10 * vy + r20 * vz)
            j1[i] = -(r01 * vx + r11 * vy + r21 * vz)
            j2[i] = r02 * vx + r12 * vy + r22 * vz
            j3[i] = -(r00 * zx + r10 * zy + r20 * zz)
            j4[i] = -(r01 * zx + r11 * zy + r21 * zz)
            j5[i] = r02 * zx + r12 * zy + r22 * zz
        else:
            j0[i] = vx
            j1[i] = vy
            j2[i] = vz
            j3[i] = zx
            j4[i] = zy
            j5[i] = zz
    lam2 = damping * damping
    fx, fy, fz = _solve_3x6_block((j0, j1, j2), tau, lam2)
    tau_rem = (
        t0 - (j0[0] * fx + j1[0] * fy + j2[0] * fz),
        t1 - (j0[1] * fx + j1[1] * fy + j2[1] * fz),
        t2 - (j0[2] * fx + j1[2] * fy + j2[2] * fz),
        t3 - (j0[3] * fx + j1[3] * fy + j2[3] * fz),
        t4 - (j0[4] * fx + j1[4] * fy + j2[4] * fz),
        t5 - (j0[5] * fx + j1[5] * fy + j2[5] * fz),
    )
    mx, my, mz = _solve_3x6_block((j3, j4, j5), tau_rem, max(lam2, 1e-2))
    return (fx, fy, fz, mx, my, mz)


def _solve_3x6_block(
    rows3: tuple[list[float], list[float], list[float]],
    tau: tuple[float, float, float, float, float, float],
    lam2: float,
) -> tuple[float, float, float]:
    r0, r1, r2 = rows3
    t0, t1, t2, t3, t4, t5 = tau
    a00 = r0[0]*r0[0] + r0[1]*r0[1] + r0[2]*r0[2] + r0[3]*r0[3] + r0[4]*r0[4] + r0[5]*r0[5] + lam2
    a01 = r0[0]*r1[0] + r0[1]*r1[1] + r0[2]*r1[2] + r0[3]*r1[3] + r0[4]*r1[4] + r0[5]*r1[5]
    a02 = r0[0]*r2[0] + r0[1]*r2[1] + r0[2]*r2[2] + r0[3]*r2[3] + r0[4]*r2[4] + r0[5]*r2[5]
    a11 = r1[0]*r1[0] + r1[1]*r1[1] + r1[2]*r1[2] + r1[3]*r1[3] + r1[4]*r1[4] + r1[5]*r1[5] + lam2
    a12 = r1[0]*r2[0] + r1[1]*r2[1] + r1[2]*r2[2] + r1[3]*r2[3] + r1[4]*r2[4] + r1[5]*r2[5]
    a22 = r2[0]*r2[0] + r2[1]*r2[1] + r2[2]*r2[2] + r2[3]*r2[3] + r2[4]*r2[4] + r2[5]*r2[5] + lam2
    b0 = r0[0]*t0 + r0[1]*t1 + r0[2]*t2 + r0[3]*t3 + r0[4]*t4 + r0[5]*t5
    b1 = r1[0]*t0 + r1[1]*t1 + r1[2]*t2 + r1[3]*t3 + r1[4]*t4 + r1[5]*t5
    b2 = r2[0]*t0 + r2[1]*t1 + r2[2]*t2 + r2[3]*t3 + r2[4]*t4 + r2[5]*t5
    # 3x3 symmetric positive-definite elimination
    inv0 = 1.0 / a00
    f10 = a01 * inv0
    f20 = a02 * inv0
    m11 = a11 - f10 * a01
    m12 = a12 - f10 * a02
    rhs1 = b1 - f10 * b0
    m22 = a22 - f20 * a02
    rhs2 = b2 - f20 * b0
    inv1 = 1.0 / m11
    f21 = m12 * inv1
    m22 -= f21 * m12
    rhs2 -= f21 * rhs1
    x2 = rhs2 / m22
    x1 = (rhs1 - m12 * x2) * inv1
    x0 = (b0 - a01 * x1 - a02 * x2) * inv0
    return (float(x0), float(x1), float(x2))


def estimate_end_wrench(
    position: tuple[float, ...] | list[float],
    velocity: tuple[float, ...] | list[float],
    target_position: tuple[float, ...] | list[float],
    target_velocity: tuple[float, ...] | list[float],
    kp: tuple[float, ...] | list[float],
    kd: tuple[float, ...] | list[float],
    torque_cmd: tuple[float, ...] | list[float] = (0.0,) * 6,
    acceleration: tuple[float, ...] | list[float] = (0.0,) * 6,
    *,
    end_effector: str = 'teaching_handle',
    mass_kg: float | None = None,
    com_m: tuple[float, ...] | None = None,
    gravity_model: 'ArmGravityModel | None' = None,
    tool_offset_m: tuple[float, ...] | list[float] = (0.0, 0.0, 0.0),
    damping: float = 1e-2,
    frame: str = 'gripper',
) -> tuple[float, ...]:
    """Estimate 6-axis end-effector wrench (fx, fy, fz, tx, ty, tz) from MIT impedance state."""
    tau_ext = estimate_joint_torques(
        position, velocity, target_position, target_velocity,
        kp, kd, torque_cmd, acceleration,
        end_effector=end_effector, mass_kg=mass_kg, com_m=com_m, gravity_model=gravity_model,
    )
    return end_effector_wrench(
        tau_ext, position,
        gravity_model=gravity_model, tool_offset_m=tool_offset_m, damping=damping, frame=frame,
    )


@dataclass(frozen=True)
class JointGravityModel:
    """Single-joint calibrated gravity correction and quasi-static arrest friction."""

    scale: float = 1.0
    angle_offset: float = 0.0
    bias: float = 0.0
    static_friction: float = 0.0

    def __post_init__(self) -> None:
        for name in ('scale', 'angle_offset', 'bias', 'static_friction'):
            val = getattr(self, name)
            if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(val):
                raise ValueError(f'{name} must be finite')
        if self.scale < 0.0:
            raise ValueError('scale must be non-negative')
        if self.static_friction < 0.0:
            raise ValueError('static_friction must be non-negative')

    def to_dict(self) -> dict[str, float]:
        return {
            'scale': float(self.scale),
            'angle_offset': float(self.angle_offset),
            'bias': float(self.bias),
            'static_friction': float(self.static_friction),
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'JointGravityModel':
        if not isinstance(data, dict):
            raise ValueError('Joint gravity model must be a mapping')
        return cls(
            scale=float(data.get('scale', 1.0)),
            angle_offset=float(data.get('angle_offset', 0.0)),
            bias=float(data.get('bias', 0.0)),
            static_friction=float(data.get('static_friction', 0.0)),
        )


_DEFAULT_JOINT_GRAVITY = JointGravityModel()


@dataclass(frozen=True)
class ArmGravityModel:
    """Six-joint calibrated gravity model container for one PiPER-X arm."""

    arm_id: str = 'arm_1'
    joints: dict[int, JointGravityModel] = field(default_factory=dict)
    effector_model: str = ''
    added_mass_kg: float = 0.0

    def __post_init__(self):
        if not math.isfinite(self.added_mass_kg) or self.added_mass_kg < 0:
            raise ValueError('Added gripper mass must be finite and non-negative')

    def for_effector(self, model: str) -> 'ArmGravityModel':
        terminal = terminal_for_model(model)
        if terminal is None or terminal.payload_link is None:
            return replace(self, effector_model='', added_mass_kg=0.0)
        mass = self.added_mass_kg if self.effector_model in ('', terminal.model) else 0.0
        return replace(self, effector_model=terminal.model, added_mass_kg=mass)

    def with_added_mass(self, mass_kg: float) -> 'ArmGravityModel':
        return replace(self, added_mass_kg=float(mass_kg))

    def effector_body(self, mass_kg=None, com_m=None, end_effector='teaching_handle') -> RigidBody:
        name = self.effector_model or end_effector
        if not self.effector_model:
            mass_kg = float(_MASS[6]) if mass_kg is None else mass_kg
            com_m = tuple(_COM[6]) if com_m is None else com_m
        return terminal_body(
            name if terminal_for_model(name) is not None else 'teaching_handle',
            self.added_mass_kg, mass_kg, None if com_m is None else tuple(com_m))

    def joint(self, index: int) -> JointGravityModel:
        if not 1 <= index <= 6:
            raise ValueError(f'Joint index must be in 1..6, got {index}')
        return self.joints.get(index, _DEFAULT_JOINT_GRAVITY)

    def with_joint(self, index: int, model: JointGravityModel) -> 'ArmGravityModel':
        if not 1 <= index <= 6:
            raise ValueError(f'Joint index must be in 1..6, got {index}')
        updated = dict(self.joints)
        updated[index] = model
        return replace(self, joints=updated)

    def torques(
        self,
        position: tuple[float, ...] | list[float],
        mass_kg: float | None = None,
        com_m: tuple[float, ...] | None = None,
    ) -> tuple[float, ...]:
        if len(position) < 6:
            raise ValueError('Six joint positions are required')
        q_eff = tuple(
            float(position[i]) + self.joint(i + 1).angle_offset
            for i in range(6)
        )
        body = self.effector_body(mass_kg, com_m)
        cad = gravity_torques(q_eff, mass_kg=body.mass_kg, com_m=body.com_m)
        return tuple(
            float(self.joint(i + 1).scale * cad[i] + self.joint(i + 1).bias)
            for i in range(6)
        )

    def inertias(
        self,
        position: tuple[float, ...] | list[float],
        end_effector: str = 'teaching_handle',
        mass_kg: float | None = None,
        com_m: tuple[float, ...] | None = None,
    ) -> tuple[float, ...]:
        """Return pose-dependent joint inertias J_j(q) scaled proportionally by calibrated gravity scale."""
        if len(position) < 6:
            raise ValueError('Six joint positions are required')
        q_eff = tuple(
            float(position[i]) + self.joint(i + 1).angle_offset
            for i in range(6)
        )
        scales = tuple(self.joint(i + 1).scale or 1.0 for i in range(6))
        body = self.effector_body(mass_kg, com_m, end_effector)
        return joint_inertias(
            q_eff, end_effector=end_effector, mass_kg=body.mass_kg, com_m=body.com_m,
            scales=scales, inertia_tensor=body.inertia,
        )

    def jacobian(
        self,
        position: tuple[float, ...] | list[float],
        tool_offset_m: tuple[float, ...] | list[float] = (0.0, 0.0, 0.0),
        *,
        frame: str = 'gripper',
    ) -> np.ndarray:
        """Return 6x6 geometric Jacobian J(q) with calibrated joint angle offsets."""
        if len(position) < 6:
            raise ValueError('Six joint positions are required')
        q_eff = tuple(
            float(position[i]) + self.joint(i + 1).angle_offset
            for i in range(6)
        )
        return geometric_jacobian(q_eff, tool_offset_m=tool_offset_m, frame=frame)

    def scaled_inertia_tensors(
        self,
        end_effector: str = 'teaching_handle',
    ) -> tuple[np.ndarray, ...]:
        """Return the 6 link 3x3 inertia tensors (plus effector) scaled proportionally without changing shape."""
        eff_inertia = self.effector_body(end_effector=end_effector).inertia
        tensors = [
            _LINK_INERTIAS[i] * (self.joint(i + 1).scale or 1.0)
            for i in range(6)
        ]
        tensors.append(eff_inertia * (self.joint(6).scale or 1.0))
        return tuple(tensors)

    def to_dict(self) -> dict:
        data = {
            'version': 1,
            'arm_id': self.arm_id,
            'joints': {
                f'joint{index}': self.joint(index).to_dict()
                for index in sorted(self.joints)
            },
        }
        if self.effector_model:
            data.update(effector_model=self.effector_model, added_mass_kg=float(self.added_mass_kg))
        return data

    @classmethod
    def from_dict(cls, data: dict) -> 'ArmGravityModel':
        if not isinstance(data, dict) or data.get('version', 1) != 1:
            raise ValueError('Unsupported gravity model format or version')
        raw_joints = data.get('joints', {})
        if not isinstance(raw_joints, dict):
            raise ValueError('joints must be a mapping')
        joints: dict[int, JointGravityModel] = {}
        for key, value in raw_joints.items():
            if isinstance(key, int):
                idx = key
            elif isinstance(key, str) and key.startswith('joint'):
                idx = int(key[5:])
            else:
                idx = int(key)
            if not 1 <= idx <= 6:
                raise ValueError(f'Joint index out of range: {key}')
            joints[idx] = JointGravityModel.from_dict(value)
        return cls(
            arm_id=str(data.get('arm_id', 'arm_1')), joints=joints,
            effector_model=str(data.get('effector_model', '')),
            added_mass_kg=float(data.get('added_mass_kg', 0.0)),
        )


def save_gravity_model(model: ArmGravityModel, path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(model.to_dict(), stream, allow_unicode=True, sort_keys=False)
    return target


def load_gravity_model(path: str | Path) -> ArmGravityModel:
    source = Path(path)
    with source.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    return ArmGravityModel.from_dict(data)



def damping_main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description='Compute six-joint critical damping kd from kp.')
    parser.add_argument('--kp', '-k', type=float, nargs=6, required=True)
    parser.add_argument(
        '--effector', '-e', default='teaching_handle',
        choices=('teaching_handle', 'gripper', 'gripper_with_wrench'),
    )
    parser.add_argument('--gravity-config', '-g', type=Path, default=None)
    args = parser.parse_args(argv)
    g_model = load_gravity_model(args.gravity_config) if args.gravity_config else None
    kd = max_critical_kd(args.kp, args.effector, g_model)
    print('kd: [' + ', '.join(f'{v:g}' for v in kd) + ']')
    return 0
