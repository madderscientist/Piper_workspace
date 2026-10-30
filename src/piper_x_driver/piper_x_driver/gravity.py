"""Static compensation for an upright PiPER-X with a configured end effector.

Inertial parameters from agilexrobotics/agx_arm_urdf at
680fa925cd00db7e2628563d2edc5540963045de (piper_x_description.urdf and
piper_x_with_gripper_description.xacro). Finger moments cancel by symmetry.
No payload, friction, Coriolis or acceleration compensation is assumed.

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

from dataclasses import dataclass, field
import math
from pathlib import Path

import numpy as np
import yaml


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
_MASS = np.array([0.71, 1.16, 0.67, 0.39, 0.35, 0.01655408133, 0.54])
_COM = np.array([
    (0.00053, -0.00039, -0.00315), (0.21856, -0.00859, 0.00227),
    (0.2014, -0.00418, -0.00141), (0.059, -0.05831, 0.002),
    (0.00005, -0.01282, -0.00429), (0.00134940815, 0, -0.00344975303),
    (0.00001822434565775221, -0.00009521502789096145, 0.03953134028382456),
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
    'teaching_handle': np.array([
        [0.00126369, 1.041e-05, -8e-08],
        [1.041e-05, 0.00104075, 8.975e-05],
        [-8e-08, 8.975e-05, 0.00067711],
    ]),
    'gripper': np.array([
        [0.00099737, -2.4e-07, -2.5e-07],
        [-2.4e-07, 0.00121497, -7.74e-06],
        [-2.5e-07, -7.74e-06, 0.00041479],
    ]),
}
_MAX_INERTIA = {
    'teaching_handle': (0.712418, 0.722116, 0.169451, 0.015078, 0.014926, 0.001023),
    'piper_description_with_teach.urdf': (0.712418, 0.722116, 0.169451, 0.015078, 0.014926, 0.001023),
    'gripper': (0.694602, 0.704391, 0.163588, 0.014434, 0.014274, 0.000726),
    'piper_description.urdf': (0.694602, 0.704391, 0.163588, 0.014434, 0.014274, 0.000726),
}


def max_critical_kd(
    kp: tuple[float, ...] | list[float],
    end_effector: str = 'teaching_handle',
    gravity_model: 'ArmGravityModel | None' = None,
) -> tuple[float, ...]:
    """Return maximum critical damping kd (Nm/(rad/s)) across all URDF poses for kp."""
    inertias = _MAX_INERTIA.get(end_effector, _MAX_INERTIA['teaching_handle'])
    scales = (
        (1.0,) * 6
        if gravity_model is None
        else tuple(gravity_model.joint(i).scale or 1.0 for i in range(1, 7))
    )
    return tuple(
        round(math.sqrt(2.0 * (scale * inertia) * max(0.0, float(k))), 2)
        for scale, inertia, k in zip(scales, inertias, kp)
    )


def _kinematics(position: tuple[float, ...], effector_com=None):
    rotation = np.eye(3)
    translation = np.zeros(3)
    origins, axes, centers = np.empty((6, 3)), np.empty((6, 3)), np.empty((7, 3))
    for index, q in enumerate(position):
        translation = translation + rotation @ _XYZ[index]
        rotation = rotation @ _ROTATIONS[index]
        origins[index] = translation
        axes[index] = rotation[:, 2]
        c, s = math.cos(q), math.sin(q)
        r0 = rotation[:, 0].copy()
        rotation[:, 0] = r0 * c + rotation[:, 1] * s
        rotation[:, 1] = rotation[:, 1] * c - r0 * s
        centers[index] = translation + rotation @ _COM[index]
    centers[6] = translation + rotation @ (_COM[6] if effector_com is None else effector_com)
    return origins, axes, centers


def joint_inertias(
    position: tuple[float, ...] | list[float],
    end_effector: str = 'teaching_handle',
    mass_kg: float = float(_MASS[6]),
    com_m: tuple[float, ...] = tuple(_COM[6]),
    scales: tuple[float, ...] = (1.0,) * 6,
) -> tuple[float, ...]:
    """Return pose-dependent diagonal joint-space inertias J_j(q) (kg*m^2) from URDF 3x3 tensors.

    When `scales` is provided (e.g. from gravity calibration), each joint's effective inertia
    is scaled proportionally without altering the 3x3 inertia tensor's geometric shape.
    """
    rotation = np.eye(3)
    translation = np.zeros(3)
    origins = np.empty((6, 3))
    axes = np.empty((6, 3))
    centers = np.empty((7, 3))
    rotations = np.empty((7, 3, 3))
    for index, q in enumerate(position[:6]):
        translation = translation + rotation @ _XYZ[index]
        rotation = rotation @ _ROTATIONS[index]
        origins[index] = translation
        axes[index] = rotation[:, 2]
        c, s = math.cos(float(q)), math.sin(float(q))
        r0 = rotation[:, 0].copy()
        rotation[:, 0] = r0 * c + rotation[:, 1] * s
        rotation[:, 1] = rotation[:, 1] * c - r0 * s
        centers[index] = translation + rotation @ _COM[index]
        rotations[index] = rotation
    centers[6] = translation + rotation @ np.asarray(com_m, dtype=float)
    rotations[6] = rotation

    masses = np.append(_MASS[:6], mass_kg)
    eff_inertia = _EFFECTOR_INERTIAS.get(end_effector, _EFFECTOR_INERTIAS['teaching_handle'])
    local_tensors = [*_LINK_INERTIAS, eff_inertia]

    out: list[float] = []
    for j in range(6):
        z_j = axes[j]
        o_j = origins[j]
        total_j = 0.0
        for k in range(j, 7):
            r_jk = centers[k] - o_j
            v_lin = np.cross(z_j, r_jk)
            z_local = rotations[k].T @ z_j
            rot_part = float(z_local @ local_tensors[k] @ z_local)
            total_j += float(masses[k] * np.dot(v_lin, v_lin)) + rot_part
        out.append(float(scales[j] * total_j))
    return tuple(out)


def gravity_torques(
    position: tuple[float, ...], mass_kg: float = float(_MASS[6]),
    com_m: tuple[float, ...] = tuple(_COM[6]),
) -> tuple[float, ...]:
    """Return joint-side Nm opposing gravity, not motor-side current/torque."""
    origins, axes, centers = _kinematics(position, com_m)
    masses = np.append(_MASS[:6], mass_kg)
    arms = (
        _DESCENDANT.T @ (masses[:, None] * centers)
        - (_DESCENDANT.T @ masses)[:, None] * origins
    )
    return tuple(map(float, -_GRAVITY[2] * (arms[:, 1] * axes[:, 0] - arms[:, 0] * axes[:, 1])))


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


@dataclass(frozen=True)
class ArmGravityModel:
    """Six-joint calibrated gravity model container for one PiPER-X arm."""

    arm_id: str = 'arm_1'
    joints: dict[int, JointGravityModel] = field(default_factory=dict)

    def joint(self, index: int) -> JointGravityModel:
        if not 1 <= index <= 6:
            raise ValueError(f'Joint index must be in 1..6, got {index}')
        return self.joints.get(index, JointGravityModel())

    def with_joint(self, index: int, model: JointGravityModel) -> 'ArmGravityModel':
        if not 1 <= index <= 6:
            raise ValueError(f'Joint index must be in 1..6, got {index}')
        updated = dict(self.joints)
        updated[index] = model
        return ArmGravityModel(arm_id=self.arm_id, joints=updated)

    def torques(
        self,
        position: tuple[float, ...] | list[float],
        mass_kg: float = float(_MASS[6]),
        com_m: tuple[float, ...] = tuple(_COM[6]),
    ) -> tuple[float, ...]:
        if len(position) < 6:
            raise ValueError('Six joint positions are required')
        q_eff = tuple(
            float(position[i]) + self.joint(i + 1).angle_offset
            for i in range(6)
        )
        cad = gravity_torques(q_eff, mass_kg=mass_kg, com_m=com_m)
        return tuple(
            float(self.joint(i + 1).scale * cad[i] + self.joint(i + 1).bias)
            for i in range(6)
        )

    def inertias(
        self,
        position: tuple[float, ...] | list[float],
        end_effector: str = 'teaching_handle',
        mass_kg: float = float(_MASS[6]),
        com_m: tuple[float, ...] = tuple(_COM[6]),
    ) -> tuple[float, ...]:
        """Return pose-dependent joint inertias J_j(q) scaled proportionally by calibrated gravity scale."""
        if len(position) < 6:
            raise ValueError('Six joint positions are required')
        q_eff = tuple(
            float(position[i]) + self.joint(i + 1).angle_offset
            for i in range(6)
        )
        scales = tuple(self.joint(i + 1).scale or 1.0 for i in range(6))
        return joint_inertias(
            q_eff, end_effector=end_effector, mass_kg=mass_kg, com_m=com_m, scales=scales,
        )

    def scaled_inertia_tensors(
        self,
        end_effector: str = 'teaching_handle',
    ) -> tuple[np.ndarray, ...]:
        """Return the 6 link 3x3 inertia tensors (plus effector) scaled proportionally without changing shape."""
        eff_inertia = _EFFECTOR_INERTIAS.get(end_effector, _EFFECTOR_INERTIAS['teaching_handle'])
        tensors = [
            _LINK_INERTIAS[i] * (self.joint(i + 1).scale or 1.0)
            for i in range(6)
        ]
        tensors.append(eff_inertia * (self.joint(6).scale or 1.0))
        return tuple(tensors)

    def to_dict(self) -> dict:
        return {
            'version': 1,
            'arm_id': self.arm_id,
            'joints': {
                f'joint{index}': self.joint(index).to_dict()
                for index in sorted(self.joints)
            },
        }

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
        return cls(arm_id=str(data.get('arm_id', 'arm_1')), joints=joints)


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
        '--effector', '-e', default='teaching_handle', choices=('teaching_handle', 'gripper'),
    )
    parser.add_argument('--gravity-config', '-g', type=Path, default=None)
    args = parser.parse_args(argv)
    g_model = load_gravity_model(args.gravity_config) if args.gravity_config else None
    kd = max_critical_kd(args.kp, args.effector, g_model)
    print('kd: [' + ', '.join(f'{v:g}' for v in kd) + ']')
    return 0
