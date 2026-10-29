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

import math

import numpy as np
from scipy.spatial.transform import Rotation


_XYZ = np.array([
    (0, 0, 0.123), (0, 0, 0), (0.28503, 0, 0),
    (0.27364, 0, 0), (0.07466, 0, 0), (0, -0.035, 0),
])
_RPY = [
    (0, 0, 3.1415926), (-1.5707963, -3.005806, -3.1415926),
    (0, 0, 2.8380798), (0, 0, 0.0806342),
    (-1.5707963, 1.5707963, 0), (1.5707963, 0, 0),
]
_ROTATIONS = Rotation.from_euler('xyz', _RPY).as_matrix()
_MASS = np.array([0.71, 1.16, 0.67, 0.39, 0.35, 0.01655408133, 0.54])
_COM = np.array([
    (0.00053, -0.00039, -0.00315), (0.21856, -0.00859, 0.00227),
    (0.2014, -0.00418, -0.00141), (0.059, -0.05831, 0.002),
    (0.00005, -0.01282, -0.00429), (0.00134940815, 0, -0.00344975303),
    (0.00001822434565775221, -0.00009521502789096145, 0.03953134028382456),
])
_DESCENDANT = np.arange(7)[:, None] >= np.arange(6)[None, :]
_GRAVITY = np.array([0.0, 0.0, -9.81])


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
