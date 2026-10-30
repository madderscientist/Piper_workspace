"""Mathematical model and YAML serialization for Stribeck + periodic joint friction."""

from dataclasses import dataclass, field
import math
from pathlib import Path

import yaml


@dataclass(frozen=True)
class SpatialHarmonic:
    """One spatial harmonic component at angular frequency omega (cycles per 2*pi rad).

    - (a, b): direction-independent cogging torque coefficients:
      a * cos(omega * theta) + b * sin(omega * theta)
    - (c, d): direction-dependent meshing friction modulation coefficients:
      c * cos(omega * theta) + d * sin(omega * theta)
    """

    omega: float
    a: float = 0.0
    b: float = 0.0
    c: float = 0.0
    d: float = 0.0

    def __post_init__(self) -> None:
        for name in ('omega', 'a', 'b', 'c', 'd'):
            val = getattr(self, name)
            if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(val):
                raise ValueError(f'Harmonic {name} must be a finite number')
        if self.omega <= 0:
            raise ValueError('Harmonic omega must be positive')

    def cogging(self, theta: float) -> float:
        phase = self.omega * theta
        return self.a * math.cos(phase) + self.b * math.sin(phase)

    def modulation(self, theta: float) -> float:
        phase = self.omega * theta
        return self.c * math.cos(phase) + self.d * math.sin(phase)

    @property
    def cogging_amplitude(self) -> float:
        return math.hypot(self.a, self.b)

    @property
    def modulation_amplitude(self) -> float:
        return math.hypot(self.c, self.d)

    def to_dict(self) -> dict[str, float]:
        return {
            'omega': float(self.omega),
            'a': float(self.a),
            'b': float(self.b),
            'c': float(self.c),
            'd': float(self.d),
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'SpatialHarmonic':
        if not isinstance(data, dict) or 'omega' not in data:
            raise ValueError('Invalid harmonic entry')
        return cls(
            omega=float(data['omega']),
            a=float(data.get('a', 0.0)),
            b=float(data.get('b', 0.0)),
            c=float(data.get('c', 0.0)),
            d=float(data.get('d', 0.0)),
        )


@dataclass(frozen=True)
class JointFrictionModel:
    """Single-joint friction model combining Stribeck curve and low-speed periodic ripple."""

    fc: float = 0.0
    fs: float = 0.0
    vs: float = 0.1
    delta: float = 2.0
    fv: float = 0.0
    v0: float = 0.01
    vr: float = 0.5
    harmonics: tuple[SpatialHarmonic, ...] = ()

    def __post_init__(self) -> None:
        for name in ('fc', 'fs', 'vs', 'delta', 'fv', 'v0', 'vr'):
            val = getattr(self, name)
            if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(val):
                raise ValueError(f'{name} must be finite')
        if self.vs <= 0:
            raise ValueError('vs must be finite and positive')
        if self.v0 <= 0:
            raise ValueError('v0 must be finite and positive')
        if self.vr <= 0:
            raise ValueError('vr must be finite and positive')
        if self.delta <= 0:
            raise ValueError('delta must be finite and positive')

    def cogging_torque(self, theta: float) -> float:
        """Direction-independent spatial cogging torque tau_cog(theta)."""
        return sum(h.cogging(theta) for h in self.harmonics)

    def modulation_torque(self, theta: float) -> float:
        """Direction-dependent spatial meshing modulation tau_mod(theta)."""
        return sum(h.modulation(theta) for h in self.harmonics)

    def envelope(self, velocity: float) -> float:
        """Speed decay envelope w(dq) = exp(-(dq / vr)^2), ~1 at low speed and ->0 at high speed."""
        ratio = velocity / self.vr
        return math.exp(-(ratio * ratio))

    def stribeck_magnitude(self, speed: float) -> float:
        """Steady-state rate-dependent friction magnitude F(|dq|) for speed >= 0."""
        s = abs(speed)
        stribeck_term = (self.fs - self.fc) * math.exp(-((s / self.vs) ** self.delta))
        return self.fc + stribeck_term + self.fv * s

    def stribeck_torque(self, velocity: float) -> float:
        """Smooth rate-dependent friction torque without spatial ripple."""
        s = abs(velocity)
        dry = self.fc + (self.fs - self.fc) * math.exp(-((s / self.vs) ** self.delta))
        return dry * math.tanh(velocity / self.v0) + self.fv * velocity

    def torque(
        self,
        theta: float,
        velocity: float,
        *,
        cogging_scale: float = 1.0,
        friction_scale: float = 1.0,
    ) -> float:
        """Total modeled joint friction torque tau_fric(theta, velocity) in Nm.

        Smoothly gated by tanh(|velocity| / v0) so torque is strictly 0 at rest (velocity == 0)
        while preserving direction-independent cogging sign and full amplitude once moving.
        """
        s = abs(velocity)
        if s == 0.0:
            return 0.0
        # 1. 高速衰减包络 w(dq) = exp(-(dq / vr)^2)：低速接近 1（完整补偿齿轮感），高速趋近 0（平滑退火）
        env = self.envelope(velocity)
        # 2. 零速平滑门控 tanh(|dq| / v0)：静止时为 0 防止齿槽补偿在零速自激，一旦运动迅速升至 1
        motion_gate = math.tanh(s / self.v0)
        # 3. 与运动方向相关的摩擦项：(Stribeck 干摩擦 + 周期啮合调制) * tanh(dq / v0) + 粘滞摩擦 fv * dq
        dry = self.fc + (self.fs - self.fc) * math.exp(-((s / self.vs) ** self.delta))
        mod = env * self.modulation_torque(theta)
        directional = (dry + mod) * math.tanh(velocity / self.v0) + self.fv * velocity
        # 4. 与运动方向无关的齿槽转矩项：w(dq) * tanh(|dq| / v0) * tau_cog(theta)
        cog = env * motion_gate * self.cogging_torque(theta)
        return friction_scale * directional + cogging_scale * cog

    def to_dict(self) -> dict:
        return {
            'fc': float(self.fc),
            'fs': float(self.fs),
            'vs': float(self.vs),
            'delta': float(self.delta),
            'fv': float(self.fv),
            'v0': float(self.v0),
            'vr': float(self.vr),
            'harmonics': [h.to_dict() for h in self.harmonics],
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'JointFrictionModel':
        if not isinstance(data, dict):
            raise ValueError('Joint friction model must be a mapping')
        raw_harmonics = data.get('harmonics', ())
        if not isinstance(raw_harmonics, (list, tuple)):
            raise ValueError('harmonics must be a list')
        harmonics = tuple(SpatialHarmonic.from_dict(item) for item in raw_harmonics)
        return cls(
            fc=float(data.get('fc', 0.0)),
            fs=float(data.get('fs', 0.0)),
            vs=float(data.get('vs', 0.1)),
            delta=float(data.get('delta', 2.0)),
            fv=float(data.get('fv', 0.0)),
            v0=float(data.get('v0', 0.01)),
            vr=float(data.get('vr', 0.5)),
            harmonics=harmonics,
        )


@dataclass(frozen=True)
class ArmFrictionModel:
    """Six-joint friction model container for one PiPER-X arm."""

    arm_id: str = 'arm_1'
    joints: dict[int, JointFrictionModel] = field(default_factory=dict)

    def joint(self, index: int) -> JointFrictionModel:
        if not 1 <= index <= 6:
            raise ValueError(f'Joint index must be in 1..6, got {index}')
        return self.joints.get(index, JointFrictionModel())

    def with_joint(self, index: int, model: JointFrictionModel) -> 'ArmFrictionModel':
        if not 1 <= index <= 6:
            raise ValueError(f'Joint index must be in 1..6, got {index}')
        updated = dict(self.joints)
        updated[index] = model
        return ArmFrictionModel(arm_id=self.arm_id, joints=updated)

    def torques(
        self,
        positions: tuple[float, ...] | list[float],
        velocities: tuple[float, ...] | list[float],
        *,
        cogging_scale: float = 1.0,
        friction_scale: float = 1.0,
    ) -> tuple[float, ...]:
        if len(positions) < 6 or len(velocities) < 6:
            raise ValueError('Six joint positions and velocities are required')
        return tuple(
            self.joint(index).torque(
                float(positions[index - 1]),
                float(velocities[index - 1]),
                cogging_scale=cogging_scale,
                friction_scale=friction_scale,
            )
            for index in range(1, 7)
        )

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
    def from_dict(cls, data: dict) -> 'ArmFrictionModel':
        if not isinstance(data, dict) or data.get('version', 1) != 1:
            raise ValueError('Unsupported friction model format or version')
        raw_joints = data.get('joints', {})
        if not isinstance(raw_joints, dict):
            raise ValueError('joints must be a mapping')
        joints: dict[int, JointFrictionModel] = {}
        for key, value in raw_joints.items():
            if isinstance(key, int):
                idx = key
            elif isinstance(key, str) and key.startswith('joint'):
                idx = int(key[5:])
            else:
                idx = int(key)
            if not 1 <= idx <= 6:
                raise ValueError(f'Joint index out of range: {key}')
            joints[idx] = JointFrictionModel.from_dict(value)
        return cls(arm_id=str(data.get('arm_id', 'arm_1')), joints=joints)


def save_friction_model(model: ArmFrictionModel, path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(model.to_dict(), stream, allow_unicode=True, sort_keys=False)
    return target


def load_friction_model(path: str | Path) -> ArmFrictionModel:
    source = Path(path)
    with source.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    return ArmFrictionModel.from_dict(data)
