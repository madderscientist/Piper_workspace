"""Mathematical model and YAML serialization for Stribeck + periodic joint and non-periodic gripper friction."""

from bisect import bisect_right
from dataclasses import dataclass, field
import math
from pathlib import Path

import yaml


def _interp_1d(x: float, xs: tuple[float, ...], ys: tuple[float, ...]) -> float:
    """Piecewise-linear 1D interpolation with endpoint clamping."""
    n = len(xs)
    if n == 0 or len(ys) != n:
        return 0.0
    if n == 1 or x <= xs[0]:
        return float(ys[0])
    if x >= xs[-1]:
        return float(ys[-1])
    idx = bisect_right(xs, x)
    x0, x1 = xs[idx - 1], xs[idx]
    y0, y1 = ys[idx - 1], ys[idx]
    span = x1 - x0
    if span <= 1e-12:
        return float(y0)
    alpha = (x - x0) / span
    return float(y0 + alpha * (y1 - y0))


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
    load_ratio: float = 0.0
    error_epsilon: float = 0.05
    velocity_epsilon: float = 0.02
    harmonics: tuple[SpatialHarmonic, ...] = ()

    def __post_init__(self) -> None:
        for name in (
            'fc', 'fs', 'vs', 'delta', 'fv', 'v0', 'vr',
            'load_ratio', 'error_epsilon', 'velocity_epsilon',
        ):
            val = getattr(self, name)
            if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(val):
                raise ValueError(f'{name} must be finite')
        for nonneg in ('load_ratio', 'error_epsilon', 'velocity_epsilon'):
            if getattr(self, nonneg) < 0.0:
                raise ValueError(f'{nonneg} must be finite and non-negative')
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

    def _mod_and_cogging(self, theta: float) -> tuple[float, float]:
        if not self.harmonics:
            return 0.0, 0.0
        mod = 0.0
        cog = 0.0
        for h in self.harmonics:
            phase = h.omega * theta
            c = math.cos(phase)
            s = math.sin(phase)
            cog += h.a * c + h.b * s
            mod += h.c * c + h.d * s
        return mod, cog

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

    def stiction_torque(
        self,
        theta: float,
        position_error: float = 0.0,
        target_velocity: float = 0.0,
        *,
        gravity_torque: float = 0.0,
        static_friction: float = 0.0,
        error_epsilon: float | None = None,
        velocity_epsilon: float | None = None,
    ) -> float:
        """Smooth breakaway stiction torque driven by position error and target velocity:

            tau_stiction = max(0, load_ratio * |G| + tau_0) * tanh(e / eps_e + dq_target / eps_v)
        """
        eps_e = self.error_epsilon if error_epsilon is None else float(error_epsilon)
        eps_v = self.velocity_epsilon if velocity_epsilon is None else float(velocity_epsilon)
        drive = 0.0
        if eps_e > 0.0:
            drive += float(position_error) / eps_e
        if eps_v > 0.0:
            drive += float(target_velocity) / eps_v
        if drive == 0.0:
            return 0.0
        floor = float(static_friction) if float(static_friction) > 0.0 else max(self.fs, self.fc)
        magnitude = max(
            0.0,
            self.load_ratio * abs(float(gravity_torque))
            + floor
            + self.modulation_torque(float(theta)),
        )
        return float(magnitude * math.tanh(drive))

    def torque(
        self,
        theta: float,
        velocity: float,
        *,
        position_error: float = 0.0,
        target_velocity: float = 0.0,
        gravity_torque: float = 0.0,
        static_friction: float = 0.0,
        error_epsilon: float | None = None,
        velocity_epsilon: float | None = None,
        cogging_scale: float = 1.0,
        friction_scale: float = 1.0,
    ) -> float:
        """Total modeled joint friction torque tau_fric in Nm.

        Combines Stribeck + periodic ripple when moving with error/target-velocity
        breakaway stiction at rest so static friction deadband is overcome smoothly.
        """
        theta_f = float(theta)
        mod_raw, cog_raw = self._mod_and_cogging(theta_f)
        eps_e = self.error_epsilon if error_epsilon is None else float(error_epsilon)
        eps_v = self.velocity_epsilon if velocity_epsilon is None else float(velocity_epsilon)
        drive = 0.0
        if eps_e > 0.0:
            drive += float(position_error) / eps_e
        if eps_v > 0.0:
            drive += float(target_velocity) / eps_v
        if drive == 0.0:
            stiction = 0.0
        else:
            floor = float(static_friction) if float(static_friction) > 0.0 else max(self.fs, self.fc)
            magnitude = max(0.0, self.load_ratio * abs(float(gravity_torque)) + floor + mod_raw)
            stiction = float(magnitude * math.tanh(drive))
        s = abs(velocity)
        if s == 0.0:
            return float(friction_scale * stiction)
        # 1. 高速衰减包络 w(dq) = exp(-(dq / vr)^2)：低速接近 1（完整补偿齿轮感），高速趋近 0（平滑退火）
        env = self.envelope(velocity)
        # 2. 零速平滑门控 tanh(|dq| / v0)：静止时为 0 防止齿槽补偿在零速自激，一旦运动迅速升至 1
        tanh_v = math.tanh(velocity / self.v0)
        motion_gate = abs(tanh_v)
        # 3. 与运动方向相关的摩擦项：随 |dq| 从静摩擦平滑过渡到 Stribeck + 周期啮合调制 + 粘滞摩擦
        dry = (
            self.load_ratio * abs(float(gravity_torque))
            + self.fc
            + (self.fs - self.fc) * math.exp(-((s / self.vs) ** self.delta))
        )
        mod = env * mod_raw
        kinetic = max(0.0, dry + mod) * tanh_v + self.fv * velocity
        directional = (1.0 - motion_gate) * stiction + motion_gate * kinetic
        # 4. 与运动方向无关的齿槽转矩项：w(dq) * tanh(|dq| / v0) * tau_cog(theta)
        cog = env * motion_gate * cog_raw
        return float(friction_scale * directional + cogging_scale * cog)

    def to_dict(self) -> dict:
        out = {
            'fc': float(self.fc),
            'fs': float(self.fs),
            'vs': float(self.vs),
            'delta': float(self.delta),
            'fv': float(self.fv),
            'v0': float(self.v0),
            'vr': float(self.vr),
            'harmonics': [h.to_dict() for h in self.harmonics],
        }
        if self.load_ratio != 0.0:
            out['load_ratio'] = float(self.load_ratio)
        if self.error_epsilon != 0.05:
            out['error_epsilon'] = float(self.error_epsilon)
        if self.velocity_epsilon != 0.02:
            out['velocity_epsilon'] = float(self.velocity_epsilon)
        return out

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
            load_ratio=float(data.get('load_ratio', 0.0)),
            error_epsilon=float(data.get('error_epsilon', 0.05)),
            velocity_epsilon=float(data.get('velocity_epsilon', 0.02)),
            harmonics=harmonics,
        )


_DEFAULT_JOINT_FRICTION = JointFrictionModel()


@dataclass(frozen=True)
class GripperFrictionModel:
    """Non-periodic spatial profile + Stribeck friction model for the gripper / teaching handle.

    Unlike rotary joints J1..J6 (which exhibit periodic gear/cogging harmonics), the gripper uses
    a linkage/guideway mechanism whose transmission ratio, guide rail normal preload, and return
    spring force vary non-periodically across the opening stroke w in [0, w_max] (m):
    - w_grid: monotonically increasing opening width knots (m)
    - bias_grid: direction-independent position bias force F_bias(w) at each knot (N, half-sum)
    - modulation_grid: direction-dependent spatial friction variation dF_c(w) at each knot (N, centered half-diff)
    """

    fc: float = 0.0
    fs: float = 0.0
    vs: float = 0.015
    delta: float = 2.0
    fv: float = 0.0
    v0: float = 0.002
    vr: float = 0.08
    kp_equiv: float = 45.0
    kv_equiv: float = 12.5
    w_grid: tuple[float, ...] = ()
    bias_grid: tuple[float, ...] = ()
    modulation_grid: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        for name in ('fc', 'fs', 'vs', 'delta', 'fv', 'v0', 'vr', 'kp_equiv', 'kv_equiv'):
            val = getattr(self, name)
            if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(val):
                raise ValueError(f'{name} must be finite')
        for pos_name in ('vs', 'v0', 'vr', 'delta', 'kp_equiv', 'kv_equiv'):
            if getattr(self, pos_name) <= 0:
                raise ValueError(f'{pos_name} must be finite and positive')
        n = len(self.w_grid)
        if len(self.bias_grid) != n or len(self.modulation_grid) != n:
            raise ValueError('w_grid, bias_grid, and modulation_grid must have identical lengths')
        for seq_name in ('w_grid', 'bias_grid', 'modulation_grid'):
            for val in getattr(self, seq_name):
                if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(val):
                    raise ValueError(f'{seq_name} entries must be finite numbers')
        for i in range(1, n):
            if self.w_grid[i] <= self.w_grid[i - 1]:
                raise ValueError('w_grid must be strictly increasing')

    def bias_force(self, width: float) -> float:
        """Direction-independent non-periodic position bias force F_bias(w) in N."""
        return _interp_1d(float(width), self.w_grid, self.bias_grid)

    def modulation_force(self, width: float) -> float:
        """Direction-dependent non-periodic friction variation dF_c(w) in N."""
        return _interp_1d(float(width), self.w_grid, self.modulation_grid)

    def cogging_torque(self, width: float) -> float:
        """Alias for bias_force(width) to keep unified diagnostic interface."""
        return self.bias_force(width)

    def modulation_torque(self, width: float) -> float:
        """Alias for modulation_force(width) to keep unified diagnostic interface."""
        return self.modulation_force(width)

    def local_fc(self, width: float) -> float:
        """Position-dependent Coulomb friction magnitude F_c(w) >= 0 at opening width."""
        return max(0.0, self.fc + self.modulation_force(width))

    def envelope(self, velocity: float) -> float:
        """Speed decay envelope w(dw) = exp(-(dw / vr)^2)."""
        ratio = velocity / self.vr
        return math.exp(-(ratio * ratio))

    def stribeck_magnitude(self, speed: float) -> float:
        """Mean rate-dependent friction magnitude F(|dw|) for speed >= 0."""
        s = abs(speed)
        stribeck_term = (self.fs - self.fc) * math.exp(-((s / self.vs) ** self.delta))
        return self.fc + stribeck_term + self.fv * s

    def force(
        self,
        width: float,
        velocity: float,
        *,
        bias_scale: float = 1.0,
        friction_scale: float = 1.0,
    ) -> float:
        """Total modeled gripper friction & position bias force F_grip(w, dw) in N.

        Uses smooth tanh(velocity / v0) without velocity filter lag:
        - velocity == 0.0: strictly 0.0.
        - |velocity| > 0: smoothly transitions to sign(velocity).
        """
        s = abs(velocity)
        if s == 0.0:
            return 0.0
        env = self.envelope(velocity)
        motion_gate = math.tanh(s / self.v0)
        dry = self.fc + (self.fs - self.fc) * math.exp(-((s / self.vs) ** self.delta))
        mod = env * self.modulation_force(width)
        directional = max(0.0, dry + mod) * math.tanh(velocity / self.v0) + self.fv * velocity
        bias = motion_gate * self.bias_force(width)
        return friction_scale * directional + bias_scale * bias

    def torque(
        self,
        theta: float,
        velocity: float,
        *,
        cogging_scale: float = 1.0,
        friction_scale: float = 1.0,
    ) -> float:
        """Unified evaluation alias matching JointFrictionModel.torque signature."""
        return self.force(
            theta, velocity, bias_scale=cogging_scale, friction_scale=friction_scale,
        )

    def compensate_command(
        self,
        w_cmd: float,
        effort_cmd: float,
        w_meas: float,
        velocity: float,
        *,
        scale: float = 1.0,
    ) -> tuple[float, float, float]:
        """Compute unified velocity-indexed (width_out, effort_out, f_comp) for CAN 0x159.

        Unified across master, slave, bilateral, and standalone code control:
          f_comp = scale * force(w_meas, velocity)
          effort_out = max(0.0, effort_cmd) + |f_comp|
          w_out = w_cmd                                     (active control: effort_cmd > 0)
                  w_meas + f_comp / kp_equiv + v / kv_equiv (passive master: effort_cmd <= 0)
        """
        base_effort = max(0.0, float(effort_cmd))
        if scale <= 0.0:
            return float(w_cmd), base_effort, 0.0
        v_eff = float(velocity)
        if v_eff == 0.0:
            return float(w_cmd), base_effort, 0.0

        f_comp = float(scale * self.force(w_meas, v_eff))
        if abs(f_comp) < 1e-3:
            return float(w_cmd), base_effort, 0.0

        effort_out = base_effort + abs(f_comp)
        if effort_cmd > 0.0:
            w_out = float(w_cmd)
        else:
            w_out = float(w_meas) + f_comp / self.kp_equiv + v_eff / self.kv_equiv
        return w_out, effort_out, f_comp

    def to_dict(self) -> dict:
        return {
            'fc': float(self.fc),
            'fs': float(self.fs),
            'vs': float(self.vs),
            'delta': float(self.delta),
            'fv': float(self.fv),
            'v0': float(self.v0),
            'vr': float(self.vr),
            'kp_equiv': float(self.kp_equiv),
            'kv_equiv': float(self.kv_equiv),
            'w_grid': [float(x) for x in self.w_grid],
            'bias_grid': [float(x) for x in self.bias_grid],
            'modulation_grid': [float(x) for x in self.modulation_grid],
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'GripperFrictionModel':
        if not isinstance(data, dict):
            raise ValueError('Gripper friction model must be a mapping')
        w_grid = tuple(float(x) for x in data.get('w_grid', ()))
        bias_grid = tuple(float(x) for x in data.get('bias_grid', ()))
        modulation_grid = tuple(float(x) for x in data.get('modulation_grid', ()))
        return cls(
            fc=float(data.get('fc', 0.0)),
            fs=float(data.get('fs', 0.0)),
            vs=float(data.get('vs', 0.015)),
            delta=float(data.get('delta', 2.0)),
            fv=float(data.get('fv', 0.0)),
            v0=float(data.get('v0', 0.002)),
            vr=float(data.get('vr', 0.08)),
            kp_equiv=float(data.get('kp_equiv', 45.0)),
            kv_equiv=float(data.get('kv_equiv', 12.5)),
            w_grid=w_grid,
            bias_grid=bias_grid,
            modulation_grid=modulation_grid,
        )


@dataclass(frozen=True)
class ArmFrictionModel:
    """Six-joint + optional gripper friction model container for one PiPER-X arm."""

    arm_id: str = 'arm_1'
    joints: dict[int, JointFrictionModel] = field(default_factory=dict)
    gripper: GripperFrictionModel | None = None

    def joint(self, index: int) -> JointFrictionModel:
        if not 1 <= index <= 6:
            raise ValueError(f'Joint index must be in 1..6, got {index}')
        return self.joints.get(index, _DEFAULT_JOINT_FRICTION)

    def with_joint(
        self, index: int, model: JointFrictionModel | GripperFrictionModel,
    ) -> 'ArmFrictionModel':
        if index == 7:
            if not isinstance(model, GripperFrictionModel):
                raise ValueError('Joint 7 (gripper) requires a GripperFrictionModel')
            return self.with_gripper(model)
        if not 1 <= index <= 6:
            raise ValueError(f'Joint index must be in 1..6 (or 7 for gripper), got {index}')
        if not isinstance(model, JointFrictionModel):
            raise ValueError('Joints 1..6 require a JointFrictionModel')
        updated = dict(self.joints)
        updated[index] = model
        return ArmFrictionModel(arm_id=self.arm_id, joints=updated, gripper=self.gripper)

    def with_gripper(self, model: GripperFrictionModel | None) -> 'ArmFrictionModel':
        return ArmFrictionModel(arm_id=self.arm_id, joints=dict(self.joints), gripper=model)

    def gripper_force(
        self,
        width: float,
        velocity: float,
        *,
        bias_scale: float = 1.0,
        friction_scale: float = 1.0,
    ) -> float:
        if self.gripper is None:
            return 0.0
        return self.gripper.force(
            float(width),
            float(velocity),
            bias_scale=bias_scale,
            friction_scale=friction_scale,
        )

    def torques(
        self,
        positions: tuple[float, ...] | list[float],
        velocities: tuple[float, ...] | list[float],
        *,
        position_errors: tuple[float, ...] | list[float] = (0.0,) * 6,
        target_velocities: tuple[float, ...] | list[float] = (0.0,) * 6,
        gravity_torques: tuple[float, ...] | list[float] = (0.0,) * 6,
        static_frictions: tuple[float, ...] | list[float] = (0.0,) * 6,
        error_epsilon: float | None = None,
        velocity_epsilon: float | None = None,
        cogging_scale: float = 1.0,
        friction_scale: float = 1.0,
    ) -> tuple[float, ...]:
        if len(positions) < 6 or len(velocities) < 6:
            raise ValueError('Six joint positions and velocities are required')
        return tuple(
            self.joint(index).torque(
                float(positions[index - 1]),
                float(velocities[index - 1]),
                position_error=(
                    float(position_errors[index - 1]) if index - 1 < len(position_errors) else 0.0
                ),
                target_velocity=(
                    float(target_velocities[index - 1]) if index - 1 < len(target_velocities) else 0.0
                ),
                gravity_torque=(
                    float(gravity_torques[index - 1]) if index - 1 < len(gravity_torques) else 0.0
                ),
                static_friction=(
                    float(static_frictions[index - 1]) if index - 1 < len(static_frictions) else 0.0
                ),
                error_epsilon=error_epsilon,
                velocity_epsilon=velocity_epsilon,
                cogging_scale=cogging_scale,
                friction_scale=friction_scale,
            )
            for index in range(1, 7)
        )

    def to_dict(self) -> dict:
        out: dict = {
            'version': 1,
            'arm_id': self.arm_id,
            'joints': {
                f'joint{index}': self.joint(index).to_dict()
                for index in sorted(self.joints)
            },
        }
        if self.gripper is not None:
            out['gripper'] = self.gripper.to_dict()
        return out

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
        raw_gripper = data.get('gripper')
        gripper = (
            GripperFrictionModel.from_dict(raw_gripper)
            if isinstance(raw_gripper, dict)
            else None
        )
        return cls(arm_id=str(data.get('arm_id', 'arm_1')), joints=joints, gripper=gripper)


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
