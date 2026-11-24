"""Fleet configuration: one YAML file per arm; only explicit driver configuration requests write it."""

from dataclasses import asdict, dataclass
import fcntl
import math
import os
from pathlib import Path
import re
import tempfile

import yaml

from .friction import ArmFrictionModel, load_friction_model
from .gravity import ArmGravityModel, load_gravity_model, max_critical_kd
from .terminal import terminal_body, terminal_profiles


@dataclass(frozen=True)
class Effector:
    label: str
    mass_kg: float
    com_m: tuple[float, ...]
    source: str
    model: str


@dataclass(frozen=True)
class ArmConfig:
    id: str
    name: str
    namespace: str
    can_port: str
    end_effector: str
    gravity_scale: float
    mounting: str
    kp: tuple[float, ...]
    kd: tuple[float, ...]
    s_a: float = 2.0
    friction_scale: float = 1.0
    friction_config: str = ''
    gravity_config: str = ''
    friction_error_epsilon: float = 0.05
    friction_velocity_epsilon: float = 0.02
    target_velocity_cutoff_hz: float = 5.0
    adaptive_stiffness_b: float = 0.08
    adaptive_stiffness_power: float = 3.0
    offset_ramp_s: float = 2.0

    @property
    def debug_kp(self) -> tuple[float, ...]:
        return self.kp

    @property
    def debug_kd(self) -> tuple[float, ...]:
        return self.kd

    @property
    def adaptive_stiffness_scale(self) -> float:
        return self.s_a


def default_config_path() -> Path:
    """Directory holding this machine's arm files: $PIPER_CONFIG_DIR/arms."""
    root = os.environ.get('PIPER_CONFIG_DIR')
    if not root:
        raise FileNotFoundError('PIPER_CONFIG_DIR is not set; source setup_env.bash')
    return Path(root) / 'arms'


def bounded(value, name: str, lower: float, upper: float) -> float:
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not lower <= value <= upper):
        raise ValueError(f'{name} must be finite and in [{lower}, {upper}]')
    return float(value)


def _vector(values, name: str, length: int, lower: float, upper: float) -> tuple[float, ...]:
    if not isinstance(values, list) or len(values) != length:
        raise ValueError(f'{name} requires {length} values')
    return tuple(bounded(v, name, lower, upper) for v in values)


def _calibration(raw: str, base: Path, loader):
    """Resolve a calibration path; no path, no file or invalid content means uncalibrated."""
    text = raw.strip()
    if not text:
        return '', None
    path = (base / text).resolve()
    try:
        return str(path), loader(path)
    except Exception:
        return str(path), None


class FleetConfig:
    """Arm directory: every *.yaml in it configures one arm present on this machine."""

    def __init__(self, path: str | Path):
        self.path = Path(path).resolve(strict=True)
        files = sorted(self.path.glob('*.yaml'))
        if not files:
            raise ValueError(f'No arm configuration in {self.path}; copy from config/examples/arms')
        self.effectors: dict[str, Effector] = {}
        for key, profile in terminal_profiles().items():
            body = terminal_body(key)
            self.effectors[key] = Effector(
                profile['label'], float(body.mass_kg), tuple(float(v) for v in body.com_m),
                profile['source'], profile['model'],
            )
        self.arms: list[ArmConfig] = []
        self.files: dict[str, Path] = {}
        self.friction_models: dict[str, ArmFrictionModel | None] = {}
        self.gravity_models: dict[str, ArmGravityModel | None] = {}
        for file in files:
            file = file.resolve()
            try:
                arm, friction_model, gravity_model = self._load_arm(file)
            except ValueError as error:
                raise ValueError(f'{file.name}: {error}') from error
            self.arms.append(arm)
            self.files[arm.id] = file
            self.friction_models[arm.id] = friction_model
            self.gravity_models[arm.id] = gravity_model
        for name in ('id', 'namespace', 'can_port'):
            if len({getattr(arm, name) for arm in self.arms}) != len(self.arms):
                raise ValueError(f'Duplicate {name}')

    def _load_arm(
        self, file: Path,
    ) -> tuple[ArmConfig, ArmFrictionModel | None, ArmGravityModel | None]:
        values = yaml.safe_load(file.read_text(encoding='utf-8'))
        if not isinstance(values, dict):
            raise ValueError('Invalid arm configuration fields')
        raw_values = dict(values)
        if 'kp' not in raw_values and 'debug_kp' in raw_values:
            raw_values['kp'] = raw_values.pop('debug_kp')
        elif 'debug_kp' in raw_values:
            legacy_kp = _vector(raw_values.pop('debug_kp'), 'debug_kp', 6, 0.0, 500.0)
            if 'kp' not in values:
                raw_values['kp'] = list(legacy_kp)
        if 'kd' not in raw_values and 'debug_kd' in raw_values:
            raw_values['kd'] = raw_values.pop('debug_kd')
        elif 'debug_kd' in raw_values:
            legacy_kd = _vector(raw_values.pop('debug_kd'), 'debug_kd', 6, -100.0, 100.0)
            if 'kd' not in values:
                raw_values['kd'] = list(legacy_kd)
        if 's_a' not in raw_values and 'adaptive_stiffness_scale' in raw_values:
            raw_values['s_a'] = raw_values.pop('adaptive_stiffness_scale')
        elif 'adaptive_stiffness_scale' in raw_values:
            legacy_sa = bounded(
                raw_values.pop('adaptive_stiffness_scale'), 'adaptive_stiffness_scale', 0.0, 20.0)
            if 's_a' not in values:
                raw_values['s_a'] = legacy_sa
        kp_vec = _vector(raw_values.get('kp'), 'kp', 6, 0.0, 500.0)
        values = {
            'kd': list(max_critical_kd(kp_vec, str(raw_values.get('end_effector', '')))),
            's_a': 2.0,
            'friction_scale': 1.0,
            'friction_config': '',
            'gravity_config': '',
            'friction_error_epsilon': 0.05,
            'friction_velocity_epsilon': 0.02,
            'target_velocity_cutoff_hz': 5.0,
            'adaptive_stiffness_b': 0.08,
            'adaptive_stiffness_power': 3.0,
            'offset_ramp_s': 2.0,
            **raw_values,
        }
        if values.get('friction_config') is None:
            values['friction_config'] = ''
        if values.get('gravity_config') is None:
            values['gravity_config'] = ''
        if set(values) != set(ArmConfig.__dataclass_fields__):
            raise ValueError('Invalid arm configuration fields')
        if not isinstance(values['friction_config'], str):
            raise ValueError('friction_config must be a string path')
        if not isinstance(values['gravity_config'], str):
            raise ValueError('gravity_config must be a string path')
        for name in ('id', 'name', 'namespace', 'can_port', 'end_effector', 'mounting'):
            if not isinstance(values[name], str) or not values[name]:
                raise ValueError(f'{name} must be a non-empty string')
        if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', values['id']):
            raise ValueError('Invalid arm id')
        if not re.fullmatch(r'(?:/[A-Za-z_][A-Za-z0-9_]*)+', values['namespace']):
            raise ValueError('Invalid arm namespace')
        if not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', values['can_port']):
            raise ValueError('Invalid SocketCAN interface')
        if values['mounting'] != 'upright':
            raise ValueError('Only upright mounting is supported by this gravity model')
        if values['end_effector'] not in self.effectors:
            raise ValueError('Unknown end_effector')
        friction_str, friction_model = _calibration(
            values['friction_config'], file.parent, load_friction_model)
        gravity_str, gravity_model = _calibration(
            values['gravity_config'], file.parent, load_gravity_model)
        candidate = (gravity_model or ArmGravityModel(arm_id=values['id'])).for_effector(
            self.effectors[values['end_effector']].model)
        if gravity_model is not None or candidate.effector_model:
            gravity_model = candidate
        auto_kd = max_critical_kd(kp_vec, str(values.get('end_effector', '')), gravity_model)
        raw_kd = _vector(values['kd'], 'kd', 6, -100.0, 100.0)
        resolved_kd = tuple(auto if v < 0.0 else v for v, auto in zip(raw_kd, auto_kd))
        vel_cutoff = bounded(
            values['target_velocity_cutoff_hz'], 'target_velocity_cutoff_hz', 0.0, 1000.0)
        if vel_cutoff <= 0.0:
            raise ValueError('target_velocity_cutoff_hz must be finite and positive')
        adapt_b = bounded(values['adaptive_stiffness_b'], 'adaptive_stiffness_b', 0.0, 10.0)
        if adapt_b <= 0.0:
            raise ValueError('adaptive_stiffness_b must be finite and positive')
        adapt_power = bounded(
            values['adaptive_stiffness_power'], 'adaptive_stiffness_power', 0.0, 20.0)
        if adapt_power <= 0.0:
            raise ValueError('adaptive_stiffness_power must be finite and positive')
        arm = ArmConfig(
            values['id'], values['name'], values['namespace'],
            values['can_port'], values['end_effector'],
            bounded(values['gravity_scale'], 'gravity_scale', 0, 1),
            values['mounting'],
            kp_vec,
            resolved_kd,
            bounded(values['s_a'], 's_a', 0.0, 20.0),
            bounded(values['friction_scale'], 'friction_scale', 0, 1),
            friction_str,
            gravity_str,
            bounded(values['friction_error_epsilon'], 'friction_error_epsilon', 0.0, 10.0),
            bounded(values['friction_velocity_epsilon'], 'friction_velocity_epsilon', 0.0, 100.0),
            vel_cutoff,
            adapt_b,
            adapt_power,
            bounded(values['offset_ramp_s'], 'offset_ramp_s', 0.0, 30.0),
        )
        return arm, friction_model, gravity_model

    def arm(self, identifier: str) -> ArmConfig:
        for arm in self.arms:
            if arm.id == identifier:
                return arm
        raise ValueError(f'Unknown configured arm: {identifier}')

    def friction_model(self, arm: str | ArmConfig) -> ArmFrictionModel | None:
        arm_id = arm.id if isinstance(arm, ArmConfig) else arm
        return self.friction_models.get(arm_id)

    def gravity_model(self, arm: str | ArmConfig) -> ArmGravityModel | None:
        arm_id = arm.id if isinstance(arm, ArmConfig) else arm
        return self.gravity_models.get(arm_id)

    def public(self, arm: ArmConfig) -> dict:
        data = {
            **asdict(arm), 'config_file': str(self.files[arm.id]),
            'debug_kp': list(arm.kp),
            'debug_kd': list(arm.kd),
            'adaptive_stiffness_scale': float(arm.s_a),
            'effector': asdict(self.effectors[arm.end_effector]),
            'effectors': {key: asdict(value) for key, value in self.effectors.items()},
        }
        model = self.gravity_model(arm)
        if model is not None and model.effector_model:
            effector = self.effectors[arm.end_effector]
            body = model.effector_body(effector.mass_kg, effector.com_m)
            data['gravity_payload'] = {
                'added_mass_kg': model.added_mass_kg,
                'total_mass_kg': body.mass_kg,
                'com_m': body.com_m,
                'inertia_kg_m2': body.inertia.tolist(),
            }
        return data

    def save_arm(
        self,
        current: ArmConfig,
        end_effector: str,
        gravity_scale: float,
        friction_scale: float | None = None,
    ) -> 'FleetConfig':
        bounded(gravity_scale, 'gravity_scale', 0, 1)
        next_friction_scale = (
            current.friction_scale
            if friction_scale is None
            else bounded(friction_scale, 'friction_scale', 0, 1)
        )
        if end_effector not in self.effectors:
            raise ValueError('Unknown end_effector')
        file = self.files[current.id]
        # A separate lock survives atomic replacement of the arm file.
        with file.with_name(file.name + '.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            if FleetConfig(self.path).arm(current.id) != current:
                raise ValueError('Configuration changed externally; restart drivers before saving')
            data = yaml.safe_load(file.read_text(encoding='utf-8'))
            data.update(
                end_effector=end_effector, gravity_scale=gravity_scale,
                friction_scale=next_friction_scale)
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(
                    mode='w', encoding='utf-8', dir=file.parent,
                    prefix=f'.{file.stem}-', suffix='.tmp', delete=False,
                ) as stream:
                    temporary = Path(stream.name)
                    yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
                    stream.flush()
                    os.fsync(stream.fileno())
                self._load_arm(temporary)
                os.chmod(temporary, file.stat().st_mode & 0o777)
                os.replace(temporary, file)
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
        return FleetConfig(self.path)
