"""Shared fleet configuration; only explicit driver configuration requests write it."""

from dataclasses import asdict, dataclass, replace
import fcntl
import math
import os
from pathlib import Path
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
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
    return Path(get_package_share_directory('piper_x_driver')) / 'config/arms.yaml'


def bounded(value, name: str, lower: float, upper: float) -> float:
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not lower <= value <= upper):
        raise ValueError(f'{name} must be finite and in [{lower}, {upper}]')
    return float(value)


def _vector(values, name: str, length: int, lower: float, upper: float) -> tuple[float, ...]:
    if not isinstance(values, list) or len(values) != length:
        raise ValueError(f'{name} requires {length} values')
    return tuple(bounded(v, name, lower, upper) for v in values)


class FleetConfig:
    def __init__(self, path: str | Path):
        self.path = Path(path).resolve(strict=True)
        with self.path.open(encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        if (not isinstance(data, dict) or not {'version', 'arms'} <= set(data)
                or set(data) - {'version', 'effectors', 'arms'}):
            raise ValueError('Fleet configuration requires version and arms')
        if type(data['version']) is not int or data['version'] != 1:
            raise ValueError('Unsupported fleet configuration version')
        self._inline_effectors = data.get('effectors')
        profiles = self._inline_effectors
        if profiles is None:
            profiles = {}
            for key, profile in terminal_profiles().items():
                body = terminal_body(key)
                profiles[key] = {
                    'label': profile['label'], 'model': profile['model'],
                    'source': profile['source'], 'mass_kg': body.mass_kg,
                    'com_m': list(body.com_m),
                }
        arms = data['arms']
        if not isinstance(profiles, dict) or not profiles:
            raise ValueError('At least one effector profile is required')
        self.effectors: dict[str, Effector] = {}
        for key, profile in profiles.items():
            if (not isinstance(key, str) or not isinstance(profile, dict)
                    or set(profile) != set(Effector.__dataclass_fields__)):
                raise ValueError('Invalid effector profile')
            for name in ('label', 'source', 'model'):
                if not isinstance(profile[name], str) or not profile[name].strip():
                    raise ValueError(f'Effector {name} must be a non-empty string')
            if profile['model'] not in (
                'piper_description.urdf', 'piper_description_with_teach.urdf',
                'piper_description_with_wrench.urdf',
                'piper_no_gripper_description.urdf',
            ):
                raise ValueError('Unsupported visualization model')
            self.effectors[key] = Effector(
                profile['label'],
                bounded(profile['mass_kg'], 'mass_kg', 0, 2),
                _vector(profile['com_m'], 'com_m', 3, -0.5, 0.5),
                profile['source'],
                profile['model'],
            )
        if not isinstance(arms, list) or len(arms) != 4:
            raise ValueError('Exactly four configured arms are required')
        self.arms: list[ArmConfig] = []
        self.friction_models: dict[str, ArmFrictionModel | None] = {}
        self.gravity_models: dict[str, ArmGravityModel | None] = {}
        for values in arms:
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
                    raw_values.pop('adaptive_stiffness_scale'),
                    'adaptive_stiffness_scale',
                    0.0,
                    20.0,
                )
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
            friction_str = values['friction_config'].strip()
            friction_model = None
            if friction_str:
                raw_path = Path(friction_str)
                try:
                    resolved_path = (
                        raw_path if raw_path.is_absolute()
                        else (self.path.parent / raw_path)
                    ).resolve(strict=True)
                except OSError as error:
                    raise ValueError(f'friction_config file not found: {friction_str}') from error
                friction_model = load_friction_model(resolved_path)
            self.friction_models[values['id']] = friction_model
            gravity_str = values['gravity_config'].strip()
            gravity_model = None
            if gravity_str:
                raw_g_path = Path(gravity_str)
                try:
                    resolved_g_path = (
                        raw_g_path if raw_g_path.is_absolute()
                        else (self.path.parent / raw_g_path)
                    ).resolve(strict=True)
                except OSError as error:
                    raise ValueError(f'gravity_config file not found: {gravity_str}') from error
                gravity_model = load_gravity_model(resolved_g_path)
            candidate = (gravity_model or ArmGravityModel(arm_id=values['id'])).for_effector(
                self.effectors[values['end_effector']].model)
            if gravity_model is not None or candidate.effector_model:
                gravity_model = candidate
            self.gravity_models[values['id']] = gravity_model
            auto_kd = max_critical_kd(kp_vec, str(values.get('end_effector', '')), gravity_model)
            raw_kd = _vector(values['kd'], 'kd', 6, -100.0, 100.0)
            resolved_kd = tuple(auto if v < 0.0 else v for v, auto in zip(raw_kd, auto_kd))
            error_eps = bounded(
                values['friction_error_epsilon'], 'friction_error_epsilon', 0.0, 10.0,
            )
            vel_eps = bounded(
                values['friction_velocity_epsilon'], 'friction_velocity_epsilon', 0.0, 100.0,
            )
            vel_cutoff = bounded(
                values['target_velocity_cutoff_hz'], 'target_velocity_cutoff_hz', 0.0, 1000.0,
            )
            if vel_cutoff <= 0.0:
                raise ValueError('target_velocity_cutoff_hz must be finite and positive')
            adapt_scale = bounded(values['s_a'], 's_a', 0.0, 20.0)
            adapt_b = bounded(
                values['adaptive_stiffness_b'], 'adaptive_stiffness_b', 0.0, 10.0,
            )
            if adapt_b <= 0.0:
                raise ValueError('adaptive_stiffness_b must be finite and positive')
            adapt_power = bounded(
                values['adaptive_stiffness_power'], 'adaptive_stiffness_power', 0.0, 20.0,
            )
            if adapt_power <= 0.0:
                raise ValueError('adaptive_stiffness_power must be finite and positive')
            ramp_s = bounded(values['offset_ramp_s'], 'offset_ramp_s', 0.0, 30.0)
            self.arms.append(ArmConfig(
                values['id'], values['name'], values['namespace'],
                values['can_port'], values['end_effector'],
                bounded(values['gravity_scale'], 'gravity_scale', 0, 1),
                values['mounting'],
                kp_vec,
                resolved_kd,
                adapt_scale,
                bounded(values['friction_scale'], 'friction_scale', 0, 1),
                friction_str,
                gravity_str,
                error_eps,
                vel_eps,
                vel_cutoff,
                adapt_b,
                adapt_power,
                ramp_s,
            ))
        for name in ('id', 'namespace', 'can_port'):
            if len({getattr(arm, name) for arm in self.arms}) != 4:
                raise ValueError(f'Duplicate {name}')

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
            **asdict(arm), 'config_file': str(self.path),
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
        # A separate lock survives atomic replacement and serializes all four writers.
        with self.path.with_suffix('.yaml.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            latest = FleetConfig(self.path)
            if latest.arm(current.id) != current or latest.effectors != self.effectors:
                raise ValueError('Configuration changed externally; restart drivers before saving')
            data = {
                'version': 1,
                'arms': [
                    asdict(replace(
                        arm,
                        end_effector=end_effector,
                        gravity_scale=gravity_scale,
                        friction_scale=next_friction_scale,
                    ) if arm.id == current.id else arm)
                    for arm in latest.arms
                ],
            }
            if latest._inline_effectors is not None:
                data['effectors'] = latest._inline_effectors
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(
                    mode='w', encoding='utf-8', dir=self.path.parent,
                    prefix='.arms-', suffix='.yaml', delete=False,
                ) as stream:
                    temporary = Path(stream.name)
                    yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
                    stream.flush()
                    os.fsync(stream.fileno())
                updated = FleetConfig(temporary)
                os.chmod(temporary, self.path.stat().st_mode & 0o777)
                os.replace(temporary, self.path)
                updated.path = self.path
                return updated
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
