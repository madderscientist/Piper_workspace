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
    debug_kp: tuple[float, ...]
    debug_kd: tuple[float, ...]
    friction_scale: float = 1.0
    friction_config: str = ''
    gravity_config: str = ''


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
        if not isinstance(data, dict) or set(data) != {'version', 'effectors', 'arms'}:
            raise ValueError('Fleet configuration requires version, effectors and arms')
        if type(data['version']) is not int or data['version'] != 1:
            raise ValueError('Unsupported fleet configuration version')
        profiles, arms = data['effectors'], data['arms']
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
            debug_kp = _vector(values.get('debug_kp'), 'debug_kp', 6, 0.0, 500.0)
            values = {
                'debug_kd': list(max_critical_kd(debug_kp, str(values.get('end_effector', '')))),
                'friction_scale': 1.0,
                'friction_config': '',
                'gravity_config': '',
                **values,
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
            self.gravity_models[values['id']] = gravity_model
            self.arms.append(ArmConfig(
                values['id'], values['name'], values['namespace'],
                values['can_port'], values['end_effector'],
                bounded(values['gravity_scale'], 'gravity_scale', 0, 1),
                values['mounting'],
                _vector(values['debug_kp'], 'debug_kp', 6, 0.0, 500.0),
                _vector(values['debug_kd'], 'debug_kd', 6, 0.0, 100.0),
                bounded(values['friction_scale'], 'friction_scale', 0, 1),
                friction_str,
                gravity_str,
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
        return {
            **asdict(arm), 'config_file': str(self.path),
            'effector': asdict(self.effectors[arm.end_effector]),
            'effectors': {key: asdict(value) for key, value in self.effectors.items()},
        }

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
                'effectors': {key: asdict(value) for key, value in latest.effectors.items()},
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
