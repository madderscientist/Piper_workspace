"""Shared effector mass properties in the driver's original link6 frame."""

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import yaml


@dataclass(frozen=True)
class RigidBody:
    mass_kg: float
    com_m: tuple[float, ...]
    inertia: np.ndarray


def combine_bodies(bodies) -> RigidBody:
    bodies = tuple(bodies)
    mass = sum(body.mass_kg for body in bodies)
    center = sum((body.mass_kg * np.asarray(body.com_m) for body in bodies), np.zeros(3)) / mass
    inertia = np.zeros((3, 3))
    for body in bodies:
        delta = np.asarray(body.com_m) - center
        inertia += body.inertia + body.mass_kg * (
            np.dot(delta, delta) * np.eye(3) - np.outer(delta, delta))
    return RigidBody(mass, tuple(map(float, center)), inertia)


@dataclass(frozen=True)
class CompositeTerminal:
    model: str
    payload_link: str | None
    components: dict[str, RigidBody]
    max_joint_inertia_kg_m2: tuple[float, ...]

    def body(self) -> RigidBody:
        return combine_bodies(self.components.values())


@lru_cache(maxsize=None)
def terminal_profiles() -> dict:
    source_config = Path(__file__).resolve().parent.parent / 'config/effectors.yaml'
    path = source_config if source_config.is_file() else (
        Path(get_package_share_directory('piper_x_driver')) / 'config/effectors.yaml')
    return yaml.safe_load(path.read_text(encoding='utf-8'))['effectors']


@lru_cache(maxsize=None)
def terminal_for_model(model: str) -> CompositeTerminal | None:
    profiles = terminal_profiles()
    data = profiles.get(model) or next(
        (profile for profile in profiles.values() if profile['model'] == model), None)
    if data is None:
        return None
    return CompositeTerminal(
        data['model'], data['payload_link'], {
            name: RigidBody(
                float(part['mass_kg']), tuple(part['com_m']),
                np.asarray(part['inertia_per_kg'], dtype=float) * float(part['mass_kg']))
            for name, part in data['components'].items()
        }, tuple(data['max_joint_inertia_kg_m2']))


@lru_cache(maxsize=128)
def terminal_body(
    model: str, added_mass_kg: float = 0.0,
    nominal_mass_kg: float | None = None, nominal_com_m: tuple[float, ...] | None = None,
) -> RigidBody:
    terminal = terminal_for_model(model)
    if terminal is None:
        raise ValueError(f'No composite terminal geometry for {model}')
    nominal = terminal.body()
    mass = nominal.mass_kg if nominal_mass_kg is None else nominal_mass_kg
    center = nominal.com_m if nominal_com_m is None else nominal_com_m
    bodies = [RigidBody(mass, center, nominal.inertia * (mass / nominal.mass_kg))]
    if added_mass_kg and terminal.payload_link is not None:
        shape = terminal.components[terminal.payload_link]
        bodies.append(RigidBody(
            added_mass_kg, shape.com_m, shape.inertia * (added_mass_kg / shape.mass_kg)))
    return combine_bodies(bodies)
