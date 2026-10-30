from concurrent.futures import ThreadPoolExecutor
import importlib.util
from pathlib import Path
import shutil
from typing import Any
import xml.etree.ElementTree as ET
from unittest.mock import patch

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from launch import LaunchContext
import yaml

from piper_x_driver.configuration import default_config_path, FleetConfig
from piper_x_driver.gravity import _GRAVITY, _MASS, _kinematics, gravity_torques


@pytest.fixture
def fleet_path(tmp_path):
    target = tmp_path / 'arms.yaml'
    shutil.copyfile(default_config_path(), target)
    return target


def test_hardware_mapping_and_defaults(fleet_path):
    fleet = FleetConfig(fleet_path)
    assert [arm.can_port for arm in fleet.arms] == ['can0', 'can1', 'can2', 'can3']
    assert [arm.namespace for arm in fleet.arms] == [f'/arm_{i}' for i in range(1, 5)]
    assert [arm.end_effector for arm in fleet.arms] == [
        'teaching_handle', 'teaching_handle', 'gripper', 'gripper']
    assert all(arm.gravity_scale == 1 for arm in fleet.arms)
    assert all(0 < arm.friction_scale <= 1 for arm in fleet.arms)
    assert all(arm.friction_config.endswith(f'{arm.id}_friction.yaml') for arm in fleet.arms)
    assert all(fleet.friction_model(arm) is not None for arm in fleet.arms)
    assert all(arm.gravity_config.endswith(f'{arm.id}_gravity.yaml') for arm in fleet.arms)
    assert all(fleet.gravity_model(arm) is not None for arm in fleet.arms)


@pytest.mark.parametrize('field,value', [
    ('gravity_scale', -0.01), ('gravity_scale', 1.01),
    ('gravity_scale', float('nan')), ('gravity_scale', True),
    ('friction_scale', -0.01), ('friction_scale', 1.01),
    ('friction_scale', float('nan')), ('friction_scale', True),
    ('end_effector', 'unknown'), ('mounting', 'sideways'),
    ('can_port', '../can0'), ('namespace', '/bad name'),
    ('debug_kp', [-1.0] * 6), ('debug_kd', [101.0] * 6),
    ('friction_config', '/nonexistent/friction.yaml'),
])
def test_invalid_configuration_is_rejected(fleet_path, field, value):
    data = yaml.safe_load(fleet_path.read_text())
    data['arms'][0][field] = value
    fleet_path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        FleetConfig(fleet_path)


@pytest.mark.parametrize('field', ['can_port', 'id', 'namespace'])
def test_duplicate_identity_is_rejected(fleet_path, field):
    data = yaml.safe_load(fleet_path.read_text())
    data['arms'][1][field] = data['arms'][0][field]
    fleet_path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='Duplicate'):
        FleetConfig(fleet_path)


def test_concurrent_updates_preserve_other_arms_and_resolve_install_symlink(fleet_path):
    link = fleet_path.parent / 'installed.yaml'
    link.symlink_to(fleet_path)
    initial = FleetConfig(link)

    def save(index):
        return initial.save_arm(initial.arms[index], 'gripper', index / 4)

    with ThreadPoolExecutor(max_workers=4) as executor:
        list(executor.map(save, range(4)))
    assert link.is_symlink()
    updated = FleetConfig(link)
    assert [arm.gravity_scale for arm in updated.arms] == [0, 0.25, 0.5, 0.75]
    assert all(arm.end_effector == 'gripper' for arm in updated.arms)
    assert not list(fleet_path.parent.glob('.arms-*'))


def test_stale_writer_and_failed_write_never_replace_configuration(fleet_path, monkeypatch):
    fleet = FleetConfig(fleet_path)
    fleet.save_arm(fleet.arms[0], 'gripper', 0.5)
    with pytest.raises(ValueError, match='changed externally'):
        fleet.save_arm(fleet.arms[0], 'gripper', 0.6)
    current = FleetConfig(fleet_path)
    before = fleet_path.read_bytes()

    def fail(*_):
        raise OSError('Read-only test filesystem')

    monkeypatch.setattr('piper_x_driver.configuration.os.replace', fail)
    with pytest.raises(OSError, match='Read-only'):
        current.save_arm(current.arms[0], 'gripper', 0.7)
    assert fleet_path.read_bytes() == before
    assert not list(fleet_path.parent.glob('.arms-*'))


@pytest.mark.parametrize('width', [0.0, 0.04, 0.07])
def test_teaching_profile_matches_urdf_combined_terminal_mass_and_com(width):
    path = Path(get_package_share_directory('piper_description'))
    root: Any = ET.parse(path / 'urdf/piper_description_with_teach.urdf').getroot()
    links = {link.attrib['name']: link for link in root.findall('link')}
    transforms = {'teach_base': (np.eye(3), np.zeros(3))}
    for joint in root.findall('joint'):
        if joint.find('parent').attrib['link'] != 'teach_base':
            continue
        origin = joint.find('origin')
        rotation = Rotation.from_euler(
            'xyz', list(map(float, origin.attrib['rpy'].split()))).as_matrix()
        translation = np.array(list(map(float, origin.attrib['xyz'].split())))
        axis = np.array(list(map(float, joint.find('axis').attrib['xyz'].split())))
        q = width / 2 if joint.attrib['name'] == 'joint7' else -width / 2
        transforms[joint.find('child').attrib['link']] = (
            rotation, translation + rotation @ (q * axis))
    mass, moment = 0.0, np.zeros(3)
    for name, (rotation, translation) in transforms.items():
        inertial = links[name].find('inertial')
        value = float(inertial.find('mass').attrib['value'])
        center = np.array(list(map(float, inertial.find('origin').attrib['xyz'].split())))
        mass += value
        moment += value * (translation + rotation @ center)
    profile = FleetConfig(default_config_path()).effectors['teaching_handle']
    assert profile.mass_kg == pytest.approx(mass)
    assert profile.com_m == pytest.approx(moment / mass, abs=1e-8)


@pytest.mark.parametrize('effector', ['teaching_handle', 'gripper'])
def test_configured_gravity_matches_potential_energy_gradient(effector):
    profile = FleetConfig(default_config_path()).effectors[effector]
    position = (0.1, 0.5, -0.5, 0.2, -0.1, 0.3)
    masses = np.append(_MASS[:6], profile.mass_kg)
    numerical = []
    for joint in range(6):
        energies = []
        for delta in (1e-6, -1e-6):
            q = (*position[:joint], position[joint] + delta, *position[joint + 1:])
            _, _, centers = _kinematics(q, profile.com_m)
            energies.append(-np.sum(masses * (centers @ _GRAVITY)))
        numerical.append((energies[0] - energies[1]) / 2e-6)
    assert gravity_torques(position, profile.mass_kg, profile.com_m) == pytest.approx(
        numerical, abs=1e-7)


def test_four_arm_launch_uses_shared_mapping_without_starting_hardware(fleet_path):
    share = Path(get_package_share_directory('piper_x_driver'))
    spec = importlib.util.spec_from_file_location(
        'four_arm_launch', share / 'launch/four_arms.launch.py')
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations['config_file'] = str(fleet_path)
    with patch.object(module, 'Node') as node:
        module._drivers(context)
    assert [call.kwargs['namespace'] for call in node.call_args_list] == [
        f'/arm_{index}' for index in range(1, 5)]
    assert [call.kwargs['parameters'] for call in node.call_args_list] == [
        [{'config_file': str(fleet_path), 'arm_id': f'arm_{index}'}]
        for index in range(1, 5)]
