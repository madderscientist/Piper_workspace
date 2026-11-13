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
from piper_x_driver.terminal import terminal_body, terminal_for_model


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
        'teaching_handle', 'teaching_handle',
        'gripper_with_wrench', 'gripper_with_wrench']
    assert fleet.effectors['gripper_with_wrench'].model == 'piper_description_with_wrench.urdf'
    assert all(arm.gravity_scale == 1 for arm in fleet.arms)
    assert all(arm.s_a == 2.0 for arm in fleet.arms)
    assert all(0 < arm.friction_scale <= 1 for arm in fleet.arms)
    assert all(arm.friction_config.endswith(f'{arm.id}_friction.yaml') for arm in fleet.arms)
    assert all(fleet.friction_model(arm) is not None for arm in fleet.arms)
    assert all(arm.gravity_config.endswith(f'{arm.id}_gravity.yaml') for arm in fleet.arms)
    assert all(fleet.gravity_model(arm) is not None for arm in fleet.arms)
    for key, profile in fleet.effectors.items():
        body = terminal_body(key)
        assert profile.mass_kg == pytest.approx(body.mass_kg)
        assert profile.com_m == pytest.approx(body.com_m)
        term = terminal_for_model(profile.model)
        assert term is not None and term.model == profile.model


@pytest.mark.parametrize('field,value', [
    ('gravity_scale', -0.01), ('gravity_scale', 1.01),
    ('gravity_scale', float('nan')), ('gravity_scale', True),
    ('friction_scale', -0.01), ('friction_scale', 1.01),
    ('friction_scale', float('nan')), ('friction_scale', True),
    ('end_effector', 'unknown'), ('mounting', 'sideways'),
    ('can_port', '../can0'), ('namespace', '/bad name'),
    ('kp', [-1.0] * 6), ('kd', [101.0] * 6), ('s_a', -0.1),
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
    assert 'effectors' not in yaml.safe_load(fleet_path.read_text())
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
    mount = next(j for j in root.findall('joint') if j.attrib['name'] == 'joint6_to_teach_base')
    mount_orig = mount.find('origin')
    mount_rot = Rotation.from_euler(
        'xyz', list(map(float, mount_orig.attrib['rpy'].split()))).as_matrix()
    mount_xyz = np.array(list(map(float, mount_orig.attrib['xyz'].split())))
    transforms = {'teach_base': (mount_rot, mount_xyz)}
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
            mount_rot @ rotation,
            mount_xyz + mount_rot @ (translation + rotation @ (q * axis)),
        )
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


def test_wrench_gripper_profile_matches_uniform_mesh_terminal():
    from piper_x_driver.mesh_inertia import terminal_from_meshes

    path = Path(get_package_share_directory('piper_description')) / 'urdf'
    generated = terminal_from_meshes(
        path / 'piper_description_with_wrench.urdf', path / 'piper_description.urdf')
    config = Path(get_package_share_directory('piper_x_driver')) / 'config/effectors.yaml'
    data = yaml.safe_load(config.read_text())
    profile = data['effectors']['gripper_with_wrench']
    for key, value in generated.items():
        assert value == profile.get(key, data.get(key)), key
    terminal = terminal_for_model('gripper_with_wrench')
    assert terminal is not None
    assert len(terminal.components) == 6
    assert terminal.payload_link == 'gripper_base'


def test_uniform_stl_inertia_matches_translated_box(tmp_path):
    from piper_x_driver.mesh_inertia import stl_mass_properties

    center = np.array([4.0, -5.0, 6.0])
    vertices = np.array([[x, y, z] for x in (-1, 1) for y in (-2, 2) for z in (-3, 3)]) + center
    # Two outward-facing triangles on each box face.
    triangles = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5),
                 (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6),
                 (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    lines = ['solid box']
    for indices in triangles:
        lines += ['facet normal 0 0 0', 'outer loop']
        lines += ['vertex ' + ' '.join(map(str, vertex)) for vertex in vertices[list(indices)]]
        lines += ['endloop', 'endfacet']
    path = tmp_path / 'box.stl'
    path.write_text('\n'.join(lines + ['endsolid box']))
    actual_center, unit_inertia = stl_mass_properties(path)
    assert actual_center == pytest.approx(center)
    np.testing.assert_allclose(unit_inertia, np.diag([52 / 12, 40 / 12, 20 / 12]), atol=1e-12)


@pytest.mark.parametrize('effector', ['teaching_handle', 'gripper', 'gripper_with_wrench'])
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
            energies.append(-float(np.sum(masses * (centers @ _GRAVITY))))
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


def test_cpp_core_matches_python_dynamics_and_can_wire_protocol(capsys):
    import piper_x_driver._core as cpp
    import piper_x_driver.friction as f_mod
    import piper_x_driver.gravity as g_mod
    from piper_x_driver.gravity import damping_main, forward_kinematics, max_critical_kd
    from piper_x_driver.protocol import MAX_GRAVITY_OFFSET, MIT_GAIN_SCALE

    fleet = FleetConfig(default_config_path())
    arm_cfg = fleet.arm('arm_3')
    gm = fleet.gravity_model(arm_cfg)
    fm = fleet.friction_model(arm_cfg)
    eff = fleet.effectors[arm_cfg.end_effector]
    assert gm is not None and fm is not None

    rng = np.random.default_rng(42)
    q = tuple(rng.uniform(-1.5, 1.5, size=6))
    dq = tuple(rng.uniform(-0.8, 0.8, size=6))
    err = tuple(rng.uniform(-0.08, 0.08, size=6))
    sf = tuple(gm.joint(i).static_friction for i in range(1, 7))

    try:
        g_mod._cpp_core = None
        f_mod._cpp_core = None
        g_mod._gravity_torques_cached.cache_clear()
        g_mod._joint_inertias_cached.cache_clear()
        py_g = gm.torques(q, eff.mass_kg, eff.com_m)
        py_f = fm.torques(q, dq, position_errors=err, gravity_torques=py_g, static_frictions=sf)
        py_w = g_mod.end_effector_wrench(py_g, q, gravity_model=gm)
        py_fk = forward_kinematics(q, (0.01, -0.02, 0.08))
    finally:
        g_mod._cpp_core = cpp
        f_mod._cpp_core = cpp
        g_mod._gravity_torques_cached.cache_clear()
        g_mod._joint_inertias_cached.cache_clear()

    assert gm.torques(q, eff.mass_kg, eff.com_m) == pytest.approx(py_g, abs=1e-12)
    assert fm.torques(
        q, dq, position_errors=err, gravity_torques=py_g, static_frictions=sf,
    ) == pytest.approx(py_f, abs=1e-12)
    assert g_mod.end_effector_wrench(py_g, q, gravity_model=gm) == pytest.approx(py_w, abs=1e-12)
    assert forward_kinematics(q, (0.01, -0.02, 0.08)) == pytest.approx(py_fk, abs=1e-12)

    frames = cpp.encode_mit_command(
        (0.0, 0.72, -0.8, 0.0, 0.0, 0.0),
        (0.0,) * 6,
        (5.0, 5.0, 5.0, 1.0, 0.7, 0.7),
        (0.5, 0.8, 0.8, 0.2, 0.2, 0.2),
        (0.0,) * 6,
        (0.0, 0.52, -0.8, 0.33, 0.0, 0.0),
    )
    assert [f['can_id'] for f in frames] == [0x15A, 0x15B, 0x15C, 0x15D, 0x15E, 0x15F]
    assert all(len(f['data']) == 8 for f in frames)
    assert max_critical_kd((15.0, 15.0, 15.0, 3.0, 2.0, 2.0), 'gripper') == (
        4.56, 4.6, 2.22, 0.29, 0.24, 0.05)
    desc_urdf = Path(get_package_share_directory('piper_description')) / 'urdf/piper_description.urdf'
    urdf_root = ET.parse(desc_urdf).getroot()
    T_urdf = np.eye(4)
    for idx, qi in enumerate(q, 1):
        j_el = next(el for el in urdf_root.findall('joint') if el.attrib['name'] == f'joint{idx}')
        orig = j_el.find('origin')
        Tj = np.eye(4)
        Tj[:3, :3] = Rotation.from_euler('xyz', list(map(float, orig.attrib['rpy'].split()))).as_matrix()
        Tj[:3, 3] = list(map(float, orig.attrib['xyz'].split()))
        Tr = np.eye(4)
        Tr[:3, :3] = Rotation.from_euler('z', qi).as_matrix()
        T_urdf = T_urdf @ Tj @ Tr
    assert forward_kinematics(q) == pytest.approx(T_urdf, abs=1e-12)
    assert damping_main(['--kp', '60', '60', '60', '12', '8', '8']) == 0
    assert 'kd: [9.25, 9.31, 4.51, 0.6, 0.49, 0.13]' in capsys.readouterr().out
    assert len(MIT_GAIN_SCALE) == 6 and MAX_GRAVITY_OFFSET == 1.0


def test_inverse_kinematics_matrix_rpy_and_calibrated_offsets():
    from piper_x_driver import IkResult, forward_kinematics, inverse_kinematics
    from piper_x_driver.gravity import ArmGravityModel, JointGravityModel

    q_true = (0.3, 1.0, -1.3, -0.2, 0.45, 0.5)
    tool_off = (0.01, -0.02, 0.16)
    T_des = forward_kinematics(q_true, tool_offset_m=tool_off)
    q_seed = (0.15, 0.85, -1.1, 0.0, 0.3, 0.2)

    res = inverse_kinematics(T_des, q_seed, tool_offset_m=tool_off, return_result=True)
    assert isinstance(res, IkResult) and res.converged
    assert forward_kinematics(res.joints, tool_offset_m=tool_off) == pytest.approx(T_des, abs=1e-4)

    rpy = Rotation.from_matrix(T_des[:3, :3]).as_euler('xyz')
    pose_6d = (*T_des[:3, 3], *rpy)
    q_6d = inverse_kinematics(pose_6d, q_seed, tool_offset_m=tool_off)
    assert isinstance(q_6d, tuple) and len(q_6d) == 6
    assert forward_kinematics(q_6d, tool_offset_m=tool_off) == pytest.approx(T_des, abs=1e-4)

    gm = ArmGravityModel(joints={2: JointGravityModel(angle_offset=0.025), 5: JointGravityModel(angle_offset=-0.03)})
    T_cal = gm.forward_kinematics(q_true, tool_offset_m=tool_off)
    q_cal = gm.inverse_kinematics(T_cal, q_seed, tool_offset_m=tool_off)
    assert gm.forward_kinematics(q_cal, tool_offset_m=tool_off) == pytest.approx(T_cal, abs=1e-4)
