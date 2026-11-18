"""MuJoCo simulation of PiPER-X with wrist PX6D-75A force/torque sensor for InsertAnything."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Any, Sequence
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from piper_x_driver.configuration import ArmConfig, FleetConfig, default_config_path
from piper_x_driver.friction import ArmFrictionModel
from piper_x_driver.gravity import ArmGravityModel, geometric_jacobian
from piper_x_driver.protocol import MAX_GRAVITY_OFFSET, MIT_KD_LSB, MIT_KP_LSB, MIT_POSITION_LIMIT

from .policy import (
    CARTESIAN_CARRIER_LSB_MULT,
    CARTESIAN_TIP_KP,
    DEFAULT_TIP_OFFSET_M,
    PAXINI_SENSOR_OFFSET_M,
    POLICY_PERIOD_S,
    GoalPose,
    InsertAnythingPolicy,
    PolicyStepResult,
    RuleBasedInsertPolicy,
    WrenchFilter,
    canonical_quat,
    default_onnx_path,
    forward_tip_pose,
    hole_target_to_base,
    mat_to_quat,
    normalize_tip_offset,
    quat_inv,
    quat_mul,
    quat_rotvec,
    quat_to_mat,
    relative_pose_in_hole,
    resolve_cartesian_damping,
    rotate,
    rotvec_quat,
    solve_tip_ik,
)

SHAPES = ('triangle', 'square', 'pentagon', 'hexagon', 'l', 'circle')
DEFAULT_Q_INSERTED: tuple[float, ...] = (
    -1.316,
    math.radians(107.0),
    math.radians(-85.0),
    math.radians(72.6),
    0.0,
    0.0,
)


def default_wrench_urdf_path() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / 'src' / 'piper_description' / 'urdf' / 'piper_description_with_wrench.urdf'
        if cand.is_file():
            return cand
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('piper_description')) / 'urdf/piper_description_with_wrench.urdf'
    except Exception as exc:
        raise FileNotFoundError('Cannot locate piper_description_with_wrench.urdf') from exc


def _euler_xyz_to_quat_wxyz(rpy: Sequence[float]) -> np.ndarray:
    rx, ry, rz = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cx, sx = math.cos(rx * 0.5), math.sin(rx * 0.5)
    cy, sy = math.cos(ry * 0.5), math.sin(ry * 0.5)
    cz, sz = math.cos(rz * 0.5), math.sin(rz * 0.5)
    return canonical_quat([
        cx * cy * cz + sx * sy * sz,
        sx * cy * cz - cx * sy * sz,
        cx * sy * cz + sx * cy * sz,
        cx * cy * sz - sx * sy * cz,
    ])


def _parse_floats(text: str | None, default: Sequence[float]) -> tuple[float, ...]:
    if not text:
        return tuple(float(v) for v in default)
    return tuple(float(v) for v in text.split())


def _inertia_elements_to_full(attrib: dict[str, str], rpy: Sequence[float] = (0.0, 0.0, 0.0)) -> str:
    ixx, iyy, izz = float(attrib['ixx']), float(attrib['iyy']), float(attrib['izz'])
    ixy, ixz, iyz = float(attrib['ixy']), float(attrib['ixz']), float(attrib['iyz'])
    imat = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=np.float64)
    if any(abs(float(v)) > 1e-12 for v in rpy):
        rot = quat_to_mat(_euler_xyz_to_quat_wxyz(rpy))
        imat = rot @ imat @ rot.T
    vals = [imat[0, 0], imat[1, 1], imat[2, 2], imat[0, 1], imat[0, 2], imat[1, 2]]
    return ' '.join(f'{v:.12g}' for v in vals)


def _inset_polygon(poly: np.ndarray, dist: float) -> np.ndarray:
    edges = np.roll(poly, -1, axis=0) - poly
    inward = np.column_stack((-edges[:, 1], edges[:, 0]))
    inward /= np.linalg.norm(inward, axis=1)[:, None]
    bounds = (inward * poly).sum(axis=1) + dist
    return np.array([
        np.linalg.solve(np.stack((inward[i - 1], inward[i])), np.array((bounds[i - 1], bounds[i])))
        for i in range(len(poly))
    ], dtype=np.float64)


def _shape_polygons(shape: str, radius: float, clearance: float) -> tuple[np.ndarray, np.ndarray]:
    if shape not in SHAPES:
        raise ValueError(f'Unsupported shape {shape!r}; choose from {SHAPES}')
    r, c = float(radius), float(clearance) * 0.5
    if shape == 'l':
        hole = r * np.array([[-1, -1], [1, -1], [1, 0], [0, 0], [0, 1], [-1, 1]], dtype=np.float64)
        peg = np.array([
            [-r + c, -r + c], [r - c, -r + c], [r - c, -c],
            [-c, -c], [-c, r - c], [-r + c, r - c],
        ], dtype=np.float64)
        return hole, peg
    n = {'triangle': 3, 'square': 4, 'pentagon': 5, 'hexagon': 6, 'circle': 64}[shape]
    angles = np.arange(n, dtype=np.float64) * (2.0 * math.pi / n) + 0.5 * math.pi
    hole = r * np.column_stack((np.cos(angles), np.sin(angles)))
    return hole, _inset_polygon(hole, c)


def _add_box_geom(
    parent: ET.Element, name: str, low_xy: Sequence[float], high_xy: Sequence[float],
    z_low: float, z_high: float, rgba: str, *, yaw: float = 0.0, contype: int = 1, conaffinity: int = 2,
) -> None:
    low = np.asarray(low_xy, dtype=np.float64)
    high = np.asarray(high_xy, dtype=np.float64)
    center = np.array([(low[0] + high[0]) * 0.5, (low[1] + high[1]) * 0.5, (z_low + z_high) * 0.5])
    size = np.array([(high[0] - low[0]) * 0.5, (high[1] - low[1]) * 0.5, (z_high - z_low) * 0.5])
    x, y = float(center[0]), float(center[1])
    center[0] = math.cos(yaw) * x - math.sin(yaw) * y
    center[1] = math.sin(yaw) * x + math.cos(yaw) * y
    ET.SubElement(
        parent, 'geom', name=name, type='box',
        pos=' '.join(f'{v:.8g}' for v in center),
        size=' '.join(f'{v:.8g}' for v in size),
        quat=f'{math.cos(0.5 * yaw):.8g} 0 0 {math.sin(0.5 * yaw):.8g}',
        rgba=rgba, contype=str(contype), conaffinity=str(conaffinity),
    )


def _add_prism_geom(
    parent: ET.Element, asset: ET.Element, name: str, polygon: np.ndarray,
    z_low: float, z_high: float, rgba: str, *, contype: int, conaffinity: int,
) -> None:
    verts = np.concatenate((
        np.column_stack((polygon, np.full(len(polygon), z_low))),
        np.column_stack((polygon, np.full(len(polygon), z_high))),
    ))
    ET.SubElement(asset, 'mesh', name=name, vertex=' '.join(f'{v:.8g}' for v in verts.ravel()))
    ET.SubElement(
        parent, 'geom', name=name, type='mesh', mesh=name, rgba=rgba,
        contype=str(contype), conaffinity=str(conaffinity),
    )


@dataclass
class SimSceneConfig:
    shape: str = 'hexagon'
    clearance_m: float = 0.0005
    radius_m: float = 0.010
    hole_depth_m: float = 0.025
    peg_length_m: float = 0.035
    plate_outer_radius_m: float = 0.060
    contact_friction: float = 0.45
    physics_dt: float = 0.001
    gravity_z: float = -9.81
    tip_offset_m: tuple[float, float, float] = DEFAULT_TIP_OFFSET_M
    plug_mass_kg: float = 0.030
    added_gripper_mass_kg: float = 0.0
    joint_damping: tuple[float, ...] = (0.08, 0.08, 0.05, 0.02, 0.015, 0.01)
    joint_frictionloss: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def build_piper_wrench_mjcf(
    hole_pos_b: Sequence[float],
    hole_quat_bh: Sequence[float],
    cfg: SimSceneConfig = SimSceneConfig(),
    *,
    urdf_path: str | Path | None = None,
) -> str:
    """Convert piper_description_with_wrench.urdf into a complete MuJoCo scene XML with FT sensor, peg, and hole."""
    urdf_file = Path(urdf_path) if urdf_path is not None else default_wrench_urdf_path()
    mesh_dir = (urdf_file.parent.parent / 'meshes').resolve(strict=True)
    robot_xml = ET.parse(urdf_file).getroot()

    links = {el.attrib['name']: el for el in robot_xml.findall('link')}
    children_by_parent: dict[str, list[ET.Element]] = {}
    for j_el in robot_xml.findall('joint'):
        p_el = j_el.find('parent')
        assert p_el is not None
        children_by_parent.setdefault(p_el.attrib['link'], []).append(j_el)

    mj_root = ET.Element('mujoco', model=f'piper_wrench_insert_{cfg.shape}')
    ET.SubElement(
        mj_root, 'compiler', angle='radian', meshdir=str(mesh_dir),
        balanceinertia='true', boundmass='1e-6', boundinertia='1e-8',
    )
    ET.SubElement(
        mj_root, 'option', timestep=f'{cfg.physics_dt:.6g}',
        gravity=f'0 0 {cfg.gravity_z:.6g}', integrator='implicitfast',
        iterations='60', ls_iterations='20',
    )
    ET.SubElement(mj_root, 'size', nconmax='256', njmax='1024')

    default = ET.SubElement(mj_root, 'default')
    ET.SubElement(
        default, 'geom', condim='3', friction=f'{cfg.contact_friction:.4g} 0.005 0.0001',
        solref='0.002 1', solimp='0.99 0.999 0.00001', margin='0', gap='0',
    )
    asset = ET.SubElement(mj_root, 'asset')
    seen_meshes: set[str] = set()

    def register_urdf_mesh(uri: str) -> str:
        fname = Path(uri.replace('package://piper_description/meshes/', '')).name
        mname = f'urdf_{Path(fname).stem}'
        if mname not in seen_meshes:
            seen_meshes.add(mname)
            ET.SubElement(asset, 'mesh', name=mname, file=fname)
        return mname

    def add_link_visual_and_inertial(body_el: ET.Element, link_name: str, extra_mass: float = 0.0) -> None:
        l_el = links[link_name]
        iner = l_el.find('inertial')
        if iner is not None:
            i_orig = iner.find('origin')
            ixyz = _parse_floats(i_orig.attrib.get('xyz') if i_orig is not None else None, (0.0, 0.0, 0.0))
            irpy = _parse_floats(i_orig.attrib.get('rpy') if i_orig is not None else None, (0.0, 0.0, 0.0))
            mass_el = iner.find('mass')
            imat_el = iner.find('inertia')
            assert mass_el is not None and imat_el is not None
            base_m = float(mass_el.attrib['value'])
            total_m = base_m + float(extra_mass)
            scale = total_m / base_m if base_m > 0.0 else 1.0
            scaled_attr = {k: str(float(v) * scale) for k, v in imat_el.attrib.items()}
            ET.SubElement(
                body_el, 'inertial',
                pos=' '.join(f'{v:.10g}' for v in ixyz),
                mass=f'{total_m:.10g}',
                fullinertia=_inertia_elements_to_full(scaled_attr, irpy),
            )
        vis = l_el.find('visual')
        if vis is not None:
            v_orig = vis.find('origin')
            vxyz = _parse_floats(v_orig.attrib.get('xyz') if v_orig is not None else None, (0.0, 0.0, 0.0))
            vrpy = _parse_floats(v_orig.attrib.get('rpy') if v_orig is not None else None, (0.0, 0.0, 0.0))
            vquat = _euler_xyz_to_quat_wxyz(vrpy)
            mesh_el = vis.find('geometry/mesh')
            color_el = vis.find('material/color')
            rgba = color_el.attrib.get('rgba', '0.8 0.82 0.93 1') if color_el is not None else '0.8 0.82 0.93 1'
            if mesh_el is not None:
                mname = register_urdf_mesh(mesh_el.attrib['filename'])
                ET.SubElement(
                    body_el, 'geom', name=f'vis_{link_name}', type='mesh', mesh=mname,
                    pos=' '.join(f'{v:.10g}' for v in vxyz),
                    quat=' '.join(f'{v:.10g}' for v in vquat),
                    rgba=rgba, contype='0', conaffinity='0', group='1',
                )

    world = ET.SubElement(mj_root, 'worldbody')
    ET.SubElement(world, 'light', pos='0.3 -0.3 0.9', dir='0 0.2 -1', diffuse='0.85 0.85 0.85')
    ET.SubElement(world, 'light', pos='-0.2 0.4 0.8', dir='0.2 -0.3 -1', diffuse='0.45 0.45 0.5')
    ET.SubElement(
        world, 'geom', name='table_plane', type='plane', pos='0 0 -0.04', size='1.0 1.0 0.01',
        rgba='0.18 0.20 0.24 1', contype='0', conaffinity='0',
    )

    base_body = ET.SubElement(world, 'body', name='base_link', pos='0 0 0')
    add_link_visual_and_inertial(base_body, 'base_link')

    offset_gripper = normalize_tip_offset(cfg.tip_offset_m)
    r, hole_poly, peg_poly = cfg.radius_m, *_shape_polygons(cfg.shape, cfg.radius_m, cfg.clearance_m)

    def build_subtree(parent_body: ET.Element, parent_link: str) -> None:
        for j_el in children_by_parent.get(parent_link, []):
            j_name = j_el.attrib['name']
            j_type = j_el.attrib['type']
            c_el = j_el.find('child')
            assert c_el is not None
            child_link = c_el.attrib['link']
            orig = j_el.find('origin')
            xyz = _parse_floats(orig.attrib.get('xyz') if orig is not None else None, (0.0, 0.0, 0.0))
            rpy = _parse_floats(orig.attrib.get('rpy') if orig is not None else None, (0.0, 0.0, 0.0))
            quat = _euler_xyz_to_quat_wxyz(rpy)

            child_body = ET.SubElement(
                parent_body, 'body', name=child_link,
                pos=' '.join(f'{v:.10g}' for v in xyz),
                quat=' '.join(f'{v:.10g}' for v in quat),
            )
            extra_m = cfg.added_gripper_mass_kg if child_link == 'gripper_base' else 0.0
            add_link_visual_and_inertial(child_body, child_link, extra_mass=extra_m)

            if j_type in ('revolute', 'prismatic'):
                axis_el = j_el.find('axis')
                lim_el = j_el.find('limit')
                axis = _parse_floats(axis_el.attrib.get('xyz') if axis_el is not None else None, (0.0, 0.0, 1.0))
                lo = float(lim_el.attrib['lower']) if lim_el is not None else -3.14
                hi = float(lim_el.attrib['upper']) if lim_el is not None else 3.14
                mj_type = 'hinge' if j_type == 'revolute' else 'slide'
                j_attrs: dict[str, str] = {
                    'name': j_name,
                    'type': mj_type,
                    'axis': ' '.join(f'{v:.8g}' for v in axis),
                    'limited': 'true',
                    'range': f'{lo:.8g} {hi:.8g}',
                }
                if j_name.startswith('joint') and j_name[5:].isdigit():
                    j_idx = int(j_name[5:]) - 1
                    if 0 <= j_idx < 6:
                        j_attrs['damping'] = f'{cfg.joint_damping[j_idx]:.6g}'
                        j_attrs['frictionloss'] = f'{cfg.joint_frictionloss[j_idx]:.6g}'
                    else:
                        j_attrs['damping'] = '20.0'
                ET.SubElement(child_body, 'joint', **j_attrs)

            if child_link == 'wrench_sensor':
                # PX6D-75A measurement reference site inside wrench_sensor body.
                # wrench_sensor origin is at link6 z = +7.5 mm; sensor center is +12.0 mm further (+19.5 mm total).
                ET.SubElement(
                    child_body, 'site', name='paxini_ft_site',
                    pos=f'0 0 {PAXINI_SENSOR_OFFSET_M[2] - 0.0075:.8g}',
                    size='0.004', rgba='0.1 0.85 0.95 0.9',
                )

            if child_link == 'gripper_base':
                # gripper_base in piper_description_with_wrench.urdf is at link6 z = +0.0403 m, Rz(+90 deg):
                #   X_gb = +Y_link6 = -Y_gripper, Y_gb = -X_link6 = +X_gripper, Z_gb = +Z_link6 = +Z_gripper
                # Tool frame T has: X_T = +X_gripper = +Y_gb, Y_T = -Y_gripper = -X_gb, Z_T = -Z_gripper = -Z_gb
                # => R_{gb, T} = [[0, -1, 0], [1, 0, 0], [0, 0, -1]] -> quat_wxyz = [0, 1/sqrt(2), 1/sqrt(2), 0].
                tip_pos_gb = (
                    -offset_gripper[1],
                    offset_gripper[0],
                    offset_gripper[2] - 0.0403,
                )
                s2 = math.sqrt(0.5)
                plug_body = ET.SubElement(
                    child_body, 'body', name='grasped_plug',
                    pos=' '.join(f'{v:.8g}' for v in tip_pos_gb),
                    quat=f'0 {s2:.10g} {s2:.10g} 0',
                )
                ET.SubElement(
                    plug_body, 'inertial', pos=f'0 0 {0.5 * cfg.peg_length_m:.8g}',
                    mass=f'{cfg.plug_mass_kg:.8g}', diaginertia='1e-5 1e-5 3e-6',
                )
                ET.SubElement(
                    plug_body, 'site', name='plug_tip_site', pos='0 0 0',
                    size='0.0025', rgba='1.0 0.25 0.25 1.0',
                )
                # Visual stem connecting gripper fingers to the calibrated plug base
                stem_top = max(cfg.peg_length_m + 0.002, offset_gripper[2] - 0.145)
                if stem_top > cfg.peg_length_m:
                    ET.SubElement(
                        plug_body, 'geom', name='plug_holder_vis', type='cylinder',
                        pos=f'0 0 {0.5 * (cfg.peg_length_m + stem_top):.8g}',
                        size=f'{0.55 * r:.8g} {0.5 * (stem_top - cfg.peg_length_m):.8g}',
                        rgba='0.35 0.38 0.44 1', contype='0', conaffinity='0', group='1',
                    )
                # Collision + visual peg geometry in Tool frame T: tip at z=0, extending along +Z_T (up into fingers)
                if cfg.shape == 'square':
                    a = r / math.sqrt(2.0) - 0.5 * cfg.clearance_m
                    _add_box_geom(
                        plug_body, 'peg_col', (-a, -a), (a, a), 0.0, cfg.peg_length_m,
                        '0.90 0.65 0.12 1', yaw=0.25 * math.pi, contype=2, conaffinity=1,
                    )
                elif cfg.shape == 'l':
                    c = 0.5 * cfg.clearance_m
                    _add_box_geom(
                        plug_body, 'peg_bottom', (-r + c, -r + c), (r - c, -c),
                        0.0, cfg.peg_length_m, '0.90 0.65 0.12 1', contype=2, conaffinity=1,
                    )
                    _add_box_geom(
                        plug_body, 'peg_left', (-r + c, -c), (-c, r - c),
                        0.0, cfg.peg_length_m, '0.90 0.65 0.12 1', contype=2, conaffinity=1,
                    )
                elif cfg.shape == 'circle':
                    ET.SubElement(
                        plug_body, 'geom', name='peg_col', type='cylinder',
                        pos=f'0 0 {0.5 * cfg.peg_length_m:.8g}',
                        size=f'{r - 0.5 * cfg.clearance_m:.8g} {0.5 * cfg.peg_length_m:.8g}',
                        rgba='0.90 0.65 0.12 1', contype='2', conaffinity='1',
                    )
                else:
                    _add_prism_geom(
                        plug_body, asset, 'peg_mesh', peg_poly, 0.0, cfg.peg_length_m,
                        '0.90 0.65 0.12 1', contype=2, conaffinity=1,
                    )

            build_subtree(child_body, child_link)

    build_subtree(base_body, 'base_link')

    # Hole fixture placed at hole_pos_b, hole_quat_bh in base_link frame (+Z_H outward, insertion along -Z_H)
    p_bh = np.asarray(hole_pos_b, dtype=np.float64).reshape(3)
    q_bh = canonical_quat(hole_quat_bh)
    hole_body = ET.SubElement(
        world, 'body', name='hole_fixture', mocap='true',
        pos=' '.join(f'{v:.8g}' for v in p_bh),
        quat=' '.join(f'{v:.8g}' for v in q_bh),
    )
    ET.SubElement(hole_body, 'site', name='hole_entrance_site', pos='0 0 0', size='0.0025', rgba='0.2 0.95 0.35 1')
    outer = max(cfg.plate_outer_radius_m, 2.2 * r)
    if cfg.shape == 'square':
        a = r / math.sqrt(2.0)
        b = outer
        for idx, (lo_xy, hi_xy) in enumerate((([-b, -b], [-a, b]), ([a, -b], [b, b]), ([-a, -b], [a, -a]), ([-a, a], [a, b]))):
            _add_box_geom(
                hole_body, f'wall_{idx}', lo_xy, hi_xy, -cfg.hole_depth_m, 0.0,
                '0.45 0.50 0.62 0.85', yaw=0.25 * math.pi, contype=1, conaffinity=2,
            )
    elif cfg.shape == 'l':
        _add_box_geom(hole_body, 'wall_left', (-outer, -outer), (-r, outer), -cfg.hole_depth_m, 0.0, '0.45 0.50 0.62 0.85')
        _add_box_geom(hole_body, 'wall_right', (r, -outer), (outer, outer), -cfg.hole_depth_m, 0.0, '0.45 0.50 0.62 0.85')
        _add_box_geom(hole_body, 'wall_bottom', (-r, -outer), (r, -r), -cfg.hole_depth_m, 0.0, '0.45 0.50 0.62 0.85')
        _add_box_geom(hole_body, 'wall_top', (-r, r), (r, outer), -cfg.hole_depth_m, 0.0, '0.45 0.50 0.62 0.85')
        _add_box_geom(hole_body, 'wall_notch', (0.0, 0.0), (r, r), -cfg.hole_depth_m, 0.0, '0.45 0.50 0.62 0.85')
    else:
        edges = np.roll(hole_poly, -1, axis=0) - hole_poly
        inradius = float(np.min(np.abs(edges[:, 0] * hole_poly[:, 1] - edges[:, 1] * hole_poly[:, 0]) / np.linalg.norm(edges, axis=1)))
        out_poly = hole_poly * (outer / inradius)
        for i in range(len(hole_poly)):
            j = (i + 1) % len(hole_poly)
            quad = np.array([hole_poly[i], out_poly[i], out_poly[j], hole_poly[j]], dtype=np.float64)
            _add_prism_geom(
                hole_body, asset, f'wall_{i}', quad, -cfg.hole_depth_m, 0.0,
                '0.45 0.50 0.62 0.85', contype=1, conaffinity=2,
            )
    # Hole bottom stop plate below -hole_depth_m
    _add_box_geom(
        hole_body, 'hole_bottom', (-outer, -outer), (outer, outer),
        -cfg.hole_depth_m - 0.008, -cfg.hole_depth_m, '0.32 0.36 0.44 0.9', contype=1, conaffinity=2,
    )

    actuator = ET.SubElement(mj_root, 'actuator')
    for idx in range(1, 7):
        ET.SubElement(
            actuator, 'motor', name=f'torq_j{idx}', joint=f'joint{idx}',
            ctrlrange='-100 100', ctrllimited='true',
        )
    ET.SubElement(actuator, 'position', name='pos_j7', joint='joint7', kp='400', dampratio='1', ctrlrange='0 0.035', ctrllimited='true')
    ET.SubElement(actuator, 'position', name='pos_j8', joint='joint8', kp='400', dampratio='1', ctrlrange='-0.035 0', ctrllimited='true')

    sensor = ET.SubElement(mj_root, 'sensor')
    ET.SubElement(sensor, 'force', name='paxini_force', site='paxini_ft_site')
    ET.SubElement(sensor, 'torque', name='paxini_torque', site='paxini_ft_site')

    return ET.tostring(mj_root, encoding='unicode')


class PiperWrenchInsertSim:
    """Closed-loop MuJoCo simulation of PiPER-X + PX6D-75A + MIT driver + InsertAnything policy."""

    def __init__(
        self,
        *,
        scene_cfg: SimSceneConfig | None = None,
        arm_id: str = 'arm_4',
        config_file: str | Path | None = None,
        onnx_path: str | Path | None = None,
        q_inserted: Sequence[float] = DEFAULT_Q_INSERTED,
        target_depth_m: float = 0.015,
        use_driver_mit: bool = True,
        kp_override: Sequence[float] | None = None,
        kd_override: Sequence[float] | None = None,
        s_a_override: float | None = None,
        gravity_compensation_mode: str = 'analytic',
        arm_gravity_mode: str = 'matched',
        impedance_mode: str = 'cartesian_tip',
        cartesian_kp: Sequence[float] = CARTESIAN_TIP_KP,
        inner_control_hz: int = 200,
        can_delay_substeps: int = 0,
    ) -> None:
        self.fleet = FleetConfig(Path(config_file) if config_file else default_config_path())
        self.arm_cfg: ArmConfig = self.fleet.arm(arm_id)
        self.gravity_model: ArmGravityModel | None = self.fleet.gravity_model(self.arm_cfg)
        self.friction_model: ArmFrictionModel | None = self.fleet.friction_model(self.arm_cfg)
        self.scene_cfg = scene_cfg or SimSceneConfig()
        if scene_cfg is None and self.gravity_model is not None:
            self.scene_cfg.added_gripper_mass_kg = float(self.gravity_model.added_mass_kg)

        self.target_depth_m = float(target_depth_m)
        self.use_driver_mit = bool(use_driver_mit)
        self.gravity_compensation_mode = gravity_compensation_mode  # 'analytic' (paxini_driver) or 'tare_only' (web.py start_policy)
        self.arm_gravity_mode = arm_gravity_mode  # 'matched' (true MJ gravity) or 'driver_yaml' (arms.yaml gravity_model)
        self.impedance_mode = impedance_mode  # 'cartesian_tip' (J_tip^T F_tip) or 'joint_mit' (IK + joint PD)
        self.cartesian_kp = np.asarray(cartesian_kp, dtype=np.float64).reshape(6)
        self.inner_control_hz = max(50, int(inner_control_hz))
        self.can_delay_substeps = max(0, int(can_delay_substeps))
        self.base_kp = np.asarray(kp_override if kp_override is not None else self.arm_cfg.kp, dtype=np.float64)
        self.base_kd = np.asarray(kd_override if kd_override is not None else self.arm_cfg.kd, dtype=np.float64)
        self.s_a = float(self.arm_cfg.s_a if s_a_override is None else s_a_override)

        # Compute hole entrance frame H so that q_inserted places the plug tip at [0, 0, -target_depth_m] in H
        self.q_inserted = tuple(float(v) for v in q_inserted[:6])
        p_bt_ins, q_bt_ins = forward_tip_pose(
            self.q_inserted, tip_offset_m=self.scene_cfg.tip_offset_m, gravity_model=self.gravity_model,
        )
        self.q_bh = canonical_quat(q_bt_ins)
        # In H frame, p_inserted_H = [0, 0, -target_depth_m] => p_BH = p_BT_ins - R_BH @ [0, 0, -target_depth_m]
        self.p_bh = p_bt_ins - rotate(self.q_bh, [0.0, 0.0, -self.target_depth_m])

        xml = build_piper_wrench_mjcf(self.p_bh, self.q_bh, self.scene_cfg)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)

        self.ft_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'paxini_ft_site')
        self.tip_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'plug_tip_site')
        self.hole_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'hole_entrance_site')
        self.sensor_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'wrench_sensor')
        self._grav_data = mujoco.MjData(self.model)

        # Compute exact downstream mass & COM relative to paxini_ft_site for analytic gravity compensation
        self.downstream_mass_kg, self.downstream_com_s = self._compute_sensor_downstream_mass_com()

        self.policy = InsertAnythingPolicy(onnx_path=onnx_path)
        self.rule_policy = RuleBasedInsertPolicy()
        self.paxini_filter = WrenchFilter(sensor_offset_m=PAXINI_SENSOR_OFFSET_M)
        self.estimated_filter = WrenchFilter()

        self.cmd_q = np.array(self.q_inserted, dtype=np.float64)
        self.cmd_v = np.zeros(6, dtype=np.float64)
        self.cmd_p_b: np.ndarray | None = None
        self.cmd_q_bt: np.ndarray | None = None
        self._latched_cart_tau = np.zeros(6, dtype=np.float64)
        self._latched_foc_kp = np.zeros(6, dtype=np.float64)
        self._latched_foc_pos = np.array(self.q_inserted, dtype=np.float64)
        _, self._latched_joint_kd = resolve_cartesian_damping(
            self.q_inserted, self.cartesian_kp,
            tip_offset_m=self.scene_cfg.tip_offset_m, gravity_model=self.gravity_model,
        )
        self._fb_delay_buf: deque[tuple[np.ndarray, np.ndarray]] = deque(
            [(np.array(self.q_inserted, dtype=np.float64), np.zeros(6, dtype=np.float64))] * (self.can_delay_substeps + 1),
            maxlen=self.can_delay_substeps + 1,
        )
        self.applied_kp = self.base_kp.copy()
        self.applied_kd = self.base_kd.copy()
        self.decimation = max(1, round(POLICY_PERIOD_S / self.scene_cfg.physics_dt))

        # GoalPose Options:
        # 1) Entrance goal (p_bh at z_H=0, goal_pos_h=[0,0,-target_depth_m]) -> matches InsertAnything training
        # 2) Inserted-bottom goal (p_bh at recorded bottom, goal_pos_h=[0,0,0]) -> matches current web.py record_goal!
        self.goal_at_entrance = GoalPose(
            joints=self.q_inserted,
            p_bh=(float(self.p_bh[0]), float(self.p_bh[1]), float(self.p_bh[2])),
            q_bh=tuple(float(v) for v in self.q_bh),
            tip_length_m=float(self.scene_cfg.tip_offset_m[2]),
            recorded_at=0.0,
            tip_offset_m=self.scene_cfg.tip_offset_m,
        )
        self.goal_at_bottom = GoalPose(
            joints=self.q_inserted,
            p_bh=(float(p_bt_ins[0]), float(p_bt_ins[1]), float(p_bt_ins[2])),
            q_bh=tuple(float(v) for v in self.q_bh),
            tip_length_m=float(self.scene_cfg.tip_offset_m[2]),
            recorded_at=0.0,
            tip_offset_m=self.scene_cfg.tip_offset_m,
        )

    def _compute_sensor_downstream_mass_com(self) -> tuple[float, np.ndarray]:
        """Compute exact mass and COM in wrench_sensor frame of all bodies at/below wrench_sensor."""
        self.set_sdk_joints(self.q_inserted)
        descendants: set[int] = {self.sensor_body_id}
        for b_id in range(self.model.nbody):
            p = int(self.model.body_parentid[b_id])
            if p in descendants and b_id != 0:
                descendants.add(b_id)
        r_bs = self.data.site_xmat[self.ft_site_id].reshape(3, 3)
        p_bs = self.data.site_xpos[self.ft_site_id]
        total_m = 0.0
        weighted_pos_b = np.zeros(3, dtype=np.float64)
        for b_id in descendants:
            m = float(self.model.body_mass[b_id])
            total_m += m
            weighted_pos_b += m * self.data.xipos[b_id]
        com_b = weighted_pos_b / max(total_m, 1e-12)
        com_s = r_bs.T @ (com_b - p_bs)
        return total_m, com_s

    def sdk_joints(self) -> np.ndarray:
        """Read 6 joint angles in PiPER-X SDK/driver convention."""
        q = self.data.qpos[:6].copy()
        # In MuJoCo, qpos has ref=pi/4 on joint6, meaning qpos=0 is URDF 0 = SDK +pi/4?
        # Wait: in MuJoCo, joint angle = qpos - qpos0_ref; let's verify via FK in tests and keep exact mapping:
        return q

    def set_sdk_joints(self, q_sdk: Sequence[float], gripper_width_m: float = 0.0) -> None:
        for i in range(6):
            self.data.qpos[i] = float(q_sdk[i])
        self.data.qpos[6] = 0.5 * float(gripper_width_m)
        self.data.qpos[7] = -0.5 * float(gripper_width_m)
        self.data.qvel[:] = 0.0
        self.data.ctrl[6] = 0.5 * float(gripper_width_m)
        self.data.ctrl[7] = -0.5 * float(gripper_width_m)
        mujoco.mj_forward(self.model, self.data)

    def true_tip_pose_in_base(self) -> tuple[np.ndarray, np.ndarray]:
        p_bt = self.data.site_xpos[self.tip_site_id].copy()
        r_bt = self.data.site_xmat[self.tip_site_id].reshape(3, 3).copy()
        return p_bt, mat_to_quat(r_bt)

    def true_tip_pose_in_hole(self) -> tuple[np.ndarray, np.ndarray]:
        p_bt, q_bt = self.true_tip_pose_in_base()
        return relative_pose_in_hole(p_bt, q_bt, self.p_bh, self.q_bh)

    def raw_paxini_wrench_sensor_frame(self) -> np.ndarray:
        """Return raw PX6D-75A 6-axis reading in sensor/link6 frame (external load + downstream gravity)."""
        # MuJoCo force/torque sensor reports parent-on-child internal reaction = -F_external + m*g_upward.
        # Flipping sign yields F_external + F_gravity_downward, matching physical PX6D-75A raw strain reading!
        return -self.data.sensordata[:6].copy()

    def compensated_paxini_wrench_sensor_frame(self) -> np.ndarray:
        """Compensate downstream tool gravity dynamically using current FK orientation (like paxini_driver)."""
        raw = self.raw_paxini_wrench_sensor_frame()
        if self.gravity_compensation_mode != 'analytic':
            return raw
        q_meas = self.sdk_joints()
        t_l6 = (
            self.gravity_model.forward_kinematics(q_meas)
            if self.gravity_model is not None
            else None
        )
        if t_l6 is not None:
            r_bs = t_l6[:3, :3]
        else:
            from piper_x_driver.gravity import link6_rotation_in_base
            r_bs = link6_rotation_in_base(q_meas)
        # Gravity vector in base is [0, 0, gravity_z]
        f_grav_s = r_bs.T @ np.array([0.0, 0.0, self.downstream_mass_kg * self.scene_cfg.gravity_z])
        m_grav_s = np.cross(self.downstream_com_s, f_grav_s)
        return raw - np.concatenate([f_grav_s, m_grav_s])

    def ingest_sensor_substep(self) -> None:
        comp_s = self.compensated_paxini_wrench_sensor_frame()
        # web.py _on_paxini_wrench converts link6/sensor frame (+x=-gripper_x, +y=-gripper_y, +z=+gripper_z)
        gripper_wrench = (
            -float(comp_s[0]), -float(comp_s[1]), float(comp_s[2]),
            -float(comp_s[3]), -float(comp_s[4]), float(comp_s[5]),
        )
        self.paxini_filter.update(gripper_wrench, dt=self.scene_cfg.physics_dt, already_at_tip=False)

    def compute_joint_torques(self, *, update_inner_loop: bool = True) -> np.ndarray:
        """Compute 6-joint actuator torques via either joint-space MIT PD or Cartesian tip impedance."""
        q_true = self.sdk_joints()
        dq_true = self.data.qvel[:6].copy()
        if self.can_delay_substeps > 0:
            self._fb_delay_buf.append((q_true.copy(), dq_true.copy()))
            q_meas, dq_meas = self._fb_delay_buf[0]
        else:
            q_meas, dq_meas = q_true, dq_true

        if self.arm_gravity_mode == 'matched':
            self._grav_data.qpos[:] = self.data.qpos[:]
            self._grav_data.qvel[:] = 0.0
            mujoco.mj_forward(self.model, self._grav_data)
            raw_g = self._grav_data.qfrc_bias[:6].copy()
        elif self.gravity_model is not None:
            raw_g = np.asarray(self.gravity_model.torques(q_meas), dtype=np.float64)
        else:
            from piper_x_driver.gravity import gravity_torques
            raw_g = np.asarray(gravity_torques(q_meas), dtype=np.float64)
        g_vec = float(self.arm_cfg.gravity_scale) * raw_g

        if self.impedance_mode == 'cartesian_tip' and self.cmd_p_b is not None and self.cmd_q_bt is not None:
            if update_inner_loop:
                offset_gripper = normalize_tip_offset(self.scene_cfg.tip_offset_m)
                offset_link6 = (-offset_gripper[0], -offset_gripper[1], offset_gripper[2])
                p_bt, q_bt = forward_tip_pose(
                    q_meas, tip_offset_m=offset_gripper, gravity_model=self.gravity_model,
                )
                j_b = (
                    geometric_jacobian(q_meas, tool_offset_m=offset_link6, frame='base_link')
                    if self.gravity_model is None
                    else self.gravity_model.jacobian(q_meas, tool_offset_m=offset_link6, frame='base_link')
                )
                e_p = self.cmd_p_b - p_bt
                e_r = quat_rotvec(quat_mul(self.cmd_q_bt, quat_inv(q_bt)))
                task_kd, self._latched_joint_kd = resolve_cartesian_damping(
                    q_meas, self.cartesian_kp,
                    tip_offset_m=offset_gripper, gravity_model=self.gravity_model,
                )
                wrench_b = self.cartesian_kp * np.concatenate([e_p, e_r]) - task_kd * (j_b @ dq_meas)
                tau_cart = j_b.T @ wrench_b
                if np.any(self.cartesian_kp > 0.0):
                    kp_foc = np.asarray(CARTESIAN_CARRIER_LSB_MULT, dtype=np.float64) * np.asarray(MIT_KP_LSB, dtype=np.float64)
                    dq_fold = np.clip(tau_cart / kp_foc, -0.8, 0.8)
                else:
                    kp_foc = np.zeros(6, dtype=np.float64)
                    dq_fold = np.zeros(6, dtype=np.float64)
                self._latched_foc_kp = kp_foc
                self._latched_foc_pos = q_meas + dq_fold
                self._latched_cart_tau = tau_cart - kp_foc * dq_fold
            # Motor hardware FOC executes kp_foc & joint_kd at ~20 kHz against true instantaneous motor state
            return (
                g_vec
                + self._latched_foc_kp * (self._latched_foc_pos - q_true)
                + self._latched_cart_tau
                - self._latched_joint_kd * dq_true
            )

        pos_err = self.cmd_q - q_meas
        kp_eff = self.base_kp.copy()
        kd_eff = self.base_kd.copy()
        if self.use_driver_mit and self.s_a > 0.0:
            for i in range(6):
                if kp_eff[i] > 0.0 and kd_eff[i] > 0.0:
                    ratio = abs(float(pos_err[i])) / float(self.arm_cfg.adaptive_stiffness_b)
                    mult = 1.0 + self.s_a / ((ratio ** float(self.arm_cfg.adaptive_stiffness_power)) + 1.0)
                    kp_new = min(500.0, kp_eff[i] * mult)
                    kd_new = min(100.0, kd_eff[i] * math.sqrt(kp_new / kp_eff[i]))
                    kp_eff[i] = kp_new
                    kd_eff[i] = kd_new

        self.applied_kp = kp_eff
        self.applied_kd = kd_eff

        tau = np.zeros(6, dtype=np.float64)
        for i in range(6):
            active = (kd_eff[i] >= MIT_KD_LSB[i]) and (round(kp_eff[i] / MIT_KP_LSB[i]) > 0)
            q_des = float(self.cmd_q[i])
            if active:
                lo = max(-MAX_GRAVITY_OFFSET, -MIT_POSITION_LIMIT - q_des)
                hi = min(MAX_GRAVITY_OFFSET, MIT_POSITION_LIMIT - q_des)
                raw_off = g_vec[i] / kp_eff[i]
                off = max(lo, min(hi, raw_off))
                res = 0.0 if off == raw_off else (g_vec[i] - off * kp_eff[i])
                p_wire = q_des + off
                tau[i] = kp_eff[i] * (p_wire - q_meas[i]) + kd_eff[i] * (self.cmd_v[i] - dq_meas[i]) + res
            else:
                tau[i] = g_vec[i]
        return tau

    def reset_episode(
        self,
        *,
        initial_offset_h: Sequence[float] = (0.005, -0.004, 0.015),
        initial_rotvec_h: Sequence[float] = (0.0, 0.0, 0.0),
        tare_at_start: bool = True,
    ) -> np.ndarray:
        """Place the plug tip at initial_offset_h (relative to hole entrance H) and settle static equilibrium."""
        p_init_h = np.asarray(initial_offset_h, dtype=np.float64).reshape(3)
        q_init_ht = canonical_quat(rotvec_quat(initial_rotvec_h))
        p_start_b, q_start_bt = hole_target_to_base(p_init_h, q_init_ht, self.p_bh, self.q_bh)
        q_start = solve_tip_ik(
            self.q_inserted, p_start_b, q_start_bt,
            tip_offset_m=self.scene_cfg.tip_offset_m, gravity_model=self.gravity_model, max_iters=50,
        )
        self.set_sdk_joints(q_start)
        self.cmd_q = np.array(q_start, dtype=np.float64)
        self.cmd_v.fill(0.0)
        self.cmd_p_b = p_start_b.copy()
        self.cmd_q_bt = q_start_bt.copy()
        self._latched_cart_tau.fill(0.0)
        self._latched_foc_kp.fill(0.0)
        self._latched_foc_pos = np.array(q_start, dtype=np.float64)
        self._fb_delay_buf = deque(
            [(np.array(q_start, dtype=np.float64), np.zeros(6, dtype=np.float64))] * (self.can_delay_substeps + 1),
            maxlen=self.can_delay_substeps + 1,
        )
        self.policy.reset()
        self.rule_policy.reset()
        self.paxini_filter.clear_tare()
        self.estimated_filter.clear_tare()

        inner_div = max(1, round(1.0 / (self.scene_cfg.physics_dt * self.inner_control_hz)))
        # Settle under active controller for 0.15 s so gravity sag & sensor transient stabilize
        for sub in range(150):
            self.data.ctrl[:6] = self.compute_joint_torques(update_inner_loop=(sub % inner_div == 0))
            mujoco.mj_step(self.model, self.data)
            self.ingest_sensor_substep()

        if tare_at_start:
            self.paxini_filter.tare()
            # Re-prime filter for a few substeps after tare
            for sub in range(5):
                self.data.ctrl[:6] = self.compute_joint_torques(update_inner_loop=(sub % inner_div == 0))
                mujoco.mj_step(self.model, self.data)
                self.ingest_sensor_substep()
        return np.array(q_start, dtype=np.float64)

    def step_policy(
        self,
        *,
        controller: str = 'onnx',
        goal_mode: str = 'deployed_bottom',
        accumulate_target: bool = False,
        rotation_scale_factor: float = 1.0,
    ) -> dict[str, Any]:
        """Advance one 50 Hz policy step (20 x 1 ms MuJoCo physics substeps) and return rich diagnostics."""
        q_meas = self.sdk_joints()
        wrench_t = self.paxini_filter.tip_wrench(tip_offset_m=self.scene_cfg.tip_offset_m)

        if goal_mode == 'deployed_bottom':
            # Exact current web.py behavior: GoalPose recorded at bottom of hole, goal_pos_h = (0, 0, 0)
            goal = self.goal_at_bottom
            goal_pos_h = (0.0, 0.0, 0.0)
        elif goal_mode == 'training_entrance':
            # InsertAnything training contract: H origin at entrance, goal_pos_h = (0, 0, -target_depth_m)
            goal = self.goal_at_entrance
            goal_pos_h = (0.0, 0.0, -self.target_depth_m)
        else:
            raise ValueError(f'Unknown goal_mode {goal_mode!r}')

        orig_rot_limit = self.policy.rotation_limit_rad
        try:
            self.policy.rotation_limit_rad = orig_rot_limit * float(rotation_scale_factor)
            if controller == 'rule':
                res: PolicyStepResult = self.rule_policy.step(
                    q_meas, goal, wrench_t,
                    tip_offset_m=self.scene_cfg.tip_offset_m,
                    gravity_model=self.gravity_model,
                    goal_pos_h=goal_pos_h,
                )
            else:
                res = self.policy.step(
                    q_meas, goal, wrench_t,
                    tip_offset_m=self.scene_cfg.tip_offset_m,
                    gravity_model=self.gravity_model,
                    goal_pos_h=goal_pos_h,
                    accumulate_target=accumulate_target,
                    solve_ik=(self.impedance_mode == 'joint_mit'),
                )
        finally:
            self.policy.rotation_limit_rad = orig_rot_limit

        self.cmd_q = np.asarray(res.q_cmd, dtype=np.float64)
        self.cmd_v = np.asarray(res.v_cmd, dtype=np.float64)
        self.cmd_p_b = res.p_target_b.copy()
        self.cmd_q_bt = res.q_target_bt.copy()

        inner_div = max(1, round(1.0 / (self.scene_cfg.physics_dt * self.inner_control_hz)))
        peak_force_n = 0.0
        peak_moment_nm = 0.0
        for sub in range(self.decimation):
            self.data.ctrl[:6] = self.compute_joint_torques(update_inner_loop=(sub % inner_div == 0))
            mujoco.mj_step(self.model, self.data)
            self.ingest_sensor_substep()
            wt_sub = self.paxini_filter.tip_wrench(tip_offset_m=self.scene_cfg.tip_offset_m)
            peak_force_n = max(peak_force_n, float(np.linalg.norm(wt_sub[:3])))
            peak_moment_nm = max(peak_moment_nm, float(np.linalg.norm(wt_sub[3:6])))

        true_p_ht, true_q_ht = self.true_tip_pose_in_hole()
        z_axis_h = rotate(true_q_ht, [0.0, 0.0, 1.0])
        tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(z_axis_h[2])))))
        rotvec_deg = np.degrees(quat_rotvec(true_q_ht))

        return {
            'step': res.step,
            'true_p_ht_mm': (true_p_ht * 1000.0).tolist(),
            'fk_p_ht_mm': (res.p_ht * 1000.0).tolist(),
            'xy_err_mm': float(np.linalg.norm(true_p_ht[:2]) * 1000.0),
            'depth_mm': float(-true_p_ht[2] * 1000.0),
            'tilt_deg': tilt_deg,
            'rotvec_deg': rotvec_deg.tolist(),
            'obs': res.obs.tolist(),
            'actions': res.actions.tolist(),
            'act_dp_mm': (res.actions[:3] * self.policy.translation_limit_m * 1000.0).tolist(),
            'act_dr_deg': np.degrees(res.actions[3:6] * self.policy.rotation_limit_rad * float(rotation_scale_factor)).tolist(),
            'wrench_t': res.wrench_t.tolist(),
            'force_norm_n': float(np.linalg.norm(res.wrench_t[:3])),
            'moment_norm_nm': float(np.linalg.norm(res.wrench_t[3:6])),
            'peak_force_n': peak_force_n,
            'peak_moment_nm': peak_moment_nm,
            'q_meas_deg': np.degrees(self.sdk_joints()).tolist(),
            'q_cmd_deg': np.degrees(self.cmd_q).tolist(),
            'ncon': int(self.data.ncon),
            'success': bool(true_p_ht[2] <= -self.target_depth_m + 0.0008 and np.linalg.norm(true_p_ht[:2]) <= 0.0025),
        }


def run_episode(
    sim: PiperWrenchInsertSim,
    *,
    steps: int = 250,
    controller: str = 'onnx',
    goal_mode: str = 'deployed_bottom',
    accumulate_target: bool = False,
    rotation_scale_factor: float = 1.0,
    initial_offset_h: Sequence[float] = (0.005, -0.004, 0.015),
    initial_rotvec_h: Sequence[float] = (0.0, 0.0, 0.0),
    tare_at_start: bool = True,
) -> dict[str, Any]:
    sim.reset_episode(
        initial_offset_h=initial_offset_h,
        initial_rotvec_h=initial_rotvec_h,
        tare_at_start=tare_at_start,
    )
    trace: list[dict[str, Any]] = []
    success_hold = 0
    succeeded_step: int | None = None
    for _ in range(steps):
        info = sim.step_policy(
            controller=controller,
            goal_mode=goal_mode,
            accumulate_target=accumulate_target,
            rotation_scale_factor=rotation_scale_factor,
        )
        trace.append(info)
        success_hold = success_hold + 1 if info['success'] else 0
        if success_hold >= 5 and succeeded_step is None:
            succeeded_step = info['step']
            break

    tilts = [row['tilt_deg'] for row in trace]
    depths = [row['depth_mm'] for row in trace]
    forces = [row['peak_force_n'] for row in trace]
    moments = [row['peak_moment_nm'] for row in trace]
    return {
        'controller': controller,
        'impedance_mode': sim.impedance_mode,
        'goal_mode': goal_mode,
        'gravity_compensation_mode': sim.gravity_compensation_mode,
        'accumulate_target': accumulate_target,
        'rotation_scale_factor': rotation_scale_factor,
        'steps_run': len(trace),
        'succeeded': succeeded_step is not None,
        'succeeded_step': succeeded_step,
        'max_tilt_deg': round(max(tilts) if tilts else 0.0, 2),
        'final_tilt_deg': round(tilts[-1] if tilts else 0.0, 2),
        'max_depth_mm': round(max(depths) if depths else 0.0, 2),
        'final_p_ht_mm': [round(v, 2) for v in trace[-1]['true_p_ht_mm']] if trace else [],
        'max_force_n': round(max(forces) if forces else 0.0, 2),
        'max_moment_nm': round(max(moments) if moments else 0.0, 3),
        'trace': trace,
    }


def _serve_viser(sim: PiperWrenchInsertSim, args: argparse.Namespace) -> None:
    import trimesh
    import viser

    server = viser.ViserServer(host=args.host, port=args.port)
    print(f'[sim_mujoco] Viser 3D server listening at http://{args.host}:{args.port}', flush=True)

    geom_handles: dict[int, Any] = {}
    for g_id in range(sim.model.ngeom):
        g_type = int(sim.model.geom_type[g_id])
        if g_type == int(mujoco.mjtGeom.mjGEOM_PLANE):
            continue
        rgba = sim.model.geom_rgba[g_id]
        color = (int(rgba[0] * 255), int(rgba[1] * 255), int(rgba[2] * 255))
        opacity = float(rgba[3])
        name = mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, g_id) or f'geom_{g_id}'
        if g_type == int(mujoco.mjtGeom.mjGEOM_BOX):
            mesh = trimesh.creation.box(extents=2.0 * sim.model.geom_size[g_id])
        elif g_type == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
            r, hh = sim.model.geom_size[g_id, :2]
            mesh = trimesh.creation.cylinder(radius=float(r), height=float(2.0 * hh), sections=36)
        elif g_type == int(mujoco.mjtGeom.mjGEOM_MESH):
            m_id = int(sim.model.geom_dataid[g_id])
            v_adr, v_num = int(sim.model.mesh_vertadr[m_id]), int(sim.model.mesh_vertnum[m_id])
            f_adr, f_num = int(sim.model.mesh_faceadr[m_id]), int(sim.model.mesh_facenum[m_id])
            mesh = trimesh.Trimesh(
                vertices=sim.model.mesh_vert[v_adr:v_adr + v_num].copy(),
                faces=sim.model.mesh_face[f_adr:f_adr + f_num].copy(),
                process=False,
            )
        else:
            continue
        geom_handles[g_id] = server.scene.add_mesh_simple(
            f'/piper_sim/{g_id}_{name}', mesh.vertices, mesh.faces,
            color=color, opacity=opacity, side='double',
        )

    tip_frame = server.scene.add_frame('/piper_sim/plug_tip', axes_length=0.03, axes_radius=0.0012)
    hole_frame = server.scene.add_frame('/piper_sim/hole_entrance', axes_length=0.04, axes_radius=0.0012)
    ft_frame = server.scene.add_frame('/piper_sim/paxini_sensor', axes_length=0.025, axes_radius=0.001)

    def sync_scene() -> None:
        for g_id, handle in geom_handles.items():
            handle.position = tuple(float(v) for v in sim.data.geom_xpos[g_id])
            q = mat_to_quat(sim.data.geom_xmat[g_id].reshape(3, 3))
            handle.wxyz = tuple(float(v) for v in q)
        tip_frame.position = tuple(float(v) for v in sim.data.site_xpos[sim.tip_site_id])
        tip_frame.wxyz = tuple(float(v) for v in mat_to_quat(sim.data.site_xmat[sim.tip_site_id].reshape(3, 3)))
        hole_frame.position = tuple(float(v) for v in sim.data.site_xpos[sim.hole_site_id])
        hole_frame.wxyz = tuple(float(v) for v in mat_to_quat(sim.data.site_xmat[sim.hole_site_id].reshape(3, 3)))
        ft_frame.position = tuple(float(v) for v in sim.data.site_xpos[sim.ft_site_id])
        ft_frame.wxyz = tuple(float(v) for v in mat_to_quat(sim.data.site_xmat[sim.ft_site_id].reshape(3, 3)))

    with server.gui.add_folder('Simulation Control'):
        ctrl_dd = server.gui.add_dropdown('Controller', ('onnx', 'rule'), initial_value=args.controller)
        imp_dd = server.gui.add_dropdown('Low-Level Impedance', ('joint_mit', 'cartesian_tip'), initial_value=args.impedance)
        goal_dd = server.gui.add_dropdown('Goal Mode', ('deployed_bottom', 'training_entrance'), initial_value=args.goal_mode)
        grav_dd = server.gui.add_dropdown('Sensor Gravity Comp', ('analytic', 'tare_only'), initial_value=args.gravity_comp)
        rot_slider = server.gui.add_slider('Rotation Scale Factor', min=0.0, max=1.0, step=0.05, initial_value=args.rotation_scale)
        reset_btn = server.gui.add_button('Reset Episode')
        pause_cb = server.gui.add_checkbox('Paused', initial_value=False)
        status_md = server.gui.add_markdown('Initializing...')

    need_reset = True

    @reset_btn.on_click
    def _on_reset(_event: Any) -> None:
        nonlocal need_reset
        need_reset = True

    try:
        while True:
            if need_reset:
                sim.impedance_mode = str(imp_dd.value)
                sim.gravity_compensation_mode = str(grav_dd.value)
                sim.reset_episode(
                    initial_offset_h=args.initial_offset,
                    initial_rotvec_h=np.radians(args.initial_tilt_deg),
                )
                sync_scene()
                need_reset = False
            if not pause_cb.value:
                t0 = time.monotonic()
                sim.impedance_mode = str(imp_dd.value)
                sim.gravity_compensation_mode = str(grav_dd.value)
                info = sim.step_policy(
                    controller=str(ctrl_dd.value),
                    goal_mode=str(goal_dd.value),
                    accumulate_target=args.accumulate_target,
                    rotation_scale_factor=float(rot_slider.value),
                )
                sync_scene()
                p_mm = info['true_p_ht_mm']
                status_md.content = (
                    f"**Step**: `{info['step']}` | **Tilt**: `{info['tilt_deg']:.2f}°` | **Success**: `{info['success']}`  \n"
                    f"**Tip in Hole (mm)**: `[{p_mm[0]:+.2f}, {p_mm[1]:+.2f}, {p_mm[2]:+.2f}]`  \n"
                    f"**Tip Force / Moment**: `{info['force_norm_n']:.2f} N` / `{info['moment_norm_nm']:.3f} N·m`  \n"
                    f"**Action dR (deg)**: `[{info['act_dr_deg'][0]:+.2f}, {info['act_dr_deg'][1]:+.2f}, {info['act_dr_deg'][2]:+.2f}]`"
                )
                time.sleep(max(0.0, POLICY_PERIOD_S - (time.monotonic() - t0)))
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--shape', choices=SHAPES, default='hexagon')
    parser.add_argument('--clearance', type=float, default=0.0005)
    parser.add_argument('--target-depth', type=float, default=0.015)
    parser.add_argument('--controller', choices=('onnx', 'rule'), default='onnx')
    parser.add_argument('--impedance', choices=('cartesian_tip', 'joint_mit'), default='cartesian_tip')
    parser.add_argument('--inner-hz', type=int, default=200)
    parser.add_argument('--goal-mode', choices=('deployed_bottom', 'training_entrance'), default='deployed_bottom')
    parser.add_argument('--gravity-comp', choices=('analytic', 'tare_only'), default='analytic')
    parser.add_argument('--arm-gravity', choices=('matched', 'driver_yaml'), default='matched')
    parser.add_argument('--accumulate-target', action='store_true')
    parser.add_argument('--rotation-scale', type=float, default=1.0)
    parser.add_argument('--steps', type=int, default=250)
    parser.add_argument('--initial-offset', nargs=3, type=float, default=[0.005, -0.004, 0.008], metavar=('X', 'Y', 'Z'))
    parser.add_argument('--initial-tilt-deg', nargs=3, type=float, default=[2.0, -1.5, 2.5], metavar=('RX', 'RY', 'RZ'))
    parser.add_argument('--onnx', type=Path, default=None)
    parser.add_argument('--snapshot', type=Path, default=None, help='Save side-by-side PNG trajectory snapshots')
    parser.add_argument('--compare', action='store_true', help='Run ablation matrix diagnosing tilt & execution issues')
    parser.add_argument('--viser', action='store_true', help='Launch interactive 3D browser viewer via Viser')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8095)
    args = parser.parse_args(list(argv) if argv is not None else None)

    scene_cfg = SimSceneConfig(shape=args.shape, clearance_m=args.clearance)
    sim = PiperWrenchInsertSim(
        scene_cfg=scene_cfg,
        onnx_path=args.onnx or default_onnx_path(),
        target_depth_m=args.target_depth,
        gravity_compensation_mode=args.gravity_comp,
        arm_gravity_mode=args.arm_gravity,
        impedance_mode=args.impedance,
        inner_control_hz=args.inner_hz,
    )

    if args.viser:
        _serve_viser(sim, args)
        return 0

    if args.compare:
        cases = [
            ('1_joint_mit + tare_only (无姿态重力补偿 + 关节PD)', 'onnx', 'joint_mit', 'deployed_bottom', 'tare_only'),
            ('2_joint_mit + analytic_grav (姿态重力补偿 + 关节PD)', 'onnx', 'joint_mit', 'deployed_bottom', 'analytic'),
            ('3_cartesian_tip_200Hz + tare_only (无姿态重力补偿 + 尖端笛卡尔阻抗)', 'onnx', 'cartesian_tip', 'deployed_bottom', 'tare_only'),
            ('4_cartesian_tip_200Hz + analytic_grav (姿态重力补偿 + 尖端笛卡尔阻抗)', 'onnx', 'cartesian_tip', 'deployed_bottom', 'analytic'),
            ('5_rule_based + joint_mit (规则策略 + 关节PD)', 'rule', 'joint_mit', 'deployed_bottom', 'analytic'),
        ]
        results = []
        for label, ctrl, imp_mode, g_mode, grav_mode in cases:
            case_sim = PiperWrenchInsertSim(
                scene_cfg=scene_cfg,
                onnx_path=args.onnx or default_onnx_path(),
                target_depth_m=args.target_depth,
                gravity_compensation_mode=grav_mode,
                arm_gravity_mode=args.arm_gravity,
                impedance_mode=imp_mode,
                inner_control_hz=args.inner_hz,
            )
            rep = run_episode(
                case_sim,
                steps=args.steps,
                controller=ctrl,
                goal_mode=g_mode,
                accumulate_target=args.accumulate_target,
                rotation_scale_factor=args.rotation_scale,
                initial_offset_h=args.initial_offset,
                initial_rotvec_h=np.radians(args.initial_tilt_deg),
            )
            rep.pop('trace', None)
            rep['case'] = label
            results.append(rep)
        print(json.dumps(results, indent=2, ensure_ascii=False))
        return 0

    rep = run_episode(
        sim,
        steps=args.steps,
        controller=args.controller,
        goal_mode=args.goal_mode,
        accumulate_target=args.accumulate_target,
        rotation_scale_factor=args.rotation_scale,
        initial_offset_h=args.initial_offset,
        initial_rotvec_h=np.radians(args.initial_tilt_deg),
    )
    if args.snapshot is not None:
        from PIL import Image
        cam = mujoco.MjvCamera()
        cam.lookat[:] = sim.p_bh + np.array([0.0, 0.0, 0.03])
        cam.distance = 0.32
        cam.azimuth = 135.0
        cam.elevation = -22.0
        with mujoco.Renderer(sim.model, height=540, width=720) as renderer:
            renderer.update_scene(sim.data, camera=cam)
            args.snapshot.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(renderer.render()).save(args.snapshot)
    summary = {k: v for k, v in rep.items() if k != 'trace'}
    summary['sample_steps'] = [
        {
            'step': r['step'],
            'p_ht_mm': [round(x, 2) for x in r['true_p_ht_mm']],
            'tilt_deg': round(r['tilt_deg'], 2),
            'act_dr_deg': [round(x, 2) for x in r['act_dr_deg']],
            'force_n': round(r['force_norm_n'], 2),
            'moment_nm': round(r['moment_norm_nm'], 3),
        }
        for r in rep['trace']
        if r['step'] in (1, 3, 5, 8, 12, 20, 35, 50, 80, 120, len(rep['trace']))
    ]
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
