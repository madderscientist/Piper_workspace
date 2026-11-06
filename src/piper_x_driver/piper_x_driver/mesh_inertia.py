"""Generate composite terminal properties by integrating uniform-density STL volumes.

Run with --help to update a profile in config/effectors.yaml from the URDF meshes.
"""

import argparse
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def stl_mass_properties(path: Path, scale=(1.0, 1.0, 1.0)):
    """Return volume centroid and COM inertia per kg from signed tetrahedra."""
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else 0
    if len(data) == 84 + 50 * count:
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
        faces = np.frombuffer(data, dtype=dtype, count=count, offset=84)['vertices'].astype(float)
    else:
        vertices = [list(map(float, line.split()[1:]))
                    for line in data.decode('ascii').splitlines()
                    if line.lstrip().startswith('vertex ')]
        faces = np.asarray(vertices, dtype=float).reshape(-1, 3, 3)
    faces *= np.asarray(scale)
    offset = faces.reshape(-1, 3).mean(axis=0)
    faces -= offset
    volume = np.einsum('ij,ij->i', faces[:, 0], np.cross(faces[:, 1], faces[:, 2])) / 6
    total = volume.sum()
    if not np.isfinite(total) or abs(total) < np.finfo(float).eps:
        raise ValueError(f'STL has no finite solid volume: {path}')
    sums = faces.sum(axis=1)
    center = np.einsum('i,ij->j', volume, sums) / (4 * total)
    second = (np.einsum('i,ij,ik->jk', volume / 20, sums, sums)
              + np.einsum('i,iaj,iak->jk', volume / 20, faces, faces)) / total
    inertia = (np.trace(second) * np.eye(3) - second
               - np.dot(center, center) * np.eye(3) + np.outer(center, center))
    return center + offset, inertia


def _pose(element):
    origin = element.find('origin')
    attrs = {} if origin is None else origin.attrib
    xyz = np.array([float(v) for v in attrs.get('xyz', '0 0 0').split()])
    rpy = [float(v) for v in attrs.get('rpy', '0 0 0').split()]
    return Rotation.from_euler('xyz', rpy).as_matrix(), xyz


def terminal_from_meshes(urdf: Path, reference_urdf: Path, payload_link='gripper_base') -> dict:
    root = ET.parse(urdf).getroot()
    reference = ET.parse(reference_urdf).getroot()
    old_j6 = next(j for j in reference.findall('joint') if j.attrib['name'] == 'joint6')
    new_j6 = next(j for j in root.findall('joint') if j.attrib['name'] == 'joint6')
    old_rotation, old_translation = _pose(old_j6)
    new_rotation, new_translation = _pose(new_j6)
    links = {link.attrib['name']: link for link in root.findall('link')}
    children = {}
    for joint in root.findall('joint'):
        children.setdefault(joint.find('parent').attrib['link'], []).append(joint)
    components = {}
    pending = [('link6', old_rotation.T @ new_rotation,
                old_rotation.T @ (new_translation - old_translation))]
    while pending:
        parent, rotation, translation = pending.pop()
        for joint in children.get(parent, []):
            name = joint.find('child').attrib['link']
            joint_rotation, joint_translation = _pose(joint)
            child_rotation = rotation @ joint_rotation
            child_translation = translation + rotation @ joint_translation
            link = links[name]
            visual = link.find('visual')
            mesh = visual.find('geometry/mesh')
            uri = mesh.attrib['filename']
            if uri.startswith('package://'):
                package, relative = uri[len('package://'):].split('/', 1)
                local = urdf.parent.parent / relative
                mesh_path = local if local.is_file() else Path(get_package_share_directory(package)) / relative
            else:
                mesh_path = urdf.parent / uri
            scale = tuple(float(v) for v in mesh.attrib.get('scale', '1 1 1').split())
            center, inertia = stl_mass_properties(mesh_path, scale)
            visual_rotation, visual_translation = _pose(visual)
            frame = child_rotation @ visual_rotation
            center = child_translation + child_rotation @ visual_translation + frame @ center
            components[name] = {
                'mesh': uri,
                'mass_kg': float(link.find('inertial/mass').attrib['value']),
                'com_m': center.tolist(),
                'inertia_per_kg': (frame @ inertia @ frame.T).tolist(),
            }
            pending.append((name, child_rotation, child_translation))
    if payload_link not in components:
        raise ValueError(f'Payload link is absent from terminal: {payload_link}')
    return {'version': 1, 'model': urdf.name, 'reference_frame': 'original link6',
            'density_assumption': 'uniform within each mesh; URDF component masses retained',
            'payload_link': payload_link, 'components': components}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', type=Path)
    parser.add_argument('--reference-urdf', type=Path, required=True)
    parser.add_argument('--payload-link', default='gripper_base')
    parser.add_argument('--effector', default='gripper_with_wrench', help='Profile to update in output YAML')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = terminal_from_meshes(args.urdf, args.reference_urdf, args.payload_link)
    config = yaml.safe_load(args.output.read_text(encoding='utf-8'))
    config['effectors'][args.effector].update({
        key: value for key, value in data.items() if key not in ('version', 'reference_frame')})
    args.output.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding='utf-8')


if __name__ == '__main__':
    main()
