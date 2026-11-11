"""采集数据的落盘布局，以及 calibration.yaml 和 board.yaml 的读写。"""

from __future__ import annotations

import datetime
import json
import os
import shutil
import threading
from pathlib import Path

import cv2
import numpy as np
import yaml

from camera_calibration.board import Detection

_PNG = [cv2.IMWRITE_PNG_COMPRESSION, 0]
_SECTIONS = ('intrinsics', 'extrinsics', 'profile_relations', 'urdf_overrides')
VERSION = 1


def profile_key(width: int, height: int) -> str:
    return f'{int(width)}x{int(height)}'


def _stamp() -> str:
    return datetime.datetime.now().isoformat(timespec='seconds')


def save_board_config(path: str | Path, board_spec: dict, full_config: dict | None = None) -> str:
    """原子更新 board.yaml 中的 board 规格（自动跟随 symlink）。"""
    target = Path(path).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    data = dict(full_config or {})
    if not data and target.is_file():
        data = yaml.safe_load(target.read_text(encoding='utf-8')) or {}
    data['board'] = {
        'squares_x': int(board_spec['squares_x']),
        'squares_y': int(board_spec['squares_y']),
        'square_size': float(board_spec['square_size']),
        'marker_size': float(board_spec['marker_size']),
        'dictionary': str(board_spec['dictionary']),
    }
    data.setdefault('probe', {
        'dictionaries': ['DICT_5X5_50', 'DICT_5X5_100', 'DICT_5X5_250', 'DICT_5X5_1000',
                         'DICT_4X4_50', 'DICT_4X4_100', 'DICT_4X4_250',
                         'DICT_6X6_50', 'DICT_6X6_100', 'DICT_6X6_250',
                         'DICT_7X7_50', 'DICT_7X7_100', 'DICT_7X7_250'],
        'orientations': [[9, 12], [12, 9]],
    })
    text = yaml.safe_dump(data, allow_unicode=True, sort_keys=False, width=120)
    temporary = target.with_suffix('.yaml.tmp')
    temporary.write_text(text, encoding='utf-8')
    os.replace(temporary, target)
    return str(target)


class Store:
    """一次标定会话的全部产物。root 下面自解释，可以直接拷走离线重跑。"""

    def __init__(self, root, calib_file) -> None:
        self.root = Path(root).expanduser()
        self.calib_file = Path(calib_file).expanduser().resolve()
        self._calib_cache: tuple[int | None, dict] = (None, {})
        self._shots_cache: dict[tuple[str, int, int], tuple[tuple[str, ...], list[dict]]] = {}
        self._poses_cache: dict[str | None, tuple[tuple[str, ...], list[dict]]] = {}
        self._pending_images: dict[str, np.ndarray] = {}
        self._pending_lock = threading.Lock()
        (self.root / 'intrinsic').mkdir(parents=True, exist_ok=True)
        (self.root / 'extrinsic').mkdir(parents=True, exist_ok=True)

    # ---------- 内参 ----------

    def intrinsic_dir(self, camera: str, width: int, height: int) -> Path:
        path = self.root / 'intrinsic' / _safe(camera) / profile_key(width, height)
        path.mkdir(parents=True, exist_ok=True)
        return path

    def save_intrinsic_shot(self, camera: str, image, detection: Detection) -> str:
        width, height = detection.size
        folder = self.intrinsic_dir(camera, width, height)
        name = f'{_next_index(folder.glob("*.json")):04d}'
        (folder / f'{name}.json').write_text(json.dumps({
            'name': name, 'camera': camera, 'stamp': _stamp(),
            'detection': detection.to_json(),
        }), encoding='utf-8')
        png_path = folder / f'{name}.png'
        self._save_image_async(png_path, image)
        self._shots_cache.pop((camera, width, height), None)
        return name

    def _read_shots(self, camera: str, width: int, height: int):
        for path in sorted(self.intrinsic_dir(camera, width, height).glob('*.json')):
            data = json.loads(path.read_text(encoding='utf-8'))
            yield data, Detection.from_json(data['detection'])

    def list_intrinsic_shots(self, camera: str, width: int, height: int) -> list[dict]:
        folder = self.intrinsic_dir(camera, width, height)
        sig = tuple(sorted(p.name for p in folder.glob('*.json')))
        key = (camera, width, height)
        cached = self._shots_cache.get(key)
        if cached is not None and cached[0] == sig:
            return list(cached[1])
        items = [{'name': data['name'], 'stamp': data.get('stamp', ''),
                  'corners': detection.count, 'markers': detection.marker_count,
                  'coverage': round(detection.coverage(), 4)}
                 for data, detection in self._read_shots(camera, width, height)]
        self._shots_cache[key] = (sig, items)
        return list(items)

    def load_intrinsic_views(self, camera: str, width: int, height: int) -> list[dict]:
        return [{'name': data['name'], 'detection': detection}
                for data, detection in self._read_shots(camera, width, height)]

    def read_intrinsic_image(self, camera: str, width: int, height: int, name: str):
        return self._read_image(self.intrinsic_dir(camera, width, height) / f'{_safe(name)}.png')

    def delete_intrinsic_shot(self, camera: str, width: int, height: int,
                              name: str) -> None:
        folder = self.intrinsic_dir(camera, width, height)
        for suffix in ('.png', '.json'):
            (folder / f'{_safe(name)}{suffix}').unlink(missing_ok=True)
        self._shots_cache.pop((camera, width, height), None)

    def clear_intrinsic(self, camera: str, width: int, height: int) -> None:
        shutil.rmtree(self.intrinsic_dir(camera, width, height), ignore_errors=True)
        self._shots_cache.pop((camera, width, height), None)

    # ---------- 外参 ----------

    def extrinsic_dir(self, camera: str | None = None) -> Path:
        folder = self.root / 'extrinsic'
        if camera:
            folder = folder / _safe(camera)
        folder.mkdir(parents=True, exist_ok=True)
        return folder

    def save_extrinsic_pose(self, images: dict, meta: dict, camera: str | None = None) -> str:
        folder = self.extrinsic_dir(camera)
        name = f'pose_{_next_index(folder.glob("pose_*")):03d}'
        target = folder / name
        target.mkdir()
        (target / 'meta.json').write_text(
            json.dumps({**meta, 'name': name, 'stamp': _stamp()},
                       ensure_ascii=False, default=_jsonable),
            encoding='utf-8')
        for cam_name, image in images.items():
            self._save_image_async(target / f'{_safe(cam_name)}.png', image)
        self._poses_cache.pop(camera, None)
        return name

    def list_extrinsic_poses(self, camera: str | None = None) -> list[dict]:
        folder = self.extrinsic_dir(camera)
        paths = sorted(folder.glob('pose_*/meta.json'))
        sig = tuple(p.parent.name for p in paths)
        cached = self._poses_cache.get(camera)
        if cached is not None and cached[0] == sig:
            return list(cached[1])
        items = [json.loads(path.read_text(encoding='utf-8')) for path in paths]
        self._poses_cache[camera] = (sig, items)
        return list(items)

    def read_extrinsic_image(self, name: str, camera: str, scope_camera: str | None = None):
        folder = self.extrinsic_dir(scope_camera)
        return self._read_image(folder / _safe(name) / f'{_safe(camera)}.png')

    def _save_image_async(self, path: Path, image: np.ndarray) -> None:
        key = str(path.resolve())
        img_copy = image.copy()
        with self._pending_lock:
            self._pending_images[key] = img_copy

        def _write() -> None:
            try:
                cv2.imwrite(key, img_copy, _PNG)
            finally:
                with self._pending_lock:
                    self._pending_images.pop(key, None)

        threading.Thread(target=_write, daemon=True).start()

    def _read_image(self, path: Path):
        key = str(path.resolve())
        with self._pending_lock:
            pending = self._pending_images.get(key)
        if pending is not None:
            return pending
        return _imread(path)

    def delete_extrinsic_pose(self, name: str, camera: str | None = None) -> None:
        folder = self.extrinsic_dir(camera)
        shutil.rmtree(folder / _safe(name), ignore_errors=True)
        self._poses_cache.pop(camera, None)

    def clear_extrinsic(self, camera: str | None = None) -> None:
        folder = self.extrinsic_dir(camera)
        shutil.rmtree(folder, ignore_errors=True)
        folder.mkdir(parents=True, exist_ok=True)
        self._poses_cache.pop(camera, None)

    # ---------- calibration.yaml ----------

    def read_calibration(self) -> dict:
        if self.calib_file.is_file():
            mtime_ns = self.calib_file.stat().st_mtime_ns
            if self._calib_cache[0] == mtime_ns:
                return dict(self._calib_cache[1])
            data = yaml.safe_load(self.calib_file.read_text(encoding='utf-8')) or {}
        else:
            mtime_ns = None
            data = {}
        data.setdefault('version', VERSION)
        for section in _SECTIONS:
            data.setdefault(section, {})
        self._calib_cache = (mtime_ns, data)
        return data

    def write_calibration(self, data: dict) -> str:
        self.calib_file.parent.mkdir(parents=True, exist_ok=True)
        text = yaml.safe_dump(data, allow_unicode=True, sort_keys=False, width=120)
        temporary = self.calib_file.with_suffix('.yaml.tmp')
        temporary.write_text(text, encoding='utf-8')
        os.replace(temporary, self.calib_file)
        self._calib_cache = (self.calib_file.stat().st_mtime_ns, data)
        return str(self.calib_file)

    def put_intrinsic(self, camera: str, result: dict, extra: dict | None = None) -> str:
        data = self.read_calibration()
        entry = {
            'width': result['width'], 'height': result['height'],
            'camera_matrix': result['camera_matrix'],
            'distortion_model': result['distortion_model'],
            'distortion_coefficients': result['distortion_coefficients'],
            'rms': result['rms'], 'images': result['images'],
            'coverage': result['coverage'], 'fov_deg': result['fov_deg'],
            'stamp': _stamp(),
        }
        entry.update(extra or {})
        entries = [e for e in data['intrinsics'].get(camera, [])
                   if (e['width'], e['height']) != (entry['width'], entry['height'])]
        entries.append(entry)
        entries.sort(key=lambda e: (-e['width'], -e['height']))
        data['intrinsics'][camera] = entries
        return self.write_calibration(data)

    def put_profile_relation(self, camera: str, relation: dict) -> str:
        data = self.read_calibration()
        relations = [r for r in data['profile_relations'].get(camera, [])
                     if (r['from'], r['to']) != (relation['from'], relation['to'])]
        relations.append(relation)
        data['profile_relations'][camera] = relations
        return self.write_calibration(data)

    def put_extrinsic(self, camera: str, entry: dict) -> str:
        data = self.read_calibration()
        data['extrinsics'][camera] = {**entry, 'stamp': _stamp()}
        return self.write_calibration(data)

    def put_urdf_override(self, joint: str, entry: dict) -> str:
        data = self.read_calibration()
        data['urdf_overrides'][joint] = {**entry, 'stamp': _stamp()}
        return self.write_calibration(data)


def find_intrinsic(data: dict, camera: str, width: int, height: int,
                   allow_scale: bool = True) -> dict | None:
    entries = data.get('intrinsics', {}).get(camera, [])
    for entry in entries:
        if entry['width'] == width and entry['height'] == height:
            return dict(entry)
    if not allow_scale:
        return None
    for relation in data.get('profile_relations', {}).get(camera, []):
        if relation.get('kind') != 'scale':
            continue
        source = tuple(relation['from'])
        if tuple(relation['to']) != (width, height):
            continue
        for entry in entries:
            if (entry['width'], entry['height']) != source:
                continue
            return _scale_entry(entry, width, height, source)
    return None


def _scale_entry(entry: dict, width: int, height: int, source) -> dict:
    ratio_x = width / source[0]
    ratio_y = height / source[1]
    matrix = np.asarray(entry['camera_matrix'], float).reshape(3, 3).copy()
    matrix[0, :] *= ratio_x
    matrix[1, :] *= ratio_y
    scaled = dict(entry)
    scaled.update({
        'width': width, 'height': height,
        'camera_matrix': [float(v) for v in matrix.reshape(9)],
        'scaled_from': list(source),
    })
    return scaled


def _imread(path: Path):
    return cv2.imread(str(path)) if path.is_file() else None


def _safe(name: str) -> str:
    name = str(name)
    if not name or name in ('.', '..') or os.path.basename(name) != name:
        raise ValueError(f'非法名字：{name}')
    return name


def _next_index(paths) -> int:
    largest = 0
    for path in paths:
        digits = ''.join(c for c in path.stem if c.isdigit())
        if digits:
            largest = max(largest, int(digits))
    return largest + 1


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    raise TypeError(f'不能序列化 {type(value)}')
