"""PiPER-X 单臂腕部相机内外参标定服务（纯 Python HTTP + RTSP 直连，非 ROS 节点）。"""

from __future__ import annotations

import argparse
from pathlib import Path
import threading
import time
from urllib.parse import parse_qs

import cv2
import numpy as np
import yaml

from wrist_camera import extrinsic, intrinsic, transforms
from wrist_camera.board import Board, probe
from wrist_camera.rtsp import default_calibration_path, default_cameras_config_path
from wrist_camera.sources import (
    CameraSource,
    Frames,
    GRIPPER_BASE_WITHOUT_FLANGE_TOP_Z_M,
    GRIPPER_BASE_WITH_FLANGE_TOP_Z_M,
    GRIPPER_WITH_WRENCH_TOP_Z_M,
    MotionGate,
    select_arm_camera,
)
from wrist_camera.storage import Store, find_intrinsic, save_board_config
from wrist_camera.webui import Panel, make_handler


class CalibrationApp:
    """单臂腕部相机 ChArUco 内参 + Eye-in-Hand 手眼外参标定应用。"""

    def __init__(
        self,
        *,
        arm: str = '',
        camera: str = '',
        connect_camera: bool = True,
        subscribe_joints: bool = True,
        dashboard_port: int = 8300,
        data_root: str | Path = '~/camera_calib_data',
        board_config: str | Path = '',
        cameras_config: str | Path = '',
        calib_file: str | Path = '',
        base_frame: str = '',
        preview_width: int = 720,
        jpeg_quality: int = 75,
        detect_period_s: float = 0.15,
    ) -> None:
        cv2.setNumThreads(1)
        cfg_dir = default_cameras_config_path().parent

        self.preview_width = int(preview_width)
        self.jpeg_quality = int(jpeg_quality)
        self.detect_period_s = float(detect_period_s)

        self.board_config_path = Path(board_config or (cfg_dir / 'board.yaml')).expanduser().resolve()
        self.cameras_config_path = Path(cameras_config or default_cameras_config_path()).expanduser().resolve()
        self.board_config = _read_yaml(self.board_config_path)
        self.cameras_config = _read_yaml(self.cameras_config_path)

        self.arm_id, self.camera_name, self.camera_cfg = select_arm_camera(
            self.cameras_config, arm, camera)
        self.base_frame = str(base_frame or self.camera_cfg.get('base_frame', 'base_link'))

        self.board = Board.from_config(self.board_config)
        self.store = Store(
            data_root or '~/camera_calib_data',
            calib_file or default_calibration_path(),
        )

        self.sources = {
            self.camera_name: CameraSource(
                self.camera_name, self.camera_cfg, auto_start=connect_camera),
        }
        gate_cfg = dict(self.cameras_config.get('motion_gate', {}))
        if self.camera_cfg.get('joint_states_topic'):
            gate_cfg['joint_states_topic'] = self.camera_cfg['joint_states_topic']
        default_joint_topic = f'/{self.arm_id}/joint_states_feedback'
        self.gate = MotionGate(
            gate_cfg, default_topic=default_joint_topic, subscribe_ros=subscribe_joints)
        self.gripper_variant = str(self.camera_cfg.get('gripper_variant', 'gripper_with_wrench'))
        default_top_z = float(self.camera_cfg.get('link6_to_top_z_m', GRIPPER_WITH_WRENCH_TOP_Z_M))
        self.frames = Frames(arm_id=self.arm_id, link6_to_top_z_m=default_top_z)

        self._live: dict[str, dict] = {}
        self._live_lock = threading.Lock()
        self._jpeg_cache: dict[tuple[str, bool], tuple[int, int, bytes]] = {}
        self._jpeg_lock = threading.Lock()
        self._results: dict[str, dict] = {}
        self._stop = threading.Event()
        threading.Thread(target=self._detect_loop, daemon=True, name='calib-detect').start()

        self.panel = Panel(
            make_handler(self._actions(), self._route),
            int(dashboard_port),
            f'PiPER-X 腕部相机标定 ({self.arm_id} / {self.camera_name})',
        )
        self.panel.start()

    @property
    def active_source(self) -> CameraSource:
        return self.sources[self.camera_name]

    def close(self) -> None:
        self._stop.set()
        self.panel.stop()
        for source in self.sources.values():
            source.close()
        self.gate.close()

    destroy_node = close

    def __enter__(self) -> 'CalibrationApp':
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def _detect_loop(self) -> None:
        period = self.detect_period_s
        while not self._stop.wait(0.05):
            started = time.monotonic()
            board = self.board
            for name, source in self.sources.items():
                frame = source.latest()
                if frame is None:
                    continue
                with self._live_lock:
                    cached = self._live.get(name)
                if cached is not None and cached['seq'] == frame[0] and cached.get('board') is board:
                    continue
                try:
                    detection = board.detect(frame[1])
                except Exception:
                    continue
                with self._live_lock:
                    self._live[name] = {'seq': frame[0], 'detection': detection,
                                        'image': frame[1], 'board': board}
            self._stop.wait(max(0.0, period - (time.monotonic() - started)))

    def _live_of(self, name: str):
        with self._live_lock:
            return self._live.get(name)

    def _intrinsic_for(self, source: CameraSource, size, calibration):
        entry = find_intrinsic(calibration, source.name, *size)
        if entry is not None:
            return entry, 'calibrated'
        return None, 'none'

    def state(self) -> dict:
        calibration = self.store.read_calibration()
        cameras = []
        for name, source in self.sources.items():
            status = source.status()
            live = self._live_of(name)
            detection = live['detection'] if live else None
            size = (status.get('width'), status.get('height'))
            origin = self._intrinsic_for(source, size, calibration)[1] if None not in size else 'none'
            status.update({
                'frame': source.frame,
                'parent_frame': source.parent_frame,
                'profiles': [{'width': p['width'], 'height': p['height']}
                             for p in source.profiles],
                'switchable': source.switchable,
                'corners': detection.count if detection else 0,
                'markers': detection.marker_count if detection else 0,
                'coverage': round(detection.coverage(), 3) if detection else 0.0,
                'max_corners': self.board.corner_count,
                'shots': self._shot_counts(name, calibration),
                'intrinsic_source': origin,
            })
            cameras.append(status)
        return {
            'arm': self.arm_id,
            'arm_namespace': self.camera_cfg.get('arm_namespace', f'/{self.arm_id}'),
            'camera': self.camera_name,
            'camera_ip': self.camera_cfg.get('ip', ''),
            'board': self.board.describe(),
            'board_config_file': str(self.board_config_path),
            'base_frame': self.base_frame,
            'reference_point': self._reference_point_info(),
            'cameras': cameras,
            'motion': self.gate.state(),
            'tf': self._tf_state(),
            'poses': [_pose_summary(p, self.camera_name)
                      for p in self.store.list_extrinsic_poses(self.camera_name)],
            'calib_file': str(self.store.calib_file),
            'saved_extrinsic': (calibration.get('extrinsics') or {}).get(self.camera_name),
            'results': self._results,
        }

    def _shot_counts(self, camera: str, calibration: dict) -> list[dict]:
        counts = []
        for profile in self.sources[camera].profiles:
            width, height = profile['width'], profile['height']
            entry = find_intrinsic(calibration, camera, width, height, allow_scale=False)
            shots = self.store.list_intrinsic_shots(camera, width, height)
            counts.append({
                'width': width, 'height': height,
                'shots': len(shots),
                'names': [s['name'] for s in shots],
                'rms': entry['rms'] if entry else None,
            })
        return counts

    def _tf_state(self) -> dict:
        needed = {}
        for source in self.sources.values():
            frame = source.parent_frame
            if frame:
                needed[source.name] = self.frames.status(
                    self.base_frame, frame, gate=self.gate)
        return {'base_frame': self.base_frame, 'links': needed,
                'ok': all(v['ok'] for v in needed.values())}

    def _reference_point_info(self) -> dict:
        return {
            'frame': 'gripper_base_top',
            'variant': self.gripper_variant,
            'link6_to_top_z_m': round(self.frames.link6_to_top_z_m, 6),
            'link6_to_top_z_mm': round(self.frames.link6_to_top_z_m * 1000.0, 2),
            'with_wrench_top_z_mm': round(GRIPPER_WITH_WRENCH_TOP_Z_M * 1000.0, 2),
            'with_flange_top_z_mm': round(GRIPPER_BASE_WITH_FLANGE_TOP_Z_M * 1000.0, 2),
            'without_flange_top_z_mm': round(GRIPPER_BASE_WITHOUT_FLANGE_TOP_Z_M * 1000.0, 2),
        }

    def update_reference_point(self, body: dict) -> dict:
        variant = str(body.get('variant') or self.gripper_variant).strip()
        if variant == 'gripper_with_wrench':
            z_m = GRIPPER_WITH_WRENCH_TOP_Z_M
        elif variant == 'with_flange':
            z_m = GRIPPER_BASE_WITH_FLANGE_TOP_Z_M
        elif variant == 'without_flange':
            z_m = GRIPPER_BASE_WITHOUT_FLANGE_TOP_Z_M
        elif variant == 'custom':
            mm = float(body.get('link6_to_top_z_mm', self.frames.link6_to_top_z_m * 1000.0))
            if mm <= 0:
                raise ValueError('link6 到夹爪最高点距离必须为正数')
            z_m = mm / 1000.0
        else:
            raise ValueError(f'未知夹爪参考点类型：{variant}')
        self.gripper_variant = variant
        self.frames.link6_to_top_z_m = float(z_m)
        return self._reference_point_info()

    def update_board(self, body: dict) -> dict:
        current = self.board.describe()
        squares_x = int(body.get('squares_x', current['squares_x']))
        squares_y = int(body.get('squares_y', current['squares_y']))
        if 'square_size_mm' in body and body['square_size_mm'] is not None:
            square_size = float(body['square_size_mm']) / 1000.0
        else:
            square_size = float(body.get('square_size', current['square_size']))
        if 'marker_size_mm' in body and body['marker_size_mm'] is not None:
            marker_size = float(body['marker_size_mm']) / 1000.0
        else:
            marker_size = float(body.get('marker_size', current['marker_size']))
        dictionary = str(body.get('dictionary') or current['dictionary']).strip()

        new_board = Board(squares_x, squares_y, square_size, marker_size, dictionary)
        self.board = new_board
        described = new_board.describe()
        self.board_config['board'] = {
            'squares_x': described['squares_x'],
            'squares_y': described['squares_y'],
            'square_size': described['square_size'],
            'marker_size': described['marker_size'],
            'dictionary': described['dictionary'],
        }
        saved_path = save_board_config(
            self.board_config_path, self.board_config['board'], self.board_config)
        with self._live_lock:
            self._live.clear()
        return {'board': described, 'path': saved_path}

    def capture_intrinsic(self, body: dict) -> dict:
        names = body.get('cameras') or [self.camera_name]
        saved = []
        for name in names:
            source = self._source(name)
            live = self._live_of(name)
            if live is None:
                raise RuntimeError(f'{source.label} 还没收到图')
            detection = live['detection']
            if detection.count < intrinsic.MIN_CORNERS:
                saved.append({'camera': name, 'ok': False,
                              'reason': f'只认出 {detection.count} 个角点，至少要 '
                                        f'{intrinsic.MIN_CORNERS} 个'})
                continue
            shot = self.store.save_intrinsic_shot(name, live['image'], detection)
            saved.append({'camera': name, 'ok': True, 'name': shot,
                          'corners': detection.count,
                          'coverage': round(detection.coverage(), 3)})
        return {'saved': saved}

    def solve_intrinsic(self, body: dict) -> dict:
        name = body.get('camera') or self.camera_name
        width, height = _size(body)
        self._source(name)
        views = self.store.load_intrinsic_views(name, width, height)
        result = intrinsic.calibrate(self.board, views)
        if result.get('ok'):
            result['outliers'] = intrinsic.outliers(result)
        self._results[f'{name}:{width}x{height}'] = result
        return result

    def save_intrinsic(self, body: dict) -> dict:
        name = body.get('camera') or self.camera_name
        width, height = _size(body)
        result = self._results.get(f'{name}:{width}x{height}')
        if not result or not result.get('ok'):
            raise RuntimeError('先点「求解」并确认结果合理再保存')
        path = self.store.put_intrinsic(name, result, {
            'board': self.board.describe(), 'source': 'wrist_camera'})
        return {'path': path, 'rms': result['rms']}

    def drop_outliers(self, body: dict) -> dict:
        name = body.get('camera') or self.camera_name
        width, height = _size(body)
        result = self._results.get(f'{name}:{width}x{height}')
        if not result or not result.get('ok'):
            raise RuntimeError('先点「求解」')
        bad = result.get('outliers') or intrinsic.outliers(result)
        for shot in bad:
            self.store.delete_intrinsic_shot(name, width, height, shot)
        return {'dropped': bad, 'result': self.solve_intrinsic(body)}

    def delete_shot(self, body: dict) -> dict:
        name = body.get('camera') or self.camera_name
        self.store.delete_intrinsic_shot(name, *_size(body), body['name'])
        return {'deleted': body['name']}

    def clear_shots(self, body: dict) -> dict:
        name = body.get('camera') or self.camera_name
        width, height = _size(body)
        self.store.clear_intrinsic(name, width, height)
        self._results.pop(f'{name}:{width}x{height}', None)
        return {'cleared': True}

    def probe_board(self, body: dict) -> dict:
        name = body.get('camera') or self.camera_name
        live = self._live_of(name)
        if live is None:
            raise RuntimeError(f'{name} 还没收到图')
        return {'camera': name, 'candidates': probe(live['image'], self.board_config)}

    def relate_profiles(self, body: dict) -> dict:
        name = body.get('camera') or self.camera_name
        calibration = self.store.read_calibration()
        entries = calibration.get('intrinsics', {}).get(name, [])
        if len(entries) < 2:
            raise RuntimeError(f'{name} 至少要标完两个档位才能判断它们是缩放还是裁剪')
        ordered = sorted(entries, key=lambda e: e['width'] * e['height'], reverse=True)
        highest = ordered[0]
        relations = []
        for lower in ordered[1:]:
            verdict = intrinsic.relate_profiles(lower, highest)
            record = {
                'from': [highest['width'], highest['height']],
                'to': [lower['width'], lower['height']], **verdict,
            }
            self.store.put_profile_relation(name, record)
            relations.append(record)
        return {'camera': name, 'relations': relations}

    def _pose_sample(self, source: CameraSource, detection, calibration):
        entry, origin = self._intrinsic_for(source, detection.size, calibration)
        if entry is None:
            width, height = detection.size
            raise RuntimeError(
                f'{source.label} 在 {width}x{height} 下没有内参，先去「内参」页标完并保存')
        pose = self.board.estimate_pose(
            detection, entry['camera_matrix'], entry['distortion_coefficients'])
        if pose is None:
            raise RuntimeError(
                f'{source.label} 只看到 {detection.count} 个角点，解不出板位姿（至少要 6 个）')
        return pose, entry, origin

    def capture_pose(self, body: dict) -> dict:
        if not body.get('force'):
            motion = self.gate.state()
            if not motion['ok']:
                raise RuntimeError(f"不能采集：{motion['reason']}")

        source = self.active_source
        target_live = self._live_of(source.name)
        if target_live is None:
            raise RuntimeError(f'{source.label} 还没收到图')

        calibration = self.store.read_calibration()
        pose, entry, origin = self._pose_sample(
            source, target_live['detection'], calibration)
        link = self.frames.lookup(
            self.base_frame, source.parent_frame, gate=self.gate)
        joints = self.gate.latest_joints()

        images = {source.name: target_live['image']}
        meta = {
            'arm': self.arm_id,
            'camera': source.name,
            'base_frame': self.base_frame,
            'reference_point': self._reference_point_info(),
            'targets': {
                source.name: {
                    'frame': source.frame,
                    'parent': source.parent_frame,
                    'corners': target_live['detection'].count,
                    'width': entry['width'],
                    'height': entry['height'],
                    'intrinsic_source': origin,
                    'link6_to_top_z_m': self.frames.link6_to_top_z_m,
                    'gripper_variant': self.gripper_variant,
                    'joint_positions': list(joints) if joints is not None else None,
                    'T_base_link': link.tolist(),
                    'T_cam_board': pose.tolist(),
                },
            },
        }
        name = self.store.save_extrinsic_pose(images, meta, camera=self.camera_name)
        return {'name': name, 'targets': [source.name], 'corners': target_live['detection'].count}

    def solve_extrinsic(self, body: dict) -> dict:
        poses = self.store.list_extrinsic_poses(self.camera_name)
        if not poses:
            raise RuntimeError('一个姿态都还没采')
        source = self.active_source
        name = source.name
        samples = []
        for pose in poses:
            target = pose.get('targets', {}).get(name)
            if not target:
                continue
            samples.append({
                'name': pose['name'],
                'T_base_link': np.asarray(target['T_base_link'], float),
                'T_cam_board': np.asarray(target['T_cam_board'], float),
            })
        if not samples:
            raise RuntimeError(f'{source.label} 没有可用姿态样本')

        handeye = extrinsic.solve_handeye(samples)
        if handeye.get('ok'):
            handeye['mesh_origins'] = _mesh_origins(handeye['transform'])
        joint = extrinsic.solve_joint(samples)
        if joint.get('ok'):
            joint['mesh_origins'] = _mesh_origins(joint['transform'])
        ref_matrix = (
            joint.get('matrix') if joint.get('ok')
            else (handeye.get('matrix') if handeye.get('ok') else None)
        )
        stability = extrinsic.board_stability(samples, transform=ref_matrix)
        entry = {
            'camera': name,
            'arm': self.arm_id,
            'label': source.label,
            'samples': len(samples),
            'pose_names': [s['name'] for s in samples],
            'parent': source.parent_frame,
            'child': source.frame,
            'board_stability': stability,
            'joint': joint,
            'handeye': handeye,
        }
        if joint.get('ok') and handeye.get('ok'):
            entry['cross_check'] = extrinsic.compare(joint['matrix'], handeye['matrix'])
        payload = {'cameras': {name: entry}}
        self._results['extrinsic'] = payload
        return payload

    def save_extrinsic(self, body: dict) -> dict:
        name = body.get('camera') or self.camera_name
        method = body.get('method', 'joint')
        stored = self._results.get('extrinsic') or {}
        entry = (stored.get('cameras') or {}).get(name)
        if not entry:
            raise RuntimeError('先点求解')
        solution = entry.get(method)
        if not solution or not solution.get('ok'):
            raise RuntimeError(f'{method} 这一路没有可用解')
        extra = {key: solution[key] for key in
                 ('residual_mm', 'well_posed', 'condition', 'board_pose', 'best')
                 if key in solution}
        if 'cross_check' in entry:
            extra['cross_check'] = entry['cross_check']

        source = self._source(name)
        payload = {
            'arm': self.arm_id,
            'reference_point': 'gripper_base_top',
            'gripper_variant': self.gripper_variant,
            'link6_to_top_z_m': round(self.frames.link6_to_top_z_m, 6),
            'mesh_origins': _mesh_origins(solution['transform']),
            'parent': source.parent_frame,
            'child': source.frame,
            'method': method,
            'samples': solution.get('samples', entry.get('samples')),
            'consistency': solution['consistency'],
            **solution['transform'],
            **extra,
        }
        return {'path': self.store.put_extrinsic(name, payload)}

    def delete_pose(self, body: dict) -> dict:
        self.store.delete_extrinsic_pose(body['name'], camera=self.camera_name)
        return {'deleted': body['name']}

    def clear_poses(self, body: dict) -> dict:
        self.store.clear_extrinsic(camera=self.camera_name)
        self._results.pop('extrinsic', None)
        return {'cleared': True}

    def apply_profile(self, body: dict) -> dict:
        name = body.get('camera') or self.camera_name
        return self._source(name).apply_profile(*_size(body))

    def _source(self, name: str) -> CameraSource:
        source = self.sources.get(name)
        if source is None:
            raise ValueError(f'当前标定会话仅加载了 {self.camera_name}，没有 {name}')
        return source

    def _actions(self) -> dict:
        return {
            '/api/board': self.update_board,
            '/api/reference_point': self.update_reference_point,
            '/api/profile': self.apply_profile,
            '/api/capture': self.capture_intrinsic,
            '/api/delete_shot': self.delete_shot,
            '/api/clear_shots': self.clear_shots,
            '/api/solve_intrinsic': self.solve_intrinsic,
            '/api/save_intrinsic': self.save_intrinsic,
            '/api/drop_outliers': self.drop_outliers,
            '/api/probe_board': self.probe_board,
            '/api/relate': self.relate_profiles,
            '/api/capture_pose': self.capture_pose,
            '/api/delete_pose': self.delete_pose,
            '/api/clear_poses': self.clear_poses,
            '/api/solve_extrinsic': self.solve_extrinsic,
            '/api/save_extrinsic': self.save_extrinsic,
        }

    def _route(self, handler, url) -> None:
        path = url.path
        query = parse_qs(url.query)
        if path in ('/', '/index.html'):
            return handler.send_static('index.html')
        if path in ('/app.js', '/app.css', '/common.js'):
            return handler.send_static(path.lstrip('/'))
        if path == '/api/state':
            return handler.send_json(self.state())
        if path == '/api/stream':
            return self._send_stream(handler, query)
        if path == '/api/preview':
            return self._send_preview(handler, query)
        if path == '/api/shot':
            return self._send_image(handler, self.store.read_intrinsic_image(
                _one(query, 'camera', self.camera_name), int(_one(query, 'width')),
                int(_one(query, 'height')), _one(query, 'name')))
        if path == '/api/pose_image':
            cam = _one(query, 'camera', self.camera_name)
            return self._send_image(handler, self.store.read_extrinsic_image(
                _one(query, 'name'), cam, scope_camera=self.camera_name))
        return handler.send_json({'error': 'not found'}, 404)

    def _send_image(self, handler, image) -> None:
        if image is None:
            return handler.send_json({'error': 'not found'}, 404)
        return handler.send_bytes(200, self._jpeg(image), 'image/jpeg')

    def _send_preview(self, handler, query) -> None:
        cam = _one(query, 'camera', self.camera_name)
        source = self.sources.get(cam)
        frame = source.latest() if source is not None else None
        live = self._live_of(cam)
        if frame is None and live is None:
            return handler.send_json({'error': '还没收到图'}, 503)
        overlay = _one(query, 'overlay', '1') != '0'
        if frame is not None:
            seq, image, _ = frame
            return handler.send_bytes(
                200, self._preview_jpeg(cam, seq, image, overlay=overlay), 'image/jpeg')
        assert live is not None
        image = live['image']
        detection = live['detection'] if overlay else None
        return handler.send_bytes(200, self._jpeg(image, detection=detection), 'image/jpeg')

    def _send_stream(self, handler, query) -> None:
        cam = _one(query, 'camera', self.camera_name)
        source = self._source(cam)
        overlay = _one(query, 'overlay', '1') != '0'
        handler.send_response(200)
        handler.send_header('Cache-Control', 'no-store')
        handler.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        handler.end_headers()
        seq = 0
        try:
            while not self._stop.is_set():
                frame = source.wait_frame(seq, timeout=1.0)
                if frame is None:
                    continue
                seq, image, _ = frame
                jpeg = self._preview_jpeg(cam, seq, image, overlay=overlay)
                handler.wfile.write(
                    b'--frame\r\nContent-Type: image/jpeg\r\n'
                    + f'Content-Length: {len(jpeg)}\r\n\r\n'.encode('ascii')
                    + jpeg + b'\r\n'
                )
                handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def _preview_jpeg(self, cam: str, seq: int, image, *, overlay: bool) -> bytes:
        live = self._live_of(cam) if overlay else None
        det_seq = (
            live['seq']
            if live is not None and live['detection'].size == (image.shape[1], image.shape[0])
            else 0
        )
        key = (cam, overlay)
        with self._jpeg_lock:
            cached = self._jpeg_cache.get(key)
            if cached is not None and cached[0] == seq and cached[1] == det_seq:
                return cached[2]
        detection = live['detection'] if (det_seq and live is not None) else None
        jpeg = self._jpeg(image, detection=detection)
        with self._jpeg_lock:
            self._jpeg_cache[key] = (seq, det_seq, jpeg)
        return jpeg

    def _jpeg(self, image, detection=None) -> bytes:
        resized = False
        if image.shape[1] > self.preview_width:
            scale = self.preview_width / image.shape[1]
            image = cv2.resize(image, None, fx=scale, fy=scale,
                               interpolation=cv2.INTER_LINEAR)
            resized = True
        if detection is not None:
            image = self.board.overlay(image, detection, inplace=resized)
        ok, buffer = cv2.imencode('.jpg', image,
                                  [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            raise RuntimeError('JPEG 编码失败')
        return buffer.tobytes()


CalibrationNode = CalibrationApp


def _mesh_origins(transform: dict) -> dict:
    tx, ty, tz = (float(v) for v in transform['translation'])
    return {
        'gripper_base_top': [round(tx, 6), round(ty, 6), round(tz, 6)],
        'gripper_base_without_flange': [
            round(tx, 6), round(ty, 6), round(tz + GRIPPER_BASE_WITHOUT_FLANGE_TOP_Z_M, 6),
        ],
        'gripper_base_with_flange': [
            round(tx, 6), round(ty, 6), round(tz + GRIPPER_BASE_WITH_FLANGE_TOP_Z_M, 6),
        ],
        'link6_with_wrench': [
            round(tx, 6), round(ty, 6), round(tz + GRIPPER_WITH_WRENCH_TOP_Z_M, 6),
        ],
    }


def _pose_summary(pose: dict, camera_name: str) -> dict:
    target = (pose.get('targets') or {}).get(camera_name, {})
    t_base_link = target.get('T_base_link')
    xyz = None
    if t_base_link is not None:
        mat = np.asarray(t_base_link, float)
        if mat.shape == (4, 4):
            xyz = [round(float(v), 4) for v in mat[:3, 3]]
    return {
        'name': pose['name'],
        'stamp': pose.get('stamp', ''),
        'corners': target.get('corners', 0),
        'intrinsic_source': target.get('intrinsic_source', ''),
        'xyz': xyz,
        'targets': {k: v.get('corners', 0) for k, v in (pose.get('targets') or {}).items()},
    }


def _read_yaml(path) -> dict:
    text = Path(path).expanduser().read_text(encoding='utf-8')
    return yaml.safe_load(text) or {}


def _size(body: dict) -> tuple[int, int]:
    return int(body['width']), int(body['height'])


def _one(query: dict, key: str, default=None) -> str:
    values = query.get(key)
    if not values:
        if default is None:
            raise ValueError(f'缺少查询参数 {key}')
        return default
    return values[0]


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description='PiPER-X 单臂腕部相机内外参标定 Web 面板')
    parser.add_argument('--arm', default='', help='目标机械臂：arm_3 或 arm_4')
    parser.add_argument('--camera', default='', help='可选相机别名（如 camera_arm_4、cam3、IP）')
    parser.add_argument('--port', type=int, default=8300)
    parser.add_argument('--data-root', default='~/camera_calib_data')
    parser.add_argument('--board-config', default='')
    parser.add_argument('--cameras-config', default='')
    parser.add_argument('--calib-file', default='')
    parser.add_argument('--preview-width', type=int, default=720)
    parser.add_argument('--jpeg-quality', type=int, default=75)
    args = parser.parse_args(argv)

    app = CalibrationApp(
        arm=args.arm,
        camera=args.camera,
        dashboard_port=args.port,
        data_root=args.data_root,
        board_config=args.board_config,
        cameras_config=args.cameras_config,
        calib_file=args.calib_file,
        preview_width=args.preview_width,
        jpeg_quality=args.jpeg_quality,
    )
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        app.close()


if __name__ == '__main__':
    main()
