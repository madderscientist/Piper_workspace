import json
from pathlib import Path
import sys
import time
from urllib.request import Request, urlopen

import cv2
import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from conftest import front_pose, render
from piper_x_driver.gravity import _fast_fk
from wrist_camera import transforms
from wrist_camera.board import Board
from wrist_camera.calib import CalibrationApp
from wrist_camera.sources import (
    CameraSource,
    GRIPPER_BASE_WITHOUT_FLANGE_TOP_Z_M,
    GRIPPER_BASE_WITH_FLANGE_TOP_Z_M,
    piper_fk,
    select_arm_camera,
)

CONFIG = Path(__file__).resolve().parents[1] / 'config'


@pytest.fixture(scope='module')
def cameras_cfg():
    return yaml.safe_load((CONFIG / 'cameras.yaml').read_text(encoding='utf-8'))


@pytest.fixture(scope='module')
def board_cfg():
    return yaml.safe_load((CONFIG / 'board.yaml').read_text(encoding='utf-8'))


def _post(port: int, path: str, body: dict) -> dict:
    req = Request(
        f'http://127.0.0.1:{port}{path}',
        data=json.dumps(body).encode('utf-8'),
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    with urlopen(req, timeout=5.0) as resp:
        return json.loads(resp.read().decode('utf-8'))['result']


def test_config_arm_selection_and_fk_agreement(board_cfg, cameras_cfg):
    board = Board.from_config(board_cfg)
    assert {(board.squares_x, board.squares_y)} <= {(9, 12), (12, 9)}
    assert board.dictionary_name == 'DICT_5X5_100'

    arm3, cam3, cfg3 = select_arm_camera(cameras_cfg, 'arm_3')
    assert (arm3, cam3, cfg3['ip']) == ('arm_3', 'camera_arm_3', '192.168.123.97')
    arm4, cam4, cfg4 = select_arm_camera(cameras_cfg, '', 'cam4')
    assert (arm4, cam4, cfg4['ip']) == ('arm_4', 'camera_arm_4', '192.168.123.98')

    for bad_arm, bad_cam in [('', ''), ('arm_1', ''), ('arm_3', 'camera_arm_4')]:
        with pytest.raises(ValueError):
            select_arm_camera(cameras_cfg, bad_arm, bad_cam)

    q = (0.15, 0.85, -1.25, 0.35, -0.45, 0.55)
    _, _, centers, rots = _fast_fk(q, (0.0, 0.0, GRIPPER_BASE_WITH_FLANGE_TOP_Z_M))
    expected = transforms.rt_to_matrix(np.asarray(rots[5], float), centers[6])
    assert np.allclose(piper_fk(q, tool_offset_z_m=GRIPPER_BASE_WITH_FLANGE_TOP_Z_M), expected, atol=1e-12)


def test_camera_source_apply_profile_switches_native_stream(cameras_cfg):
    _, _, cfg = select_arm_camera(cameras_cfg, 'arm_4')
    opened = []

    class _FakeReader:
        def __init__(self, command, width, height, on_frame):
            opened.append((list(command), width, height))
            self.width = width
            self.height = height
            self._stopped = False
            on_frame(bytes([7, 8, 9]) * (width * height), width, height)

        def alive(self):
            return not self._stopped

        def frame_age(self):
            return 0.0

        def stop(self):
            self._stopped = True

    src = CameraSource('camera_arm_4', cfg, reader_cls=_FakeReader, poll_period_s=0.05)
    try:
        first = src.wait_frame(0, timeout=1.0)
        assert first is not None and first[1].shape == (480, 720, 3)
        assert 'rtsp://admin:123456@192.168.123.98/stream1' in opened[0][0]

        applied = src.apply_profile(1920, 1080, timeout=1.0)
        assert applied == {'width': 1920, 'height': 1080}
        assert src.latest()[1].shape == (1080, 1920, 3)
        assert 'rtsp://admin:123456@192.168.123.98/stream0' in opened[-1][0]
    finally:
        src.close()


def test_dashboard_single_arm_extrinsic_workflow(tmp_path, board, flat):
    board_file = tmp_path / 'board.yaml'
    board_file.write_text((CONFIG / 'board.yaml').read_text(encoding='utf-8'), encoding='utf-8')
    calib_file = tmp_path / 'calibration.yaml'
    data_root = tmp_path / 'calib_data'

    with pytest.raises(ValueError):
        CalibrationApp(connect_camera=False, subscribe_joints=False, dashboard_port=0)

    with CalibrationApp(
        arm='arm_3', connect_camera=False, subscribe_joints=False, dashboard_port=0,
        data_root=data_root, board_config=board_file, calib_file=calib_file,
    ) as app:
        port = app.panel.port
        with urlopen(f'http://127.0.0.1:{port}/api/state', timeout=3.0) as resp:
            state = json.loads(resp.read().decode('utf-8'))
        assert state['arm'] == 'arm_3' and state['camera'] == 'camera_arm_3'

        app.active_source.push_bgr(np.zeros((1080, 1920, 3), dtype=np.uint8))
        with urlopen(f'http://127.0.0.1:{port}/api/preview?camera=camera_arm_3&overlay=1', timeout=3.0) as resp:
            preview_img = cv2.imdecode(np.frombuffer(resp.read(), np.uint8), cv2.IMREAD_COLOR)
        assert preview_img is not None and preview_img.shape[1] == app.preview_width

        _post(port, '/api/board', {
            'squares_x': 9, 'squares_y': 12, 'square_size_mm': 30.0,
            'marker_size_mm': 22.5, 'dictionary': 'DICT_5X5',
        })

        k_mat = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]])
        d_vec = np.zeros(5)
        size = (1280, 720)
        app.store.put_intrinsic('camera_arm_3', {
            'width': 1280, 'height': 720,
            'camera_matrix': k_mat.reshape(9).tolist(),
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': d_vec.tolist(),
            'rms': 0.15, 'images': 15, 'coverage': 0.7, 'fov_deg': [70.8, 43.6],
        })

        true_link_cam = transforms.rt_to_matrix(
            cv2.Rodrigues(np.array([0.05, -0.08, 0.12]))[0], [0.02, -0.01, 0.05])
        t_base_board = transforms.rt_to_matrix(
            cv2.Rodrigues(np.array([2.9, 0.1, -0.1]))[0], [0.45, 0.0, 0.05])
        cam_views = [
            front_pose(board, [0.15, -0.20, 0.10], [0.02, -0.01, 0.55]),
            front_pose(board, [-0.25, 0.18, -0.20], [-0.03, 0.02, 0.58]),
            front_pose(board, [0.22, 0.25, -0.30], [0.01, 0.03, 0.52]),
            front_pose(board, [-0.18, -0.28, 0.25], [-0.02, -0.02, 0.60]),
        ]

        for idx, t_cam_board in enumerate(cam_views):
            t_base_link = t_base_board @ transforms.invert(t_cam_board) @ transforms.invert(true_link_cam)
            app.frames.lookup = lambda *_a, _m=t_base_link, **_kw: _m
            scene, _ = render(board, flat, t_cam_board, k_mat, d_vec, size)
            det = app.board.detect(scene)
            assert det.count >= 6
            app.active_source.push_bgr(scene)
            cur_seq = app.active_source.latest()[0]
            with app.gate._lock:
                app.gate._speed = 0.0
                app.gate._still_since = time.monotonic() - 1.0
                app.gate._stamp = time.monotonic()
                app.gate._joints = (0.0, 0.7, -1.2, 0.0, -0.3, 0.0)
            with app._live_lock:
                app._live['camera_arm_3'] = {
                    'seq': cur_seq, 'detection': det, 'image': scene, 'board': app.board,
                }
            cap = _post(port, '/api/capture_pose', {})
            assert cap['name'] == f'pose_{idx + 1:03d}'

        solved = _post(port, '/api/solve_extrinsic', {})
        cam_res = solved['cameras']['camera_arm_3']
        assert cam_res['joint']['ok'] and cam_res['handeye']['ok']
        err_deg, err_m = transforms.transform_delta(
            true_link_cam, np.asarray(cam_res['joint']['matrix'], float))
        assert err_deg < 1.0 and err_m < 0.01

        saved = _post(port, '/api/save_extrinsic', {'camera': 'camera_arm_3', 'method': 'joint'})
        assert Path(saved['path']).is_file()
        calib_data = app.store.read_calibration()
        assert 'urdf_overrides' not in calib_data
        assert calib_data['extrinsics']['camera_arm_3']['parent'] == 'gripper_base_top'
        origins = calib_data['extrinsics']['camera_arm_3']['mesh_origins']
        assert origins['gripper_base_without_flange'][2] == pytest.approx(
            origins['gripper_base_top'][2] + GRIPPER_BASE_WITHOUT_FLANGE_TOP_Z_M, abs=1e-5)
        assert origins['gripper_base_with_flange'][2] == pytest.approx(
            origins['gripper_base_top'][2] + GRIPPER_BASE_WITH_FLANGE_TOP_Z_M, abs=1e-5)
