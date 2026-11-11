import json
from pathlib import Path
import socket
import sys
import time
from urllib.request import Request, urlopen

import cv2
import numpy as np
import pytest
import rclpy
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from camera_calibration import storage, transforms
from camera_calibration.board import Board
from camera_calibration.calib_node import CalibrationNode
from camera_calibration.sources import (
    CameraSource,
    GRIPPER_BASE_WITHOUT_FLANGE_TOP_Z_M,
    GRIPPER_BASE_WITH_FLANGE_TOP_Z_M,
    piper_fk,
    select_arm_camera,
)
from conftest import front_pose, render
from piper_x_driver.gravity import _fast_fk

CONFIG = Path(__file__).resolve().parents[1] / 'config'


@pytest.fixture(scope='module')
def cameras_cfg():
    return yaml.safe_load((CONFIG / 'cameras.yaml').read_text(encoding='utf-8'))


@pytest.fixture(scope='module')
def board_cfg():
    return yaml.safe_load((CONFIG / 'board.yaml').read_text(encoding='utf-8'))


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('127.0.0.1', 0))
        return int(s.getsockname()[1])


def _post(port: int, path: str, body: dict) -> dict:
    req = Request(
        f'http://127.0.0.1:{port}{path}',
        data=json.dumps(body).encode('utf-8'),
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    with urlopen(req, timeout=5.0) as resp:
        return json.loads(resp.read().decode('utf-8'))['result']


def test_default_board_and_cameras_config(board_cfg, cameras_cfg):
    board = Board.from_config(board_cfg)
    assert {(board.squares_x, board.squares_y)} <= {(9, 12), (12, 9)}
    assert board.square_size > board.marker_size > 0
    assert board.dictionary_name == 'DICT_5X5_100'

    cams = cameras_cfg['cameras']
    assert set(cams) == {'camera_arm_3', 'camera_arm_4'}
    assert cams['camera_arm_3']['ip'] == '192.168.123.97'
    assert cams['camera_arm_4']['ip'] == '192.168.123.98'
    assert cams['camera_arm_3']['profiles'][0] == {
        'width': 720, 'height': 480,
        'url': 'rtsp://admin:123456@192.168.123.97/stream1',
    }
    assert cams['camera_arm_4']['profiles'][0] == {
        'width': 720, 'height': 480,
        'url': 'rtsp://admin:123456@192.168.123.98/stream1',
    }


def test_select_arm_camera_requires_both_and_validates_match(cameras_cfg):
    arm3, cam3, cfg3 = select_arm_camera(cameras_cfg, 'arm3', 'arm_3_camera')
    assert arm3 == 'arm_3' and cam3 == 'camera_arm_3' and cfg3['ip'] == '192.168.123.97'

    arm4, cam4, cfg4 = select_arm_camera(cameras_cfg, 'arm_4', 'camera_arm_4')
    assert arm4 == 'arm_4' and cam4 == 'camera_arm_4' and cfg4['ip'] == '192.168.123.98'

    for bad_arm, bad_cam in [
        ('', 'camera_arm_3'),
        ('arm_3', ''),
        ('arm_1', 'camera_arm_3'),
        ('arm_2', 'camera_arm_4'),
        ('arm_3', 'camera_arm_4'),
    ]:
        with pytest.raises(ValueError):
            select_arm_camera(cameras_cfg, bad_arm, bad_cam)


def test_camera_source_apply_profile_switches_native_stream(cameras_cfg):
    _, _, cfg = select_arm_camera(cameras_cfg, 'arm_4', 'camera_arm_4')
    sent = []
    src = CameraSource.__new__(CameraSource)
    src.label = cfg['label']
    src.config = cfg
    src.profiles = cfg['profiles']
    src._client = object()
    src._local_camera = None
    src._call = lambda params, timeout=10.0: sent.extend(params)
    src.grab = lambda timeout=1.0: np.zeros((1080, 1920, 3), dtype=np.uint8)

    applied = src.apply_profile(1920, 1080, timeout=1.0)
    assert applied == {'width': 1920, 'height': 1080}
    by_name = {
        p.name: (p.value.string_value or p.value.integer_value)
        for p in sent
    }
    assert by_name == {
        'rtsp_url': 'rtsp://admin:123456@192.168.123.98/stream0',
        'image_width': 0,
        'image_height': 0,
    }

    local_params = []
    src_local = CameraSource.__new__(CameraSource)
    src_local.label = cfg['label']
    src_local.config = cfg
    src_local.profiles = cfg['profiles']
    src_local._client = None
    src_local._lock = time
    frames = []
    src_local.push_bgr = lambda img: frames.append(img.shape)
    src_local.grab = lambda timeout=1.0: np.zeros((1080, 1920, 3), dtype=np.uint8)

    class _DummyLocalCam:
        def add_local_sink(self, cb):
            self.sink = cb

        def set_parameters(self, params):
            local_params.extend(params)
            return [type('R', (), {'successful': True, 'reason': ''})() for _ in params]

    dummy_cam = _DummyLocalCam()
    src_local.bind_local_camera(dummy_cam)
    dummy_cam.sink(bytes(1920 * 1080 * 3), 1920, 1080, None)
    assert frames == [(1080, 1920, 3)]
    assert src_local.apply_profile(1920, 1080, timeout=1.0) == {'width': 1920, 'height': 1080}
    assert {p.name: p.value for p in local_params} == {
        'rtsp_url': 'rtsp://admin:123456@192.168.123.98/stream0',
        'image_width': 0,
        'image_height': 0,
    }


def test_piper_fk_matches_driver_gravity_fk():
    q = (0.15, 0.85, -1.25, 0.35, -0.45, 0.55)
    _, _, centers, rots = _fast_fk(q, (0.0, 0.0, GRIPPER_BASE_WITH_FLANGE_TOP_Z_M))
    expected = transforms.rt_to_matrix(np.asarray(rots[5], float), centers[6])
    actual = piper_fk(q, tool_offset_z_m=GRIPPER_BASE_WITH_FLANGE_TOP_Z_M)
    assert np.allclose(actual, expected, atol=1e-12)


def test_dashboard_single_arm_board_intrinsic_and_extrinsic_workflow(tmp_path, board, flat):
    board_file = tmp_path / 'board.yaml'
    board_file.write_text((CONFIG / 'board.yaml').read_text(encoding='utf-8'), encoding='utf-8')
    calib_file = tmp_path / 'calibration.yaml'
    data_root = tmp_path / 'calib_data'
    port = _free_port()

    needs_init = not rclpy.ok()
    if needs_init:
        rclpy.init()
    node = None
    try:
        with pytest.raises(ValueError):
            CalibrationNode()

        node = CalibrationNode(
            arm='arm_3', camera='camera_arm_3', dashboard_port=port,
            data_root=data_root, board_config=board_file, calib_file=calib_file,
        )

        with urlopen(f'http://127.0.0.1:{port}/api/state', timeout=3.0) as resp:
            state = json.loads(resp.read().decode('utf-8'))
        assert state['arm'] == 'arm_3'
        assert state['camera'] == 'camera_arm_3'
        assert state['camera_ip'] == '192.168.123.97'
        assert [c['name'] for c in state['cameras']] == ['camera_arm_3']
        assert (state['board']['squares_x'], state['board']['squares_y']) in {(9, 12), (12, 9)}

        # 验证缩放到 preview_width 后再画叠加图的返回尺寸与速度
        dummy_1080 = np.zeros((1080, 1920, 3), dtype=np.uint8)
        node.active_source.push_bgr(dummy_1080)
        with urlopen(f'http://127.0.0.1:{port}/api/preview?camera=camera_arm_3&overlay=1', timeout=3.0) as resp:
            preview_bytes = resp.read()
        preview_img = cv2.imdecode(np.frombuffer(preview_bytes, np.uint8), cv2.IMREAD_COLOR)
        assert preview_img is not None and preview_img.shape[1] == node.preview_width

        # 1. 页面修改标定板规格再切回默认 9x12 / 30mm / 22.5mm / DICT_5X5
        updated = _post(port, '/api/board', {
            'squares_x': 7, 'squares_y': 10, 'square_size_mm': 25.0,
            'marker_size_mm': 18.0, 'dictionary': 'DICT_4X4',
        })
        assert (updated['board']['squares_x'], updated['board']['squares_y']) == (7, 10)
        assert updated['board']['dictionary'] == 'DICT_4X4_50'

        restored = _post(port, '/api/board', {
            'squares_x': 9, 'squares_y': 12, 'square_size_mm': 30.0,
            'marker_size_mm': 22.5, 'dictionary': 'DICT_5X5',
        })
        assert (restored['board']['squares_x'], restored['board']['squares_y']) == (9, 12)
        assert restored['board']['dictionary'] == 'DICT_5X5_100'

        # 2. 写入内参并测试单臂手眼外参采集 -> 求解 -> 保存
        k_mat = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]])
        d_vec = np.zeros(5)
        size = (1280, 720)
        node.store.put_intrinsic('camera_arm_3', {
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
            node.frames.lookup = lambda *_a, _m=t_base_link, **_kw: _m
            scene, _ = render(board, flat, t_cam_board, k_mat, d_vec, size)
            det = node.board.detect(scene)
            assert det.count >= 6
            with node.gate._lock:
                node.gate._speed = 0.0
                node.gate._still_since = time.monotonic() - 1.0
                node.gate._stamp = time.monotonic()
                node.gate._joints = (0.0, 0.7, -1.2, 0.0, -0.3, 0.0)
            with node._live_lock:
                node._live['camera_arm_3'] = {
                    'seq': idx + 1, 'detection': det, 'image': scene, 'board': node.board,
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
        calib_data = node.store.read_calibration()
        assert 'camera_arm_3' in calib_data['extrinsics']
        assert calib_data['extrinsics']['camera_arm_3']['arm'] == 'arm_3'
        assert calib_data['extrinsics']['camera_arm_3']['parent'] == 'gripper_base_top'
        origins = calib_data['extrinsics']['camera_arm_3']['mesh_origins']
        assert origins['gripper_base_without_flange'][2] == pytest.approx(
            origins['gripper_base_top'][2] + GRIPPER_BASE_WITHOUT_FLANGE_TOP_Z_M, abs=1e-5)
        assert origins['gripper_base_with_flange'][2] == pytest.approx(
            origins['gripper_base_top'][2] + GRIPPER_BASE_WITH_FLANGE_TOP_Z_M, abs=1e-5)
        ref_out = _post(port, '/api/reference_point', {'variant': 'without_flange'})
        assert ref_out['link6_to_top_z_mm'] == pytest.approx(52.5)
    finally:
        if node is not None:
            node.destroy_node()
        if needs_init and rclpy.ok():
            rclpy.shutdown()
