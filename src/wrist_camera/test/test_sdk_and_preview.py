import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from unittest.mock import patch
from urllib.request import urlopen

import numpy as np
import pytest

from wrist_camera import (
    WristCamera,
    camera_pose_in_base,
    extrinsic_matrix,
    get_extrinsic,
    get_intrinsics,
    grab_frame,
    load_camera_info,
    piper_fk,
    resolve_arm_camera,
    resolve_camera_selection,
    resolve_stream_url,
)
from wrist_camera.preview import PreviewServer


def test_pure_python_import_without_rclpy_or_cv2_and_ppm_decode():
    pkg_root = str(Path(__file__).resolve().parent.parent)
    env = {'PATH': os.environ.get('PATH', ''), 'PYTHONPATH': pkg_root}
    check = subprocess.run(
        [
            sys.executable, '-S', '-c',
            f'import site, sys; site.addsitedir({pkg_root!r}); '
            + ''.join(f'site.addsitedir({p!r}); ' for p in sys.path if 'site-packages' in p and '/opt/ros' not in p)
            + "import wrist_camera; assert 'rclpy' not in sys.modules and 'cv2' not in sys.modules",
        ],
        env=env, capture_output=True, text=True, timeout=5.0,
    )
    assert check.returncode == 0, check.stderr

    both = resolve_camera_selection()
    assert [c['name'] for c in both] == ['camera_arm_3', 'camera_arm_4']
    assert resolve_arm_camera('192.168.123.97')['name'] == 'camera_arm_3'
    assert resolve_stream_url('cam4', 'stream0') == (
        'rtsp://admin:123456@192.168.123.98/stream0', (1920, 1080),
    )

    ppm = b'P6\n# comment\n2 1\n255\n' + bytes([10, 20, 30, 40, 50, 60])
    completed = subprocess.CompletedProcess(args=['ffmpeg'], returncode=0, stdout=ppm, stderr=b'')
    with patch('wrist_camera.rtsp.subprocess.run', return_value=completed) as run_mock:
        bgr = grab_frame('arm_3', stream='stream0', width=640)
    assert bgr.shape == (1, 2, 3) and bgr.dtype == np.uint8
    assert bgr[0, 0].tolist() == [30, 20, 10]
    assert bgr[0, 1].tolist() == [60, 50, 40]
    assert 'scale=640:-2:flags=area' in run_mock.call_args.args[0]


def test_intrinsics_extrinsics_pose_and_wrist_camera_reader():
    intr_1080 = get_intrinsics('arm_4', stream='stream0')
    intr_480 = load_camera_info('cam4', 720, 480)
    assert intr_1080 is not None and (intr_1080['width'], intr_1080['height']) == (1920, 1080)
    assert intr_480 is not None and (intr_480['width'], intr_480['height']) == (720, 480)

    ext = get_extrinsic('arm_4')
    assert ext is not None and ext['parent'] == 'gripper_base_top'
    t_top_cam = extrinsic_matrix('arm_4')
    assert t_top_cam.shape == (4, 4)

    q = (0.1, 0.6, -1.1, 0.2, -0.4, 0.3)
    zero_offsets = (0.0,) * 6
    t_base_cam = camera_pose_in_base('arm_4', q, angle_offsets=zero_offsets)
    expected = piper_fk(q, zero_offsets, tool_offset_z_m=ext['link6_to_top_z_m']) @ t_top_cam
    assert np.allclose(t_base_cam, expected, atol=1e-12)

    holders = {}

    class FakeReader:
        def __init__(self, command, width, height, on_frame):
            holders['on_frame'] = on_frame
            self.stopped = False

        def alive(self):
            return not self.stopped

        def stop(self):
            self.stopped = True

    with WristCamera('arm_4', stream='stream1', reader_cls=FakeReader) as cam:
        assert cam.intrinsics()['width'] == 720
        assert cam.extrinsic()['child'] == 'camera_arm_4'
        assert np.allclose(cam.pose_in_base(q, angle_offsets=zero_offsets), expected, atol=1e-12)

        holders['on_frame'](bytes([1, 2, 3]) * (720 * 480), 720, 480)
        assert cam.grab(fresh=False)[0, 0].tolist() == [1, 2, 3]
        threading.Timer(
            0.03, lambda: holders['on_frame'](bytes([9, 8, 7]) * (720 * 480), 720, 480),
        ).start()
        assert cam.grab(timeout=1.0)[0, 0].tolist() == [9, 8, 7]


def test_preview_server_streams_on_demand_and_stops_when_idle():
    events = {'opened': 0, 'stopped': 0}

    class FakePreviewReader:
        def __init__(self, command, width, height, on_frame):
            events['opened'] += 1
            self.width = width
            self.height = height
            self._alive = True
            on_frame(bytes([20, 40, 60]) * (width * height), width, height)

        def alive(self):
            return self._alive

        def frame_age(self):
            return 0.0

        def stop(self):
            self._alive = False
            events['stopped'] += 1

    srv = PreviewServer(0, host='127.0.0.1', arm='arm_4', reader_cls=FakePreviewReader)
    port = srv.server_port
    try:
        with urlopen(f'http://127.0.0.1:{port}/status', timeout=2.0) as resp:
            st = json.loads(resp.read().decode('utf-8'))
        assert st['cameras'][0]['state'] == 'idle'
        assert events['opened'] == 0

        with urlopen(f'http://127.0.0.1:{port}/video_feed?camera=arm_4&stream=stream1', timeout=3.0) as feed:
            chunk = feed.read(512)
        assert b'--frame' in chunk and b'image/jpeg' in chunk
        assert events['opened'] >= 1

        deadline = time.monotonic() + 2.0
        while events['stopped'] < events['opened'] and time.monotonic() < deadline:
            time.sleep(0.05)
        assert events['stopped'] == events['opened']
    finally:
        srv.stop()
