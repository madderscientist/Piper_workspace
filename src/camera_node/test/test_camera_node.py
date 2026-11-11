"""不依赖 ROS 运行时和相机的部分：机械臂相机预设、ffmpeg 命令拼装、按需拉流判据、读帧线程、内参加载。"""

import array
import os
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
from builtin_interfaces.msg import Time
from rclpy.parameter import Parameter

from camera_node import WristCamera, grab_frame, resolve_stream_url
from camera_node.camera_node import (
    ARM_CAMERA_PRESETS,
    CameraNode,
    RtspReader,
    camera_info_message,
    ffmpeg_command,
    fit_size,
    load_camera_info,
    load_cameras_config,
    resolve_arm_camera,
    resolve_camera_selection,
)

_BOUND = ('_step', '_open_reader', '_close_reader', '_fail', '_on_frame',
          '_setting', '_stream_settings', '_native_size', '_check_parameters',
          '_load_camera_info', '_warn_once', 'add_local_sink')
_DEFAULTS = {'arm': 'arm_3', 'rtsp_url': 'rtsp://admin:123456@192.168.123.97/stream1', 'fps': 15,
             'image_width': 0, 'image_height': 0, 'jpeg_quality': 60,
             'calib_file': ''}


def _node(subscribers=0, viewers=0, reader=None, **overrides):
    params = dict(_DEFAULTS, **overrides)
    clock = MagicMock()
    clock.return_value.now.return_value.to_msg.return_value = Time()
    node = SimpleNamespace(
        params=params,
        get_parameter=lambda name: SimpleNamespace(value=params[name]),
        _publisher=SimpleNamespace(
            get_subscription_count=MagicMock(return_value=subscribers),
            publish=MagicMock()),
        _frames=threading.Condition(),
        _viewers=viewers,
        _local_sinks=[],
        _wake=threading.Event(),
        _publish_frames=False,
        _preview=None,
        _reader=reader,
        _applied=None,
        _probed={},
        _error=None,
        _seq=0,
        _frame=None,
        _stale_timeout=5.0,
        _frame_id='camera_arm_3',
        _camera_info=None,
        _calib_source='',
        _info_publisher=SimpleNamespace(
            publish=MagicMock(), topic_name='/camera_arm_3/camera_info'),
        get_clock=clock,
        get_logger=MagicMock(return_value=MagicMock()),
    )
    node.get_name = lambda: 'camera_arm_3'
    for name in _BOUND:
        setattr(node, name, MethodType(getattr(CameraNode, name), node))
    return node


def _healthy(reader):
    reader.alive.return_value = True
    reader.frame_age.return_value = 0.0
    return reader


class ArmPresetTest(unittest.TestCase):
    def test_config_alias_ip_mapping_and_single_or_dual_selection(self) -> None:
        cameras = load_cameras_config()
        self.assertEqual(set(cameras), {'camera_arm_3', 'camera_arm_4'})

        # 默认不指定或指定 all 时同时返回两个臂的相机
        both = resolve_camera_selection()
        self.assertEqual([c['name'] for c in both], ['camera_arm_3', 'camera_arm_4'])
        both_all = resolve_camera_selection(arm='all')
        self.assertEqual([c['name'] for c in both_all], ['camera_arm_3', 'camera_arm_4'])

        # 按 arm、别名或 IP 单独选择一台相机
        p3 = resolve_camera_selection(arm='arm_3')
        self.assertEqual(len(p3), 1)
        self.assertEqual(p3[0]['ip'], '192.168.123.97')
        self.assertEqual(p3[0]['name'], 'camera_arm_3')

        p4_alias = resolve_camera_selection(camera='cam4')
        self.assertEqual(len(p4_alias), 1)
        self.assertEqual(p4_alias[0]['ip'], '192.168.123.98')
        self.assertEqual(p4_alias[0]['name'], 'camera_arm_4')

        p3_ip = resolve_arm_camera('192.168.123.97')
        self.assertEqual(p3_ip['name'], 'camera_arm_3')

        for invalid in ('arm_1', 'arm_2', 'arm1', 'arm2', ''):
            with self.assertRaises(ValueError):
                resolve_arm_camera(invalid)
        with self.assertRaises(ValueError):
            resolve_camera_selection(arm='arm_3', camera='cam4')


class FitSizeTest(unittest.TestCase):
    def test_aspect_ratio_and_explicit_dimensions(self) -> None:
        self.assertEqual(fit_size((640, 360), 0, 240), (426, 240))
        self.assertEqual(fit_size((640, 360), 320, 0), (320, 180))
        self.assertEqual(fit_size((640, 360), 320, 240), (320, 240))
        self.assertEqual(fit_size((640, 360), 0, 0), (640, 360))


class FfmpegCommandTest(unittest.TestCase):
    def test_low_delay_and_scale_filter(self) -> None:
        cmd = ffmpeg_command('rtsp://camera/stream1')
        self.assertIn('low_delay', cmd)
        self.assertNotIn('nobuffer', cmd)
        self.assertNotIn('-vf', cmd)

        cmd_scaled = ffmpeg_command('rtsp://camera/stream0', fps=15, scale=(640, 360))
        self.assertEqual(cmd_scaled[cmd_scaled.index('-vf') + 1], 'fps=15,scale=640:360:flags=area')


class DemandGateAndReconfigureTest(unittest.TestCase):
    def test_demand_gate_and_runtime_reconfigure(self) -> None:
        node = _node()
        with patch('camera_node.camera_node.RtspReader') as reader_type:
            node._step()
        reader_type.assert_not_called()

        node = _node(subscribers=1, image_width=640, image_height=360)
        with patch('camera_node.camera_node.RtspReader') as reader_type:
            _healthy(reader_type.return_value)
            node._step()
            self.assertTrue(node._publish_frames)
            first = node._reader
            node.params['image_height'] = 240
            node.params['image_width'] = 0
            with patch('camera_node.camera_node.probe_size', return_value=(640, 360)):
                node._step()
        first.stop.assert_called_once_with()
        self.assertEqual(reader_type.call_args.args[1:3], (426, 240))

    def test_parameter_validation(self) -> None:
        node = _node()
        self.assertFalse(node._check_parameters(
            [Parameter('image_height', Parameter.Type.INTEGER, -1)]).successful)
        self.assertFalse(node._check_parameters(
            [Parameter('rtsp_url', Parameter.Type.STRING, '')]).successful)
        self.assertTrue(node._check_parameters(
            [Parameter('image_height', Parameter.Type.INTEGER, 240)]).successful)
        self.assertTrue(node._wake.is_set())

    def test_local_sink_keeps_stream_alive_and_forwards_frames(self) -> None:
        node = _node(subscribers=0, viewers=0)
        received = []
        node.add_local_sink(lambda raw, w, h, info: received.append((raw, w, h, info)))
        with patch('camera_node.camera_node.RtspReader') as reader_type:
            _healthy(reader_type.return_value)
            with patch('camera_node.camera_node.probe_size', return_value=(720, 480)):
                node._step()
        reader_type.assert_called_once()
        self.assertFalse(node._publish_frames)
        node._on_frame(bytes(12), 2, 2)
        node._publisher.publish.assert_not_called()
        self.assertEqual(received, [(bytes(12), 2, 2, None)])


class RtspReaderAndPublishTest(unittest.TestCase):
    def test_reads_whole_frames_and_publishes_array_payload(self) -> None:
        frames = []
        command = ['python3', '-c', 'import sys; sys.stdout.buffer.write(bytes([1]) * 15)']
        reader = RtspReader(command, 2, 2, lambda raw, w, h: frames.append(raw))
        deadline = time.monotonic() + 5.0
        while reader.alive() and time.monotonic() < deadline:
            time.sleep(0.02)
        reader.stop()
        self.assertEqual(frames, [bytes([1]) * 12])

        node = _node()
        node._publish_frames = True
        node._on_frame(bytes(12), 2, 2)
        message = node._publisher.publish.call_args.args[0]
        self.assertIsInstance(message.data, array.array)
        self.assertEqual(message.header.frame_id, 'camera_arm_3')


_CALIB = """
version: 1
intrinsics:
  camera_arm_3:
    - width: 1920
      height: 1080
      camera_matrix: [1500.0, 0.0, 960.0, 0.0, 1502.0, 540.0, 0.0, 0.0, 1.0]
      distortion_model: plumb_bob
      distortion_coefficients: [-0.31, 0.09, 0.001, -0.002, 0.0]
profile_relations:
  camera_arm_3:
    - {from: [1920, 1080], to: [640, 360], kind: scale}
    - {from: [1920, 1080], to: [1440, 1080], kind: crop}
"""


class CameraInfoTest(unittest.TestCase):
    def setUp(self) -> None:
        self.path = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
        self.path.write(_CALIB)
        self.path.close()
        self.addCleanup(os.unlink, self.path.name)

    def test_exact_alias_and_scale_profile_lookup(self) -> None:
        entry = load_camera_info(self.path.name, 'camera_arm_3', 1920, 1080)
        assert entry is not None
        self.assertEqual(entry['camera_matrix'][0], 1500.0)

        alias_entry = load_camera_info(self.path.name, 'arm_3_camera', 640, 360)
        assert alias_entry is not None
        self.assertAlmostEqual(alias_entry['camera_matrix'][0], 500.0)
        self.assertIsNone(load_camera_info(self.path.name, 'camera_arm_3', 1440, 1080))

        msg = camera_info_message(entry, 'camera_arm_3')
        self.assertEqual(msg.header.frame_id, 'camera_arm_3')
        self.assertEqual(len(msg.p), 12)


class DirectRtspApiTest(unittest.TestCase):
    def test_pure_python_import_without_rclpy_and_one_shot_ppm_decode(self) -> None:
        pkg_root = str(Path(__file__).resolve().parent.parent)
        env = {'PATH': os.environ.get('PATH', ''), 'PYTHONPATH': pkg_root}
        check = subprocess.run(
            [sys.executable, '-S', '-c',
             f"import site, sys; site.addsitedir({pkg_root!r}); "
             + ''.join(f'site.addsitedir({p!r}); ' for p in sys.path if 'site-packages' in p and '/opt/ros' not in p)
             + "import camera_node; assert 'rclpy' not in sys.modules"],
            env=env, capture_output=True, text=True, timeout=5.0,
        )
        self.assertEqual(check.returncode, 0, check.stderr)

        self.assertEqual(
            resolve_stream_url('arm_3', 'stream1'),
            ('rtsp://admin:123456@192.168.123.97/stream1', (720, 480)),
        )
        self.assertEqual(
            resolve_stream_url('cam4', 'stream0'),
            ('rtsp://admin:123456@192.168.123.98/stream0', (1920, 1080)),
        )

        # PPM 为 RGB 排列，grab_frame 应解码并反转为 BGR uint8 (H, W, 3)
        ppm = b'P6\n# comment\n2 1\n255\n' + bytes([10, 20, 30, 40, 50, 60])
        completed = subprocess.CompletedProcess(args=['ffmpeg'], returncode=0, stdout=ppm, stderr=b'')
        with patch('camera_node.rtsp.subprocess.run', return_value=completed) as run_mock:
            bgr = grab_frame('arm_3', stream='stream0', width=640)
        self.assertEqual(bgr.shape, (1, 2, 3))
        self.assertEqual(bgr.dtype, np.uint8)
        self.assertEqual(bgr[0, 0].tolist(), [30, 20, 10])
        self.assertEqual(bgr[0, 1].tolist(), [60, 50, 40])
        self.assertIn('scale=640:-2:flags=area', run_mock.call_args.args[0])

    def test_wrist_camera_waits_for_post_call_frame(self) -> None:
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
            self.assertIsNone(cam.latest())
            # 先推一帧代表机械臂移动前的旧缓存
            holders['on_frame'](bytes([1, 2, 3]) * (720 * 480), 720, 480)
            self.assertEqual(cam.latest()[0, 0].tolist(), [1, 2, 3])
            self.assertEqual(cam.grab(fresh=False)[0, 0].tolist(), [1, 2, 3])

            # grab() 必须等待调用之后新到达的第二帧
            threading.Timer(
                0.03,
                lambda: holders['on_frame'](bytes([9, 8, 7]) * (720 * 480), 720, 480),
            ).start()
            fresh = cam.grab(timeout=1.0)
            self.assertEqual(fresh.shape, (480, 720, 3))
            self.assertEqual(fresh[0, 0].tolist(), [9, 8, 7])

            # 超时断流后不应把旧缓存当成新鲜画面返回
            cam._frame_stamp -= 10.0
            self.assertIsNone(cam.latest())
        with self.assertRaises(RuntimeError):
            cam.grab()


if __name__ == '__main__':
    unittest.main()
