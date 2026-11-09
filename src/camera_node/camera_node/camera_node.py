"""IP 相机 RTSP → sensor_msgs/Image。没人要图的时候不拉流。"""

import array
import json
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, Image
import yaml

ARM_CAMERA_PRESETS = {
    'arm_3': {
        'name': 'camera_arm_3',
        'arm': 'arm_3',
        'label': '机械臂 3 腕部相机',
        'ip': '192.168.123.97',
        'rtsp_url': 'rtsp://admin:123456@192.168.123.97/stream1',
        'stream0_url': 'rtsp://admin:123456@192.168.123.97/stream0',
        'stream1_url': 'rtsp://admin:123456@192.168.123.97/stream1',
        'image_topic': '/camera_arm_3/image_raw',
        'camera_info_topic': '/camera_arm_3/camera_info',
        'frame_id': 'camera_arm_3',
        'server_port': 0,
        'aliases': ('camera_arm_3', 'arm_3_camera', 'arm3_camera', 'camera_arm3',
                    'camera_3', 'cam3', 'arm_3', 'arm3', '/arm_3', '3', '192.168.123.97'),
    },
    'arm_4': {
        'name': 'camera_arm_4',
        'arm': 'arm_4',
        'label': '机械臂 4 腕部相机',
        'ip': '192.168.123.98',
        'rtsp_url': 'rtsp://admin:123456@192.168.123.98/stream1',
        'stream0_url': 'rtsp://admin:123456@192.168.123.98/stream0',
        'stream1_url': 'rtsp://admin:123456@192.168.123.98/stream1',
        'image_topic': '/camera_arm_4/image_raw',
        'camera_info_topic': '/camera_arm_4/camera_info',
        'frame_id': 'camera_arm_4',
        'server_port': 0,
        'aliases': ('camera_arm_4', 'arm_4_camera', 'arm4_camera', 'camera_arm4',
                    'camera_4', 'cam4', 'arm_4', 'arm4', '/arm_4', '4', '192.168.123.98'),
    },
}

_DEFAULTS = {
    'config_file': '',
    'arm': '',
    'camera': '',
    'rtsp_url': '',
    'image_topic': '~/image_raw',
    'camera_info_topic': '~/camera_info',
    'calib_file': '',
    'frame_id': '',
    'image_width': 0,
    'image_height': 0,
    'fps': 0,
    'server_port': 0,
    'jpeg_quality': 60,
    'poll_period_s': 1.0,
    'stale_timeout_s': 5.0,
}
_SIZE_PARAMS = ('fps', 'image_width', 'image_height')
_STREAM_PARAMS = ('rtsp_url',) + _SIZE_PARAMS


def default_cameras_config_path() -> Path:
    local = Path(__file__).resolve().parent.parent / 'config' / 'cameras.yaml'
    if local.is_file():
        return local
    try:
        share = Path(get_package_share_directory('camera_node')) / 'config' / 'cameras.yaml'
        if share.is_file():
            return share
    except PackageNotFoundError:
        pass
    return local


def load_cameras_config(config_file: str | Path | None = None) -> dict[str, dict[str, Any]]:
    """加载 cameras.yaml 中的相机与别名-IP 映射表。"""
    path = Path(config_file).expanduser() if config_file else default_cameras_config_path()
    if not path.is_file():
        return {v['name']: dict(v) for v in ARM_CAMERA_PRESETS.values()}
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    defaults = data.get('defaults') or {}
    user = defaults.get('rtsp_user', 'admin')
    password = defaults.get('rtsp_password', '123456')
    default_stream = defaults.get('default_stream', 'stream1')
    cameras: dict[str, dict[str, Any]] = {}
    for name, raw_cfg in (data.get('cameras') or {}).items():
        cfg = dict(raw_cfg or {})
        ip = str(cfg.get('ip', '')).strip()
        auth = f'{user}:{password}@' if user else ''
        cfg['name'] = name
        cfg.setdefault('arm', name)
        cfg.setdefault('label', name)
        cfg.setdefault('stream0_url', f'rtsp://{auth}{ip}/stream0' if ip else '')
        cfg.setdefault('stream1_url', f'rtsp://{auth}{ip}/stream1' if ip else '')
        cfg.setdefault('rtsp_url', f'rtsp://{auth}{ip}/{default_stream}' if ip else '')
        cfg.setdefault('image_topic', f'/{name}/image_raw')
        cfg.setdefault('camera_info_topic', f'/{name}/camera_info')
        cfg.setdefault('frame_id', name)
        cfg.setdefault('server_port', 0)
        aliases = [str(a).strip() for a in (cfg.get('aliases') or []) if str(a).strip()]
        for extra in (name, cfg['arm'], ip, cfg['frame_id']):
            if extra and extra not in aliases:
                aliases.append(extra)
        cfg['aliases'] = tuple(aliases)
        cameras[name] = cfg
    return cameras or {v['name']: dict(v) for v in ARM_CAMERA_PRESETS.values()}


def resolve_arm_camera(selector: str, config_file: str | Path | None = None) -> dict[str, Any]:
    """按机械臂名、相机别名或 IP 解析单个腕部相机配置（仅 arm_3 / arm_4 有腕部相机）。"""
    key = str(selector or '').strip().lower()
    if not key:
        raise ValueError('必须指定机械臂或相机别名（仅 arm_3 / arm_4 配置了腕部相机）')
    cameras = load_cameras_config(config_file)
    for cfg in cameras.values():
        alias_set = {a.lower() for a in cfg.get('aliases', ())}
        alias_set.update({cfg['name'].lower(), str(cfg.get('arm', '')).lower(), str(cfg.get('ip', '')).lower()})
        if key in alias_set or key.strip('/') in alias_set:
            return dict(cfg)
    raise ValueError(f'只有 arm_3 和 arm_4 配置了腕部相机，未找到匹配的相机或别名：{selector!r}')


def resolve_camera_selection(
    arm: str = '',
    camera: str = '',
    config_file: str | Path | None = None,
) -> list[dict[str, Any]]:
    """解析启动目标相机列表：
    - arm 与 camera 均为空，或为 all / both：同时返回 arm_3 与 arm_4 两台腕部相机；
    - 指定单个或逗号分隔的 arm / camera 别名或 IP：返回对应相机配置并校验一致性。
    """
    cameras = load_cameras_config(config_file)
    arm_str = str(arm or '').strip()
    cam_str = str(camera or '').strip()
    if (not arm_str and not cam_str) or arm_str.lower() in ('all', 'both', '*') or cam_str.lower() in ('all', 'both', '*'):
        return [dict(cfg) for cfg in cameras.values()]

    if arm_str and cam_str:
        by_arm = resolve_arm_camera(arm_str, config_file=config_file)
        by_cam = resolve_arm_camera(cam_str, config_file=config_file)
        if by_arm['name'] != by_cam['name']:
            raise ValueError(
                f"指定的 arm={arm_str!r} ({by_arm['name']}) 与 camera={cam_str!r} ({by_cam['name']}) 不匹配")
        return [by_cam]

    target = cam_str or arm_str
    tokens = [t.strip() for t in target.split(',') if t.strip()]
    selected: list[dict[str, Any]] = []
    seen: set[str] = set()
    for token in tokens:
        entry = resolve_arm_camera(token, config_file=config_file)
        if entry['name'] not in seen:
            seen.add(entry['name'])
            selected.append(entry)
    return selected


def _camera_candidate_keys(camera: str, config_file: str | Path | None = None) -> list[str]:
    keys = [camera]
    try:
        entry = resolve_arm_camera(camera, config_file=config_file)
        for alias in (entry['name'], entry.get('arm', ''), *entry.get('aliases', ())):
            if alias and alias not in keys:
                keys.append(alias)
    except ValueError:
        pass
    return keys


def probe_size(url):
    """用 ffprobe 读原生分辨率，读不到返回 None"""
    command = [
        'ffprobe', '-v', 'error', '-rtsp_transport', 'tcp',
        '-select_streams', 'v:0', '-show_entries', 'stream=width,height',
        '-of', 'json', url,
    ]
    try:
        probe = subprocess.run(
            command, capture_output=True, text=True, timeout=8.0)
    except (OSError, subprocess.SubprocessError):
        return None
    if probe.returncode != 0:
        return None
    try:
        stream = json.loads(probe.stdout)['streams'][0]
        return int(stream['width']), int(stream['height'])
    except (LookupError, TypeError, ValueError):
        return None


def ffmpeg_command(url, fps=0, scale=None):
    """限帧和缩放都交给 ffmpeg，管道上只走目标分辨率的裸 BGR"""
    filters = []
    if fps > 0:
        filters.append(f'fps={fps}')
    if scale is not None:
        filters.append(f'scale={scale[0]}:{scale[1]}:flags=area')
    command = [
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin',
        '-rtsp_transport', 'tcp', '-flags', 'low_delay',
        '-an', '-i', url,
    ]
    if filters:
        command += ['-vf', ','.join(filters)]
    return command + ['-f', 'rawvideo', '-pix_fmt', 'bgr24', '-']


def fit_size(native, width, height):
    """只给一边就按原生宽高比补另一边，补出来的边取偶数"""
    if width > 0 and height > 0:
        return width, height
    if height > 0:
        return _even(native[0] * height / native[1]), height
    if width > 0:
        return width, _even(native[1] * width / native[0])
    return native


def _even(value):
    return max(2, round(value / 2) * 2)


def load_camera_info(path, camera, width, height):
    """从 camera_calibration 写的 yaml 里挑出这个档位的内参，没有就返回 None。"""
    data = yaml.safe_load(Path(path).expanduser().read_text(encoding='utf-8')) or {}
    intrinsics = data.get('intrinsics', {}) or {}
    relations = data.get('profile_relations', {}) or {}
    candidates = _camera_candidate_keys(camera)
    for key in candidates:
        for entry in intrinsics.get(key, []):
            if (entry.get('width'), entry.get('height')) == (width, height):
                return entry
    for key in candidates:
        for relation in relations.get(key, []):
            if relation.get('kind') != 'scale' or tuple(relation.get('to', ())) != (width, height):
                continue
            source = tuple(relation.get('from', ()))
            for entry in intrinsics.get(key, []):
                if (entry.get('width'), entry.get('height')) != source:
                    continue
                matrix = list(entry['camera_matrix'])
                ratio_x, ratio_y = width / source[0], height / source[1]
                for index in range(3):
                    matrix[index] *= ratio_x
                    matrix[3 + index] *= ratio_y
                scaled = dict(entry)
                scaled.update({'width': width, 'height': height, 'camera_matrix': matrix,
                               'scaled_from': list(source)})
                return scaled
    return None


def camera_info_message(entry, frame_id):
    message = CameraInfo()
    message.header.frame_id = frame_id
    message.width = int(entry['width'])
    message.height = int(entry['height'])
    message.distortion_model = entry.get('distortion_model', 'plumb_bob')
    message.d = [float(v) for v in entry['distortion_coefficients']]
    matrix = [float(v) for v in entry['camera_matrix']]
    message.k = matrix
    message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    message.p = matrix[0:3] + [0.0] + matrix[3:6] + [0.0] + matrix[6:9] + [0.0]
    return message


class RtspReader:
    """一个 ffmpeg 子进程配一个读帧线程，逐帧调 on_frame(raw, width, height)"""

    def __init__(self, command, width, height, on_frame):
        self.width = width
        self.height = height
        self._frame_bytes = width * height * 3
        self._on_frame = on_frame
        self._stamp = time.monotonic()
        self._process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self._thread = threading.Thread(
            target=self._pump, daemon=True, name='rtsp-reader')
        self._thread.start()

    def frame_age(self):
        return time.monotonic() - self._stamp

    def alive(self):
        return self._process.poll() is None

    def _pump(self):
        stdout = self._process.stdout
        assert stdout is not None
        while True:
            raw = stdout.read(self._frame_bytes)
            if len(raw) != self._frame_bytes:
                return
            self._stamp = time.monotonic()
            self._on_frame(raw, self.width, self.height)

    def stop(self):
        self._process.kill()
        try:
            self._process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            pass
        self._thread.join(timeout=3.0)


class CameraNode(Node):
    """有 ROS 订阅者或网页在看才开 ffmpeg，都断开就把流关掉"""

    def __init__(self):
        super().__init__('camera_node')
        for name, default in _DEFAULTS.items():
            self.declare_parameter(name, default)
        cfg_file = str(self._setting('config_file') or '').strip() or None
        arm = str(self._setting('arm') or '').strip()
        camera = str(self._setting('camera') or '').strip()
        if arm or camera:
            selected = resolve_camera_selection(arm=arm, camera=camera, config_file=cfg_file)[0]
            if not self._setting('rtsp_url'):
                self.set_parameters([Parameter('rtsp_url', Parameter.Type.STRING, selected['rtsp_url'])])
            if not self._setting('frame_id'):
                self.set_parameters([Parameter('frame_id', Parameter.Type.STRING, selected['frame_id'])])
        if not self._setting('rtsp_url'):
            raise ValueError('rtsp_url 不能为空（或指定 arm / camera 别名）')
        self.add_on_set_parameters_callback(self._check_parameters)

        self._poll_period = self._setting('poll_period_s')
        self._stale_timeout = self._setting('stale_timeout_s')
        self._frame_id = self._setting('frame_id') or self.get_name()

        self._publisher = self.create_publisher(
            Image, self._setting('image_topic'), QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST, depth=1))
        self._info_publisher = self.create_publisher(
            CameraInfo, self._setting('camera_info_topic'), QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST, depth=1))
        self._camera_info = None
        self._calib_source = None

        self._frames = threading.Condition()
        self._frame = None
        self._seq = 0
        self._viewers = 0
        self._publish_frames = False
        self._reader = None
        self._applied = None
        self._probed = {}
        self._error = None
        self._shutdown = threading.Event()

        self._preview = None
        port = self._setting('server_port')
        if port > 0:
            from camera_node.preview import PreviewServer
            self._preview = PreviewServer(self, port)
            self.get_logger().info(f'预览页 http://0.0.0.0:{port}')

        self._supervisor = threading.Thread(
            target=self._supervise, daemon=True, name='camera-supervisor')
        self._supervisor.start()

    @property
    def stopping(self):
        return self._shutdown.is_set()

    @property
    def jpeg_quality(self):
        return self._setting('jpeg_quality')

    def add_viewer(self):
        with self._frames:
            self._viewers += 1

    def remove_viewer(self):
        with self._frames:
            self._viewers -= 1

    def wait_frame(self, seq, timeout):
        with self._frames:
            if self._frame is None or self._frame[0] <= seq:
                self._frames.wait(timeout)
            frame = self._frame
        if frame is None or frame[0] <= seq:
            return None
        return frame

    def status(self):
        reader = self._reader
        error = self._error
        if error is not None:
            return {'is_running': False, 'state': 'error', 'message': error}
        if reader is None:
            return {'is_running': True, 'state': 'idle',
                    'message': '空闲：没有订阅者，也没人在看'}
        return {
            'is_running': True, 'state': 'streaming',
            'message': (f'{reader.width}x{reader.height}，'
                        f'最近一帧 {reader.frame_age():.1f} s 前'),
        }

    def _on_frame(self, raw, width, height):
        if self._publish_frames:
            message = Image()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self._frame_id
            message.height = height
            message.width = width
            message.encoding = 'bgr8'
            message.step = width * 3
            message._data = array.array('B', raw)
            self._publisher.publish(message)
            if self._camera_info is not None:
                self._camera_info.header = message.header
                self._info_publisher.publish(self._camera_info)
        if self._preview is not None:
            with self._frames:
                self._seq += 1
                self._frame = (self._seq, raw, width, height)
                self._frames.notify_all()
        self._error = None

    def _supervise(self):
        while not self._shutdown.wait(self._poll_period):
            try:
                self._step()
            except Exception as error:
                self._error = str(error)
                self.get_logger().error(f'拉流管理异常：{error}')

    def _step(self):
        subscribers = self._publisher.get_subscription_count()
        self._publish_frames = subscribers > 0
        with self._frames:
            viewers = self._viewers
        reader = self._reader
        if subscribers == 0 and viewers == 0:
            if reader is not None:
                self._close_reader('没人要图，停止拉流')
            return
        if reader is None:
            self._open_reader()
        elif not reader.alive():
            self._close_reader('ffmpeg 已退出，重连')
            self._open_reader()
        elif reader.frame_age() > self._stale_timeout:
            self._close_reader(f'{self._stale_timeout:.0f} s 没有新帧，重连')
            self._open_reader()
        elif self._stream_settings() != self._applied:
            self._close_reader('参数改了，重开流')
            self._open_reader()
        elif self._setting('calib_file') != self._calib_source:
            self._load_camera_info((reader.width, reader.height))

    def _setting(self, name) -> Any:
        return self.get_parameter(name).value

    def _stream_settings(self):
        return tuple(self._setting(name) for name in _STREAM_PARAMS)

    def _check_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'rtsp_url' and not parameter.value:
                return SetParametersResult(
                    successful=False, reason='rtsp_url 不能为空')
            if parameter.name in _SIZE_PARAMS and parameter.value < 0:
                return SetParametersResult(
                    successful=False, reason=f'{parameter.name} 不能为负')
        return SetParametersResult(successful=True)

    def _native_size(self, url):
        size = self._probed.get(url)
        if size is None:
            size = probe_size(url)
            if size is not None:
                self._probed[url] = size
        return size

    def _open_reader(self):
        settings = self._stream_settings()
        url, fps, width, height = settings
        if width > 0 and height > 0:
            size = scale = (width, height)
        else:
            native = self._native_size(url)
            if native is None:
                self._fail(f'ffprobe 读不到分辨率：{url}')
                return
            size = fit_size(native, width, height)
            scale = None if size == native else size
        try:
            self._reader = RtspReader(
                ffmpeg_command(url, fps, scale),
                size[0], size[1], self._on_frame)
        except OSError as error:
            self._fail(f'启动 ffmpeg 失败：{error}')
            return
        self._applied = settings
        self._load_camera_info(size)
        self.get_logger().info(f'开始拉流 {size[0]}x{size[1]}')

    def _load_camera_info(self, size):
        self._camera_info = None
        path = self._setting('calib_file')
        self._calib_source = path
        if not path:
            return
        try:
            entry = load_camera_info(path, self.get_name(), size[0], size[1])
        except (OSError, ValueError, KeyError) as error:
            self._warn_once(f'读不了标定文件 {path}：{error}')
            return
        if entry is None:
            self._warn_once(
                f'{path} 里没有 {self.get_name()} 在 {size[0]}x{size[1]} 下的内参，'
                f'不发 camera_info')
            return
        self._camera_info = camera_info_message(entry, self._frame_id)
        note = f"（由 {entry['scaled_from']} 缩放）" if entry.get('scaled_from') else ''
        self.get_logger().info(
            f'加载内参 {size[0]}x{size[1]}{note} → {self._info_publisher.topic_name}')

    def _warn_once(self, message):
        if message != getattr(self, '_last_warning', None):
            self._last_warning = message
            self.get_logger().warn(message)

    def _close_reader(self, reason):
        reader = self._reader
        self._reader = None
        if reader is not None:
            self.get_logger().info(reason)
            reader.stop()

    def _fail(self, message):
        self._error = message
        self.get_logger().warn(message)

    def destroy_node(self):
        self._shutdown.set()
        self._supervisor.join(timeout=3.0)
        if self._preview is not None:
            self._preview.stop()
        self._close_reader('节点退出')
        with self._frames:
            self._frames.notify_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
