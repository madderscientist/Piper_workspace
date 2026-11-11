"""IP 相机 RTSP → sensor_msgs/Image。没人要图的时候不拉流。"""

import array
import threading
from pathlib import Path
from typing import Any

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, Image

from camera_node.rtsp import (
    ARM_CAMERA_PRESETS,
    RtspReader,
    default_cameras_config_path,
    ffmpeg_command,
    fit_size,
    load_camera_info,
    load_cameras_config,
    probe_size,
    resolve_arm_camera,
    resolve_camera_selection,
)

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
CYCLONEDDS_DEFAULT_URI = (
    '<CycloneDDS><Domain>'
    '<General>'
    '<FragmentSize>64500 B</FragmentSize>'
    '<MaxMessageSize>65500 B</MaxMessageSize>'
    '<MaxRexmitMessageSize>64500 B</MaxRexmitMessageSize>'
    '</General>'
    '<Internal>'
    '<Watermarks>'
    '<WhcLow>64 KiB</WhcLow><WhcHigh>8 MiB</WhcHigh>'
    '<WhcHighInit>4 MiB</WhcHighInit><WhcAdaptive>false</WhcAdaptive>'
    '</Watermarks>'
    '<BurstSize><MaxInitTransmit>131072</MaxInitTransmit><MaxRexmit>128 KiB</MaxRexmit></BurstSize>'
    '<DefragReliableMaxSamples>32</DefragReliableMaxSamples>'
    '<NackDelay>2 ms</NackDelay><PreEmptiveAckDelay>1 ms</PreEmptiveAckDelay>'
    '<AutoReschedNackDelay>10 ms</AutoReschedNackDelay>'
    '<HeartbeatInterval min="2 ms" minsched="2 ms">5 ms</HeartbeatInterval>'
    '</Internal>'
    '</Domain></CycloneDDS>'
)

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


class CameraNode(Node):
    """有 ROS 订阅者或网页在看才开 ffmpeg，都断开就把流关掉"""

    def __init__(self, *, node_name: str = 'camera_node', parameter_overrides=None):
        super().__init__(node_name, parameter_overrides=parameter_overrides)
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
        self._wake = threading.Event()
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
        self._local_sinks = []
        self._publish_frames = False
        self._reader = None
        self._applied = None
        self._probed = {}
        if arm or camera:
            if selected.get('stream1_url'):
                self._probed[selected['stream1_url']] = (720, 480)
            if selected.get('stream0_url'):
                self._probed[selected['stream0_url']] = (1920, 1080)
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
        self._wake.set()

    def remove_viewer(self):
        with self._frames:
            self._viewers -= 1
        self._wake.set()

    def add_local_sink(self, callback):
        with self._frames:
            self._local_sinks.append(callback)
        self._wake.set()

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
        for sink in self._local_sinks:
            sink(raw, width, height, self._camera_info)
        if self._preview is not None:
            with self._frames:
                self._seq += 1
                self._frame = (self._seq, raw, width, height)
                self._frames.notify_all()
        self._error = None

    def _supervise(self):
        while not self._shutdown.is_set():
            self._wake.wait(self._poll_period)
            self._wake.clear()
            if self._shutdown.is_set():
                break
            try:
                self._step()
            except Exception as error:
                self._error = str(error)
                self.get_logger().error(f'拉流管理异常：{error}')

    def _step(self):
        subscribers = self._publisher.get_subscription_count()
        local_sinks = len(self._local_sinks)
        self._publish_frames = subscribers > 0
        with self._frames:
            viewers = self._viewers
        reader = self._reader
        if subscribers == 0 and viewers == 0 and local_sinks == 0:
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
        if any(parameter.name in _STREAM_PARAMS or parameter.name == 'calib_file' for parameter in parameters):
            wake = getattr(self, '_wake', None)
            if wake is not None:
                wake.set()
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
        self._wake.set()
        self._supervisor.join(timeout=3.0)
        if self._preview is not None:
            self._preview.stop()
        self._close_reader('节点退出')
        with self._frames:
            self._frames.notify_all()
        super().destroy_node()


def main(args=None):
    import os
    os.environ.setdefault('CYCLONEDDS_URI', CYCLONEDDS_DEFAULT_URI)
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
