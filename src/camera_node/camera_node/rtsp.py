"""腕部 IP 相机纯 Python RTSP 接口（不依赖 rclpy / ROS 运行时）。"""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import threading
import time
from typing import Any

import numpy as np
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


def default_cameras_config_path() -> Path:
    local = Path(__file__).resolve().parent.parent / 'config' / 'cameras.yaml'
    if local.is_file():
        return local
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('camera_node')) / 'config' / 'cameras.yaml'
        if share.is_file():
            return share
    except Exception:
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


def resolve_stream_url(
    camera: str,
    stream: str = 'stream1',
    config_file: str | Path | None = None,
) -> tuple[str, tuple[int, int] | None]:
    """把相机别名 / IP / rtsp:// 地址与码流档位解析为 (rtsp_url, 已知原生尺寸或 None)。"""
    raw = str(camera or '').strip()
    if not raw:
        raise ValueError('必须指定腕部相机（如 arm_3、arm_4、cam3、IP 或 rtsp:// 地址）')
    if raw.startswith(('rtsp://', 'rtsps://')):
        return raw, None
    cfg = resolve_arm_camera(raw, config_file=config_file)
    key = str(stream or '').strip().lower()
    if not key:
        url = cfg['rtsp_url']
        if url == cfg.get('stream1_url'):
            return url, (720, 480)
        if url == cfg.get('stream0_url'):
            return url, (1920, 1080)
        return url, None
    if key in ('stream1', '1'):
        return cfg['stream1_url'], (720, 480)
    if key in ('stream0', '0'):
        return cfg['stream0_url'], (1920, 1080)
    if f'{key}_url' in cfg:
        return cfg[f'{key}_url'], None
    raise ValueError(f"不支持的码流 {stream!r}，可选 'stream1' (720x480) 或 'stream0' (1920x1080)")


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


def probe_size(url: str) -> tuple[int, int] | None:
    """用 ffprobe 读原生分辨率，读不到返回 None。"""
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


def ffmpeg_command(url: str, fps: int = 0, scale: tuple[int, int] | None = None) -> list[str]:
    """限帧和缩放都交给 ffmpeg，管道上只走目标分辨率的裸 BGR。"""
    filters = []
    if fps > 0:
        filters.append(f'fps={fps}')
    if scale is not None:
        filters.append(f'scale={scale[0]}:{scale[1]}:flags=area')
    command = [
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin',
        '-rtsp_transport', 'tcp', '-flags', 'low_delay',
        '-probesize', '32768', '-analyzeduration', '0',
        '-an', '-i', url,
    ]
    if filters:
        command += ['-vf', ','.join(filters)]
    return command + ['-f', 'rawvideo', '-pix_fmt', 'bgr24', '-']


def fit_size(native: tuple[int, int], width: int, height: int) -> tuple[int, int]:
    """只给一边就按原生宽高比补另一边，补出来的边取偶数。"""
    if width > 0 and height > 0:
        return width, height
    if height > 0:
        return _even(native[0] * height / native[1]), height
    if width > 0:
        return width, _even(native[1] * width / native[0])
    return native


def _even(value: float) -> int:
    return max(2, round(value / 2) * 2)


def load_camera_info(path: str | Path, camera: str, width: int, height: int) -> dict[str, Any] | None:
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


def _decode_ppm_to_bgr(data: bytes) -> np.ndarray:
    idx = 0
    n = len(data)
    tokens: list[bytes] = []
    while len(tokens) < 4 and idx < n:
        while idx < n and data[idx] in b' \t\r\n':
            idx += 1
        if idx < n and data[idx] == ord('#'):
            while idx < n and data[idx] not in b'\r\n':
                idx += 1
            continue
        start = idx
        while idx < n and data[idx] not in b' \t\r\n#':
            idx += 1
        if idx > start:
            tokens.append(data[start:idx])
    if len(tokens) < 4 or tokens[0] != b'P6' or idx >= n or data[idx] not in b' \t\r\n':
        raise RuntimeError('ffmpeg 未返回有效的单帧图像数据')
    idx += 1
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    expected = width * height * 3
    if maxval != 255 or len(data) - idx < expected:
        raise RuntimeError(f'单帧图像尺寸或位深异常：{width}x{height} maxval={maxval}')
    rgb = np.frombuffer(data, dtype=np.uint8, offset=idx, count=expected).reshape(height, width, 3)
    return np.ascontiguousarray(rgb[:, :, ::-1])


def grab_frame(
    camera: str,
    *,
    stream: str = 'stream1',
    width: int = 0,
    height: int = 0,
    timeout: float = 10.0,
    config_file: str | Path | None = None,
) -> np.ndarray:
    """临时连接 RTSP 抓拍单帧并立即断开，平时零后台开销；返回 BGR uint8 (H, W, 3)。"""
    url, _ = resolve_stream_url(camera, stream=stream, config_file=config_file)
    command = [
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin',
        '-rtsp_transport', 'tcp', '-flags', 'low_delay',
        '-probesize', '32768', '-analyzeduration', '0',
        '-an', '-i', url,
    ]
    if width > 0 or height > 0:
        w_arg = width if width > 0 else -2
        h_arg = height if height > 0 else -2
        command += ['-vf', f'scale={w_arg}:{h_arg}:flags=area']
    command += ['-frames:v', '1', '-f', 'image2pipe', '-vcodec', 'ppm', '-']
    proc = subprocess.run(command, capture_output=True, timeout=timeout)
    if proc.returncode != 0:
        err = proc.stderr.decode('utf-8', errors='replace').strip()
        raise RuntimeError(f'RTSP 单帧抓拍失败（{url}）：{err or proc.returncode}')
    return _decode_ppm_to_bgr(proc.stdout)


class RtspReader:
    """一个 ffmpeg 子进程配一个读帧线程，逐帧调 on_frame(raw, width, height)。"""

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
        nbytes = self._frame_bytes
        buf = bytearray(nbytes)
        view = memoryview(buf)
        while True:
            offset = 0
            while offset < nbytes:
                read_n = stdout.readinto(view[offset:])
                if not read_n:
                    return
                offset += read_n
            self._stamp = time.monotonic()
            self._on_frame(bytes(buf), self.width, self.height)

    def stop(self):
        self._process.kill()
        try:
            self._process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            pass
        self._thread.join(timeout=3.0)


class WristCamera:
    """腕部相机 RTSP 热待机读帧器：后台持续抽帧覆盖最新画面，毫秒级返回 BGR uint8 (H, W, 3)。"""

    def __init__(
        self,
        camera: str,
        *,
        stream: str = 'stream1',
        fps: int = 0,
        width: int = 0,
        height: int = 0,
        stale_timeout_s: float = 5.0,
        config_file: str | Path | None = None,
        probe_fn=probe_size,
        reader_cls=RtspReader,
    ) -> None:
        self.url, native = resolve_stream_url(camera, stream=stream, config_file=config_file)
        self._reader_cls = reader_cls
        self._stale_timeout = stale_timeout_s
        if width > 0 and height > 0:
            size = scale = (width, height)
        else:
            if native is None:
                native = probe_fn(self.url)
            if native is None:
                raise RuntimeError(f'无法探测 RTSP 分辨率：{self.url}')
            size = fit_size(native, width, height)
            scale = None if size == native else size
        self.width, self.height = size
        self._command = ffmpeg_command(self.url, fps=fps, scale=scale)
        self._cond = threading.Condition()
        self._seq = 0
        self._raw: bytes | None = None
        self._frame_stamp = 0.0
        self._closed = False
        self._reader: RtspReader | None = None
        self.open()
        self._watchdog = threading.Thread(
            target=self._supervise, daemon=True, name='wrist-camera-watchdog')
        self._watchdog.start()

    def open(self) -> None:
        with self._cond:
            if self._closed:
                return
            if (self._reader is not None and self._reader.alive()
                    and (self._frame_stamp == 0.0
                         or time.monotonic() - self._frame_stamp <= self._stale_timeout)):
                return
            old = self._reader
            self._reader = self._reader_cls(self._command, self.width, self.height, self._on_frame)
        if old is not None:
            old.stop()

    def _supervise(self) -> None:
        while True:
            with self._cond:
                self._cond.wait(1.0)
                if self._closed:
                    return
                reader = self._reader
                stale = (self._frame_stamp > 0.0
                         and time.monotonic() - self._frame_stamp > self._stale_timeout)
            if reader is None or not reader.alive() or stale:
                with self._cond:
                    if self._closed:
                        return
                    old = self._reader
                    self._frame_stamp = 0.0
                    self._reader = self._reader_cls(
                        self._command, self.width, self.height, self._on_frame)
                if old is not None:
                    old.stop()

    def _on_frame(self, raw: bytes, _width: int, _height: int) -> None:
        with self._cond:
            self._seq += 1
            self._raw = raw
            self._frame_stamp = time.monotonic()
            self._cond.notify_all()

    def grab(self, timeout: float = 5.0, *, fresh: bool = True) -> np.ndarray:
        """从常驻流中截图，返回 BGR uint8 (H, W, 3)：
        - fresh=True（默认）：等待调用时刻之后到达的下一帧（30 fps 下约 16~33 ms），确保机械臂停稳后不读旧帧；
        - fresh=False：流内已有新鲜帧时立即返回（< 1 ms），刚启动尚未出帧时自动等到首帧。
        """
        if self._closed:
            raise RuntimeError('WristCamera 已关闭')
        if self._reader is None or not self._reader.alive():
            self.open()
        deadline = time.monotonic() + timeout
        for attempt in range(2):
            with self._cond:
                if (not fresh and self._raw is not None
                        and time.monotonic() - self._frame_stamp <= self._stale_timeout):
                    raw = self._raw
                    break
                start_seq = self._seq
                while self._seq <= start_seq:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError(f'等待腕部相机新帧超时（{timeout:.1f}s）：{self.url}')
                    if self._reader is not None and not self._reader.alive():
                        break
                    self._cond.wait(min(remaining, 0.2))
                if self._seq > start_seq and self._raw is not None:
                    raw = self._raw
                    break
            if attempt == 0 and time.monotonic() < deadline:
                self.open()
            else:
                raise TimeoutError(f'等待腕部相机新帧超时（{timeout:.1f}s）：{self.url}')
        return np.frombuffer(raw, dtype=np.uint8).reshape(self.height, self.width, 3).copy()

    def latest(self) -> np.ndarray | None:
        """立即返回内存里缓存的最新一帧 BGR 图像；尚未收到帧或流已超时断连时返回 None。"""
        with self._cond:
            if self._raw is None or time.monotonic() - self._frame_stamp > self._stale_timeout:
                return None
            raw = self._raw
        return np.frombuffer(raw, dtype=np.uint8).reshape(self.height, self.width, 3).copy()

    def close(self) -> None:
        with self._cond:
            self._closed = True
            reader = self._reader
            self._reader = None
            self._cond.notify_all()
        if reader is not None:
            reader.stop()
        self._watchdog.join(timeout=2.0)

    def __enter__(self) -> 'WristCamera':
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()
