"""腕部 IP 相机纯 Python RTSP 取图与内外参/位姿读取库（不依赖 rclpy 或 cv2）。"""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import threading
import time
from typing import Any, Sequence

import numpy as np
import yaml

from wrist_camera.transforms import (
    GRIPPER_WITH_WRENCH_TOP_Z_M,
    load_arm_angle_offsets,
    matrix_from_dict,
    piper_fk,
)

ARM_CAMERA_PRESETS = {
    'arm_3': {
        'name': 'camera_arm_3',
        'arm': 'arm_3',
        'arm_namespace': '/arm_3',
        'label': '机械臂 3 腕部相机',
        'ip': '192.168.123.97',
        'rtsp_url': 'rtsp://admin:123456@192.168.123.97/stream1',
        'stream0_url': 'rtsp://admin:123456@192.168.123.97/stream0',
        'stream1_url': 'rtsp://admin:123456@192.168.123.97/stream1',
        'frame': 'camera_arm_3',
        'parent_frame': 'gripper_base_top',
        'base_frame': 'base_link',
        'link6_to_top_z_m': GRIPPER_WITH_WRENCH_TOP_Z_M,
        'gripper_variant': 'gripper_with_wrench',
        'joint_states_topic': '/arm_3/joint_states_feedback',
        'profiles': [
            {'name': 'stream1', 'width': 720, 'height': 480, 'url': 'rtsp://admin:123456@192.168.123.97/stream1'},
            {'name': 'stream0', 'width': 1920, 'height': 1080, 'url': 'rtsp://admin:123456@192.168.123.97/stream0'},
        ],
        'aliases': ('camera_arm_3', 'arm_3_camera', 'arm3_camera', 'camera_arm3',
                    'camera_3', 'cam3', 'arm_3', 'arm3', '/arm_3', '3', '192.168.123.97'),
    },
    'arm_4': {
        'name': 'camera_arm_4',
        'arm': 'arm_4',
        'arm_namespace': '/arm_4',
        'label': '机械臂 4 腕部相机',
        'ip': '192.168.123.98',
        'rtsp_url': 'rtsp://admin:123456@192.168.123.98/stream1',
        'stream0_url': 'rtsp://admin:123456@192.168.123.98/stream0',
        'stream1_url': 'rtsp://admin:123456@192.168.123.98/stream1',
        'frame': 'camera_arm_4',
        'parent_frame': 'gripper_base_top',
        'base_frame': 'base_link',
        'link6_to_top_z_m': GRIPPER_WITH_WRENCH_TOP_Z_M,
        'gripper_variant': 'gripper_with_wrench',
        'joint_states_topic': '/arm_4/joint_states_feedback',
        'profiles': [
            {'name': 'stream1', 'width': 720, 'height': 480, 'url': 'rtsp://admin:123456@192.168.123.98/stream1'},
            {'name': 'stream0', 'width': 1920, 'height': 1080, 'url': 'rtsp://admin:123456@192.168.123.98/stream0'},
        ],
        'aliases': ('camera_arm_4', 'arm_4_camera', 'arm4_camera', 'camera_arm4',
                    'camera_4', 'cam4', 'arm_4', 'arm4', '/arm_4', '4', '192.168.123.98'),
    },
}

_STREAM_SIZES = {
    'stream1': (720, 480),
    '1': (720, 480),
    'stream0': (1920, 1080),
    '0': (1920, 1080),
}


def _config_dir() -> Path:
    local = Path(__file__).resolve().parent.parent / 'config'
    if local.is_dir():
        return local
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('wrist_camera')) / 'config'
        if share.is_dir():
            return share
    except Exception:
        pass
    return local


def default_cameras_config_path() -> Path:
    return _config_dir() / 'cameras.yaml'


def default_calibration_path() -> Path:
    return _config_dir() / 'calibration.yaml'


def load_calibrations(calib_file: str | Path | None = None) -> dict[str, Any]:
    """读取 calibration.yaml 完整字典（含 intrinsics、extrinsics、profile_relations）。"""
    path = Path(calib_file).expanduser() if calib_file else default_calibration_path()
    if not path.is_file():
        return {'intrinsics': {}, 'extrinsics': {}, 'profile_relations': {}}
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    return {
        'intrinsics': dict(data.get('intrinsics') or {}),
        'extrinsics': dict(data.get('extrinsics') or {}),
        'profile_relations': dict(data.get('profile_relations') or {}),
    }


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
        cfg.setdefault('arm_namespace', f"/{str(cfg['arm']).lstrip('/')}")
        cfg.setdefault('label', name)
        cfg.setdefault('frame', name)
        cfg.setdefault('parent_frame', 'gripper_base_top')
        cfg.setdefault('base_frame', 'base_link')
        cfg.setdefault('link6_to_top_z_m', GRIPPER_WITH_WRENCH_TOP_Z_M)
        cfg.setdefault('stream0_url', f'rtsp://{auth}{ip}/stream0' if ip else '')
        cfg.setdefault('stream1_url', f'rtsp://{auth}{ip}/stream1' if ip else '')
        cfg.setdefault('rtsp_url', f'rtsp://{auth}{ip}/{default_stream}' if ip else '')
        if not cfg.get('profiles') and ip:
            cfg['profiles'] = [
                {'name': 'stream1', 'width': 720, 'height': 480, 'url': cfg['stream1_url']},
                {'name': 'stream0', 'width': 1920, 'height': 1080, 'url': cfg['stream0_url']},
            ]
        aliases = [str(a).strip() for a in (cfg.get('aliases') or []) if str(a).strip()]
        for extra in (name, cfg['arm'], ip, cfg['frame']):
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
    """解析目标相机列表（为空或 all/both 时同时返回 arm_3 与 arm_4；支持逗号分隔或单个别名）。"""
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
        tail = raw.rstrip('/').rsplit('/', 1)[-1].lower()
        return raw, _STREAM_SIZES.get(tail)
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


def _scale_intrinsic(entry: dict[str, Any], width: int, height: int) -> dict[str, Any]:
    src_w, src_h = int(entry['width']), int(entry['height'])
    ratio_x, ratio_y = width / float(src_w), height / float(src_h)
    matrix = [float(v) for v in entry['camera_matrix']]
    for index in range(3):
        matrix[index] *= ratio_x
        matrix[3 + index] *= ratio_y
    scaled = dict(entry)
    scaled.update({
        'width': int(width),
        'height': int(height),
        'camera_matrix': matrix,
        'scaled_from': [src_w, src_h],
    })
    return scaled


def get_intrinsics(
    camera: str,
    *,
    width: int = 0,
    height: int = 0,
    stream: str = '',
    calib_file: str | Path | None = None,
    config_file: str | Path | None = None,
    allow_scale: bool = True,
) -> dict[str, Any] | None:
    """读取指定腕部相机（支持 arm_3 / arm_4 / cam4 / IP 等任意别名）的内参字典。

    - 显式指定 width/height，或传 stream='stream1' (720x480) / 'stream0' (1920x1080) 匹配对应档位；
    - 均未指定时默认匹配 stream1 (720x480)，若不存在则返回最高分辨率已标定档位。
    """
    data = load_calibrations(calib_file)
    intrinsics = data['intrinsics']
    if not intrinsics:
        return None
    relations = data['profile_relations']
    candidates = _camera_candidate_keys(camera, config_file=config_file)

    if width <= 0 or height <= 0:
        if stream:
            key = stream.strip().lower()
            if key not in _STREAM_SIZES:
                raise ValueError(f"不支持的码流 {stream!r}，可选 'stream1' 或 'stream0'")
            width, height = _STREAM_SIZES[key]
        else:
            width, height = 720, 480

    for key in candidates:
        for entry in intrinsics.get(key, []):
            if (int(entry.get('width', 0)), int(entry.get('height', 0))) == (width, height):
                return dict(entry)

    if not allow_scale:
        return None

    for key in candidates:
        for relation in relations.get(key, []):
            if relation.get('kind') != 'scale' or tuple(relation.get('to', ())) != (width, height):
                continue
            source = tuple(relation.get('from', ()))
            for entry in intrinsics.get(key, []):
                if (entry.get('width'), entry.get('height')) == source:
                    return _scale_intrinsic(entry, width, height)

    for key in candidates:
        entries = intrinsics.get(key) or []
        if entries:
            best = max(entries, key=lambda e: int(e.get('width', 0)) * int(e.get('height', 0)))
            return _scale_intrinsic(best, width, height)
    return None


def load_camera_info(
    camera_or_path: str | Path,
    camera_or_width: str | int = 0,
    width_or_height: int = 0,
    height: int = 0,
    *,
    stream: str = '',
    calib_file: str | Path | None = None,
    allow_scale: bool = True,
) -> dict[str, Any] | None:
    """兼容免路径调用 load_camera_info('arm_4', 1920, 1080) 与旧签名 load_camera_info(path, 'arm_4', 1920, 1080)。"""
    if isinstance(camera_or_width, str):
        return get_intrinsics(
            camera_or_width,
            width=int(width_or_height),
            height=int(height),
            stream=stream,
            calib_file=camera_or_path,
            allow_scale=allow_scale,
        )
    return get_intrinsics(
        str(camera_or_path),
        width=int(camera_or_width),
        height=int(width_or_height),
        stream=stream,
        calib_file=calib_file,
        allow_scale=allow_scale,
    )


def get_extrinsic(
    camera: str,
    *,
    calib_file: str | Path | None = None,
    config_file: str | Path | None = None,
) -> dict[str, Any] | None:
    """读取指定腕部相机的手眼外参字典（gripper_base_top -> camera_optical），不存在返回 None。"""
    extrinsics = load_calibrations(calib_file)['extrinsics']
    for key in _camera_candidate_keys(camera, config_file=config_file):
        entry = extrinsics.get(key)
        if isinstance(entry, dict):
            return dict(entry)
    return None


load_camera_extrinsic = get_extrinsic


def extrinsic_matrix(
    camera: str,
    *,
    calib_file: str | Path | None = None,
    config_file: str | Path | None = None,
) -> np.ndarray:
    """返回 4x4 齐次矩阵 T_top_cam（夹爪最高点 gripper_base_top -> 相机光学系）。"""
    entry = get_extrinsic(camera, calib_file=calib_file, config_file=config_file)
    if entry is None:
        raise KeyError(f'calibration.yaml 中未找到相机 {camera!r} 的手眼外参')
    return matrix_from_dict(entry)


def camera_pose_in_base(
    camera: str,
    joint_positions: Sequence[float],
    *,
    angle_offsets: Sequence[float] | None = None,
    calib_file: str | Path | None = None,
    config_file: str | Path | None = None,
) -> np.ndarray:
    """由机械臂六轴关节角 (rad) 直接解算腕部相机在基座系 base_link 下的 4x4 位姿 T_base_cam。"""
    entry = get_extrinsic(camera, calib_file=calib_file, config_file=config_file)
    if entry is None:
        raise KeyError(f'calibration.yaml 中未找到相机 {camera!r} 的手眼外参')
    arm_id = str(entry.get('arm') or '')
    if not arm_id:
        arm_id = str(resolve_arm_camera(camera, config_file=config_file).get('arm', ''))
    offsets = tuple(angle_offsets) if angle_offsets is not None else load_arm_angle_offsets(arm_id)
    dz = float(entry.get('link6_to_top_z_m', GRIPPER_WITH_WRENCH_TOP_Z_M))
    t_base_top = piper_fk(joint_positions, offsets, tool_offset_z_m=dz)
    return t_base_top @ matrix_from_dict(entry)


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

    def __init__(self, command: list[str], width: int, height: int, on_frame) -> None:
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

    def frame_age(self) -> float:
        return time.monotonic() - self._stamp

    def alive(self) -> bool:
        return self._process.poll() is None

    def _pump(self) -> None:
        stdout = self._process.stdout
        assert stdout is not None
        while True:
            raw = stdout.read(self._frame_bytes)
            if len(raw) != self._frame_bytes:
                return
            self._stamp = time.monotonic()
            self._on_frame(raw, self.width, self.height)

    def stop(self) -> None:
        self._process.kill()
        try:
            self._process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            pass
        self._thread.join(timeout=3.0)


class WristCamera:
    """腕部相机 RTSP 常驻读帧器：后台持续抽帧覆盖最新画面，并可直接查询该相机内外参与位姿。"""

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
        self.camera = camera
        self._config_file = config_file
        self.url, native = resolve_stream_url(camera, stream=stream, config_file=config_file)
        self._reader_cls = reader_cls
        self._stale_timeout = stale_timeout_s
        if width > 0 and height > 0:
            size = (width, height)
            scale = None if native in (None, size) else size
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

    def intrinsics(
        self,
        *,
        calib_file: str | Path | None = None,
        allow_scale: bool = True,
    ) -> dict[str, Any] | None:
        """返回当前流分辨率 (self.width, self.height) 对应的内参字典。"""
        return get_intrinsics(
            self.camera,
            width=self.width,
            height=self.height,
            calib_file=calib_file,
            config_file=self._config_file,
            allow_scale=allow_scale,
        )

    def extrinsic(self, *, calib_file: str | Path | None = None) -> dict[str, Any] | None:
        """返回该相机的手眼外参字典（gripper_base_top -> camera_optical）。"""
        return get_extrinsic(self.camera, calib_file=calib_file, config_file=self._config_file)

    def extrinsic_matrix(self, *, calib_file: str | Path | None = None) -> np.ndarray:
        """返回 4x4 齐次变换矩阵 T_top_cam。"""
        return extrinsic_matrix(self.camera, calib_file=calib_file, config_file=self._config_file)

    def pose_in_base(
        self,
        joint_positions: Sequence[float],
        *,
        angle_offsets: Sequence[float] | None = None,
        calib_file: str | Path | None = None,
    ) -> np.ndarray:
        """由机械臂六轴关节角 (rad) 返回基座系下的相机实时 4x4 位姿 T_base_cam。"""
        return camera_pose_in_base(
            self.camera,
            joint_positions,
            angle_offsets=angle_offsets,
            calib_file=calib_file,
            config_file=self._config_file,
        )

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
