"""ROS 2 + HTTP server for the 4-stage Agent-guided Insertion Pipeline."""

from __future__ import annotations

import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading
import time
from typing import Any, Sequence
from urllib.error import URLError
from urllib.parse import parse_qs, urlencode, urlsplit
from urllib.request import urlopen

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import cv2
import numpy as np
from piper_x_driver.configuration import default_config_path
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.utilities import remove_ros_args
from wrist_camera import (
    WristCamera,
    camera_pose_in_base,
    get_extrinsic,
    get_intrinsics,
    grab_frame,
    resolve_arm_camera,
)
import yaml

from ..policy import (
    DEFAULT_TIP_LENGTH_M,
    POLICY_PERIOD_S,
    GoalPose,
    canonical_quat,
    forward_tip_pose,
    normalize_tip_offset,
    quat_inv,
    quat_mul,
    quat_rotvec,
    rotvec_quat,
    solve_tip_ik,
)
from ..web import AUTO_MIT_GAINS, WRENCH_SOURCES, InsertDashboardNode
from .geometry import (
    BoardSpec,
    calibrate_external_camera,
    compute_alignment_plan,
    detect_charuco_pose,
    infer_gripper_tip_offset,
    load_extrinsics_file,
    pixel_to_base_ray,
    render_gpt_crop_overlay,
    render_gpt_output_overlay,
    render_perception_overlay,
    save_extrinsics_file,
    solve_axis_from_two_views,
    triangulate_two_rays,
)
from .vision_agent import (
    DEFAULT_API_BASE,
    DEFAULT_MODEL,
    DEFAULT_OBJECT_PROMPT,
    DEFAULT_OUTPUT_PROMPT,
    DEFAULT_REASONING_EFFORT,
    PerceptionCancelledError,
    perceive_dual_views,
)


def agent_web_root_path() -> Path:
    local = Path(__file__).resolve().parent.parent.parent / 'agent_web'
    if (local / 'index.html').is_file():
        return local
    try:
        share = Path(get_package_share_directory('insert')) / 'agent_web'
        if (share / 'index.html').is_file():
            return share
    except PackageNotFoundError:
        pass
    return local


def default_extrinsics_path() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / 'src' / 'insert' / 'config' / 'extrinsics.yaml'
        if cand.parent.parent.is_dir():
            return cand
    local = here.parent.parent.parent / 'config' / 'extrinsics.yaml'
    if (here.parent.parent.parent / 'setup.py').is_file():
        return local
    try:
        return Path(get_package_share_directory('insert')) / 'config' / 'extrinsics.yaml'
    except PackageNotFoundError:
        return local


def default_outputs_dir() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        src_pkg = parent / 'src' / 'insert'
        if src_pkg.is_dir():
            return src_pkg / 'outputs'
    return here.parent.parent.parent / 'outputs'


def _find_head_intrinsic_yaml() -> Path:
    candidates = [
        Path('/home/a/piper_x_ws/src/head_camera/config/intrinsic.yaml'),
    ]
    for parent in Path(__file__).resolve().parents:
        cand = parent / 'src' / 'head_camera' / 'config' / 'intrinsic.yaml'
        if cand not in candidates:
            candidates.append(cand)
    try:
        share = Path(get_package_share_directory('head_camera')) / 'config' / 'intrinsic.yaml'
        candidates.append(share)
    except Exception:
        pass
    for path in candidates:
        if path.is_file():
            return path
    return candidates[0]


def _encode_jpeg(image_bgr: np.ndarray, quality: int = 88) -> bytes:
    ok, buf = cv2.imencode('.jpg', image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError('JPEG encode failed')
    return buf.tobytes()


class AgentInsertNode(InsertDashboardNode):
    """Full-pipeline node: Float drag, 3-camera ChArUco calibration, GPT-6.1-sol perception, pre-move, and insert."""

    def __init__(
        self,
        *,
        config_file: str | Path | None = None,
        arm_selector: str = 'can4',
        namespace_override: str = '',
        onnx_path: str | Path | None = None,
        tip_length_m: float = DEFAULT_TIP_LENGTH_M,
        tip_offset_m: Sequence[float] | None = None,
        wrench_source: str = 'paxini',
        head_camera_url: str = 'http://127.0.0.1:8310',
        llm_api_base: str = DEFAULT_API_BASE,
        llm_model: str = DEFAULT_MODEL,
        llm_reasoning_effort: str = DEFAULT_REASONING_EFFORT,
        extrinsics_path: str | Path | None = None,
    ) -> None:
        super().__init__(
            config_file=config_file,
            arm_selector=arm_selector,
            namespace_override=namespace_override,
            onnx_path=onnx_path,
            tip_length_m=tip_length_m,
            tip_offset_m=tip_offset_m,
            wrench_source=wrench_source,
        )
        # Keep the arm in Float (zero-force drag) mode right from startup once enabled.
        self.mode = 'drag'
        self.status_message = '就绪：机械臂默认保持 Float（零力拖动），请按四步流程依次执行标定、识别、预移与插入'

        self.head_camera_url = head_camera_url.rstrip('/')
        self.llm_api_base = llm_api_base.rstrip('/')
        self.llm_model = llm_model
        self.llm_reasoning_effort = llm_reasoning_effort
        self.extrinsics_path = Path(extrinsics_path) if extrinsics_path else default_extrinsics_path()
        self.board_spec = BoardSpec()
        self.stop_event = threading.Event()

        self.extrinsics: dict[str, dict[str, Any]] = load_extrinsics_file(self.extrinsics_path)
        self._calib_samples: dict[str, list[dict[str, Any]]] = {'head_1': [], 'head_2': []}
        self._last_calib_report: dict[str, Any] | None = None
        self._perception_result: dict[str, Any] | None = None
        self._annotated_jpegs: dict[str, tuple[bytes, float]] = {}
        self._job_cancel = threading.Event()
        self._job_state: dict[str, Any] = {
            'running': False,
            'stage': 'idle',
            'reasoning_effort': self.llm_reasoning_effort,
            'reasoning_steps': 0,
            'started_at': 0.0,
            'elapsed_s': 0.0,
            'partial_output': '',
            'error': '',
        }

        # Trajectory state for Step 3 smooth move to 0.5 cm pre-insert pose
        self._move_start_time: float = 0.0
        self._move_duration_s: float = 2.0
        self._move_p_start: np.ndarray | None = None
        self._move_q_start: np.ndarray | None = None
        self._move_p_target: np.ndarray | None = None
        self._move_q_target: np.ndarray | None = None
        self._move_prev_joints: tuple[float, ...] | None = None
        self._move_progress: float = 0.0

        # Optional auto-hold when stationary after crossing hole entrance
        self.auto_stop_stationary: bool = False
        self._tip_trace: deque[tuple[float, np.ndarray]] = deque(maxlen=60)

        # Direct RTSP wrist camera reader (no ROS camera_node required)
        self._wrist_rtsp = None
        self._wrist_rtsp_lock = threading.Lock()
        self.wrist_preview_enabled: bool = True
        if self.extrinsics:
            loaded_keys = [k for k in ('head_1', 'head_2') if k in self.extrinsics]
            if loaded_keys:
                self._log('INFO', f'已加载外部相机外参 ({", ".join(loaded_keys)})：{self.extrinsics_path.name}')

    def shutdown_streams(self) -> None:
        self.stop_event.set()
        self._job_cancel.set()
        with self._wrist_rtsp_lock:
            rtsp = self._wrist_rtsp
            self._wrist_rtsp = None
        if rtsp is not None:
            try:
                rtsp.close()
            except BaseException:
                pass

    def set_wrist_preview(self, enabled: bool) -> dict[str, Any]:
        self.wrist_preview_enabled = bool(enabled)
        if not self.wrist_preview_enabled:
            with self._wrist_rtsp_lock:
                rtsp = self._wrist_rtsp
                self._wrist_rtsp = None
            if rtsp is not None:
                try:
                    rtsp.close()
                except Exception:
                    pass
        msg = '已开启腕部相机实时预览' if self.wrist_preview_enabled else '已暂停腕部相机实时预览'
        self._log('INFO', msg)
        return {'ok': True, 'wrist_preview_enabled': self.wrist_preview_enabled, 'message': msg}

    # ------------------------------------------------------------------
    # Camera & Intrinsics Accessors
    # ------------------------------------------------------------------

    def query_head_cameras_status(self) -> dict[str, Any]:
        try:
            with urlopen(f'{self.head_camera_url}/api/status', timeout=1.2) as resp:
                data = json.loads(resp.read().decode('utf-8'))
            cams = data.get('cameras') or []
            return {'online': True, 'cameras': cams, 'raw': data}
        except Exception as exc:
            return {'online': False, 'cameras': [], 'error': str(exc)}

    def grab_head_bgr(self, camera: str, *, fresh: bool = True, timeout: float = 3.5) -> np.ndarray:
        qs = urlencode({'camera': camera, **({'fresh': '1'} if fresh else {})})
        url = f'{self.head_camera_url}/frame/color.jpg?{qs}'
        try:
            with urlopen(url, timeout=timeout) as resp:
                raw = resp.read()
        except URLError as exc:
            raise RuntimeError(f'无法从 head_camera 拉取 {camera} 图像 ({url}): {exc.reason}') from exc
        arr = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
        if arr is None:
            raise RuntimeError(f'解码 {camera} JPEG 图像失败')
        return arr

    def get_head_intrinsic(self, camera: str) -> dict[str, Any]:
        url = f'{self.head_camera_url}/api/intrinsic?{urlencode({"camera": camera})}'
        try:
            with urlopen(url, timeout=2.0) as resp:
                info = json.loads(resp.read().decode('utf-8'))
            if info.get('available'):
                return info
        except Exception:
            pass
        path = _find_head_intrinsic_yaml()
        if path.is_file():
            raw = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
            cams = raw.get('cameras') or {}
            if camera in cams:
                prof = (cams[camera].get('profiles') or {}).get('1920x1080')
                if prof:
                    return {'available': True, 'camera_key': camera, **prof}
        raise RuntimeError(f'外部相机 {camera} 尚未标定内参，请先在 head_camera 标定内参')

    def _ensure_wrist_rtsp(self):
        with self._wrist_rtsp_lock:
            if self._wrist_rtsp is None:
                self._wrist_rtsp = WristCamera(self.arm_id, stream='stream1', fps=12)
            return self._wrist_rtsp

    def grab_wrist_preview_bgr(self) -> np.ndarray | None:
        if not self.wrist_preview_enabled:
            return None
        try:
            rtsp = self._ensure_wrist_rtsp()
            frame = rtsp.latest()
            if frame is None:
                frame = rtsp.grab(timeout=2.5, fresh=False)
            return frame
        except Exception:
            return None

    def grab_wrist_calib_bgr(self) -> np.ndarray:
        """Grab 1920x1080 stream0 frame for sharpest ChArUco corners, falling back to active stream."""
        try:
            return grab_frame(self.arm_id, stream='stream0', timeout=6.0)
        except Exception:
            preview = self.grab_wrist_preview_bgr()
            if preview is not None:
                return preview.copy()
            ip = resolve_arm_camera(self.arm_id).get('ip', '')
            raise RuntimeError(f'无法连接腕部相机 camera_{self.arm_id} ({ip})，请检查网口与供电')

    def get_annotated_jpeg(self, key: str) -> bytes | None:
        with self.lock:
            entry = self._annotated_jpegs.get(key)
            return None if entry is None else entry[0]

    def _store_annotated(self, key: str, image_bgr: np.ndarray) -> None:
        jpeg = _encode_jpeg(image_bgr, quality=88)
        with self.lock:
            self._annotated_jpegs[key] = (jpeg, time.time())

    # ------------------------------------------------------------------
    # Stage 1: Multi-Camera Extrinsic Calibration via Common ChArUco Board
    # ------------------------------------------------------------------

    def _resolve_cam_extrinsic(self, cam_name: str, serial: str = '') -> dict[str, Any] | None:
        if cam_name in self.extrinsics:
            return self.extrinsics[cam_name]
        if serial and serial in self.extrinsics:
            return self.extrinsics[serial]
        return None

    def capture_and_calibrate(self) -> dict[str, Any]:
        with self.lock:
            now = time.monotonic()
            if self.joint_positions is None or self.joints_received_at is None or (now - self.joints_received_at) > 2.0:
                raise RuntimeError(f'尚未收到新鲜的 {self.arm_namespace}/joint_states_feedback 关节反馈，无法计算腕部相机位姿')
            q_snap = tuple(float(v) for v in self.joint_positions[:6])

        if get_extrinsic(self.arm_id) is None:
            raise RuntimeError(f'缺少 camera_{self.arm_id} 的手眼外参，请先在 wrist_camera 标定')
        offsets = (
            None
            if self.gravity_model is None
            else tuple(self.gravity_model.joint(i + 1).angle_offset for i in range(6))
        )
        t_base_wrist = camera_pose_in_base(self.arm_id, q_snap, angle_offsets=offsets)

        head_status = self.query_head_cameras_status()
        cam_serials = {str(c.get('name')): str(c.get('serial', '')) for c in head_status.get('cameras', [])}

        with ThreadPoolExecutor(max_workers=3) as pool:
            f_wrist = pool.submit(self.grab_wrist_calib_bgr)
            f_h1 = pool.submit(self.grab_head_bgr, 'head_1', fresh=True)
            f_h2 = pool.submit(self.grab_head_bgr, 'head_2', fresh=True)
            wrist_bgr = f_wrist.result()
            h1_bgr = f_h1.result()
            h2_bgr = f_h2.result()

        h1_intr = self.get_head_intrinsic('head_1')
        h2_intr = self.get_head_intrinsic('head_2')
        w_intr = get_intrinsics(self.arm_id, width=wrist_bgr.shape[1], height=wrist_bgr.shape[0])
        if w_intr is None:
            raise RuntimeError(f'缺少 camera_{self.arm_id} 的内参，请先在 wrist_camera 标定')

        det_w = detect_charuco_pose(
            wrist_bgr, w_intr['camera_matrix'], w_intr['distortion_coefficients'],
            self.board_spec, label=f'camera_{self.arm_id}',
        )
        det_h1 = detect_charuco_pose(
            h1_bgr, h1_intr['camera_matrix'], h1_intr['distortion_coefficients'],
            self.board_spec, label='head_1',
        )
        det_h2 = detect_charuco_pose(
            h2_bgr, h2_intr['camera_matrix'], h2_intr['distortion_coefficients'],
            self.board_spec, label='head_2',
        )

        self._store_annotated('calib_wrist', det_w['annotated_bgr'])
        self._store_annotated('calib_head_1', det_h1['annotated_bgr'])
        self._store_annotated('calib_head_2', det_h2['annotated_bgr'])
        stamp = time.time()

        detections_summary = {
            'wrist': {
                'ok': det_w['ok'],
                'corner_count': det_w['corner_count'],
                'marker_count': det_w['marker_count'],
                'reproj_rms_px': det_w['reproj_rms_px'],
                'reason': det_w['reason'],
                'image_url': f'/api/image/calib_wrist?t={stamp:.3f}',
            },
            'head_1': {
                'ok': det_h1['ok'],
                'corner_count': det_h1['corner_count'],
                'marker_count': det_h1['marker_count'],
                'reproj_rms_px': det_h1['reproj_rms_px'],
                'reason': det_h1['reason'],
                'image_url': f'/api/image/calib_head_1?t={stamp:.3f}',
            },
            'head_2': {
                'ok': det_h2['ok'],
                'corner_count': det_h2['corner_count'],
                'marker_count': det_h2['marker_count'],
                'reproj_rms_px': det_h2['reproj_rms_px'],
                'reason': det_h2['reason'],
                'image_url': f'/api/image/calib_head_2?t={stamp:.3f}',
            },
        }

        if not det_w['ok']:
            with self.lock:
                self._last_calib_report = {'ok': False, 'detections': detections_summary, 'stamp': stamp}
            raise RuntimeError(f'腕部相机未完整识别到标定板：{det_w["reason"]}（已更新角点预览图）')

        t_base_board = t_base_wrist @ det_w['T_cam_board']
        updated_cams: list[str] = []

        for cam_name, det_h, h_intr in (('head_1', det_h1, h1_intr), ('head_2', det_h2, h2_intr)):
            if not det_h['ok']:
                continue
            with self.lock:
                self._calib_samples.setdefault(cam_name, []).append({
                    'T_base_board': t_base_board,
                    'T_head_board': det_h['T_cam_board'],
                    'corners_2d': det_h['corners_2d'],
                    'object_3d': det_h['object_3d'],
                    'stamp': stamp,
                })
                solved = calibrate_external_camera(
                    self._calib_samples[cam_name],
                    h_intr['camera_matrix'],
                    h_intr['distortion_coefficients'],
                )
                serial = cam_serials.get(cam_name) or str(h_intr.get('camera_key', ''))
                solved['camera'] = cam_name
                solved['serial'] = serial
                solved['stamp'] = time.strftime('%Y-%m-%dT%H:%M:%S')
                self.extrinsics[cam_name] = solved
                if serial:
                    self.extrinsics[serial] = solved
                updated_cams.append(cam_name)

        if not updated_cams:
            with self.lock:
                self._last_calib_report = {'ok': False, 'detections': detections_summary, 'stamp': stamp}
            raise RuntimeError('腕部相机已识别标定板，但 head_1 和 head_2 均未看清标定板（需至少 6 个角点）')

        with self.lock:
            to_save = {k: self.extrinsics[k] for k in ('head_1', 'head_2') if k in self.extrinsics}
            save_extrinsics_file(self.extrinsics_path, to_save, arm_id=self.arm_id)
            msg = (
                f'标定成功并已保存 ({", ".join(updated_cams)})：'
                + '；'.join(
                    f'{c} 样本 {self.extrinsics[c]["samples"]} 组, RMS {self.extrinsics[c]["reproj_rms_px"]:.2f}px'
                    for c in updated_cams
                )
            )
            self.status_message = msg
            self._last_calib_report = {
                'ok': True,
                'updated': updated_cams,
                'detections': detections_summary,
                'board_in_base_mm': [round(float(v * 1000.0), 1) for v in t_base_board[:3, 3]],
                'stamp': stamp,
            }
        self._log('INFO', msg)
        return {
            'ok': True,
            'message': msg,
            'updated': updated_cams,
            'detections': detections_summary,
            'extrinsics': self._public_extrinsics(),
        }

    def clear_calibration_samples(self, delete_saved: bool = False) -> dict[str, Any]:
        with self.lock:
            self._calib_samples = {'head_1': [], 'head_2': []}
            self._last_calib_report = None
            if delete_saved:
                self.extrinsics.clear()
                if self.extrinsics_path.is_file():
                    self.extrinsics_path.unlink()
                msg = '已清空标定样本及已保存的外参文件'
            else:
                msg = '已清空本次会话累计标定样本（下次采集将从第 1 组重新计算并覆盖保存）'
            self.status_message = msg
        self._log('INFO', msg)
        return {'ok': True, 'message': msg, 'extrinsics': self._public_extrinsics()}

    def _public_extrinsics(self) -> dict[str, Any]:
        out: dict[str, Any] = {}
        for key in ('head_1', 'head_2'):
            entry = self.extrinsics.get(key)
            if entry is None:
                out[key] = {'calibrated': False, 'session_samples': len(self._calib_samples.get(key, []))}
            else:
                out[key] = {
                    'calibrated': True,
                    'camera': key,
                    'serial': entry.get('serial', ''),
                    'translation_mm': entry.get('translation_mm'),
                    'rpy_deg': entry.get('rpy_deg'),
                    'samples': entry.get('samples', 1),
                    'session_samples': len(self._calib_samples.get(key, [])),
                    'reproj_rms_px': entry.get('reproj_rms_px'),
                    'consistency': entry.get('consistency'),
                    'stamp': entry.get('stamp', ''),
                }
        return out

    # ------------------------------------------------------------------
    # Stage 2: Single-Call Dual-View GPT-6.1-sol Perception
    # ------------------------------------------------------------------

    @staticmethod
    def _xy_pair(pt: Any) -> tuple[float, float]:
        if isinstance(pt, dict):
            return float(pt['u']), float(pt['v'])
        return float(pt[0]), float(pt[1])

    def cancel_agent_perception(self) -> dict[str, Any]:
        self._job_cancel.set()
        with self.lock:
            if self._job_state.get('running'):
                self._job_state['stage'] = 'cancelling'
                msg = '正在取消当前 Agent 识别请求…'
            else:
                msg = '当前没有正在运行的识别请求'
            self.status_message = msg
        self._log('INFO', msg)
        return {'ok': True, 'message': msg}

    def start_agent_perception_async(
        self,
        *,
        object_prompt: str = DEFAULT_OBJECT_PROMPT,
        output_prompt: str = DEFAULT_OUTPUT_PROMPT,
        reasoning_effort: str | None = None,
        standoff_cm: float = 0.5,
        insert_depth_cm: float = 2.0,
    ) -> dict[str, Any]:
        ext1 = self._resolve_cam_extrinsic('head_1', '000314605112')
        ext2 = self._resolve_cam_extrinsic('head_2', '000456614112')

        eff = str(reasoning_effort or self.llm_reasoning_effort).strip().lower()
        with self.lock:
            if self._job_state.get('running'):
                raise RuntimeError('已有 Agent 识别请求正在执行中，可点击“取消请求”后再试')
            self._job_cancel.clear()
            self._job_state = {
                'running': True,
                'stage': 'capturing',
                'reasoning_effort': eff,
                'reasoning_steps': 0,
                'started_at': time.monotonic(),
                'elapsed_s': 0.0,
                'partial_output': '',
                'error': '',
            }
            self.last_error = ''
            msg = f'已发起 {self.llm_model} ({eff}) 双图识别请求，正在实时流式接收…'
            self.status_message = msg

        def _worker() -> None:
            try:
                self.run_agent_perception(
                    object_prompt=object_prompt,
                    output_prompt=output_prompt,
                    reasoning_effort=eff,
                    standoff_cm=standoff_cm,
                    insert_depth_cm=insert_depth_cm,
                )
                with self.lock:
                    self._job_state['running'] = False
                    self._job_state['stage'] = 'done'
                    self._job_state['elapsed_s'] = round(time.monotonic() - self._job_state['started_at'], 1)
            except PerceptionCancelledError as exc:
                with self.lock:
                    self._job_state['running'] = False
                    self._job_state['stage'] = 'cancelled'
                    self._job_state['error'] = str(exc)
                    self.status_message = str(exc)
                self._log('INFO', str(exc))
            except Exception as exc:
                err_msg = f'Agent 识别失败：{exc}'
                with self.lock:
                    self._job_state['running'] = False
                    self._job_state['stage'] = 'error'
                    self._job_state['error'] = str(exc)
                    self.last_error = err_msg
                    self.status_message = err_msg
                self._log('ERROR', err_msg)

        threading.Thread(target=_worker, daemon=True, name='agent-perceive-worker').start()
        return {'ok': True, 'started': True, 'reasoning_effort': eff, 'message': msg}

    def run_agent_perception(
        self,
        *,
        object_prompt: str = DEFAULT_OBJECT_PROMPT,
        output_prompt: str = DEFAULT_OUTPUT_PROMPT,
        reasoning_effort: str | None = None,
        standoff_cm: float = 0.5,
        insert_depth_cm: float = 2.0,
    ) -> dict[str, Any]:
        ext1 = self._resolve_cam_extrinsic('head_1', '000314605112')
        ext2 = self._resolve_cam_extrinsic('head_2', '000456614112')
        if ext1 is None or ext2 is None:
            missing = [name for name, e in (('head_1', ext1), ('head_2', ext2)) if e is None]
            raise RuntimeError(f'请先在第一板块完成外部相机外参标定（缺少 {", ".join(missing)}）')

        eff = str(reasoning_effort or self.llm_reasoning_effort).strip().lower()
        with self.lock:
            now = time.monotonic()
            joints_fresh = self.joint_positions is not None and self.joints_received_at is not None and (now - self.joints_received_at) <= 2.0
            q_obs = (
                tuple(float(v) for v in self.joint_positions[:6])
                if self.joint_positions is not None
                else (0.0, 0.85, -1.35, 0.0, 0.50, 0.0)
            )
            tip_off = tuple(self.tip_offset_m)

        with ThreadPoolExecutor(max_workers=2) as pool:
            f1 = pool.submit(self.grab_head_bgr, 'head_1', fresh=True)
            f2 = pool.submit(self.grab_head_bgr, 'head_2', fresh=True)
            img1 = f1.result()
            img2 = f2.result()

        with self.lock:
            if self._job_state.get('running'):
                self._job_state['stage'] = 'reasoning'
        self._log('INFO', f'正在调用 {self.llm_model} ({eff}) 识别双视角图像…')

        def _on_progress(prog: dict[str, Any]) -> None:
            with self.lock:
                if self._job_state.get('running'):
                    self._job_state['stage'] = prog.get('stage', 'reasoning')
                    self._job_state['reasoning_steps'] = int(prog.get('reasoning_steps', 0))
                    self._job_state['elapsed_s'] = float(prog.get('elapsed_s', 0.0))
                    self._job_state['partial_output'] = str(prog.get('partial_output', ''))

        llm_res = perceive_dual_views(
            img1,
            img2,
            object_prompt=object_prompt,
            output_prompt=output_prompt,
            cam1_label='head_1',
            cam2_label='head_2',
            api_base=self.llm_api_base,
            model=self.llm_model,
            reasoning_effort=eff,
            on_progress=_on_progress,
            cancel_event=self._job_cancel,
        )

        c1, c2 = llm_res['cam1'], llm_res['cam2']
        sym_deg = float(llm_res.get('symmetry_deg', 0.0))

        gpt_over1 = render_gpt_output_overlay(img1, cam_label='head_1', det_2d=c1, symmetry_deg=sym_deg)
        gpt_over2 = render_gpt_output_overlay(img2, cam_label='head_2', det_2d=c2, symmetry_deg=sym_deg)
        gpt_crop1 = render_gpt_crop_overlay(img1, cam_label='head_1', det_2d=c1, symmetry_deg=sym_deg)
        gpt_crop2 = render_gpt_crop_overlay(img2, cam_label='head_2', det_2d=c2, symmetry_deg=sym_deg)
        self._store_annotated('gpt_head_1', gpt_over1)
        self._store_annotated('gpt_head_2', gpt_over2)
        self._store_annotated('gpt_crop_head_1', gpt_crop1)
        self._store_annotated('gpt_crop_head_2', gpt_crop2)
        stamp = time.time()

        if ext1 is None or ext2 is None:
            missing = [name for name, e in (('head_1', ext1), ('head_2', ext2)) if e is None]
            msg = (
                f'GPT 识别完成 ({llm_res["latency_s"]:.1f}s)，已在双图中标出全部 6 个关键点；'
                f'尚未标定外参 ({(", ".join(missing))})，完成第 1 板块标定后可解算 3D 坐标'
            )
            result_payload = {
                'ok': True,
                'calibrated_3d': False,
                'stamp': stamp,
                'model': llm_res['model'],
                'reasoning_effort': llm_res['reasoning_effort'],
                'reasoning_steps': llm_res.get('reasoning_steps', 0),
                'latency_s': llm_res['latency_s'],
                'object_prompt': llm_res['object_prompt'],
                'output_prompt': llm_res['output_prompt'],
                'raw_llm_output': llm_res.get('raw_numeric', {'symmetry_deg': sym_deg, 'cam1': c1, 'cam2': c2}),
                'symmetry_deg': sym_deg,
                'detections_2d': {'head_1': c1, 'head_2': c2},
                'images': {
                    'head_1': f'/api/image/gpt_head_1?t={stamp:.3f}',
                    'head_2': f'/api/image/gpt_head_2?t={stamp:.3f}',
                    'gpt_head_1': f'/api/image/gpt_head_1?t={stamp:.3f}',
                    'gpt_head_2': f'/api/image/gpt_head_2?t={stamp:.3f}',
                    'gpt_crop_head_1': f'/api/image/gpt_crop_head_1?t={stamp:.3f}',
                    'gpt_crop_head_2': f'/api/image/gpt_crop_head_2?t={stamp:.3f}',
                },
            }
            with self.lock:
                self._perception_result = result_payload
                self.status_message = msg
            self._log('INFO', msg)
            return {**result_payload, 'message': msg}

        intr1 = self.get_head_intrinsic('head_1')
        intr2 = self.get_head_intrinsic('head_2')
        K1, D1, T1 = intr1['camera_matrix'], intr1['distortion_coefficients'], ext1['T_base_cam']
        K2, D2, T2 = intr2['camera_matrix'], intr2['distortion_coefficients'], ext2['T_base_cam']

        h1_uv = self._xy_pair(c1['hole_center'])
        h2_uv = self._xy_pair(c2['hole_center'])
        ha1_uv = self._xy_pair(c1['hole_insert_point'])
        ha2_uv = self._xy_pair(c2['hole_insert_point'])
        p1_uv = self._xy_pair(c1['plug_tip'])
        p2_uv = self._xy_pair(c2['plug_tip'])

        o1, dh1 = pixel_to_base_ray(h1_uv[0], h1_uv[1], K1, D1, T1)
        o2, dh2 = pixel_to_base_ray(h2_uv[0], h2_uv[1], K2, D2, T2)
        hole_tri = triangulate_two_rays(o1, dh1, o2, dh2)

        _, dp1 = pixel_to_base_ray(p1_uv[0], p1_uv[1], K1, D1, T1)
        _, dp2 = pixel_to_base_ray(p2_uv[0], p2_uv[1], K2, D2, T2)
        tip_tri = triangulate_two_rays(o1, dp1, o2, dp2)

        axis_res = solve_axis_from_two_views(
            hole_tri['point_b'],
            h1_uv,
            ha1_uv,
            K1, D1, T1,
            h2_uv,
            ha2_uv,
            K2, D2, T2,
        )

        p_hole_b = hole_tri['point_b']
        p_tip_vis_b = tip_tri['point_b']
        a_insert_b = axis_res['insert_axis_b']

        hole_orient_b = None
        plug_orient_b = None
        if 'hole_orient_point' in c1 and 'hole_orient_point' in c2:
            ho1_uv = self._xy_pair(c1['hole_orient_point'])
            ho2_uv = self._xy_pair(c2['hole_orient_point'])
            hole_orient_b = solve_axis_from_two_views(
                p_hole_b, h1_uv, ho1_uv, K1, D1, T1, h2_uv, ho2_uv, K2, D2, T2,
            )['insert_axis_b']
        if 'plug_orient_point' in c1 and 'plug_orient_point' in c2:
            po1_uv = self._xy_pair(c1['plug_orient_point'])
            po2_uv = self._xy_pair(c2['plug_orient_point'])
            plug_orient_b = solve_axis_from_two_views(
                p_tip_vis_b, p1_uv, po1_uv, K1, D1, T1, p2_uv, po2_uv, K2, D2, T2,
            )['insert_axis_b']

        plan = compute_alignment_plan(
            hole_entrance_b=p_hole_b,
            insert_axis_b=a_insert_b,
            joints_rad=q_obs,
            tip_offset_m=tip_off,
            gravity_model=self.gravity_model,
            standoff_m=float(standoff_cm) / 100.0,
            insert_depth_m=float(insert_depth_cm) / 100.0,
            hole_orient_b=hole_orient_b,
            plug_orient_b=plug_orient_b,
            symmetry_deg=sym_deg,
        )

        p_fk_b, _ = forward_tip_pose(q_obs, tip_offset_m=tip_off, gravity_model=self.gravity_model)
        vis_fk_diff_mm = [round(float((p_tip_vis_b[i] - p_fk_b[i]) * 1000.0), 2) for i in range(3)]
        inferred_off_m = infer_gripper_tip_offset(q_obs, p_tip_vis_b, self.gravity_model) if joints_fresh else tip_off

        over1 = render_perception_overlay(
            img1,
            cam_label='head_1',
            det_2d=c1,
            camera_matrix=K1,
            distortion=D1,
            t_base_cam=T1,
            p_hole_b=p_hole_b,
            a_insert_b=a_insert_b,
            p_pre_b=plan['pre_insert_m'],
            p_tip_vis_b=p_tip_vis_b,
            hole_gap_mm=hole_tri['ray_gap_mm'],
            tip_gap_mm=tip_tri['ray_gap_mm'],
        )
        over2 = render_perception_overlay(
            img2,
            cam_label='head_2',
            det_2d=c2,
            camera_matrix=K2,
            distortion=D2,
            t_base_cam=T2,
            p_hole_b=p_hole_b,
            a_insert_b=a_insert_b,
            p_pre_b=plan['pre_insert_m'],
            p_tip_vis_b=p_tip_vis_b,
            hole_gap_mm=hole_tri['ray_gap_mm'],
            tip_gap_mm=tip_tri['ray_gap_mm'],
        )
        self._store_annotated('agent_head_1', over1)
        self._store_annotated('agent_head_2', over2)
        if self.extrinsics_path == default_extrinsics_path():
            try:
                out_dir = default_outputs_dir()
                out_dir.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(out_dir / 'gpt_head_1.jpg'), gpt_over1, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
                cv2.imwrite(str(out_dir / 'gpt_head_2.jpg'), gpt_over2, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
                cv2.imwrite(str(out_dir / 'gpt_crop_head_1.jpg'), gpt_crop1, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
                cv2.imwrite(str(out_dir / 'gpt_crop_head_2.jpg'), gpt_crop2, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
                cv2.imwrite(str(out_dir / 'reproj_head_1.jpg'), over1, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
                cv2.imwrite(str(out_dir / 'reproj_head_2.jpg'), over2, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
            except Exception:
                pass

        # Also update self.goal so the Hole-Frame H visualizer immediately shows live distance/angle!
        q_goal_seed = solve_tip_ik(
            q_obs,
            plan['goal_inserted_m'],
            plan['q_bh_wxyz'],
            tip_offset_m=tip_off,
            gravity_model=self.gravity_model,
            max_iters=25,
        )
        result_payload = {
            'ok': True,
            'calibrated_3d': True,
            'stamp': stamp,
            'model': llm_res['model'],
            'reasoning_effort': llm_res['reasoning_effort'],
            'reasoning_steps': llm_res.get('reasoning_steps', 0),
            'latency_s': llm_res['latency_s'],
            'object_prompt': llm_res['object_prompt'],
            'output_prompt': llm_res['output_prompt'],
            'raw_llm_output': llm_res.get('raw_numeric', {'symmetry_deg': sym_deg, 'cam1': c1, 'cam2': c2}),
            'symmetry_deg': sym_deg,
            'detections_2d': {
                'head_1': c1,
                'head_2': c2,
            },
            'hole_entrance_m': plan['hole_entrance_m'],
            'hole_entrance_mm': plan['hole_entrance_mm'],
            'hole_ray_gap_mm': hole_tri['ray_gap_mm'],
            'ray_angle_deg': hole_tri['ray_angle_deg'],
            'plug_tip_vis_m': [float(v) for v in p_tip_vis_b],
            'plug_tip_vis_mm': [round(float(v * 1000.0), 2) for v in p_tip_vis_b],
            'plug_tip_fk_mm': [round(float(v * 1000.0), 2) for v in p_fk_b],
            'plug_vis_minus_fk_mm': vis_fk_diff_mm,
            'plug_ray_gap_mm': tip_tri['ray_gap_mm'],
            'inferred_tip_offset_cm': [round(float(v * 100.0), 2) for v in inferred_off_m],
            'relative_hole_minus_tip_m': [float(p_hole_b[i] - p_tip_vis_b[i]) for i in range(3)],
            'relative_hole_minus_tip_mm': [round(float((p_hole_b[i] - p_tip_vis_b[i]) * 1000.0), 2) for i in range(3)],
            'insert_axis_unit': plan['insert_axis_unit'],
            'insert_axis_method': axis_res['method'],
            'plane_angle_deg': axis_res['plane_angle_deg'],
            'hole_orient_b': None if hole_orient_b is None else [float(v) for v in hole_orient_b],
            'plug_orient_b': None if plug_orient_b is None else [float(v) for v in plug_orient_b],
            'alignment': plan,
            'joints_fresh_at_capture': joints_fresh,
            'q_obs_rad': list(q_obs),
            'images': {
                'head_1': f'/api/image/gpt_head_1?t={stamp:.3f}',
                'head_2': f'/api/image/gpt_head_2?t={stamp:.3f}',
                'gpt_head_1': f'/api/image/gpt_head_1?t={stamp:.3f}',
                'gpt_head_2': f'/api/image/gpt_head_2?t={stamp:.3f}',
                'gpt_crop_head_1': f'/api/image/gpt_crop_head_1?t={stamp:.3f}',
                'gpt_crop_head_2': f'/api/image/gpt_crop_head_2?t={stamp:.3f}',
                'reproj_head_1': f'/api/image/agent_head_1?t={stamp:.3f}',
                'reproj_head_2': f'/api/image/agent_head_2?t={stamp:.3f}',
            },
        }

        with self.lock:
            self._perception_result = result_payload
            p_goal = plan['goal_inserted_m']
            q_goal = plan['q_bh_wxyz']
            self.goal = GoalPose(
                joints=q_goal_seed,
                p_bh=(float(p_goal[0]), float(p_goal[1]), float(p_goal[2])),
                q_bh=(float(q_goal[0]), float(q_goal[1]), float(q_goal[2]), float(q_goal[3])),
                tip_length_m=self.tip_length_m,
                recorded_at=stamp,
                tip_offset_m=self.tip_offset_m,
            )
            self.reached_goal = False
            self.reached_goal_steps = 0
            msg = (
                f'Agent 识别完成 ({llm_res["latency_s"]:.1f}s)：孔口 B = {plan["hole_entrance_mm"]} mm，'
                f'插入轴 = {plan["insert_axis_unit"]}，双目交会误差 {hole_tri["ray_gap_mm"]:.1f} mm'
            )
            self.status_message = msg
        self._log('INFO', msg)
        return {**result_payload, 'message': msg}

    # ------------------------------------------------------------------
    # Stage 3: Smooth Preliminary Move to 0.5 cm in Front of Hole
    # ------------------------------------------------------------------

    def start_pre_insert_move(
        self,
        *,
        standoff_cm: float = 0.5,
        insert_depth_cm: float = 2.0,
        duration_s: float | None = None,
        use_relative_delta: bool = False,
    ) -> dict[str, Any]:
        with self.lock:
            if self._perception_result is None:
                raise RuntimeError('请先在第二板块执行 Agent 视觉识别，获取孔口位置与朝向')
            if not self._perception_result.get('calibrated_3d', True):
                raise RuntimeError('当前仅有 GPT 2D 像素标记结果，请先在第一板块完成外参标定并重新识别后再移动')
            now = time.monotonic()
            if self.joint_positions is None or self.joints_received_at is None or (now - self.joints_received_at) > 2.0:
                raise RuntimeError(f'尚未收到新鲜的 {self.arm_namespace}/joint_states_feedback 关节反馈')
            if self.arm_status is not None and not all(self.arm_status.motor_enabled):
                raise RuntimeError('机械臂尚未使能，请先点击右上角“使能机械臂”')

            q_curr = tuple(float(v) for v in self.joint_positions[:6])
            perc = self._perception_result
            if use_relative_delta:
                # Apply the visually observed 3D vector (p_hole_vis - p_tip_vis) to FK tip at capture pose q_obs
                p_fk_obs, _ = forward_tip_pose(
                    perc['q_obs_rad'], tip_offset_m=self.tip_offset_m, gravity_model=self.gravity_model,
                )
                rel_vec = np.asarray(perc['relative_hole_minus_tip_m'], dtype=np.float64)
                p_hole_eff = p_fk_obs + rel_vec
            else:
                p_hole_eff = np.asarray(perc['hole_entrance_m'], dtype=np.float64)

            a_ins = np.asarray(perc['insert_axis_unit'], dtype=np.float64)
            plan = compute_alignment_plan(
                hole_entrance_b=p_hole_eff,
                insert_axis_b=a_ins,
                joints_rad=q_curr,
                tip_offset_m=self.tip_offset_m,
                gravity_model=self.gravity_model,
                standoff_m=float(standoff_cm) / 100.0,
                insert_depth_m=float(insert_depth_cm) / 100.0,
                hole_orient_b=perc.get('hole_orient_b'),
                plug_orient_b=perc.get('plug_orient_b'),
                symmetry_deg=float(perc.get('symmetry_deg', 0.0)),
            )

            p_start, q_start = forward_tip_pose(q_curr, tip_offset_m=self.tip_offset_m, gravity_model=self.gravity_model)
            p_target = np.asarray(plan['pre_insert_m'], dtype=np.float64)
            q_target = np.asarray(plan['q_bh_wxyz'], dtype=np.float64)

            dist_m = float(np.linalg.norm(p_target - p_start))
            ang_rad = float(np.linalg.norm(quat_rotvec(quat_mul(q_target, quat_inv(q_start)))))
            auto_dur = max(1.5, dist_m / 0.06, ang_rad / 0.65)
            dur = float(duration_s) if (duration_s is not None and float(duration_s) >= 0.4) else auto_dur

            q_goal_seed = solve_tip_ik(
                q_curr,
                plan['goal_inserted_m'],
                q_target,
                tip_offset_m=self.tip_offset_m,
                gravity_model=self.gravity_model,
                max_iters=25,
            )
            p_goal = plan['goal_inserted_m']
            self.goal = GoalPose(
                joints=q_goal_seed,
                p_bh=(float(p_goal[0]), float(p_goal[1]), float(p_goal[2])),
                q_bh=(float(q_target[0]), float(q_target[1]), float(q_target[2]), float(q_target[3])),
                tip_length_m=self.tip_length_m,
                recorded_at=time.time(),
                tip_offset_m=self.tip_offset_m,
            )
            with self._policy_lock:
                self.policy.reset()
                self.rule_policy.reset()
            self._policy_generation += 1
            self.reached_goal = False
            self.reached_goal_steps = 0

            self._move_start_time = now
            self._move_duration_s = dur
            self._move_p_start = p_start
            self._move_q_start = q_start
            self._move_p_target = p_target
            self._move_q_target = q_target
            self._move_prev_joints = q_curr
            self._move_progress = 0.0
            self.last_cmd_joints = q_curr
            self.last_cmd_velocity = (0.0,) * 6
            self._ensure_driver_mit_mode(q_curr)
            self.mode = 'move'

            msg = (
                f'开始初步移动：目标孔前 {float(standoff_cm):.2f} cm '
                f'({plan["pre_insert_mm"]} mm)，位移 {dist_m*1000:.1f} mm，转角 {math.degrees(ang_rad):.1f}°，预计 {dur:.1f} s'
            )
            self.status_message = msg
        self._log('INFO', msg)
        return {'ok': True, 'mode': self.mode, 'duration_s': round(dur, 2), 'alignment': plan, 'message': msg}

    # ------------------------------------------------------------------
    # Stage 4 & Control Loop Overrides
    # ------------------------------------------------------------------

    def start_policy(self, controller: str = 'onnx') -> dict[str, Any]:
        with self.lock:
            self._tip_trace.clear()
        return super().start_policy(controller)

    def _publish_last_target(self) -> None:
        with self.lock:
            self._sync_estimation_source()
            if self.gripper_state in ('open', 'closed') and self.gripper_cmd_effort > 0.0:
                self._publish_gripper(self.gripper_cmd_width_m, self.gripper_cmd_effort)
            if self.mode in ('policy', 'move') and self.last_cmd_joints is not None:
                self._ensure_driver_mit_mode(self.last_cmd_joints)
                self._publish_mit(
                    self.last_cmd_joints, AUTO_MIT_GAINS, AUTO_MIT_GAINS, self.last_cmd_velocity,
                )

    def _tick(self) -> None:
        now = time.monotonic()
        with self.lock:
            if self.joint_positions is None or self.joints_received_at is None or (now - self.joints_received_at) > 1.5:
                return
            if self.mode == 'move':
                self._step_pre_insert_move(now)
                return

        super()._tick()

        # Optional stationary-in-hole exit check
        with self.lock:
            if self.mode == 'policy' and self.auto_stop_stationary and self.last_step_result is not None:
                p_bt = np.asarray(self.last_step_result.p_bt, dtype=np.float64)
                self._tip_trace.append((now, p_bt))
                penetration_m = self._current_hole_penetration_m(p_bt)
                if penetration_m is not None and penetration_m >= 0.005 and len(self._tip_trace) >= 25:
                    t_old, p_old = self._tip_trace[0]
                    if (now - t_old) >= 0.5:
                        recent_pts = np.stack([pt for _, pt in self._tip_trace], axis=0)
                        span_m = float(np.max(np.linalg.norm(recent_pts - p_bt, axis=1)))
                        if span_m <= 0.0005:
                            self.hold_joints = tuple(float(v) for v in self.joint_positions[:6])
                            self.last_cmd_velocity = (0.0,) * 6
                            self.mode = 'hold'
                            self._policy_generation += 1
                            msg = f'插头已深入孔内 {penetration_m*1000:.1f} mm 且位置稳定 (<0.5 mm)，已自动停止并切 Hold'
                            self.status_message = msg
                            self._log('INFO', msg)

    def _current_hole_penetration_m(self, p_bt: np.ndarray) -> float | None:
        if self._perception_result is not None and 'hole_entrance_m' in self._perception_result:
            p_ent = np.asarray(self._perception_result['hole_entrance_m'], dtype=np.float64)
            a_ins = np.asarray(self._perception_result['insert_axis_unit'], dtype=np.float64)
            return float(np.dot(p_bt - p_ent, a_ins))
        return None

    def _step_pre_insert_move(self, now: float) -> None:
        assert self._move_p_start is not None and self._move_q_start is not None
        assert self._move_p_target is not None and self._move_q_target is not None
        q_seed = self._move_prev_joints if self._move_prev_joints is not None else self.joint_positions
        assert q_seed is not None

        tau = min(1.0, max(0.0, (now - self._move_start_time) / max(self._move_duration_s, 0.1)))
        s = tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau))
        p_des = self._move_p_start + s * (self._move_p_target - self._move_p_start)
        dr = quat_rotvec(quat_mul(self._move_q_target, quat_inv(self._move_q_start)))
        q_des = canonical_quat(quat_mul(rotvec_quat(s * dr), self._move_q_start))

        q_cmd = solve_tip_ik(
            q_seed,
            p_des,
            q_des,
            tip_offset_m=self.tip_offset_m,
            gravity_model=self.gravity_model,
            max_iters=35 if tau >= 1.0 else 20,
        )
        if tau < 1.0:
            v_cmd = tuple(float((q_cmd[i] - q_seed[i]) / POLICY_PERIOD_S) for i in range(6))
        else:
            v_cmd = (0.0,) * 6

        self._move_prev_joints = q_cmd
        self._move_progress = round(tau, 4)
        self.last_cmd_joints = q_cmd
        self.last_cmd_velocity = v_cmd
        self._ensure_driver_mit_mode(q_cmd)
        self._publish_mit(q_cmd, AUTO_MIT_GAINS, AUTO_MIT_GAINS, v_cmd)

        if tau >= 1.0:
            self.hold_joints = q_cmd
            self.last_cmd_velocity = (0.0,) * 6
            self.mode = 'hold'
            self.status_message = '已完成初步移动：插头末端已对准并停在孔前预插位置 (Hold)，可选择执行插入策略'
            self._log('INFO', self.status_message)

    def status_payload(self) -> dict[str, Any]:
        base = super().status_payload()
        with self.lock:
            penetration_mm = None
            pre_dist_mm = None
            if self.joint_positions is not None and self._perception_result is not None:
                p_bt, _ = forward_tip_pose(
                    self.joint_positions, tip_offset_m=self.tip_offset_m, gravity_model=self.gravity_model,
                )
                pen_m = self._current_hole_penetration_m(p_bt)
                penetration_mm = None if pen_m is None else round(pen_m * 1000.0, 2)
                if self._move_p_target is not None:
                    pre_dist_mm = round(float(np.linalg.norm(p_bt - self._move_p_target) * 1000.0), 2)
                elif 'alignment' in self._perception_result:
                    p_pre = np.asarray(self._perception_result['alignment']['pre_insert_m'], dtype=np.float64)
                    pre_dist_mm = round(float(np.linalg.norm(p_bt - p_pre) * 1000.0), 2)

            base['agent_flow'] = {
                'extrinsics': self._public_extrinsics(),
                'extrinsics_path': str(self.extrinsics_path),
                'last_calib_report': self._last_calib_report,
                'perception': self._perception_result,
                'job': {
                    **self._job_state,
                    'elapsed_s': (
                        round(time.monotonic() - self._job_state['started_at'], 1)
                        if self._job_state.get('running') and self._job_state.get('started_at')
                        else self._job_state.get('elapsed_s', 0.0)
                    ),
                },
                'move': {
                    'active': self.mode == 'move',
                    'progress': self._move_progress,
                    'duration_s': round(self._move_duration_s, 2),
                    'distance_to_pre_mm': pre_dist_mm,
                },
                'penetration_past_entrance_mm': penetration_mm,
                'auto_stop_stationary': self.auto_stop_stationary,
                'wrist_preview_enabled': self.wrist_preview_enabled,
                'llm': {
                    'api_base': self.llm_api_base,
                    'model': self.llm_model,
                    'reasoning_effort': self.llm_reasoning_effort,
                    'default_object_prompt': DEFAULT_OBJECT_PROMPT,
                    'default_output_prompt': DEFAULT_OUTPUT_PROMPT,
                },
            }
        return base


class _DaemonThreadingHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class _AgentFlowHTTPHandler(BaseHTTPRequestHandler):
    node: AgentInsertNode
    static_dir: Path

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _send_bytes(self, status: int, data: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get('Content-Length', '0'))
        raw = self.rfile.read(length).decode('utf-8') if length > 0 else '{}'
        data = json.loads(raw or '{}')
        if not isinstance(data, dict):
            raise ValueError('Expected JSON object body')
        return data

    def _proxy_head_mjpeg(self, camera: str) -> None:
        boundary = b'headframe'
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=headframe')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            while not self.node.stop_event.is_set():
                try:
                    bgr = self.node.grab_head_bgr(camera, fresh=False, timeout=2.0)
                    small = cv2.resize(bgr, (640, 360), interpolation=cv2.INTER_AREA)
                    jpg = _encode_jpeg(small, quality=72)
                    sleep_s = 0.10
                except Exception:
                    canvas = np.zeros((360, 640, 3), dtype=np.uint8)
                    canvas[:] = (28, 36, 52)
                    cv2.putText(canvas, f'Waiting for {camera}...', (140, 185),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (180, 200, 225), 2, cv2.LINE_AA)
                    jpg = _encode_jpeg(canvas, quality=72)
                    sleep_s = 0.5
                header = (
                    b'--' + boundary + b'\r\n'
                    b'Content-Type: image/jpeg\r\n'
                    b'Content-Length: ' + str(len(jpg)).encode('ascii') + b'\r\n\r\n'
                )
                self.wfile.write(header + jpg + b'\r\n')
                self.wfile.flush()
                time.sleep(sleep_s)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def _stream_wrist_mjpeg(self) -> None:
        boundary = b'wristframe'
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=wristframe')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            while not self.node.stop_event.is_set():
                frame = self.node.grab_wrist_preview_bgr()
                if frame is None:
                    canvas = np.zeros((360, 640, 3), dtype=np.uint8)
                    canvas[:] = (28, 36, 52)
                    txt = 'Wrist Stream Paused' if not self.node.wrist_preview_enabled else 'Waiting for camera_arm_4...'
                    cv2.putText(canvas, txt, (95, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (180, 200, 225), 2, cv2.LINE_AA)
                    jpg = _encode_jpeg(canvas, quality=75)
                    sleep_s = 0.5
                else:
                    jpg = _encode_jpeg(frame, quality=72)
                    sleep_s = 0.08
                header = (
                    b'--' + boundary + b'\r\n'
                    b'Content-Type: image/jpeg\r\n'
                    b'Content-Length: ' + str(len(jpg)).encode('ascii') + b'\r\n\r\n'
                )
                self.wfile.write(header + jpg + b'\r\n')
                self.wfile.flush()
                time.sleep(sleep_s)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def do_GET(self) -> None:
        parsed = urlsplit(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)

        if path == '/api/status':
            payload = self.node.status_payload()
            if qs.get('cameras') == ['1']:
                payload['head_cameras'] = self.node.query_head_cameras_status()
            self._send_json(200, payload)
            return

        if path == '/api/cameras':
            self._send_json(200, self.node.query_head_cameras_status())
            return

        if path.startswith('/api/stream/'):
            cam = path.split('/api/stream/', 1)[1].strip('/')
            if cam in ('head_1', 'head_2'):
                self._proxy_head_mjpeg(cam)
                return
            if cam == 'wrist':
                self._stream_wrist_mjpeg()
                return
            self._send_json(404, {'ok': False, 'error': f'Unknown stream: {cam}'})
            return

        if path.startswith('/api/frame/'):
            cam = path.split('/api/frame/', 1)[1].strip('/')
            try:
                if cam in ('head_1', 'head_2'):
                    bgr = self.node.grab_head_bgr(cam, fresh=False)
                elif cam == 'wrist':
                    bgr = self.node.grab_wrist_preview_bgr()
                    if bgr is None:
                        raise RuntimeError('腕部相机当前无画面')
                else:
                    self._send_json(404, {'ok': False, 'error': f'Unknown camera: {cam}'})
                    return
                self._send_bytes(200, _encode_jpeg(bgr, quality=85), 'image/jpeg')
            except Exception as exc:
                self._send_json(502, {'ok': False, 'error': str(exc)})
            return

        if path.startswith('/api/image/'):
            key = path.split('/api/image/', 1)[1].strip('/')
            jpg = self.node.get_annotated_jpeg(key)
            if jpg is None:
                self._send_json(404, {'ok': False, 'error': f'No annotated image for {key}'})
                return
            self._send_bytes(200, jpg, 'image/jpeg')
            return

        rel_path = Path('index.html' if path in ('', '/') else path.lstrip('/'))
        if '..' in rel_path.parts or not (self.static_dir / rel_path).is_file():
            self._send_json(404, {'ok': False, 'error': f'Not found: {path}'})
            return
        file_path = self.static_dir / rel_path
        mime = {
            '.html': 'text/html; charset=utf-8',
            '.css': 'text/css; charset=utf-8',
            '.js': 'application/javascript; charset=utf-8',
        }.get(file_path.suffix, 'application/octet-stream')
        self._send_bytes(200, file_path.read_bytes(), mime)

    def do_POST(self) -> None:
        path = urlsplit(self.path).path
        try:
            body = self._read_json()
            if path == '/api/power':
                res = self.node.set_power(bool(body.get('enable', True)))
            elif path == '/api/drag':
                res = self.node.enter_drag()
            elif path == '/api/gripper':
                w, e = body.get('width_m'), body.get('effort')
                res = self.node.set_gripper(
                    str(body.get('action', 'open')),
                    width_m=None if w is None else float(w),
                    effort=None if e is None else float(e),
                )
            elif path == '/api/calib/capture':
                res = self.node.capture_and_calibrate()
            elif path == '/api/calib/clear':
                res = self.node.clear_calibration_samples(delete_saved=bool(body.get('delete_saved', False)))
            elif path == '/api/wrist/preview':
                res = self.node.set_wrist_preview(bool(body.get('enabled', True)))
            elif path == '/api/agent/perceive':
                if body.get('async', False):
                    res = self.node.start_agent_perception_async(
                        object_prompt=str(body.get('object_prompt') or DEFAULT_OBJECT_PROMPT),
                        output_prompt=str(body.get('output_prompt') or DEFAULT_OUTPUT_PROMPT),
                        reasoning_effort=body.get('reasoning_effort'),
                        standoff_cm=float(body.get('standoff_cm', 0.5)),
                        insert_depth_cm=float(body.get('insert_depth_cm', 2.0)),
                    )
                else:
                    res = self.node.run_agent_perception(
                        object_prompt=str(body.get('object_prompt') or DEFAULT_OBJECT_PROMPT),
                        output_prompt=str(body.get('output_prompt') or DEFAULT_OUTPUT_PROMPT),
                        reasoning_effort=body.get('reasoning_effort'),
                        standoff_cm=float(body.get('standoff_cm', 0.5)),
                        insert_depth_cm=float(body.get('insert_depth_cm', 2.0)),
                    )
            elif path == '/api/agent/cancel':
                res = self.node.cancel_agent_perception()
            elif path == '/api/move/pre_insert':
                dur = body.get('duration_s')
                res = self.node.start_pre_insert_move(
                    standoff_cm=float(body.get('standoff_cm', 0.5)),
                    insert_depth_cm=float(body.get('insert_depth_cm', 2.0)),
                    duration_s=None if dur is None or dur == '' else float(dur),
                    use_relative_delta=bool(body.get('use_relative_delta', False)),
                )
            elif path == '/api/wrench/tare':
                res = self.node.tare_wrench()
            elif path == '/api/wrench/source':
                res = self.node.update_parameters({'wrench_source': body.get('source', body.get('wrench_source'))})
            elif path == '/api/policy/start':
                if body.get('wrench_source') is not None:
                    self.node.update_parameters({'wrench_source': body['wrench_source']})
                if 'auto_stop_stationary' in body:
                    self.node.auto_stop_stationary = bool(body['auto_stop_stationary'])
                res = self.node.start_policy(str(body.get('controller', 'onnx')))
            elif path == '/api/rule/start':
                if body.get('wrench_source') is not None:
                    self.node.update_parameters({'wrench_source': body['wrench_source']})
                if 'auto_stop_stationary' in body:
                    self.node.auto_stop_stationary = bool(body['auto_stop_stationary'])
                res = self.node.start_policy('rule')
            elif path == '/api/policy/stop':
                res = self.node.stop_policy(str(body.get('mode', 'drag')))
            elif path == '/api/parameters':
                if 'auto_stop_stationary' in body:
                    self.node.auto_stop_stationary = bool(body['auto_stop_stationary'])
                has_other = any(k in body for k in ('tip_offset_m', 'tip_offset_cm', 'tip_length_m', 'tip_length_cm', 'wrench_source'))
                res = self.node.update_parameters(body) if has_other else {
                    'ok': True,
                    'auto_stop_stationary': self.node.auto_stop_stationary,
                    'message': f'已更新自动停机设置：{self.node.auto_stop_stationary}',
                }
            else:
                self._send_json(404, {'ok': False, 'error': f'Unknown endpoint: {path}'})
                return
            self._send_json(200, res)
        except Exception as error:
            with self.node.lock:
                self.node.last_error = str(error)
                self.node.status_message = str(error)
            self.node._log('ERROR', str(error))
            self._send_json(400, {'ok': False, 'error': str(error)})


def create_agent_http_server(
    node: AgentInsertNode,
    host: str = '127.0.0.1',
    port: int = 8093,
) -> ThreadingHTTPServer:
    handler = type(
        'BoundAgentFlowHandler',
        (_AgentFlowHTTPHandler,),
        {'node': node, 'static_dir': agent_web_root_path()},
    )
    return _DaemonThreadingHTTPServer((host, port), handler)


def agent_web_main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Launch the 4-stage Agent-guided Insertion Pipeline Web UI for PiPER-X (arm_4).'
    )
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8093)
    parser.add_argument('--config', default=str(default_config_path()))
    parser.add_argument('--arm', default='can4')
    parser.add_argument('--namespace', default='')
    parser.add_argument('--onnx', default='')
    parser.add_argument('--tip-length', type=float, default=DEFAULT_TIP_LENGTH_M)
    parser.add_argument('--wrench-source', default='paxini', choices=WRENCH_SOURCES)
    parser.add_argument('--head-url', default='http://127.0.0.1:8310')
    parser.add_argument('--llm-url', default=DEFAULT_API_BASE)
    parser.add_argument('--model', default=DEFAULT_MODEL)
    parser.add_argument('--reasoning-effort', default=DEFAULT_REASONING_EFFORT)
    clean_argv = remove_ros_args(list(argv) if argv is not None else None)
    args = parser.parse_args(clean_argv[1:] if argv is None else clean_argv)

    rclpy.init(args=list(argv) if argv is not None else None)
    node = AgentInsertNode(
        config_file=args.config,
        arm_selector=args.arm,
        namespace_override=args.namespace,
        onnx_path=args.onnx or None,
        tip_length_m=args.tip_length,
        wrench_source=args.wrench_source,
        head_camera_url=args.head_url,
        llm_api_base=args.llm_url,
        llm_model=args.model,
        llm_reasoning_effort=args.reasoning_effort,
    )
    server = create_agent_http_server(node, host=args.host, port=args.port)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()
    node.get_logger().info(
        f'Agent Insertion Pipeline Web UI listening at http://{args.host}:{args.port}/ '
        f'(target={node.arm_id} {node.arm_namespace}, model={args.model} {args.reasoning_effort})'
    )

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for step in (
            node.shutdown_streams,
            server.shutdown,
            server.server_close,
            executor.shutdown,
            node.destroy_node,
            rclpy.try_shutdown,
        ):
            try:
                step()
            except BaseException:
                pass
    return 0


if __name__ == '__main__':
    raise SystemExit(agent_web_main())
