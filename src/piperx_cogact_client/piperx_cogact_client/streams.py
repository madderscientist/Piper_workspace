"""Local camera stream pullers (head_camera HTTP + wrist_camera RTSP) and atomic snapshot capture."""
from __future__ import annotations

from dataclasses import dataclass
import json
import threading
import time
from typing import Any
from urllib.parse import urlencode
import urllib.request

import numpy as np

from .calibration import CalibrationManager, invert_se3
from .cogact_api import CameraInput, matrix_to_pose_xyzw
from .config import ArmConfig, CameraConfig
from .kinematics import DriverKinematics
from .ros_io import ArmFeedback, RosArmBridge


@dataclass
class LiveCameraFrame:
    cam_id: str
    role: str
    bgr: np.ndarray | None
    jpeg_bytes: bytes | None
    width: int
    height: int
    sequence: int
    receive_monotonic_ns: int
    receive_unix_ns: int
    intrinsic_matrix: list[list[float]] | None = None
    distortion_coefficients: list[float] | None = None
    distortion_model: str = "unknown"

    def ensure_bgr(self) -> np.ndarray:
        if self.bgr is not None:
            return self.bgr
        if not self.jpeg_bytes:
            raise ValueError(f"Camera {self.cam_id} has no frame data")
        import cv2
        arr = cv2.imdecode(np.frombuffer(self.jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        if arr is None:
            raise ValueError(f"Failed to decode JPEG for camera {self.cam_id}")
        self.bgr = arr
        self.height, self.width = arr.shape[:2]
        return arr

    def ensure_jpeg(self, quality: int = 85) -> bytes:
        if self.jpeg_bytes is not None:
            return self.jpeg_bytes
        if self.bgr is None:
            raise ValueError(f"Camera {self.cam_id} has no frame data")
        import cv2
        ok, buf = cv2.imencode(".jpg", self.bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise RuntimeError(f"Failed to encode JPEG for camera {self.cam_id}")
        self.jpeg_bytes = buf.tobytes()
        return self.jpeg_bytes


@dataclass
class CapturedSnapshot:
    snapshot_id: str
    monotonic_ns: int
    unix_ns: int
    calibration_id: str | None
    cameras: dict[str, CameraInput]       # keyed by role ("head", "wrist_left", "wrist_right")
    preview_jpegs: dict[str, bytes]       # keyed by cam_id for Dashboard snapshot display
    arms_by_role: dict[str, dict[str, Any]]
    arms_by_id: dict[str, ArmFeedback]


def _reshape_k(raw_k) -> list[list[float]] | None:
    if raw_k is None:
        return None
    arr = np.asarray(raw_k, dtype=np.float64)
    if arr.size != 9:
        return None
    mat = arr.reshape(3, 3)
    if mat[0, 0] <= 0.0 or mat[1, 1] <= 0.0:
        return None
    return [[float(v) for v in row] for row in mat]


def _jpeg_dimensions(data: bytes) -> tuple[int, int] | None:
    """Fast pure-Python SOF header parser to read JPEG (width, height) without decoding pixels."""
    n = len(data)
    if n < 4 or data[0] != 0xFF or data[1] != 0xD8:
        return None
    i = 2
    while i + 9 < n:
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        while marker == 0xFF and i + 2 < n:
            i += 1
            marker = data[i + 1]
        if marker in (0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
            i += 2
            continue
        if i + 4 > n:
            break
        seg_len = (data[i + 2] << 8) | data[i + 3]
        if seg_len < 2 or i + 2 + seg_len > n:
            break
        if marker in (0xC0, 0xC1, 0xC2) and i + 9 <= n:
            h = (data[i + 5] << 8) | data[i + 6]
            w = (data[i + 7] << 8) | data[i + 8]
            return (w, h) if w > 0 and h > 0 else None
        i += 2 + seg_len
    return None


class StreamHub:
    """Continuously pulls configured camera streams and builds atomic CogACT snapshots."""

    def __init__(
        self,
        camera_configs: list[CameraConfig],
        arm_configs: list[ArmConfig],
        arm_bridge: RosArmBridge,
        *,
        kinematics: DriverKinematics | None = None,
        start_workers: bool = True,
    ):
        self.camera_configs = {c.id: c for c in camera_configs}
        self.arm_configs = list(arm_configs)
        self.arm_bridge = arm_bridge
        self.kinematics = kinematics or DriverKinematics(arm_configs)
        self._lock = threading.Lock()
        self._frames: dict[str, LiveCameraFrame] = {}
        self._errors: dict[str, str] = {}
        self._latest_snapshot: CapturedSnapshot | None = None
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        if start_workers:
            self.start()

    def start(self) -> None:
        for cfg in self.camera_configs.values():
            target = self._run_wrist_rtsp if cfg.type == "wrist_rtsp" else self._run_http_camera
            t = threading.Thread(target=target, args=(cfg,), name=f"cam-{cfg.id}", daemon=True)
            t.start()
            self._threads.append(t)

    def close(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=3.0)

    def _store_frame(
        self,
        cfg: CameraConfig,
        *,
        bgr: np.ndarray | None,
        jpeg_bytes: bytes | None,
        width: int,
        height: int,
        K: list[list[float]] | None,
        D: list[float] | None,
        dist_model: str,
    ) -> None:
        now_mono = time.monotonic_ns()
        now_unix = time.time_ns()
        with self._lock:
            prev = self._frames.get(cfg.id)
            seq = 1 if prev is None else prev.sequence + 1
            self._frames[cfg.id] = LiveCameraFrame(
                cam_id=cfg.id,
                role=cfg.role,
                bgr=bgr,
                jpeg_bytes=jpeg_bytes,
                width=width,
                height=height,
                sequence=seq,
                receive_monotonic_ns=now_mono,
                receive_unix_ns=now_unix,
                intrinsic_matrix=K,
                distortion_coefficients=list(D or []),
                distortion_model=dist_model,
            )
            self._errors.pop(cfg.id, None)

    def inject_frame(
        self,
        cam_id: str,
        bgr: np.ndarray,
        *,
        intrinsic_matrix: list[list[float]] | None = None,
        distortion_coefficients: list[float] | None = None,
    ) -> None:
        cfg = self.camera_configs[cam_id]
        h, w = bgr.shape[:2]
        self._store_frame(
            cfg, bgr=bgr.copy(), jpeg_bytes=None, width=w, height=h,
            K=intrinsic_matrix, D=distortion_coefficients, dist_model="plumb_bob" if intrinsic_matrix else "unknown",
        )

    def _run_http_camera(self, cfg: CameraConfig) -> None:
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        period = 1.0 / max(0.5, cfg.pull_hz)
        cached_intr: tuple[list[list[float]] | None, list[float], str] = (None, [], "unknown")
        next_intr_t = 0.0
        base = cfg.url.rstrip("/")
        query = ("?" + urlencode({"camera": cfg.camera})) if cfg.camera else ""

        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                if cfg.type == "head_http" and t0 >= next_intr_t:
                    next_intr_t = t0 + 10.0
                    try:
                        with opener.open(f"{base}/api/intrinsic{query}", timeout=2.0) as r:
                            info = json.load(r)
                        if info.get("available"):
                            K = _reshape_k(info.get("camera_matrix"))
                            D = [float(x) for x in (info.get("distortion_coefficients") or [])]
                            cached_intr = (K, D, str(info.get("distortion_model", "plumb_bob")))
                    except Exception:
                        pass
                frame_url = f"{base}/frame/color.jpg{query}" if cfg.type == "head_http" else cfg.url
                with opener.open(frame_url, timeout=2.5) as resp:
                    jpeg = resp.read()
                dims = _jpeg_dimensions(jpeg)
                bgr = None
                if dims is None:
                    import cv2
                    bgr = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if bgr is None:
                        raise RuntimeError("Invalid JPEG stream")
                    dims = (bgr.shape[1], bgr.shape[0])
                self._store_frame(
                    cfg, bgr=bgr, jpeg_bytes=jpeg, width=dims[0], height=dims[1],
                    K=cached_intr[0], D=cached_intr[1], dist_model=cached_intr[2],
                )
            except Exception as exc:
                with self._lock:
                    self._errors[cfg.id] = f"{type(exc).__name__}: {exc}"
            self._stop.wait(max(0.01, period - (time.monotonic() - t0)))

    def _run_wrist_rtsp(self, cfg: CameraConfig) -> None:
        period = 1.0 / max(0.5, cfg.pull_hz)
        cam = None
        cached_size = None
        cached_intr: tuple[list[list[float]] | None, list[float], str] = (None, [], "unknown")
        try:
            from wrist_camera import WristCamera, get_intrinsics
            cam = WristCamera(cfg.device, stream=cfg.stream)
            while not self._stop.is_set():
                t0 = time.monotonic()
                try:
                    bgr = cam.grab(timeout=2.5, fresh=False)
                    h, w = bgr.shape[:2]
                    if (w, h) != cached_size:
                        cached_size = (w, h)
                        try:
                            prof = get_intrinsics(cfg.device, width=w, height=h)
                            K = _reshape_k(prof.get("camera_matrix")) if prof else None
                            D = [float(x) for x in (prof.get("distortion_coefficients") or [])] if prof else []
                            model = str(prof.get("distortion_model", "plumb_bob")) if K else "unknown"
                            cached_intr = (K, D, model)
                        except Exception:
                            cached_intr = (None, [], "unknown")
                    self._store_frame(
                        cfg, bgr=bgr, jpeg_bytes=None, width=w, height=h,
                        K=cached_intr[0], D=cached_intr[1], dist_model=cached_intr[2],
                    )
                except Exception as exc:
                    with self._lock:
                        self._errors[cfg.id] = f"{type(exc).__name__}: {exc}"
                self._stop.wait(max(0.01, period - (time.monotonic() - t0)))
        except Exception as exc:
            with self._lock:
                self._errors[cfg.id] = f"{type(exc).__name__}: {exc}"
        finally:
            if cam is not None:
                try:
                    cam.close()
                except Exception:
                    pass

    def get_live_frame(self, cam_id: str) -> LiveCameraFrame | None:
        with self._lock:
            f = self._frames.get(cam_id)
            if f is None:
                return None
            return LiveCameraFrame(
                cam_id=f.cam_id, role=f.role,
                bgr=None if f.bgr is None else f.bgr.copy(),
                jpeg_bytes=f.jpeg_bytes, width=f.width, height=f.height,
                sequence=f.sequence, receive_monotonic_ns=f.receive_monotonic_ns,
                receive_unix_ns=f.receive_unix_ns,
                intrinsic_matrix=f.intrinsic_matrix,
                distortion_coefficients=list(f.distortion_coefficients or []),
                distortion_model=f.distortion_model,
            )

    def get_preview_jpeg(self, cam_id: str) -> tuple[bytes, int] | None:
        with self._lock:
            f = self._frames.get(cam_id)
            if f is None:
                return None
            jpeg = f.ensure_jpeg(quality=82)
            return jpeg, f.sequence

    def camera_status_list(self) -> list[dict[str, Any]]:
        now_mono = time.monotonic_ns()
        out = []
        with self._lock:
            for cid, cfg in self.camera_configs.items():
                f = self._frames.get(cid)
                err = self._errors.get(cid)
                age_ms = None if f is None else round((now_mono - f.receive_monotonic_ns) / 1e6, 1)
                online = f is not None and age_ms is not None and age_ms < 3000.0
                out.append({
                    "id": cid,
                    "role": cfg.role,
                    "label": cfg.label,
                    "type": cfg.type,
                    "online": online,
                    "sequence": 0 if f is None else f.sequence,
                    "width": None if f is None else f.width,
                    "height": None if f is None else f.height,
                    "age_ms": age_ms,
                    "calibrated_intrinsics": bool(f and f.intrinsic_matrix is not None),
                    "error": err,
                })
        return out

    def capture_snapshot(self, calibration_mgr: CalibrationManager) -> CapturedSnapshot:
        """Atomically capture all cameras + arm joint_states and compute world poses."""
        from wrist_camera import camera_pose_in_base

        mono_ns = time.monotonic_ns()
        unix_ns = time.time_ns()
        cal = calibration_mgr.snapshot_transforms()

        arms_by_id: dict[str, ArmFeedback] = {}
        arms_by_role: dict[str, dict[str, Any]] = {}
        base_from_wrist_cam: dict[str, np.ndarray | None] = {}

        for arm_cfg in self.arm_configs:
            fb = self.arm_bridge.get_feedback(arm_cfg.id)
            if fb is None:
                raise RuntimeError(f"尚未收到机械臂 {arm_cfg.id} ({arm_cfg.joint_state_topic}) 的 joint_state 反馈")
            arms_by_id[arm_cfg.id] = fb
            t_base_flange = self.kinematics.fk_link6(arm_cfg.id, fb.joints_rad)
            w_from_b = np.asarray(cal["world_from_base"][arm_cfg.role], dtype=np.float64)
            f_from_tcp = np.asarray(cal["flange_from_tcp"][arm_cfg.role], dtype=np.float64)
            w_from_tcp = w_from_b @ t_base_flange @ f_from_tcp
            arms_by_role[arm_cfg.role] = {
                "arm_id": arm_cfg.id,
                "joint_positions_rad": list(fb.joints_rad),
                "gripper_opening_m": float(fb.gripper_m),
                "tcp_pose_world_xyzw": matrix_to_pose_xyzw(w_from_tcp),
            }
            try:
                base_from_wrist_cam[arm_cfg.role] = camera_pose_in_base(arm_cfg.id, fb.joints_rad)
            except Exception:
                base_from_wrist_cam[arm_cfg.role] = None

        cam_inputs: dict[str, CameraInput] = {}
        preview_jpegs: dict[str, bytes] = {}
        for cid, cfg in self.camera_configs.items():
            frame = self.get_live_frame(cid)
            if frame is None:
                raise RuntimeError(f"尚未收到相机 {cfg.label} ({cid}) 的画面")
            jpeg_bytes = frame.ensure_jpeg(quality=92)
            preview_jpegs[cid] = jpeg_bytes

            w2c = None
            if cfg.role == "head" and cal.get("world_from_head_camera_cv") is not None:
                w2c = invert_se3(cal["world_from_head_camera_cv"]).tolist()
            elif cfg.role in ("wrist_left", "wrist_right"):
                side = "left" if cfg.role == "wrist_left" else "right"
                t_b_cam = base_from_wrist_cam.get(side)
                if t_b_cam is not None and side in cal["world_from_base"]:
                    t_w_cam = np.asarray(cal["world_from_base"][side], dtype=np.float64) @ t_b_cam
                    w2c = invert_se3(t_w_cam).tolist()

            cam_inputs[cfg.role] = CameraInput(
                role=cfg.role,
                bgr=frame.bgr,
                encoded_bytes=jpeg_bytes,
                mime="image/jpeg",
                image_size_wh=(frame.width, frame.height),
                intrinsic_matrix=frame.intrinsic_matrix,
                distortion_coefficients=frame.distortion_coefficients or [],
                distortion_model=frame.distortion_model,
                world_to_camera_cv=w2c,
            )

        snap = CapturedSnapshot(
            snapshot_id=f"snap_{int(unix_ns // 1_000_000)}",
            monotonic_ns=mono_ns,
            unix_ns=unix_ns,
            calibration_id=cal.get("calibration_id"),
            cameras=cam_inputs,
            preview_jpegs=preview_jpegs,
            arms_by_role=arms_by_role,
            arms_by_id=arms_by_id,
        )
        with self._lock:
            self._latest_snapshot = snap
        return snap

    def latest_snapshot(self) -> CapturedSnapshot | None:
        with self._lock:
            return self._latest_snapshot

