"""ChArUco 外参和起始位管理：启动时加载 YAML，更新时原子写回。"""
from __future__ import annotations

import copy
import math
from pathlib import Path
import threading
import time
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

from .config import BoardConfig


def invert_se3(matrix: Sequence[Sequence[float]] | np.ndarray) -> np.ndarray:
    mat = np.asarray(matrix, dtype=np.float64).reshape(4, 4)
    rot = mat[:3, :3]
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rot.T
    out[:3, 3] = -rot.T @ mat[:3, 3]
    return out


def xyz_rpy_deg_to_matrix(xyz_m: Sequence[float], rpy_deg: Sequence[float]) -> np.ndarray:
    rx, ry, rz = (math.radians(float(v)) for v in rpy_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = [
        [cy * cz, cz * sx * sy - cx * sz, cx * cz * sy + sx * sz],
        [cy * sz, cx * cz + sx * sy * sz, cx * sy * sz - cz * sx],
        [-sy, cy * sx, cx * cy],
    ]
    out[:3, 3] = [float(v) for v in xyz_m]
    return out


def matrix_to_xyz_rpy_deg(matrix: Sequence[Sequence[float]] | np.ndarray) -> tuple[list[float], list[float]]:
    mat = np.asarray(matrix, dtype=np.float64).reshape(4, 4)
    r = mat[:3, :3]
    sy = max(-1.0, min(1.0, float(-r[2, 0])))
    pitch = math.asin(sy)
    if abs(sy) > 1.0 - 1e-9:
        roll, yaw = 0.0, math.atan2(float(-r[0, 1]), float(r[1, 1]))
    else:
        roll = math.atan2(float(r[2, 1]), float(r[2, 2]))
        yaw = math.atan2(float(r[1, 0]), float(r[0, 0]))
    xyz = [round(float(v), 5) for v in mat[:3, 3]]
    rpy = [round(math.degrees(v), 3) for v in (roll, pitch, yaw)]
    return xyz, rpy


def detect_charuco_board(
    image_bgr: np.ndarray,
    camera_matrix: Sequence[Sequence[float]] | np.ndarray,
    distortion: Sequence[float] | np.ndarray,
    board_cfg: BoardConfig,
    label: str = "",
) -> dict[str, Any]:
    """Detect ChArUco corners, solve T_cam_board, and return JPEG overlay."""
    import cv2

    img = np.asarray(image_bgr, dtype=np.uint8)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    K = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    dist = np.asarray(distortion if distortion is not None else [], dtype=np.float64).reshape(-1, 1)

    aruco: Any = cv2.aruco
    dict_id = getattr(aruco, board_cfg.dictionary, None)
    if dict_id is None:
        raise ValueError(f"Unsupported ArUco dictionary: {board_cfg.dictionary}")
    dictionary = (
        aruco.getPredefinedDictionary(dict_id)
        if hasattr(aruco, "getPredefinedDictionary")
        else aruco.Dictionary_get(dict_id)
    )
    board = (
        aruco.CharucoBoard((board_cfg.squares_x, board_cfg.squares_y), board_cfg.square_size_m, board_cfg.marker_size_m, dictionary)
        if not hasattr(aruco, "CharucoBoard_create")
        else aruco.CharucoBoard_create(board_cfg.squares_x, board_cfg.squares_y, board_cfg.square_size_m, board_cfg.marker_size_m, dictionary)
    )
    params = aruco.DetectorParameters() if not hasattr(aruco, "DetectorParameters_create") else aruco.DetectorParameters_create()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    all_obj = np.asarray(board.getChessboardCorners() if hasattr(board, "getChessboardCorners") else board.chessboardCorners, dtype=np.float64)

    charuco_corners = charuco_ids = marker_ids = None
    marker_corners: Sequence[Any] = ()
    if hasattr(aruco, "CharucoDetector"):
        c_params = aruco.CharucoParameters()
        c_params.tryRefineMarkers = True
        charuco_corners, charuco_ids, marker_corners, marker_ids = aruco.CharucoDetector(board, c_params, params).detectBoard(gray)
    else:
        m_corners, m_ids, rejected = aruco.detectMarkers(gray, dictionary, parameters=params)
        if m_ids is not None and len(m_ids) > 0:
            m_corners, m_ids, _, _ = aruco.refineDetectedMarkers(gray, board, m_corners, m_ids, rejected)
            marker_corners, marker_ids = list(m_corners), m_ids
            if m_ids is not None and len(m_ids) > 0:
                _, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(m_corners, m_ids, gray, board)

    overlay = img.copy() if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    m_count = 0 if marker_ids is None else int(len(marker_ids))
    c_count = 0 if charuco_ids is None else int(len(charuco_ids))
    if m_count > 0:
        aruco.drawDetectedMarkers(overlay, marker_corners, marker_ids)
    if c_count > 0 and charuco_corners is not None and charuco_ids is not None:
        aruco.drawDetectedCornersCharuco(overlay, charuco_corners, charuco_ids, (0, 215, 255))

    def _encode_overlay(text: str, ok_flag: bool) -> bytes:
        h, w = overlay.shape[:2]
        bh = max(32, int(h * 0.05))
        band = overlay.copy()
        cv2.rectangle(band, (0, 0), (w, bh), (20, 82, 48) if ok_flag else (36, 36, 128), -1)
        cv2.addWeighted(band, 0.78, overlay, 0.22, 0, overlay)
        cv2.putText(overlay, text, (12, int(bh * 0.72)), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (245, 250, 255), 2, cv2.LINE_AA)
        _, buf = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, 86])
        return buf.tobytes()

    if c_count < 6:
        return {
            "ok": False,
            "corner_count": c_count,
            "marker_count": m_count,
            "reason": f"仅检测到 {c_count} 个角点（至少需 6 个）",
            "T_cam_board": None,
            "reproj_rms_px": None,
            "overlay_jpeg": _encode_overlay(f"{label} | Corners {c_count} (<6)", False),
        }

    ids_flat = np.asarray(charuco_ids, dtype=np.int32).reshape(-1)
    corners_2d = np.asarray(charuco_corners, dtype=np.float64).reshape(-1, 2)
    object_3d = all_obj[ids_flat]
    ok, rvec, tvec = cv2.solvePnP(object_3d, corners_2d.reshape(-1, 1, 2), K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return {
            "ok": False,
            "corner_count": c_count,
            "marker_count": m_count,
            "reason": "solvePnP 未收敛",
            "T_cam_board": None,
            "reproj_rms_px": None,
            "overlay_jpeg": _encode_overlay(f"{label} | solvePnP failed", False),
        }

    proj, _ = cv2.projectPoints(object_3d, rvec, tvec, K, dist)
    err = proj.reshape(-1, 2) - corners_2d
    rms_px = float(np.sqrt(np.mean(np.sum(err * err, axis=1))))
    rot, _ = cv2.Rodrigues(rvec)
    t_cam_board = np.eye(4, dtype=np.float64)
    t_cam_board[:3, :3] = rot
    t_cam_board[:3, 3] = tvec.reshape(3)
    cv2.drawFrameAxes(overlay, K, dist, rvec, tvec, 0.075, 3)
    hud = f"{label} | Corners {c_count}/{len(all_obj)} | RMS {rms_px:.2f}px"
    return {
        "ok": True,
        "corner_count": c_count,
        "marker_count": m_count,
        "reason": "",
        "T_cam_board": t_cam_board,
        "reproj_rms_px": round(rms_px, 3),
        "overlay_jpeg": _encode_overlay(hud, True),
    }


class CalibrationManager:
    """Manages workspace extrinsics, TCP offsets, and calibrated arm home poses on disk."""

    def __init__(self, path: Path, board_cfg: BoardConfig, *, default_tcp_z_by_role: Mapping[str, float] | None = None):
        self.path = Path(path)
        self.board_cfg = board_cfg
        self.default_tcp_z = dict(default_tcp_z_by_role or {"left": 0.16483, "right": 0.16483})
        self.lock = threading.RLock()
        self.overlays: dict[str, bytes] = {}
        self.data = self._default_state()
        self.loaded_from_disk = False
        self.load()

    def _default_state(self) -> dict[str, Any]:
        ident = np.eye(4, dtype=np.float64).tolist()
        tcp_left = np.eye(4, dtype=np.float64)
        tcp_left[2, 3] = float(self.default_tcp_z.get("left", 0.16483))
        tcp_right = np.eye(4, dtype=np.float64)
        tcp_right[2, 3] = float(self.default_tcp_z.get("right", 0.16483))
        return {
            "version": 1,
            "calibration_id": "default_uncalibrated",
            "updated_at": None,
            "world_origin": "dual_base_board",  # dual_base_board | board_up | left_base | right_base
            "base_mid_x_m": float(self.board_cfg.base_mid_x_m),
            "board_z_m": float(self.board_cfg.board_z_m),
            "board_observations": {},  # head / left / right -> {matrix_board_from_frame, rms_px, corners, updated_at}
            "world_from_base": {"left": copy.deepcopy(ident), "right": copy.deepcopy(ident)},
            "flange_from_tcp": {"left": tcp_left.tolist(), "right": tcp_right.tolist()},
            "world_from_head_camera_cv": None,
            "home_poses": {},  # arm_id -> {joints_rad: [...], gripper_m: float, updated_at: str}
        }

    def load(self) -> bool:
        with self.lock:
            if not self.path.is_file():
                self.loaded_from_disk = False
                return False
            raw = yaml.safe_load(self.path.read_text(encoding="utf-8")) or {}
            if not isinstance(raw, dict):
                return False
            state = self._default_state()
            for key in ("calibration_id", "updated_at", "world_origin", "base_mid_x_m", "board_z_m"):
                if raw.get(key) is not None:
                    state[key] = raw[key]
            if state["world_origin"] in ("board", "mid_base"):
                state["world_origin"] = "board_up" if state["world_origin"] == "board" else "dual_base_board"
            for group in ("world_from_base", "flange_from_tcp"):
                src = raw.get(group) or {}
                for side in ("left", "right"):
                    if src.get(side) is not None:
                        state[group][side] = np.asarray(src[side], dtype=np.float64).reshape(4, 4).tolist()
            if raw.get("world_from_head_camera_cv") is not None:
                state["world_from_head_camera_cv"] = np.asarray(raw["world_from_head_camera_cv"], dtype=np.float64).reshape(4, 4).tolist()
            if isinstance(raw.get("board_observations"), dict):
                state["board_observations"] = copy.deepcopy(raw["board_observations"])
            if isinstance(raw.get("home_poses"), dict):
                state["home_poses"] = copy.deepcopy(raw["home_poses"])
            self.data = state
            self.loaded_from_disk = True
            return True

    def save(self) -> None:
        with self.lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y-%m-%dT%H:%M:%S")
            self.data["updated_at"] = stamp
            if not self.data.get("calibration_id") or self.data["calibration_id"] == "default_uncalibrated":
                self.data["calibration_id"] = "calib_" + time.strftime("%Y%m%d_%H%M%S")
            tmp = self.path.with_suffix(".yaml.tmp")
            tmp.write_text(yaml.safe_dump(self.data, sort_keys=False, allow_unicode=True), encoding="utf-8")
            tmp.replace(self.path)
            self.loaded_from_disk = True

    def _recompute_world_transforms(self) -> None:
        obs = self.data["board_observations"]
        origin_mode = self.data.get("world_origin", "dual_base_board")
        base_mid_x = float(self.data.get("base_mid_x_m", self.board_cfg.base_mid_x_m))
        board_z = float(self.data.get("board_z_m", self.board_cfg.board_z_m))

        # 板平放且打印面朝上时，OpenCV 的 +Z 朝桌内；固定翻转法向，不按基座高度猜正负。
        board_up = np.diag([1.0, -1.0, -1.0, 1.0])
        board_up[2, 3] = board_z
        t_world_board = board_up

        if origin_mode == "left_base" and "left" in obs:
            t_world_board = invert_se3(obs["left"]["T_board_frame"])
        elif origin_mode == "right_base" and "right" in obs:
            t_world_board = invert_se3(obs["right"]["T_board_frame"])
        elif origin_mode == "dual_base_board" and "left" in obs and "right" in obs:
            p_left = np.asarray(obs["left"]["T_board_frame"], dtype=np.float64)[:3, 3]
            p_right = np.asarray(obs["right"]["T_board_frame"], dtype=np.float64)[:3, 3]
            z_w_in_b = np.array([0.0, 0.0, -1.0])
            v_lr = p_left - p_right
            v_lr_proj = v_lr - float(np.dot(v_lr, z_w_in_b)) * z_w_in_b
            norm_lr = float(np.linalg.norm(v_lr_proj))
            if norm_lr > 1e-4:
                y_w_in_b = v_lr_proj / norm_lr
                x_w_in_b = np.cross(y_w_in_b, z_w_in_b)
                # 双基座连线定义 +Y，板面法向定义 +Z，叉积补全右手世界系。
                r_b_w = np.column_stack((x_w_in_b, y_w_in_b, z_w_in_b))
                r_w_b = r_b_w.T
                mid = 0.5 * (p_left + p_right)
                m_proj = mid - float(np.dot(mid, z_w_in_b)) * z_w_in_b
                t_world_board = np.eye(4, dtype=np.float64)
                t_world_board[:3, :3] = r_w_b
                t_world_board[:3, 3] = np.array([base_mid_x, 0.0, board_z]) - r_w_b @ m_proj

        for side in ("left", "right"):
            if side in obs:
                mat = t_world_board @ np.asarray(obs[side]["T_board_frame"], dtype=np.float64)
                self.data["world_from_base"][side] = mat.tolist()
        if "head" in obs:
            mat_h = t_world_board @ np.asarray(obs["head"]["T_board_frame"], dtype=np.float64)
            self.data["world_from_head_camera_cv"] = mat_h.tolist()
        self.data["calibration_id"] = "calib_" + time.strftime("%Y%m%d_%H%M%S")

    def update_board_observation(
        self,
        target: str,  # "head" | "left" | "right"
        t_board_frame: np.ndarray,
        *,
        corner_count: int,
        reproj_rms_px: float,
        overlay_jpeg: bytes | None = None,
        world_origin: str | None = None,
    ) -> None:
        with self.lock:
            if world_origin in ("dual_base_board", "board_up", "left_base", "right_base"):
                self.data["world_origin"] = world_origin
            xyz, rpy = matrix_to_xyz_rpy_deg(t_board_frame)
            self.data["board_observations"][target] = {
                "T_board_frame": np.asarray(t_board_frame, dtype=np.float64).reshape(4, 4).tolist(),
                "xyz_m": xyz,
                "rpy_deg": rpy,
                "corner_count": int(corner_count),
                "reproj_rms_px": float(reproj_rms_px),
                "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            }
            if overlay_jpeg is not None:
                self.overlays[target] = overlay_jpeg
            self._recompute_world_transforms()
            self.save()

    def set_world_origin_and_tcp(
        self,
        *,
        world_origin: str | None = None,
        base_mid_x_m: float | None = None,
        board_z_m: float | None = None,
        flange_tcp_left_xyz_rpy: tuple[Sequence[float], Sequence[float]] | None = None,
        flange_tcp_right_xyz_rpy: tuple[Sequence[float], Sequence[float]] | None = None,
    ) -> dict[str, Any]:
        with self.lock:
            if base_mid_x_m is not None:
                self.data["base_mid_x_m"] = float(base_mid_x_m)
            if board_z_m is not None:
                self.data["board_z_m"] = float(board_z_m)
            if world_origin in ("dual_base_board", "board_up", "left_base", "right_base"):
                self.data["world_origin"] = world_origin
            if self.data["board_observations"]:
                self._recompute_world_transforms()
            if flange_tcp_left_xyz_rpy is not None:
                self.data["flange_from_tcp"]["left"] = xyz_rpy_deg_to_matrix(*flange_tcp_left_xyz_rpy).tolist()
            if flange_tcp_right_xyz_rpy is not None:
                self.data["flange_from_tcp"]["right"] = xyz_rpy_deg_to_matrix(*flange_tcp_right_xyz_rpy).tolist()
            self.data["calibration_id"] = "calib_" + time.strftime("%Y%m%d_%H%M%S")
            self.save()
            return self.summary()

    def record_home_poses(self, arm_states: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
        """Record current measured arm joint positions & gripper widths to disk as default home."""
        with self.lock:
            stamp = time.strftime("%Y-%m-%dT%H:%M:%S")
            for arm_id, st in arm_states.items():
                joints = [round(float(v), 6) for v in st["joints_rad"][:6]]
                grip = round(float(st.get("gripper_m", 0.0)), 6)
                self.data["home_poses"][arm_id] = {
                    "joints_rad": joints,
                    "gripper_m": grip,
                    "updated_at": stamp,
                }
            self.save()
            return copy.deepcopy(self.data["home_poses"])

    def snapshot_transforms(self) -> dict[str, Any]:
        with self.lock:
            return copy.deepcopy(self.data)

    def overlay_jpeg(self, target: str) -> bytes | None:
        with self.lock:
            return self.overlays.get(target)

    def summary(self) -> dict[str, Any]:
        with self.lock:
            w_base = {}
            f_tcp = {}
            for side in ("left", "right"):
                xyz, rpy = matrix_to_xyz_rpy_deg(self.data["world_from_base"][side])
                w_base[side] = {"xyz_m": xyz, "rpy_deg": rpy}
                txyz, trpy = matrix_to_xyz_rpy_deg(self.data["flange_from_tcp"][side])
                f_tcp[side] = {"xyz_m": txyz, "rpy_deg": trpy}
            head_pose = None
            if self.data["world_from_head_camera_cv"] is not None:
                hxyz, hrpy = matrix_to_xyz_rpy_deg(self.data["world_from_head_camera_cv"])
                head_pose = {"xyz_m": hxyz, "rpy_deg": hrpy}
            obs_summary = {
                k: {kk: vv for kk, vv in v.items() if kk != "T_board_frame"}
                for k, v in self.data["board_observations"].items()
            }
            return {
                "file": str(self.path),
                "loaded_from_disk": self.loaded_from_disk,
                "calibration_id": self.data["calibration_id"],
                "updated_at": self.data["updated_at"],
                "world_origin": self.data["world_origin"],
                "base_mid_x_m": float(self.data.get("base_mid_x_m", self.board_cfg.base_mid_x_m)),
                "board_z_m": float(self.data.get("board_z_m", self.board_cfg.board_z_m)),
                "board": {
                    "squares_x": self.board_cfg.squares_x,
                    "squares_y": self.board_cfg.squares_y,
                    "square_size_mm": round(self.board_cfg.square_size_m * 1000.0, 2),
                    "marker_size_mm": round(self.board_cfg.marker_size_m * 1000.0, 2),
                    "dictionary": self.board_cfg.dictionary,
                },
                "board_observations": obs_summary,
                "has_overlays": list(self.overlays.keys()),
                "world_from_base": w_base,
                "flange_from_tcp": f_tcp,
                "world_from_head": head_pose,
                "home_poses": copy.deepcopy(self.data["home_poses"]),
            }

