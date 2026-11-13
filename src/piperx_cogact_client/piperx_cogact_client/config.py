"""Configuration models and persistent calibration path resolution."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


def default_package_config_dir() -> Path:
    # Prefer source tree config/ when editable/symlink-installed so persisted calibration survives rebuilds.
    src_candidate = Path(__file__).resolve().parents[1] / "config"
    if src_candidate.is_dir():
        return src_candidate
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("piperx_cogact_client")) / "config"
    except Exception:
        return src_candidate


@dataclass
class PreprocessConfig:
    resize_wh: tuple[int, int] | None = None
    image_format: str = "keep"  # keep | jpeg | png
    jpeg_quality: int = 92


@dataclass
class ServerConfig:
    url: str = "http://10.209.224.190:8332"
    timeout_s: float = 120.0
    seed: int = 0
    instruction: str = ""
    preprocess: PreprocessConfig = field(default_factory=PreprocessConfig)


@dataclass
class ArmConfig:
    id: str
    role: str  # "left" | "right"
    label: str
    joint_state_topic: str
    mit_topic: str
    gripper_topic: str
    kp: list[float] = field(default_factory=lambda: [-1.0] * 6)
    kd: list[float] = field(default_factory=lambda: [-1.0] * 6)
    gripper_effort: float = 1.5


@dataclass
class CameraConfig:
    id: str
    role: str  # "head" | "wrist_left" | "wrist_right"
    label: str
    type: str  # "head_http" | "wrist_rtsp" | "http_jpeg"
    url: str = "http://127.0.0.1:8310"
    camera: str = ""
    device: str = ""
    stream: str = "stream1"
    pull_hz: float = 15.0


@dataclass
class ExecutionConfig:
    execute_hz: float = 30.0
    settle_delay_s: float = 1.0
    hold_hz: float = 50.0
    home_move_duration_s: float = 2.5
    enable_gripper: bool = True


@dataclass
class IkConfig:
    damping: float = 1e-3
    max_iters: int = 80
    pos_tol_m: float = 5e-4
    rot_tol_rad: float = 5e-3


@dataclass
class BoardConfig:
    squares_x: int = 12
    squares_y: int = 9
    square_size_m: float = 0.015
    marker_size_m: float = 0.01125
    dictionary: str = "DICT_5X5_100"
    base_mid_x_m: float = 0.06
    board_z_m: float = 0.75


@dataclass
class ClientConfig:
    server: ServerConfig = field(default_factory=ServerConfig)
    arms: list[ArmConfig] = field(default_factory=list)
    cameras: list[CameraConfig] = field(default_factory=list)
    execution: ExecutionConfig = field(default_factory=ExecutionConfig)
    ik: IkConfig = field(default_factory=IkConfig)
    board: BoardConfig = field(default_factory=BoardConfig)
    calibration_file: Path = field(default_factory=lambda: default_package_config_dir() / "calibration.yaml")
    host: str = "0.0.0.0"
    port: int = 8095

    def arm_by_role(self, role: str) -> ArmConfig | None:
        return next((a for a in self.arms if a.role == role), None)

    def arm_by_id(self, arm_id: str) -> ArmConfig | None:
        return next((a for a in self.arms if a.id == arm_id), None)


def load_client_config(path: str | Path | None = None, *, calibration_file: str | Path | None = None) -> ClientConfig:
    cfg_dir = default_package_config_dir()
    cfg_path = Path(path) if path else (cfg_dir / "default.yaml")
    raw: dict[str, Any] = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {} if cfg_path.is_file() else {}

    srv_raw = raw.get("server") or {}
    prep_raw = srv_raw.get("preprocess") or {}
    rwh = prep_raw.get("resize_wh")
    preprocess = PreprocessConfig(
        resize_wh=(int(rwh[0]), int(rwh[1])) if isinstance(rwh, (list, tuple)) and len(rwh) == 2 else None,
        image_format=str(prep_raw.get("image_format", "keep")).lower(),
        jpeg_quality=int(prep_raw.get("jpeg_quality", 92)),
    )
    server = ServerConfig(
        url=str(srv_raw.get("url", "http://10.209.224.190:8332")),
        timeout_s=float(srv_raw.get("timeout_s", 120.0)),
        seed=int(srv_raw.get("seed", 0)),
        instruction=str(srv_raw.get("instruction", "")),
        preprocess=preprocess,
    )

    arms_raw = raw.get("arms")
    if not arms_raw:
        arms_raw = [
            {"id": "arm_3", "role": "left", "label": "左臂 (arm_3)"},
            {"id": "arm_4", "role": "right", "label": "右臂 (arm_4)"},
        ]
    arms = []
    for idx, item in enumerate(arms_raw):
        aid = str(item["id"]).strip().lstrip("/")
        role = str(item.get("role") or ("left" if idx == 0 else "right"))
        arms.append(ArmConfig(
            id=aid,
            role=role,
            label=str(item.get("label", aid)),
            joint_state_topic=str(item.get("joint_state_topic", f"/{aid}/joint_states_feedback")),
            mit_topic=str(item.get("mit_topic", f"/{aid}/command/mit")),
            gripper_topic=str(item.get("gripper_topic", f"/{aid}/command/gripper")),
            kp=[float(v) for v in item.get("kp", [-1.0] * 6)],
            kd=[float(v) for v in item.get("kd", [-1.0] * 6)],
            gripper_effort=float(item.get("gripper_effort", 1.5)),
        ))

    cams_raw = raw.get("cameras") or [
        {"id": "head", "role": "head", "label": "头部相机", "type": "head_http", "url": "http://127.0.0.1:8310", "camera": "head_2"},
        {"id": "wrist_left", "role": "wrist_left", "label": "左腕相机", "type": "wrist_rtsp", "device": "arm_3", "stream": "stream1"},
        {"id": "wrist_right", "role": "wrist_right", "label": "右腕相机", "type": "wrist_rtsp", "device": "arm_4", "stream": "stream1"},
    ]
    cameras = [
        CameraConfig(
            id=str(c["id"]),
            role=str(c.get("role", c["id"])),
            label=str(c.get("label", c["id"])),
            type=str(c["type"]),
            url=str(c.get("url", "http://127.0.0.1:8310")),
            camera=str(c.get("camera", "")),
            device=str(c.get("device", "")),
            stream=str(c.get("stream", "stream1")),
            pull_hz=float(c.get("pull_hz", 15.0)),
        )
        for c in cams_raw
    ]

    exe_raw = raw.get("execution") or {}
    execution = ExecutionConfig(
        execute_hz=float(exe_raw.get("execute_hz", 30.0)),
        settle_delay_s=float(exe_raw.get("settle_delay_s", 1.0)),
        hold_hz=float(exe_raw.get("hold_hz", 50.0)),
        home_move_duration_s=float(exe_raw.get("home_move_duration_s", 2.5)),
        enable_gripper=bool(exe_raw.get("enable_gripper", True)),
    )

    ik_raw = raw.get("ik") or {}
    ik = IkConfig(
        damping=float(ik_raw.get("damping", 1e-3)),
        max_iters=int(ik_raw.get("max_iters", 80)),
        pos_tol_m=float(ik_raw.get("pos_tol_m", 5e-4)),
        rot_tol_rad=float(ik_raw.get("rot_tol_rad", 5e-3)),
    )

    cal_raw = raw.get("calibration") or {}
    brd_raw = cal_raw.get("board") or {}
    board = BoardConfig(
        squares_x=int(brd_raw.get("squares_x", 12)),
        squares_y=int(brd_raw.get("squares_y", 9)),
        square_size_m=float(brd_raw.get("square_size_m", 0.015)),
        marker_size_m=float(brd_raw.get("marker_size_m", 0.01125)),
        dictionary=str(brd_raw.get("dictionary", "DICT_5X5_100")),
        base_mid_x_m=float(cal_raw.get("base_mid_x_m", 0.06)),
        board_z_m=float(cal_raw.get("board_z_m", 0.75)),
    )
    cal_file_str = str(calibration_file or cal_raw.get("file") or "").strip()
    cal_path = Path(cal_file_str) if cal_file_str else (cfg_dir / "calibration.yaml")

    dash_raw = raw.get("dashboard") or {}
    return ClientConfig(
        server=server,
        arms=arms,
        cameras=cameras,
        execution=execution,
        ik=ik,
        board=board,
        calibration_file=cal_path,
        host=str(dash_raw.get("host", "0.0.0.0")),
        port=int(dash_raw.get("port", 8095)),
    )

