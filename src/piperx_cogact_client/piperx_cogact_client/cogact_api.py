"""无状态 CogACT 客户端：图像预处理、请求打包、响应解析和位姿换算。

GET /health，POST /predict；请求/响应分别使用 piperx_inference_request_v1 / response_v1。
"""
from __future__ import annotations

import base64
from dataclasses import dataclass, field
import json
import math
import time
from typing import Any, Callable, Mapping, Sequence
import urllib.error
import urllib.request
import uuid

import numpy as np

VIEWS = ("head", "wrist_left", "wrist_right")
REQUEST_SCHEMA = "piperx_inference_request_v1"
RESPONSE_SCHEMA = "piperx_inference_response_v1"
DEFAULT_SERVER_URL = "http://10.209.224.190:8332"
ALLOWED_DISTORTION_LENGTHS: dict[str, set[int]] = {
    "opencv": {0, 4, 5, 8, 12, 14},
    "plumb_bob": {0, 4, 5},
    "rational_polynomial": {0, 8},
    "none": {0},
    "unknown": {0},
}


@dataclass
class PreprocessOptions:
    resize_wh: tuple[int, int] | None = None
    image_format: str = "keep"  # keep | jpeg | png
    jpeg_quality: int = 92


@dataclass
class CameraInput:
    """One camera view input before preprocessing."""
    role: str  # "head" | "wrist_left" | "wrist_right"
    bgr: np.ndarray | None = None
    encoded_bytes: bytes | None = None
    mime: str | None = None
    image_size_wh: tuple[int, int] | None = None
    intrinsic_matrix: list[list[float]] | None = None
    distortion_coefficients: list[float] = field(default_factory=list)
    distortion_model: str = "unknown"
    world_to_camera_cv: list[list[float]] | None = None


@dataclass
class ArmTarget:
    """Decoded single-arm target for one step."""
    tcp_pose_world_xyzw: list[float] | None = None
    rotation_world_from_tcp: list[list[float]] | None = None
    joint_positions_rad: list[float] | None = None
    gripper_opening_m: float | None = None
    ik_converged: bool | None = None
    ik_pos_err_mm: float | None = None
    ik_rot_err_deg: float | None = None


@dataclass
class ActionStep:
    step_index: int
    target_offset_s: float
    left: ArmTarget
    right: ArmTarget

    def by_role(self, role: str) -> ArmTarget:
        return self.left if role == "left" else self.right


@dataclass
class ActionChunk:
    request_id: str
    action_kind: str
    steps: list[ActionStep]
    observation_monotonic_ns: int
    observation_unix_ns: int
    calibration_id: str | None = None
    model: dict[str, Any] | None = None
    server_timing: dict[str, Any] | None = None
    preprocessing: Any = None
    ik_summary: dict[str, Any] | None = None
    latency_ms: float = 0.0

    def to_summary_dict(self) -> dict[str, Any]:
        preview = []
        for s in self.steps:
            def _fmt(t: ArmTarget) -> dict[str, Any]:
                return {
                    "tcp_pose_world_xyzw": [round(v, 5) for v in t.tcp_pose_world_xyzw] if t.tcp_pose_world_xyzw else None,
                    "joint_positions_rad": [round(v, 5) for v in t.joint_positions_rad] if t.joint_positions_rad else None,
                    "gripper_opening_m": None if t.gripper_opening_m is None else round(t.gripper_opening_m, 5),
                    "ik_converged": t.ik_converged,
                    "ik_pos_err_mm": t.ik_pos_err_mm,
                    "ik_rot_err_deg": t.ik_rot_err_deg,
                }
            preview.append({
                "step_index": s.step_index,
                "target_offset_s": round(s.target_offset_s, 4),
                "left": _fmt(s.left),
                "right": _fmt(s.right),
            })
        return {
            "request_id": self.request_id,
            "action_kind": self.action_kind,
            "total_steps": len(self.steps),
            "calibration_id": self.calibration_id,
            "latency_ms": round(self.latency_ms, 1),
            "model": self.model,
            "server_timing": self.server_timing,
            "preprocessing": self.preprocessing,
            "ik_summary": self.ik_summary,
            "steps": preview,
        }


def _vector(value: Sequence[float], length: int, name: str) -> list[float]:
    if (not isinstance(value, (list, tuple, np.ndarray)) or len(value) != length
            or any(isinstance(x, (bool, np.bool_)) or not isinstance(x, (int, float, np.number))
                   or not math.isfinite(float(x)) for x in value)):
        raise ValueError(f"{name} must contain {length} finite numbers")
    return [float(x) for x in value]


def _unit(vec: Sequence[float]) -> list[float]:
    length = math.hypot(*(float(x) for x in vec))
    if not math.isfinite(length) or length <= 1e-8:
        raise ValueError("Cannot normalize a zero/degenerate vector")
    return [float(x) / length for x in vec]


def rotation6d_to_matrix(values: Sequence[float]) -> list[list[float]]:
    """按协议对旋转矩阵前两行做 Gram-Schmidt 正交化，再以叉积恢复第三行。"""
    a = _vector(values, 6, "rotation6d")
    row0 = _unit(a[:3])
    proj = sum(x * y for x, y in zip(row0, a[3:]))
    row1 = _unit([a[i + 3] - proj * row0[i] for i in range(3)])
    row2 = [
        row0[1] * row1[2] - row0[2] * row1[1],
        row0[2] * row1[0] - row0[0] * row1[2],
        row0[0] * row1[1] - row0[1] * row1[0],
    ]
    return [row0, row1, row2]


def matrix_to_xyzw(rotation: Sequence[Sequence[float]] | np.ndarray) -> list[float]:
    """Convert 3x3 proper rotation matrix to canonical [qx, qy, qz, qw] (qw >= 0)."""
    r = [[float(rotation[i][j]) for j in range(3)] for i in range(3)]
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(1.0 + trace)
        q = [(r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s, 0.25 * s]
    else:
        i = max(range(3), key=lambda k: r[k][k])
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(1.0 + r[i][i] - r[j][j] - r[k][k])
        q = [0.0] * 4
        q[i] = 0.25 * s
        q[j] = (r[i][j] + r[j][i]) / s
        q[k] = (r[i][k] + r[k][i]) / s
        q[3] = (r[k][j] - r[j][k]) / s
    q = _unit(q)
    return [-x for x in q] if q[3] < 0.0 else q


def pose_xyzw_to_matrix(pose_xyzw: Sequence[float]) -> np.ndarray:
    p = _vector(pose_xyzw, 7, "pose xyz+xyzw")
    if not math.isclose(math.hypot(*p[3:]), 1.0, abs_tol=1e-5, rel_tol=0.0):
        raise ValueError("Quaternion must have unit norm in xyzw order")
    x, y, z, w = _unit(p[3:])
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]
    mat[:3, 3] = p[:3]
    return mat


def matrix_to_pose_xyzw(transform: Sequence[Sequence[float]] | np.ndarray) -> list[float]:
    mat = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    return [float(mat[0, 3]), float(mat[1, 3]), float(mat[2, 3])] + matrix_to_xyzw(mat[:3, :3])


def rigid_transform(value: Sequence[Sequence[float]] | np.ndarray, name: str = "transform") -> np.ndarray:
    """Validate a 4x4 proper rigid transform (orthonormal rotation with det +1)."""
    if not isinstance(value, (list, tuple, np.ndarray)) or len(value) != 4:
        raise ValueError(f"{name} must be a 4x4 rigid transform")
    rows = [_vector(row, 4, name) for row in value]
    mat = np.asarray(rows, dtype=np.float64)
    if not np.allclose(mat[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6, rtol=0.0):
        raise ValueError(f"{name} has an invalid last row")
    rot = mat[:3, :3]
    if not np.allclose(rot @ rot.T, np.eye(3), atol=1e-5, rtol=0.0) or not math.isclose(float(np.linalg.det(rot)), 1.0, abs_tol=1e-5, rel_tol=0.0):
        raise ValueError(f"{name} rotation is not orthonormal with determinant +1")
    return mat


def inverse_rigid(transform: Sequence[Sequence[float]] | np.ndarray) -> list[list[float]]:
    """Invert a parent_from_child rigid transform into child_from_parent."""
    mat = rigid_transform(transform, "transform")
    rot_t = mat[:3, :3].T
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rot_t
    out[:3, 3] = -rot_t @ mat[:3, 3]
    return out.tolist()


def tcp_world_from_reported_pose(pose_in_base_xyzw, world_from_base, reported_from_tcp) -> list[float]:
    """world_from_tcp = world_from_base @ base_from_reported @ reported_from_tcp."""
    w_b = rigid_transform(world_from_base, "world_from_base")
    b_r = pose_xyzw_to_matrix(pose_in_base_xyzw)
    r_t = rigid_transform(reported_from_tcp, "reported_from_tcp")
    # 工具偏移随末端姿态旋转，不能直接在世界系中加减固定 XYZ。
    return matrix_to_pose_xyzw(w_b @ b_r @ r_t)


def reported_pose_from_world_tcp(tcp_pose_world_xyzw, world_from_base, reported_from_tcp) -> list[float]:
    """base_from_reported = inv(world_from_base) @ world_from_tcp @ inv(reported_from_tcp)."""
    b_w = np.asarray(inverse_rigid(world_from_base))
    w_t = pose_xyzw_to_matrix(tcp_pose_world_xyzw)
    t_r = np.asarray(inverse_rigid(reported_from_tcp))
    return matrix_to_pose_xyzw(b_w @ w_t @ t_r)


def _resolve_distortion_model(k_present: bool, dist_model: str, d_len: int) -> str:
    if not k_present:
        if d_len != 0:
            raise ValueError("Nonempty distortion_coefficients requires intrinsic_matrix K")
        return "unknown"
    model = str(dist_model or "").strip().lower()
    if not model or model == "unknown":
        if d_len == 0:
            model = "none"
        elif d_len in (4, 5):
            model = "plumb_bob"
        elif d_len == 8:
            model = "rational_polynomial"
        elif d_len in (12, 14):
            model = "opencv"
        else:
            raise ValueError(f"Unsupported distortion_coefficients length {d_len}")
    allowed = ALLOWED_DISTORTION_LENGTHS.get(model)
    if allowed is None or model == "unknown" or d_len not in allowed:
        raise ValueError(f"Invalid distortion_model={model!r} with len(D)={d_len} when K is provided")
    return model


def preprocess_camera(cam: CameraInput, options: PreprocessOptions | None = None) -> dict[str, Any]:
    """Resize/encode camera image and synchronize intrinsics metadata for CogACT."""
    if cam.role not in VIEWS:
        raise ValueError(f"Unsupported camera role {cam.role!r}; expected {VIEWS}")
    opts = options or PreprocessOptions()

    K = None
    if cam.intrinsic_matrix is not None:
        K = np.asarray([_vector(row, 3, f"{cam.role}.K") for row in cam.intrinsic_matrix], dtype=np.float64)
        if K[0, 0] <= 0.0 or K[1, 1] <= 0.0 or not np.allclose(K[2], [0.0, 0.0, 1.0], atol=1e-6, rtol=0.0):
            raise ValueError(f"Invalid pinhole intrinsic_matrix K for {cam.role}")
    D = _vector(cam.distortion_coefficients or [], len(cam.distortion_coefficients or []), f"{cam.role}.D")
    dist_model = str(cam.distortion_model or "")

    need_resize = opts.resize_wh is not None
    need_reencode = opts.image_format in ("jpeg", "png")

    if not need_resize and not need_reencode and cam.encoded_bytes and cam.mime in ("image/jpeg", "image/png") and cam.image_size_wh:
        encoded = cam.encoded_bytes
        mime = cam.mime
        width, height = int(cam.image_size_wh[0]), int(cam.image_size_wh[1])
    else:
        import cv2
        bgr = cam.bgr
        if bgr is None:
            if not cam.encoded_bytes:
                raise ValueError(f"Camera {cam.role} has neither BGR pixels nor encoded bytes")
            bgr = cv2.imdecode(np.frombuffer(cam.encoded_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                raise ValueError(f"Failed to decode image bytes for camera {cam.role}")
        bgr = np.asarray(bgr)
        if bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError(f"Camera {cam.role} image must be 3-channel uint8 color (got shape={bgr.shape}, dtype={bgr.dtype})")
        src_h, src_w = bgr.shape[:2]
        if opts.resize_wh is not None and (src_w, src_h) != tuple(opts.resize_wh):
            dst_w, dst_h = int(opts.resize_wh[0]), int(opts.resize_wh[1])
            # Per inference_v2.md: never resize a distorted image while keeping raw D.
            if K is not None and any(abs(v) > 0.0 for v in D):
                bgr = cv2.undistort(bgr, K, np.asarray(D, dtype=np.float64))
                D = []
                dist_model = "none"
            interp = cv2.INTER_AREA if dst_w <= src_w else cv2.INTER_LINEAR
            bgr = cv2.resize(bgr, (dst_w, dst_h), interpolation=interp)
            if K is not None:
                sx, sy = dst_w / float(src_w), dst_h / float(src_h)
                K[0, 0] *= sx
                K[0, 2] *= sx
                K[1, 1] *= sy
                K[1, 2] *= sy
        height, width = bgr.shape[:2]
        fmt = opts.image_format if opts.image_format in ("jpeg", "png") else ("png" if cam.mime == "image/png" else "jpeg")
        if fmt == "png":
            ok, buf = cv2.imencode(".png", bgr)
            mime = "image/png"
        else:
            ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, int(opts.jpeg_quality)])
            mime = "image/jpeg"
        if not ok:
            raise RuntimeError(f"Failed to encode {fmt} for camera {cam.role}")
        encoded = buf.tobytes()

    w2c = None
    if cam.world_to_camera_cv is not None:
        w2c = [[float(v) for v in row] for row in rigid_transform(cam.world_to_camera_cv, f"{cam.role}.world_to_camera_cv")]
    k_list = None if K is None else [[float(v) for v in row] for row in K]
    dist_model = _resolve_distortion_model(k_list is not None, dist_model, len(D))
    valid = k_list is not None and w2c is not None
    return {
        "name": cam.role,
        "image_base64": base64.b64encode(encoded).decode("ascii"),
        "mime": mime,
        "image_size_wh": [width, height],
        "intrinsic_matrix": k_list,
        "distortion_coefficients": D,
        "distortion_model": dist_model,
        "world_to_camera_cv": w2c,
        "calibration_valid": bool(valid),
    }


def build_request(
    *,
    instruction: str,
    observation_monotonic_ns: int,
    observation_unix_ns: int,
    arms: Mapping[str, Mapping[str, Any]],
    cameras: Mapping[str, CameraInput],
    calibration_id: str | None = None,
    seed: int = 0,
    request_id: str | None = None,
    preprocess: PreprocessOptions | None = None,
) -> dict[str, Any]:
    """Pack dual-arm state and three camera views into piperx_inference_request_v1."""
    if not isinstance(instruction, str) or not instruction.strip():
        raise ValueError("instruction 不能为空")
    for name, val in (("observation_monotonic_ns", observation_monotonic_ns), ("observation_unix_ns", observation_unix_ns)):
        if type(val) is not int or val < 0:
            raise ValueError(f"{name} must be a nonnegative integer nanosecond timestamp")
    if type(seed) is not int or not 0 <= seed < 2**63:
        raise ValueError("seed must be an integer in [0, 2**63)")
    joint14: list[float] = []
    poses: list[list[float]] = []
    for side in ("left", "right"):
        if side not in arms:
            raise ValueError(f"Missing arm state for role {side!r}")
        arm = arms[side]
        joint14.extend(_vector(arm["joint_positions_rad"], 6, f"{side}.joint_positions_rad"))
        joint14.extend(_vector([arm["gripper_opening_m"]], 1, f"{side}.gripper_opening_m"))
        pose = _vector(arm["tcp_pose_world_xyzw"], 7, f"{side}.tcp_pose_world_xyzw")
        pose_xyzw_to_matrix(pose)
        poses.append(pose)
    cam_payloads = [preprocess_camera(cameras[role], preprocess) for role in VIEWS]
    return {
        "schema": REQUEST_SCHEMA,
        "request_id": request_id or uuid.uuid4().hex,
        "observation_monotonic_ns": observation_monotonic_ns,
        "observation_unix_ns": observation_unix_ns,
        "instruction": instruction.strip(),
        "seed": seed,
        "observation": {
            "joint14": joint14,
            "tcp_pose_world_xyzw": poses,
            "cameras": cam_payloads,
            "calibration_id": calibration_id,
        },
    }


def check_health(url: str = DEFAULT_SERVER_URL, timeout_s: float = 5.0) -> dict[str, Any]:
    """GET <url>/health bypassing environment HTTP proxies."""
    req = urllib.request.Request(url.rstrip("/") + "/health", method="GET")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(req, timeout=timeout_s) as resp:
        return json.load(resp)


def predict_http(request: Mapping[str, Any], url: str = DEFAULT_SERVER_URL, timeout_s: float = 120.0) -> dict[str, Any]:
    """POST <url>/predict bypassing environment HTTP proxies."""
    body = json.dumps(request, allow_nan=False).encode("utf-8")
    req = urllib.request.Request(
        url.rstrip("/") + "/predict",
        data=body,
        headers={"Content-Type": "application/json", "Accept": "application/json"},
    )
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(req, timeout=timeout_s) as resp:
            return json.load(resp)
    except urllib.error.HTTPError as error:
        detail = error.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"CogACT HTTP {error.code}: {detail}") from error


def parse_response(response: Mapping[str, Any], request: Mapping[str, Any], *, latency_ms: float = 0.0) -> ActionChunk:
    """Validate and decode piperx_inference_response_v1 (supports tcp_world_absolute20 and joint_absolute14)."""
    if not isinstance(response, Mapping) or response.get("schema") != RESPONSE_SCHEMA:
        raise ValueError("Unexpected inference response schema")
    if (not isinstance(response.get("request_id"), str) or not response["request_id"]
            or type(response.get("observation_monotonic_ns")) is not int or response["observation_monotonic_ns"] < 0):
        raise ValueError("Response has invalid request_id or observation_monotonic_ns")
    for key in ("request_id", "observation_monotonic_ns"):
        if response.get(key) != request.get(key):
            raise ValueError(f"Response {key} does not match request")
    kind = str(response.get("action_kind", ""))
    actions = response.get("actions")
    offsets = response.get("target_offsets_s")
    if not isinstance(actions, list) or not actions or not isinstance(offsets, list) or len(actions) != len(offsets):
        raise ValueError("Invalid actions / target_offsets_s in response")
    checked_offsets = _vector(offsets, len(actions), "target_offsets_s")

    steps: list[ActionStep] = []
    if kind == "tcp_world_absolute20":
        if len(actions) != 30:
            raise ValueError("Expected exactly 30 actions for tcp_world_absolute20")
        if response.get("calibration_id") != request["observation"].get("calibration_id"):
            raise ValueError("Response calibration_id does not match request")
        for idx, (raw_row, offset) in enumerate(zip(actions, checked_offsets), start=1):
            if abs(offset - idx / 30.0) > 1e-8:
                raise ValueError("Expected observation-relative offsets k/30 seconds")
            row = _vector(raw_row, 20, f"action {idx}")
            left_rot = rotation6d_to_matrix(row[3:9])
            right_rot = rotation6d_to_matrix(row[13:19])
            steps.append(ActionStep(
                step_index=idx,
                target_offset_s=offset,
                left=ArmTarget(
                    tcp_pose_world_xyzw=row[0:3] + matrix_to_xyzw(left_rot),
                    rotation_world_from_tcp=left_rot,
                    gripper_opening_m=row[9],
                ),
                right=ArmTarget(
                    tcp_pose_world_xyzw=row[10:13] + matrix_to_xyzw(right_rot),
                    rotation_world_from_tcp=right_rot,
                    gripper_opening_m=row[19],
                ),
            ))
    elif kind == "joint_absolute14":
        for idx, (raw_row, offset) in enumerate(zip(actions, checked_offsets), start=1):
            row = _vector(raw_row, 14, f"action {idx}")
            steps.append(ActionStep(
                step_index=idx,
                target_offset_s=offset,
                left=ArmTarget(joint_positions_rad=row[0:6], gripper_opening_m=row[6]),
                right=ArmTarget(joint_positions_rad=row[7:13], gripper_opening_m=row[13]),
            ))
    else:
        raise ValueError(f"Unsupported action_kind: {kind!r}")

    return ActionChunk(
        request_id=str(response["request_id"]),
        action_kind=kind,
        steps=steps,
        observation_monotonic_ns=int(request["observation_monotonic_ns"]),
        observation_unix_ns=int(request["observation_unix_ns"]),
        calibration_id=response.get("calibration_id"),
        model=response.get("model"),
        server_timing=response.get("timing"),
        preprocessing=response.get("preprocessing"),
        latency_ms=float(latency_ms),
    )


def request_cogact(
    *,
    url: str = DEFAULT_SERVER_URL,
    instruction: str,
    observation_monotonic_ns: int,
    observation_unix_ns: int,
    arms: Mapping[str, Mapping[str, Any]],
    cameras: Mapping[str, CameraInput],
    calibration_id: str | None = None,
    seed: int = 0,
    timeout_s: float = 120.0,
    preprocess: PreprocessOptions | None = None,
    transport: Callable[[dict[str, Any], str, float], dict[str, Any]] | None = None,
) -> tuple[ActionChunk, dict[str, Any], dict[str, Any]]:
    """Unified stateless CogACT request entry point -> (parsed_chunk, raw_request, raw_response)."""
    req = build_request(
        instruction=instruction,
        observation_monotonic_ns=observation_monotonic_ns,
        observation_unix_ns=observation_unix_ns,
        arms=arms,
        cameras=cameras,
        calibration_id=calibration_id,
        seed=seed,
        preprocess=preprocess,
    )
    sender = predict_http if transport is None else transport
    t0 = time.monotonic()
    resp = sender(req, url, timeout_s)
    latency_ms = (time.monotonic() - t0) * 1000.0
    chunk = parse_response(resp, req, latency_ms=latency_ms)
    return chunk, req, resp


class CogActClient:
    """Configurable client wrapper around request_cogact."""

    def __init__(
        self,
        url: str = DEFAULT_SERVER_URL,
        *,
        timeout_s: float = 120.0,
        seed: int = 0,
        instruction: str = "",
        preprocess: PreprocessOptions | None = None,
        transport: Callable[[dict[str, Any], str, float], dict[str, Any]] | None = None,
    ):
        self.url = url
        self.timeout_s = timeout_s
        self.seed = seed
        self.instruction = instruction
        self.preprocess = preprocess or PreprocessOptions()
        self.transport = transport

    def health(self) -> dict[str, Any]:
        return check_health(self.url, timeout_s=min(5.0, self.timeout_s))

    def predict(
        self,
        *,
        observation_monotonic_ns: int,
        observation_unix_ns: int,
        arms: Mapping[str, Mapping[str, Any]],
        cameras: Mapping[str, CameraInput],
        calibration_id: str | None = None,
        instruction: str | None = None,
        seed: int | None = None,
    ) -> ActionChunk:
        text = self.instruction if instruction is None else instruction
        chunk, _, _ = request_cogact(
            url=self.url,
            instruction=text,
            observation_monotonic_ns=observation_monotonic_ns,
            observation_unix_ns=observation_unix_ns,
            arms=arms,
            cameras=cameras,
            calibration_id=calibration_id,
            seed=self.seed if seed is None else seed,
            timeout_s=self.timeout_s,
            preprocess=self.preprocess,
            transport=self.transport,
        )
        return chunk

