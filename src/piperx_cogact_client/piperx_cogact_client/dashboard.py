"""Layer 3: Local HTTP Web Dashboard for stream monitoring, calibration, and CogACT MIT execution."""
from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import mimetypes
import os
from pathlib import Path
import signal
import threading
import time
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

from piper_x_driver.gravity import default_arm_urdf_path

from .calibration import CalibrationManager, invert_se3, xyz_rpy_deg_to_matrix
from .cogact_api import CogActClient, PreprocessOptions
from .config import ClientConfig, load_client_config
from .engine import ExecutionEngine
from .kinematics import DriverKinematics
from .ros_io import RosArmBridge
from .streams import StreamHub

WEB_DIR = Path(__file__).with_name("web")
DESCRIPTION_DIR = default_arm_urdf_path().parents[1]
URDF_MODELS = {
    "gripper": "piper_description.urdf",
    "gripper_with_wrench": "piper_description_with_wrench.urdf",
    "teaching_handle": "piper_description_with_teach.urdf",
}
STATIC_FILES = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/styles.css": ("styles.css", "text/css; charset=utf-8"),
    "/app.js": ("app.js", "application/javascript; charset=utf-8"),
    "/simulator": ("simulator.html", "text/html; charset=utf-8"),
    "/simulator.css": ("simulator.css", "text/css; charset=utf-8"),
    "/simulator.bundle.js": ("simulator.bundle.js", "application/javascript; charset=utf-8"),
}


def _model_link6_from_driver_link6(model_path: Path) -> list[list[float]]:
    origins = []
    for path in (model_path, default_arm_urdf_path()):
        origin = ET.parse(path).find("joint[@name='joint6']/origin")
        xyz = [float(value) for value in origin.get("xyz", "0 0 0").split()]
        rpy_deg = [math.degrees(float(value)) for value in origin.get("rpy", "0 0 0").split()]
        origins.append(xyz_rpy_deg_to_matrix(xyz, rpy_deg))
    return (invert_se3(origins[0]) @ origins[1]).tolist()


def make_dashboard_handler(engine: ExecutionEngine):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *_args):
            pass

        def send_bytes(self, status: int, body: bytes, content_type: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store, max-age=0")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

        def send_json(self, status: int, payload: dict) -> None:
            body = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8")
            self.send_bytes(status, body, "application/json; charset=utf-8")

        def read_json(self) -> dict:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0:
                return {}
            if length > 4 * 1024 * 1024:
                raise ValueError("请求体过大")
            raw = self.rfile.read(length)
            data = json.loads(raw.decode("utf-8"))
            return data if isinstance(data, dict) else {}

        def _serve_mjpeg(self, cam_id: str) -> None:
            if cam_id not in engine.stream_hub.camera_configs:
                self.send_json(404, {"ok": False, "error": "未知相机"})
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store, max-age=0")
            self.end_headers()
            last_seq = -1
            try:
                while not engine._shutdown.is_set():
                    item = engine.stream_hub.get_preview_jpeg(cam_id)
                    if item is not None and item[1] != last_seq:
                        jpeg, last_seq = item
                        header = (
                            f"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n"
                        ).encode("ascii")
                        self.wfile.write(header)
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                    time.sleep(0.06)
            except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                pass

        def do_GET(self):
            path = urlsplit(self.path).path
            if path in STATIC_FILES:
                fname, mime = STATIC_FILES[path]
                self.send_bytes(200, (WEB_DIR / fname).read_bytes(), mime)
                return
            if path == "/api/state":
                self.send_json(200, {"ok": True, **engine.status_dict()})
                return
            if path == "/api/simulator":
                calibration = engine.calibration.snapshot_transforms()
                arms = []
                for arm in engine.config.arms:
                    model = engine.kinematics.gravity_models.get(arm.id)
                    effector = engine.kinematics.end_effectors.get(arm.id, "gripper_with_wrench")
                    model_name = URDF_MODELS.get(effector, URDF_MODELS["gripper"])
                    arms.append({
                        "id": arm.id, "role": arm.role, "label": arm.label,
                        "urdf_url": "/model-assets/urdf/" + model_name,
                        "model_link6_from_driver_link6": _model_link6_from_driver_link6(DESCRIPTION_DIR / "urdf" / model_name),
                        "world_from_base": calibration["world_from_base"][arm.role],
                        "flange_from_tcp": calibration["flange_from_tcp"][arm.role],
                        "angle_offsets_rad": [model.joint(index).angle_offset if model else 0.0 for index in range(1, 7)],
                    })
                self.send_json(200, {"ok": True, "calibration_id": calibration["calibration_id"],
                                     "board_z_m": calibration["board_z_m"], "arms": arms})
                return
            if path.startswith("/model-assets/"):
                asset = (DESCRIPTION_DIR / unquote(path[len("/model-assets/"):])).resolve()
                if not asset.is_relative_to(DESCRIPTION_DIR.resolve()) or not asset.is_file():
                    self.send_json(404, {"ok": False, "error": "模型资源不存在"})
                    return
                if asset.suffix == ".urdf":
                    self.send_bytes(200, asset.read_bytes(), "application/xml")
                else:
                    self.send_bytes(200, asset.read_bytes(), mimetypes.guess_type(asset.name)[0] or "application/octet-stream")
                return
            if path == "/api/server/health":
                try:
                    info = engine.client.health()
                    self.send_json(200, {"ok": True, "health": info})
                except Exception as exc:
                    self.send_json(502, {"ok": False, "error": str(exc)})
                return
            if path.startswith("/api/cameras/") and path.endswith("/stream.mjpg"):
                cam_id = path[len("/api/cameras/"):-len("/stream.mjpg")]
                self._serve_mjpeg(cam_id)
                return
            if path.startswith("/api/cameras/") and path.endswith("/frame.jpg"):
                cam_id = path[len("/api/cameras/"):-len("/frame.jpg")]
                item = engine.stream_hub.get_preview_jpeg(cam_id)
                if item is None:
                    self.send_bytes(503, b"Camera offline", "text/plain; charset=utf-8")
                else:
                    self.send_bytes(200, item[0], "image/jpeg")
                return
            if path.startswith("/api/snapshot/") and path.endswith(".jpg"):
                cam_id = path[len("/api/snapshot/"):-len(".jpg")]
                snap = engine.stream_hub.latest_snapshot()
                jpeg = None if snap is None else snap.preview_jpegs.get(cam_id)
                if jpeg is None:
                    self.send_bytes(404, b"No snapshot", "text/plain; charset=utf-8")
                else:
                    self.send_bytes(200, jpeg, "image/jpeg")
                return
            if path.startswith("/api/calibration/overlay/") and path.endswith(".jpg"):
                target = path[len("/api/calibration/overlay/"):-len(".jpg")]
                jpeg = engine.calibration.overlay_jpeg(target)
                if jpeg is None:
                    self.send_bytes(404, b"No calibration overlay", "text/plain; charset=utf-8")
                else:
                    self.send_bytes(200, jpeg, "image/jpeg")
                return
            self.send_json(404, {"ok": False, "error": "Not found"})

        def do_POST(self):
            path = urlsplit(self.path).path
            try:
                payload = self.read_json()
                if path == "/api/settings":
                    engine.update_settings(payload)
                    self.send_json(200, {"ok": True, "state": engine.status_dict()})
                elif path == "/api/capture":
                    res = engine.capture_only()
                    self.send_json(200, {"ok": True, **res})
                elif path == "/api/request":
                    chunk = engine.request_once(instruction=payload.get("instruction"))
                    self.send_json(200, {"ok": True, "chunk": chunk.to_summary_dict()})
                elif path == "/api/simulator/preview":
                    snap, chunk = engine.preview_once(instruction=payload.get("instruction"))
                    self.send_json(200, {"ok": True, "chunk": chunk.to_summary_dict(),
                                         "initial_arms": snap.arms_by_role,
                                         "execute_hz": engine.config.execution.execute_hz})
                elif path == "/api/execute/step":
                    res = engine.step_once(instruction=payload.get("instruction"))
                    self.send_json(200, {"ok": True, **res})
                elif path == "/api/execute/batch":
                    fetch_new = bool(payload.get("fetch_new", True))
                    engine.start_batch(fetch_new=fetch_new, instruction=payload.get("instruction"))
                    self.send_json(202, {"ok": True, "mode": engine.mode})
                elif path == "/api/execute/auto":
                    engine.start_auto_loop(instruction=payload.get("instruction"))
                    self.send_json(202, {"ok": True, "mode": engine.mode})
                elif path == "/api/execute/async":
                    engine.start_async_loop(instruction=payload.get("instruction"))
                    self.send_json(202, {"ok": True, "mode": engine.mode})
                elif path == "/api/stop":
                    release_mit = bool(payload.get("release_mit", True))
                    clear_results = bool(payload.get("clear_results", False))
                    engine.stop(release_mit=release_mit, clear_results=clear_results)
                    self.send_json(200, {"ok": True, "mode": engine.mode})
                elif path == "/api/calibration/charuco":
                    targets = payload.get("targets")
                    world_origin = payload.get("world_origin")
                    res = engine.calibrate_charuco(targets=targets, world_origin=world_origin)
                    self.send_json(200, {"ok": True, **res})
                elif path == "/api/calibration/params":
                    left_tcp = payload.get("left_tcp")
                    right_tcp = payload.get("right_tcp")
                    summary = engine.calibration.set_world_origin_and_tcp(
                        world_origin=payload.get("world_origin"),
                        base_mid_x_m=payload.get("base_mid_x_m"),
                        board_z_m=payload.get("board_z_m"),
                        flange_tcp_left_xyz_rpy=(left_tcp["xyz_m"], left_tcp["rpy_deg"]) if left_tcp else None,
                        flange_tcp_right_xyz_rpy=(right_tcp["xyz_m"], right_tcp["rpy_deg"]) if right_tcp else None,
                    )
                    engine.log(f"已更新并落盘坐标参数 -> {engine.calibration.path.name}")
                    self.send_json(200, {"ok": True, "calibration": summary})
                elif path == "/api/calibration/record_home":
                    saved = engine.record_home_poses()
                    self.send_json(200, {"ok": True, "home_poses": saved})
                elif path == "/api/calibration/go_home":
                    engine.move_to_calibrated_home()
                    self.send_json(202, {"ok": True, "mode": engine.mode})
                else:
                    self.send_json(404, {"ok": False, "error": "未知接口"})
            except NotImplementedError as exc:
                self.send_json(501, {"ok": False, "error": str(exc)})
            except Exception as exc:
                self.send_json(400, {"ok": False, "error": str(exc)})

    return Handler


def create_runtime(cfg: ClientConfig, *, enable_ros: bool = True, start_camera_workers: bool = True) -> tuple[ExecutionEngine, RosArmBridge, StreamHub]:
    kinematics = DriverKinematics(cfg.arms)
    default_tcp_z = {arm.role: kinematics.default_flange_to_tcp_z_m(arm.id) for arm in cfg.arms}
    calibration_mgr = CalibrationManager(cfg.calibration_file, cfg.board, default_tcp_z_by_role=default_tcp_z)
    arm_bridge = RosArmBridge(cfg.arms, enable_ros=enable_ros)
    stream_hub = StreamHub(cfg.cameras, cfg.arms, arm_bridge, kinematics=kinematics, start_workers=start_camera_workers)
    client = CogActClient(
        url=cfg.server.url,
        timeout_s=cfg.server.timeout_s,
        seed=cfg.server.seed,
        instruction=cfg.server.instruction,
        preprocess=PreprocessOptions(
            resize_wh=cfg.server.preprocess.resize_wh,
            image_format=cfg.server.preprocess.image_format,
            jpeg_quality=cfg.server.preprocess.jpeg_quality,
        ),
    )
    engine = ExecutionEngine(cfg, arm_bridge, stream_hub, calibration_mgr, client, kinematics=kinematics)
    if calibration_mgr.loaded_from_disk:
        engine.log(f"已加载落盘标定默认值：{calibration_mgr.path} ({calibration_mgr.data['calibration_id']})")
    return engine, arm_bridge, stream_hub


def main(argv=None) -> int:
    for var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ.setdefault(var, "1")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, help="YAML configuration file (defaults to config/default.yaml)")
    parser.add_argument("--calibration-file", type=Path, help="Override path for persisted calibration YAML")
    parser.add_argument("--host", help="Override HTTP bind host")
    parser.add_argument("--port", type=int, help="Override HTTP bind port")
    parser.add_argument("--no-ros", action="store_true", help="Disable ROS 2 node for offline UI debugging")
    args = parser.parse_args(argv)

    cfg = load_client_config(args.config, calibration_file=args.calibration_file)
    if args.host:
        cfg.host = args.host
    if args.port:
        cfg.port = args.port

    stop_event = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop_event.set())

    engine, arm_bridge, stream_hub = create_runtime(cfg, enable_ros=not args.no_ros)
    server = ThreadingHTTPServer((cfg.host, cfg.port), make_dashboard_handler(engine))
    server.daemon_threads = True
    srv_thread = threading.Thread(target=server.serve_forever, name="cogact-http", daemon=True)
    srv_thread.start()
    print(f"PiPER-X CogACT Dashboard running at http://{cfg.host}:{server.server_port}/", flush=True)
    try:
        while not stop_event.wait(0.25):
            pass
    finally:
        server.shutdown()
        server.server_close()
        engine.close()
        stream_hub.close()
        arm_bridge.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

