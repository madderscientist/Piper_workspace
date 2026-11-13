from __future__ import annotations

from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pytest

from piper_x_driver import forward_kinematics
from piper_x_driver.gravity import default_arm_urdf_path
from piperx_cogact_client.calibration import CalibrationManager, detect_charuco_board, xyz_rpy_deg_to_matrix
from piperx_cogact_client.cogact_api import (
    ActionChunk,
    ActionStep,
    ArmTarget,
    CameraInput,
    CogActClient,
    PreprocessOptions,
    pose_xyzw_to_matrix,
    preprocess_camera,
    reported_pose_from_world_tcp,
    request_cogact,
    tcp_world_from_reported_pose,
)
from piperx_cogact_client.config import BoardConfig, load_client_config
from piperx_cogact_client.dashboard import make_dashboard_handler
from piperx_cogact_client.engine import ExecutionEngine, QueuedAction, blend_action_queues
from piperx_cogact_client.kinematics import DriverKinematics
from piperx_cogact_client.ros_io import RosArmBridge
from piperx_cogact_client.streams import StreamHub


def _render_synthetic_charuco(board_cfg: BoardConfig) -> tuple[np.ndarray, list[list[float]], list[float]]:
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, board_cfg.dictionary))
    board = (
        aruco.CharucoBoard((board_cfg.squares_x, board_cfg.squares_y), board_cfg.square_size_m, board_cfg.marker_size_m, dictionary)
        if not hasattr(aruco, "CharucoBoard_create")
        else aruco.CharucoBoard_create(board_cfg.squares_x, board_cfg.squares_y, board_cfg.square_size_m, board_cfg.marker_size_m, dictionary)
    )
    board_img = board.generateImage((720, 540), marginSize=30) if hasattr(board, "generateImage") else board.draw((720, 540), marginSize=30)
    bgr = cv2.cvtColor(board_img, cv2.COLOR_GRAY2BGR)
    K = [[700.0, 0.0, 360.0], [0.0, 700.0, 270.0], [0.0, 0.0, 1.0]]
    return bgr, K, [0.0, 0.0, 0.0, 0.0, 0.0]


def test_cogact_api_preprocess_and_protocol():
    raw = np.zeros((480, 640, 3), dtype=np.uint8)
    raw[:, :, 1] = 180
    K = [[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]]
    resized = preprocess_camera(
        CameraInput(role="head", bgr=raw, intrinsic_matrix=K, distortion_coefficients=[0.01, -0.01, 0.0, 0.0, 0.0]),
        PreprocessOptions(resize_wh=(320, 240), image_format="jpeg"),
    )
    assert resized["image_size_wh"] == [320, 240]
    assert resized["distortion_coefficients"] == []
    assert resized["distortion_model"] == "none"
    assert np.isclose(resized["intrinsic_matrix"][0][0], 300.0)

    w_b = [[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]]
    r_t = [[1, 0, 0, 0.2], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    base_pose = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    world_pose = tcp_world_from_reported_pose(base_pose, w_b, r_t)
    assert np.allclose(world_pose[:3], [1.0, 2.3, 3.0], atol=1e-6)
    assert np.allclose(reported_pose_from_world_tcp(world_pose, w_b, r_t), base_pose, atol=1e-6)

    def fake_tcp20_server(req: dict, _url: str, _timeout: float) -> dict:
        assert req["schema"] == "piperx_inference_request_v1"
        assert len(req["observation"]["joint14"]) == 14
        assert [c["name"] for c in req["observation"]["cameras"]] == ["head", "wrist_left", "wrist_right"]
        row = [0.3, 0.2, 0.1, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.04,
               0.3, -0.2, 0.1, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.05]
        return {
            "schema": "piperx_inference_response_v1",
            "request_id": req["request_id"],
            "observation_monotonic_ns": req["observation_monotonic_ns"],
            "calibration_id": req["observation"]["calibration_id"],
            "action_kind": "tcp_world_absolute20",
            "actions": [row for _ in range(30)],
            "target_offsets_s": [k / 30 for k in range(1, 31)],
            "model": {"variant": "tcpstate20_abs_measured_opening_m"},
            "preprocessing": [{"name": "head", "calibration_valid": False}],
        }

    arms = {
        side: {
            "joint_positions_rad": [0.0] * 6,
            "gripper_opening_m": 0.04,
            "tcp_pose_world_xyzw": [0.3, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
        }
        for side in ("left", "right")
    }
    cams = {role: CameraInput(role=role, bgr=raw, intrinsic_matrix=K) for role in ("head", "wrist_left", "wrist_right")}
    chunk, _, _ = request_cogact(
        instruction="lift towel",
        observation_monotonic_ns=1000,
        observation_unix_ns=2000,
        arms=arms,
        cameras=cams,
        calibration_id="cal_01",
        transport=fake_tcp20_server,
    )
    assert chunk.action_kind == "tcp_world_absolute20" and len(chunk.steps) == 30
    assert chunk.steps[0].left.tcp_pose_world_xyzw == [0.3, 0.2, 0.1, 0.0, 0.0, 0.0, 1.0]
    assert chunk.steps[0].right.gripper_opening_m == 0.05


@pytest.mark.parametrize("base_z_in_board", [-0.10, 0.10])
def test_calibration_persistence_and_default_reload(tmp_path: Path, base_z_in_board: float):
    board_cfg = BoardConfig()
    bgr, K, D = _render_synthetic_charuco(board_cfg)
    det = detect_charuco_board(bgr, K, D, board_cfg, label="head")
    assert det["ok"] is True and det["corner_count"] >= 20

    cal_file = tmp_path / "calibration.yaml"
    mgr = CalibrationManager(cal_file, board_cfg)
    assert mgr.loaded_from_disk is False
    mgr.update_board_observation(
        "head", np.linalg.inv(det["T_cam_board"]),
        corner_count=det["corner_count"], reproj_rms_px=det["reproj_rms_px"], overlay_jpeg=det["overlay_jpeg"],
    )
    t_b_left = np.eye(4)
    t_b_left[:3, 3] = [-0.30, 0.30, base_z_in_board]
    t_b_right = np.eye(4)
    t_b_right[:3, 3] = [-0.30, -0.30, base_z_in_board]
    mgr.update_board_observation("left", t_b_left, corner_count=40, reproj_rms_px=0.2)
    mgr.update_board_observation("right", t_b_right, corner_count=40, reproj_rms_px=0.2)
    snap_cal = mgr.snapshot_transforms()
    wl = np.asarray(snap_cal["world_from_base"]["left"])
    wr = np.asarray(snap_cal["world_from_base"]["right"])
    assert np.allclose(wl[:3, 3], [0.06, 0.30, board_cfg.board_z_m - base_z_in_board], atol=1e-5)
    assert np.allclose(wr[:3, 3], [0.06, -0.30, board_cfg.board_z_m - base_z_in_board], atol=1e-5)
    assert np.allclose(wl[:3, :3], np.diag([-1.0, 1.0, -1.0]), atol=1e-5)

    mgr.record_home_poses({
        "arm_3": {"joints_rad": [0.1, 0.2, -0.3, 0.0, 0.1, 0.0], "gripper_m": 0.035},
        "arm_4": {"joints_rad": [-0.1, 0.2, -0.3, 0.0, 0.1, 0.0], "gripper_m": 0.04},
    })

    reloaded = CalibrationManager(cal_file, board_cfg)
    summary = reloaded.summary()
    assert reloaded.loaded_from_disk is True
    assert summary["world_from_head"] is not None
    assert summary["home_poses"]["arm_3"]["joints_rad"][0] == pytest.approx(0.1)


def test_blend_action_queues_linear_weights():
    old_q = [
        QueuedAction(arms={"arm_3": ([1.0] * 6, 0.02)}, chunk_id="c1", step_index=i)
        for i in range(1, 3)
    ]
    new_q = [
        QueuedAction(arms={"arm_3": ([2.0] * 6, 0.06)}, chunk_id="c2", step_index=i)
        for i in range(1, 5)
    ]
    merged, overlap = blend_action_queues(old_q, new_q)
    assert overlap == 2 and len(merged) == 4
    # Virtual endpoints (1,0) and (0,1) -> overlap=2 weights are (2/3, 1/3) and (1/3, 2/3), then pure new (0, 1).
    expected_q = [1.0 * (2 / 3) + 2.0 * (1 / 3), 1.0 * (1 / 3) + 2.0 * (2 / 3), 2.0, 2.0]
    expected_g = [0.02 * (2 / 3) + 0.06 * (1 / 3), 0.02 * (1 / 3) + 0.06 * (2 / 3), 0.06, 0.06]
    for item, eq, eg in zip(merged, expected_q, expected_g):
        assert item.arms["arm_3"][0][0] == pytest.approx(eq)
        assert item.arms["arm_3"][1] == pytest.approx(eg)


@pytest.mark.parametrize("effector", ["gripper", "gripper_with_wrench", "teaching_handle"])
def test_simulator_resources_are_read_only(tmp_path: Path, effector: str):
    cfg = load_client_config(calibration_file=tmp_path / "calib.yaml")
    calibration = CalibrationManager(cfg.calibration_file, cfg.board)
    engine = SimpleNamespace(
        config=cfg, calibration=calibration,
        kinematics=SimpleNamespace(gravity_models={}, end_effectors={arm.id: effector for arm in cfg.arms}),
        arm_bridge=Mock(),
    )
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_dashboard_handler(engine))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        with urllib.request.urlopen(base + "/api/simulator", timeout=5) as response:
            simulator = json.load(response)
        assert [arm["id"] for arm in simulator["arms"]] == [arm.id for arm in cfg.arms]
        assert np.allclose(simulator["arms"][0]["world_from_base"], calibration.data["world_from_base"]["left"])
        assert simulator["arms"][0]["angle_offsets_rad"] == [0.0] * 6
        with urllib.request.urlopen(base + simulator["arms"][0]["urdf_url"], timeout=5) as response:
            model_bytes = response.read()
        model_name = simulator["arms"][0]["urdf_url"].rsplit("/", 1)[-1]
        assert model_bytes == (default_arm_urdf_path().parent / model_name).read_bytes()
        robot = ET.fromstring(model_bytes)
        assert robot.tag == "robot" and robot.find("joint[@name='joint6']") is not None
        correction = np.asarray(simulator["arms"][0]["model_link6_from_driver_link6"])
        for joints in ([0.0] * 6, [0.2, 0.7, -0.9, 0.1, -0.2, 0.37]):
            model_link6 = np.eye(4)
            for index, angle in enumerate(joints, 1):
                joint = robot.find(f"joint[@name='joint{index}']")
                origin = joint.find("origin")
                xyz = [float(value) for value in origin.get("xyz", "0 0 0").split()]
                rpy = np.degrees([float(value) for value in origin.get("rpy", "0 0 0").split()])
                rotation = np.eye(4)
                axis = np.asarray([float(value) for value in joint.find("axis").get("xyz").split()])
                rotation[:3, :3] = cv2.Rodrigues(axis * angle)[0]
                model_link6 = model_link6 @ xyz_rpy_deg_to_matrix(xyz, rpy) @ rotation
            assert np.allclose(model_link6 @ correction, forward_kinematics(joints), atol=1e-10)
        with urllib.request.urlopen(base + "/model-assets/meshes/base_link.STL", timeout=5) as response:
            assert len(response.read()) > 84
        engine.arm_bridge.publish_mit.assert_not_called()
    finally:
        server.shutdown()
        server.server_close()


def test_preview_always_captures_without_replacing_execution(tmp_path: Path):
    cfg = load_client_config(calibration_file=tmp_path / "calib.yaml")
    cfg.execution.hold_hz = 0
    bridge = RosArmBridge(cfg.arms, enable_ros=False)
    calibration = CalibrationManager(cfg.calibration_file, cfg.board)
    kinematics = Mock(spec=DriverKinematics)
    hub = Mock(spec=StreamHub, kinematics=kinematics)
    hub.capture_snapshot.side_effect = [
        SimpleNamespace(monotonic_ns=index, unix_ns=index, arms_by_role={}, arms_by_id={},
                        cameras={}, calibration_id="preview") for index in (1, 2)
    ]
    client = Mock(spec=CogActClient)
    chunks = [ActionChunk(str(index), "joint_absolute14", [], index, index) for index in (1, 2)]
    client.predict.side_effect = chunks
    engine = ExecutionEngine(cfg, bridge, hub, calibration, client, kinematics=kinematics)
    cached = ActionChunk("execution", "joint_absolute14", [], 0, 0)
    engine.current_chunk = cached
    try:
        assert engine.preview_once()[1] is chunks[0]
        assert engine.preview_once()[1] is chunks[1]
        assert hub.capture_snapshot.call_count == client.predict.call_count == 2
        assert [call.kwargs["observation_monotonic_ns"] for call in client.predict.call_args_list] == [1, 2]
        assert engine.current_chunk is cached and engine.mode == "idle"
        assert bridge.last_sent_info("arm_3") is None
    finally:
        engine.close()
        bridge.close()


def test_driver_tcp_ik_and_three_dashboard_modes(tmp_path: Path):
    cfg = load_client_config(calibration_file=tmp_path / "calib.yaml")
    cfg.execution.execute_hz = 250.0
    cfg.execution.settle_delay_s = 0.04
    cfg.execution.hold_hz = 0.0
    cfg.execution.home_move_duration_s = 0.2

    kinematics = DriverKinematics(cfg.arms)
    arm_bridge = RosArmBridge(cfg.arms, enable_ros=False)
    stream_hub = StreamHub(cfg.cameras, cfg.arms, arm_bridge, kinematics=kinematics, start_workers=False)
    cal_mgr = CalibrationManager(cfg.calibration_file, cfg.board)

    q_init = [0.0, 0.7, -0.9, 0.1, -0.2, 0.0]
    arm_bridge.inject_feedback("arm_3", q_init, gripper_m=0.03)
    arm_bridge.inject_feedback("arm_4", q_init, gripper_m=0.03)
    dummy_img = np.full((240, 320, 3), 90, dtype=np.uint8)
    for cam in cfg.cameras:
        stream_hub.inject_frame(cam.id, dummy_img)

    def fake_cloud_tcp20(req: dict, _url: str, _timeout: float) -> dict:
        # Generate 30 reachable world TCP steps (+0.5 mm Z per step from reported start TCP).
        left_tcp0, right_tcp0 = req["observation"]["tcp_pose_world_xyzw"]
        m_l = pose_xyzw_to_matrix(left_tcp0)
        m_r = pose_xyzw_to_matrix(right_tcp0)
        r6_l = [*m_l[0, :3].tolist(), *m_l[1, :3].tolist()]
        r6_r = [*m_r[0, :3].tolist(), *m_r[1, :3].tolist()]
        actions = []
        for k in range(1, 31):
            dz = 0.0005 * k
            actions.append([
                m_l[0, 3], m_l[1, 3], m_l[2, 3] + dz, *r6_l, 0.04,
                m_r[0, 3], m_r[1, 3], m_r[2, 3] + dz, *r6_r, 0.05,
            ])
        return {
            "schema": "piperx_inference_response_v1",
            "request_id": req["request_id"],
            "observation_monotonic_ns": req["observation_monotonic_ns"],
            "calibration_id": req["observation"]["calibration_id"],
            "action_kind": "tcp_world_absolute20",
            "actions": actions,
            "target_offsets_s": [k / 30 for k in range(1, 31)],
        }

    client = CogActClient(url="http://127.0.0.1:9999", instruction="test task", transport=fake_cloud_tcp20)
    engine = ExecutionEngine(cfg, arm_bridge, stream_hub, cal_mgr, client, kinematics=kinematics)
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_dashboard_handler(engine))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"

    def post(path: str, payload: dict) -> dict:
        req = urllib.request.Request(
            base + path, data=json.dumps(payload).encode("utf-8"), headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.load(r)

    try:
        post("/api/calibration/record_home", {})
        assert (tmp_path / "calib.yaml").is_file()

        # Mode 1: Request 30 TCP steps -> solved via piper_x_driver C++ IK -> step 1 & step 2.
        res_req = post("/api/request", {"instruction": "step tcp->ik"})
        assert res_req["chunk"]["ik_summary"]["all_converged"] is True
        assert res_req["chunk"]["ik_summary"]["max_pos_err_mm"] < 0.2
        r1 = post("/api/execute/step", {})
        r2 = post("/api/execute/step", {})
        assert r1["step_cursor"] == 1 and r2["step_cursor"] == 2
        assert arm_bridge.last_sent_info("arm_3")["tx_count"] == 2

        # Verify solved joint angle on step 2 reproduces target TCP (+1.0 mm Z) via driver FK.
        q_step2 = arm_bridge.last_sent_info("arm_3")["joints_rad"]
        cal = cal_mgr.snapshot_transforms()
        tcp_fk = np.asarray(cal["world_from_base"]["left"]) @ kinematics.fk_link6("arm_3", q_step2) @ np.asarray(cal["flange_from_tcp"]["left"])
        tcp_init = np.asarray(cal["world_from_base"]["left"]) @ kinematics.fk_link6("arm_3", q_init) @ np.asarray(cal["flange_from_tcp"]["left"])
        assert np.allclose(tcp_fk[:3, 3] - tcp_init[:3, 3], [0.0, 0.0, 0.001], atol=1e-4)

        with engine._lock:
            engine.step_cursor = len(engine.current_chunk.steps)
        sent_before_request = arm_bridge.last_sent_info("arm_3")["tx_count"]
        with pytest.raises(urllib.error.HTTPError) as exhausted:
            post("/api/execute/step", {})
        assert "请点击「请求」获取新批次" in json.load(exhausted.value)["error"]
        post("/api/request", {})
        assert engine.step_cursor == 0
        assert arm_bridge.last_sent_info("arm_3")["tx_count"] == sent_before_request
        assert post("/api/execute/step", {})["step_cursor"] == 1

        # Mode 2: 1 click executes all 30 TCP->IK steps at execute_hz.
        post("/api/execute/batch", {"fetch_new": True})
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            with urllib.request.urlopen(base + "/api/state", timeout=2) as r:
                st = json.load(r)
            if st["mode"] == "idle" and st["step_cursor"] == 30:
                break
            time.sleep(0.02)
        assert st["step_cursor"] == 30

        # Mode 3: Auto serial loop executes multiple 30-step TCP->IK cycles separated by settle_delay_s.
        post("/api/execute/auto", {})
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            with urllib.request.urlopen(base + "/api/state", timeout=2) as r:
                st = json.load(r)
            if st["auto_cycles"] >= 2:
                break
            time.sleep(0.03)
        post("/api/stop", {"release_mit": True})
        assert st["auto_cycles"] >= 2

        # Mode 4: Async overlap mode with simulated network latency t -> skips to t and blends overlap.
        cfg.execution.execute_hz = 25.0  # 30 steps = 1.20s duration

        def delayed_tcp20(req: dict, url: str, timeout: float) -> dict:
            time.sleep(0.12)  # t ~ 0.12s (3 steps), so overlap at 2t is ~30 - 6 = 24 steps
            return fake_cloud_tcp20(req, url, timeout)

        client.transport = delayed_tcp20
        post("/api/execute/async", {})
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            with urllib.request.urlopen(base + "/api/state", timeout=2) as r:
                st = json.load(r)
            if st.get("async_stats") and st["async_stats"]["chunks"] >= 3 and st["async_stats"]["overlap_steps"] > 0:
                break
            time.sleep(0.03)
        post("/api/stop", {"release_mit": True})
        astats = st["async_stats"]
        assert astats["chunks"] >= 3
        assert astats["start_index"] >= 3
        assert astats["overlap_steps"] >= 1
    finally:
        server.shutdown()
        server.server_close()
        engine.close()
        stream_hub.close()
        arm_bridge.close()


@pytest.mark.parametrize("old_mode", ["step", "batch", "auto", "async"])
def test_mode_switch_stops_clears_and_discards_late_results(tmp_path: Path, old_mode: str):
    cfg = load_client_config(calibration_file=tmp_path / "calib.yaml")
    cfg.execution.hold_hz = 0
    bridge = RosArmBridge(cfg.arms, enable_ros=False)
    calibration = CalibrationManager(cfg.calibration_file, cfg.board)
    kinematics = Mock(spec=DriverKinematics)
    hub = Mock(spec=StreamHub, kinematics=kinematics)
    hub.capture_snapshot.return_value = SimpleNamespace(
        monotonic_ns=1, unix_ns=1, arms_by_role={}, arms_by_id={}, cameras={}, calibration_id=None,
    )
    hub.latest_snapshot.return_value = None
    hub.camera_status_list.return_value = []
    target = ArmTarget(joint_positions_rad=[0.0] * 6, gripper_opening_m=0.03)
    steps = [ActionStep(index, index / 30, target, target) for index in (1, 2)]
    old_chunk = ActionChunk("old", "joint_absolute", steps, 1, 1)
    new_chunk = ActionChunk("new", "joint_absolute", steps, 2, 2)
    request_started, request_release = threading.Event(), threading.Event()

    def blocked_predict(**_kwargs):
        request_started.set()
        assert request_release.wait(5)
        return old_chunk

    client = Mock(spec=CogActClient)
    client.predict.side_effect = blocked_predict
    engine = ExecutionEngine(cfg, bridge, hub, calibration, client, kinematics=kinematics)
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_dashboard_handler(engine))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_port}"

    def post(path: str, payload: dict) -> dict:
        request = urllib.request.Request(
            base + path, data=json.dumps(payload).encode(), headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(request, timeout=5) as response:
            return json.load(response)

    cancelled_requests = []

    def start_old_mode():
        try:
            post(f"/api/execute/{old_mode}", {})
        except urllib.error.HTTPError as error:
            cancelled_requests.append(json.load(error))

    pending = threading.Thread(target=start_old_mode, daemon=True)
    try:
        pending.start()
        assert request_started.wait(2)
        previous_worker, previous_consumer = engine._worker, engine._consumer_worker
        engine.current_chunk = old_chunk
        engine.step_cursor = 1
        engine.auto_cycles = 3
        engine.async_stats = {"chunks": 3}
        engine._held_targets["arm_3"] = ([0.0] * 6, 0.03, time.monotonic())
        started = time.monotonic()
        post("/api/stop", {"release_mit": True, "clear_results": True})
        assert time.monotonic() - started < 1.0
        cleared = engine.status_dict()
        assert cleared["chunk"] is None and cleared["async_stats"] is None
        assert cleared["step_cursor"] == cleared["total_steps"] == cleared["auto_cycles"] == 0
        assert cleared["holding_arms"] == [] and cleared["last_error"] is None

        client.predict.side_effect = None
        client.predict.return_value = new_chunk
        post("/api/execute/step", {})
        sent_after_switch = bridge.last_sent_info("arm_3")["tx_count"]
        request_release.set()
        for worker in (pending, previous_worker, previous_consumer):
            if worker is not None:
                worker.join(timeout=2)
                assert not worker.is_alive()
        assert cancelled_requests == ([{"ok": False, "error": "执行已停止"}] if old_mode == "step" else [])
        assert engine.current_chunk is new_chunk and engine.step_cursor == 1
        assert engine.mode == "ready" and engine.async_stats is None
        assert bridge.last_sent_info("arm_3")["tx_count"] == sent_after_switch
    finally:
        request_release.set()
        server.shutdown()
        server.server_close()
        engine.close()
        bridge.close()

