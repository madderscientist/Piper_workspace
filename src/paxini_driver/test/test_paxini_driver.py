"""Focused protocol and ROS node tests for paxini_driver."""

from __future__ import annotations

import json
import os
import struct
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from typing import List, Optional, Tuple

import numpy as np
import pytest
import rclpy
import yaml
from geometry_msgs.msg import WrenchStamped
from paxini_driver.calibration import (
    WrenchStabilityWindow,
    sensor_rotation_in_base,
    solve_wrench_calibration,
    tool_gravity_wrench,
)
from paxini_driver.calibrator import PoseCalibrator
from paxini_driver.configuration import (
    resolve_sensor_configs,
    save_sensor_calibration_to_yaml,
)
from paxini_driver.node import PaxiniDriverNode
from paxini_driver.protocol import (
    CMD_CALIBRATE,
    CMD_GET_VERSION,
    CMD_READ_ONCE,
    CMD_SET_FREQ,
    CMD_SET_ID,
    CMD_SET_PORT_PARAMS,
    CMD_START_AUTO,
    CMD_STOP_AUTO,
    CanFrameParser,
    ProtocolEvent,
    SerialFrameParser,
    WrenchSample,
    build_can_command,
    build_can_set_protocol,
    build_serial_frame,
    freq_to_code,
)
from paxini_driver.transport import SensorTransport, SerialSensorTransport
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from piper_x_interfaces.msg import ArmStatus, MitCommand


def test_serial_protocol_matches_manual_examples():
    assert build_serial_frame(CMD_SET_ID, 0x01) == bytes.fromhex("AA 55 7F 01 01 AE")
    assert build_serial_frame(CMD_SET_FREQ, freq_to_code(4)) == bytes.fromhex(
        "AA 55 7F 02 01 91"
    )
    assert build_serial_frame(CMD_SET_FREQ, freq_to_code(100)) == bytes.fromhex(
        "AA 55 7F 02 19 D9"
    )
    assert build_serial_frame(CMD_START_AUTO) == bytes.fromhex("AA 55 7F 03 04 9F")
    assert build_serial_frame(CMD_STOP_AUTO) == bytes.fromhex("AA 55 7F 04 00 E8")
    assert build_serial_frame(CMD_READ_ONCE) == bytes.fromhex("AA 55 7F 05 01 FA")
    assert build_serial_frame(CMD_GET_VERSION) == bytes.fromhex("AA 55 7F 07 01 D0")
    assert build_serial_frame(CMD_CALIBRATE) == bytes.fromhex("AA 55 7F 10 01 EC")

    parser = SerialFrameParser(device_id=0x7F)
    stream = (
        b"\x00\xaa\x55\x7f\x03\xff"  # corrupted / false header
        + bytes.fromhex("AA 55 7F 07 00 76 30 2E 31 2E 31 00 5B")
        + bytes.fromhex(
            "AA 55 7F 03 31 0C 0B BD 5C 76 38 BD FC 35 A2 3C "
            "6E E8 B7 BA E6 9C 24 39 36 C8 9F B9 43"
        )
        + bytes.fromhex("AA 55 7F 10 00 00 00 00 00 00 00 00 E2")
    )
    events = parser.feed(stream[:15]) + parser.feed(stream[15:])
    assert len(events) == 3
    assert events[0].version == "v0.1.1"
    assert events[1].wrench is not None
    assert events[1].wrench.fx == pytest.approx(-0.03394717, abs=1e-6)
    assert events[1].wrench.fy == pytest.approx(-0.04503475, abs=1e-6)
    assert events[1].wrench.fz == pytest.approx(0.01980113, abs=1e-6)
    assert events[2].ack_cmd == CMD_CALIBRATE


def test_serial_wait_receives_delayed_frame():
    read_fd, write_fd = os.pipe()
    ready = threading.Event()
    frame = bytes.fromhex(
        "AA 55 7F 03 31 0C 0B BD 5C 76 38 BD FC 35 A2 3C "
        "6E E8 B7 BA E6 9C 24 39 36 C8 9F B9 43"
    )

    class PipeSerial:
        is_open = True

        def fileno(self):
            return read_fd

        @property
        def in_waiting(self):
            return len(frame) if ready.is_set() else 0

        def read(self, count):
            ready.clear()
            return os.read(read_fd, count)

        def close(self):
            self.is_open = False
            os.close(read_fd)

    transport = SerialSensorTransport(serial_factory=lambda *_: PipeSerial())

    def delayed_frame():
        time.sleep(0.01)
        ready.set()
        os.write(write_fd, frame)

    writer = threading.Thread(target=delayed_frame)
    transport.open()
    writer.start()
    try:
        events = transport.poll_events(timeout=0.25)
        assert len(events) == 1 and events[0].wrench is not None
        assert events[0].wrench.as_tuple() == WrenchSample.from_bytes(frame[4:-1]).as_tuple()
    finally:
        writer.join()
        transport.close()
        os.close(write_fd)


def test_can_and_canfd_protocol_matches_manual():
    assert build_can_command(CMD_SET_ID, 0x01) == (
        0x27F,
        bytes.fromhex("01 01 00 00 00 00 00 00"),
    )
    assert build_can_command(CMD_SET_FREQ, freq_to_code(4)) == (
        0x27F,
        bytes.fromhex("02 01 00 00 00 00 00 00"),
    )
    assert build_can_command(CMD_START_AUTO) == (
        0x27F,
        bytes.fromhex("03 00 00 00 00 00 00 00"),
    )
    assert build_can_command(CMD_STOP_AUTO) == (
        0x27F,
        bytes.fromhex("04 00 00 00 00 00 00 00"),
    )
    assert build_can_command(CMD_READ_ONCE) == (
        0x27F,
        bytes.fromhex("05 00 00 00 00 00 00 00"),
    )
    assert build_can_command(CMD_GET_VERSION) == (
        0x27F,
        bytes.fromhex("07 00 00 00 00 00 00 00"),
    )
    assert build_can_command(CMD_CALIBRATE) == (
        0x27F,
        bytes.fromhex("10 01 00 00 00 00 00 00"),
    )
    assert build_can_set_protocol(True) == (
        0x27F,
        bytes.fromhex("11 01 00 01 03 01 00 00"),
    )
    assert build_can_set_protocol(False) == (
        0x27F,
        bytes.fromhex("11 00 00 01 03 00 00 00"),
    )

    parser = CanFrameParser(device_id=0x7F)
    ver_ev = parser.push(0x1FF, bytes.fromhex("00 07 76 30 2E 31 2E 31"))
    assert ver_ev is not None and ver_ev.version == "v0.1.1"

    mode_ev = parser.push(0x1FF, bytes.fromhex("00 11 00 00 00 00 00 00"))
    assert (
        mode_ev is not None
        and mode_ev.ack_cmd == CMD_SET_PORT_PARAMS
        and mode_ev.status == 0
    )

    expected = (1.5, -2.25, 12.0, 0.15, -0.25, 0.05)
    raw24 = struct.pack("<6f", *expected)
    fd_ev = parser.push(0x2FF, raw24)
    assert fd_ev is not None and fd_ev.wrench is not None
    assert fd_ev.wrench.as_tuple() == pytest.approx(expected)

    assert parser.push(0x2FF, raw24[0:8]) is None
    assert parser.push(0x3FF, raw24[8:16]) is None
    can_ev = parser.push(0x4FF, raw24[16:24])
    assert can_ev is not None and can_ev.wrench is not None
    assert can_ev.wrench.as_tuple() == pytest.approx(expected)


class _FakeTransport(SensorTransport):
    interface_name = "usb"

    def __init__(self) -> None:
        self.available = False
        self._open = False
        self.commands: List[Tuple[int, Optional[int]]] = []
        self.events: List[ProtocolEvent] = []

    @property
    def is_open(self) -> bool:
        return self._open

    def open(self) -> None:
        if not self.available:
            raise OSError("No such device /dev/ttyUSB0")
        self._open = True

    def close(self) -> None:
        self._open = False

    def poll_events(self, timeout: float = 0.002) -> List[ProtocolEvent]:
        out = list(self.events)
        self.events.clear()
        return out

    def send_command(self, cmd: int, arg: Optional[int] = None) -> None:
        self.commands.append((cmd, arg))


def test_wrench_stability_requires_full_window_and_detects_fluctuation():
    window = WrenchStabilityWindow(0.8, [0.001] * 6)
    for t in (0.0, 0.4):
        window.update([0.0] * 6, t)
    assert not window.is_stable(0.4)
    window.update([0.0005] * 6, 0.8)
    assert window.is_stable(0.8)
    # One fluctuating wrench channel invalidates stability until a new stationary window exists.
    q = [0.0] * 6
    q[3] = 0.01
    window.update(q, 0.9)
    assert not window.is_stable(0.9)
    window.update(q, 1.4)
    assert not window.is_stable(1.4)
    window.update(q, 1.8)
    assert window.is_stable(1.8)
    assert not window.is_stable(2.7)  # Old feedback cannot prove current stability.


def test_real_arm3_calibration_matches_independent_sdk_pose():
    data = json.loads(
        (Path(__file__).parent / "fixtures/arm3_static_calibration.json").read_text()
    )
    x, y, z, w = data["sdk_pose_quaternion_xyzw"]
    sdk_rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    assert sensor_rotation_in_base(data["sdk_joint_position_rad"]) == pytest.approx(
        sdk_rotation, abs=5e-5
    )
    result = solve_wrench_calibration(data["poses"], data["wrenches"])
    assert result.tool_mass_kg == pytest.approx(0.614, abs=0.005)
    assert result.force_rms_n < 0.30
    assert result.torque_rms_nm < 0.015


def test_ros_driver_raw_compensation_and_reload(tmp_path, monkeypatch):
    cfg_path = tmp_path / "px6d.yaml"
    cfg_path.write_text(
        yaml.safe_dump({"sensors": {"arm_3": {"namespace": "/paxini_test/arm_3"}}})
    )
    configs = resolve_sensor_configs()
    assert [c["arm_id"] for c in configs] == ["arm_3", "arm_4"]
    assert resolve_sensor_configs("右手")[0]["arm_id"] == "arm_4"
    rclpy.init()
    transport = _FakeTransport()
    node = PaxiniDriverNode(
        namespace="/paxini_test/arm_3",
        transport=transport,
        start_worker=False,
        parameter_overrides=[
            Parameter("arm_id", value="arm_3"),
            Parameter("config_file", value=str(cfg_path)),
            Parameter("reconnect_sec", value=0.01),
            Parameter("frequency_hz", value=200),
        ],
    )
    received, raw_received = [], []
    logs = {"info": [], "warning": []}
    logger = SimpleNamespace(info=logs["info"].append, warning=logs["warning"].append)
    monkeypatch.setattr(node, "get_logger", lambda: logger)
    node.create_subscription(WrenchStamped, "wrench_stamped", received.append, 10)
    node.create_subscription(
        WrenchStamped, "wrench_raw_stamped", raw_received.append, 10
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        malformed_joints = JointState()
        malformed_joints.name = ["gripper"] + [f"joint{i}" for i in range(1, 7)]
        malformed_joints.position = [0.0] * 6
        node._on_joints(malformed_joints)
        assert node.latest_joint_positions is None
        assert node.status_dict()["frequency_hz"] == 200
        assert node.status_dict()["poll_rate_hz"] == 1000.0
        assert not node.status_dict()["connected"]
        assert node.calibrate_hardware()[0] is False
        transport.available = True
        node._last_connect_attempt = 0.0
        transport.events = [
            ProtocolEvent(cmd=CMD_GET_VERSION, version="v1.0.3"),
            ProtocolEvent(
                cmd=CMD_START_AUTO, wrench=WrenchSample(10, -5, 20, 0.1, -0.2, 0.3)
            ),
        ]
        assert node.step() == 2
        deadline = time.monotonic() + 1
        while (not received or not raw_received) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert raw_received[-1].wrench.force.z == 20
        assert received[-1].wrench.force.z == 20
        assert transport.commands[:3] == [
            (CMD_GET_VERSION, None),
            (CMD_START_AUTO, None),
            (CMD_SET_FREQ, 200),
        ]
        save_sensor_calibration_to_yaml(
            "arm_3", tare_offset=[10, -5, 20, 0.1, -0.2, 0.3], config_path=cfg_path
        )
        before_commands = list(transport.commands)
        assert node.reload_config()[0]
        assert (
            transport.commands == before_commands
        )  # Reload only applies data; no hardware commands.
        transport.events = [
            ProtocolEvent(
                cmd=CMD_START_AUTO, wrench=WrenchSample(12, -5, 25, 0.1, -0.2, 0.3)
            )
        ]
        node.step()
        assert node.latest_raw is not None and node.latest_wrench is not None
        assert node.latest_raw.as_tuple() == (12, -5, 25, 0.1, -0.2, 0.3)
        assert node.latest_wrench.as_tuple() == pytest.approx((2, 0, 5, 0, 0, 0))
        assert not hasattr(node, "mit_pub")
        assert not hasattr(node, "auto_calibrate")
        assert node.calibrate_hardware()[0]
        assert logs["info"] == [] and logs["warning"] == []
        last = node._last_wrench_time
        node._check_stream_timeout(last + 0.5)
        assert logs["warning"] == []
        node._check_stream_timeout(last + 1.1)
        node._check_stream_timeout(last + 2.0)
        assert len(logs["warning"]) == 1 and "断流" in logs["warning"][0]
        node._process_event(
            ProtocolEvent(
                cmd=CMD_START_AUTO, wrench=WrenchSample(12, -5, 25, 0.1, -0.2, 0.3)
            ),
            last + 2.1,
        )
        assert not node.status_dict()["stream_gap"]
        node._handle_disconnect(OSError("test disconnect"))
        node._handle_disconnect(OSError("test disconnect"))
        assert len(logs["warning"]) == 2 and "通信中断" in logs["warning"][1]
        transport.open()
        node.connected = True
        node.streaming = True
        node._process_event(
            ProtocolEvent(
                cmd=CMD_START_AUTO, wrench=WrenchSample(12, -5, 25, 0.1, -0.2, 0.3)
            ),
            last + 2.2,
        )
        assert logs["info"] == []
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
    assert transport.commands[-1] == (CMD_STOP_AUTO, None)


def test_exact_wrench_fit_uses_mean_gravity_and_reports_observability():
    # Independent analytic inputs: means of two different gravity directions,
    # rather than directions recomputed from a mean joint angle.
    g = np.array(
        [[0.5, 0.5, 0], [-0.5, 0.5, 0], [0, -0.5, 0.5], [0.5, 0, -0.5], [-0.5, 0, -0.5]]
    )
    bias = np.array([1.2, -2.3, 0.4, 0.01, -0.02, 0.03])
    mass = 0.65
    com = np.array([0.02, -0.015, 0.04])
    force = mass * 9.81 * g
    torque = np.cross(com, force)
    observed = np.column_stack((force, torque)) + bias
    result = solve_wrench_calibration(
        [[0] * 6] * len(g), observed, gravity_directions=g
    )
    assert result.tare_offset == pytest.approx(bias, abs=1e-12)
    assert result.tool_mass_kg == pytest.approx(mass, abs=1e-12)
    assert result.tool_com_m == pytest.approx(com, abs=1e-12)
    assert result.force_rms_n < 1e-12 and result.torque_rms_nm < 1e-12
    assert (
        result.to_dict()["mass_identifiable"] and result.to_dict()["com_identifiable"]
    )
    static = solve_wrench_calibration(
        [[0] * 6] * 2, [observed[0]] * 2, gravity_directions=[g[0]] * 2
    )
    assert not static.to_dict()["mass_identifiable"]
    assert not static.to_dict()["com_identifiable"]


def test_independent_calibrator_samples_saves_returns_and_reloads(tmp_path):
    cfg_path = tmp_path / "px6d.yaml"
    cfg_path.write_text(
        yaml.safe_dump(
            {
                "defaults": {
                    "calib_settle_sec": 0.1,
                    "calib_sample_sec": 0.05,
                    "calib_stability_range": [0.1] * 3 + [0.01] * 3,
                },
                "sensors": {
                    "arm_3": {
                        "namespace": "/paxini_test/arm_3",
                        "tare_offset": [3.0] * 6,
                    },
                    "arm_4": {
                        "namespace": "/paxini_test/arm_4",
                        "tare_offset": [1.0] * 6,
                    },
                },
            }
        )
    )
    bias = (1.5, -2.5, 3.5, 0.02, -0.03, 0.04)
    mass = 0.5
    com = (0.01, -0.02, 0.06)
    poses = [
        (0, 0.8, -1.2, 0, 0, 0),
        (0, 0.8, -1.2, 0.9, 0.6, -0.4),
        (0, 0.8, -1.2, -0.8, -0.6, 0.7),
        (0.3, 1, -1.5, 0.5, -0.8, -1.1),
    ]
    rclpy.init()
    transport = _FakeTransport()
    transport.available = True
    driver = PaxiniDriverNode(
        namespace="/paxini_test/arm_3",
        transport=transport,
        start_worker=False,
        parameter_overrides=[
            Parameter("arm_id", value="arm_3"),
            Parameter("config_file", value=str(cfg_path)),
        ],
    )
    client = PoseCalibrator("arm_3", cfg_path)
    other = PoseCalibrator("arm_4", cfg_path)
    fake_arm = rclpy.create_node("fake_arm", namespace="/paxini_test/arm_3")
    q_pub = fake_arm.create_publisher(JointState, "joint_states_feedback", 10)
    status_pub = fake_arm.create_publisher(ArmStatus, "arm_status", 10)
    initial = (0.2, 0.9, -1.3, 0.1, 0.2, 0.3)
    state = {"q": initial, "disturbed": False, "commands": []}

    def command_cb(cmd):
        state["q"] = tuple(cmd.position[:6])
        state["commands"].append(state["q"])

    fake_arm.create_subscription(MitCommand, "command/mit", command_cb, 10)

    def feedback():
        q = state["q"]
        js = JointState()
        js.name = [f"joint{i}" for i in range(1, 7)]
        js.position = [float(v) for v in q]
        q_pub.publish(js)
        status = ArmStatus()
        status.ctrl_mode = 1
        status.mode_feedback = 6
        status.motor_enabled = [True] * 6
        status_pub.publish(status)
        g = tool_gravity_wrench(q, mass, com)
        raw = WrenchSample(*(bias[i] + g[i] for i in range(6)))
        if client._calib_collecting and not state["disturbed"]:
            raw = WrenchSample(raw.fx + 100, raw.fy, raw.fz, raw.mx, raw.my, raw.mz)
            state["disturbed"] = True
        driver._process_event(
            ProtocolEvent(cmd=CMD_START_AUTO, wrench=raw), time.monotonic()
        )

    fake_arm.create_timer(0.005, feedback)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (driver, client, other, fake_arm):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    try:
        for index, q in enumerate(poses):
            js = JointState()
            js.name = [f"joint{i}" for i in range(1, 7)]
            js.position = [float(v) for v in q]
            recorder = other if index == len(poses) - 1 else client
            recorder._on_joints(js)
            recorder.record_pose()
        assert resolve_sensor_configs("arm_4", str(cfg_path))[0][
            "calibration_poses"
        ] == [list(q) for q in poses]
        client.latest_joint_positions = None
        thread.start()
        client._wait_for_joints()
        time.sleep(0.03)
        result = client.run()
        assert state["disturbed"]
        deadline = time.monotonic() + 1
        while not np.allclose(state["q"], initial) and time.monotonic() < deadline:
            time.sleep(0.005)
        assert state["q"] == pytest.approx(initial)
        assert all(
            any(np.allclose(q, t) for t in [initial, *poses]) for q in state["commands"]
        )
        assert result["tare_offset"] == pytest.approx(bias, abs=0.03)
        assert result["tool_mass_kg"] == pytest.approx(mass, abs=0.005)
        assert result["tool_com_m"] == pytest.approx(com, abs=0.005)
        assert driver.tare_offset == pytest.approx(result["tare_offset"], abs=1e-6)
        assert driver.tool_mass_kg == pytest.approx(result["tool_mass_kg"], abs=1e-6)
        saved = yaml.safe_load(cfg_path.read_text())
        samples = saved["sensors"]["arm_3"]["calibration_samples"]
        assert len(samples) == 4 and all(p["duration_sec"] >= 0.05 for p in samples)
        assert all(len(p["gravity_unit_mean"]) == 3 for p in samples)
        assert saved["sensors"]["arm_4"]["tare_offset"] == [1.0] * 6
    finally:
        client._stop_event.set()
        other._stop_event.set()
        executor.shutdown()
        if thread.ident is not None:
            thread.join(timeout=2)
        for node in (client, other, fake_arm, driver):
            node.destroy_node()
        rclpy.try_shutdown()
