"""Focused calibration math and end-to-end C++ driver + Python calibrator tests."""

from __future__ import annotations

import json
import os
import pty
import select
import signal
import struct
import subprocess
import threading
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix
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
from paxini_driver.configuration import resolve_sensor_configs
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState

from piper_x_interfaces.msg import ArmStatus, MitCommand


def _crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (((crc << 1) ^ 0x07) & 0xFF) if (crc & 0x80) else ((crc << 1) & 0xFF)
    return crc


def _serial_wrench_frame(values) -> bytes:
    body = bytes([0xAA, 0x55, 0x7F, 0x03]) + struct.pack("<6f", *values)
    return body + bytes([_crc8(body)])


def test_wrench_stability_requires_full_window_and_detects_fluctuation():
    window = WrenchStabilityWindow(0.8, [0.001] * 6)
    for t in (0.0, 0.4):
        window.update([0.0] * 6, t)
    assert not window.is_stable(0.4)
    window.update([0.0005] * 6, 0.8)
    assert window.is_stable(0.8)
    q = [0.0] * 6
    q[3] = 0.01
    window.update(q, 0.9)
    assert not window.is_stable(0.9)
    window.update(q, 1.4)
    assert not window.is_stable(1.4)
    window.update(q, 1.8)
    assert window.is_stable(1.8)
    assert not window.is_stable(2.7)


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


def test_exact_wrench_fit_uses_mean_gravity_and_reports_observability():
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


def test_independent_calibrator_with_cpp_driver_pty_reloads_and_compensates(tmp_path):
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

    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    driver_bin = Path(get_package_prefix("paxini_driver")) / "lib" / "paxini_driver" / "driver"
    driver_proc = subprocess.Popen(
        [
            str(driver_bin),
            "--ros-args",
            "-r",
            "__ns:=/paxini_test/arm_3",
            "-p",
            "arm_id:=arm_3",
            "-p",
            f"config_file:={cfg_path}",
            "-p",
            "interface:=usb",
            "-p",
            f"serial_port:={slave_path}",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    rclpy.init()
    client = PoseCalibrator("arm_3", cfg_path)
    other = PoseCalibrator("arm_4", cfg_path)
    fake_arm = rclpy.create_node("fake_arm", namespace="/paxini_test/arm_3")
    q_pub = fake_arm.create_publisher(JointState, "joint_states_feedback", 10)
    status_pub = fake_arm.create_publisher(ArmStatus, "arm_status", 10)
    compensated_msgs = []
    fake_arm.create_subscription(
        WrenchStamped, "wrench_stamped", compensated_msgs.append, 10
    )
    initial = (0.2, 0.9, -1.3, 0.1, 0.2, 0.3)
    state = {"q": initial, "disturbed": False, "commands": []}
    tx_from_driver = bytearray()

    def command_cb(cmd):
        state["q"] = tuple(cmd.position[:6])
        state["commands"].append(state["q"])

    fake_arm.create_subscription(MitCommand, "command/mit", command_cb, 10)

    def feedback():
        while select.select([master_fd], [], [], 0.0)[0]:
            tx_from_driver.extend(os.read(master_fd, 256))
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
        raw = [bias[i] + g[i] for i in range(6)]
        if client._calib_collecting and not state["disturbed"]:
            raw[0] += 100.0
            state["disturbed"] = True
        os.write(master_fd, _serial_wrench_frame(raw))

    fake_arm.create_timer(0.005, feedback)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (client, other, fake_arm):
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

        time.sleep(0.05)
        compensated_msgs.clear()
        deadline = time.monotonic() + 1.0
        while len(compensated_msgs) < 3 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert compensated_msgs
        latest_w = compensated_msgs[-1].wrench
        assert (
            latest_w.force.x,
            latest_w.force.y,
            latest_w.force.z,
            latest_w.torque.x,
            latest_w.torque.y,
            latest_w.torque.z,
        ) == pytest.approx((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), abs=0.05)

        saved = yaml.safe_load(cfg_path.read_text())
        samples = saved["sensors"]["arm_3"]["calibration_samples"]
        assert len(samples) == 4 and all(p["duration_sec"] >= 0.05 for p in samples)
        assert all(len(p["gravity_unit_mean"]) == 3 for p in samples)
        assert saved["sensors"]["arm_4"]["tare_offset"] == [1.0] * 6

        driver_proc.send_signal(signal.SIGINT)
        driver_proc.wait(timeout=3.0)
        while select.select([master_fd], [], [], 0.02)[0]:
            tx_from_driver.extend(os.read(master_fd, 256))
        assert bytes.fromhex("AA 55 7F 04 00 E8") in bytes(tx_from_driver)
    finally:
        client._stop_event.set()
        other._stop_event.set()
        executor.shutdown()
        if thread.ident is not None:
            thread.join(timeout=2)
        for node in (client, other, fake_arm):
            node.destroy_node()
        rclpy.try_shutdown()
        if driver_proc.poll() is None:
            driver_proc.kill()
            driver_proc.wait(timeout=2)
        os.close(slave_fd)
        os.close(master_fd)
