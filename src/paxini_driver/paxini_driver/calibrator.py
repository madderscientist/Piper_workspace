"""独立 ROS 多姿态标定客户端：控制机械臂走位、等待原始力稳定采样、最小二乘解算、写回 YAML 并通知 C++ 驱动热重载。"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from piper_x_interfaces.msg import ArmStatus, MitCommand
from piper_x_interfaces.srv import Enable, SetMode

from .calibration import (
    WrenchStabilityWindow,
    gravity_unit_in_sensor,
    solve_wrench_calibration,
)
from .configuration import (
    resolve_sensor_configs,
    save_sensor_calibration_to_yaml,
    source_config_path,
)

JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
CALIB_KP = (25.0, 25.0, 25.0, 8.0, 6.0, 4.0)
CALIB_KD = (4.5, 4.5, 2.2, 0.3, 0.24, 0.06)
MIT_POSITION_LIMIT = (
    12.5  # MIT 协议编码表示上限，并非额外附加的软件运动限位。
)


@dataclass(frozen=True)
class CalibrationPoint:
    position_rad: Tuple[float, ...]
    wrench: Tuple[float, ...]
    gravity_unit_mean: Tuple[float, ...]
    sample_count: int
    duration_sec: float

    def to_dict(self):
        return {
            "position_rad": [float(v) for v in self.position_rad],
            "wrench": [float(v) for v in self.wrench],
            "gravity_unit_mean": [float(v) for v in self.gravity_unit_mean],
            "sample_count": self.sample_count,
            "duration_sec": self.duration_sec,
        }


class PoseCalibrator(Node):
    def __init__(self, arm_id: str, config_path: Optional[Path] = None):
        entry = resolve_sensor_configs(
            arm_id, str(config_path) if config_path else None
        )[0]
        super().__init__("paxini_calibration_client", namespace=entry["namespace"])
        self.arm_id = entry["arm_id"]
        self.config_file = source_config_path(config_path)
        self.calib_settle_sec = float(entry.get("calib_settle_sec", 0.8))
        self.calib_sample_sec = float(entry.get("calib_sample_sec", 0.5))
        self._wrench_stability = WrenchStabilityWindow(
            self.calib_settle_sec, entry["calib_stability_range"]
        )
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self.latest_joint_positions = None
        self.latest_arm_status = None
        self._calib_collecting = False
        self._calib_samples = []
        self.create_subscription(
            JointState,
            "joint_states_feedback",
            self._on_joints,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            WrenchStamped,
            entry.get("raw_wrench_topic", "wrench_raw_stamped"),
            self._on_wrench,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ArmStatus, "arm_status", self._on_arm_status, qos_profile_sensor_data
        )
        self.mit_pub = self.create_publisher(MitCommand, "command/mit", 10)
        self.enable_client = self.create_client(Enable, "enable_srv")
        self.mode_client = self.create_client(SetMode, "set_mode")
        self.reload_client = self.create_client(Trigger, "reload_config")

    def _on_joints(self, msg: JointState):
        names = list(msg.name)
        if not all(name in names for name in JOINT_NAMES):
            return
        order = [names.index(name) for name in JOINT_NAMES]
        if not all(index < len(msg.position) for index in order):
            return
        q = tuple(float(msg.position[index]) for index in order)
        if all(math.isfinite(v) for v in q):
            with self._lock:
                self.latest_joint_positions = q

    def _on_arm_status(self, msg: ArmStatus):
        with self._lock:
            self.latest_arm_status = msg

    def _on_wrench(self, msg: WrenchStamped):
        w = msg.wrench
        raw = (w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z)
        if not all(math.isfinite(v) for v in raw):
            return
        now = time.monotonic()
        with self._lock:
            self._wrench_stability.update(raw, now)
            if not self._calib_collecting:
                return
            if (
                self._wrench_stability.is_stable(now)
                and self.latest_joint_positions is not None
            ):
                q = self.latest_joint_positions
                self._calib_samples.append(
                    (now, q, raw, tuple(gravity_unit_in_sensor(q)))
                )
            else:
                self._reset_collection()

    def _reset_collection(self) -> None:
        """在已持有 self._lock 的前提下清空当前姿态采样缓存。"""
        self._calib_collecting = False
        self._calib_samples.clear()

    def _wait_for_joints(self):
        while True:
            if self._stop_event.is_set():
                raise RuntimeError("校准程序已停止")
            with self._lock:
                if self.latest_joint_positions is not None:
                    return tuple(self.latest_joint_positions)
            time.sleep(0.005)

    def record_pose(self):
        q = self._wait_for_joints()
        entry = resolve_sensor_configs(self.arm_id, str(self.config_file))[0]
        poses = list(entry["calibration_poses"])
        poses.append([round(v, 6) for v in q])
        save_sensor_calibration_to_yaml(
            self.arm_id, calibration_poses=poses, config_path=self.config_file
        )
        return f"已记录公共姿态 {len(poses)}: {poses[-1]}"

    def _publish_mit_cmd(
        self,
        position: Sequence[float],
        kp: Sequence[float] = CALIB_KP,
        kd: Sequence[float] = CALIB_KD,
    ) -> None:
        msg = MitCommand()
        msg.position = [
            max(-MIT_POSITION_LIMIT, min(MIT_POSITION_LIMIT, float(v)))
            for v in position[:6]
        ]
        msg.velocity = [0.0] * 6
        msg.kp = [float(v) for v in kp[:6]]
        msg.kd = [float(v) for v in kd[:6]]
        msg.torque = [0.0] * 6
        self.mit_pub.publish(msg)

    def _ensure_arm_mit_ready(self, q_init: Sequence[float]) -> tuple[bool, str]:
        with self._lock:
            st = self.latest_arm_status
        if st is not None and not all(st.motor_enabled):
            if self.enable_client.service_is_ready():
                fut = self.enable_client.call_async(Enable.Request(enable_request=True))
                deadline = time.monotonic() + 3.0
                while not fut.done() and time.monotonic() < deadline:
                    time.sleep(0.01)
                res = fut.result() if fut.done() else None
                if res is None or not res.enable_response:
                    return (
                        False,
                        f"使能机械臂失败: {getattr(res, 'message', 'timeout')}",
                    )
        self._publish_mit_cmd(q_init, kp=(0.0,) * 6, kd=(0.0,) * 6)
        if st is None or not (st.ctrl_mode == 1 and st.mode_feedback == 6):
            if self.mode_client.service_is_ready():
                fut = self.mode_client.call_async(SetMode.Request(mode="mit"))
                deadline = time.monotonic() + 2.0
                while not fut.done() and time.monotonic() < deadline:
                    self._publish_mit_cmd(q_init, kp=(0.0,) * 6, kd=(0.0,) * 6)
                    time.sleep(0.01)
                res = fut.result() if fut.done() else None
                if res is not None and not res.success:
                    return False, f"切换 MIT 模式失败: {res.message}"
        # 在初始姿态处平滑爬升 MIT 刚度与阻尼，避免切入控制瞬间产生冲击。
        ramp_sec = 0.30
        steps = max(1, int(ramp_sec / 0.005))
        for i in range(1, steps + 1):
            alpha = i / steps
            self._publish_mit_cmd(
                q_init,
                kp=[alpha * k for k in CALIB_KP],
                kd=[alpha * d for d in CALIB_KD],
            )
            time.sleep(0.005)
        return True, "ok"

    def _command_pose(self, q_target: Sequence[float]) -> None:
        """一次性下发六个关节的最终目标角，重置力稳定窗口，由底层机械臂驱动完成运动。"""
        if self._stop_event.is_set():
            raise RuntimeError("校准已停止")
        with self._lock:
            self._wrench_stability.reset()
        self._publish_mit_cmd(q_target)

    def _wait_for_wrench_stability(self, q_target: Sequence[float]) -> None:
        while True:
            if self._stop_event.is_set():
                raise RuntimeError("校准已停止")
            self._publish_mit_cmd(q_target)
            with self._lock:
                if self._wrench_stability.is_stable(time.monotonic()):
                    return
            time.sleep(0.005)

    def _move_and_sample_pose(self, q_target: Sequence[float]) -> CalibrationPoint:
        with self._lock:
            self._reset_collection()
        self._command_pose(q_target)
        self.get_logger().info("等待原始六维力读数稳定后采样")
        collecting = False
        while True:
            if self._stop_event.is_set():
                raise RuntimeError("校准已停止")
            self._publish_mit_cmd(q_target)
            with self._lock:
                stable = self._wrench_stability.is_stable(time.monotonic())
                if not stable or (collecting and not self._calib_collecting):
                    if collecting:
                        self.get_logger().info(
                            "六维力再次波动，丢弃本段采样并重新等待稳定"
                        )
                    collecting = False
                    self._reset_collection()
                if stable and not collecting:
                    collecting = self._calib_collecting = True
                    self.get_logger().info("六维力读数已稳定，开始采集位置和六维力")
                if len(self._calib_samples) >= 2:
                    duration = self._calib_samples[-1][0] - self._calib_samples[0][0]
                    if duration >= self.calib_sample_sec:
                        self._calib_collecting = False
                        samples = list(self._calib_samples)
                        break
            time.sleep(0.005)
        means = [
            tuple(float(v) for v in np.mean([s[i] for s in samples], axis=0))
            for i in (1, 2, 3)
        ]
        point = CalibrationPoint(means[0], means[1], means[2], len(samples), duration)
        self.get_logger().info(
            f"姿态采样完成: {point.sample_count} 条，"
            f"关节均值 {[round(v, 6) for v in point.position_rad]}，"
            f"六维力均值 {[round(v, 6) for v in point.wrench]}"
        )
        return point

    def run(self):
        entry = resolve_sensor_configs(self.arm_id, str(self.config_file))[0]
        poses = entry["calibration_poses"]
        if not poses:
            raise ValueError("尚未记录公共校准姿态")
        self.get_logger().info("等待机械臂反馈和 paxini_driver 的 reload_config 服务")
        while not self.reload_client.wait_for_service(timeout_sec=0.2):
            if self._stop_event.is_set():
                raise RuntimeError("校准程序已停止")
        initial_q = self._wait_for_joints()
        ok, reason = self._ensure_arm_mit_ready(initial_q)
        if not ok:
            raise RuntimeError(reason)
        points = []
        for index, target in enumerate(poses, 1):
            self.get_logger().info(f"校准姿态 {index}/{len(poses)}")
            point = self._move_and_sample_pose(target)
            points.append(point)
        self.get_logger().info("所有姿态采样完毕，返回校准开始前的位置")
        self._command_pose(initial_q)
        self._wait_for_wrench_stability(initial_q)
        steps = max(1, int(0.25 / 0.005))
        for i in range(steps - 1, -1, -1):
            alpha = i / steps
            self._publish_mit_cmd(
                initial_q,
                kp=[alpha * k for k in CALIB_KP],
                kd=[alpha * d for d in CALIB_KD],
            )
            time.sleep(0.005)
        result = solve_wrench_calibration(
            [p.position_rad for p in points],
            [p.wrench for p in points],
            gravity_directions=[p.gravity_unit_mean for p in points],
        )
        values = result.to_dict()
        quality = {
            k: v
            for k, v in values.items()
            if k not in ("tare_offset", "tool_mass_kg", "tool_com_m")
        }
        quality["kinematics"] = "piper_x_sdk_link6"
        path = save_sensor_calibration_to_yaml(
            self.arm_id,
            tare_offset=result.tare_offset,
            tool_mass_kg=result.tool_mass_kg,
            tool_com_m=result.tool_com_m,
            calibration_samples=[p.to_dict() for p in points],
            calibration_quality=quality,
            config_path=self.config_file,
        )
        future = self.reload_client.call_async(Trigger.Request())
        while not future.done():
            if self._stop_event.is_set():
                raise RuntimeError("校准结果已保存，配置重载请求尚未完成")
            time.sleep(0.005)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(
                f"结果已保存到 {path}，driver 重载失败: {getattr(response, 'message', 'no response')}"
            )
        return {**values, "config_file": str(path), "driver_reload": response.message}

    def destroy_node(self):
        self._stop_event.set()
        return super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(
        description="独立的 PaXini 多姿态零偏和重力校准程序"
    )
    parser.add_argument(
        "--arm", default="arm_3", help="arm_3 / arm_4 / 左手 / 右手 / all"
    )
    parser.add_argument("--config-file", default="")
    parser.add_argument("action", choices=("record", "list", "clear", "run"))
    options = parser.parse_args(args)
    cfg = source_config_path(Path(options.config_file) if options.config_file else None)
    entries = resolve_sensor_configs(options.arm, str(cfg))
    if options.action == "list":
        for entry in entries:
            print(
                json.dumps(
                    {
                        k: entry.get(k)
                        for k in (
                            "arm_id",
                            "calibration_poses",
                            "tare_offset",
                            "tool_mass_kg",
                            "tool_com_m",
                            "calibration_quality",
                        )
                    },
                    ensure_ascii=False,
                    indent=2,
                )
            )
        return
    if options.action == "clear":
        save_sensor_calibration_to_yaml(
            entries[0]["arm_id"], calibration_poses=[], config_path=cfg
        )
        print("已清空公共校准姿态")
        return
    rclpy.init()
    try:
        for entry in entries:
            node = PoseCalibrator(entry["arm_id"], cfg)
            executor = MultiThreadedExecutor()
            executor.add_node(node)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            try:
                result = (
                    node.record_pose() if options.action == "record" else node.run()
                )
                print(
                    json.dumps(result, ensure_ascii=False, indent=2)
                    if isinstance(result, dict)
                    else result
                )
            finally:
                node._stop_event.set()
                executor.shutdown()
                thread.join(timeout=2.0)
                node.destroy_node()
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
