"""ROS 2 joint_state subscriber and pure MIT command publisher.

Isolates ROS transport from camera/inference/dashboard logic.
Publishes exclusively to MIT topics (/command/mit + /command/gripper); never Cartesian.
"""
from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Sequence

from .config import ArmConfig

JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


@dataclass
class ArmFeedback:
    arm_id: str
    joints_rad: list[float]
    velocities_rad_s: list[float]
    gripper_m: float
    receive_monotonic_ns: int
    receive_unix_ns: int
    sequence: int


class RosArmBridge:
    """Subscribes to joint_state topics and publishes MIT joint/gripper commands."""

    def __init__(self, arm_configs: Sequence[ArmConfig], *, enable_ros: bool = True):
        self.arm_configs = {cfg.id: cfg for cfg in arm_configs}
        self.enable_ros = enable_ros
        self._lock = threading.Lock()
        self._feedback: dict[str, ArmFeedback] = {}
        self._last_sent: dict[str, tuple[list[float], float | None, int]] = {}
        self._stop = threading.Event()
        self._context = None
        self._node = None
        self._executor = None
        self._spin_thread: threading.Thread | None = None
        self._mit_pubs = {}
        self._grip_pubs = {}
        self._mit_cls = None
        self._grip_cls = None
        if enable_ros:
            self._start_ros()

    def _start_ros(self) -> None:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import JointState
        from piper_x_interfaces.msg import GripperCommand, MitCommand

        self._mit_cls = MitCommand
        self._grip_cls = GripperCommand
        self._context = Context()
        rclpy.init(context=self._context, signal_handler_options=SignalHandlerOptions.NO)
        self._node = rclpy.create_node(
            "piperx_cogact_client", context=self._context, enable_rosout=False, start_parameter_services=False,
        )
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)

        sub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        pub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)

        for arm_id, cfg in self.arm_configs.items():
            self._node.create_subscription(
                JointState, cfg.joint_state_topic, lambda msg, aid=arm_id: self._on_joint_state(aid, msg), sub_qos,
            )
            self._mit_pubs[arm_id] = self._node.create_publisher(MitCommand, cfg.mit_topic, pub_qos)
            self._grip_pubs[arm_id] = self._node.create_publisher(GripperCommand, cfg.gripper_topic, pub_qos)

        self._spin_thread = threading.Thread(target=self._spin_loop, name="cogact-ros-spin", daemon=True)
        self._spin_thread.start()

    def _spin_loop(self) -> None:
        while not self._stop.is_set() and self._context is not None and self._context.ok():
            try:
                self._executor.spin_once(timeout_sec=0.05)
            except Exception:
                if self._stop.is_set():
                    break

    def _on_joint_state(self, arm_id: str, msg) -> None:
        names = list(msg.name) if msg.name else []
        positions = [float(v) for v in msg.position]
        velocities = [float(v) for v in msg.velocity] if len(msg.velocity) >= len(positions) else [0.0] * len(positions)
        if names and all(n in names for n in JOINT_NAMES):
            idx_map = {n: i for i, n in enumerate(names)}
            joints = [positions[idx_map[n]] for n in JOINT_NAMES]
            vels = [velocities[idx_map[n]] for n in JOINT_NAMES]
            grip = positions[idx_map["gripper"]] if "gripper" in idx_map else (positions[6] if len(positions) > 6 else 0.0)
        elif len(positions) >= 6:
            joints = positions[:6]
            vels = velocities[:6]
            grip = positions[6] if len(positions) > 6 else 0.0
        else:
            return
        now_mono = time.monotonic_ns()
        now_unix = time.time_ns()
        with self._lock:
            prev = self._feedback.get(arm_id)
            seq = 1 if prev is None else prev.sequence + 1
            self._feedback[arm_id] = ArmFeedback(
                arm_id=arm_id,
                joints_rad=joints,
                velocities_rad_s=vels,
                gripper_m=grip,
                receive_monotonic_ns=now_mono,
                receive_unix_ns=now_unix,
                sequence=seq,
            )

    def inject_feedback(self, arm_id: str, joints_rad: Sequence[float], gripper_m: float = 0.0) -> None:
        """Test/offline helper to populate arm joint_state without ROS."""
        now_mono = time.monotonic_ns()
        now_unix = time.time_ns()
        with self._lock:
            prev = self._feedback.get(arm_id)
            self._feedback[arm_id] = ArmFeedback(
                arm_id=arm_id,
                joints_rad=[float(v) for v in joints_rad[:6]],
                velocities_rad_s=[0.0] * 6,
                gripper_m=float(gripper_m),
                receive_monotonic_ns=now_mono,
                receive_unix_ns=now_unix,
                sequence=1 if prev is None else prev.sequence + 1,
            )

    def get_feedback(self, arm_id: str) -> ArmFeedback | None:
        with self._lock:
            fb = self._feedback.get(arm_id)
            if fb is None:
                return None
            return ArmFeedback(
                arm_id=fb.arm_id,
                joints_rad=list(fb.joints_rad),
                velocities_rad_s=list(fb.velocities_rad_s),
                gripper_m=fb.gripper_m,
                receive_monotonic_ns=fb.receive_monotonic_ns,
                receive_unix_ns=fb.receive_unix_ns,
                sequence=fb.sequence,
            )

    def get_all_feedback(self) -> dict[str, ArmFeedback]:
        return {aid: fb for aid in self.arm_configs if (fb := self.get_feedback(aid)) is not None}

    def publish_mit(
        self,
        arm_id: str,
        joints_rad: Sequence[float],
        *,
        gripper_m: float | None = None,
        enable_gripper: bool = True,
    ) -> None:
        cfg = self.arm_configs[arm_id]
        q = [float(v) for v in joints_rad[:6]]
        if len(q) != 6:
            raise ValueError(f"{arm_id} MIT command requires 6 joint positions")
        with self._lock:
            prev_count = self._last_sent.get(arm_id, ([], None, 0))[2]
            self._last_sent[arm_id] = (q, None if gripper_m is None else float(gripper_m), prev_count + 1)

        if not self.enable_ros or self._node is None:
            return
        mit_msg = self._mit_cls()
        mit_msg.position[:] = q
        mit_msg.velocity[:] = [0.0] * 6
        mit_msg.kp[:] = cfg.kp[:6]
        mit_msg.kd[:] = cfg.kd[:6]
        mit_msg.torque[:] = [0.0] * 6
        self._mit_pubs[arm_id].publish(mit_msg)

        if enable_gripper and gripper_m is not None:
            grip_msg = self._grip_cls()
            grip_msg.width = float(gripper_m)
            grip_msg.effort = float(cfg.gripper_effort)
            self._grip_pubs[arm_id].publish(grip_msg)

    def last_sent_info(self, arm_id: str) -> dict | None:
        with self._lock:
            entry = self._last_sent.get(arm_id)
            if entry is None:
                return None
            return {"joints_rad": list(entry[0]), "gripper_m": entry[1], "tx_count": entry[2]}

    def close(self) -> None:
        self._stop.set()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
        if self._executor is not None:
            try:
                self._executor.shutdown()
            except Exception:
                pass
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
        if self._context is not None:
            try:
                self._context.try_shutdown()
            except Exception:
                pass
            self._context = None

