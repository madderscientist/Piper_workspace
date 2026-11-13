"""观测 -> 云端推理 -> C++ IK -> ROS MIT 执行。

支持单步、整批、串行闭环和异步重叠，以及目标维持、标定与平滑归位。
"""
from __future__ import annotations

from collections import deque
from collections.abc import Iterator
from concurrent.futures import CancelledError
from contextlib import contextmanager
from dataclasses import dataclass
import threading
import time
from typing import Any

import numpy as np

from .calibration import CalibrationManager, detect_charuco_board, invert_se3
from .cogact_api import ActionChunk, ActionStep, ArmTarget, CogActClient
from .config import ClientConfig
from .kinematics import DriverKinematics
from .ros_io import RosArmBridge
from .streams import CapturedSnapshot, StreamHub


@dataclass
class QueuedAction:
    arms: dict[str, tuple[list[float], float | None]]  # arm_id -> (joints_rad[6], gripper_m)
    chunk_id: str
    step_index: int
    blended: bool = False
    new_weight: float = 1.0


def blend_action_queues(
    old_actions: list[QueuedAction],
    new_actions: list[QueuedAction],
) -> tuple[list[QueuedAction], int]:
    """Linearly blend overlapping prefix using virtual boundary nodes (old=1,new=0) and (old=0,new=1)."""
    if not new_actions:
        return list(old_actions), 0
    overlap = min(len(old_actions), len(new_actions))
    if overlap == 0:
        return list(new_actions), 0

    merged: list[QueuedAction] = []
    for j in range(overlap):
        # 两端保留纯旧/纯新的虚拟节点，重叠段只取开区间内的混合权重。
        w = (j + 1) / float(overlap + 1)
        old_item = old_actions[j]
        new_item = new_actions[j]
        blended_arms: dict[str, tuple[list[float], float | None]] = {}
        for arm_id, (new_q, new_g) in new_item.arms.items():
            if arm_id in old_item.arms:
                old_q, old_g = old_item.arms[arm_id]
                q_mix = [(1.0 - w) * oq + w * nq for oq, nq in zip(old_q, new_q)]
                if old_g is not None and new_g is not None:
                    g_mix: float | None = (1.0 - w) * old_g + w * new_g
                else:
                    g_mix = new_g if new_g is not None else old_g
                blended_arms[arm_id] = (q_mix, g_mix)
            else:
                blended_arms[arm_id] = (list(new_q), new_g)
        merged.append(QueuedAction(
            arms=blended_arms,
            chunk_id=new_item.chunk_id,
            step_index=new_item.step_index,
            blended=True,
            new_weight=round(w, 4),
        ))
    merged.extend(new_actions[overlap:])
    return merged, overlap


class ExecutionEngine:
    def __init__(
        self,
        config: ClientConfig,
        arm_bridge: RosArmBridge,
        stream_hub: StreamHub,
        calibration_mgr: CalibrationManager,
        cogact_client: CogActClient,
        *,
        kinematics: DriverKinematics | None = None,
    ):
        self.config = config
        self.arm_bridge = arm_bridge
        self.stream_hub = stream_hub
        self.calibration = calibration_mgr
        self.client = cogact_client
        self.kinematics = kinematics or stream_hub.kinematics

        self._lock = threading.RLock()
        self._cancel = threading.Event()
        self._shutdown = threading.Event()
        self._worker: threading.Thread | None = None
        self._consumer_worker: threading.Thread | None = None
        self._async_ready = threading.Condition(threading.Lock())
        self._async_queue: deque[QueuedAction] = deque()
        self._async_next_tick_t: float | None = None

        self.mode: str = "idle"  # idle | ready | requesting | batch_running | auto_running | async_running | homing | error
        self.message: str = "就绪"
        self.last_error: str | None = None
        self.current_chunk: ActionChunk | None = None
        self.step_cursor: int = 0
        self.auto_cycles: int = 0
        self.async_stats: dict[str, Any] | None = None
        self._held_targets: dict[str, tuple[list[float], float | None, float]] = {}
        self._logs: deque[dict[str, Any]] = deque(maxlen=80)

        self._hold_thread = threading.Thread(target=self._hold_loop, name="cogact-mit-hold", daemon=True)
        self._hold_thread.start()

    def log(self, text: str, level: str = "info", *, cancel: threading.Event | None = None) -> None:
        with self._lock:
            if cancel is not None and (cancel.is_set() or cancel is not self._cancel):
                return
            self._logs.appendleft({
                "time": time.strftime("%H:%M:%S"),
                "level": level,
                "text": text,
            })
            self.message = text
            if level == "error":
                self.last_error = text

    @contextmanager
    def _task_lock(self, cancel: threading.Event) -> Iterator[None]:
        with self._lock:
            # 切换模式会替换取消令牌，旧请求即使迟到也不能写入新任务或下发动作。
            if cancel.is_set() or cancel is not self._cancel:
                raise CancelledError("执行已停止")
            yield

    def _fail_task(self, cancel: threading.Event, message: str) -> None:
        with self._lock:
            if cancel.is_set() or cancel is not self._cancel:
                return
            self.mode = "error"
            self.log(message, level="error")

    def _hold_loop(self) -> None:
        while not self._shutdown.is_set():
            hz = self.config.execution.hold_hz
            if hz <= 0.0:
                self._shutdown.wait(0.1)
                continue
            period = 1.0 / max(1.0, hz)
            now = time.monotonic()
            with self._lock:
                for arm_id, (joints, grip, last_t) in list(self._held_targets.items()):
                    if now - last_t >= period:
                        self._held_targets[arm_id] = (joints, grip, now)
                        try:
                            self.arm_bridge.publish_mit(
                                arm_id, joints, gripper_m=grip, enable_gripper=self.config.execution.enable_gripper,
                            )
                        except Exception:
                            pass
            self._shutdown.wait(min(0.02, period * 0.5))

    def _ensure_not_busy(self) -> None:
        if (self._worker is not None and self._worker.is_alive()) or (
            self._consumer_worker is not None and self._consumer_worker.is_alive()
        ):
            raise RuntimeError("当前已有后台任务正在运行，请先等待完成或点击「停止」")

    def _solve_chunk_ik(self, chunk: ActionChunk, snap: CapturedSnapshot) -> dict[str, Any]:
        return self.kinematics.resolve_chunk_ik(
            chunk,
            arm_configs=self.config.arms,
            calibration_data=self.calibration.snapshot_transforms(),
            seed_joints_by_arm={aid: fb.joints_rad for aid, fb in snap.arms_by_id.items()},
            ik_cfg=self.config.ik,
        )

    def _predict_and_solve(
        self, snap: CapturedSnapshot, instruction: str | None, cancel: threading.Event | None = None,
    ) -> ActionChunk:
        chunk = self.client.predict(
            observation_monotonic_ns=snap.monotonic_ns,
            observation_unix_ns=snap.unix_ns,
            arms=snap.arms_by_role,
            cameras=snap.cameras,
            calibration_id=snap.calibration_id,
            instruction=instruction,
        )
        if cancel is not None:
            with self._task_lock(cancel):
                pass
        self._solve_chunk_ik(chunk, snap)
        return chunk

    def preview_once(self, instruction: str | None = None) -> tuple[CapturedSnapshot, ActionChunk]:
        """每次重新采集与推理；不写入真实执行批次，也不下发动作。"""
        snap = self.stream_hub.capture_snapshot(self.calibration)
        return snap, self._predict_and_solve(snap, instruction)

    def _resolve_arm_joints(self, step: ActionStep) -> dict[str, tuple[list[float], float | None]]:
        out: dict[str, tuple[list[float], float | None]] = {}
        for arm_cfg in self.config.arms:
            target: ArmTarget = step.by_role(arm_cfg.role)
            if target.joint_positions_rad is None:
                raise RuntimeError(f"第 {step.step_index} 步 ({arm_cfg.id}) 缺少关节解")
            joints = list(target.joint_positions_rad[:6])
            out[arm_cfg.id] = (joints, target.gripper_opening_m)
        return out

    def _publish_arm_commands(
        self, commands: dict[str, tuple[list[float], float | None]], cancel: threading.Event,
    ) -> None:
        with self._task_lock(cancel):
            now = time.monotonic()
            for arm_id, (joints, grip) in commands.items():
                self.arm_bridge.publish_mit(
                    arm_id, joints, gripper_m=grip, enable_gripper=self.config.execution.enable_gripper,
                )
                self._held_targets[arm_id] = (joints, grip, now)

    def _publish_step(self, step: ActionStep, cancel: threading.Event) -> None:
        self._publish_arm_commands(self._resolve_arm_joints(step), cancel)

    def capture_only(self) -> dict[str, Any]:
        snap = self.stream_hub.capture_snapshot(self.calibration)
        self.log(f"已截取本地快照 {snap.snapshot_id}（{len(snap.cameras)} 路相机，{len(snap.arms_by_id)} 条机械臂）")
        return {"snapshot_id": snap.snapshot_id}

    def request_once(self, instruction: str | None = None) -> ActionChunk:
        with self._lock:
            self._ensure_not_busy()
            self._cancel = cancel = threading.Event()
        return self._request_chunk(instruction, cancel)

    def _request_chunk(self, instruction: str | None, cancel: threading.Event) -> ActionChunk:
        with self._task_lock(cancel):
            self.mode = "requesting"
            self.last_error = None
        try:
            snap = self.stream_hub.capture_snapshot(self.calibration)
            chunk = self._predict_and_solve(snap, instruction, cancel)
            with self._task_lock(cancel):
                self.current_chunk = chunk
                self.step_cursor = 0
                self.mode = "ready"
            ik_info = chunk.ik_summary or {}
            self.log(
                f"推理+IK完成：{len(chunk.steps)} 步 ({chunk.action_kind})，"
                f"云端 {chunk.latency_ms:.0f} ms，IK {ik_info.get('solve_ms', 0):.1f} ms "
                f"(最大残差 {ik_info.get('max_pos_err_mm', 0):.2f} mm)", cancel=cancel,
            )
            return chunk
        except CancelledError:
            raise
        except Exception as exc:
            self._fail_task(cancel, f"推理请求失败：{exc}")
            raise

    def step_once(self, instruction: str | None = None) -> dict[str, Any]:
        """Built-in Mode 1: Execute exactly 1 action per click from the fetched chunk."""
        with self._lock:
            self._ensure_not_busy()
            self._cancel = cancel = threading.Event()
            need_fetch = self.current_chunk is None
        if need_fetch:
            self._request_chunk(instruction, cancel)
        with self._task_lock(cancel):
            assert self.current_chunk is not None
            total = len(self.current_chunk.steps)
            if self.step_cursor >= total:
                raise RuntimeError(f"当前批次 {total}/{total} 步已全部执行完，请点击「请求」获取新批次")
            step = self.current_chunk.steps[self.step_cursor]
        try:
            self._publish_step(step, cancel)
        except CancelledError:
            raise
        except Exception as exc:
            self._fail_task(cancel, str(exc))
            raise
        with self._task_lock(cancel):
            self.step_cursor += 1
            done = self.step_cursor >= total
            self.mode = "idle" if done else "ready"
            cur = self.step_cursor
        self.log(f"单步执行：已下发第 {cur}/{total} 步 MIT 指令", cancel=cancel)
        return {"step_cursor": cur, "total_steps": total}

    def start_batch(self, *, fetch_new: bool = True, instruction: str | None = None) -> None:
        """Built-in Mode 2: Execute all actions of 1 request at configured execute_hz."""
        with self._lock:
            self._ensure_not_busy()
            self._cancel = cancel = threading.Event()
            self.last_error = None
            self.mode = "batch_running"
            self._worker = threading.Thread(
                target=self._batch_worker, args=(fetch_new, instruction, cancel), name="cogact-batch", daemon=True,
            )
            self._worker.start()

    def _execute_loaded_chunk_at_rate(self, chunk: ActionChunk, cancel: threading.Event) -> bool:
        hz = max(0.5, float(self.config.execution.execute_hz))
        period = 1.0 / hz
        total = len(chunk.steps)
        with self._task_lock(cancel):
            start_idx = self.step_cursor if 0 < self.step_cursor < total else 0
            self.step_cursor = start_idx
        next_deadline = time.monotonic()
        for idx in range(start_idx, total):
            if cancel.is_set():
                return False
            now = time.monotonic()
            if next_deadline > now:
                if cancel.wait(next_deadline - now):
                    return False
            self._publish_step(chunk.steps[idx], cancel)
            with self._task_lock(cancel):
                self.step_cursor = idx + 1
            next_deadline += period
        return True

    def _batch_worker(self, fetch_new: bool, instruction: str | None, cancel: threading.Event) -> None:
        try:
            with self._task_lock(cancel):
                chunk = self.current_chunk
            if fetch_new or chunk is None:
                snap = self.stream_hub.capture_snapshot(self.calibration)
                self.log("整批执行：正在请求云端推理...", cancel=cancel)
                chunk = self._predict_and_solve(snap, instruction, cancel)
                with self._task_lock(cancel):
                    self.current_chunk = chunk
                    self.step_cursor = 0
            self.log(f"整批执行：按 {self.config.execution.execute_hz:g} Hz 下发 {len(chunk.steps)} 步动作...", cancel=cancel)
            completed = self._execute_loaded_chunk_at_rate(chunk, cancel)
            with self._task_lock(cancel):
                self.mode = "idle"
            self.log(f"整批执行完成（共 {len(chunk.steps)} 步）" if completed else "整批执行已中途停止", cancel=cancel)
        except CancelledError:
            pass
        except Exception as exc:
            self._fail_task(cancel, f"整批执行中止：{exc}")

    def start_auto_loop(self, instruction: str | None = None) -> None:
        """Built-in Mode 3: Serial loop [capture -> request -> batch execute -> wait settle_delay_s]."""
        with self._lock:
            self._ensure_not_busy()
            self._cancel = cancel = threading.Event()
            self.last_error = None
            self.auto_cycles = 0
            self.mode = "auto_running"
            self._worker = threading.Thread(
                target=self._auto_worker, args=(instruction, cancel), name="cogact-auto-loop", daemon=True,
            )
            self._worker.start()

    def _auto_worker(self, instruction: str | None, cancel: threading.Event) -> None:
        try:
            while not cancel.is_set():
                with self._task_lock(cancel):
                    cycle = self.auto_cycles + 1
                snap = self.stream_hub.capture_snapshot(self.calibration)
                self.log(f"连续闭环 #{cycle}：已截取状态，正在请求推理...", cancel=cancel)
                chunk = self._predict_and_solve(snap, instruction, cancel)
                with self._task_lock(cancel):
                    self.current_chunk = chunk
                    self.step_cursor = 0
                if cancel.is_set():
                    break
                self.log(f"连续闭环 #{cycle}：按 {self.config.execution.execute_hz:g} Hz 执行 {len(chunk.steps)} 步...", cancel=cancel)
                if not self._execute_loaded_chunk_at_rate(chunk, cancel):
                    break
                with self._task_lock(cancel):
                    self.auto_cycles = cycle
                delay_s = max(0.0, float(self.config.execution.settle_delay_s))
                self.log(f"连续闭环 #{cycle} 完成，静止等待 {delay_s:g} s 后发起下一轮...", cancel=cancel)
                if delay_s > 0.0 and cancel.wait(delay_s):
                    break
            with self._task_lock(cancel):
                self.mode = "idle"
            self.log(f"连续闭环已停止（累计完成 {self.auto_cycles} 轮）", cancel=cancel)
        except CancelledError:
            pass
        except Exception as exc:
            self._fail_task(cancel, f"连续闭环异常停止：{exc}")

    def start_async_loop(self, instruction: str | None = None) -> None:
        """Built-in Mode 4: Async request/execution pipeline with latency skip t and linear overlap blending."""
        with self._lock:
            self._ensure_not_busy()
            self._cancel = cancel = threading.Event()
            self.last_error = None
            self.auto_cycles = 0
            self.async_stats = {
                "chunks": 0,
                "latency_ms": 0.0,
                "start_index": 0,
                "overlap_steps": 0,
                "queue_len": 0,
            }
            self.mode = "async_running"
            with self._async_ready:
                self._async_queue.clear()
                self._async_next_tick_t = None
            self._consumer_worker = threading.Thread(
                target=self._async_consumer_loop, args=(cancel,), name="cogact-async-exec", daemon=True,
            )
            self._worker = threading.Thread(
                target=self._async_producer_loop, args=(instruction, cancel), name="cogact-async-req", daemon=True,
            )
            self._consumer_worker.start()
            self._worker.start()

    def _async_producer_loop(self, instruction: str | None, cancel: threading.Event) -> None:
        try:
            while not cancel.is_set():
                req_t = time.monotonic()
                snap = self.stream_hub.capture_snapshot(self.calibration)
                chunk = self._predict_and_solve(snap, instruction, cancel)
                if cancel.is_set():
                    break
                recv_t = time.monotonic()
                latency_s = max(0.0, recv_t - req_t)
                hz = max(0.5, float(self.config.execution.execute_hz))
                total = len(chunk.steps)
                if total == 0:
                    raise RuntimeError("云端返回空动作序列")

                with self._task_lock(cancel), self._async_ready:
                    if self._async_queue and self._async_next_tick_t is not None:
                        target_t = max(recv_t, self._async_next_tick_t)
                        raw_idx = max(0, int(round((target_t - req_t) * hz)))
                    else:
                        raw_idx = max(0, int(latency_s * hz))
                    start_idx = min(raw_idx, total - 1)
                    new_actions = [
                        QueuedAction(
                            arms=self._resolve_arm_joints(s),
                            chunk_id=chunk.request_id,
                            step_index=s.step_index,
                        )
                        for s in chunk.steps[start_idx:]
                    ]
                    merged, overlap_len = blend_action_queues(list(self._async_queue), new_actions)
                    self._async_queue = deque(merged)
                    q_len = len(self._async_queue)
                    self._async_ready.notify_all()

                with self._task_lock(cancel):
                    self.current_chunk = chunk
                    self.step_cursor = start_idx
                    self.auto_cycles += 1
                    cycle = self.auto_cycles
                    self.async_stats = {
                        "chunks": cycle,
                        "latency_ms": round(latency_s * 1000.0, 1),
                        "start_index": start_idx,
                        "raw_skip_steps": raw_idx,
                        "overlap_steps": overlap_len,
                        "queue_len": q_len,
                    }
                if raw_idx >= total:
                    self.log(
                        f"异步 #{cycle}：延时 {latency_s*1000:.0f}ms 超过整批时长 {total/hz*1000:.0f}ms "
                        f"(应跳 {raw_idx} 步，取末步 #{total})，立即发起下一请求",
                        level="warn", cancel=cancel,
                    )
                else:
                    self.log(
                        f"异步 #{cycle}：延时 {latency_s*1000:.0f}ms，从第 {start_idx+1}/{total} 步切入，"
                        f"重叠线性混合 {overlap_len} 步，立即发起下一请求", cancel=cancel,
                    )
            with self._task_lock(cancel):
                if self.mode == "async_running":
                    self.mode = "idle"
            self.log(f"异步重叠执行已停止（累计接收 {self.auto_cycles} 批）", cancel=cancel)
        except CancelledError:
            pass
        except Exception as exc:
            with self._lock, self._async_ready:
                if cancel.is_set() or cancel is not self._cancel:
                    return
                self._fail_task(cancel, f"异步重叠执行异常停止：{exc}")
                cancel.set()
                self._async_queue.clear()
                self._async_ready.notify_all()

    def _async_consumer_loop(self, cancel: threading.Event) -> None:
        try:
            while not cancel.is_set():
                hz = max(0.5, float(self.config.execution.execute_hz))
                period = 1.0 / hz
                with self._async_ready:
                    while not self._async_queue and not cancel.is_set():
                        self._async_next_tick_t = None
                        self._async_ready.wait(timeout=0.05)
                    if cancel.is_set():
                        break
                    now = time.monotonic()
                    if self._async_next_tick_t is None or self._async_next_tick_t < now - period:
                        self._async_next_tick_t = now
                    wait_s = self._async_next_tick_t - now
                if wait_s > 0.0 and cancel.wait(wait_s):
                    break
                with self._async_ready:
                    if not self._async_queue or cancel.is_set():
                        continue
                    item = self._async_queue.popleft()
                    q_rem = len(self._async_queue)
                    self._async_next_tick_t = (
                        (self._async_next_tick_t + period) if (q_rem > 0 and self._async_next_tick_t is not None) else None
                    )
                self._publish_arm_commands(item.arms, cancel)
                with self._task_lock(cancel):
                    self.step_cursor = item.step_index
                    if self.async_stats is not None:
                        self.async_stats["queue_len"] = q_rem
        except CancelledError:
            pass
        except Exception as exc:
            with self._lock:
                self._fail_task(cancel, f"异步下发线程异常：{exc}")
                cancel.set()

    def move_to_calibrated_home(self) -> None:
        """Smoothly move configured arms via MIT quintic interpolation to persisted home poses."""
        with self._lock:
            self._ensure_not_busy()
            homes = self.calibration.snapshot_transforms().get("home_poses") or {}
            if not homes:
                raise RuntimeError("尚未标定落盘双臂起始位置，请先点击「记录当前双臂位置并落盘」")
            self._cancel = cancel = threading.Event()
            self.mode = "homing"
            self._worker = threading.Thread(target=self._home_worker, args=(homes, cancel), name="cogact-home", daemon=True)
            self._worker.start()

    def _home_worker(self, homes: dict[str, Any], cancel: threading.Event) -> None:
        try:
            starts: dict[str, tuple[np.ndarray, np.ndarray, float]] = {}
            for arm_cfg in self.config.arms:
                if arm_cfg.id not in homes:
                    continue
                fb = self.arm_bridge.get_feedback(arm_cfg.id)
                if fb is None:
                    raise RuntimeError(f"未收到 {arm_cfg.id} 当前关节反馈，无法规划回起始位")
                q0 = np.asarray(fb.joints_rad[:6], dtype=np.float64)
                q1 = np.asarray(homes[arm_cfg.id]["joints_rad"][:6], dtype=np.float64)
                grip = float(homes[arm_cfg.id].get("gripper_m", fb.gripper_m))
                starts[arm_cfg.id] = (q0, q1, grip)
            if not starts:
                raise RuntimeError("当前受控机械臂在标定文件中没有起始位置记录")
            hz = max(10.0, float(self.config.execution.execute_hz))
            duration = max(0.3, float(self.config.execution.home_move_duration_s))
            steps = max(2, int(round(duration * hz)))
            dt = duration / steps
            self.log(f"正在平滑回到已落盘标定起始位（耗时 {duration:g} s）...", cancel=cancel)
            next_t = time.monotonic()
            for k in range(1, steps + 1):
                if cancel.is_set():
                    break
                now = time.monotonic()
                if next_t > now and cancel.wait(next_t - now):
                    break
                u = k / float(steps)
                # 五次插值让规划轨迹两端的速度、加速度为零。
                blend = u * u * u * (10.0 + u * (-15.0 + 6.0 * u))
                commands = {}
                for arm_id, (q0, q1, grip) in starts.items():
                    qk = (q0 + blend * (q1 - q0)).tolist()
                    commands[arm_id] = (qk, grip)
                self._publish_arm_commands(commands, cancel)
                next_t += dt
            with self._task_lock(cancel):
                self.mode = "idle"
            self.log("已到达标定起始位置", cancel=cancel)
        except CancelledError:
            pass
        except Exception as exc:
            self._fail_task(cancel, f"回起始位失败：{exc}")

    def calibrate_charuco(self, targets: list[str] | None = None, world_origin: str | None = None) -> dict[str, Any]:
        """Run ChArUco detection on selected views (head / left / right) and persist extrinsics."""
        from wrist_camera import camera_pose_in_base

        wanted = targets or ["head", "left", "right"]
        results: dict[str, Any] = {}
        role_to_cam = {c.role: c.id for c in self.config.cameras}

        for target in wanted:
            cam_role = "head" if target == "head" else f"wrist_{target}"
            cam_id = role_to_cam.get(cam_role)
            if not cam_id:
                results[target] = {"ok": False, "reason": f"未配置 {cam_role} 相机"}
                continue
            frame = self.stream_hub.get_live_frame(cam_id)
            if frame is None:
                results[target] = {"ok": False, "reason": "尚未收到相机画面"}
                continue
            if frame.intrinsic_matrix is None:
                results[target] = {"ok": False, "reason": "该相机缺少当前分辨率的内参 K"}
                continue
            det = detect_charuco_board(
                frame.ensure_bgr(), frame.intrinsic_matrix, frame.distortion_coefficients or [],
                self.calibration.board_cfg, label=cam_id,
            )
            with self.calibration.lock:
                self.calibration.overlays[target] = det["overlay_jpeg"]
            if not det["ok"]:
                results[target] = {"ok": False, "reason": det["reason"], "corner_count": det["corner_count"]}
                continue
            t_cam_board = det["T_cam_board"]
            if target == "head":
                t_board_frame = invert_se3(t_cam_board)
            else:
                arm_cfg = self.config.arm_by_role(target)
                if arm_cfg is None:
                    results[target] = {"ok": False, "reason": f"未配置 {target} 机械臂"}
                    continue
                fb = self.arm_bridge.get_feedback(arm_cfg.id)
                if fb is None:
                    results[target] = {"ok": False, "reason": f"未收到 {arm_cfg.id} 关节反馈"}
                    continue
                t_base_cam = camera_pose_in_base(arm_cfg.id, fb.joints_rad)
                t_base_board = t_base_cam @ t_cam_board
                t_board_frame = invert_se3(t_base_board)

            self.calibration.update_board_observation(
                target, t_board_frame,
                corner_count=det["corner_count"],
                reproj_rms_px=det["reproj_rms_px"],
                overlay_jpeg=det["overlay_jpeg"],
                world_origin=world_origin,
            )
            results[target] = {
                "ok": True,
                "corner_count": det["corner_count"],
                "reproj_rms_px": det["reproj_rms_px"],
            }
        ok_names = [k for k, v in results.items() if v.get("ok")]
        if ok_names:
            joined = ", ".join(ok_names)
            self.log(f"标定完成并已落盘：{joined} -> {self.calibration.path.name}")
        else:
            self.log("标定未更新：所选视角均未满足角点/反馈条件", level="warn")
        return {"results": results, "calibration": self.calibration.summary()}

    def record_home_poses(self) -> dict[str, Any]:
        states = {}
        for arm_cfg in self.config.arms:
            fb = self.arm_bridge.get_feedback(arm_cfg.id)
            if fb is None:
                raise RuntimeError(f"尚未收到 {arm_cfg.id} 关节反馈，无法记录起始位")
            states[arm_cfg.id] = {"joints_rad": fb.joints_rad, "gripper_m": fb.gripper_m}
        saved = self.calibration.record_home_poses(states)
        self.log(f"已记录双臂标定起始位并落盘至 {self.calibration.path.name}")
        return saved

    def stop(self, *, release_mit: bool = True, clear_results: bool = False) -> None:
        with self._lock:
            self._cancel.set()
            if release_mit:
                self._held_targets.clear()
            with self._async_ready:
                self._async_queue.clear()
                self._async_next_tick_t = None
                self._async_ready.notify_all()
            worker, consumer = self._worker, self._consumer_worker
            self.mode = "idle"
            if clear_results:
                self.current_chunk = None
                self.step_cursor = 0
                self.auto_cycles = 0
                self.async_stats = None
                self.last_error = None
                self._logs.clear()
                self._worker = self._consumer_worker = None
            self.log("已停止执行" + ("并释放 MIT 目标维持" if release_mit else "（保留当前 MIT 目标维持）"))
        if clear_results:
            return
        if worker is not None and worker.is_alive() and threading.current_thread() is not worker:
            worker.join(timeout=2.0)
        if consumer is not None and consumer.is_alive() and threading.current_thread() is not consumer:
            consumer.join(timeout=2.0)

    def update_settings(self, payload: dict[str, Any]) -> None:
        with self._lock:
            if "server_url" in payload and str(payload["server_url"]).strip():
                self.config.server.url = str(payload["server_url"]).strip()
                self.client.url = self.config.server.url
            if "instruction" in payload:
                self.config.server.instruction = str(payload["instruction"])
                self.client.instruction = self.config.server.instruction
            if "execute_hz" in payload and float(payload["execute_hz"]) > 0:
                self.config.execution.execute_hz = float(payload["execute_hz"])
            if "settle_delay_s" in payload and float(payload["settle_delay_s"]) >= 0:
                self.config.execution.settle_delay_s = float(payload["settle_delay_s"])
            if "hold_hz" in payload and float(payload["hold_hz"]) >= 0:
                self.config.execution.hold_hz = float(payload["hold_hz"])
            if "enable_gripper" in payload:
                self.config.execution.enable_gripper = bool(payload["enable_gripper"])
        self.log("运行参数已更新")

    def status_dict(self) -> dict[str, Any]:
        now_mono = time.monotonic_ns()
        arms_out = []
        for arm_cfg in self.config.arms:
            fb = self.arm_bridge.get_feedback(arm_cfg.id)
            sent = self.arm_bridge.last_sent_info(arm_cfg.id)
            age_ms = None if fb is None else round((now_mono - fb.receive_monotonic_ns) / 1e6, 1)
            with self._lock:
                held = self._held_targets.get(arm_cfg.id)
            arms_out.append({
                "id": arm_cfg.id,
                "role": arm_cfg.role,
                "label": arm_cfg.label,
                "topic": arm_cfg.joint_state_topic,
                "mit_topic": arm_cfg.mit_topic,
                "online": fb is not None and age_ms is not None and age_ms < 2000.0,
                "age_ms": age_ms,
                "sequence": 0 if fb is None else fb.sequence,
                "joints_rad": None if fb is None else [round(v, 4) for v in fb.joints_rad],
                "gripper_m": None if fb is None else round(fb.gripper_m, 4),
                "holding": held is not None and self.config.execution.hold_hz > 0.0,
                "last_sent": sent,
            })
        snap = self.stream_hub.latest_snapshot()
        snap_info = None
        if snap is not None:
            snap_info = {
                "snapshot_id": snap.snapshot_id,
                "unix_ms": snap.unix_ns // 1_000_000,
                "calibration_id": snap.calibration_id,
                "cameras": list(snap.preview_jpegs.keys()),
                "arms": snap.arms_by_role,
            }
        with self._lock:
            chunk_info = None if self.current_chunk is None else self.current_chunk.to_summary_dict()
            return {
                "mode": self.mode,
                "message": self.message,
                "last_error": self.last_error,
                "step_cursor": self.step_cursor,
                "total_steps": 0 if self.current_chunk is None else len(self.current_chunk.steps),
                "auto_cycles": self.auto_cycles,
                "async_stats": None if self.async_stats is None else dict(self.async_stats),
                "holding_arms": list(self._held_targets.keys()),
                "tcp_ik_ready": True,
                "settings": {
                    "server_url": self.config.server.url,
                    "instruction": self.config.server.instruction,
                    "execute_hz": self.config.execution.execute_hz,
                    "settle_delay_s": self.config.execution.settle_delay_s,
                    "hold_hz": self.config.execution.hold_hz,
                    "enable_gripper": self.config.execution.enable_gripper,
                },
                "calibration": self.calibration.summary(),
                "cameras": self.stream_hub.camera_status_list(),
                "arms": arms_out,
                "chunk": chunk_info,
                "latest_snapshot": snap_info,
                "logs": list(self._logs)[:25],
            }

    def close(self) -> None:
        self._shutdown.set()
        self.stop(release_mit=True)
        self._hold_thread.join(timeout=1.5)

