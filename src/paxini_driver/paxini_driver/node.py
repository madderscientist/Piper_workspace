"""ROS 2 driver node for PaXini PX6D 6-axis force/torque sensors."""

from __future__ import annotations

import json
import math
import signal
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Deque, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .calibration import (
    compensate_wrench,
)
from .configuration import resolve_sensor_configs
from .protocol import (
    CMD_CALIBRATE,
    CMD_GET_VERSION,
    CMD_READ_ONCE,
    CMD_SET_FREQ,
    CMD_SET_ID,
    CMD_SET_PORT_PARAMS,
    CMD_START_AUTO,
    CMD_STOP_AUTO,
    DEFAULT_BAUDRATE,
    DEFAULT_DEVICE_ID,
    ProtocolEvent,
    WrenchFilter,
    WrenchSample,
)
from .transport import SensorTransport, create_transport

JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))


class _RateTracker:
    def __init__(self, window_sec: float = 1.0) -> None:
        self.window_sec = window_sec
        self._stamps: Deque[float] = deque()

    def mark(self, now: float) -> None:
        self._stamps.append(now)
        cutoff = now - self.window_sec
        while self._stamps and self._stamps[0] < cutoff:
            self._stamps.popleft()

    def hz(self, now: float) -> float:
        cutoff = now - self.window_sec
        while self._stamps and self._stamps[0] < cutoff:
            self._stamps.popleft()
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        return (len(self._stamps) - 1) / span if span > 1e-6 else 0.0


class PaxiniDriverNode(Node):
    """ROS 2 node wrapping a PaXini PX6D sensor over RS485, USB, CAN, or CANFD."""

    def __init__(
        self,
        *,
        node_name: str = "paxini_driver",
        transport: Optional[SensorTransport] = None,
        start_worker: bool = True,
        **kwargs: Any,
    ) -> None:
        super().__init__(node_name, **kwargs)

        defaults = {
            "arm_id": "",
            "config_file": "",
            "interface": "usb",
            "serial_port": "/dev/ttyUSB0",
            "expected_version": "",
            "baudrate": DEFAULT_BAUDRATE,
            "can_channel": "can0",
            "can_interface": "socketcan",
            "can_bitrate": 1_000_000,
            "can_data_bitrate": 1_000_000,
            "device_id": DEFAULT_DEVICE_ID,
            "model": "PX6D-75A",
            "frame_id": "wrench_sensor",
            "wrench_topic": "wrench_stamped",
            "raw_wrench_topic": "wrench_raw_stamped",
            "status_topic": "sensor_status",
            "acquisition_mode": "default",
            "frequency_hz": 1000,
            "poll_rate_hz": 1000.0,
            "autostart": True,
            "calibrate_on_start": False,
            "reconnect_sec": 1.0,
            "stream_timeout_sec": 1.0,
            "require_hardware": False,
            "filter_mode": "none",
            "filter_window": 5,
            "filter_alpha": 0.2,
            "tare_offset": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "tool_mass_kg": 0.0,
            "tool_com_m": [0.0, 0.0, 0.0],
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        val = lambda k: self.get_parameter(k).value
        self.arm_id = str(val("arm_id")).strip()
        cfg_param = str(val("config_file")).strip()
        self.config_file: Optional[Path] = Path(cfg_param) if cfg_param else None
        if not self.arm_id:
            ns_clean = self.get_namespace().strip("/")
            if ns_clean in ("arm_3", "arm_4"):
                self.arm_id = ns_clean

        self.interface = str(val("interface")).strip().lower()
        self.expected_version = str(val("expected_version")).strip()
        self.device_id = int(val("device_id"))
        self.model = str(val("model")).strip().upper()
        self.frame_id = str(val("frame_id"))
        self.frequency_hz = int(val("frequency_hz"))
        self.poll_rate_hz = float(val("poll_rate_hz"))
        self.autostart = bool(val("autostart"))
        self.calibrate_on_start = bool(val("calibrate_on_start"))
        self.reconnect_sec = float(val("reconnect_sec"))
        self.stream_timeout_sec = float(val("stream_timeout_sec"))
        self.require_hardware = bool(val("require_hardware"))

        init_tare = [float(x) for x in (val("tare_offset") or [0.0] * 6)]
        if len(init_tare) != 6:
            init_tare = [0.0] * 6
        init_mass = float(val("tool_mass_kg") or 0.0)
        init_com = [float(x) for x in (val("tool_com_m") or [0.0, 0.0, 0.0])]
        if len(init_com) != 3:
            init_com = [0.0, 0.0, 0.0]
        init_quality: dict[str, Any] = {}

        if self.arm_id:
            try:
                cfg_entry = resolve_sensor_configs(
                    self.arm_id,
                    str(self.config_file) if self.config_file else None,
                )[0]
                init_quality = dict(cfg_entry.get("calibration_quality") or {})
                if init_tare == [0.0] * 6 and "tare_offset" in cfg_entry:
                    init_tare = [float(x) for x in cfg_entry["tare_offset"][:6]]
                if init_mass == 0.0 and "tool_mass_kg" in cfg_entry:
                    init_mass = float(cfg_entry["tool_mass_kg"])
                if init_com == [0.0, 0.0, 0.0] and "tool_com_m" in cfg_entry:
                    init_com = [float(x) for x in cfg_entry["tool_com_m"][:3]]
            except Exception:  # noqa: BLE001
                pass

        acq = str(val("acquisition_mode")).strip().lower()
        if acq == "default":
            self.acquisition_mode = "poll" if self.interface == "rs485" else "auto"
        elif acq in ("auto", "poll", "manual"):
            self.acquisition_mode = acq
        else:
            raise ValueError(
                f"Unsupported acquisition_mode '{acq}'; expected default, auto, poll, or manual"
            )

        self.filter = WrenchFilter(
            mode=str(val("filter_mode")),
            window=int(val("filter_window")),
            alpha=float(val("filter_alpha")),
        )
        self.transport: SensorTransport = transport or create_transport(
            self.interface,
            serial_port=str(val("serial_port")),
            baudrate=int(val("baudrate")),
            can_channel=str(val("can_channel")),
            can_interface=str(val("can_interface")),
            bitrate=int(val("can_bitrate")),
            data_bitrate=int(val("can_data_bitrate")),
            device_id=self.device_id,
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self._cb_group = ReentrantCallbackGroup()
        self._wrench_msg = WrenchStamped()
        self._wrench_msg.header.frame_id = self.frame_id
        self.wrench_pub = self.create_publisher(
            WrenchStamped, str(val("wrench_topic")), qos
        )
        self._raw_wrench_msg = WrenchStamped()
        self._raw_wrench_msg.header.frame_id = self.frame_id
        self.raw_wrench_pub = self.create_publisher(
            WrenchStamped, str(val("raw_wrench_topic")), qos
        )
        self.status_pub = self.create_publisher(String, str(val("status_topic")), 10)
        self.cmd_sub = self.create_subscription(
            String, "command", self._on_command_msg, 10, callback_group=self._cb_group
        )
        self.joint_sub = self.create_subscription(
            JointState,
            "joint_states_feedback",
            self._on_joints,
            10,
            callback_group=self._cb_group,
        )

        for srv_name, handler in (
            ("calibrate", self.calibrate_hardware),
            ("start_stream", self.start_stream),
            ("stop_stream", self.stop_stream),
            ("read_once", self.request_single_frame),
            ("get_version", self.request_version),
            ("reload_config", self.reload_config),
        ):
            self.create_service(
                Trigger,
                srv_name,
                self._make_trigger_cb(handler),
                callback_group=self._cb_group,
            )

        self._lock = threading.Lock()
        self.connected = False
        self.streaming = False
        self.firmware_version = ""
        self.last_error = ""
        self.latest_raw: Optional[WrenchSample] = None
        self.latest_wrench: Optional[WrenchSample] = None
        self.overload_warning = False
        self.tare_offset: List[float] = list(init_tare)
        self.tool_mass_kg: float = float(init_mass)
        self.tool_com_m: List[float] = list(init_com)
        self.calibration_quality = init_quality
        self.latest_joint_positions: Optional[Tuple[float, ...]] = None
        self.sample_count = 0
        self._rate_tracker = _RateTracker()
        self._last_connect_attempt = 0.0
        self._last_poll_sent = 0.0
        self._last_wrench_time = 0.0
        self._stream_started_time = 0.0
        self._communication_warning_active = False
        self._stream_gap_reported = False

        self._stop_event = threading.Event()
        self._worker_thread: Optional[threading.Thread] = None

        self._try_connect(now=time.monotonic(), initial=True)
        self._status_timer = self.create_timer(0.1, self._publish_status)

        if start_worker:
            self._worker_thread = threading.Thread(
                target=self._io_loop,
                name="paxini_px6d_io",
                daemon=True,
            )
            self._worker_thread.start()

    @staticmethod
    def _make_trigger_cb(fn):
        def _cb(_req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
            resp.success, resp.message = fn()
            return resp

        return _cb

    def _on_joints(self, msg: JointState) -> None:
        names = list(msg.name)
        if len(msg.position) < 6:
            return
        if names[:6] == list(JOINT_NAMES):
            order = list(range(6))
        else:
            idx_map = {n: i for i, n in enumerate(names)}
            if not all(jn in idx_map for jn in JOINT_NAMES):
                return
            order = [idx_map[jn] for jn in JOINT_NAMES]
        if any(i >= len(msg.position) for i in order):
            return
        pos6 = tuple(float(msg.position[i]) for i in order)
        if not all(math.isfinite(v) for v in pos6):
            return
        with self._lock:
            self.latest_joint_positions = pos6

    def _try_connect(
        self, now: float, initial: bool = False, force: bool = False
    ) -> bool:
        if self.transport.is_open:
            self.connected = True
            return True
        if not initial and not force:
            if (
                self.reconnect_sec <= 0.0
                or (now - self._last_connect_attempt) < self.reconnect_sec
            ):
                return False
        self._last_connect_attempt = now
        try:
            self.transport.open()
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self.connected = False
                self.streaming = False
                self.last_error = str(exc)
            if initial and self.require_hardware:
                raise
            self._warn_communication_once(f"PX6D 通信不可用: {exc}")
            return False

        with self._lock:
            self.connected = True
        try:
            self.transport.send_command(CMD_GET_VERSION)
            is_real_serial = (
                hasattr(self.transport, "_serial")
                and getattr(self.transport, "_serial_factory", None) is None
            )
            if is_real_serial:
                deadline = time.monotonic() + 0.02
                while time.monotonic() < deadline and not self.firmware_version:
                    for ev in self.transport.poll_events(timeout=0.002):
                        self._process_event(ev, now=time.monotonic())
                    if not self.transport.is_open:
                        return True
            if self.calibrate_on_start:
                self.transport.send_command(CMD_CALIBRATE)
            if self.autostart:
                self._send_stream_start()
            elif self.frequency_hz > 0:
                self.transport.send_command(CMD_SET_FREQ, self.frequency_hz)
            if is_real_serial:
                time.sleep(0.005)
                for ev in self.transport.poll_events(timeout=0.002):
                    self._process_event(ev, now=time.monotonic())
            self._last_poll_sent = time.monotonic()
        except Exception as exc:  # noqa: BLE001
            self._handle_disconnect(exc)
            return False
        return True

    def _send_stream_start(self) -> None:
        if self.acquisition_mode == "auto":
            self.transport.send_command(CMD_START_AUTO)
            if 0 < self.frequency_hz != 1000:
                self.transport.send_command(CMD_SET_FREQ, self.frequency_hz)
        elif self.acquisition_mode == "poll" and self.frequency_hz > 0:
            self.transport.send_command(CMD_SET_FREQ, self.frequency_hz)
        self.streaming = True
        self._stream_started_time = time.monotonic()

    def _warn_communication_once(self, message: str) -> None:
        with self._lock:
            report = not self._communication_warning_active
            self._communication_warning_active = True
        if report:
            self.get_logger().warning(message)

    def _check_stream_timeout(self, now: float) -> None:
        with self._lock:
            last = max(self._last_wrench_time, self._stream_started_time)
            report = (
                self.connected
                and self.streaming
                and self.stream_timeout_sec > 0.0
                and now - last >= self.stream_timeout_sec
                and not self._stream_gap_reported
            )
            if report:
                self._stream_gap_reported = True
                self.last_error = f"PX6D 数据断流: {now - last:.2f} 秒未收到有效六维力"
                message = self.last_error
        if report:
            self.get_logger().warning(message)

    def _handle_disconnect(self, exc: Exception) -> None:
        with self._lock:
            self.connected = False
            self.streaming = False
            self.last_error = str(exc)
        self._warn_communication_once(f"PX6D 通信中断: {exc}")
        try:
            self.transport.close()
        except Exception:  # noqa: BLE001
            pass

    def step(self, timeout: float = 0.002) -> int:
        """Run one I/O poll iteration; returns number of decoded protocol events."""
        now = time.monotonic()
        if not self.transport.is_open and not self._try_connect(now=now):
            return 0
        try:
            if (
                self.streaming
                and self.acquisition_mode == "poll"
                and self.poll_rate_hz > 0.0
            ):
                period = 1.0 / self.poll_rate_hz
                if (now - self._last_poll_sent) >= period:
                    self.transport.send_command(CMD_READ_ONCE)
                    if (
                        self._last_poll_sent <= 0.0
                        or (now - self._last_poll_sent) > 2.0 * period
                    ):
                        self._last_poll_sent = now
                    else:
                        self._last_poll_sent += period
            events = self.transport.poll_events(timeout=timeout)
        except Exception as exc:  # noqa: BLE001
            self._handle_disconnect(exc)
            return 0

        for ev in events:
            self._process_event(ev, now=time.monotonic())
        return len(events)

    def _io_loop(self) -> None:
        while not self._stop_event.is_set():
            if not self.transport.is_open:
                self._try_connect(now=time.monotonic())
                self._stop_event.wait(0.05)
                continue
            self.step(timeout=0.002)

    def _process_event(self, ev: ProtocolEvent, now: float) -> None:
        wrench_to_pub: Optional[WrenchSample] = None
        raw_to_pub: Optional[WrenchSample] = None
        version_mismatch_port: Optional[str] = None
        with self._lock:
            if ev.version:
                self.firmware_version = ev.version
                if (
                    self.expected_version
                    and ev.version != self.expected_version
                    and hasattr(self.transport, "port")
                ):
                    cur_port = str(getattr(self.transport, "port", ""))
                    candidates = sorted(
                        str(p)
                        for p in Path("/dev").glob("ttyUSB*")
                        if str(p) != cur_port
                    )
                    if candidates:
                        version_mismatch_port = candidates[0]
            if ev.wrench is not None and ev.wrench.is_finite():
                raw = ev.wrench
                self.latest_raw = raw
                self._last_wrench_time = now
                self._communication_warning_active = False
                self._stream_gap_reported = False
                self.last_error = ""
                raw_to_pub = raw
                compensated = compensate_wrench(
                    raw,
                    self.tare_offset,
                    self.latest_joint_positions,
                    self.tool_mass_kg,
                    self.tool_com_m,
                )
                filtered = self.filter.apply(compensated)
                self.latest_wrench = filtered
                self.overload_warning = raw.exceeds_overload_alarm(self.model)
                self.sample_count += 1
                self._rate_tracker.mark(now)
                wrench_to_pub = filtered
        if version_mismatch_port is not None:
            self.get_logger().warning(
                f"Port {getattr(self.transport, 'port', '')} reported firmware {self.firmware_version} "
                f"(expected {self.expected_version}); switching to {version_mismatch_port}"
            )
            try:
                self.transport.close()
            except Exception:  # noqa: BLE001
                pass
            setattr(self.transport, "port", version_mismatch_port)
            with self._lock:
                self.connected = False
                self.streaming = False
                self.firmware_version = ""
            self._try_connect(now=time.monotonic(), force=True)
            return
        if raw_to_pub is not None:
            self._publish_wrench(raw_to_pub, raw=True)
        if wrench_to_pub is not None:
            self._publish_wrench(wrench_to_pub)

    def _publish_wrench(self, sample: WrenchSample, *, raw: bool = False) -> None:
        msg = self._raw_wrench_msg if raw else self._wrench_msg
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.wrench.force.x = float(sample.fx)
        msg.wrench.force.y = float(sample.fy)
        msg.wrench.force.z = float(sample.fz)
        msg.wrench.torque.x = float(sample.mx)
        msg.wrench.torque.y = float(sample.my)
        msg.wrench.torque.z = float(sample.mz)
        (self.raw_wrench_pub if raw else self.wrench_pub).publish(msg)

    def status_dict(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            wrench = (
                self.latest_wrench.as_tuple()
                if self.latest_wrench is not None
                else None
            )
            return {
                "connected": self.connected and self.transport.is_open,
                "interface": self.interface,
                "device_id": self.device_id,
                "model": self.model,
                "frame_id": self.frame_id,
                "acquisition_mode": self.acquisition_mode,
                "streaming": self.streaming,
                "firmware_version": self.firmware_version,
                "rx_hz": round(self._rate_tracker.hz(now), 2),
                "sample_count": self.sample_count,
                "overload_warning": self.overload_warning,
                "filter_mode": self.filter.mode,
                "tare_offset": [round(v, 6) for v in self.tare_offset],
                "tool_mass_kg": round(self.tool_mass_kg, 6),
                "tool_com_m": [round(v, 6) for v in self.tool_com_m],
                "calibration_quality": self.calibration_quality,
                "latest_wrench": [round(v, 6) for v in wrench]
                if wrench is not None
                else None,
                "last_error": self.last_error,
                "stream_gap": self._stream_gap_reported,
            }

    def _publish_status(self) -> None:
        self._check_stream_timeout(time.monotonic())
        msg = String()
        msg.data = json.dumps(self.status_dict(), ensure_ascii=False)
        self.status_pub.publish(msg)

    def _exec_cmd(
        self, cmd: int, arg: Optional[int] = None, desc: str = ""
    ) -> tuple[bool, str]:
        if not self.transport.is_open and not self._try_connect(
            now=time.monotonic(), force=True
        ):
            return False, self.last_error or "Sensor hardware is not connected"
        try:
            self.transport.send_command(cmd, arg)
            return True, desc
        except ValueError as exc:
            return False, str(exc)
        except Exception as exc:  # noqa: BLE001
            self._handle_disconnect(exc)
            return False, str(exc)

    def calibrate_hardware(self) -> tuple[bool, str]:
        ok, msg = self._exec_cmd(CMD_CALIBRATE, desc="Hardware calibration (0x10) sent")
        if ok:
            with self._lock:
                self.filter.reset()
        return ok, msg

    def reload_config(self) -> tuple[bool, str]:
        """Atomically load calibration parameters produced by the independent client."""
        try:
            entry = resolve_sensor_configs(
                self.arm_id, str(self.config_file) if self.config_file else None
            )[0]
            bias = [float(v) for v in entry.get("tare_offset", [0.0] * 6)]
            mass = float(entry.get("tool_mass_kg", 0.0))
            com = [float(v) for v in entry.get("tool_com_m", [0.0] * 3)]
            if (
                len(bias) != 6
                or len(com) != 3
                or not all(math.isfinite(v) for v in bias + com + [mass])
                or mass < 0.0
            ):
                raise ValueError("配置中的标定参数格式无效")
            with self._lock:
                self.tare_offset = bias
                self.tool_mass_kg = mass
                self.tool_com_m = com
                self.calibration_quality = dict(entry.get("calibration_quality") or {})
                self.filter.reset()
                # The I/O loop publishes the next fresh sample with the new parameters.
                # Do not replay an old sample from a service thread with a new timestamp.
                self.latest_wrench = None
            return True, f"{self.arm_id} 已重载标定参数"
        except Exception as exc:
            return False, str(exc)

    def start_stream(self) -> tuple[bool, str]:
        if not self.transport.is_open and not self._try_connect(
            now=time.monotonic(), force=True
        ):
            return False, self.last_error or "Sensor hardware is not connected"
        if self.acquisition_mode == "poll":
            with self._lock:
                self.streaming = True
                self._stream_started_time = time.monotonic()
            return True, f"Polling started at {self.poll_rate_hz:.1f} Hz (0x05)"
        try:
            self._send_stream_start()
            rate_info = (
                f"{self.frequency_hz} Hz (0x03+0x02)"
                if 0 < self.frequency_hz != 1000
                else "1000 Hz (0x03)"
            )
            return True, f"Automatic transmission started at {rate_info}"
        except Exception as exc:  # noqa: BLE001
            self._handle_disconnect(exc)
            return False, str(exc)

    def stop_stream(self) -> tuple[bool, str]:
        with self._lock:
            self.streaming = False
        if not self.transport.is_open:
            return True, "Stream stopped (hardware offline)"
        return self._exec_cmd(
            CMD_STOP_AUTO, desc="Automatic transmission stopped (0x04)"
        )

    def request_single_frame(self) -> tuple[bool, str]:
        ok, msg = self._exec_cmd(CMD_READ_ONCE, desc="Single-frame request (0x05) sent")
        if ok and self._worker_thread is None:
            self.step(timeout=0.01)
        return ok, msg

    def request_version(self) -> tuple[bool, str]:
        ok, msg = self._exec_cmd(CMD_GET_VERSION, desc="Version query (0x07) sent")
        if ok and self._worker_thread is None:
            self.step(timeout=0.01)
        with self._lock:
            return ok, (self.firmware_version or msg) if ok else msg

    def _on_command_msg(self, msg: String) -> None:
        raw = msg.data.strip()
        cmd = raw.lower()
        actions = {
            "calibrate": self.calibrate_hardware,
            "start": self.start_stream,
            "stop": self.stop_stream,
            "once": self.request_single_frame,
            "read_once": self.request_single_frame,
            "version": self.request_version,
            "reload_config": self.reload_config,
        }
        if cmd in actions:
            ok, info = actions[cmd]()
        elif cmd.startswith("set_freq:"):
            try:
                hz = int(cmd.split(":", 1)[1], 0)
                ok, info = self._exec_cmd(
                    CMD_SET_FREQ, hz, f"Frequency set to {hz} Hz (0x02)"
                )
                if ok:
                    self.frequency_hz = hz
            except ValueError as exc:
                ok, info = False, str(exc)
        elif cmd.startswith("set_id:"):
            try:
                new_id = int(cmd.split(":", 1)[1], 0)
                ok, info = self._exec_cmd(
                    CMD_SET_ID, new_id, f"Set ID 0x{new_id:02X} (0x01)"
                )
            except ValueError as exc:
                ok, info = False, str(exc)
        elif cmd in ("set_mode:can", "set_mode:canfd"):
            canfd = int(cmd.endswith("canfd"))
            ok, info = self._exec_cmd(
                CMD_SET_PORT_PARAMS, canfd, f"Set protocol ({cmd})"
            )
        else:
            self.get_logger().warning(f"Unknown command '{raw}'")
            return
        if not ok:
            self.get_logger().warning(f"Command '{raw}' failed: {info}")

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._worker_thread is not None and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=1.0)
        if self.transport.is_open:
            try:
                if self.streaming:
                    self.streaming = False
                    self.transport.send_command(CMD_STOP_AUTO)
            except Exception:  # noqa: BLE001
                pass
            try:
                self.transport.close()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: Optional[PaxiniDriverNode] = None
    try:
        node = PaxiniDriverNode()
    except Exception as exc:  # noqa: BLE001
        get_logger("paxini_driver").fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        return

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    stopped = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())

    spin_thread = threading.Thread(
        target=executor.spin, name="paxini_executor", daemon=True
    )
    spin_thread.start()
    try:
        stopped.wait()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
