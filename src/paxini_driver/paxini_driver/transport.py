"""Hardware transports for PaXini PX6D sensors (RS485, USB, CAN, CANFD)."""

from __future__ import annotations

import select
import threading
import time
from typing import Any, Callable, List, Optional

from .protocol import (
    CMD_SET_FREQ,
    CMD_SET_PORT_PARAMS,
    DEFAULT_BAUDRATE,
    DEFAULT_DEVICE_ID,
    CanFrameParser,
    ProtocolEvent,
    SerialFrameParser,
    build_can_command,
    build_can_set_protocol,
    build_serial_frame,
    can_ack_id,
    can_data_ids,
    freq_to_code,
)


class SensorTransport:
    """Base transport interface for a single PX6D sensor."""

    interface_name: str = "abstract"

    @property
    def is_open(self) -> bool:
        raise NotImplementedError

    def open(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    def poll_events(self, timeout: float = 0.002) -> List[ProtocolEvent]:
        raise NotImplementedError

    def send_command(self, cmd: int, arg: Optional[int] = None) -> None:
        raise NotImplementedError


class SerialSensorTransport(SensorTransport):
    """RS485 / USB serial transport (921600 8N1, no flow control by default)."""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = DEFAULT_BAUDRATE,
        device_id: int = DEFAULT_DEVICE_ID,
        interface_name: str = "usb",
        serial_factory: Optional[Callable[..., Any]] = None,
    ) -> None:
        self.port = port
        self.baudrate = int(baudrate)
        self.device_id = int(device_id)
        self.interface_name = interface_name.lower()
        self._serial_factory = serial_factory
        self._serial: Any = None
        self._parser = SerialFrameParser(device_id=self.device_id)
        self._write_lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return bool(self._serial is not None and getattr(self._serial, "is_open", True))

    def open(self) -> None:
        if self.is_open:
            return
        if self._serial_factory is not None:
            self._serial = self._serial_factory(self.port, self.baudrate)
        else:
            import serial

            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.0,
                write_timeout=0.2,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
        self._parser.reset()

    def close(self) -> None:
        ser = self._serial
        self._serial = None
        self._parser.reset()
        if ser is not None:
            ser.close()

    def send_command(self, cmd: int, arg: Optional[int] = None) -> None:
        if cmd == CMD_SET_PORT_PARAMS:
            raise ValueError(
                "CAN/CANFD protocol conversion (0x11) requires CAN/CANFD interface"
            )
        if cmd == CMD_SET_FREQ and arg is not None:
            arg = freq_to_code(arg)
        frame = build_serial_frame(cmd, arg, self.device_id)
        if self.interface_name == "rs485":
            frame = b"\xff" + frame
        if not self.is_open or self._serial is None:
            raise OSError("Serial transport is not open")
        with self._write_lock:
            self._serial.write(frame)

    def poll_events(self, timeout: float = 0.002) -> List[ProtocolEvent]:
        if not self.is_open or self._serial is None:
            raise OSError("Serial transport is not open")
        if hasattr(self._serial, "in_waiting"):
            waiting = int(self._serial.in_waiting)
            if waiting == 0 and timeout > 0.0:
                if hasattr(self._serial, "fileno"):
                    select.select([self._serial], [], [], timeout)
                else:
                    time.sleep(min(timeout, 0.0001))
                waiting = int(self._serial.in_waiting)
            if waiting == 0:
                return []
            chunk = self._serial.read(waiting)
        else:
            chunk = self._serial.read(1)
        if not chunk:
            return []
        more = int(getattr(self._serial, "in_waiting", 0))
        if more > 0:
            chunk += self._serial.read(more)
        return self._parser.feed(chunk)


class CanSensorTransport(SensorTransport):
    """CAN 2.0 / CANFD transport via python-can (SocketCAN by default, 1 Mbps)."""

    def __init__(
        self,
        channel: str = "can0",
        can_interface: str = "socketcan",
        bitrate: int = 1_000_000,
        data_bitrate: int = 1_000_000,
        fd: bool = False,
        device_id: int = DEFAULT_DEVICE_ID,
        bus_factory: Optional[Callable[..., Any]] = None,
    ) -> None:
        self.channel = channel
        self.can_interface = can_interface
        self.bitrate = int(bitrate)
        self.data_bitrate = int(data_bitrate)
        self.fd = bool(fd)
        self.device_id = int(device_id)
        self.interface_name = "canfd" if self.fd else "can"
        self._bus_factory = bus_factory
        self._bus: Any = None
        self._parser = CanFrameParser(device_id=self.device_id)
        self._tx_lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return self._bus is not None

    def open(self) -> None:
        if self.is_open:
            return
        if self._bus_factory is not None:
            self._bus = self._bus_factory(self.channel, self.fd)
        else:
            import can

            rx_ids = [can_ack_id(self.device_id), *can_data_ids(self.device_id)]
            kwargs: dict[str, Any] = {
                "channel": self.channel,
                "interface": self.can_interface,
                "can_filters": [
                    {"can_id": cid, "can_mask": 0x7FF, "extended": False}
                    for cid in rx_ids
                ],
                "fd": self.fd,
            }
            if self.can_interface != "socketcan":
                kwargs["bitrate"] = self.bitrate
                if self.fd:
                    kwargs["data_bitrate"] = self.data_bitrate
            self._bus = can.Bus(**kwargs)
        self._parser.reset()

    def close(self) -> None:
        bus = self._bus
        self._bus = None
        self._parser.reset()
        if bus is not None and hasattr(bus, "shutdown"):
            bus.shutdown()

    def send_command(self, cmd: int, arg: Optional[int] = None) -> None:
        if self._bus is None:
            raise OSError("CAN transport is not open")
        import can

        if cmd == CMD_SET_PORT_PARAMS:
            can_id, payload = build_can_set_protocol(bool(arg), self.device_id)
        else:
            if cmd == CMD_SET_FREQ and arg is not None:
                arg = freq_to_code(arg)
            can_id, payload = build_can_command(cmd, arg, self.device_id)
        msg = can.Message(
            arbitration_id=can_id,
            data=payload,
            is_extended_id=False,
            is_fd=self.fd,
        )
        with self._tx_lock:
            self._bus.send(msg, timeout=0.1)

    def poll_events(self, timeout: float = 0.002) -> List[ProtocolEvent]:
        if self._bus is None:
            raise OSError("CAN transport is not open")
        first = self._bus.recv(timeout=timeout)
        if first is None:
            return []
        batch = [first]
        for _ in range(63):
            nxt = self._bus.recv(timeout=0.0)
            if nxt is None:
                break
            batch.append(nxt)
        events: List[ProtocolEvent] = []
        for msg in batch:
            if (
                getattr(msg, "is_error_frame", False)
                or getattr(msg, "is_remote_frame", False)
                or getattr(msg, "is_extended_id", False)
            ):
                continue
            ev = self._parser.push(int(msg.arbitration_id), bytes(msg.data))
            if ev is not None:
                events.append(ev)
        return events


def create_transport(
    interface: str,
    *,
    serial_port: str = "/dev/ttyUSB0",
    baudrate: int = DEFAULT_BAUDRATE,
    can_channel: str = "can0",
    can_interface: str = "socketcan",
    bitrate: int = 1_000_000,
    data_bitrate: int = 1_000_000,
    device_id: int = DEFAULT_DEVICE_ID,
) -> SensorTransport:
    iface = interface.strip().lower()
    if iface in ("usb", "rs485", "serial"):
        return SerialSensorTransport(
            port=serial_port,
            baudrate=baudrate,
            device_id=device_id,
            interface_name=iface,
        )
    if iface in ("can", "canfd"):
        return CanSensorTransport(
            channel=can_channel,
            can_interface=can_interface,
            bitrate=bitrate,
            data_bitrate=data_bitrate,
            fd=(iface == "canfd"),
            device_id=device_id,
        )
    raise ValueError(
        f"Unsupported interface '{interface}'; expected one of: usb, rs485, can, canfd"
    )
