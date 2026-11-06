"""Official PaXini PX6D communication protocol (RS485, USB, CANFD, CAN).

Reference:
  PaXini PX6D 6 Axis Force/Torque Sensor Instruction Manual, Section 1.2, 4 & 5.
"""

from __future__ import annotations

import math
import struct
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Sequence, Tuple, Union

SERIAL_HEADER = b"\xaa\x55"
DEFAULT_DEVICE_ID = 0x7F
DEFAULT_BAUDRATE = 921600

CMD_SET_ID = 0x01
CMD_SET_FREQ = 0x02
CMD_START_AUTO = 0x03
CMD_STOP_AUTO = 0x04
CMD_READ_ONCE = 0x05
CMD_GET_VERSION = 0x07
CMD_CALIBRATE = 0x10
CMD_SET_PORT_PARAMS = 0x11

# Published full-scale ranges (force_n, torque_nm) from Manual Section 1.2.
MODEL_RANGES = {
    "PX6D-75A": (50.0, 2.0),
    "PX6D-75B": (100.0, 4.0),
    "PX6D-75C": (200.0, 8.0),
    "PX6D-75E": (500.0, 20.0),
}

# Default 1-byte argument per Manual 5.1.3 / 5.2.2 (Serial) and 5.3.2 / 5.4.2 (CAN).
SERIAL_DEFAULT_ARG = {
    CMD_START_AUTO: 0x04,
    CMD_STOP_AUTO: 0x00,
    CMD_READ_ONCE: 0x01,
    CMD_GET_VERSION: 0x01,
    CMD_CALIBRATE: 0x01,
}
CAN_DEFAULT_ARG = {
    CMD_START_AUTO: 0x00,
    CMD_STOP_AUTO: 0x00,
    CMD_READ_ONCE: 0x00,
    CMD_GET_VERSION: 0x00,
    CMD_CALIBRATE: 0x01,
}

_FLOAT6_LE = struct.Struct("<6f")
_FLOAT2_LE = struct.Struct("<2f")


def _make_crc8_table() -> Tuple[int, ...]:
    table = []
    for byte in range(256):
        crc = byte
        for _ in range(8):
            crc = (((crc << 1) ^ 0x07) & 0xFF) if (crc & 0x80) else ((crc << 1) & 0xFF)
        table.append(crc)
    return tuple(table)


_CRC8_TABLE = _make_crc8_table()


def crc8(data: Union[bytes, bytearray, memoryview]) -> int:
    """CRC-8 (poly=0x07, init=0x00, refin=False, refout=False, xorout=0x00)."""
    crc = 0
    table = _CRC8_TABLE
    for byte in data:
        crc = table[crc ^ byte]
    return crc


def _check_u8(value: int, name: str) -> int:
    ivalue = int(value)
    if not 0 <= ivalue <= 0xFF:
        raise ValueError(f"{name} must fit in 1 byte (0..255), got {value}")
    return ivalue


def freq_to_code(freq_hz: int) -> int:
    """Convert transmission frequency (Hz) to protocol code (Freq / 4)."""
    if freq_hz < 4 or freq_hz % 4 != 0:
        raise ValueError(f"Frequency {freq_hz} Hz must be a positive multiple of 4 Hz")
    return _check_u8(freq_hz // 4, "freq_hz // 4")


@dataclass(frozen=True)
class WrenchSample:
    """Six-axis force (N) and torque (N*m) sample in the sensor measurement frame."""

    fx: float
    fy: float
    fz: float
    mx: float
    my: float
    mz: float

    def as_tuple(self) -> Tuple[float, float, float, float, float, float]:
        return (self.fx, self.fy, self.fz, self.mx, self.my, self.mz)

    def is_finite(self) -> bool:
        return all(math.isfinite(v) for v in self.as_tuple())

    def minus(self, offset: Sequence[float]) -> "WrenchSample":
        return WrenchSample(
            self.fx - offset[0],
            self.fy - offset[1],
            self.fz - offset[2],
            self.mx - offset[3],
            self.my - offset[4],
            self.mz - offset[5],
        )

    def exceeds_overload_alarm(
        self, model: str = "PX6D-75A", ratio: float = 2.0
    ) -> bool:
        """Manual Section 4.2: alarm reminder when any axis exceeds 200% of range."""
        bounds = MODEL_RANGES.get(model.upper())
        if bounds is None:
            return False
        f_max, m_max = bounds[0] * ratio, bounds[1] * ratio
        return (
            abs(self.fx) > f_max
            or abs(self.fy) > f_max
            or abs(self.fz) > f_max
            or abs(self.mx) > m_max
            or abs(self.my) > m_max
            or abs(self.mz) > m_max
        )

    @classmethod
    def from_bytes(cls, payload: Union[bytes, bytearray, memoryview]) -> "WrenchSample":
        if len(payload) != 24:
            raise ValueError(
                f"Expected 24-byte wrench payload, got {len(payload)} bytes"
            )
        return cls(*_FLOAT6_LE.unpack(payload))

    def to_bytes(self) -> bytes:
        return _FLOAT6_LE.pack(self.fx, self.fy, self.fz, self.mx, self.my, self.mz)


@dataclass(frozen=True)
class ProtocolEvent:
    """Decoded event from either RS485/USB serial or CAN/CANFD."""

    cmd: Optional[int] = None
    wrench: Optional[WrenchSample] = None
    version: Optional[str] = None
    ack_cmd: Optional[int] = None
    status: int = 0
    payload: bytes = b""


def decode_version_bytes(payload: Union[bytes, bytearray, memoryview]) -> str:
    """Decode firmware version from 8-byte serial payload or 6-byte CAN payload."""
    raw = bytes(payload)
    if len(raw) == 8 and raw[0] == 0 and raw[-1] == 0:
        raw = raw[1:7]
    return raw.strip(b"\x00 ").decode("ascii", errors="replace")


def build_serial_frame(
    cmd: int,
    data: Union[int, bytes, bytearray, Sequence[int], None] = None,
    device_id: int = DEFAULT_DEVICE_ID,
) -> bytes:
    """Build a framing packet for RS485 or USB (0xAA 0x55 | ID | CMD | DATA | CRC8)."""
    dev = _check_u8(device_id, "device_id")
    command = _check_u8(cmd, "cmd")
    if data is None:
        data = SERIAL_DEFAULT_ARG[command]
    payload = bytes([_check_u8(data, "data")]) if isinstance(data, int) else bytes(data)
    body = SERIAL_HEADER + bytes([dev, command]) + payload
    return body + bytes([crc8(body)])


_CANDIDATE_PAYLOAD_LENGTHS = {
    CMD_SET_ID: (8,),
    CMD_SET_FREQ: (1, 8),
    CMD_START_AUTO: (24, 8),
    CMD_STOP_AUTO: (8,),
    CMD_READ_ONCE: (24,),
    CMD_GET_VERSION: (8,),
    CMD_CALIBRATE: (8,),
}


class SerialFrameParser:
    """Stateful byte-stream parser for RS485 and USB frames."""

    def __init__(self, device_id: Optional[int] = DEFAULT_DEVICE_ID) -> None:
        self.device_id = (
            None if device_id is None else _check_u8(device_id, "device_id")
        )
        self._buf = bytearray()

    def reset(self) -> None:
        self._buf.clear()

    def feed(self, chunk: Union[bytes, bytearray, memoryview]) -> List[ProtocolEvent]:
        if chunk:
            self._buf.extend(chunk)
        events: List[ProtocolEvent] = []
        while True:
            idx = self._buf.find(SERIAL_HEADER)
            if idx < 0:
                if len(self._buf) > 1 and self._buf[-1] == SERIAL_HEADER[0]:
                    del self._buf[:-1]
                else:
                    self._buf.clear()
                break
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < 6:
                break
            dev_id, cmd = self._buf[2], self._buf[3]
            if self.device_id is not None and dev_id != self.device_id:
                del self._buf[:2]
                continue
            candidate_lengths = _CANDIDATE_PAYLOAD_LENGTHS.get(cmd)
            if candidate_lengths is None:
                del self._buf[:2]
                continue
            matched_frame_len: Optional[int] = None
            need_more_bytes = False
            for payload_len in candidate_lengths:
                frame_len = 5 + payload_len
                if len(self._buf) < frame_len:
                    need_more_bytes = True
                    continue
                if (
                    crc8(memoryview(self._buf)[: frame_len - 1])
                    == self._buf[frame_len - 1]
                ):
                    matched_frame_len = frame_len
                    break
            if matched_frame_len is None:
                if need_more_bytes:
                    break
                del self._buf[:2]
                continue
            payload = bytes(self._buf[4 : matched_frame_len - 1])
            del self._buf[:matched_frame_len]
            if cmd in (CMD_START_AUTO, CMD_READ_ONCE) and len(payload) == 24:
                events.append(
                    ProtocolEvent(
                        cmd=cmd,
                        wrench=WrenchSample.from_bytes(payload),
                        payload=payload,
                    )
                )
            elif cmd == CMD_GET_VERSION and len(payload) == 8:
                events.append(
                    ProtocolEvent(
                        cmd=cmd,
                        version=decode_version_bytes(payload),
                        ack_cmd=cmd,
                        payload=payload,
                    )
                )
            else:
                events.append(ProtocolEvent(cmd=cmd, ack_cmd=cmd, payload=payload))
        return events


def can_tx_id(device_id: int = DEFAULT_DEVICE_ID) -> int:
    return 0x200 + _check_u8(device_id, "device_id")


def can_ack_id(device_id: int = DEFAULT_DEVICE_ID) -> int:
    return 0x180 + _check_u8(device_id, "device_id")


def can_data_ids(device_id: int = DEFAULT_DEVICE_ID) -> Tuple[int, int, int]:
    dev = _check_u8(device_id, "device_id")
    return (0x280 + dev, 0x380 + dev, 0x480 + dev)


def build_can_command(
    cmd: int,
    arg: Optional[int] = None,
    device_id: int = DEFAULT_DEVICE_ID,
    pad_to_8: bool = True,
) -> Tuple[int, bytes]:
    """Build a CAN / CANFD command frame on 0x200 + Device_id."""
    command = _check_u8(cmd, "cmd")
    if arg is None:
        arg = CAN_DEFAULT_ARG[command]
    head = bytes([command, _check_u8(arg, "arg")])
    payload = head.ljust(8, b"\x00") if pad_to_8 else head
    return can_tx_id(device_id), payload


def build_can_set_protocol(
    canfd: bool, device_id: int = DEFAULT_DEVICE_ID
) -> Tuple[int, bytes]:
    """Manual Section 5.5: switch between CANFD (1M) and CAN 2.0 (1M)."""
    mode_byte = 0x01 if canfd else 0x00
    payload = bytes(
        [CMD_SET_PORT_PARAMS, mode_byte, 0x00, 0x01, 0x03, mode_byte, 0x00, 0x00]
    )
    return can_tx_id(device_id), payload


class CanFrameParser:
    """Parses both CANFD (24-byte single frame) and CAN 2.0 (3x8-byte frames)."""

    def __init__(self, device_id: int = DEFAULT_DEVICE_ID) -> None:
        self.device_id = _check_u8(device_id, "device_id")
        self.ack_id = can_ack_id(self.device_id)
        self.data_ids = can_data_ids(self.device_id)
        self._stage = 0
        self._fx = 0.0
        self._fy = 0.0
        self._fz = 0.0
        self._mx = 0.0

    def reset(self) -> None:
        self._stage = 0

    def push(
        self, can_id: int, data: Union[bytes, bytearray, memoryview]
    ) -> Optional[ProtocolEvent]:
        raw = bytes(data)
        if can_id == self.ack_id:
            if len(raw) < 2:
                return None
            status, cmd = raw[0], raw[1]
            version = (
                decode_version_bytes(raw[2:8])
                if (cmd == CMD_GET_VERSION and len(raw) >= 8)
                else None
            )
            return ProtocolEvent(
                cmd=cmd, ack_cmd=cmd, status=status, version=version, payload=raw
            )

        id1, id2, id3 = self.data_ids
        if can_id == id1 and len(raw) >= 24:
            self._stage = 0
            return ProtocolEvent(
                wrench=WrenchSample.from_bytes(raw[:24]), payload=raw[:24]
            )
        if can_id == id1 and len(raw) >= 8:
            self._fx, self._fy = _FLOAT2_LE.unpack(raw[:8])
            self._stage = 1
            return None
        if can_id == id2 and len(raw) >= 8:
            if self._stage != 1:
                self._stage = 0
                return None
            self._fz, self._mx = _FLOAT2_LE.unpack(raw[:8])
            self._stage = 2
            return None
        if can_id == id3 and len(raw) >= 8:
            if self._stage != 2:
                self._stage = 0
                return None
            my, mz = _FLOAT2_LE.unpack(raw[:8])
            self._stage = 0
            return ProtocolEvent(
                wrench=WrenchSample(self._fx, self._fy, self._fz, self._mx, my, mz),
                payload=raw[:8],
            )
        return None


class WrenchFilter:
    """Optional wrench filter matching Manual Section 4.3 ('none', 'moving_average', 'low_pass')."""

    def __init__(self, mode: str = "none", window: int = 5, alpha: float = 0.2) -> None:
        mode_norm = mode.strip().lower()
        if mode_norm not in ("none", "moving_average", "low_pass"):
            raise ValueError(
                f"Unsupported filter_mode '{mode}'; expected none, moving_average, or low_pass"
            )
        if window < 1:
            raise ValueError(f"filter_window must be >= 1, got {window}")
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f"filter_alpha must be in (0, 1], got {alpha}")
        self.mode = mode_norm
        self.window = int(window)
        self.alpha = float(alpha)
        self._history: Deque[Tuple[float, float, float, float, float, float]] = deque()
        self._sums: List[float] = [0.0] * 6
        self._state: Optional[List[float]] = None

    def reset(self) -> None:
        self._history.clear()
        self._sums = [0.0] * 6
        self._state = None

    def apply(self, sample: WrenchSample) -> WrenchSample:
        if self.mode == "none":
            return sample
        vals = sample.as_tuple()
        if self.mode == "moving_average":
            if len(self._history) == self.window:
                oldest = self._history.popleft()
                for i in range(6):
                    self._sums[i] -= oldest[i]
            self._history.append(vals)
            for i in range(6):
                self._sums[i] += vals[i]
            inv_n = 1.0 / len(self._history)
            return WrenchSample(*(s * inv_n for s in self._sums))
        if self._state is None:
            self._state = list(vals)
            return sample
        a = self.alpha
        for i in range(6):
            self._state[i] = a * vals[i] + (1.0 - a) * self._state[i]
        return WrenchSample(*self._state)
