"""DexHand CAN protocol (ESP32 per hand, 15 PWM servos), PC side.

Same id rule as the arm's Recoil motors: CAN id = (function << 7) | node.

    PC -> hand
      CONTROL  function 0   [mode, node]           OFF 0x00, START 0x01, HOLD 0x02, RESUME 0x13
      POSE     function 6   [seq, 0x10 | group, us0 lo, us0 hi, us1 lo, us1 hi, us2 lo, us2 hi]
                            five frames (groups 0..4) = 15 channels, all within 20 ms
      PING     function 4   [0xCA]                                         (protocol v2)
    hand -> PC (protocol v2 only; v1 firmware sends nothing but the CAN ACK)
      STATUS   function 5   [seq, result, flags, detail, applied lo, applied hi, tx_err, rx_err]
      PONG     function 3   [0xCA, version, node, flags, profile sha256[0:4]]

Left hand node 60 (0x03C / 0x33C / 0x23C -> 0x2BC / 0x1BC), right 61 (... 0x3D).
The arm uses nodes 1..10, so nothing collides.

APPLIED means the ESP32 wrote the 15 PWM duties. The ES3352 servos report
nothing, so no frame here ever says where a finger actually is.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass

FUNC_SHIFT = 7
NODE_MASK = 0x7F

F_CONTROL = 0
F_PONG = 3
F_PING = 4
F_STATUS = 5
F_POSE = 6

PING_MAGIC = 0xCA
POSE_VERSION = 1          # upper nibble of byte 1 of a pose frame
CHANNELS = 15
GROUPS = 5


class Mode:
    OFF = 0x00
    START = 0x01
    HOLD = 0x02
    RESUME = 0x13


class Result:
    APPLIED = 0x00
    REJECT_RANGE = 0x01
    REJECT_DEADLINE = 0x02
    REJECT_INHIBITED = 0x03
    REJECT_CONFLICT = 0x04
    REJECT_FORMAT = 0x05
    OUTPUT_FAIL = 0x06
    CONTROL_ACK = 0x10
    HEARTBEAT = 0x20

    NAMES = {0x00: "APPLIED", 0x01: "REJECT_RANGE", 0x02: "REJECT_DEADLINE",
             0x03: "REJECT_INHIBITED", 0x04: "REJECT_CONFLICT", 0x05: "REJECT_FORMAT",
             0x06: "OUTPUT_FAIL", 0x10: "CONTROL_ACK", 0x20: "HEARTBEAT"}


class Flag:
    PWM_ON = 1 << 0
    INHIBITED = 1 << 1
    WATCHDOG_EXPIRED = 1 << 2
    RX_OVERFLOW = 1 << 3
    CAN_RECOVERED = 1 << 4


def can_id(function: int, node: int) -> int:
    if not 0 <= node <= NODE_MASK:
        raise ValueError(f"node {node} out of range")
    return (function << FUNC_SHIFT) | node


def control_frame(node: int, mode: int) -> tuple[int, bytes]:
    return can_id(F_CONTROL, node), bytes([mode, node])


def pose_frames(node: int, sequence: int, pulses_us) -> list[tuple[int, bytes]]:
    pulses = [int(p) for p in pulses_us]
    if len(pulses) != CHANNELS:
        raise ValueError(f"need {CHANNELS} pulses, got {len(pulses)}")
    if any(not 0 <= p <= 0xFFFF for p in pulses):
        raise ValueError("pulse out of uint16 range")
    ident = can_id(F_POSE, node)
    return [(ident, struct.pack("<BB3H", sequence & 0xFF, (POSE_VERSION << 4) | g, *pulses[g * 3:g * 3 + 3]))
            for g in range(GROUPS)]


def ping_frame(node: int) -> tuple[int, bytes]:
    return can_id(F_PING, node), bytes([PING_MAGIC])


@dataclass(frozen=True)
class Status:
    node: int
    sequence: int
    result: int
    flags: int
    detail: int
    applied: int
    tx_errors: int
    rx_errors: int
    stamp: float

    @property
    def name(self) -> str:
        return Result.NAMES.get(self.result, f"0x{self.result:02X}")

    @property
    def pwm_on(self) -> bool:
        return bool(self.flags & Flag.PWM_ON)

    @property
    def inhibited(self) -> bool:
        return bool(self.flags & Flag.INHIBITED)

    def flag_names(self) -> list[str]:
        return [n for n, b in (("PWM_ON", Flag.PWM_ON), ("INHIBITED", Flag.INHIBITED),
                               ("WATCHDOG_EXPIRED", Flag.WATCHDOG_EXPIRED),
                               ("RX_OVERFLOW", Flag.RX_OVERFLOW), ("CAN_RECOVERED", Flag.CAN_RECOVERED))
                if self.flags & b]


@dataclass(frozen=True)
class Pong:
    node: int
    version: int
    flags: int
    profile_tag: str          # first 8 hex digits of the profile SHA-256
    stamp: float


def decode(arbitration_id: int, data: bytes, stamp: float | None = None) -> Status | Pong | None:
    """A hand -> PC frame, or None if it is something else."""
    stamp = time.monotonic() if stamp is None else stamp
    function, node = arbitration_id >> FUNC_SHIFT, arbitration_id & NODE_MASK
    if function == F_STATUS and len(data) >= 8:
        seq, result, flags, detail, applied, tx, rx = struct.unpack("<BBBBHBB", bytes(data[:8]))
        return Status(node, seq, result, flags, detail, applied, tx, rx, stamp)
    if function == F_PONG and len(data) >= 8 and data[0] == PING_MAGIC:
        return Pong(node, data[1], data[3], bytes(data[4:8]).hex(), stamp)
    return None


class HandBus:
    """One SocketCAN socket for one hand, separate from the arm's RecoilBus socket.

    Linux delivers every frame to every socket on can0, so the kernel filter keeps
    only this hand's STATUS / PONG here and the arm's 1000 frames/s never reach Python.
    """

    def __init__(self, channel: str, nodes, bitrate: int = 1_000_000, interface: str = "socketcan",
                 bus=None) -> None:
        import can

        self._can = can
        self.channel = channel
        filters = [{"can_id": can_id(f, n), "can_mask": 0x7FF, "extended": False}
                   for n in nodes for f in (F_STATUS, F_PONG)]
        if bus is not None:
            self._bus = bus
            self._bus.set_filters(filters)
        else:
            kwargs = {"bitrate": bitrate} if interface == "socketcan" else {}
            self._bus = can.Bus(interface=interface, channel=channel, can_filters=filters,
                                receive_own_messages=False, **kwargs)

    def send(self, ident: int, data: bytes, timeout: float = 0.002) -> bool:
        """False when the frame could not be queued (TX buffer full, bus off)."""
        try:
            self._bus.send(self._can.Message(arbitration_id=ident, data=data, is_extended_id=False),
                           timeout=timeout)
            return True
        except self._can.CanError:
            return False

    def recv(self, timeout: float) -> Status | Pong | None:
        """Next STATUS / PONG within `timeout`, or None."""
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            message = self._bus.recv(timeout=max(0.0, left))
            if message is None:
                return None
            if not message.is_error_frame and not message.is_extended_id:
                frame = decode(message.arbitration_id, message.data)
                if frame is not None:
                    return frame
            if left <= 0:
                return None

    def close(self) -> None:
        try:
            self._bus.shutdown()
        except Exception:  # noqa: BLE001 - a dead adapter must not raise on shutdown
            pass
