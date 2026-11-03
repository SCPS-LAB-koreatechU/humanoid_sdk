"""Hand profile: CAN node and the 15 channels' open / close pulse widths.

Channel order is the CAN order of dexhand_control (profiles/*_hand.json):

     1 소지 위   2 소지 아래   3 약지 위   4 약지 아래   5 중지 위   6 중지 아래
     7 검지 위   8 검지 아래   9 검지 끝  10 중지 끝  11 약지 끝  12 소지 끝
    13 엄지 오른쪽 당김  14 엄지 왼쪽 당김  15 엄지 끝

A command is a pull fraction per channel, 0 = open_us, 1 = close_us:

    us = open_us + fraction * (close_us - open_us)

It is a fraction of the servo's calibrated stroke, not a finger joint angle.
The firmware accepts min(open, close)..max(open, close) per channel, so a
fraction clamped to 0..1 can never be rejected for range.

The firmware checks each channel alone. Tendons that pull against each other
(the thumb's left and right pull, 13 and 14) are checked here instead: for every
pair in `antagonist_pairs` the two fractions may add up to at most
`max_pair_sum`. Which other channels oppose each other is not documented yet.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .hand_can import CHANNELS

CHANNEL_NAMES = (
    "little_upper", "little_lower", "ring_upper", "ring_lower", "middle_upper", "middle_lower",
    "index_upper", "index_lower", "index_tip", "middle_tip", "ring_tip", "little_tip",
    "thumb_pull_right", "thumb_pull_left", "thumb_tip",
)


@dataclass
class HandSpec:
    name: str                       # left | right
    node: int
    channel: str                    # socketcan interface, the arm's can0
    open_us: list[int]
    close_us: list[int]
    protocol: int = 2               # 1: firmware sends no replies, 2: STATUS / PONG
    profile_tag: str = ""           # first 8 hex digits of the firmware profile SHA-256

    def __post_init__(self) -> None:
        if len(self.open_us) != CHANNELS or len(self.close_us) != CHANNELS:
            raise ValueError(f"{self.name}: open_us and close_us need {CHANNELS} values")
        if self.protocol not in (1, 2):
            raise ValueError(f"{self.name}: protocol must be 1 or 2")
        if not 1 <= self.node <= 127:
            raise ValueError(f"{self.name}: node must be 1..127")

    @property
    def low_us(self) -> list[int]:
        return [min(a, b) for a, b in zip(self.open_us, self.close_us)]

    @property
    def high_us(self) -> list[int]:
        return [max(a, b) for a, b in zip(self.open_us, self.close_us)]

    def to_us(self, fractions) -> list[int]:
        values = list(fractions)
        if len(values) != CHANNELS:
            raise ValueError(f"need {CHANNELS} fractions, got {len(values)}")
        return [round(o + min(max(float(f), 0.0), 1.0) * (c - o))
                for f, o, c in zip(values, self.open_us, self.close_us)]

    def to_fractions(self, pulses_us) -> list[float]:
        values = [int(p) for p in pulses_us]
        if len(values) != CHANNELS:
            raise ValueError(f"need {CHANNELS} pulses, got {len(values)}")
        bad = [i + 1 for i, (p, lo, hi) in enumerate(zip(values, self.low_us, self.high_us))
               if not lo <= p <= hi]
        if bad:
            raise ValueError(f"{self.name}: channels {bad} outside the calibrated range")
        return [0.0 if c == o else (p - o) / (c - o) for p, o, c in zip(values, self.open_us, self.close_us)]


@dataclass
class HandConfig:
    hands: dict[str, HandSpec]
    control_rate_hz: float = 50.0          # firmware watchdog is 1 s; send every cycle
    max_speed: float = 2.0                 # [fraction / s] per channel, 0.5 s open -> closed
    reply_timeout: float = 0.005           # [s] wait for the STATUS of the set just sent
    max_missed: int = 10                   # missing STATUS in a row -> LOST (0.2 s at 50 Hz)
    ping_timeout: float = 0.05
    estop_action: str = "hold"             # hold (keep the grip) | off (stop PWM)
    grips: dict[str, list[float]] = field(default_factory=dict)   # named poses, fractions
    antagonist_pairs: list[tuple[int, int]] = field(default_factory=lambda: [(13, 14)])  # 1-based
    max_pair_sum: float = 1.0

    def check_pose(self, fractions) -> str:
        """'' if the pose is allowed, else why not."""
        values = list(fractions)
        if len(values) != CHANNELS:
            return f"need {CHANNELS} fractions, got {len(values)}"
        if any(not 0.0 <= v <= 1.0 for v in values):
            return "fractions must be in 0..1"
        for a, b in self.antagonist_pairs:
            if values[a - 1] + values[b - 1] > self.max_pair_sum + 1e-9:
                return (f"channels {a} and {b} pull against each other: "
                        f"{values[a - 1]:.2f} + {values[b - 1]:.2f} > {self.max_pair_sum:g}")
        return ""

    def hand(self, name: str) -> HandSpec:
        if name not in self.hands:
            raise KeyError(f"unknown hand {name}; known: {sorted(self.hands)}")
        return self.hands[name]


def _grip(name: str, value) -> list[float]:
    values = [float(value)] * CHANNELS if isinstance(value, (int, float)) else [float(v) for v in value]
    if len(values) != CHANNELS or any(not 0.0 <= v <= 1.0 for v in values):
        raise ValueError(f"grip {name}: one fraction or {CHANNELS} fractions in 0..1")
    return values


def load_config(path: str | Path) -> HandConfig:
    data = yaml.safe_load(Path(path).read_text())
    defaults = data.get("defaults") or {}
    hands = {}
    for name, spec in (data.get("hands") or {}).items():
        merged = {**defaults, **(spec or {})}
        hands[name] = HandSpec(
            name=name, node=int(merged["node"]), channel=str(merged.get("channel", "can0")),
            open_us=[int(v) for v in merged["open_us"]], close_us=[int(v) for v in merged["close_us"]],
            protocol=int(merged.get("protocol", 2)), profile_tag=str(merged.get("profile_tag", "")))
    nodes = [h.node for h in hands.values()]
    if len(nodes) != len(set(nodes)):
        raise ValueError(f"two hands share a CAN node: {nodes}")
    control = data.get("control") or {}
    config = HandConfig(
        hands=hands,
        control_rate_hz=float(control.get("control_rate_hz", 50.0)),
        max_speed=float(control.get("max_speed", 2.0)),
        reply_timeout=float(control.get("reply_timeout", 0.005)),
        max_missed=int(control.get("max_missed", 10)),
        ping_timeout=float(control.get("ping_timeout", 0.05)),
        estop_action=str(control.get("estop_action", "hold")),
        grips={n: _grip(n, v) for n, v in (data.get("grips") or {}).items()},
        antagonist_pairs=[(int(a), int(b)) for a, b in control.get("antagonist_pairs", [[13, 14]])],
        max_pair_sum=float(control.get("max_pair_sum", 1.0)),
    )
    for a, b in config.antagonist_pairs:
        if not (1 <= a <= CHANNELS and 1 <= b <= CHANNELS and a != b):
            raise ValueError(f"antagonist pair ({a}, {b}): channels are 1..{CHANNELS}")
    if config.estop_action not in ("hold", "off"):
        raise ValueError("control.estop_action must be hold or off")
    if 1.0 / config.control_rate_hz >= 1.0:
        raise ValueError("control_rate_hz must be above 1 Hz (firmware watchdog is 1 s)")
    config.grips.setdefault("open", [0.0] * CHANNELS)
    for name, grip in config.grips.items():
        problem = config.check_pose(grip)
        if problem:
            raise ValueError(f"grip {name}: {problem}")
    return config
