"""Commanded pull fractions -> estimated DexHand V2 joint angles, for RViz only.

This is a picture of the command, never a measurement: the ES3352 servos report
nothing, and a tendon hand's joints also depend on load and contact. The map is
linear and lives in config/dexhand.yaml (`visualization.joints`):

    joint_rad = clamp( sum_k coef[k] * fraction[k],  lower, upper )

The default map is a hypothesis to be checked on the hand: each finger's
"upper" and "lower" tendons drive the knuckle as a differential pair (both
pulled -> Pitch, one more than the other -> Yaw), the "tip" tendon drives
Flexor (the URDF's DIP joint mimics Flexor), and the thumb's right / left pull
are a differential pair for Thumb_Pitch / Thumb_Yaw. Coefficients default to the
dexhandv2_right.urdf joint limits, so a full pull reaches the URDF limit.

Joint names are the URDF's without the side prefix (R_ / L_). The robot URDFs
(v1arm, dexhand) only move <side>_<Finger>_Pitch and _Yaw; the standalone V2
hand URDF moves every joint here.
"""

from __future__ import annotations

from dataclasses import dataclass

from .hand_can import CHANNELS


@dataclass(frozen=True)
class JointMap:
    name: str
    coef: dict[int, float]       # 1-based channel -> rad per unit fraction
    lower: float
    upper: float

    def angle(self, fractions) -> float:
        value = sum(c * float(fractions[k - 1]) for k, c in self.coef.items())
        return min(max(value, self.lower), self.upper)


PREFIX = {"left": "L_", "right": "R_"}


def load_maps(data: dict | None) -> list[JointMap]:
    maps = []
    for name, spec in ((data or {}).get("joints") or {}).items():
        coef = {int(k): float(v) for k, v in (spec.get("coef") or {}).items()}
        if any(not 1 <= k <= CHANNELS for k in coef):
            raise ValueError(f"visualization joint {name}: channels are 1..{CHANNELS}")
        lower, upper = (float(v) for v in spec["limits"])
        if lower > upper:
            raise ValueError(f"visualization joint {name}: lower > upper")
        maps.append(JointMap(name, coef, lower, upper))
    return maps


def estimate(fractions, maps: list[JointMap], side: str) -> dict[str, float]:
    """{'R_Index_Pitch': rad, ...} for one hand."""
    values = list(fractions)
    if len(values) != CHANNELS:
        raise ValueError(f"need {CHANNELS} fractions, got {len(values)}")
    prefix = PREFIX[side]
    return {prefix + m.name: m.angle(values) for m in maps}
