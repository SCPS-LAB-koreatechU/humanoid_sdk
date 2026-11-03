"""Load the vendored DexHand V2 hand URDF (Berkeley-Humanoid-Lite ros2_ws/vendor/
dexhandv2_description) for the standalone hand view, with two fixes:

  * package://dexhandv2_description/... -> file://<vendor dir>/... when that
    directory is not a ROS package (it is vendored as plain files, CC BY-NC-SA,
    and not installed), so RViz finds the meshes;
  * the upstream mimic tags name their master without the side prefix
    (<mimic joint="Index_Flexor"/> for R_Index_DIP), so every tool drops them and
    the DIP joints never move; they are renamed to the joint that exists.

The meshes themselves stay in the description workspace: nothing is copied.
"""

from __future__ import annotations

import os
import re
from pathlib import Path

VENDOR = Path("vendor") / "dexhandv2_description"
URDF = Path("urdf") / "dexhandv2_right.urdf"


def default_path() -> Path:
    workspace = Path(os.environ.get("BHL_DESCRIPTION_WS", Path.home() / "Berkeley-Humanoid-Lite" / "ros2_ws"))
    return workspace / VENDOR / URDF


def fix_mimics(text: str) -> str:
    joints = set(re.findall(r'<joint\s+name="([^"]+)"', text))

    def rename(match: re.Match) -> str:
        name = match.group(1)
        if name in joints:
            return match.group(0)
        for prefix in ("R_", "L_"):
            if prefix + name in joints:
                return match.group(0).replace(f'"{name}"', f'"{prefix}{name}"')
        return match.group(0)

    return re.sub(r'<mimic\s+joint="([^"]+)"', rename, text)


def load(path: str | Path | None = None, package_found: bool = False) -> str:
    """URDF text ready for robot_state_publisher."""
    path = Path(path) if path else default_path()
    if not path.is_file():
        raise FileNotFoundError(
            f"no DexHand V2 URDF at {path}. Set BHL_DESCRIPTION_WS to the Berkeley-Humanoid-Lite "
            "ros2_ws (it has vendor/dexhandv2_description) or pass hand_urdf:=<file>")
    text = path.read_text()
    if not package_found:
        root = path.resolve().parents[1]           # .../dexhandv2_description
        text = text.replace("package://dexhandv2_description/", f"file://{root}/")
    return fix_mimics(text)
