"""Pull fractions -> estimated V2 joint angles (RViz only), and the V2 URDF loader."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from bhl_hand_sdk.config import load_config
from bhl_hand_sdk.estimate import estimate, load_maps
from bhl_hand_sdk.v2_urdf import fix_mimics, load

CONFIG = Path(__file__).resolve().parents[1] / "config" / "dexhand.yaml"
# Actuated joints of dexhandv2_right.urdf (iotdesignshop/dexhandv2_description e379b6ec),
# the DIP joints mimic Flexor and are drawn by robot_state_publisher.
V2_RIGHT = {f"R_{f}_{j}" for f in ("Index", "Middle", "Ring", "Pinky") for j in ("Pitch", "Yaw", "Flexor")} | \
           {"R_Thumb_Yaw", "R_Thumb_Roll", "R_Thumb_Pitch", "R_Thumb_Flexor"}
# Movable hand joints of berkeley_humanoid_lite_v1arm.urdf (both hands).
ROBOT_URDF = {f"{s}_{f}_{j}" for s in "RL" for f in ("Index", "Middle", "Ring", "Pinky") for j in ("Pitch", "Yaw")}


@pytest.fixture
def maps():
    return load_maps(yaml.safe_load(CONFIG.read_text())["visualization"])


def test_every_v2_joint_is_mapped_and_nothing_else(maps):
    assert set(estimate([0.0] * 15, maps, "right")) == V2_RIGHT
    left = set(estimate([0.0] * 15, maps, "left"))
    assert ROBOT_URDF & left == {n for n in ROBOT_URDF if n.startswith("L_")}


def test_open_is_zero_and_full_pull_reaches_the_limits(maps):
    assert all(v == 0.0 for v in estimate([0.0] * 15, maps, "right").values())
    full = estimate([1.0] * 15, maps, "right")
    assert full["R_Index_Pitch"] == pytest.approx(1.309) and full["R_Index_Yaw"] == pytest.approx(0.0)
    assert full["R_Index_Flexor"] == pytest.approx(1.047)


def test_differential_pair_gives_yaw_and_is_clamped(maps):
    pose = [0.0] * 15
    pose[6] = 1.0                                     # index upper only
    joints = estimate(pose, maps, "right")
    assert joints["R_Index_Yaw"] == pytest.approx(0.349) and joints["R_Index_Pitch"] == pytest.approx(0.6545)
    pose[6], pose[7] = 0.0, 1.0                       # index lower only: the other way
    assert estimate(pose, maps, "right")["R_Index_Yaw"] == pytest.approx(-0.349)


def test_grips_draw_what_their_names_say(maps):
    grips = load_config(CONFIG).grips
    point = estimate(grips["point"], maps, "right")
    assert point["R_Index_Pitch"] == 0.0 and point["R_Middle_Pitch"] > 1.0
    pinch = estimate(grips["pinch"], maps, "right")
    assert pinch["R_Index_Pitch"] > 1.0 and pinch["R_Ring_Pitch"] == 0.0 and pinch["R_Thumb_Pitch"] > 0.0


def test_bad_map_is_refused():
    with pytest.raises(ValueError, match="channels are 1..15"):
        load_maps({"joints": {"X": {"coef": {16: 1.0}, "limits": [0, 1]}}})


SNIPPET = '''<robot name="h">
  <joint name="R_Index_Flexor" type="revolute"/>
  <joint name="R_Index_DIP" type="revolute"><mimic joint="Index_Flexor" multiplier="1"/></joint>
  <joint name="R_Thumb_DIP" type="revolute"><mimic joint="R_Thumb_Flexor"/></joint>
  <link name="a"><visual><geometry><mesh filename="package://dexhandv2_description/meshes/right/a.stl"/></geometry></visual></link>
</robot>'''


def test_upstream_mimic_names_are_fixed():
    fixed = fix_mimics(SNIPPET)
    assert '<mimic joint="R_Index_Flexor"' in fixed
    assert '<mimic joint="R_Thumb_Flexor"' in fixed          # already right: untouched


def test_meshes_point_at_the_vendored_files(tmp_path):
    urdf = tmp_path / "dexhandv2_description" / "urdf" / "dexhandv2_right.urdf"
    urdf.parent.mkdir(parents=True)
    urdf.write_text(SNIPPET)
    text = load(urdf)
    assert f'file://{tmp_path / "dexhandv2_description"}/meshes/right/a.stl' in text
    assert "package://" in load(urdf, package_found=True)
    with pytest.raises(FileNotFoundError, match="BHL_DESCRIPTION_WS"):
        load(tmp_path / "missing.urdf")
