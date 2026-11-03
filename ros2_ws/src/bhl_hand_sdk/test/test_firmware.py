"""The sketches in firmware/ match config/dexhand.yaml, so the board that gets
DexHandLeft/ is the one the SDK calls "left" (node, pulse ranges, profile tag).

    python3 -m pytest -q test/test_firmware.py
"""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from bhl_hand_sdk.config import load_config

PACKAGE = Path(__file__).resolve().parents[1]
FIRMWARE = PACKAGE / "firmware"
CONFIG = PACKAGE / "config" / "dexhand.yaml"
SKETCH = {"left": "DexHandLeft", "right": "DexHandRight"}


def board_config(side: str) -> dict:
    text = (FIRMWARE / SKETCH[side] / "BoardConfig.h").read_text()

    def array(name):
        return [int(v) for v in re.search(rf"{name}\[15\] = \{{([^}}]*)\}}", text).group(1).split(",")]

    return {
        "node": int(re.search(r"kNodeId = (\d+);", text).group(1)),
        "min": array("kMinUs"),
        "max": array("kMaxUs"),
        "sha": re.search(r'kProfileSha256\[\] = "([0-9a-f]{64})"', text).group(1),
    }


@pytest.mark.parametrize("side", ["left", "right"])
def test_sketch_matches_the_sdk_profile(side):
    spec = load_config(CONFIG).hand(side)
    board = board_config(side)
    ino = (FIRMWARE / SKETCH[side] / f"{SKETCH[side]}.ino").read_text()
    assert board["node"] == spec.node
    assert re.search(rf"CAN_NODE_ID = {spec.node};", ino)
    assert board["min"] == spec.low_us, "firmware range != min(open_us, close_us)"
    assert board["max"] == spec.high_us, "firmware range != max(open_us, close_us)"
    assert board["sha"][:8] == spec.profile_tag
    assert "hand CAN protocol v2" in ino


def test_the_two_boards_differ():
    left, right = board_config("left"), board_config("right")
    assert left["node"] != right["node"] and left["sha"] != right["sha"]


def test_sketches_are_generated_from_the_shared_sources():
    result = subprocess.run([sys.executable, str(FIRMWARE / "tools" / "make_sketches.py"), "--check"],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.skipif(shutil.which("g++") is None, reason="no C++ compiler")
def test_firmware_logic_native(tmp_path):
    binary = tmp_path / "ack_test"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    f"-I{FIRMWARE / 'DexHandCan_shared'}", str(FIRMWARE / "tests" / "ack_test.cpp"),
                    "-o", str(binary)], check=True)
    result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert result.returncode == 0 and result.stdout.startswith("PASS"), result.stdout
