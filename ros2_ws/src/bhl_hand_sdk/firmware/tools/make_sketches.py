"""Regenerate DexHandLeft/ and DexHandRight/ (protocol v2).

Shared code comes from ../DexHandCan_shared/ (HandController.h, PoseReceiver.h,
ServoOutput.h, StatusReport.h, DexHand.ino.in). Each hand keeps its own
BoardConfig.h -- generated in dexhand_control from profiles/<side>_hand.json
(tools/generate_board_config.py). Copy a new BoardConfig.h into DexHandLeft/ or
DexHandRight/ first, then run this.

    python3 tools/make_sketches.py            # from bhl_hand_sdk/firmware
    python3 tools/make_sketches.py --check    # exit 1 if a sketch is out of date
"""

from __future__ import annotations

import sys
from pathlib import Path

FIRMWARE = Path(__file__).resolve().parents[1]
SHARED = FIRMWARE / "DexHandCan_shared"
HEADERS = ("HandController.h", "PoseReceiver.h", "ServoOutput.h", "StatusReport.h")
HANDS = {"Left": (60, "왼손", "left"), "Right": (61, "오른손", "right")}


def expected() -> dict[Path, str]:
    template = (SHARED / "DexHand.ino.in").read_text()
    files = {}
    for side, (node, korean, lower) in HANDS.items():
        folder = FIRMWARE / f"DexHand{side}"
        for name in HEADERS:
            files[folder / name] = (SHARED / name).read_text()
        files[folder / f"DexHand{side}.ino"] = (
            template.replace("@SIDE@", side).replace("@side@", lower).replace("@NODE@", str(node))
            .replace("@KOREAN@", korean).replace("@NODEHEX@", f"0x{node:02X}"))
    return files


def main() -> int:
    check = "--check" in sys.argv[1:]
    stale = []
    for path, text in expected().items():
        if not path.is_file() or path.read_text() != text:
            stale.append(path.relative_to(FIRMWARE))
            if not check:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(text)
    for side in HANDS:
        if not (FIRMWARE / f"DexHand{side}" / "BoardConfig.h").is_file():
            print(f"missing DexHand{side}/BoardConfig.h (copy it from dexhand_control)")
            return 1
    if check:
        print("up to date" if not stale else "out of date: " + ", ".join(map(str, stale)))
        return 1 if stale else 0
    print("wrote " + (", ".join(map(str, stale)) if stale else "nothing (already up to date)"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
