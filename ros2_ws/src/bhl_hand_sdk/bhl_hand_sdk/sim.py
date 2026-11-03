"""The simulated hand boards (firmware/sim/hand_sim.cpp), from Python.

hand_sim is the boards' real firmware logic (the same headers the ESP32 sketches
compile) with the hardware replaced: PWM becomes a log, CAN becomes SocketCAN
(vcan0) or text on stdin / stdout. Two ways to use it:

  1. On a CAN interface, one process for both boards, typed commands for faults:

         sudo modprobe vcan && sudo ip link add vcan0 type vcan && sudo ip link set up vcan0
         ros2 run bhl_hand_sdk hand_sim --channel vcan0
         ros2 launch bhl_hand_sdk sim.launch.py            # hand nodes on vcan0

  2. Without any kernel CAN (any OS with g++): set a hand's channel to `sim`.
     HandBus then starts its own `hand_sim --stdio` and talks to it through a
     pipe (SimPipeBus); faults go in through SimPipeBus.command() or, in the ROS
     node, the ~/sim_command topic.

         ros2 run bhl_hand_sdk hand_cli right grip pinch --channel sim

`hand_sim` compiles the simulator on first use (g++, C++17) into
~/.cache/bhl_hand_sdk/ and rebuilds when the firmware sources change.
"""

from __future__ import annotations

import hashlib
import os
import queue
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

import can

NODE_HAND = {60: "left", 61: "right"}


def firmware_dir() -> Path:
    """firmware/ from the installed share directory, else the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("bhl_hand_sdk")) / "firmware"
        if (share / "sim" / "hand_sim.cpp").is_file():
            return share
    except Exception:  # noqa: BLE001 - not under ROS
        pass
    return Path(__file__).resolve().parents[1] / "firmware"


def sources(root: Path) -> list[Path]:
    return sorted([root / "sim" / "hand_sim.cpp", *(root / "DexHandCan_shared").glob("*.h"),
                   root / "DexHandLeft" / "BoardConfig.h", root / "DexHandRight" / "BoardConfig.h"])


def build(root: Path | None = None, quiet: bool = False) -> Path:
    """Path of a hand_sim binary built from the current sources."""
    root = root or firmware_dir()
    digest = hashlib.sha256(b"".join(p.read_bytes() for p in sources(root))).hexdigest()[:12]
    cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "bhl_hand_sdk"
    binary = cache / f"hand_sim-{digest}"
    if binary.is_file():
        return binary
    compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("c++") or shutil.which("clang++")
    if compiler is None:
        raise RuntimeError("hand_sim needs a C++17 compiler (g++ or clang++)")
    cache.mkdir(parents=True, exist_ok=True)
    if not quiet:
        print(f"building hand_sim -> {binary}", file=sys.stderr)
    subprocess.run([compiler, "-std=c++17", "-O2", "-Wall", "-Wextra", f"-I{root / 'DexHandCan_shared'}",
                    str(root / "sim" / "hand_sim.cpp"), "-o", str(binary)], check=True)
    return binary


class SimPipeBus(can.BusABC):
    """A python-can bus whose only other nodes are the simulated hand boards."""

    def __init__(self, channel: str = "sim", nodes=(60, 61), can_filters=None, log=None, **kwargs) -> None:
        hands = ",".join(NODE_HAND[n] for n in nodes if n in NODE_HAND) or "left,right"
        self._process = subprocess.Popen(
            [str(build(quiet=True)), "--stdio", "--hands", hands, "--quiet"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log or subprocess.DEVNULL,
            text=True, bufsize=1)
        self._frames: queue.Queue[can.Message] = queue.Queue()
        self._lock = threading.Lock()
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()
        self.channel_info = f"hand_sim --stdio ({hands})"
        super().__init__(channel=channel, can_filters=can_filters, **kwargs)

    def _read(self) -> None:
        for line in self._process.stdout:
            parts = line.split()
            if len(parts) == 3 and parts[0] == "tx":
                self._frames.put(can.Message(arbitration_id=int(parts[1], 16), data=bytes.fromhex(parts[2]),
                                             is_extended_id=False, timestamp=time.time()))

    def _write(self, line: str) -> None:
        with self._lock:
            if self._process.poll() is not None:
                raise can.CanOperationError("hand_sim exited")
            self._process.stdin.write(line + "\n")
            self._process.stdin.flush()

    def send(self, msg: can.Message, timeout: float | None = None) -> None:
        if msg.is_extended_id or msg.is_remote_frame:
            return
        self._write(f"frame {msg.arbitration_id:03x} {bytes(msg.data).hex() or ''}")

    def command(self, line: str) -> None:
        """A hand_sim command: 'power right off', 'drop right 3', 'lose right 2', 'fail right', ..."""
        self._write(line.strip())

    def _recv_internal(self, timeout: float | None):
        try:
            if timeout is None:
                return self._frames.get(), False
            if timeout <= 0:
                return self._frames.get_nowait(), False
            return self._frames.get(timeout=timeout), False
        except queue.Empty:
            return None, False

    def shutdown(self) -> None:
        super().shutdown()
        if self._process.poll() is None:
            try:
                self._process.stdin.close()
                self._process.wait(timeout=2)
            except Exception:  # noqa: BLE001
                self._process.kill()


def main() -> None:
    """`hand_sim ...`: build if needed, then run with the given arguments."""
    binary = build()
    os.execv(str(binary), [str(binary), *sys.argv[1:]])


if __name__ == "__main__":
    main()
