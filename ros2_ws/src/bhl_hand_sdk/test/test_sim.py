"""The SDK against hand_sim: the boards' real firmware logic (the ESP32 sketches'
own headers) compiled for this PC and driven through a pipe, no CAN needed.

Skipped without a C++ compiler."""

from __future__ import annotations

import shutil
import time
from pathlib import Path

import pytest

from bhl_hand_sdk.config import load_config
from bhl_hand_sdk.controller import HandController
from bhl_hand_sdk.hand_can import HandBus, Mode, Pong, Result, Status, pose_frames

pytestmark = pytest.mark.skipif(not (shutil.which("g++") or shutil.which("c++")), reason="no C++ compiler")
CONFIG = Path(__file__).resolve().parents[1] / "config" / "dexhand.yaml"


@pytest.fixture
def hand():
    made = []

    def make(side="right"):
        config = load_config(CONFIG)
        config.reply_timeout = 0.05        # a pipe and two threads, not a wire
        config.ping_timeout = 0.3
        ctrl = HandController(config, side, dry_run=False, channel="sim")
        made.append(ctrl)
        return ctrl

    yield make
    for ctrl in made:
        ctrl.close()


def run(ctrl, cycles, dt=0.02):
    return [ctrl.step(dt) for _ in range(cycles)]


def settle(seconds=0.1):
    time.sleep(seconds)


def test_both_boards_answer_with_their_own_profile(hand):
    assert hand("left").ping().profile_tag == "c3c778d2"
    assert hand("right").ping().profile_tag == "c7b282e4"


def test_stream_is_applied_set_by_set(hand):
    ctrl = hand()
    assert ctrl.enable().result == "APPLIED"
    ctrl.set_grip("pinch", 1.0)
    results = run(ctrl, 30)
    assert all(r.result == "APPLIED" for r in results), {r.result for r in results}
    assert ctrl.settled() and ctrl.last_status.applied == 31 and ctrl.stats["acks_lost"] == 0


def test_hold_and_resume_acked_by_the_firmware(hand):
    ctrl = hand()
    ctrl.enable()
    ctrl.set_grip("power", 0.5)
    run(ctrl, 10)
    held = ctrl.hold()
    assert held.result == "CONTROL_ACK" and ctrl.last_status.inhibited and ctrl.last_status.pwm_on
    assert ctrl.resume().result == "APPLIED"
    assert ctrl.last_status.applied == 12


def test_powered_off_board_goes_lost_and_reboot_needs_enable(hand, monkeypatch):
    ctrl = hand()
    ctrl.enable()
    ctrl.bus.sim_command("power right off")
    settle()
    run(ctrl, ctrl.config.max_missed)
    assert ctrl.fault == "LOST" and not ctrl.live
    ctrl.bus.sim_command("power right on")      # rebooted: PWM off, counters reset
    settle()
    assert ctrl.ping().flags == 0
    assert ctrl.enable().result == "APPLIED"


def test_wrong_sketch_on_the_board_is_refused(hand):
    ctrl = hand()
    ctrl.bus.sim_command("flash right left")    # the left sketch on the right board: node 60 now
    settle()
    d = ctrl.enable()
    assert d.result == "NO_REPLY" and not ctrl.live


def test_lost_group_and_lost_reply_are_told_apart(hand):
    ctrl = hand()
    ctrl.enable()
    ctrl.bus.sim_command("lose right 3")
    settle()
    assert ctrl.step(0.02).result == "NO_REPLY"          # set never completed
    ctrl.bus.sim_command("lose right none")
    ctrl.bus.sim_command("drop right 1")
    settle()
    assert ctrl.step(0.02).result == "NO_REPLY"          # applied, reply lost
    assert ctrl.step(0.02).result == "APPLIED"
    assert ctrl.stats["acks_lost"] == 1


def test_output_failure_latches_in_the_firmware(hand):
    ctrl = hand()
    ctrl.enable()
    ctrl.bus.sim_command("fail right")
    settle()
    d = ctrl.step(0.02)
    assert d.result == "OUTPUT_FAIL" and ctrl.fault == "OUTPUT_FAIL"
    assert ctrl.enable().result == "APPLIED"             # START clears the latch


def test_firmware_range_check_rejects_raw_pulses(hand):
    """Bypass the SDK's clamping: the board itself refuses and names the channel."""
    ctrl = hand()
    ctrl.enable()
    pulses = list(ctrl.spec.open_us)
    pulses[12] = 2500                                    # channel 13 allows 700..2300
    sequence = ctrl._next_sequence()                     # must follow the stream's sequence
    for ident, data in pose_frames(61, sequence, pulses):
        ctrl.bus.send(ident, data)
    reply = ctrl._await(lambda f: isinstance(f, Status) and f.sequence == sequence, 0.5)
    assert reply.result == Result.REJECT_RANGE and reply.detail == 12


def test_stale_sequence_is_ignored_by_the_firmware(hand):
    """More than 127 behind / ahead of the current set: no apply, no reply."""
    ctrl = hand()
    ctrl.enable()
    for ident, data in pose_frames(61, (ctrl._sequence + 200) & 0xFF, ctrl.spec.open_us):
        ctrl.bus.send(ident, data)
    assert ctrl._await(lambda f: isinstance(f, Status) and f.result != Result.HEARTBEAT, 0.2) is None


def test_heartbeat_while_idle(hand):
    ctrl = hand()
    ctrl.ping()
    time.sleep(0.7)
    ctrl.poll()
    assert ctrl.last_status is not None and ctrl.last_status.name == "HEARTBEAT"


def test_handbus_sim_command_only_on_sim():
    bus = HandBus("sim", [61])
    try:
        assert bus.simulated
        bus.send(0x23D, b"\xCA")
        frame = bus.recv(0.5)
        assert isinstance(frame, Pong) and frame.version == 2
        bus.send(0x03D, bytes([Mode.HOLD, 61]))
        assert bus.recv(0.5).name == "CONTROL_ACK"
    finally:
        bus.close()
