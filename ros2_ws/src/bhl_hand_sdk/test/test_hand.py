"""bhl_hand_sdk core tests without ROS or CAN hardware: python3 -m pytest -q test

The hand is test/fake_hand.py on a python-can virtual bus, following the v2
firmware (firmware/common/HandController.h)."""

from __future__ import annotations

import itertools
import struct
import sys
import time
from pathlib import Path

import can
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fake_hand import FakeHand  # noqa: E402

from bhl_hand_sdk.config import load_config  # noqa: E402
from bhl_hand_sdk.controller import HandController  # noqa: E402
from bhl_hand_sdk.diagnostics import ERROR, OK, WARN, diagnostics  # noqa: E402
from bhl_hand_sdk.hand_can import (Flag, HandBus, Mode, Result, can_id, control_frame,  # noqa: E402
                                   decode, pose_frames)

CONFIG = Path(__file__).resolve().parents[1] / "config" / "dexhand.yaml"
_channels = itertools.count()


@pytest.fixture
def config():
    cfg = load_config(CONFIG)
    cfg.reply_timeout = 0.05          # threads on a laptop, not an ESP32 on a wire
    cfg.ping_timeout = 0.2
    return cfg


@pytest.fixture
def rig(config):
    """(controller, fake hand) for the right hand on a fresh virtual bus."""
    made = []

    def make(protocol=2, tag="c7b282e4", hand_protocol=None):
        channel = f"hand_test_{next(_channels)}"
        spec = config.hand("right")
        spec.protocol = protocol
        fake = FakeHand(channel, spec.node, spec.low_us, spec.high_us,
                        protocol=protocol if hand_protocol is None else hand_protocol, tag=tag)
        bus = HandBus(channel, [spec.node], interface="virtual")
        ctrl = HandController(config, "right", dry_run=False, bus=bus)
        made.append((ctrl, fake))
        return ctrl, fake

    yield make
    for ctrl, fake in made:
        ctrl.close()
        fake.close()


def run(ctrl, seconds, dt=0.02):
    results = []
    for _ in range(round(seconds / dt)):
        results.append(ctrl.step(dt))
    return results


# --------------------------------------------------------------------- protocol
def test_ids_follow_the_arm_rule_and_do_not_collide():
    assert control_frame(61, Mode.HOLD) == (0x03D, bytes([0x02, 61]))
    assert [i for i, _ in pose_frames(60, 1, [700] * 15)] == [0x33C] * 5
    assert can_id(5, 61) == 0x2BD and can_id(3, 60) == 0x1BC
    arm_ids = {can_id(f, n) for f in range(16) for n in range(1, 11)}
    hand_ids = {can_id(f, n) for f in (0, 3, 4, 5, 6) for n in (60, 61)}
    assert not arm_ids & hand_ids


def test_pose_frame_bytes_match_the_firmware():
    frames = pose_frames(61, 0x105, list(range(1000, 1015)))     # sequence wraps to 0x05
    assert frames[0][1] == struct.pack("<BB3H", 0x05, 0x10, 1000, 1001, 1002)
    assert frames[4][1][1] == 0x14


def test_status_decoding():
    data = struct.pack("<BBBBHBB", 7, Result.REJECT_RANGE, Flag.PWM_ON | Flag.INHIBITED, 13, 300, 1, 2)
    s = decode(0x2BD, data)
    assert (s.node, s.sequence, s.name, s.detail, s.applied) == (61, 7, "REJECT_RANGE", 13, 300)
    assert s.flag_names() == ["PWM_ON", "INHIBITED"]
    assert decode(0x33D, data) is None                                   # a pose frame is not a reply


# ----------------------------------------------------------------------- config
def test_config_profiles_and_conversion(config):
    right = config.hand("right")
    assert right.node == 61 and right.to_us([0] * 15) == right.open_us
    assert right.to_us([1] * 15) == right.close_us
    assert right.to_us([2.0] + [0] * 14)[0] == right.close_us[0]           # clamped
    assert right.to_fractions(right.close_us) == [1.0] * 15
    with pytest.raises(ValueError, match="channels \\[13\\]"):
        right.to_fractions(right.open_us[:12] + [2400] + right.open_us[13:])


def test_antagonist_thumb_pair_is_refused(config):
    ctrl = HandController(config, "right", dry_run=True)
    pose = [0.0] * 15
    pose[12], pose[13] = 0.7, 0.6
    with pytest.raises(ValueError, match="13 and 14"):
        ctrl.set_goal(pose)
    ctrl.set_grip("power")
    assert ctrl.goal[12] == 1.0 and ctrl.goal[13] == 0.0


# ---------------------------------------------------------------------- dry run
def test_dry_run_slews_and_reports_applied(config):
    ctrl = HandController(config, "right", dry_run=True)
    assert ctrl.enable().result == "APPLIED"
    ctrl.set_grip("power")
    run(ctrl, 0.24)
    assert ctrl.command[0] == pytest.approx(0.48, abs=1e-9)       # 2 /s * 0.24 s
    run(ctrl, 0.5)
    assert ctrl.settled() and ctrl.command_us[0] == config.hand("right").close_us[0]


# ------------------------------------------------------------- v2 delivery path
def test_enable_pings_starts_and_gets_applied(rig):
    ctrl, fake = rig()
    first = ctrl.enable()
    assert first.ok and first.result == "APPLIED" and first.latency_ms >= 0
    assert fake.modes == [Mode.START] and fake.pulses == ctrl.spec.open_us
    assert ctrl.last_pong.profile_tag == "c7b282e4" and ctrl.present


def test_every_step_is_confirmed_with_its_sequence(rig):
    ctrl, fake = rig()
    ctrl.enable()
    ctrl.set_grip("pinch", 0.5)
    results = run(ctrl, 0.2)
    assert all(r.ok and r.result == "APPLIED" for r in results)
    assert len({r.sequence for r in results}) == len(results)        # a new sequence every set
    assert fake.applied == 1 + len(results)
    assert ctrl.stats["applied"] == fake.applied


def test_enable_refuses_silent_hand(rig):
    ctrl, fake = rig()
    fake.silent = True
    d = ctrl.enable()
    assert not d.ok and ctrl.fault == "NO_REPLY" and not ctrl.live


def test_enable_refuses_wrong_calibration(rig):
    ctrl, fake = rig(tag="deadbeef")
    d = ctrl.enable()
    assert d.result == "PROFILE_MISMATCH" and not fake.modes      # nothing sent after the ping


def test_enable_resumes_a_hand_that_is_already_driving(rig):
    ctrl, fake = rig()
    fake.pwm_on = True                       # e.g. the node restarted while the hand holds a grip
    ctrl.enable()
    assert fake.modes == [Mode.RESUME]


def test_lost_after_max_missed(rig, config):
    ctrl, fake = rig()
    ctrl.enable()
    fake.silent = True
    results = run(ctrl, 0.02 * config.max_missed)
    assert all(r.result == "NO_REPLY" for r in results)
    assert ctrl.fault == "LOST" and not ctrl.live
    assert ctrl.step(0.02) is None           # nothing more is streamed


def test_lost_group_shows_as_no_reply_and_recovers(rig):
    ctrl, fake = rig()
    ctrl.enable()
    fake.lose_group = 2
    assert ctrl.step(0.02).result == "NO_REPLY"
    fake.lose_group = None
    assert ctrl.step(0.02).result == "APPLIED" and ctrl.missed == 0


def test_lost_reply_is_told_apart_from_lost_set(rig):
    ctrl, fake = rig()
    ctrl.enable()
    fake.drop_status = 1                     # applied, but its STATUS is lost
    assert ctrl.step(0.02).result == "NO_REPLY"
    assert ctrl.step(0.02).result == "APPLIED"
    assert ctrl.stats["acks_lost"] == 1      # the applied counter jumped by 2


def test_slow_reply_counts_as_late_not_lost(rig, config):
    ctrl, fake = rig()
    ctrl.enable()
    config.reply_timeout = 0.01
    fake.delay = 0.03                        # every reply misses its window...
    for _ in range(3 * config.max_missed):
        ctrl.step(0.02)
        time.sleep(0.02)                     # ...and arrives during the next cycle
    assert ctrl.live and ctrl.fault == ""
    assert ctrl.stats["late"] > config.max_missed


def test_hold_elsewhere_stops_streaming(rig):
    ctrl, fake = rig()
    ctrl.enable()
    fake.inhibited = True                    # another sender put the hand in HOLD
    d = ctrl.step(0.02)
    assert d.result == "REJECT_INHIBITED" and ctrl.fault == "HELD_ELSEWHERE" and not ctrl.live


def test_range_reject_reports_channel(rig):
    ctrl, fake = rig()
    ctrl.enable()
    fake.high[4] = 1000                      # firmware flashed with a narrower range than the config
    ctrl.set_grip("open")
    ctrl.command[4] = 0.0                    # open_us[4] = 1750 > 1000
    d = ctrl.step(0.02)
    assert d.result == "REJECT_RANGE" and d.message == "channel 5" and ctrl.fault == "REJECT_RANGE"


def test_hold_resume_keep_the_pose(rig):
    ctrl, fake = rig()
    ctrl.enable()
    ctrl.set_grip("power", 0.4)
    run(ctrl, 0.3)
    held = list(ctrl.command)
    h = ctrl.hold()
    assert h.ok and h.result == "CONTROL_ACK" and fake.inhibited and fake.pwm_on
    assert ctrl.step(0.02) is None
    r = ctrl.resume()
    assert r.result == "APPLIED" and fake.modes[-1] == Mode.RESUME and ctrl.goal == held


def test_estop_holds_and_latches(rig):
    ctrl, fake = rig()
    ctrl.enable()
    ctrl.estop("arm E-STOP")
    assert ctrl.estopped and fake.modes[-1] == Mode.HOLD
    assert ctrl.enable().result == "ESTOP"
    ctrl.reset_estop()
    assert ctrl.resume().ok


def test_heartbeats_mark_presence_while_idle(rig):
    ctrl, fake = rig()
    assert not ctrl.present
    fake._status(0, Result.HEARTBEAT)
    time.sleep(0.05)
    ctrl.step(0.02)                          # not live: only drains replies
    assert ctrl.present and ctrl.last_status.name == "HEARTBEAT"


# -------------------------------------------------------------- v1 firmware path
def test_v1_streams_without_confirmation(rig):
    ctrl, fake = rig(protocol=1)
    d = ctrl.enable()
    results = run(ctrl, 0.1)
    time.sleep(0.1)                          # the fake reads on its own thread
    assert d.ok and d.result == "SENT" and fake.modes == [Mode.START]
    assert all(r.result == "SENT" for r in results) and ctrl.live
    assert fake.applied == 1 + len(results)  # it does apply; we just cannot know
    assert not ctrl.present                  # nothing to judge presence by


# ------------------------------------------------------------------ diagnostics
def test_diagnostics_levels(config, rig):
    ctrl = HandController(config, "right", dry_run=True)
    assert diagnostics(ctrl.status())[0]["level"] == WARN       # output off
    ctrl.enable()
    assert diagnostics(ctrl.status())[0]["level"] == OK
    real, fake = rig()
    fake.silent = True
    real.enable()
    assert diagnostics(real.status())[0]["level"] == ERROR
