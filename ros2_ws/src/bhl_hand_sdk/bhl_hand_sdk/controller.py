"""One DexHand over CAN, without ROS: slew-limited pull fractions, 50 Hz pose sets,
and -- with protocol v2 firmware -- a delivery result for every set and command.

What a result can and cannot say (the hardware decides this, not the code):

    SENT        the five frames were queued on the PC adapter (v1: all you get)
    APPLIED     the ESP32 received all five, every check passed, PWM duties written
    REJECT_*    the ESP32 received the set and refused it, with the reason
    NO_REPLY    nothing came back within reply_timeout: frames lost, hand off the
                bus, or the reply lost or slow -- an APPLIED for it that arrives
                later is counted as "late" and clears the miss, and the applied
                counter in the next reply tells a lost reply from a lost set

No result says where a finger is: the ES3352 servos have no feedback, and
"command_us" here is always a command, never a measurement.

Safety behaviour, in the spirit of bhl_arm_sdk:

  * enable pings first (v2) and refuses a hand that does not answer, answers
    with another protocol, or carries a different calibration profile
  * a hand already driving its PWM is resumed (RESUME), never restarted (START
    drops the PWM for a moment and would let go of what it holds)
  * `max_missed` sets without a reply in a row -> fault LOST, streaming stops;
    the hand keeps its last PWM by itself (firmware watchdog)
  * a REJECT_INHIBITED we did not ask for (someone else sent HOLD / OFF, or the
    hand latched after an output failure) stops streaming: fault HELD_ELSEWHERE
  * poses that pull antagonist tendons together are refused before sending
  * dry_run (default) opens no bus: every set is "APPLIED" at once
"""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass

from .config import HandConfig, HandSpec
from .hand_can import (CHANNELS, Flag, HandBus, Mode, Pong, Result, Status, control_frame,
                       ping_frame, pose_frames)

PRESENT_FOR = 1.5      # [s] heard from within this -> present (heartbeat is 0.5 s)
MODE_NAMES = {Mode.OFF: "OFF", Mode.START: "START", Mode.HOLD: "HOLD", Mode.RESUME: "RESUME"}


@dataclass
class Delivery:
    """What happened to one pose set or control command."""

    ok: bool
    result: str               # APPLIED, CONTROL_ACK, SENT, NO_REPLY, REJECT_..., SEND_FAILED, ...
    sequence: int = -1
    latency_ms: float = -1.0  # send -> reply; -1 without a reply
    detail: int = 0xFF        # channel (1-based in `message`) or mode, as the hand sent it
    message: str = ""

    def as_dict(self) -> dict:
        return asdict(self)


class HandController:
    def __init__(self, config: HandConfig, hand: str, dry_run: bool = True, bus=None,
                 channel: str | None = None) -> None:
        self.config = config
        self.spec: HandSpec = config.hand(hand)
        if channel:
            self.spec.channel = channel      # e.g. vcan0 or sim instead of the config's can0
        self.dry_run = dry_run
        self.protocol = self.spec.protocol
        self.bus = None
        if not dry_run:
            self.bus = bus if bus is not None else HandBus(self.spec.channel, [self.spec.node])
        open_pose = list(config.grips["open"])
        self.goal = list(open_pose)
        self.command = list(open_pose)
        self.live = False             # streaming pose sets every step
        self.held = False             # we sent HOLD (or OFF) and are not streaming
        self.estopped = False
        self.fault = ""
        self.message = ""
        self._sequence = 0
        self.last_delivery = Delivery(False, "NONE")
        self.last_status: Status | None = None
        self.last_pong: Pong | None = None
        self.last_heard = 0.0
        self.stats = {"sets_sent": 0, "applied": 0, "rejected": 0, "no_reply": 0, "acks_lost": 0,
                      "send_failed": 0, "late": 0, "max_latency_ms": 0.0}
        self.missed = 0
        self._last_applied_count: int | None = None
        self._awaiting = -1           # sequence whose STATUS _send_set is waiting for

    # ------------------------------------------------------------------ helpers
    @property
    def confirmed(self) -> bool:
        """True when the hand can tell us what it did (protocol v2 on a real bus)."""
        return not self.dry_run and self.protocol >= 2

    @property
    def present(self) -> bool:
        if self.dry_run:
            return True
        return self.confirmed and time.monotonic() - self.last_heard < PRESENT_FOR

    @property
    def command_us(self) -> list[int]:
        return self.spec.to_us(self.command)

    def _next_sequence(self) -> int:
        self._sequence = (self._sequence + 1) & 0xFF
        return self._sequence

    def _note(self, frame) -> None:
        """Book-keeping for every frame from the hand."""
        self.last_heard = frame.stamp
        if isinstance(frame, Pong):
            self.last_pong = frame
            return
        self.last_status = frame
        if frame.result == Result.APPLIED and frame.sequence != self._awaiting:
            # A reply for an earlier set that missed its window: delivered, just slow.
            self.stats["late"] += 1
            self.stats["no_reply"] = max(0, self.stats["no_reply"] - 1)
            self.missed = 0
        if self._last_applied_count is not None:
            jump = (frame.applied - self._last_applied_count) & 0xFFFF
            if frame.result == Result.APPLIED and jump > 1:
                self.stats["acks_lost"] += jump - 1      # applied, but the STATUS never arrived
            elif frame.result != Result.APPLIED and jump > 0:
                self.stats["acks_lost"] += jump
        self._last_applied_count = frame.applied

    def _await(self, match, timeout: float):
        """Read replies for up to `timeout`; the first one `match` accepts, or None."""
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            frame = self.bus.recv(left)
            if frame is None:
                return None
            self._note(frame)
            if match(frame):
                return frame

    def poll(self) -> None:
        """Drain replies that arrived on their own (heartbeats, late acks)."""
        if self.confirmed:
            while (frame := self.bus.recv(0.0)) is not None:
                self._note(frame)

    # --------------------------------------------------------------- commands
    def ping(self) -> Pong | None:
        if self.dry_run:
            return Pong(self.spec.node, 2, Flag.PWM_ON if self.live else 0, self.spec.profile_tag,
                        time.monotonic())
        if not self.confirmed:
            return None
        self.bus.send(*ping_frame(self.spec.node))
        return self._await(lambda f: isinstance(f, Pong) and f.node == self.spec.node,
                           self.config.ping_timeout)

    def _control(self, mode: int) -> Delivery:
        name = MODE_NAMES[mode]
        if self.dry_run:
            return Delivery(True, "CONTROL_ACK", detail=mode, message=f"{name} (dry run)")
        sent_at = time.monotonic()
        if not self.bus.send(*control_frame(self.spec.node, mode)):
            self.stats["send_failed"] += 1
            return Delivery(False, "SEND_FAILED", detail=mode, message=f"{name}: CAN send failed")
        if not self.confirmed:
            return Delivery(True, "SENT", detail=mode, message=f"{name} sent (v1: no confirmation)")
        reply = self._await(lambda f: isinstance(f, Status) and f.result == Result.CONTROL_ACK
                            and f.detail == mode, self.config.ping_timeout)
        if reply is None:
            return Delivery(False, "NO_REPLY", detail=mode, message=f"{name}: no CONTROL_ACK")
        return Delivery(True, "CONTROL_ACK", detail=mode,
                        latency_ms=(reply.stamp - sent_at) * 1e3, message=name)

    def _send_set(self) -> Delivery:
        sequence = self._next_sequence()
        pulses = self.command_us
        self.stats["sets_sent"] += 1
        if self.dry_run:
            self.stats["applied"] += 1
            return Delivery(True, "APPLIED", sequence, 0.0, message="dry run")
        sent_at = time.monotonic()
        for ident, data in pose_frames(self.spec.node, sequence, pulses):
            if not self.bus.send(ident, data):
                # The sequence is spent: a partial set is never completed under it.
                self.stats["send_failed"] += 1
                return Delivery(False, "SEND_FAILED", sequence, message="CAN send failed mid-set")
        if not self.confirmed:
            return Delivery(True, "SENT", sequence, message="v1 firmware: delivery not confirmed")
        self._awaiting = sequence
        reply = self._await(lambda f: isinstance(f, Status) and f.sequence == sequence and
                            f.result not in (Result.HEARTBEAT, Result.CONTROL_ACK),
                            self.config.reply_timeout)
        if reply is None:
            self.stats["no_reply"] += 1
            return Delivery(False, "NO_REPLY", sequence)
        latency = (reply.stamp - sent_at) * 1e3
        self.stats["max_latency_ms"] = max(self.stats["max_latency_ms"], round(latency, 3))
        if reply.result == Result.APPLIED:
            self.stats["applied"] += 1
            return Delivery(True, "APPLIED", sequence, latency)
        self.stats["rejected"] += 1
        message = ""
        if reply.detail != 0xFF and reply.result in (Result.REJECT_RANGE, Result.REJECT_CONFLICT):
            message = f"channel {reply.detail + 1}"
        return Delivery(False, reply.name, sequence, latency, reply.detail, message)

    # ---------------------------------------------------------------- lifecycle
    def enable(self) -> Delivery:
        """Start streaming. The first set is the current goal (open unless set)."""
        if self.estopped:
            self.message = "E-STOP is latched; reset it first"
            return Delivery(False, "ESTOP", message=self.message)
        if self.live:
            return Delivery(True, "ALREADY_LIVE")
        self.fault = ""
        self.missed = 0
        mode = Mode.START
        if self.confirmed:
            pong = self.ping()
            if pong is None:
                return self._fail("NO_REPLY", "no PONG: hand off the bus, unpowered, or v1 firmware")
            if pong.version != 2:
                return self._fail("PROTOCOL", f"hand speaks protocol {pong.version}, config says 2")
            if self.spec.profile_tag and pong.profile_tag != self.spec.profile_tag.lower()[:8]:
                return self._fail("PROFILE_MISMATCH",
                                  f"hand calibration {pong.profile_tag}, config {self.spec.profile_tag[:8]}")
            if pong.flags & Flag.PWM_ON:
                mode = Mode.RESUME       # keep whatever it holds; START would drop the PWM
        control = self._control(mode)
        if not control.ok:
            return self._fail(control.result, control.message)
        self.command = list(self.goal)
        delivery = self._send_set()
        self.last_delivery = delivery
        if not delivery.ok:
            return self._fail(delivery.result, f"first set: {delivery.result} {delivery.message}".strip())
        self.live, self.held = True, False
        self.message = f"live ({MODE_NAMES[mode]}), first set {delivery.result}"
        return delivery

    def _fail(self, fault: str, message: str) -> Delivery:
        self.fault, self.message, self.live = fault, message, False
        return Delivery(False, fault, message=message)

    def hold(self) -> Delivery:
        """Stop streaming and tell the hand to keep its last PWM and ignore poses."""
        self.live = False
        self.held = True
        delivery = self._control(Mode.HOLD)
        self.message = "held" if delivery.ok else f"HOLD {delivery.result}"
        return delivery

    def resume(self) -> Delivery:
        """After hold: continue from the pose being held (no PWM drop)."""
        if self.estopped:
            return Delivery(False, "ESTOP", message="E-STOP is latched; reset it first")
        self.goal = list(self.command)
        return self.enable()

    def off(self) -> Delivery:
        """Stop the PWM. Not a torque-off guarantee, and the grip is released."""
        self.live = False
        self.held = True
        delivery = self._control(Mode.OFF)
        self.message = "PWM off" if delivery.ok else f"OFF {delivery.result}"
        return delivery

    def estop(self, reason: str = "E-STOP") -> Delivery:
        delivery = self.off() if self.config.estop_action == "off" else self.hold()
        self.estopped = True
        self.message = f"{reason} ({self.config.estop_action}: {delivery.result})"
        return delivery

    def reset_estop(self) -> None:
        self.estopped = False
        self.message = "E-STOP cleared; hand is held, call enable / resume"

    def close(self) -> None:
        if self.bus is not None:
            if self.live:
                self.hold()        # leave the hand holding, not limp
            self.bus.close()
        self.live = False

    # ------------------------------------------------------------------ targets
    def set_goal(self, fractions) -> list[float]:
        values = [float(v) for v in fractions]
        problem = self.config.check_pose(values)
        if problem:
            raise ValueError(problem)
        self.goal = values
        return self.goal

    def set_goal_us(self, pulses_us) -> list[float]:
        return self.set_goal(self.spec.to_fractions(pulses_us))

    def set_grip(self, name: str, level: float = 1.0) -> list[float]:
        """Goal between open (level 0) and the named grip (level 1)."""
        if name not in self.config.grips:
            raise KeyError(f"unknown grip {name}; known: {sorted(self.config.grips)}")
        level = min(max(float(level), 0.0), 1.0)
        start, end = self.config.grips["open"], self.config.grips[name]
        return self.set_goal([a + level * (b - a) for a, b in zip(start, end)])

    def settled(self) -> bool:
        return all(abs(c - g) <= 1e-6 for c, g in zip(self.command, self.goal))

    # --------------------------------------------------------------------- loop
    def step(self, dt: float) -> Delivery | None:
        """One cycle: slew the command, send one pose set, judge the reply."""
        if not self.live or self.estopped:
            self.poll()
            return None
        max_step = self.config.max_speed * dt
        self.command = [c + max(-max_step, min(max_step, g - c)) for c, g in zip(self.command, self.goal)]
        delivery = self._send_set()
        self.last_delivery = delivery
        if delivery.ok:
            self.missed = 0
            return delivery
        if delivery.result in ("NO_REPLY", "SEND_FAILED", "REJECT_DEADLINE"):
            self.missed += 1
            if self.missed >= self.config.max_missed:
                self._fail("LOST", f"{self.missed} sets in a row without APPLIED "
                                   f"(last {delivery.result}); hand keeps its last PWM")
        elif delivery.result == "REJECT_INHIBITED":
            self._fail("HELD_ELSEWHERE", "hand refuses poses (HOLD/OFF from someone else, "
                                         "or latched after an output failure)")
        else:
            self._fail(delivery.result, f"set {delivery.sequence}: {delivery.result} {delivery.message}".strip())
        return delivery

    def status(self) -> dict:
        s = self.last_status
        return {
            "hand": self.spec.name, "node": self.spec.node, "channel": self.spec.channel,
            "dry_run": self.dry_run, "protocol": self.protocol, "confirmed": self.confirmed,
            "present": self.present, "live": self.live, "held": self.held, "estop": self.estopped,
            "fault": self.fault, "message": self.message, "missed": self.missed,
            "settled": self.settled(),
            "goal": [round(v, 4) for v in self.goal],
            "command": [round(v, 4) for v in self.command],
            "command_us": self.command_us,         # a command, never a measurement
            "last_delivery": self.last_delivery.as_dict(),
            "hand_reported": None if s is None else {
                "result": s.name, "sequence": s.sequence, "flags": s.flag_names(),
                "applied_count": s.applied, "tx_errors": s.tx_errors, "rx_errors": s.rx_errors,
                "age_s": round(time.monotonic() - s.stamp, 3)},
            "profile_tag": None if self.last_pong is None else self.last_pong.profile_tag,
            "stats": dict(self.stats),
        }
