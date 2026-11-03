"""Bench tool for one hand, without ROS: talks to the hand directly and prints
what came back. Do not run it while hand_controller owns the same hand.

    hand_cli right ping                       # is it there, which protocol, which calibration
    hand_cli right grip power --level 0.5     # enable, slew there, report every set
    hand_cli right grip pinch --hold-after    # ... then HOLD (keep the grip)
    hand_cli right open
    hand_cli right hold | resume | off
    hand_cli right listen --seconds 3         # print heartbeats / replies only
    hand_cli right grip power --dry-run       # no CAN opened
    hand_cli right grip pinch --channel sim   # against the simulated board (firmware logic, no CAN)
    hand_cli right grip pinch --channel vcan0 # against hand_sim --channel vcan0
    hand_cli right grip power --channel sim --sim-command "drop right 3"   # inject a fault first

Exit code 0 only when the hand confirmed (v2) or, for v1 firmware, frames were sent.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import Counter
from pathlib import Path

from .config import load_config
from .controller import HandController


def default_config() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(Path(get_package_share_directory("bhl_hand_sdk")) / "config" / "dexhand.yaml")
    except Exception:  # noqa: BLE001 - not under ROS: the source tree
        return str(Path(__file__).resolve().parents[1] / "config" / "dexhand.yaml")


def stream(hand: HandController, seconds: float, settle: bool) -> Counter:
    dt = 1.0 / hand.config.control_rate_hz
    results: Counter = Counter()
    latencies = []
    end = time.monotonic() + seconds
    next_tick = time.monotonic()
    while time.monotonic() < end:
        delivery = hand.step(dt)
        if delivery is None:
            print(f"  stopped: {hand.fault} {hand.message}")
            break
        results[delivery.result] += 1
        if delivery.latency_ms >= 0:
            latencies.append(delivery.latency_ms)
        if not delivery.ok:
            print(f"  seq {delivery.sequence:3d}: {delivery.result} {delivery.message}")
        if settle and hand.settled():
            break
        next_tick += dt
        time.sleep(max(0.0, next_tick - time.monotonic()))
    summary = ", ".join(f"{k} {v}" for k, v in results.most_common())
    if latencies:
        latencies.sort()
        summary += (f"; latency median {latencies[len(latencies) // 2]:.2f} ms, "
                    f"max {latencies[-1]:.2f} ms")
    print(f"  {sum(results.values())} sets: {summary}; acks lost {hand.stats['acks_lost']}")
    return results


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("hand", help="left | right")
    parser.add_argument("action", choices=["ping", "grip", "open", "hold", "resume", "off", "listen"])
    parser.add_argument("grip", nargs="?", default="power", help="named grip for `grip`")
    parser.add_argument("--level", type=float, default=1.0, help="0 open .. 1 the grip")
    parser.add_argument("--seconds", type=float, default=2.0, help="[s] to stream / listen")
    parser.add_argument("--hold-after", action="store_true", help="HOLD at the end instead of leaving it streaming-less")
    parser.add_argument("--config", default=default_config())
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--protocol", type=int, choices=[1, 2], help="override the config")
    parser.add_argument("--channel", help="override the config's CAN channel: can0, vcan0, sim")
    parser.add_argument("--sim-command", action="append", default=[],
                        help="with --channel sim: a hand_sim command sent before the action")
    args = parser.parse_args(argv)

    config = load_config(args.config)
    if args.protocol:
        config.hand(args.hand).protocol = args.protocol
    hand = HandController(config, args.hand, dry_run=args.dry_run, channel=args.channel)
    for line in args.sim_command:
        hand.bus.sim_command(line)
    spec = hand.spec
    print(f"{spec.name} hand, node {spec.node} on {spec.channel}, protocol {spec.protocol}"
          + (" (DRY RUN)" if args.dry_run else ""))
    ok = True
    try:
        if args.action == "ping":
            pong = hand.ping()
            if pong is None:
                print("  no PONG" + ("" if hand.confirmed else " (v1 firmware cannot answer)"))
                ok = False
            else:
                match = "" if not spec.profile_tag else (
                    " (matches config)" if pong.profile_tag == spec.profile_tag[:8] else
                    f" (config says {spec.profile_tag[:8]}!)")
                print(f"  PONG: protocol {pong.version}, profile {pong.profile_tag}{match}, "
                      f"flags 0x{pong.flags:02X}")
        elif args.action == "listen":
            end = time.monotonic() + args.seconds
            while time.monotonic() < end:
                frame = hand.bus.recv(0.1) if hand.bus else None
                if frame is not None:
                    print(f"  {frame}")
        elif args.action in ("hold", "resume", "off"):
            delivery = getattr(hand, args.action)()
            print(f"  {args.action.upper()}: {delivery.result} {delivery.message}")
            ok = delivery.ok
        else:
            if args.action == "open":
                hand.set_grip("open")
            else:
                hand.set_grip(args.grip, args.level)
            hand.goal, target = list(hand.config.grips["open"]), list(hand.goal)
            first = hand.enable()            # first set: open (the known pose), then slew
            print(f"  enable: {first.result} {first.message}")
            if not first.ok:
                return 1
            hand.goal = target
            results = stream(hand, args.seconds, settle=True)
            ok = hand.live and (results.get("APPLIED", 0) or results.get("SENT", 0)) > 0
            if args.hold_after:
                print(f"  HOLD: {hand.hold().result}")
            hand.live = False              # leave the hand on its last PWM without HOLD
    except (KeyError, ValueError) as error:
        print(f"  {error}")
        ok = False
    finally:
        if hand.bus is not None:
            hand.bus.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
