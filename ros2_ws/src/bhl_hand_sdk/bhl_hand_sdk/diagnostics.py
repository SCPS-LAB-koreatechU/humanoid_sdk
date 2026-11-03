"""/diagnostics content for hand_controller, as plain dicts (testable without ROS).

    level   OK     live and the last set was APPLIED (v2), or SENT (v1, unconfirmed)
            WARN   output off / held, or v1 firmware (delivery cannot be confirmed),
                   or replies being missed but not yet LOST
            ERROR  a fault: NO_REPLY, PROTOCOL, PROFILE_MISMATCH, LOST, HELD_ELSEWHERE,
                   REJECT_*, OUTPUT_FAIL; or E-STOP latched
"""

from __future__ import annotations

OK, WARN, ERROR, STALE = 0, 1, 2, 3


def diagnostics(status: dict) -> list[dict]:
    if status["estop"]:
        level, message = ERROR, f"E-STOP latched: {status['message']}"
    elif status["fault"]:
        level, message = ERROR, f"{status['fault']}: {status['message']}"
    elif not status["live"]:
        level, message = WARN, "held" if status["held"] else "output off (call ~/enable)"
    elif status["missed"]:
        level, message = WARN, f"{status['missed']} set(s) without reply"
    elif not status["confirmed"] and not status["dry_run"]:
        level, message = WARN, "live, v1 firmware: delivery not confirmed"
    else:
        level, message = OK, "live"
    reported = status["hand_reported"] or {}
    stats = status["stats"]
    return [{
        "level": level,
        "name": f"hand_controller: {status['hand']}",
        "message": ("DRY RUN: " if status["dry_run"] else "") + message,
        "hardware_id": f"{status['channel']}:{status['node']}",
        "values": {
            "protocol": str(status["protocol"]),
            "present": str(status["present"]),
            "last_delivery": status["last_delivery"]["result"],
            "hand_flags": ",".join(reported.get("flags", [])),
            "applied_count": str(reported.get("applied_count", "")),
            "sets_sent": str(stats["sets_sent"]),
            "applied": str(stats["applied"]),
            "no_reply": str(stats["no_reply"]),
            "acks_lost": str(stats["acks_lost"]),
            "rejected": str(stats["rejected"]),
            "max_latency_ms": f"{stats['max_latency_ms']:.2f}",
            "profile_tag": str(status["profile_tag"]),
        },
    }]
