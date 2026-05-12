from __future__ import annotations

from typing import Any


def build_verification_report(
    *,
    state: dict[str, Any],
    diagnostics: dict[str, Any],
    joint_meta: dict[str, Any],
    joint_debug: dict[str, Any],
    limits: dict[str, Any],
) -> dict[str, Any]:
    report: dict[str, Any] = {
        "connected": bool(state.get("connected")),
        "hardware_error": state.get("hardware_error"),
        "joints": {},
        "summary": {
            "tracking_gaps": 0,
            "unreadable": 0,
            "rate_limited": 0,
            "limit_clamped": 0,
        },
    }

    for name, debug in joint_debug.items():
        meta = joint_meta.get(name, {})
        limit = limits.get(name, {})
        entry = {
            "servo_id": meta.get("servo_id"),
            "label": meta.get("label_for_ui", name),
            "controller_mapping": meta.get("controller_mapping", {}),
            "observed": debug.get("actual_joint"),
            "commanded": debug.get("commanded_joint"),
            "desired": debug.get("desired_joint"),
            "filtered": debug.get("filtered_joint"),
            "raw_step": debug.get("raw_step"),
            "torque_enabled": debug.get("torque_enabled"),
            "truth_confidence": debug.get("truth_confidence"),
            "active_source": debug.get("active_source"),
            "tracking_gap": bool(debug.get("target_mismatch")),
            "unreadable": bool(debug.get("unreadable")),
            "rate_limited": bool(debug.get("rate_limited")),
            "clamped_by_limit": bool(debug.get("clamped_by_limit")),
            "last_error": debug.get("last_error"),
            "limits": limit,
        }
        report["joints"][name] = entry
        if entry["tracking_gap"]:
            report["summary"]["tracking_gaps"] += 1
        if entry["unreadable"]:
            report["summary"]["unreadable"] += 1
        if entry["rate_limited"]:
            report["summary"]["rate_limited"] += 1
        if entry["clamped_by_limit"]:
            report["summary"]["limit_clamped"] += 1

    return report
