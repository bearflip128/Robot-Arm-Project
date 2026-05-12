from __future__ import annotations

import json
import time

from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts


PORT = "COM4"
BAUD = 1_000_000

# Hardcoded from the freshly calibrated safe ranges.
WAKE_SERVOS = {
    2: {"min": 729, "mid": 1895, "speed": 90, "acc": 18},
    3: {"min": 808, "mid": 1693, "speed": 85, "acc": 16},
    4: {"min": 456, "mid": 1106, "speed": 80, "acc": 14},
    6: {"min": 1258, "mid": 2510, "speed": 90, "acc": 16},
}

STEPS_TO_MIN = 6
STEPS_TO_MID = 10
STEP_HOLD_S = 0.65


def read_pos(packet: sms_sts, servo_id: int) -> dict:
    position, result, error = packet.ReadPos(servo_id)
    return {
        "position": int(position),
        "result": int(result),
        "error": int(error),
        "ok": result == COMM_SUCCESS and error == 0,
    }


def write_target(packet: sms_sts, servo_id: int, target: int, speed: int, acc: int) -> dict:
    result, error = packet.WritePosEx(servo_id, int(target), int(speed), int(acc))
    return {
        "result": int(result),
        "error": int(error),
        "ok": result == COMM_SUCCESS and error == 0,
    }


def ramp_targets(start: int, end: int, steps: int) -> list[int]:
    if steps <= 1:
        return [int(end)]
    return [
        int(round(start + (end - start) * (idx / (steps - 1))))
        for idx in range(1, steps)
    ] + [int(end)]


def main() -> None:
    port = PortHandler(PORT)
    if not port.openPort():
        raise RuntimeError(f"Unable to open {PORT}.")
    if not port.setBaudRate(BAUD):
        port.closePort()
        raise RuntimeError(f"Unable to set baud rate {BAUD} on {PORT}.")

    packet = sms_sts(port)
    report: dict[str, object] = {
        "port": PORT,
        "baud": BAUD,
        "servos": {},
        "started_at": time.time(),
    }

    try:
        for servo_id, cfg in WAKE_SERVOS.items():
            start = read_pos(packet, servo_id)
            if not start["ok"]:
                report["servos"][str(servo_id)] = {
                    "start": start,
                    "error": "read_failed_before_start",
                }
                continue

            servo_report = {
                "start": start,
                "sleep_path": [],
                "wake_path": [],
            }

            for target in ramp_targets(start["position"], cfg["min"], STEPS_TO_MIN):
                write = write_target(packet, servo_id, target, cfg["speed"], cfg["acc"])
                time.sleep(STEP_HOLD_S)
                after = read_pos(packet, servo_id)
                servo_report["sleep_path"].append(
                    {"target": target, "write": write, "after": after}
                )

            for target in ramp_targets(cfg["min"], cfg["mid"], STEPS_TO_MID):
                write = write_target(packet, servo_id, target, cfg["speed"], cfg["acc"])
                time.sleep(STEP_HOLD_S)
                after = read_pos(packet, servo_id)
                servo_report["wake_path"].append(
                    {"target": target, "write": write, "after": after}
                )

            servo_report["final"] = read_pos(packet, servo_id)
            report["servos"][str(servo_id)] = servo_report

        report["finished_at"] = time.time()
        print(json.dumps(report, indent=2))
    finally:
        port.closePort()


if __name__ == "__main__":
    main()
