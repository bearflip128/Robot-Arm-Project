from __future__ import annotations

import argparse
import json
from typing import Any

from scservo_sdk import COMM_SUCCESS, SMS_STS_ID, PortHandler, sms_sts


def open_bus(port_name: str, baud_rate: int) -> tuple[PortHandler, sms_sts]:
    port = PortHandler(port_name)
    if not port.openPort():
        raise RuntimeError(f"Unable to open port {port_name}.")
    if not port.setBaudRate(baud_rate):
        port.closePort()
        raise RuntimeError(f"Unable to set baud rate {baud_rate} on {port_name}.")
    return port, sms_sts(port)


def close_bus(port: PortHandler) -> None:
    try:
        port.closePort()
    except Exception:
        pass


def ping(packet: sms_sts, servo_id: int) -> dict[str, Any]:
    model, result, error = packet.ping(servo_id)
    return {
        "id": servo_id,
        "model": model,
        "result": result,
        "error": error,
        "responded": result == COMM_SUCCESS,
    }


def read_pos(packet: sms_sts, servo_id: int) -> dict[str, Any]:
    position, result, error = packet.ReadPos(servo_id)
    return {
        "position": position,
        "result": result,
        "error": error,
        "responded": result == COMM_SUCCESS,
    }


def cmd_scan(args: argparse.Namespace) -> None:
    port, packet = open_bus(args.port, args.baud)
    try:
        results = []
        for servo_id in range(args.start_id, args.end_id + 1):
            entry = ping(packet, servo_id)
            if entry["responded"]:
                entry["position"] = read_pos(packet, servo_id)
            results.append(entry)
        print(json.dumps(results, indent=2))
    finally:
        close_bus(port)


def cmd_set_id(args: argparse.Namespace) -> None:
    port, packet = open_bus(args.port, args.baud)
    try:
        before = ping(packet, args.current_id)
        if not before["responded"]:
            raise RuntimeError(f"Servo {args.current_id} did not respond before ID change.")

        unlock_result, unlock_error = packet.unLockEprom(args.current_id)
        if unlock_result != COMM_SUCCESS or unlock_error != 0:
            raise RuntimeError(
                f"Failed to unlock EPROM for servo {args.current_id}: result={unlock_result}, error={unlock_error}"
            )

        write_result, write_error = packet.write1ByteTxRx(
            args.current_id, SMS_STS_ID, args.new_id
        )
        if write_result != COMM_SUCCESS or write_error != 0:
            raise RuntimeError(
                f"Failed to write new ID {args.new_id}: result={write_result}, error={write_error}"
            )

        lock_result, lock_error = packet.LockEprom(args.new_id)
        if lock_result != COMM_SUCCESS or lock_error != 0:
            raise RuntimeError(
                f"Failed to lock EPROM for new servo ID {args.new_id}: result={lock_result}, error={lock_error}"
            )

        verify_old = ping(packet, args.current_id)
        verify_new = ping(packet, args.new_id)
        print(
            json.dumps(
                {
                    "before": before,
                    "verify_old": verify_old,
                    "verify_new": verify_new,
                },
                indent=2,
            )
        )
    finally:
        close_bus(port)


def cmd_probe(args: argparse.Namespace) -> None:
    port, packet = open_bus(args.port, args.baud)
    try:
        before = ping(packet, args.servo_id)
        if not before["responded"]:
            raise RuntimeError(f"Servo {args.servo_id} did not respond before probe.")

        before_pos = read_pos(packet, args.servo_id)
        if not before_pos["responded"]:
            raise RuntimeError(f"Servo {args.servo_id} position could not be read.")

        start_pos = int(before_pos["position"])
        target_pos = max(args.min_step, min(args.max_step, start_pos + args.delta))

        write_result, write_error = packet.WritePosEx(
            args.servo_id,
            target_pos,
            args.speed,
            args.acc,
        )

        after_move = None
        after_return = None

        if write_result == COMM_SUCCESS and write_error == 0:
            import time

            time.sleep(args.hold_ms / 1000)
            after_move = read_pos(packet, args.servo_id)

            if args.return_to_start:
                packet.WritePosEx(
                    args.servo_id,
                    start_pos,
                    args.speed,
                    args.acc,
                )
                time.sleep(args.return_hold_ms / 1000)
                after_return = read_pos(packet, args.servo_id)

        print(
            json.dumps(
                {
                    "servo_id": args.servo_id,
                    "before": before,
                    "before_pos": before_pos,
                    "target_pos": target_pos,
                    "write_result": write_result,
                    "write_error": write_error,
                    "after_move": after_move,
                    "after_return": after_return,
                },
                indent=2,
            )
        )
    finally:
        close_bus(port)


def cmd_move(args: argparse.Namespace) -> None:
    port, packet = open_bus(args.port, args.baud)
    try:
        before = ping(packet, args.servo_id)
        if not before["responded"]:
            raise RuntimeError(f"Servo {args.servo_id} did not respond before move.")

        before_pos = read_pos(packet, args.servo_id)
        if not before_pos["responded"]:
            raise RuntimeError(f"Servo {args.servo_id} position could not be read.")

        target_pos = max(args.min_step, min(args.max_step, args.target))
        write_result, write_error = packet.WritePosEx(
            args.servo_id,
            target_pos,
            args.speed,
            args.acc,
        )

        after_move = None
        if write_result == COMM_SUCCESS and write_error == 0:
            import time

            time.sleep(args.hold_ms / 1000)
            after_move = read_pos(packet, args.servo_id)

        print(
            json.dumps(
                {
                    "servo_id": args.servo_id,
                    "before": before,
                    "before_pos": before_pos,
                    "target_pos": target_pos,
                    "write_result": write_result,
                    "write_error": write_error,
                    "after_move": after_move,
                },
                indent=2,
            )
        )
    finally:
        close_bus(port)


def cmd_torque(args: argparse.Namespace) -> None:
    port, packet = open_bus(args.port, args.baud)
    try:
        from scservo_sdk import COMM_SUCCESS, SMS_STS_TORQUE_ENABLE

        before = ping(packet, args.servo_id)
        if not before["responded"]:
            raise RuntimeError(f"Servo {args.servo_id} did not respond before torque change.")

        result, error = packet.write1ByteTxRx(
            args.servo_id,
            SMS_STS_TORQUE_ENABLE,
            1 if args.enable else 0,
        )
        after_pos = read_pos(packet, args.servo_id)
        print(
            json.dumps(
                {
                    "servo_id": args.servo_id,
                    "action": "enable" if args.enable else "disable",
                    "before": before,
                    "result": result,
                    "error": error,
                    "ok": result == COMM_SUCCESS and error == 0,
                    "after_pos": after_pos,
                },
                indent=2,
            )
        )
    finally:
        close_bus(port)


def cmd_read(args: argparse.Namespace) -> None:
    port, packet = open_bus(args.port, args.baud)
    try:
        status = ping(packet, args.servo_id)
        position = read_pos(packet, args.servo_id)
        print(
            json.dumps(
                {
                    "servo_id": args.servo_id,
                    "status": status,
                    "position": position,
                },
                indent=2,
            )
        )
    finally:
        close_bus(port)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SCServo bus scan and ID management tool.")
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--baud", type=int, default=1_000_000)
    subparsers = parser.add_subparsers(dest="command", required=True)

    scan_parser = subparsers.add_parser("scan", help="Scan a range of servo IDs.")
    scan_parser.add_argument("--start-id", type=int, default=1)
    scan_parser.add_argument("--end-id", type=int, default=10)
    scan_parser.set_defaults(func=cmd_scan)

    set_id_parser = subparsers.add_parser("set-id", help="Persistently change a servo ID.")
    set_id_parser.add_argument("--current-id", type=int, required=True)
    set_id_parser.add_argument("--new-id", type=int, required=True)
    set_id_parser.set_defaults(func=cmd_set_id)

    probe_parser = subparsers.add_parser(
        "probe",
        help="Nudge one servo by a small delta for physical identification and readback validation.",
    )
    probe_parser.add_argument("--servo-id", type=int, required=True)
    probe_parser.add_argument("--delta", type=int, default=120)
    probe_parser.add_argument("--speed", type=int, default=120)
    probe_parser.add_argument("--acc", type=int, default=18)
    probe_parser.add_argument("--hold-ms", type=int, default=700)
    probe_parser.add_argument("--return-hold-ms", type=int, default=500)
    probe_parser.add_argument("--min-step", type=int, default=0)
    probe_parser.add_argument("--max-step", type=int, default=4095)
    probe_parser.add_argument("--return-to-start", action="store_true")
    probe_parser.set_defaults(func=cmd_probe)

    move_parser = subparsers.add_parser(
        "move",
        help="Move one servo to an absolute step target and read back the settled position.",
    )
    move_parser.add_argument("--servo-id", type=int, required=True)
    move_parser.add_argument("--target", type=int, required=True)
    move_parser.add_argument("--speed", type=int, default=120)
    move_parser.add_argument("--acc", type=int, default=18)
    move_parser.add_argument("--hold-ms", type=int, default=900)
    move_parser.add_argument("--min-step", type=int, default=0)
    move_parser.add_argument("--max-step", type=int, default=4095)
    move_parser.set_defaults(func=cmd_move)

    torque_parser = subparsers.add_parser(
        "torque",
        help="Enable or disable torque on one servo.",
    )
    torque_parser.add_argument("--servo-id", type=int, required=True)
    torque_group = torque_parser.add_mutually_exclusive_group(required=True)
    torque_group.add_argument("--enable", action="store_true")
    torque_group.add_argument("--disable", action="store_true")
    torque_parser.set_defaults(func=cmd_torque)

    read_parser = subparsers.add_parser(
        "read",
        help="Read one servo's current position.",
    )
    read_parser.add_argument("--servo-id", type=int, required=True)
    read_parser.set_defaults(func=cmd_read)

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
