from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml


BASE_DIR = Path(__file__).resolve().parent.parent
LOG_DIR = BASE_DIR / "calibration_logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)


@dataclass
class CalibrationSession:
    config_path: Path
    log_path: Path

    def log(self, event: str, **payload: Any) -> None:
        record = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "event": event,
            **payload,
        }
        with self.log_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record) + "\n")


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def create_session(config_path: Path, label: str | None = None) -> CalibrationSession:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    name = f"{stamp}-{label}" if label else stamp
    log_path = LOG_DIR / f"{name}.jsonl"
    session = CalibrationSession(config_path=config_path, log_path=log_path)
    session.log("session_started", config_path=str(config_path))
    return session


def command_snapshot(args: argparse.Namespace) -> None:
    session = create_session(Path(args.config), args.label)
    config = load_config(Path(args.config))
    session.log("config_snapshot", config=config)
    print(
        json.dumps(
            {
                "ok": True,
                "log_path": str(session.log_path),
                "config_path": str(Path(args.config)),
            },
            indent=2,
        )
    )


def command_note(args: argparse.Namespace) -> None:
    session = CalibrationSession(config_path=Path(args.config), log_path=Path(args.log))
    session.log(
        "operator_note",
        servo_id=args.servo_id,
        joint_name=args.joint_name,
        note=args.note,
        confidence=args.confidence,
    )
    print(json.dumps({"ok": True, "log_path": str(session.log_path)}, indent=2))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Calibration log/session helper.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    snapshot = subparsers.add_parser("snapshot", help="Create a calibration session and snapshot the config.")
    snapshot.add_argument("--config", default=str(BASE_DIR / "config" / "config.yaml"))
    snapshot.add_argument("--label", default=None)
    snapshot.set_defaults(func=command_snapshot)

    note = subparsers.add_parser("note", help="Append an operator note to an existing calibration log.")
    note.add_argument("--config", default=str(BASE_DIR / "config" / "config.yaml"))
    note.add_argument("--log", required=True)
    note.add_argument("--servo-id", type=int, required=True)
    note.add_argument("--joint-name", required=True)
    note.add_argument("--confidence", choices=["high", "medium", "low"], default="low")
    note.add_argument("--note", required=True)
    note.set_defaults(func=command_note)

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
