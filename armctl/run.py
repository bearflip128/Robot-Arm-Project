"""Entry point.

    python run.py                 # serve with the real arm on COM4
    python run.py --sim           # simulated arm, no hardware needed
    python run.py --dev           # Flask dev server instead of waitress
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from armctl import config
from armctl.bus import FeetechBus, SimBus
from armctl.controller import Controller
from armctl.pad_service import PadService
from armctl.server import create_app

ROOT = Path(__file__).parent


def main() -> int:
    parser = argparse.ArgumentParser(description="armctl robot arm server")
    parser.add_argument("--config", default=str(ROOT / "config" / "arm.yaml"))
    parser.add_argument("--sim", action="store_true", help="run against a simulated arm")
    parser.add_argument("--dev", action="store_true", help="use the Flask dev server")
    parser.add_argument("--no-pad", action="store_true", help="skip the wired gamepad reader")
    parser.add_argument("--port", type=int, default=None)
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s  %(message)s",
        datefmt="%H:%M:%S",
    )
    log = logging.getLogger("armctl")

    settings = config.load(args.config)
    port = args.port or settings.port
    simulated = args.sim or settings.driver == "sim"

    if simulated:
        bus = SimBus(settings.specs)
        log.info("Simulator mode — no serial hardware will be touched.")
    else:
        bus = FeetechBus(
            settings.specs,
            port=settings.serial_port,
            baud=settings.baud,
            latency_ms=settings.latency_ms,
            goal_speed=settings.goal_speed,
            goal_accel=settings.goal_accel,
            nominal_volts=settings.nominal_volts,
            undervolt_warn=settings.undervolt_warn,
        )

    controller = Controller(
        settings.specs,
        bus,
        control_hz=settings.control_hz,
        intent_timeout=settings.intent_timeout,
    )
    controller.start()

    pad = None
    if not args.no_pad:
        pad = PadService(controller, layout=settings.gamepad)
        pad.start()

    app = create_app(settings, controller, ROOT / "data", pad_service=pad)

    log.info("Dashboard on http://%s:%s  (control loop %.0f Hz)",
             settings.host, port, settings.control_hz)
    try:
        if args.dev:
            app.run(host=settings.host, port=port, threaded=True, use_reloader=False)
        else:
            from waitress import serve

            serve(app, host=settings.host, port=port, threads=8)
    except KeyboardInterrupt:
        log.info("Shutting down.")
    finally:
        if pad:
            pad.stop()
        controller.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
