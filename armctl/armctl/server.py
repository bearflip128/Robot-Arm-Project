"""HTTP layer.

Not one handler touches the serial port. Reads return the latest telemetry
snapshot the control loop published; writes drop a value into the control
loop's mailbox. Both are O(1) and lock-free enough that a slider drag can
never stall the arm, and the arm can never stall a slider drag.
"""

from __future__ import annotations

import json
import logging
import re
import threading
import time
from functools import wraps
from pathlib import Path

from flask import Flask, jsonify, redirect, render_template, request, session, url_for

from .config import Settings
from .controller import Controller
from .inputs import PadSnapshot

log = logging.getLogger("armctl.server")
SAFE_NAME = re.compile(r"[^a-zA-Z0-9 _-]+")

# A joint swept less than this clearly was not moved on purpose, so its
# captured span is noise and must not overwrite a good calibration.
MIN_SWEEP_STEPS = 120

# Upper sanity bound on a captured span, as a fraction of the 4096-step
# encoder. No joint on this arm travels anywhere near a full revolution, so a
# span this wide means the encoder wrapped during the sweep and the reading
# jumped 4095 -> 0. Writing that would remap every logical degree onto roughly
# twice the steps it should, which is how a "successful" calibration ends up
# driving joints into their stops.
MAX_SWEEP_FRACTION = 0.85
ENCODER_STEPS = 4096


def write_calibration(config_path: Path, spans: dict[str, list[int]]) -> dict[str, list[int]]:
    """Rewrite step_lo / step_hi in place, preserving comments and layout.

    Edited textually rather than via a YAML round-trip because PyYAML discards
    every comment in the file, and this config is mostly comments explaining
    why the numbers are what they are.
    """
    text = config_path.read_text(encoding="utf-8")
    config_path.with_suffix(config_path.suffix + ".bak").write_text(text, encoding="utf-8")

    written: dict[str, list[int]] = {}
    for joint, (low, high) in spans.items():
        for key, value in (("step_lo", int(low)), ("step_hi", int(high))):
            pattern = re.compile(rf"(^  {re.escape(joint)}:\n(?:.*\n)*?    {key}: )-?\d+", re.M)
            text, count = pattern.subn(rf"\g<1>{value}", text, count=1)
            if not count:
                raise ValueError(f"could not find {joint}.{key} in the config")
        written[joint] = [int(low), int(high)]

    config_path.write_text(text, encoding="utf-8")
    log.info("Calibration written for %s", ", ".join(sorted(written)))
    return written


def create_app(settings: Settings, controller: Controller, data_dir: Path,
               pad_service: object | None = None) -> Flask:
    app = Flask(__name__, template_folder="web", static_folder="web/static")
    app.secret_key = settings.secret_key
    poses_dir = data_dir / "poses"
    poses_dir.mkdir(parents=True, exist_ok=True)
    player = Player(controller)

    @app.context_processor
    def asset_version():
        """Stamp CSS/JS URLs with the newest asset mtime.

        Without this the browser keeps serving a cached app.js after a deploy,
        and the dashboard silently runs old code against a new API.
        """
        static = Path(app.static_folder)
        newest = max((p.stat().st_mtime_ns for p in static.glob("*")), default=0)
        return {"asset_v": newest}

    def login_required(view):
        @wraps(view)
        def wrapped(*args, **kwargs):
            if session.get("user") != settings.username:
                if request.path.startswith("/api/"):
                    return jsonify({"ok": False, "error": "Not signed in."}), 401
                return redirect(url_for("login"))
            return view(*args, **kwargs)

        return wrapped

    def fail(message: str, code: int = 400):
        return jsonify({"ok": False, "error": message}), code

    def ok(**extra):
        return jsonify({"ok": True, "telemetry": controller.snapshot(), **extra})

    def require_motion():
        snapshot = controller.snapshot()
        if snapshot.get("estopped"):
            return fail("Emergency stop is active.", 423)
        if not snapshot.get("armed"):
            return fail("Arm the robot before commanding motion.", 423)
        return None

    # -- auth --------------------------------------------------------------

    @app.route("/login", methods=["GET", "POST"])
    def login():
        if request.method == "POST":
            if (request.form.get("username") == settings.username
                    and request.form.get("password") == settings.password):
                session["user"] = settings.username
                return redirect(url_for("dashboard"))
            return render_template("login.html", error="Incorrect username or password."), 401
        return render_template("login.html", error=None)

    @app.post("/logout")
    def logout():
        session.clear()
        return redirect(url_for("login"))

    # -- pages -------------------------------------------------------------

    @app.get("/")
    @login_required
    def dashboard():
        return render_template("index.html", joints=list(settings.specs.values()))

    @app.get("/healthz")
    def healthz():
        snapshot = controller.snapshot()
        return jsonify({
            "ok": bool(snapshot.get("connected")),
            "armed": snapshot.get("armed"),
            "loop": snapshot.get("loop"),
        })

    # -- telemetry ---------------------------------------------------------

    @app.get("/api/telemetry")
    @login_required
    def telemetry():
        return jsonify(controller.snapshot())

    # -- motion ------------------------------------------------------------

    @app.post("/api/goal")
    @login_required
    def goal():
        blocked = require_motion()
        if blocked:
            return blocked
        payload = request.get_json(silent=True) or {}
        targets = payload.get("joints")
        if not isinstance(targets, dict) or not targets:
            return fail("Expected a joints object.")
        try:
            numeric = {str(k): float(v) for k, v in targets.items()}
        except (TypeError, ValueError):
            return fail("Joint targets must be numbers.")
        player.cancel()
        return ok(accepted=controller.set_goal(numeric, source=str(payload.get("source") or "ui")))

    @app.post("/api/pad")
    @login_required
    def pad():
        blocked = require_motion()
        if blocked:
            return blocked
        payload = request.get_json(silent=True) or {}
        controller.set_pad(PadSnapshot.parse(payload), source=str(payload.get("source") or "pad"))
        return jsonify({"ok": True})

    @app.post("/api/home")
    @login_required
    def home():
        blocked = require_motion()
        if blocked:
            return blocked
        player.cancel()
        return ok(accepted=controller.set_goal(
            {name: spec.home for name, spec in settings.specs.items()}, source="home"
        ))

    # -- safety ------------------------------------------------------------

    @app.post("/api/arm")
    @login_required
    def arm():
        try:
            controller.submit(lambda c: c.do_arm())
        except Exception as exc:
            return fail(str(exc), 409)
        return ok()

    @app.post("/api/disarm")
    @login_required
    def disarm():
        player.cancel()
        controller.submit(lambda c: c.do_disarm())
        return ok()

    @app.post("/api/estop")
    @login_required
    def estop():
        player.cancel()
        controller.submit(lambda c: c.do_estop())
        return ok()

    @app.post("/api/reset")
    @login_required
    def reset():
        controller.submit(lambda c: c.do_reset())
        return ok()

    @app.post("/api/relax")
    @login_required
    def relax():
        player.cancel()
        try:
            controller.submit(lambda c: c.do_relax())
        except Exception as exc:
            return fail(str(exc), 409)
        return ok()

    @app.post("/api/recover")
    @login_required
    def recover():
        """Bring joints released after a stall back under torque."""
        try:
            controller.submit(lambda c: c.do_recover())
        except Exception as exc:
            return fail(str(exc), 409)
        return ok()

    @app.get("/api/pad")
    @login_required
    def pad_status():
        """Live state of the USB pad wired to the host running this server."""
        # NB: the POST view below is also named `pad`, which shadows a
        # parameter of that name inside this closure. Hence `pad_service`.
        if pad_service is None:
            return jsonify({"ok": True, "available": False,
                            "reason": "Wired pad reader is disabled (--no-pad)."})
        return jsonify({"ok": True, "available": True, **pad_service.status()})

    # -- calibration -------------------------------------------------------

    @app.post("/api/calibrate/start")
    @login_required
    def calibrate_start():
        player.cancel()
        try:
            controller.submit(lambda c: c.do_calibrate_start())
        except Exception as exc:
            return fail(str(exc), 409)
        return ok()

    @app.post("/api/calibrate/save")
    @login_required
    def calibrate_save():
        """Write the captured travel back into the config file.

        The previous config is kept alongside as .bak so a bad sweep is one
        file copy away from being undone.
        """
        captured = controller.submit(lambda c: c.do_calibrate_stop())

        usable, too_small, implausible = {}, [], []
        for name, span in captured.items():
            if not span:
                continue
            width = span[1] - span[0]
            if width >= ENCODER_STEPS * MAX_SWEEP_FRACTION:
                implausible.append(name)
            elif width < MIN_SWEEP_STEPS:
                too_small.append(name)
            else:
                usable[name] = span

        if implausible:
            names = ", ".join(sorted(implausible))
            return fail(
                f"Refusing to save: {names} recorded almost the entire encoder range. "
                "That is not noise — it means the joint's travel straddles the encoder "
                "rollover, so it genuinely does sweep every step number, and no single "
                "step range can describe it. Re-home the servo first so the rollover "
                "sits in the arc the joint never reaches:\n"
                "  python tools/set_home_offset.py COM4 <servo id> --offset <shift>\n"
                "Capture the travel first to find the shift — the unvisited arc's "
                "midpoint is the value you want. Then re-run this calibration.")
        if not usable:
            return fail(
                "No joint was moved far enough to calibrate. Sweep each joint "
                f"through at least {MIN_SWEEP_STEPS} steps of travel."
                + (f" Too small: {', '.join(sorted(too_small))}." if too_small else ""))
        try:
            written = write_calibration(settings.config_path, usable)
        except Exception as exc:
            return fail(f"Could not write calibration: {exc}", 500)
        return ok(calibrated=written, restart_required=True)

    @app.post("/api/calibrate/cancel")
    @login_required
    def calibrate_cancel():
        controller.submit(lambda c: c.do_calibrate_stop())
        return ok()

    # -- poses -------------------------------------------------------------

    def pose_path(name: str) -> Path:
        safe = SAFE_NAME.sub("", str(name)).strip()
        if not safe:
            raise ValueError("Pose name must contain letters or numbers.")
        return poses_dir / f"{safe}.json"

    @app.get("/api/poses")
    @login_required
    def list_poses():
        poses = sorted(p.stem for p in poses_dir.glob("*.json"))
        return jsonify({"ok": True, "poses": poses})

    @app.post("/api/poses")
    @login_required
    def save_pose():
        payload = request.get_json(silent=True) or {}
        try:
            path = pose_path(payload.get("name", ""))
        except ValueError as exc:
            return fail(str(exc))
        snapshot = controller.snapshot()
        joints = {
            j["name"]: j["observed"] if j["observed"] is not None else j["setpoint"]
            for j in snapshot.get("joints", [])
        }
        path.write_text(json.dumps({"name": path.stem, "joints": joints}, indent=2), encoding="utf-8")
        return ok(pose=path.stem)

    @app.post("/api/poses/<name>/go")
    @login_required
    def go_pose(name: str):
        blocked = require_motion()
        if blocked:
            return blocked
        try:
            path = pose_path(name)
        except ValueError as exc:
            return fail(str(exc))
        if not path.exists():
            return fail(f"No pose named {name}.", 404)
        joints = json.loads(path.read_text(encoding="utf-8")).get("joints", {})
        player.cancel()
        return ok(accepted=controller.set_goal(joints, source="pose"))

    @app.delete("/api/poses/<name>")
    @login_required
    def delete_pose(name: str):
        try:
            path = pose_path(name)
        except ValueError as exc:
            return fail(str(exc))
        path.unlink(missing_ok=True)
        return jsonify({"ok": True})

    # -- teach and repeat --------------------------------------------------

    @app.post("/api/record/start")
    @login_required
    def record_start():
        try:
            controller.submit(lambda c: c.do_relax())
        except Exception as exc:
            return fail(str(exc), 409)
        player.start_recording()
        return ok()

    @app.post("/api/record/stop")
    @login_required
    def record_stop():
        return ok(frames=player.stop_recording())

    @app.post("/api/play")
    @login_required
    def play():
        blocked = require_motion()
        if blocked:
            return blocked
        if not player.frames:
            return fail("Nothing recorded yet.")
        player.play()
        return ok(frames=len(player.frames))

    @app.post("/api/play/stop")
    @login_required
    def play_stop():
        player.cancel()
        return ok()

    @app.get("/api/record")
    @login_required
    def record_state():
        return jsonify({"ok": True, **player.state()})

    return app


class Player:
    """Teach-and-repeat.

    Recording samples the telemetry snapshot the control loop already
    publishes, so it adds zero serial traffic. Playback streams goals into the
    same trajectory generator every other input uses.
    """

    MIN_DELTA = 0.25
    SAMPLE_S = 0.02

    def __init__(self, controller: Controller) -> None:
        self._controller = controller
        self.frames: list[tuple[float, dict[str, float]]] = []
        self._recording = False
        self._playing = False
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def state(self) -> dict:
        return {"recording": self._recording, "playing": self._playing, "frames": len(self.frames)}

    def start_recording(self) -> None:
        self.cancel()
        self.frames = []
        self._recording = True
        self._stop.clear()
        self._thread = threading.Thread(target=self._record_loop, daemon=True, name="armctl-record")
        self._thread.start()

    def stop_recording(self) -> int:
        self._recording = False
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        return len(self.frames)

    def play(self) -> None:
        self.cancel()
        self._playing = True
        self._stop.clear()
        self._thread = threading.Thread(target=self._play_loop, daemon=True, name="armctl-play")
        self._thread.start()

    def cancel(self) -> None:
        if not (self._recording or self._playing):
            return
        self._recording = self._playing = False
        self._stop.set()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)

    def _record_loop(self) -> None:
        started = time.perf_counter()
        while not self._stop.is_set():
            joints = {
                j["name"]: j["observed"]
                for j in self._controller.snapshot().get("joints", [])
                if j["observed"] is not None
            }
            if joints and (not self.frames or _moved(self.frames[-1][1], joints, self.MIN_DELTA)):
                self.frames.append((time.perf_counter() - started, joints))
            self._stop.wait(self.SAMPLE_S)
        self._recording = False

    def _play_loop(self) -> None:
        started = time.perf_counter()
        for stamp, joints in self.frames:
            if self._stop.is_set():
                break
            wait = stamp - (time.perf_counter() - started)
            if wait > 0:
                self._stop.wait(wait)
            self._controller.set_goal(joints, source="playback")
        self._playing = False


def _moved(previous: dict[str, float], current: dict[str, float], threshold: float) -> bool:
    return any(abs(value - previous.get(name, value)) >= threshold for name, value in current.items())
