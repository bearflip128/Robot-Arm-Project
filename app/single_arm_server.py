from __future__ import annotations

import json
import os
import sys
import threading
import time
from functools import wraps
from pathlib import Path
from typing import Any, Dict

from flask import (
    Flask,
    jsonify,
    redirect,
    render_template_string,
    request,
    session,
    url_for,
)
from dotenv import load_dotenv

BASE_DIR = Path(__file__).resolve().parent.parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

load_dotenv(BASE_DIR / ".env")

from app.robot_arm import (
    build_adapter,
    build_joint_definitions,
    default_joint_positions,
    load_config,
)
from app.control import (
    PS4InputAdapter,
    PlaybackInputAdapter,
    JointStateStore,
    MotionFilter,
    PS4ControllerMapper,
    RobotControlRuntime,
    UIInputAdapter,
    build_joint_runtime_configs,
    build_verification_report,
)
from app.control.controller_mapping import PS4Snapshot


DEFAULT_CONFIG = (
    BASE_DIR / "config" / "config.yaml"
    if (BASE_DIR / "config" / "config.yaml").exists()
    else BASE_DIR / "config" / "config.example.yaml"
)
POSES_DIR = BASE_DIR / "data" / "poses"
SEQUENCES_DIR = BASE_DIR / "data" / "sequences"
POSES_DIR.mkdir(parents=True, exist_ok=True)
SEQUENCES_DIR.mkdir(parents=True, exist_ok=True)

app = Flask(__name__)

config_path = Path(os.environ.get("QUBIT_CONFIG", DEFAULT_CONFIG))
config = load_config(config_path)
joint_definitions = build_joint_definitions(config)
joint_runtime_configs = build_joint_runtime_configs(config, joint_definitions)
adapter = build_adapter(config)
state_store = JointStateStore(joint_runtime_configs)
motion_filter = MotionFilter(joint_runtime_configs)
ps4_mapper = PS4ControllerMapper(joint_runtime_configs)
ui_input_adapter = UIInputAdapter()
ps4_input_adapter = PS4InputAdapter(ps4_mapper)
playback_input_adapter = PlaybackInputAdapter()
control_runtime = RobotControlRuntime(
    adapter=adapter,
    joint_definitions=joint_definitions,
    joint_configs=joint_runtime_configs,
    state_store=state_store,
    motion_filter=motion_filter,
    config=config,
    loop_hz=40.0,
)
control_runtime.start()
app.secret_key = os.environ.get(
    "ROBOT_ARM_SECRET_KEY",
    config.get("auth", {}).get("secret_key", "change-me-session-secret"),
)
API_BEARER_TOKEN = os.environ.get("ROBOT_API_KEY", "").strip()

AUTH_USERNAME = os.environ.get(
    "ROBOT_ARM_USERNAME", config.get("auth", {}).get("username", "admin")
)
AUTH_PASSWORD = os.environ.get(
    "ROBOT_ARM_PASSWORD", config.get("auth", {}).get("password", "change-me-robot")
)

RECORDING_SAMPLE_S = 0.025
RECORDING_EPSILON = 0.25
RECORDING_STATE: Dict[str, Any] = {
    "active": False,
    "started_at": None,
    "frames": [],
    "duration_s": 0.0,
    "worker": None,
    "stop_event": None,
}
RECORDING_LOCK = threading.Lock()


def clamp_targets(raw_targets: Dict[str, float]) -> Dict[str, float]:
    return control_runtime.clamp_targets(raw_targets)


def desired_targets_snapshot() -> Dict[str, float]:
    return control_runtime.desired_targets_snapshot()


def set_desired_targets(targets: Dict[str, float], source: str = "system") -> Dict[str, float]:
    return control_runtime.set_desired_targets(targets, source=source)


def reset_desired_targets(targets: Dict[str, float] | None = None, source: str = "system") -> Dict[str, float]:
    return control_runtime.reset_desired_targets(targets, source=source)


def set_controller_intent(intents: Dict[str, float], source: str = "controller") -> Dict[str, float]:
    return control_runtime.set_controller_intent(intents, source=source)


def clear_controller_intent(source: str = "system") -> Dict[str, float]:
    return control_runtime.clear_controller_intent(source=source)


def controller_runtime_snapshot() -> Dict[str, Any]:
    return control_runtime.controller_runtime_snapshot()


def current_joint_source(name: str) -> str:
    return control_runtime.current_joint_source(name)


def _recording_snapshot() -> Dict[str, Any]:
    with RECORDING_LOCK:
        started_at = RECORDING_STATE["started_at"]
        duration_s = float(RECORDING_STATE.get("duration_s") or 0.0)
        if RECORDING_STATE["active"] and started_at is not None:
            duration_s = max(duration_s, time.perf_counter() - float(started_at))
        return {
            "active": bool(RECORDING_STATE["active"]),
            "has_recording": bool(RECORDING_STATE["frames"]),
            "frame_count": len(RECORDING_STATE["frames"]),
            "duration_s": round(duration_s, 3),
        }


def _recording_should_append(
    frames: list[Dict[str, Any]], joints: Dict[str, float], force: bool = False
) -> bool:
    if force or not frames:
        return True
    last_joints = frames[-1]["joints"]
    for joint_name, value in joints.items():
        if abs(float(value) - float(last_joints.get(joint_name, value))) >= RECORDING_EPSILON:
            return True
    return False


def _recording_append_sample(
    frames: list[Dict[str, Any]], started_at: float, joints: Dict[str, float], force: bool = False
) -> None:
    if not _recording_should_append(frames, joints, force=force):
        return
    frames.append(
        {
            "t": round(time.perf_counter() - started_at, 3),
            "joints": {name: float(value) for name, value in joints.items()},
        }
    )


def _recording_worker(stop_event: threading.Event) -> None:
    started_at = time.perf_counter()
    frames: list[Dict[str, Any]] = []
    while not stop_event.is_set():
        try:
            state = adapter.get_state()
            _recording_append_sample(frames, started_at, state["joints"], force=not frames)
        except Exception:
            pass
        stop_event.wait(RECORDING_SAMPLE_S)

    try:
        state = adapter.get_state()
        _recording_append_sample(frames, started_at, state["joints"], force=True)
    except Exception:
        pass

    with RECORDING_LOCK:
        RECORDING_STATE["active"] = False
        RECORDING_STATE["started_at"] = None
        RECORDING_STATE["frames"] = frames
        RECORDING_STATE["duration_s"] = frames[-1]["t"] if frames else 0.0
        RECORDING_STATE["worker"] = None
        RECORDING_STATE["stop_event"] = None


def _interpolate_recording_frames(
    frames: list[Dict[str, Any]], step_s: float = RECORDING_SAMPLE_S
) -> list[Dict[str, Any]]:
    if len(frames) < 2:
        return list(frames)

    normalized_frames = [
        {
            "t": float(frame.get("t", 0.0)),
            "joints": {name: float(value) for name, value in (frame.get("joints") or {}).items()},
        }
        for frame in frames
    ]
    normalized_frames.sort(key=lambda frame: frame["t"])

    step_s = max(0.01, float(step_s))
    resampled: list[Dict[str, Any]] = []
    target_t = normalized_frames[0]["t"]
    final_t = normalized_frames[-1]["t"]
    source_index = 0

    while target_t <= final_t + 1e-9:
        while (
            source_index < len(normalized_frames) - 2
            and normalized_frames[source_index + 1]["t"] < target_t
        ):
            source_index += 1

        left = normalized_frames[source_index]
        right = normalized_frames[min(source_index + 1, len(normalized_frames) - 1)]

        if right["t"] <= left["t"]:
            alpha = 0.0
        else:
            alpha = max(0.0, min(1.0, (target_t - left["t"]) / (right["t"] - left["t"])))

        joint_names = set(left["joints"]) | set(right["joints"])
        joints = {}
        for name in joint_names:
            left_value = float(left["joints"].get(name, right["joints"].get(name, 0.0)))
            right_value = float(right["joints"].get(name, left_value))
            joints[name] = left_value + (right_value - left_value) * alpha

        resampled.append({"t": round(target_t, 3), "joints": joints})
        target_t += step_s

    last_frame = normalized_frames[-1]
    if not resampled or abs(float(resampled[-1]["t"]) - float(last_frame["t"])) > 1e-6:
        resampled.append({"t": round(float(last_frame["t"]), 3), "joints": dict(last_frame["joints"])})
    return resampled


def pose_path(name: str) -> Path:
    safe_name = "".join(ch for ch in name.lower() if ch.isalnum() or ch in {"-", "_"})
    if not safe_name:
        raise ValueError("Pose name must include letters or numbers.")
    return POSES_DIR / f"{safe_name}.json"


def current_payload():
    state = safe_adapter_state()
    safety = control_runtime.safety_snapshot()
    diagnostics = {}
    try:
        diagnostics = adapter.get_diagnostics()
    except Exception:
        diagnostics = {}
    servo_map = config.get("robot", {}).get("servo_map", {})
    motion_policy = config.get("robot", {}).get("motion_policy", {})
    joint_meta = {}
    for name, definition in joint_definitions.items():
        mapping = servo_map.get(name, {})
        controller_mapping = mapping.get("controller_mapping", {})
        if isinstance(controller_mapping, str):
            controller_mapping = {"physical_ps4": controller_mapping}
        joint_meta[name] = {
            "joint_name": mapping.get("joint_name", name),
            "label_for_ui": mapping.get("label_for_ui", name.replace("_", " ").title()),
            "servo_id": mapping.get("servo_id"),
            "enabled": bool(mapping.get("enabled", True)),
            "neutral": float(mapping.get("neutral", definition.default)),
            "default_start": float(mapping.get("default_start", definition.default)),
            "inverted": bool(mapping.get("invert", False)),
            "semantic_negative_label": mapping.get("semantic_negative_label", "Min"),
            "semantic_positive_label": mapping.get("semantic_positive_label", "Max"),
            "controller_mapping": controller_mapping,
            "controller": mapping.get("controller", {}),
            "notes": mapping.get("notes", ""),
            "servo_min_step": mapping.get("servo_min_step", config["robot"].get("servo_min_step")),
            "servo_max_step": mapping.get("servo_max_step", config["robot"].get("servo_max_step")),
            "max_delta_per_command": mapping.get(
                "max_delta_per_command", motion_policy.get("default_max_delta_per_command")
            ),
            "max_degrees_per_second": mapping.get(
                "max_degrees_per_second", motion_policy.get("default_max_degrees_per_second")
            ),
            "controller_max_degrees_per_second": mapping.get(
                "controller_max_degrees_per_second",
                config.get("robot", {}).get("teleop", {}).get("default_controller_max_degrees_per_second"),
            ),
        }
    joint_debug = build_joint_debug(state, diagnostics, joint_meta)
    return {
        "auth": {
            "logged_in": bool(session.get("authenticated")),
            "username": AUTH_USERNAME,
        },
        "robot": {
            "name": config["robot"]["name"],
            "driver": config["robot"]["driver"],
            "connected": state["connected"],
            "hardware_error": state.get("hardware_error"),
        },
        "diagnostics": diagnostics,
        "safety": {
            "armed": safety.armed,
            "estopped": safety.estopped,
            "lock_reason": safety.lock_reason,
            "motion_allowed": safety.motion_allowed,
        },
        "joints": state["joints"],
        "actual_joints": state["joints"],
        "commanded_joints": state.get("commanded_joints", state["joints"]),
        "desired_joints": state_store.snapshot().desired,
        "joint_meta": joint_meta,
        "joint_debug": joint_debug,
        "limits": {
            name: {
                "min": definition.minimum,
                "max": definition.maximum,
                "default": definition.default,
                "servo_id": servo_map.get(name, {}).get("servo_id"),
            }
            for name, definition in joint_definitions.items()
        },
        "recording": _recording_snapshot(),
        "safe_poses": {
            "startup": config.get("robot", {}).get("safe_startup_pose", {}),
            "recovery": config.get("robot", {}).get("safe_recovery_pose", {}),
        },
        "control_runtime": {
            **control_runtime.loop_runtime_snapshot(),
        },
        "poses": sorted(path.stem for path in POSES_DIR.glob("*.json")),
        "sequences": sorted(path.stem for path in SEQUENCES_DIR.glob("*.json")),
        "verification": build_verification_report(
            state=state,
            diagnostics=diagnostics,
            joint_meta=joint_meta,
            joint_debug=joint_debug,
            limits={
                name: {
                    "min": definition.minimum,
                    "max": definition.maximum,
                    "default": definition.default,
                    "servo_id": servo_map.get(name, {}).get("servo_id"),
                }
                for name, definition in joint_definitions.items()
            },
        ),
    }


def build_joint_debug(state: Dict[str, Any], diagnostics: Dict[str, Any], joint_meta: Dict[str, Any]):
    servo_status = diagnostics.get("servo_status", {}) if isinstance(diagnostics, dict) else {}
    actual_joints = state.get("joints", {})
    commanded = state.get("commanded_joints", {})
    snapshot = state_store.snapshot()
    desired = snapshot.desired
    filtered = snapshot.filtered
    joint_debug: Dict[str, Dict[str, Any]] = {}
    for name, definition in joint_definitions.items():
        status = servo_status.get(name, {}) if isinstance(servo_status, dict) else {}
        actual_value = float(actual_joints.get(name, definition.default))
        commanded_value = float(commanded.get(name, actual_value))
        unreadable = bool(status.get("unreadable", False))
        last_error = status.get("last_error") or state.get("hardware_error")
        confidence = status.get("truth_confidence")
        if not confidence:
            if not state.get("connected"):
                confidence = "low"
            elif last_error:
                confidence = "medium"
            else:
                confidence = "high"
        joint_debug[name] = {
            "ui_supported": True,
            "actual_joint": actual_value,
            "desired_joint": float(desired.get(name, commanded_value)),
            "filtered_joint": float(filtered.get(name, commanded_value)),
            "commanded_joint": commanded_value,
            "last_sent_joint": status.get("last_sent_joint", commanded_value),
            "pending_joint": status.get("pending_joint"),
            "raw_step": status.get("raw_step"),
            "torque_enabled": status.get("torque_enabled"),
            "last_error": last_error,
            "active_source": status.get("active_source", "system"),
            "clamped_by_limit": bool(status.get("clamped_by_limit", False)),
            "rate_limited": bool(status.get("rate_limited", False)),
            "filtered_reasons": status.get("filtered_reasons", []),
            "unreadable": unreadable,
            "truth_confidence": confidence,
            "target_mismatch": abs(commanded_value - actual_value) >= 1.0,
            "servo_id": joint_meta.get(name, {}).get("servo_id"),
            "last_read_at": status.get("last_read_at"),
            "last_command_at": status.get("last_command_at"),
        }
    return joint_debug


def safe_adapter_state():
    return control_runtime.safe_adapter_state()


def error_response(message: str, status_code: int = 400):
    return jsonify({"ok": False, "error": message}), status_code


def is_authenticated() -> bool:
    return bool(session.get("authenticated"))


def has_valid_api_token() -> bool:
    if not API_BEARER_TOKEN:
        return False
    header = request.headers.get("Authorization", "")
    if not header.startswith("Bearer "):
        return False
    return header.removeprefix("Bearer ").strip() == API_BEARER_TOKEN


def motion_allowed() -> bool:
    return control_runtime.motion_allowed()


def require_login(view):
    @wraps(view)
    def wrapped(*args, **kwargs):
        if is_authenticated():
            return view(*args, **kwargs)
        if request.path.startswith("/api/"):
            return error_response("Login required.", 401)
        return redirect(url_for("login_page"))

    return wrapped


def require_api_access(view):
    @wraps(view)
    def wrapped(*args, **kwargs):
        if is_authenticated() or has_valid_api_token():
            return view(*args, **kwargs)
        return error_response("API authentication required.", 401)

    return wrapped


def require_motion_ready():
    safety = control_runtime.safety_snapshot()
    if safety.estopped:
        return error_response("Emergency stop is active. Reset before moving.", 423)
    if not safety.armed:
        return error_response("Controls are disarmed. Arm the system to move.", 423)
    return None


def sequence_path(name: str) -> Path:
    safe_name = "".join(ch for ch in name.lower() if ch.isalnum() or ch in {"-", "_"})
    if not safe_name:
        raise ValueError("Sequence name must include letters or numbers.")
    return SEQUENCES_DIR / f"{safe_name}.json"


def list_saved_pose_names():
    return sorted(path.stem for path in POSES_DIR.glob("*.json"))


def list_saved_sequence_names():
    return sorted(path.stem for path in SEQUENCES_DIR.glob("*.json"))


def load_pose_definition(name: str) -> dict:
    path = pose_path(name)
    if not path.exists():
        raise FileNotFoundError(f"Pose '{name}' does not exist.")
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def load_sequence_definition(name: str) -> dict:
    path = sequence_path(name)
    if not path.exists():
        raise FileNotFoundError(f"Sequence '{name}' does not exist.")
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def execute_pose_move(name: str):
    pose = load_pose_definition(name)
    try:
        targets = set_desired_targets(pose["joints"], source=f"pose:{name}")
        return targets
    except Exception as exc:
        raise RuntimeError(str(exc)) from exc


def execute_sequence_run(name: str):
    sequence = load_sequence_definition(name)
    poses = sequence.get("poses", [])
    if not isinstance(poses, list) or not poses:
        raise ValueError("Sequence must include at least one pose.")

    visited = []
    joints = adapter.get_state()["joints"]
    for pose_name in poses:
        joints = execute_pose_move(str(pose_name))
        visited.append(str(pose_name))

    return {
        "name": sequence.get("name", name),
        "poses": visited,
        "final_joints": joints,
    }


@app.get("/")
@require_login
def index():
    return render_template_string(HTML, robot_name=config["robot"]["name"])


@app.get("/api/health")
def health():
    return jsonify({"ok": True, "config_path": str(config_path)})


@app.get("/login")
def login_page():
    if is_authenticated():
        return redirect(url_for("index"))
    return render_template_string(LOGIN_HTML, robot_name=config["robot"]["name"], error="")


@app.post("/login")
def login_submit():
    username = (request.form.get("username") or "").strip()
    password = request.form.get("password") or ""
    if username == AUTH_USERNAME and password == AUTH_PASSWORD:
        session["authenticated"] = True
        return redirect(url_for("index"))
    return render_template_string(
        LOGIN_HTML,
        robot_name=config["robot"]["name"],
        error="Invalid credentials. Check your username and password.",
    )


@app.post("/logout")
@require_login
def logout():
    session.clear()
    control_runtime.disarm(
        source="logout",
        reason="Signed out. Controls returned to a safe disarmed state.",
    )
    return redirect(url_for("login_page"))


@app.post("/api/connect")
@require_login
def connect():
    try:
        if adapter.get_state()["connected"]:
            return jsonify({"ok": True, "state": current_payload()})
        connected = adapter.connect()
        if connected:
            connected_state = safe_adapter_state()
            control_runtime.sync_from_state(connected_state, source="connect")
            clear_controller_intent(source="connect")
    except Exception as exc:
        return error_response(str(exc), 409)
    return jsonify({"ok": connected, "state": current_payload()})


@app.post("/api/disconnect")
@require_login
def disconnect():
    adapter.disconnect()
    control_runtime.disarm(
        source="disconnect",
        reason="Disconnected from robot. Controls disarmed.",
    )
    return jsonify({"ok": True, "state": current_payload()})


@app.get("/api/status")
@require_login
def status():
    return jsonify(current_payload())


@app.post("/api/joints")
@require_login
def move_joints():
    blocked = require_motion_ready()
    if blocked:
        return blocked
    payload = request.get_json(silent=True) or {}
    source = str(payload.get("source") or "slider")
    command = ui_input_adapter.absolute_targets(payload.get("targets", {}), source=source)
    targets = clamp_targets(command.targets)
    try:
        desired = set_desired_targets(targets, source=command.source)
    except Exception as exc:
        return error_response(str(exc), 409)
    return jsonify({"ok": True, "joints": desired, "state": current_payload()})


@app.post("/api/controller_intent")
@require_login
def controller_intent():
    blocked = require_motion_ready()
    if blocked:
        return blocked
    payload = request.get_json(silent=True) or {}
    source = str(payload.get("source") or "controller")
    intents = payload.get("intents", {})
    if not isinstance(intents, dict):
        return error_response("Controller intent payload must be an object.")
    normalized = set_controller_intent(intents, source=source)
    return jsonify({"ok": True, "intent": normalized, "state": current_payload()})


@app.post("/api/controller_state")
@require_login
def controller_state():
    blocked = require_motion_ready()
    if blocked:
        return blocked
    payload = request.get_json(silent=True) or {}
    source = str(payload.get("source") or "PS4")
    axes = payload.get("axes", {})
    buttons = payload.get("buttons", {})
    if not isinstance(axes, dict) or not isinstance(buttons, dict):
        return error_response("Controller state must provide axes and buttons objects.")
    command, intents = ps4_input_adapter.from_snapshot(
        axes={str(name): float(value) for name, value in axes.items()},
        buttons={str(name): float(value) for name, value in buttons.items()},
        source=source,
    )
    normalized = set_controller_intent(intents, source=command.source)
    return jsonify({"ok": True, "intent": normalized, "state": current_payload()})


@app.post("/api/torque_off")
@require_login
def torque_off():
    adapter.disable_torque()
    control_runtime.disarm(
        source="torque_off",
        reason="Torque disabled. Controls disarmed.",
    )
    return jsonify({"ok": True, "state": current_payload()})


@app.post("/api/torque_on")
@require_login
def torque_on():
    try:
        adapter.enable_torque()
    except Exception as exc:
        return error_response(str(exc), 409)
    control_runtime.disarm(
        source="torque_on",
        reason="Servo torque enabled. Arm will hold position; re-arm for live motion.",
    )
    return jsonify({"ok": True, "state": current_payload()})


@app.post("/api/calibration/torque")
@require_login
def calibration_torque():
    payload = request.get_json(silent=True) or {}
    joint = (payload.get("joint") or "").strip()
    enabled = bool(payload.get("enabled"))
    if not joint:
        return error_response("Calibration torque control requires a joint name.")
    try:
        adapter.set_joint_torque(joint, enabled)
    except Exception as exc:
        return error_response(str(exc), 409)
    control_runtime.disarm(
        source="calibration",
        reason=f"Calibration mode changed torque for {joint}. Re-arm before normal motion.",
    )
    if not enabled:
        reset_desired_targets(safe_adapter_state().get("joints", {}), source="teach")
    return jsonify({"ok": True, "joint": joint, "enabled": enabled, "state": current_payload()})


@app.get("/api/calibration/read")
@require_login
def calibration_read():
    joint = (request.args.get("joint") or "").strip()
    if not joint:
        return error_response("Calibration read requires a joint name.")
    try:
        raw_step = adapter.get_joint_raw_step(joint)
    except Exception as exc:
        return error_response(str(exc), 409)
    return jsonify({"ok": True, "joint": joint, "raw_step": raw_step})


@app.post("/api/pose/save")
@require_login
def save_pose():
    payload = request.get_json(silent=True) or {}
    name = payload.get("name", "").strip()
    try:
        path = pose_path(name)
    except ValueError as exc:
        return error_response(str(exc))
    state = adapter.get_state()
    with path.open("w", encoding="utf-8") as handle:
        json.dump({"name": name, "joints": state["joints"]}, handle, indent=2)
    return jsonify({"ok": True, "pose": name, "state": current_payload()})


@app.post("/api/pose/load")
@require_login
def load_pose():
    blocked = require_motion_ready()
    if blocked:
        return blocked
    payload = request.get_json(silent=True) or {}
    name = payload.get("name", "").strip()
    try:
        joints = execute_pose_move(name)
    except ValueError as exc:
        return error_response(str(exc))
    except FileNotFoundError as exc:
        return error_response(str(exc), 404)
    except RuntimeError as exc:
        return error_response(str(exc), 409)
    return jsonify({"ok": True, "joints": joints, "state": current_payload()})


@app.post("/api/pose/delete")
@require_login
def delete_pose():
    payload = request.get_json(silent=True) or {}
    name = payload.get("name", "").strip()
    try:
        path = pose_path(name)
    except ValueError as exc:
        return error_response(str(exc))
    if not path.exists():
        return error_response(f"Pose '{name}' does not exist.", 404)
    path.unlink()
    return jsonify({"ok": True, "state": current_payload()})


@app.post("/api/home")
@require_login
def home():
    blocked = require_motion_ready()
    if blocked:
        return blocked
    defaults = clamp_targets(
        config.get("robot", {}).get("safe_startup_pose", {})
        or default_joint_positions(joint_definitions)
    )
    try:
        joints = set_desired_targets(defaults, source="home")
        clear_controller_intent(source="home")
    except Exception:
        # Mock mode and disconnected real hardware should still let the UI reset.
        reset = getattr(adapter, "reset_to_defaults", None)
        if callable(reset):
            joints = reset()
        else:
            return error_response("Unable to move to home while robot is disconnected.", 409)
    return jsonify({"ok": True, "joints": joints, "state": current_payload()})


@app.post("/api/sequence/save")
@require_login
def save_sequence():
    payload = request.get_json(silent=True) or {}
    name = payload.get("name", "").strip()
    pose_names = payload.get("poses", [])
    try:
        path = sequence_path(name)
    except ValueError as exc:
        return error_response(str(exc))
    if not isinstance(pose_names, list) or not pose_names:
        return error_response("Sequence must include at least one pose.")
    with path.open("w", encoding="utf-8") as handle:
        json.dump({"name": name, "poses": pose_names}, handle, indent=2)
    return jsonify({"ok": True, "sequence": name, "state": current_payload()})


@app.post("/api/sequence/load")
@require_login
def load_sequence():
    payload = request.get_json(silent=True) or {}
    name = payload.get("name", "").strip()
    try:
        path = sequence_path(name)
    except ValueError as exc:
        return error_response(str(exc))
    if not path.exists():
        return error_response(f"Sequence '{name}' does not exist.", 404)
    with path.open("r", encoding="utf-8") as handle:
        sequence = json.load(handle)
    return jsonify({"ok": True, "sequence": sequence, "state": current_payload()})


@app.post("/api/sequence/run")
@require_login
def run_sequence():
    blocked = require_motion_ready()
    if blocked:
        return blocked
    payload = request.get_json(silent=True) or {}
    name = payload.get("name", "").strip()
    try:
        result = execute_sequence_run(name)
    except ValueError as exc:
        return error_response(str(exc))
    except FileNotFoundError as exc:
        return error_response(str(exc), 404)
    except RuntimeError as exc:
        return error_response(str(exc), 409)
    return jsonify({"ok": True, "result": result, "state": current_payload()})


@app.post("/api/sequence/delete")
@require_login
def delete_sequence():
    payload = request.get_json(silent=True) or {}
    name = payload.get("name", "").strip()
    try:
        path = sequence_path(name)
    except ValueError as exc:
        return error_response(str(exc))
    if not path.exists():
        return error_response(f"Sequence '{name}' does not exist.", 404)
    path.unlink()
    return jsonify({"ok": True, "state": current_payload()})


@app.post("/api/recording/start")
@require_login
def start_recording():
    if control_runtime.safety_snapshot().estopped:
        return error_response("Reset emergency stop before recording.", 423)
    state = safe_adapter_state()
    if not state["connected"]:
        return error_response("Connect the robot before recording.", 409)
    with RECORDING_LOCK:
        if RECORDING_STATE["active"]:
            return error_response("Recording is already active.", 409)
        stop_event = threading.Event()
        worker = threading.Thread(
            target=_recording_worker,
            args=(stop_event,),
            daemon=True,
            name="robot-recording",
        )
        RECORDING_STATE["active"] = True
        RECORDING_STATE["started_at"] = time.perf_counter()
        RECORDING_STATE["duration_s"] = 0.0
        RECORDING_STATE["worker"] = worker
        RECORDING_STATE["stop_event"] = stop_event
        RECORDING_STATE["frames"] = []
    try:
        adapter.disable_torque()
    except Exception as exc:
        with RECORDING_LOCK:
            RECORDING_STATE["active"] = False
            RECORDING_STATE["started_at"] = None
            RECORDING_STATE["worker"] = None
            RECORDING_STATE["stop_event"] = None
        return error_response(str(exc), 409)
    clear_controller_intent(source="recording")
    control_runtime.disarm(
        source="recording",
        reason="Recording movement. Servos unlocked for hand-guided motion.",
    )
    reset_desired_targets(state.get("joints", {}), source="recording")
    worker.start()
    return jsonify({"ok": True, "state": current_payload()})


@app.post("/api/recording/stop")
@require_login
def stop_recording():
    with RECORDING_LOCK:
        worker = RECORDING_STATE.get("worker")
        stop_event = RECORDING_STATE.get("stop_event")
        if not RECORDING_STATE["active"] or worker is None or stop_event is None:
            return error_response("No active recording to stop.", 409)
        stop_event.set()
    worker.join(timeout=3.0)
    clear_controller_intent(source="recording")
    control_runtime.disarm(
        source="recording",
        reason="Recording captured. Use Play to replay or Lock Servos to hold pose.",
    )
    return jsonify({"ok": True, "state": current_payload()})


@app.post("/api/recording/play")
@require_login
def play_recording():
    if control_runtime.safety_snapshot().estopped:
        return error_response("Reset emergency stop before playback.", 423)
    with RECORDING_LOCK:
        if RECORDING_STATE["active"]:
            return error_response("Stop the active recording before playback.", 409)
        raw_frames = list(RECORDING_STATE.get("frames") or [])
    if len(raw_frames) < 2:
        return error_response("Record a movement first before playback.", 409)
    frames = _interpolate_recording_frames(raw_frames)
    state = safe_adapter_state()
    if not state["connected"]:
        return error_response("Connect the robot before playback.", 409)
    try:
        adapter.enable_torque()
    except Exception as exc:
        return error_response(str(exc), 409)
    clear_controller_intent(source="playback")
    control_runtime.arm(source="playback")
    control_runtime.set_lock_reason("Playing recorded movement.")

    previous_t = None
    final_joints = desired_targets_snapshot()
    try:
        for frame in frames:
            playback_frame = playback_input_adapter.frame(
                frame.get("joints", {}),
                frame.get("t", 0.0),
                source="playback",
            )
            frame_t = playback_frame.timestamp_s
            if previous_t is not None:
                delay = max(0.0, min(0.5, frame_t - previous_t))
                if delay:
                    time.sleep(delay)
            previous_t = frame_t
            final_joints = set_desired_targets(playback_frame.joints, source=playback_frame.source)
    except Exception as exc:
        clear_controller_intent(source="playback")
        control_runtime.disarm(
            source="playback",
            reason="Playback failed. Review recording and robot state.",
        )
        return error_response(str(exc), 409)

    clear_controller_intent(source="playback")
    control_runtime.disarm(
        source="playback",
        reason="Recorded movement replay complete. Re-arm for live control or record again.",
    )
    return jsonify({"ok": True, "joints": final_joints, "state": current_payload()})


@app.get("/api/verification")
@require_login
def verification_report():
    payload = current_payload()
    return jsonify({"ok": True, "verification": payload.get("verification", {}), "state": payload})


@app.post("/api/safety/arm")
@require_login
def arm_system():
    if control_runtime.safety_snapshot().estopped:
        return error_response("Reset emergency stop before arming.", 423)
    control_runtime.arm(source="arm")
    return jsonify({"ok": True, "state": current_payload()})


@app.post("/api/safety/disarm")
@require_login
def disarm_system():
    control_runtime.disarm(source="disarm", reason="System disarmed. Motion commands are blocked.")
    return jsonify({"ok": True, "state": current_payload()})


@app.post("/api/safety/estop")
@require_login
def emergency_stop():
    control_runtime.emergency_stop(source="estop")
    try:
        adapter.disable_torque()
    except Exception:
        pass
    return jsonify({"ok": True, "state": current_payload()})


@app.post("/api/safety/reset")
@require_login
def reset_estop():
    control_runtime.reset_estop(source="reset")
    return jsonify({"ok": True, "state": current_payload()})


@app.get("/status")
@require_api_access
def compat_status():
    payload = current_payload()
    return jsonify(
        {
            "online": bool(payload["robot"]["connected"]),
            "busy": False,
            "fault": bool(payload["safety"]["estopped"]),
            "motion_allowed": bool(payload["safety"]["motion_allowed"]),
            "lock_reason": payload["safety"]["lock_reason"],
            "robot": payload["robot"],
            "safety": payload["safety"],
            "joints": payload["joints"],
        }
    )


@app.get("/positions")
@require_api_access
def compat_positions():
    return jsonify(
        {
            "positions": list_saved_pose_names(),
            "allowed_sequences": list_saved_sequence_names(),
        }
    )


@app.post("/move/saved")
@require_api_access
def compat_move_saved():
    blocked = require_motion_ready()
    if blocked:
        return blocked
    payload = request.get_json(silent=True) or {}
    name = (payload.get("position_name") or "").strip()
    try:
        joints = execute_pose_move(name)
    except ValueError as exc:
        return error_response(str(exc))
    except FileNotFoundError as exc:
        return error_response(str(exc), 404)
    except RuntimeError as exc:
        return error_response(str(exc), 409)
    return jsonify({"ok": True, "position_name": name, "joints": joints})


@app.post("/sequence/run")
@require_api_access
def compat_sequence_run():
    blocked = require_motion_ready()
    if blocked:
        return blocked
    payload = request.get_json(silent=True) or {}
    name = (payload.get("sequence_name") or "").strip()
    try:
        result = execute_sequence_run(name)
    except ValueError as exc:
        return error_response(str(exc))
    except FileNotFoundError as exc:
        return error_response(str(exc), 404)
    except RuntimeError as exc:
        return error_response(str(exc), 409)
    return jsonify({"ok": True, "sequence_name": name, "result": result})


@app.post("/stop")
@require_api_access
def compat_stop():
    control_runtime.emergency_stop(source="compat_stop")
    try:
        adapter.disable_torque()
    except Exception:
        pass
    return jsonify({"ok": True, "stopped": True, "state": current_payload()})


@app.errorhandler(500)
def handle_internal_error(error):
    if request.path.startswith("/api/") or request.path in {
        "/status",
        "/positions",
        "/move/saved",
        "/sequence/run",
        "/stop",
    }:
        return error_response("Internal server error.", 500)
    return error


LOGIN_HTML = """
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{{ robot_name }} Login</title>
  <style>
    :root {
      --bg: #030303;
      --panel: rgba(14, 14, 14, 0.92);
      --ink: #f5f5f2;
      --muted: #9a9a95;
      --line: rgba(255, 255, 255, 0.1);
      --danger: #ff8a8a;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      place-items: center;
      padding: 24px;
      color: var(--ink);
      font-family: Bahnschrift, "Aptos", "Segoe UI", sans-serif;
      background:
        radial-gradient(circle at top left, rgba(255,255,255,0.06), transparent 22%),
        linear-gradient(180deg, #020202 0%, #090909 52%, #030303 100%);
    }
    .login-shell {
      width: min(100%, 460px);
      padding: 28px;
      border: 1px solid var(--line);
      border-radius: 28px;
      background: linear-gradient(180deg, rgba(20,20,20,0.94), rgba(8,8,8,0.96));
      box-shadow: 0 24px 60px rgba(0,0,0,0.38);
    }
    .eyebrow {
      text-transform: uppercase;
      letter-spacing: 0.28em;
      font-size: 11px;
      color: #d6d6d1;
      margin-bottom: 14px;
    }
    h1 {
      margin: 0 0 12px;
      font-size: clamp(34px, 7vw, 56px);
      line-height: 0.92;
      text-transform: uppercase;
    }
    p {
      margin: 0 0 22px;
      color: var(--muted);
      line-height: 1.6;
    }
    form {
      display: grid;
      gap: 14px;
    }
    label {
      display: grid;
      gap: 8px;
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.18em;
      color: var(--muted);
    }
    input {
      width: 100%;
      padding: 14px 16px;
      border-radius: 16px;
      border: 1px solid rgba(255,255,255,0.1);
      background: rgba(8,8,8,0.98);
      color: var(--ink);
      font: inherit;
    }
    button {
      border: 0;
      border-radius: 999px;
      padding: 14px 18px;
      font: inherit;
      font-weight: 700;
      letter-spacing: 0.04em;
      cursor: pointer;
      color: #040404;
      background: linear-gradient(135deg, #ffffff, #d6d6d1);
    }
    .error {
      border: 1px solid rgba(255,138,138,0.18);
      background: rgba(67, 17, 17, 0.5);
      color: var(--danger);
      border-radius: 16px;
      padding: 12px 14px;
      margin-bottom: 16px;
    }
  </style>
</head>
<body>
  <main class="login-shell">
    <div class="eyebrow">Protected Control Console</div>
    <h1>{{ robot_name }}</h1>
    <p>Sign in before opening the remote control surface. The app starts disarmed after every login for safer remote use.</p>
    {% if error %}
    <div class="error">{{ error }}</div>
    {% endif %}
    <form method="post" action="/login">
      <label>
        Username
        <input type="text" name="username" autocomplete="username" required>
      </label>
      <label>
        Password
        <input type="password" name="password" autocomplete="current-password" required>
      </label>
      <button type="submit">Unlock Controls</button>
    </form>
  </main>
</body>
</html>
"""


HTML = """
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{{ robot_name }}</title>
  <style>
    :root {
      --bg: #030303;
      --bg-2: #0a0a0a;
      --panel: rgba(14, 14, 14, 0.8);
      --panel-strong: rgba(18, 18, 18, 0.94);
      --ink: #f5f5f2;
      --muted: #9a9a95;
      --accent: #ffffff;
      --accent-2: #d6d6d1;
      --accent-3: #b6b6b0;
      --line: rgba(255, 255, 255, 0.1);
      --glow: rgba(255, 255, 255, 0.12);
      --danger: #ff7f7f;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: Bahnschrift, "Aptos", "Segoe UI Variable", "Trebuchet MS", sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(255, 255, 255, 0.06), transparent 22%),
        radial-gradient(circle at 82% 0%, rgba(255, 255, 255, 0.04), transparent 18%),
        linear-gradient(180deg, #020202 0%, #090909 52%, #030303 100%);
      min-height: 100vh;
    }
    body::before {
      content: "";
      position: fixed;
      inset: 0;
      background-image:
        linear-gradient(rgba(255, 255, 255, 0.025) 1px, transparent 1px),
        linear-gradient(90deg, rgba(255, 255, 255, 0.025) 1px, transparent 1px);
      background-size: 34px 34px;
      pointer-events: none;
      opacity: 0.5;
    }
    .shell {
      position: relative;
      max-width: 1380px;
      margin: 0 auto;
      padding: 30px 22px 48px;
    }
    .hero {
      display: grid;
      grid-template-columns: 1.25fr 0.75fr;
      gap: 18px;
      margin-bottom: 22px;
    }
    .eyebrow {
      text-transform: uppercase;
      letter-spacing: 0.28em;
      font-size: 11px;
      color: var(--accent-2);
    }
    h1 {
      margin: 0;
      font-family: Bahnschrift, "Arial Nova", sans-serif;
      font-size: clamp(44px, 7vw, 84px);
      line-height: 0.9;
      letter-spacing: 0.02em;
      text-transform: uppercase;
    }
    .subhead {
      max-width: 760px;
      font-size: 18px;
      line-height: 1.6;
      color: var(--muted);
    }
    .hero-copy,
    .hero-meta {
      border: 1px solid var(--line);
      border-radius: 28px;
      padding: 24px;
      background:
        linear-gradient(180deg, rgba(22, 22, 22, 0.92), rgba(8, 8, 8, 0.9));
      box-shadow:
        0 18px 50px rgba(0, 0, 0, 0.35),
        inset 0 1px 0 rgba(255, 255, 255, 0.04);
      backdrop-filter: blur(18px);
    }
    .hero-meta {
      display: grid;
      gap: 14px;
      align-content: start;
    }
    .meta-kicker {
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.24em;
      color: var(--accent-3);
    }
    .status-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
    }
    .status-tile {
      border-radius: 18px;
      padding: 14px;
      border: 1px solid rgba(255, 255, 255, 0.08);
      background: rgba(16, 16, 16, 0.9);
    }
    .status-label {
      display: block;
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: 0.2em;
      color: var(--muted);
      margin-bottom: 6px;
    }
    .status-value {
      font-size: 24px;
      font-weight: 700;
      color: var(--ink);
    }
    .hero-note {
      color: var(--muted);
      font-size: 14px;
      line-height: 1.6;
    }
    .layout {
      display: grid;
      grid-template-columns: minmax(0, 1.2fr) minmax(380px, 0.8fr);
      gap: 22px;
      align-items: start;
    }
    .safety-bar {
      display: grid;
      grid-template-columns: minmax(0, 1.2fr) minmax(320px, 0.8fr);
      gap: 22px;
      margin-bottom: 22px;
    }
    .card {
      background:
        linear-gradient(180deg, rgba(17, 17, 17, 0.9), rgba(8, 8, 8, 0.95));
      border: 1px solid var(--line);
      border-radius: 28px;
      padding: 22px;
      box-shadow:
        0 24px 60px rgba(0, 0, 0, 0.35),
        inset 0 1px 0 rgba(255, 255, 255, 0.04);
      backdrop-filter: blur(16px);
    }
    .card-controls {
      order: 1;
    }
    .card-preview {
      order: 2;
    }
    .card h2 {
      margin-top: 0;
      margin-bottom: 14px;
      font-size: 22px;
      letter-spacing: 0.06em;
      text-transform: uppercase;
    }
    .controls {
      display: grid;
      gap: 16px;
    }
    .stack {
      display: grid;
      gap: 16px;
    }
    .joint {
      padding: 16px;
      border-radius: 20px;
      background: linear-gradient(180deg, rgba(18, 18, 18, 0.94), rgba(10, 10, 10, 0.92));
      border: 1px solid rgba(255, 255, 255, 0.08);
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.04);
    }
    .joint-header, .toolbar {
      display: flex;
      justify-content: space-between;
      gap: 12px;
      align-items: center;
      flex-wrap: wrap;
    }
    .joint-header {
      align-items: flex-start;
    }
    .joint-title {
      display: grid;
      gap: 4px;
    }
    .joint-meta {
      display: grid;
      gap: 8px;
      margin-top: 10px;
    }
    .joint-meta-row {
      display: flex;
      justify-content: space-between;
      gap: 10px;
      align-items: center;
      flex-wrap: wrap;
      font-size: 12px;
      color: var(--muted);
    }
    .joint-badges {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
    }
    .joint-badge {
      padding: 6px 10px;
      border-radius: 999px;
      border: 1px solid rgba(255, 255, 255, 0.1);
      background: rgba(9, 9, 9, 0.92);
      font-size: 11px;
      letter-spacing: 0.04em;
      color: var(--ink);
    }
    .joint-range {
      display: flex;
      justify-content: space-between;
      gap: 12px;
      margin-top: 8px;
      font-size: 12px;
      color: var(--muted);
    }
    .joint-range span:last-child {
      text-align: right;
    }
    .joint-note {
      font-size: 12px;
      color: var(--muted);
      line-height: 1.45;
      margin-top: 8px;
    }
    button, input, select {
      font: inherit;
    }
    button {
      border: 0;
      border-radius: 999px;
      padding: 10px 16px;
      cursor: pointer;
      background: linear-gradient(135deg, var(--accent), var(--accent-2));
      color: #040404;
      font-weight: 700;
      letter-spacing: 0.04em;
      box-shadow: 0 0 0 1px rgba(255, 255, 255, 0.08), 0 10px 24px rgba(255, 255, 255, 0.08);
      transition: transform 0.16s ease, box-shadow 0.16s ease, filter 0.16s ease;
    }
    button:hover {
      transform: translateY(-1px);
      filter: brightness(1.05);
    }
    button:active {
      transform: translateY(0);
    }
    button.secondary {
      background: rgba(11, 11, 11, 0.95);
      color: var(--ink);
      border: 1px solid rgba(255, 255, 255, 0.1);
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.04);
    }
    button.secondary:hover {
      background: rgba(22, 22, 22, 0.98);
    }
    button.danger {
      background: linear-gradient(135deg, #ff8a8a, #ff5c5c);
      color: #1d0202;
      box-shadow: 0 0 0 1px rgba(255, 138, 138, 0.18), 0 10px 24px rgba(255, 92, 92, 0.18);
    }
    button.ghost {
      background: transparent;
      color: var(--muted);
      border: 1px solid rgba(255,255,255,0.1);
      box-shadow: none;
    }
    select, input {
      color: var(--ink);
    }
    input[type="range"] {
      width: 100%;
      accent-color: var(--accent);
      filter: saturate(1.2);
    }
    .status {
      white-space: pre-wrap;
      font-family: Consolas, "SFMono-Regular", "Cascadia Code", monospace;
      font-size: 13px;
      color: #e8e8e5;
      background: rgba(7, 7, 7, 0.94);
      padding: 14px;
      border-radius: 18px;
      border: 1px solid rgba(255, 255, 255, 0.08);
      min-height: 160px;
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.03);
    }
    .pose-row, .sequence-row {
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      margin-top: 14px;
    }
    .record-row {
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      margin-top: 14px;
      align-items: center;
    }
    .record-status {
      margin-top: 10px;
      font-size: 12px;
      color: rgba(255,255,255,0.7);
      letter-spacing: 0.04em;
    }
    .pose-row input, .pose-row select, .sequence-row input, .sequence-row select {
      flex: 1 1 180px;
      padding: 12px 14px;
      border-radius: 999px;
      border: 1px solid rgba(255, 255, 255, 0.1);
      background: rgba(10, 10, 10, 0.98);
    }
    .pill-list {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      margin-top: 14px;
    }
    .pill {
      background: rgba(14, 14, 14, 0.96);
      border: 1px solid rgba(255, 255, 255, 0.1);
      border-radius: 999px;
      padding: 8px 12px;
      cursor: pointer;
      color: var(--ink);
    }
    .hint {
      font-size: 14px;
      line-height: 1.5;
      color: var(--muted);
    }
    .safety-panel {
      display: grid;
      gap: 14px;
    }
    .safety-banner {
      border-radius: 20px;
      padding: 16px 18px;
      border: 1px solid rgba(255,255,255,0.08);
      background: rgba(11, 11, 11, 0.96);
    }
    .safety-banner strong {
      display: block;
      margin-bottom: 6px;
      font-size: 13px;
      letter-spacing: 0.18em;
      text-transform: uppercase;
    }
    .safety-banner span {
      color: var(--muted);
      line-height: 1.5;
      font-size: 14px;
    }
    .safety-actions {
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
    }
    .safety-summary {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
    }
    .summary-tile {
      border-radius: 18px;
      padding: 14px;
      border: 1px solid rgba(255,255,255,0.08);
      background: rgba(12,12,12,0.92);
    }
    .summary-tile span {
      display: block;
    }
    .summary-label {
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: 0.16em;
      color: var(--muted);
      margin-bottom: 8px;
    }
    .summary-value {
      font-size: 18px;
      font-weight: 700;
    }
    .control-title {
      display: flex;
      flex-direction: column;
      gap: 4px;
    }
    .control-title span:last-child {
      font-size: 12px;
      letter-spacing: 0.16em;
      text-transform: uppercase;
      color: var(--muted);
    }
    .preview-shell {
      position: relative;
      overflow: hidden;
      padding: 16px;
      border-radius: 24px;
      background:
        radial-gradient(circle at 50% 10%, rgba(255, 255, 255, 0.08), transparent 45%),
        linear-gradient(180deg, rgba(6, 6, 6, 0.95), rgba(14, 14, 14, 0.95));
      border: 1px solid rgba(255, 255, 255, 0.1);
    }
    .preview-shell::after {
      content: "";
      position: absolute;
      inset: 12px;
      border: 1px solid rgba(255, 255, 255, 0.06);
      border-radius: 18px;
      pointer-events: none;
    }
    .flash-status {
      min-height: 52px;
      margin-bottom: 14px;
    }
    .panel-section {
      margin-top: 18px;
    }
    .hud-line {
      height: 1px;
      margin: 18px 0;
      background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.18), transparent);
    }
    .controller-shell {
      display: grid;
      gap: 14px;
      margin-top: 16px;
      padding: 18px;
      border-radius: 24px;
      border: 1px solid rgba(255,255,255,0.08);
      background:
        radial-gradient(circle at 50% 25%, rgba(255,255,255,0.06), transparent 38%),
        linear-gradient(180deg, rgba(15,15,15,0.98), rgba(7,7,7,0.96));
      box-shadow: inset 0 1px 0 rgba(255,255,255,0.04);
    }
    .controller-toolbar {
      display: flex;
      justify-content: space-between;
      gap: 10px;
      align-items: center;
      flex-wrap: wrap;
    }
    .controller-top {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    .controller-main {
      display: grid;
      grid-template-columns: 1fr auto 1fr;
      gap: 16px;
      align-items: center;
    }
    .stick-zone {
      display: grid;
      gap: 10px;
      justify-items: center;
    }
    .stick-label {
      font-size: 11px;
      letter-spacing: 0.18em;
      text-transform: uppercase;
      color: var(--muted);
    }
    .stick-pad {
      position: relative;
      width: min(34vw, 160px);
      height: min(34vw, 160px);
      max-width: 160px;
      max-height: 160px;
      border-radius: 50%;
      border: 1px solid rgba(255,255,255,0.1);
      background:
        radial-gradient(circle at 50% 50%, rgba(255,255,255,0.06), rgba(255,255,255,0.01) 55%, transparent 56%),
        linear-gradient(180deg, rgba(18,18,18,0.98), rgba(7,7,7,0.98));
      box-shadow: inset 0 1px 0 rgba(255,255,255,0.04);
      touch-action: none;
    }
    .stick-pad::before,
    .stick-pad::after {
      content: "";
      position: absolute;
      background: rgba(255,255,255,0.08);
    }
    .stick-pad::before {
      left: 50%;
      top: 10%;
      bottom: 10%;
      width: 1px;
      transform: translateX(-50%);
    }
    .stick-pad::after {
      top: 50%;
      left: 10%;
      right: 10%;
      height: 1px;
      transform: translateY(-50%);
    }
    .stick-thumb {
      position: absolute;
      left: 50%;
      top: 50%;
      width: 44px;
      height: 44px;
      border-radius: 50%;
      transform: translate(-50%, -50%);
      background: linear-gradient(180deg, #f7f7f2, #bfbfba);
      box-shadow: 0 10px 18px rgba(0,0,0,0.3);
      pointer-events: none;
    }
    .controller-center {
      display: grid;
      gap: 12px;
      justify-items: center;
      min-width: 112px;
    }
    .controller-cluster {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
      width: 100%;
    }
    .controller-button {
      min-height: 52px;
      border-radius: 18px;
      border: 1px solid rgba(255,255,255,0.12);
      background: rgba(14,14,14,0.96);
      color: var(--ink);
      display: grid;
      place-items: center;
      font-size: 13px;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      box-shadow: inset 0 1px 0 rgba(255,255,255,0.04);
      user-select: none;
      -webkit-user-select: none;
      touch-action: manipulation;
    }
    .controller-button:active {
      background: rgba(28,28,28,0.98);
    }
    .controller-readout {
      font-size: 12px;
      color: var(--muted);
      text-align: center;
      line-height: 1.5;
    }
    .site-nav {
      position: sticky;
      top: 0;
      z-index: 10;
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 18px;
      padding: 16px 28px;
      background: rgba(0, 0, 0, 0.98);
      border-bottom: 1px solid rgba(255, 255, 255, 0.08);
    }
    .site-brand,
    .site-links {
      display: flex;
      align-items: center;
      gap: 16px;
    }
    .brand-mark {
      width: 36px;
      height: 36px;
      display: grid;
      place-items: center;
      background: #ffffff;
      color: #000000;
      font-weight: 800;
      font-size: 22px;
      letter-spacing: 0.02em;
    }
    .brand-name {
      font-size: 28px;
      color: #f2dfef;
      letter-spacing: 0.01em;
    }
    .site-links a {
      color: var(--ink);
      text-decoration: none;
      font-size: 14px;
    }
    .nav-logout {
      padding: 8px 14px;
    }
    .dashboard-shell {
      max-width: 1720px;
      min-height: calc(100vh - 76px);
      padding-top: 18px;
      padding-bottom: 18px;
      display: grid;
      grid-template-rows: minmax(0, 1fr) auto;
      gap: 18px;
    }
    .dashboard-grid {
      display: grid;
      grid-template-columns: minmax(620px, 0.95fr) minmax(560px, 1.05fr);
      gap: 18px;
      min-height: 0;
    }
    .dashboard-side {
      display: grid;
      grid-template-rows: auto auto minmax(0, 1fr);
      gap: 18px;
      min-height: 0;
    }
    .top-panels {
      display: grid;
      grid-template-columns: minmax(0, 1fr) minmax(0, 1fr);
      gap: 18px;
    }
    .compact-card {
      padding: 18px;
    }
    .panel-topbar {
      display: flex;
      justify-content: space-between;
      gap: 16px;
      align-items: flex-start;
      margin-bottom: 14px;
      flex-wrap: wrap;
    }
    .compact-toolbar {
      gap: 8px;
    }
    .field-inline {
      display: flex;
      align-items: center;
      gap: 8px;
    }
    .field-inline label {
      font-size: 12px;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      color: var(--muted);
    }
    .slider-panel {
      display: grid;
      grid-template-rows: auto auto minmax(0, 1fr);
      min-height: 0;
    }
    .panel-utility-row {
      margin-bottom: 14px;
    }
    .slider-grid {
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
      align-content: start;
      overflow: hidden;
    }
    .slider-grid .joint {
      padding: 14px;
    }
    .sequence-panel {
      display: grid;
      align-content: start;
    }
    .compact-pills {
      max-height: 68px;
      overflow: auto;
      padding-right: 4px;
    }
    .controller-panel {
      display: grid;
      grid-template-rows: auto auto minmax(0, 1fr);
      min-height: 0;
    }
    .controller-hint {
      margin-top: 8px;
      margin-bottom: 10px;
    }
    .dashboard-controller {
      margin-top: 0;
      padding: 14px;
      gap: 10px;
    }
    .dashboard-controller .stick-pad {
      width: min(18vw, 130px);
      height: min(18vw, 130px);
      max-width: 130px;
      max-height: 130px;
    }
    .dashboard-controller .controller-button {
      min-height: 44px;
      font-size: 12px;
    }
    .compact-status {
      min-height: 78px;
      font-size: 12px;
      padding: 12px;
    }
    .raw-panel .status {
      min-height: 140px;
      max-height: 180px;
      overflow: auto;
    }
    .truth-summary {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 10px;
      margin-bottom: 14px;
    }
    .truth-tile {
      border: 1px solid rgba(255,255,255,0.08);
      background: rgba(255,255,255,0.03);
      border-radius: 16px;
      padding: 12px 14px;
      display: grid;
      gap: 4px;
    }
    .truth-tile strong {
      font-size: 18px;
    }
    .truth-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
      margin-bottom: 14px;
    }
    .truth-card {
      border: 1px solid rgba(255,255,255,0.08);
      background: rgba(255,255,255,0.03);
      border-radius: 16px;
      padding: 12px 14px;
      display: grid;
      gap: 8px;
    }
    .truth-card-head {
      display: flex;
      justify-content: space-between;
      gap: 8px;
      align-items: center;
    }
    .truth-card-title {
      display: grid;
      gap: 2px;
    }
    .truth-card-title strong {
      font-size: 14px;
    }
    .truth-card-title span,
    .truth-card-head .joint-badge {
      font-size: 11px;
    }
    .truth-lines {
      display: grid;
      gap: 4px;
      font-size: 12px;
      color: var(--muted);
    }
    .truth-flags {
      display: flex;
      gap: 6px;
      flex-wrap: wrap;
    }
    .truth-flag {
      border-radius: 999px;
      padding: 4px 8px;
      font-size: 10px;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      border: 1px solid rgba(255,255,255,0.08);
      background: rgba(255,255,255,0.04);
      color: var(--muted);
    }
    .truth-flag.warn {
      color: #f0c972;
      border-color: rgba(240,201,114,0.22);
      background: rgba(240,201,114,0.08);
    }
    .truth-flag.good {
      color: #8de3a6;
      border-color: rgba(141,227,166,0.22);
      background: rgba(141,227,166,0.08);
    }
    .truth-flag.bad {
      color: #ff9b9b;
      border-color: rgba(255,155,155,0.24);
      background: rgba(255,155,155,0.08);
    }
    .raw-panel {
      padding: 18px;
    }
    .mobile-dock {
      position: sticky;
      bottom: 0;
      display: none;
      gap: 10px;
      padding: 14px 0 0;
      margin-top: 18px;
      background: linear-gradient(180deg, rgba(3,3,3,0), rgba(3,3,3,0.95) 30%);
    }
    .mobile-dock button {
      flex: 1;
    }
    @media (max-width: 860px) {
      .site-nav {
        padding: 14px 18px;
        flex-direction: column;
        align-items: flex-start;
      }
      .site-links {
        flex-wrap: wrap;
      }
      .dashboard-shell {
        min-height: auto;
        grid-template-rows: auto auto;
      }
      .dashboard-grid {
        grid-template-columns: 1fr;
      }
      .dashboard-side,
      .top-panels {
        grid-template-columns: 1fr;
      }
      .slider-grid {
        grid-template-columns: 1fr;
      }
      .truth-summary,
      .truth-grid {
        grid-template-columns: 1fr;
      }
      .hero {
        grid-template-columns: 1fr;
      }
      .safety-bar {
        grid-template-columns: 1fr;
      }
      .layout {
        grid-template-columns: 1fr;
      }
      .card-preview {
        order: 1;
      }
      .card-controls {
        order: 2;
      }
      .safety-summary {
        grid-template-columns: 1fr;
      }
      .virtual-grid {
        grid-template-columns: 1fr;
      }
      .mobile-dock {
        display: flex;
      }
      .controller-top {
        grid-template-columns: 1fr;
      }
      .controller-main {
        grid-template-columns: 1fr;
      }
      .controller-center {
        width: 100%;
      }
    }
  </style>
</head>
<body>
  <nav class="site-nav">
    <div class="site-brand">
      <div class="brand-mark">N</div>
      <div class="brand-name">Nate Walinder</div>
    </div>
    <div class="site-links">
      <a href="#">Home</a>
      <a href="#">Professional Work</a>
      <a href="#">Personal Work</a>
      <a href="#">Contact</a>
      <form id="logoutForm" method="post" action="/logout" style="margin: 0;">
        <button type="submit" class="ghost nav-logout">Log Out</button>
      </form>
    </div>
  </nav>

  <div class="shell dashboard-shell">
    <section class="dashboard-grid">
      <div class="card slider-panel">
        <div class="panel-topbar">
          <div class="control-title">
            <span>Servo Sliders</span>
            <span>Direct Joint Manipulation</span>
          </div>
          <div class="toolbar compact-toolbar">
            <button id="connectBtn">Connect</button>
            <button class="secondary" id="disconnectBtn">Disconnect</button>
            <button class="secondary" id="homeBtn">Home</button>
            <button class="secondary" id="refreshBtn">Refresh</button>
          </div>
        </div>
        <div class="toolbar compact-toolbar panel-utility-row">
          <div class="field-inline">
            <label for="stepSize"><strong>Step Size</strong></label>
            <select id="stepSize">
              <option value="1">1 degree</option>
              <option value="5" selected>5 degrees</option>
              <option value="10">10 degrees</option>
            </select>
          </div>
        </div>
        <div class="controls slider-grid" id="jointControls"></div>
      </div>

      <div class="dashboard-side">
        <div class="top-panels">
          <div class="card safety-panel compact-card">
            <div class="control-title">
              <span>Safety Console</span>
              <span>Remote Safe States</span>
            </div>
            <div id="safetyBanner" class="safety-banner">
              <strong>System Locked</strong>
              <span>Loading safety state...</span>
            </div>
            <div class="safety-actions">
              <button id="armBtn">Arm</button>
              <button id="disarmBtn" class="secondary">Disarm</button>
              <button id="lockBtn" class="secondary">Lock Servos</button>
              <button id="unlockBtn" class="secondary">Unlock Servos</button>
              <button id="resetBtn" class="secondary">Reset E-Stop</button>
              <button id="estopBtn" class="danger">Emergency Stop</button>
            </div>
          </div>

          <div class="card compact-card">
            <div class="control-title">
              <span>Connection State</span>
              <span>Live Readiness</span>
            </div>
            <div class="safety-summary">
              <div class="summary-tile">
                <span class="summary-label">Session</span>
                <span class="summary-value" id="sessionValue">Locked</span>
              </div>
              <div class="summary-tile">
                <span class="summary-label">Motion</span>
                <span class="summary-value" id="motionValue">Blocked</span>
              </div>
              <div class="summary-tile">
                <span class="summary-label">Robot Link</span>
                <span class="summary-value" id="linkValue">Offline</span>
              </div>
            </div>
            <div id="flashBox" class="status flash-status compact-status">Ready.</div>
          </div>
        </div>

        <div class="card sequence-panel compact-card">
          <div class="control-title">
            <span>Record Sequence Box</span>
            <span>Poses And Playback</span>
          </div>
          <div class="record-row">
            <button id="recordBtn">Record</button>
            <button class="secondary" id="stopRecordBtn">Stop</button>
            <button class="secondary" id="playRecordBtn">Play</button>
          </div>
          <div class="record-status" id="recordingStatus">No recording yet.</div>
          <div class="pose-row">
            <input id="poseName" placeholder="pose name">
            <button id="savePoseBtn">Save Pose</button>
          </div>
          <div class="pose-row">
            <select id="poseSelect"></select>
            <button class="secondary" id="loadPoseBtn">Load Pose</button>
            <button class="secondary" id="deletePoseBtn">Delete Pose</button>
          </div>
          <div class="pill-list compact-pills" id="sequencePoseList"></div>
          <div class="sequence-row">
            <input id="sequenceName" placeholder="sequence name">
            <button id="saveSequenceBtn">Save Sequence</button>
          </div>
          <div class="sequence-row">
            <select id="sequenceSelect"></select>
            <button class="secondary" id="viewSequenceBtn">View Sequence</button>
            <button class="secondary" id="deleteSequenceBtn">Delete Sequence</button>
          </div>
        </div>

        <div class="card controller-panel compact-card">
          <div class="controller-toolbar">
            <div class="control-title">
              <span>Virtual PS4 Controller</span>
              <span>Touch And Hardware Input</span>
            </div>
            <div class="toolbar compact-toolbar">
              <button class="secondary" id="gamepadToggleBtn">Enable</button>
              <div class="field-inline">
                <label for="gamepadSpeed"><strong>PS4 Speed</strong></label>
                <select id="gamepadSpeed">
                  <option value="2">Slow</option>
                  <option value="4" selected>Medium</option>
                  <option value="7">Fast</option>
                </select>
              </div>
              <div class="field-inline">
                <label for="controllerSpeed"><strong>Touch Speed</strong></label>
                <select id="controllerSpeed">
                  <option value="2">Fine</option>
                  <option value="4" selected>Medium</option>
                  <option value="7">Fast</option>
                </select>
              </div>
            </div>
          </div>
          <div class="hint controller-hint" id="gamepadStatus">
            Connect a PS4 controller to your computer, then press any button with this page focused.
          </div>
          <div class="controller-shell dashboard-controller">
            <div class="controller-top">
              <button class="controller-button" data-control="l2">L2 Base Left</button>
              <button class="controller-button" data-control="r2">R2 Base Right</button>
            </div>
            <div class="controller-main">
              <section class="stick-zone">
                <div class="stick-label">Left Stick</div>
                <div class="stick-pad" data-stick="left">
                  <div class="stick-thumb"></div>
                </div>
              </section>
              <section class="controller-center">
                <div class="controller-cluster">
                  <button class="controller-button" data-control="l1">L1 Open</button>
                  <button class="controller-button" data-control="r1">R1 Close</button>
                </div>
                <div class="controller-readout">
                  Left stick controls shoulder lift and wrist roll.<br>
                  Right stick controls elbow and wrist flex.
                </div>
              </section>
              <section class="stick-zone">
                <div class="stick-label">Right Stick</div>
                <div class="stick-pad" data-stick="right">
                  <div class="stick-thumb"></div>
                </div>
              </section>
            </div>
          </div>
        </div>
      </div>
    </section>

    <section class="card raw-panel">
      <div class="control-title">
        <span>Raw System Snapshot</span>
        <span>Live State</span>
      </div>
      <div class="truth-summary" id="truthSummary"></div>
      <div class="truth-summary" id="verificationSummary"></div>
      <div class="record-status" id="verificationStatus">Verification loading...</div>
      <div class="truth-grid" id="jointDebugGrid"></div>
      <div class="status" id="statusBox">Loading...</div>
    </section>

    <div class="mobile-dock">
      <button id="mobileArmBtn">Arm</button>
      <button id="mobileStopBtn" class="danger">E-Stop</button>
    </div>
  </div>

  <script>
    const jointControls = document.getElementById("jointControls");
    const statusBox = document.getElementById("statusBox");
    const poseSelect = document.getElementById("poseSelect");
    const flashBox = document.getElementById("flashBox");
    const stepSize = document.getElementById("stepSize");
    const gamepadToggleBtn = document.getElementById("gamepadToggleBtn");
    const gamepadStatus = document.getElementById("gamepadStatus");
    const gamepadSpeed = document.getElementById("gamepadSpeed");
    const recordBtn = document.getElementById("recordBtn");
    const stopRecordBtn = document.getElementById("stopRecordBtn");
    const playRecordBtn = document.getElementById("playRecordBtn");
    const recordingStatus = document.getElementById("recordingStatus");
    const truthSummary = document.getElementById("truthSummary");
    const verificationSummary = document.getElementById("verificationSummary");
    const verificationStatus = document.getElementById("verificationStatus");
    const jointDebugGrid = document.getElementById("jointDebugGrid");
    const sequencePoseList = document.getElementById("sequencePoseList");
    const sequenceSelect = document.getElementById("sequenceSelect");
    const previewCanvas = document.getElementById("armPreview");
    const previewContext = previewCanvas ? previewCanvas.getContext("2d") : null;
    const controllerSpeed = document.getElementById("controllerSpeed");
    const safetyBanner = document.getElementById("safetyBanner");
    const sessionValue = document.getElementById("sessionValue");
    const motionValue = document.getElementById("motionValue");
    const linkValue = document.getElementById("linkValue");
    const armBtn = document.getElementById("armBtn");
    const disarmBtn = document.getElementById("disarmBtn");
    const lockBtn = document.getElementById("lockBtn");
    const unlockBtn = document.getElementById("unlockBtn");
    const resetBtn = document.getElementById("resetBtn");
    const estopBtn = document.getElementById("estopBtn");
    const mobileArmBtn = document.getElementById("mobileArmBtn");
    const mobileStopBtn = document.getElementById("mobileStopBtn");
    let latestState = null;
    let actualJoints = {};
    let commandedJoints = {};
    let uiJoints = {};
    let pendingTargets = {};
    let pendingSources = {};
    let uiSources = {};
    const jointElements = {};
    let activeSliderName = null;
    let commandFlushTimer = null;
    let commandQueue = {};
    let commandInFlight = false;
    let pendingFlashMessage = null;
    let lastCommandSentAt = 0;
    let lastControllerIntentSentAt = 0;
    let lastControllerIntentSignature = "";
    const COMMAND_RATE_MS = 40;
    const GAMEPAD_COMMAND_RATE_MS = 40;
    const CONTROLLER_INTENT_RATE_MS = 40;
    const STATUS_POLL_MS = 1500;
    const TARGET_EPSILON = 0.25;
    const GAMEPAD_AXIS_DEADZONE = 0.18;
    const GAMEPAD_TRIGGER_DEADZONE = 0.12;
    const GAMEPAD_BUTTON_THRESHOLD = 0.55;
    const DEBUG_TARGET_MISMATCH_EPSILON = 1.0;
    let selectedSequencePoses = [];
    let gamepadEnabled = false;
    let lastGamepadSendAt = 0;
    let gamepadLoopStarted = false;
    let preferredGamepadIndex = null;
    let controllerPulse = null;
    const controllerState = {
      left: { x: 0, y: 0, active: false },
      right: { x: 0, y: 0, active: false },
      buttons: { l1: 0, r1: 0, l2: 0, r2: 0 },
    };
    let activeControllerSource = null;

    async function fetchJson(url, options = {}) {
      const response = await fetch(url, {
        headers: { "Content-Type": "application/json" },
        ...options,
      });
      const contentType = response.headers.get("content-type") || "";
      const rawText = await response.text();
      let data = null;

      if (rawText) {
        if (contentType.includes("application/json")) {
          try {
            data = JSON.parse(rawText);
          } catch (error) {
            throw new Error("The robot server returned invalid JSON.");
          }
        } else {
          throw new Error(
            `The robot server returned ${contentType || "an unexpected response"} instead of JSON.`
          );
        }
      } else {
        data = {};
      }

      if (!response.ok || data.ok === false) {
        throw new Error(data.error || `Request failed: ${response.status}`);
      }
      return data;
    }

    function setFlash(message, isError = false) {
      flashBox.textContent = message;
      flashBox.style.color = isError ? "#8b1e14" : "#1e1d1b";
      flashBox.style.background = isError ? "#fde8e4" : "#fffaf5";
    }

    function setGamepadStatus(message) {
      gamepadStatus.textContent = message;
    }

    function buttonGroup(disabled) {
      [armBtn, disarmBtn, lockBtn, unlockBtn, resetBtn, estopBtn, mobileArmBtn, mobileStopBtn].forEach((button) => {
        button.disabled = disabled;
        button.style.opacity = disabled ? "0.45" : "1";
      });
    }

    function updateSafetyUI(data) {
      const auth = data.auth || {};
      const safety = data.safety || {};
      const robot = data.robot || {};

      sessionValue.textContent = auth.logged_in ? "Unlocked" : "Locked";
      motionValue.textContent = safety.motion_allowed ? "Live" : "Blocked";
      linkValue.textContent = robot.connected ? "Online" : "Offline";

      if (safety.estopped) {
        safetyBanner.innerHTML = "<strong>Emergency Stop Active</strong><span>All motion commands are blocked until you reset and re-arm the system.</span>";
        safetyBanner.style.borderColor = "rgba(255,138,138,0.35)";
        safetyBanner.style.background = "rgba(60, 12, 12, 0.72)";
      } else if (safety.armed) {
        safetyBanner.innerHTML = `<strong>System Armed</strong><span>${safety.lock_reason || "Motion commands are enabled."}</span>`;
        safetyBanner.style.borderColor = "rgba(255,255,255,0.12)";
        safetyBanner.style.background = "rgba(10, 18, 10, 0.72)";
      } else {
        safetyBanner.innerHTML = `<strong>System Disarmed</strong><span>${safety.lock_reason || "Controls are blocked until you arm the system."}</span>`;
        safetyBanner.style.borderColor = "rgba(255,255,255,0.08)";
        safetyBanner.style.background = "rgba(11, 11, 11, 0.96)";
      }

      const controlDisabled = !safety.motion_allowed;
      gamepadToggleBtn.disabled = controlDisabled;
      gamepadToggleBtn.style.opacity = controlDisabled ? "0.45" : "1";
      armBtn.disabled = safety.estopped;
      armBtn.style.opacity = safety.estopped ? "0.45" : "1";
      lockBtn.disabled = !robot.connected;
      lockBtn.style.opacity = !robot.connected ? "0.45" : "1";
      unlockBtn.disabled = !robot.connected;
      unlockBtn.style.opacity = !robot.connected ? "0.45" : "1";
      mobileArmBtn.disabled = safety.estopped;
      mobileArmBtn.style.opacity = safety.estopped ? "0.45" : "1";
    }

    function updateRecordingUI(data) {
      const recording = data.recording || {};
      const robot = data.robot || {};
      const active = Boolean(recording.active);
      const hasRecording = Boolean(recording.has_recording);
      const duration = Number(recording.duration_s || 0).toFixed(1);
      const frames = Number(recording.frame_count || 0);

      recordBtn.disabled = !robot.connected || active;
      stopRecordBtn.disabled = !active;
      playRecordBtn.disabled = !robot.connected || active || !hasRecording;

      recordBtn.style.opacity = recordBtn.disabled ? "0.45" : "1";
      stopRecordBtn.style.opacity = stopRecordBtn.disabled ? "0.45" : "1";
      playRecordBtn.style.opacity = playRecordBtn.disabled ? "0.45" : "1";

      if (active) {
        recordingStatus.textContent = `Recording unlocked motion... ${duration}s captured so far.`;
      } else if (hasRecording) {
        recordingStatus.textContent = `Last take ready: ${duration}s across ${frames} samples.`;
      } else {
        recordingStatus.textContent = "No recording yet.";
      }
    }

    async function safetyAction(path, successMessage = null, options = {}) {
      try {
        if (options.resetInputs) {
          await resetInteractiveInputs(options.source || "system");
        }
        await fetchJson(path, { method: "POST", body: "{}" });
        if (successMessage) {
          setFlash(successMessage);
        }
        await refresh();
      } catch (error) {
        setFlash(error.message, true);
      }
    }

    function drawArmPreview(joints) {
      if (!previewCanvas || !previewContext) {
        return;
      }
      const shoulderPan = joints.shoulder_pan || 0;
      const shoulderLift = joints.shoulder_lift || 0;
      const elbowFlex = joints.elbow_flex || 0;
      const wristFlex = joints.wrist_flex || 0;
      const gripper = joints.gripper || 0;

      previewContext.clearRect(0, 0, previewCanvas.width, previewCanvas.height);
      previewContext.fillStyle = "#050505";
      previewContext.fillRect(0, 0, previewCanvas.width, previewCanvas.height);

      const previewGradient = previewContext.createRadialGradient(180, 80, 20, 180, 140, 220);
      previewGradient.addColorStop(0, "rgba(255,255,255,0.10)");
      previewGradient.addColorStop(0.45, "rgba(255,255,255,0.03)");
      previewGradient.addColorStop(1, "rgba(255,255,255,0)");
      previewContext.fillStyle = previewGradient;
      previewContext.fillRect(0, 0, previewCanvas.width, previewCanvas.height);

      previewContext.strokeStyle = "rgba(255,255,255,0.05)";
      previewContext.lineWidth = 1;
      for (let x = 24; x < previewCanvas.width; x += 48) {
        previewContext.beginPath();
        previewContext.moveTo(x, 0);
        previewContext.lineTo(x, previewCanvas.height);
        previewContext.stroke();
      }
      for (let y = 24; y < previewCanvas.height; y += 48) {
        previewContext.beginPath();
        previewContext.moveTo(0, y);
        previewContext.lineTo(previewCanvas.width, y);
        previewContext.stroke();
      }

      previewContext.strokeStyle = "rgba(255,255,255,0.10)";
      previewContext.lineWidth = 1;
      previewContext.beginPath();
      previewContext.moveTo(0, 220);
      previewContext.lineTo(previewCanvas.width, 220);
      previewContext.stroke();

      const baseX = 80 + shoulderPan * 0.3;
      const baseY = 220;
      const upperLength = 80;
      const foreLength = 70;
      const wristLength = 40;

      const upperAngle = (-90 + shoulderLift) * Math.PI / 180;
      const elbowAngle = (elbowFlex - 20) * Math.PI / 180;
      const wristAngle = wristFlex * Math.PI / 180;

      const elbowX = baseX + Math.cos(upperAngle) * upperLength;
      const elbowY = baseY + Math.sin(upperAngle) * upperLength;
      const wristX = elbowX + Math.cos(upperAngle + elbowAngle) * foreLength;
      const wristY = elbowY + Math.sin(upperAngle + elbowAngle) * foreLength;
      const toolX = wristX + Math.cos(upperAngle + elbowAngle + wristAngle) * wristLength;
      const toolY = wristY + Math.sin(upperAngle + elbowAngle + wristAngle) * wristLength;
      const toolAngle = upperAngle + elbowAngle + wristAngle;

      previewContext.strokeStyle = "rgba(255,255,255,0.16)";
      previewContext.lineWidth = 2;
      previewContext.beginPath();
      previewContext.ellipse(baseX, baseY + 8, 26, 11, 0, 0, Math.PI * 2);
      previewContext.stroke();

      previewContext.fillStyle = "rgba(255,255,255,0.12)";
      previewContext.beginPath();
      previewContext.arc(baseX, baseY, 18, 0, Math.PI * 2);
      previewContext.fill();

      previewContext.strokeStyle = "rgba(255,255,255,0.12)";
      previewContext.lineWidth = 24;
      previewContext.lineCap = "round";
      previewContext.beginPath();
      previewContext.moveTo(baseX, baseY);
      previewContext.lineTo(elbowX, elbowY);
      previewContext.lineTo(wristX, wristY);
      previewContext.lineTo(toolX, toolY);
      previewContext.stroke();

      previewContext.strokeStyle = "#f3f3f0";
      previewContext.lineWidth = 12;
      previewContext.lineCap = "round";
      previewContext.beginPath();
      previewContext.moveTo(baseX, baseY);
      previewContext.lineTo(elbowX, elbowY);
      previewContext.lineTo(wristX, wristY);
      previewContext.lineTo(toolX, toolY);
      previewContext.stroke();

      previewContext.strokeStyle = "rgba(255,255,255,0.28)";
      previewContext.lineWidth = 2;
      previewContext.setLineDash([5, 7]);
      previewContext.beginPath();
      previewContext.moveTo(baseX, baseY);
      previewContext.lineTo(toolX, toolY);
      previewContext.stroke();
      previewContext.setLineDash([]);

      previewContext.fillStyle = "#ffffff";
      [[baseX, baseY], [elbowX, elbowY], [wristX, wristY], [toolX, toolY]].forEach(([x, y]) => {
        previewContext.beginPath();
        previewContext.fillStyle = "rgba(255,255,255,0.12)";
        previewContext.arc(x, y, 13, 0, Math.PI * 2);
        previewContext.fill();
        previewContext.beginPath();
        previewContext.fillStyle = "#ffffff";
        previewContext.arc(x, y, 7, 0, Math.PI * 2);
        previewContext.fill();
      });

      const gripperSpread = 8 + (gripper / 100) * 18;
      const jawLength = 22;
      const jawBaseOffset = 8;
      const jawAngle = Math.PI / 2;

      const jawBaseX = toolX + Math.cos(toolAngle) * jawBaseOffset;
      const jawBaseY = toolY + Math.sin(toolAngle) * jawBaseOffset;

      const perpX = Math.cos(toolAngle + jawAngle);
      const perpY = Math.sin(toolAngle + jawAngle);

      const upperJawBaseX = jawBaseX + perpX * (gripperSpread / 2);
      const upperJawBaseY = jawBaseY + perpY * (gripperSpread / 2);
      const lowerJawBaseX = jawBaseX - perpX * (gripperSpread / 2);
      const lowerJawBaseY = jawBaseY - perpY * (gripperSpread / 2);

      const upperJawTipX = upperJawBaseX + Math.cos(toolAngle) * jawLength;
      const upperJawTipY = upperJawBaseY + Math.sin(toolAngle) * jawLength;
      const lowerJawTipX = lowerJawBaseX + Math.cos(toolAngle) * jawLength;
      const lowerJawTipY = lowerJawBaseY + Math.sin(toolAngle) * jawLength;

      previewContext.strokeStyle = "rgba(255,255,255,0.16)";
      previewContext.lineWidth = 10;
      previewContext.beginPath();
      previewContext.moveTo(upperJawBaseX, upperJawBaseY);
      previewContext.lineTo(upperJawTipX, upperJawTipY);
      previewContext.moveTo(lowerJawBaseX, lowerJawBaseY);
      previewContext.lineTo(lowerJawTipX, lowerJawTipY);
      previewContext.stroke();

      previewContext.strokeStyle = "#ffffff";
      previewContext.lineWidth = 4;
      previewContext.beginPath();
      previewContext.moveTo(upperJawBaseX, upperJawBaseY);
      previewContext.lineTo(upperJawTipX, upperJawTipY);
      previewContext.moveTo(lowerJawBaseX, lowerJawBaseY);
      previewContext.lineTo(lowerJawTipX, lowerJawTipY);
      previewContext.stroke();

      previewContext.fillStyle = "#ffffff";
      previewContext.beginPath();
      previewContext.arc(jawBaseX, jawBaseY, 4, 0, Math.PI * 2);
      previewContext.fill();

      previewContext.strokeStyle = "rgba(255,255,255,0.12)";
      previewContext.lineWidth = 1;
      previewContext.strokeRect(14, 14, previewCanvas.width - 28, previewCanvas.height - 28);

      previewContext.fillStyle = "rgba(255,255,255,0.72)";
      previewContext.font = '12px Bahnschrift, "Segoe UI", sans-serif';
      previewContext.letterSpacing = "0.12em";
      previewContext.fillText(`GRIPPER ${Math.round(gripper)}`, 20, 28);
      previewContext.fillText(`PAN ${Math.round(shoulderPan)}`, 20, previewCanvas.height - 18);
      previewContext.fillText(`LIFT ${Math.round(shoulderLift)}`, 112, previewCanvas.height - 18);
      previewContext.fillText(`ELBOW ${Math.round(elbowFlex)}`, 212, previewCanvas.height - 18);
    }

    function formatJointValue(value) {
      return Number(value ?? 0).toFixed(1);
    }

    function valueOrDefault(name, fallback = 0) {
      if (Object.prototype.hasOwnProperty.call(uiJoints, name)) {
        return Number(uiJoints[name]);
      }
      if (Object.prototype.hasOwnProperty.call(pendingTargets, name)) {
        return Number(pendingTargets[name]);
      }
      if (Object.prototype.hasOwnProperty.call(commandedJoints, name)) {
        return Number(commandedJoints[name]);
      }
      if (Object.prototype.hasOwnProperty.call(actualJoints, name)) {
        return Number(actualJoints[name]);
      }
      return Number(fallback);
    }

    function commandedValueOrDefault(name, fallback = 0) {
      if (Object.prototype.hasOwnProperty.call(pendingTargets, name)) {
        return Number(pendingTargets[name]);
      }
      if (Object.prototype.hasOwnProperty.call(commandedJoints, name)) {
        return Number(commandedJoints[name]);
      }
      if (Object.prototype.hasOwnProperty.call(actualJoints, name)) {
        return Number(actualJoints[name]);
      }
      return Number(fallback);
    }

    function clampJointValue(name, value) {
      const limits = jointLimits(name);
      return clamp(Number(value), Number(limits.min), Number(limits.max));
    }

    function sameTarget(left, right) {
      return Math.abs(Number(left) - Number(right)) < TARGET_EPSILON;
    }

    function speedScalar(selectValue) {
      const speed = Number(selectValue || 4);
      return { 2: 0.35, 4: 0.6, 7: 1.0 }[speed] || 0.6;
    }

    async function sendGamepadSnapshot(snapshot, source = "PS4", force = false) {
      const axes = snapshot.axes || {};
      const buttons = snapshot.buttons || {};
      const signature = JSON.stringify({
        source,
        axes: Object.keys(axes).sort().reduce((acc, key) => {
          acc[key] = Number(axes[key] || 0).toFixed(3);
          return acc;
        }, {}),
        buttons: Object.keys(buttons).sort().reduce((acc, key) => {
          acc[key] = Number(buttons[key] || 0).toFixed(3);
          return acc;
        }, {}),
      });
      const now = Date.now();
      const hasActivity = Object.values(axes).some((value) => Math.abs(Number(value || 0)) >= 0.001)
        || Object.values(buttons).some((value) => Math.abs(Number(value || 0)) >= 0.001);
      if (!force && signature === lastControllerIntentSignature && (!hasActivity || now - lastControllerIntentSentAt < CONTROLLER_INTENT_RATE_MS)) {
        return;
      }
      lastControllerIntentSignature = signature;
      lastControllerIntentSentAt = now;
      activeControllerSource = source;
      try {
        const response = await fetchJson("/api/controller_state", {
          method: "POST",
          body: JSON.stringify({ source, axes, buttons }),
        });
        if (response.state) {
          renderState(response.state);
        }
      } catch (error) {
        setFlash(error.message, true);
      }
    }

    function activeSourceForJoint(name, data) {
      const jointDebug = (data && data.joint_debug && data.joint_debug[name]) || {};
      return (
        pendingSources[name] ||
        uiSources[name] ||
        jointDebug.active_source ||
        "system"
      );
    }

    function pendingValueForJoint(name) {
      if (Object.prototype.hasOwnProperty.call(pendingTargets, name)) {
        return Number(pendingTargets[name]);
      }
      return null;
    }

    function formatConfidence(value) {
      const normalized = String(value || "low").toLowerCase();
      if (normalized === "high") {
        return "High confidence";
      }
      if (normalized === "medium") {
        return "Medium confidence";
      }
      return "Low confidence";
    }

    function jointDisplayLabel(name, meta) {
      const label = meta.label_for_ui || name.replaceAll("_", " ");
      const servoText = meta.servo_id ? ` | Servo ${meta.servo_id}` : "";
      return `${label}${servoText}`;
    }

    function buildJointControls(limits, jointMeta) {
      jointControls.innerHTML = "";
      Object.keys(jointElements).forEach((key) => delete jointElements[key]);

      Object.entries(limits).forEach(([name, definition]) => {
        const meta = jointMeta[name] || {};
        const section = document.createElement("section");
        section.className = "joint";

        const header = document.createElement("div");
        header.className = "joint-header";

        const title = document.createElement("div");
        title.className = "joint-title";

        const titleLine = document.createElement("strong");
        const targetValue = document.createElement("span");
        targetValue.className = "joint-badge";

        title.appendChild(titleLine);
        title.appendChild(targetValue);

        const badgeRow = document.createElement("div");
        badgeRow.className = "joint-badges";

        const actualBadge = document.createElement("span");
        actualBadge.className = "joint-badge";
        const mappingBadge = document.createElement("span");
        mappingBadge.className = "joint-badge";
        const invertBadge = document.createElement("span");
        invertBadge.className = "joint-badge";

        badgeRow.appendChild(actualBadge);
        badgeRow.appendChild(mappingBadge);
        badgeRow.appendChild(invertBadge);

        header.appendChild(title);
        header.appendChild(badgeRow);

        const slider = document.createElement("input");
        slider.type = "range";
        slider.min = definition.min;
        slider.max = definition.max;
        slider.step = "1";
        slider.addEventListener("pointerdown", () => {
          activeSliderName = name;
        });
        slider.addEventListener("input", () => {
          uiJoints[name] = clampJointValue(name, slider.value);
          uiSources[name] = "slider";
          updateJointControl(name, latestState || {});
        });
        slider.addEventListener("change", async () => {
          const nextValue = clampJointValue(name, slider.value);
          await sendJointTarget(name, nextValue);
          activeSliderName = null;
          delete uiJoints[name];
          delete uiSources[name];
        });

        const rangeRow = document.createElement("div");
        rangeRow.className = "joint-range";
        const negativeLabel = document.createElement("span");
        const positiveLabel = document.createElement("span");
        rangeRow.appendChild(negativeLabel);
        rangeRow.appendChild(positiveLabel);

        const metaWrap = document.createElement("div");
        metaWrap.className = "joint-meta";

        const valuesRow = document.createElement("div");
        valuesRow.className = "joint-meta-row";
        const valuesText = document.createElement("span");
        const neutralButton = document.createElement("button");
        neutralButton.className = "secondary";
        neutralButton.textContent = "Neutral";
        neutralButton.addEventListener("click", async () => {
          const neutral = Number(meta.neutral ?? definition.default);
          await sendJointTarget(name, neutral);
        });
        valuesRow.appendChild(valuesText);
        valuesRow.appendChild(neutralButton);

        const note = document.createElement("div");
        note.className = "joint-note";

        const nudgeRow = document.createElement("div");
        nudgeRow.className = "toolbar";
        nudgeRow.style.marginTop = "10px";

        const minusButton = document.createElement("button");
        minusButton.className = "secondary";
        const plusButton = document.createElement("button");
        plusButton.className = "secondary";

        minusButton.addEventListener("click", async () => {
          await nudgeJoint(name, -Number(stepSize.value));
        });
        plusButton.addEventListener("click", async () => {
          await nudgeJoint(name, Number(stepSize.value));
        });

        stepSize.addEventListener("change", () => {
          minusButton.textContent = `-${stepSize.value}`;
          plusButton.textContent = `+${stepSize.value}`;
        });
        minusButton.textContent = `-${stepSize.value}`;
        plusButton.textContent = `+${stepSize.value}`;

        nudgeRow.appendChild(minusButton);
        nudgeRow.appendChild(plusButton);

        metaWrap.appendChild(valuesRow);
        metaWrap.appendChild(note);

        section.appendChild(header);
        section.appendChild(slider);
        section.appendChild(rangeRow);
        section.appendChild(metaWrap);
        section.appendChild(nudgeRow);
        jointControls.appendChild(section);

        jointElements[name] = {
          section,
          titleLine,
          targetValue,
          actualBadge,
          mappingBadge,
          invertBadge,
          slider,
          negativeLabel,
          positiveLabel,
          valuesText,
          neutralButton,
          note,
          minusButton,
          plusButton,
        };
      });
    }

    function updateJointControl(name, data) {
      const elements = jointElements[name];
      if (!elements) {
        return;
      }
      const limits = (data.limits || {})[name] || { min: -180, max: 180, default: 0 };
      const meta = (data.joint_meta || {})[name] || {};
      const jointDebug = (data.joint_debug || {})[name] || {};
      const safety = data.safety || {};
      const actualValue = Number(actualJoints[name] ?? limits.default ?? 0);
      const commandedValue = Number(
        jointDebug.commanded_joint ?? jointDebug.last_sent_joint ?? valueOrDefault(name, limits.default)
      );
      const filteredValue = Number(
        jointDebug.filtered_joint ?? commandedValue
      );
      const pendingValue = pendingValueForJoint(name);
      const sliderValue = activeSliderName === name
        ? clampJointValue(name, uiJoints[name] ?? commandedValue)
        : clampJointValue(name, commandedValue);
      const mismatch = Math.abs(commandedValue - actualValue) >= DEBUG_TARGET_MISMATCH_EPSILON;
      const activeSource = activeSourceForJoint(name, data);

      elements.titleLine.textContent = jointDisplayLabel(name, meta);
      elements.targetValue.textContent = `UI ${formatJointValue(sliderValue)}`;
      elements.actualBadge.textContent = `Actual ${formatJointValue(actualValue)}`;
      elements.mappingBadge.textContent = `PS4 ${meta.controller_mapping?.physical_ps4 || "Unmapped"}`;
      elements.invertBadge.textContent = mismatch ? "Tracking gap" : (meta.inverted ? "Inverted" : "Normal");
      elements.invertBadge.style.color = mismatch ? "#ffb3b3" : "";
      elements.slider.min = limits.min;
      elements.slider.max = limits.max;
      if (activeSliderName !== name) {
        elements.slider.value = sliderValue;
      }
      const disabled = !safety.motion_allowed || meta.enabled === false;
      elements.slider.disabled = disabled;
      elements.slider.style.opacity = disabled ? "0.45" : "1";
      elements.minusButton.disabled = disabled;
      elements.plusButton.disabled = disabled;
      elements.minusButton.style.opacity = disabled ? "0.45" : "1";
      elements.plusButton.style.opacity = disabled ? "0.45" : "1";
      elements.neutralButton.disabled = disabled;
      elements.neutralButton.style.opacity = disabled ? "0.45" : "1";
      elements.section.style.borderColor = mismatch
        ? "rgba(255,155,155,0.28)"
        : jointDebug.rate_limited
          ? "rgba(240,201,114,0.22)"
          : "rgba(255,255,255,0.08)";
      elements.negativeLabel.textContent = `${meta.semantic_negative_label || "Min"} (${limits.min})`;
      elements.positiveLabel.textContent = `${meta.semantic_positive_label || "Max"} (${limits.max})`;
      elements.valuesText.textContent =
        `UI ${formatJointValue(sliderValue)} | Pending ${pendingValue === null ? "--" : formatJointValue(pendingValue)} | Filtered ${formatJointValue(filteredValue)} | Sent ${formatJointValue(commandedValue)} | Actual ${formatJointValue(actualValue)} | Neutral ${formatJointValue(meta.neutral ?? limits.default)}`;
      const notes = [];
      if (meta.notes) {
        notes.push(meta.notes);
      }
      const virtualMap = meta.controller_mapping?.virtual;
      if (virtualMap) {
        notes.push(`Virtual: ${virtualMap}`);
      }
      notes.push(`Source: ${activeSource}`);
      notes.push(`Confidence: ${formatConfidence(jointDebug.truth_confidence)}`);
      if (jointDebug.clamped_by_limit) {
        notes.push("Clamped by joint limit.");
      }
      if (jointDebug.rate_limited) {
        notes.push(`Motion filtered: ${(jointDebug.filtered_reasons || []).join(", ") || "rate policy"}.`);
      }
      if (jointDebug.unreadable) {
        notes.push("Servo unreadable; showing cached or last-known value.");
      }
      if (jointDebug.last_error) {
        notes.push(`Last error: ${jointDebug.last_error}`);
      }
      elements.note.textContent = notes.join(" ");
    }

    function renderTruthDebug(data) {
      const debug = data.joint_debug || {};
      const values = Object.values(debug);
      const mismatchCount = values.filter((entry) => entry.target_mismatch).length;
      const unreadableCount = values.filter((entry) => entry.unreadable).length;
      const filteredCount = values.filter((entry) => entry.clamped_by_limit || entry.rate_limited).length;
      const controllerRuntime = ((data.control_runtime || {}).controller) || {};
      const confidence = values.reduce((current, entry) => {
        const next = String(entry.truth_confidence || "low").toLowerCase();
        const order = { low: 0, medium: 1, high: 2 };
        return order[next] < order[current] ? next : current;
      }, "high");

      truthSummary.innerHTML = "";
      [
        { label: "Truth Confidence", value: formatConfidence(confidence) },
        { label: "Tracking Gaps", value: String(mismatchCount) },
        { label: "Filtered / Unreadable", value: `${filteredCount} / ${unreadableCount}` },
        {
          label: "Controller Intent",
          value: controllerRuntime.intent_active
            ? `${controllerRuntime.source || "controller"} live`
            : "idle",
        },
      ].forEach((item) => {
        const tile = document.createElement("div");
        tile.className = "truth-tile";
        const label = document.createElement("span");
        label.textContent = item.label;
        const value = document.createElement("strong");
        value.textContent = item.value;
        tile.appendChild(label);
        tile.appendChild(value);
        truthSummary.appendChild(tile);
      });

      jointDebugGrid.innerHTML = "";
      Object.entries(debug).forEach(([name, entry]) => {
        const meta = (data.joint_meta || {})[name] || {};
        const card = document.createElement("div");
        card.className = "truth-card";

        const head = document.createElement("div");
        head.className = "truth-card-head";
        const titleWrap = document.createElement("div");
        titleWrap.className = "truth-card-title";
        const title = document.createElement("strong");
        title.textContent = jointDisplayLabel(name, meta);
        const subtitle = document.createElement("span");
        subtitle.textContent = `Source ${activeSourceForJoint(name, data)} | ${formatConfidence(entry.truth_confidence)}`;
        titleWrap.appendChild(title);
        titleWrap.appendChild(subtitle);

        const badge = document.createElement("span");
        badge.className = "joint-badge";
        badge.textContent = entry.unreadable ? "Unreadable" : "Readable";
        head.appendChild(titleWrap);
        head.appendChild(badge);

        const lines = document.createElement("div");
        lines.className = "truth-lines";
        const uiValue = Object.prototype.hasOwnProperty.call(uiJoints, name)
          ? formatJointValue(uiJoints[name])
          : "--";
        const pendingValue = pendingValueForJoint(name);
        const rawStepText = entry.raw_step === null || entry.raw_step === undefined ? "--" : String(entry.raw_step);
        lines.innerHTML = `
          <span>UI target: ${uiValue}</span>
          <span>Desired target: ${formatJointValue(entry.desired_joint ?? 0)}</span>
          <span>Pending target: ${pendingValue === null ? "--" : formatJointValue(pendingValue)}</span>
          <span>Commanded target: ${formatJointValue(entry.last_sent_joint ?? entry.commanded_joint ?? 0)}</span>
          <span>Actual joint: ${formatJointValue(entry.actual_joint ?? 0)}</span>
          <span>Raw step: ${rawStepText}</span>
          <span>Torque: ${entry.torque_enabled === null ? "Unknown" : entry.torque_enabled ? "Locked" : "Unlocked"}</span>
          <span>Last error: ${entry.last_error || "None"}</span>
        `;

        const flags = document.createElement("div");
        flags.className = "truth-flags";
        const flagItems = [];
        if (entry.target_mismatch) {
          flagItems.push({ text: "Target != actual", kind: "bad" });
        } else {
          flagItems.push({ text: "Tracking aligned", kind: "good" });
        }
        if (entry.clamped_by_limit) {
          flagItems.push({ text: "Clamped", kind: "warn" });
        }
        if (entry.rate_limited) {
          flagItems.push({ text: "Rate limited", kind: "warn" });
        }
        if (entry.unreadable) {
          flagItems.push({ text: "Servo unreadable", kind: "bad" });
        }
        if (!flagItems.length) {
          flagItems.push({ text: "No flags", kind: "good" });
        }
        flagItems.forEach((item) => {
          const flag = document.createElement("span");
          flag.className = `truth-flag ${item.kind}`;
          flag.textContent = item.text;
          flags.appendChild(flag);
        });

        card.appendChild(head);
        card.appendChild(lines);
        card.appendChild(flags);
        jointDebugGrid.appendChild(card);
      });
    }

    function renderVerification(data) {
      const verification = data.verification || {};
      const summary = verification.summary || {};
      const connected = Boolean(verification.connected);
      const hardwareError = verification.hardware_error;

      verificationSummary.innerHTML = "";
      [
        { label: "Verification Link", value: connected ? "Connected" : "Disconnected" },
        { label: "Joint Tracking Gaps", value: String(summary.tracking_gaps || 0) },
        { label: "Unreadable Joints", value: String(summary.unreadable || 0) },
        { label: "Clamped / Rate Limited", value: `${summary.limit_clamped || 0} / ${summary.rate_limited || 0}` },
      ].forEach((item) => {
        const tile = document.createElement("div");
        tile.className = "truth-tile";
        const label = document.createElement("span");
        label.textContent = item.label;
        const value = document.createElement("strong");
        value.textContent = item.value;
        tile.appendChild(label);
        tile.appendChild(value);
        verificationSummary.appendChild(tile);
      });

      if (hardwareError) {
        verificationStatus.textContent = `Hardware verification error: ${hardwareError}`;
      } else if (!connected) {
        verificationStatus.textContent = "Verification waiting for robot connection.";
      } else {
        verificationStatus.textContent =
          "Verification compares desired, filtered, commanded, and observed motion so we can catch mapping mistakes, unreadable servos, and safety-limit clamps quickly.";
      }
    }

    function renderState(data) {
      latestState = data;
      actualJoints = { ...(data.actual_joints || data.joints || {}) };
      commandedJoints = { ...actualJoints, ...(data.commanded_joints || {}) };
      updateSafetyUI(data);
      statusBox.textContent = JSON.stringify(data, null, 2);
      const safety = data.safety || {};
      const limits = data.limits || {};
      const joints = actualJoints;
      updateRecordingUI(data);
      if (Object.keys(jointElements).length !== Object.keys(limits).length) {
        buildJointControls(limits, data.joint_meta || {});
      }
      Object.keys(limits).forEach((name) => updateJointControl(name, data));
      renderTruthDebug(data);
      renderVerification(data);
      drawArmPreview(joints);

      poseSelect.innerHTML = "";
      (data.poses || []).forEach((poseName) => {
        const option = document.createElement("option");
        option.value = poseName;
        option.textContent = poseName;
        poseSelect.appendChild(option);
      });

      sequencePoseList.innerHTML = "";
      (data.poses || []).forEach((poseName) => {
        const pill = document.createElement("button");
        pill.className = "pill";
        pill.textContent = selectedSequencePoses.includes(poseName) ? `${poseName} [x]` : poseName;
        pill.addEventListener("click", () => {
          if (selectedSequencePoses.includes(poseName)) {
            selectedSequencePoses = selectedSequencePoses.filter((item) => item !== poseName);
          } else {
            selectedSequencePoses = [...selectedSequencePoses, poseName];
          }
          renderState(latestState);
        });
        sequencePoseList.appendChild(pill);
      });

      sequenceSelect.innerHTML = "";
      (data.sequences || []).forEach((sequenceName) => {
        const option = document.createElement("option");
        option.value = sequenceName;
        option.textContent = sequenceName;
        sequenceSelect.appendChild(option);
      });
    }

    function scheduleCommandFlush() {
      if (commandFlushTimer) {
        return;
      }
      const waitMs = Math.max(0, COMMAND_RATE_MS - (Date.now() - lastCommandSentAt));
      commandFlushTimer = setTimeout(async () => {
        commandFlushTimer = null;
        if (commandInFlight || !Object.keys(commandQueue).length) {
          return;
        }
        const targets = { ...commandQueue };
        commandQueue = {};
        const sources = { ...pendingSources };
        commandInFlight = true;
        lastCommandSentAt = Date.now();
        try {
          const response = await fetchJson("/api/joints", {
            method: "POST",
            body: JSON.stringify({
              targets,
              source: Object.values(sources).find(Boolean) || "ui",
            }),
          });
          Object.keys(targets).forEach((name) => {
            delete pendingTargets[name];
            delete pendingSources[name];
          });
          if (response.state) {
            renderState(response.state);
          } else {
            await refresh();
          }
          if (pendingFlashMessage) {
            setFlash(pendingFlashMessage);
          }
          pendingFlashMessage = null;
        } catch (error) {
          pendingFlashMessage = null;
          setFlash(error.message, true);
          await refresh();
        } finally {
          commandInFlight = false;
          if (Object.keys(commandQueue).length) {
            scheduleCommandFlush();
          }
        }
      }, waitMs);
    }

    async function queueJointTargets(targets, options = {}) {
      const nextTargets = {};
      Object.entries(targets || {}).forEach(([name, value]) => {
        const nextValue = clampJointValue(name, value);
        const currentValue = commandedValueOrDefault(name, nextValue);
        if (sameTarget(nextValue, currentValue)) {
          return;
        }
        nextTargets[name] = nextValue;
      });

      if (!Object.keys(nextTargets).length) {
        return;
      }

      Object.entries(nextTargets).forEach(([name, value]) => {
        commandQueue[name] = value;
        pendingTargets[name] = value;
        pendingSources[name] = options.source || "ui";
        if (activeSliderName !== name) {
          uiJoints[name] = value;
        }
      });

      if (options.flashMessage) {
        pendingFlashMessage = options.flashMessage;
      }

      if (latestState) {
        Object.keys(nextTargets).forEach((name) => updateJointControl(name, latestState));
      }
      scheduleCommandFlush();
    }

    async function sendJointTarget(name, value) {
      try {
        await queueJointTargets({ [name]: value }, {
          source: "slider",
          flashMessage: `Moved ${name} to ${formatJointValue(value)}.`,
        });
      } catch (error) {
        setFlash(error.message, true);
      }
    }

    async function sendJointTargets(targets, flashMessage = null, source = "ui") {
      try {
        await queueJointTargets(targets, { flashMessage, source });
      } catch (error) {
        setFlash(error.message, true);
      }
    }

    function startControllerLoop() {
      if (controllerPulse) {
        return;
      }
      controllerPulse = setInterval(() => {
        if (!latestState || !actualJoints) {
          return;
        }
        if (!latestState.safety || !latestState.safety.motion_allowed) {
          return;
        }
        const scalar = speedScalar(controllerSpeed.value);
        const left = controllerState.left;
        const right = controllerState.right;
        const buttons = controllerState.buttons;

        sendGamepadSnapshot(
          {
            axes: {
              left_x: left.x * scalar,
              left_y: left.y * scalar,
              right_x: right.x * scalar,
              right_y: right.y * scalar,
            },
            buttons: {
              l1: buttons.l1,
              r1: buttons.r1,
              l2: buttons.l2,
              r2: buttons.r2,
            },
          },
          "virtual"
        );
      }, CONTROLLER_INTENT_RATE_MS);
    }

    function updateStickVisual(stickPad, x, y) {
      const thumb = stickPad.querySelector(".stick-thumb");
      const radius = stickPad.clientWidth / 2;
      const thumbTravel = radius - 26;
      thumb.style.transform = `translate(calc(-50% + ${x * thumbTravel}px), calc(-50% + ${y * thumbTravel}px))`;
    }

    function bindStick(stickPad) {
      const stickName = stickPad.dataset.stick;
      const setStick = (clientX, clientY) => {
        const rect = stickPad.getBoundingClientRect();
        const centerX = rect.left + rect.width / 2;
        const centerY = rect.top + rect.height / 2;
        const maxRadius = rect.width / 2 - 20;
        let dx = clientX - centerX;
        let dy = clientY - centerY;
        const distance = Math.sqrt(dx * dx + dy * dy);
        if (distance > maxRadius) {
          dx = (dx / distance) * maxRadius;
          dy = (dy / distance) * maxRadius;
        }
        const x = dx / maxRadius;
        const y = dy / maxRadius;
        controllerState[stickName] = { x, y, active: true };
        updateStickVisual(stickPad, x, y);
      };
      const resetStick = () => {
        controllerState[stickName] = { x: 0, y: 0, active: false };
        updateStickVisual(stickPad, 0, 0);
      };

      stickPad.addEventListener("pointerdown", (event) => {
        if (!latestState || !latestState.safety || !latestState.safety.motion_allowed) {
          setFlash("Arm the system before using the virtual controller.", true);
          return;
        }
        stickPad.setPointerCapture(event.pointerId);
        setStick(event.clientX, event.clientY);
      });
      stickPad.addEventListener("pointermove", (event) => {
        if (!controllerState[stickName].active) {
          return;
        }
        setStick(event.clientX, event.clientY);
      });
      ["pointerup", "pointercancel", "lostpointercapture"].forEach((eventName) => {
        stickPad.addEventListener(eventName, resetStick);
      });
      updateStickVisual(stickPad, 0, 0);
    }

    function bindControllerButton(button) {
      const control = button.dataset.control;
      const setActive = (active) => {
        if (!latestState || !latestState.safety || !latestState.safety.motion_allowed) {
          if (active) {
            setFlash("Arm the system before using the virtual controller.", true);
          }
          return;
        }
        if (["l1", "r1", "l2", "r2"].includes(control)) {
          controllerState.buttons[control] = active ? 1 : 0;
        }
      };
      const down = (event) => {
        event.preventDefault();
        setActive(true);
      };
      const up = () => setActive(false);
      button.addEventListener("pointerdown", down);
      ["pointerup", "pointerleave", "pointercancel"].forEach((eventName) => {
        button.addEventListener(eventName, up);
      });
    }

    async function resetInteractiveInputs(source = "system") {
      commandQueue = {};
      pendingTargets = {};
      pendingSources = {};
      uiJoints = {};
      uiSources = {};
      activeSliderName = null;
      controllerState.left = { x: 0, y: 0, active: false };
      controllerState.right = { x: 0, y: 0, active: false };
      controllerState.buttons = { l1: 0, r1: 0, l2: 0, r2: 0 };
      document.querySelectorAll(".stick-pad").forEach((stickPad) => updateStickVisual(stickPad, 0, 0));
      await sendGamepadSnapshot(
        { axes: { left_x: 0, left_y: 0, right_x: 0, right_y: 0 }, buttons: { l1: 0, r1: 0, l2: 0, r2: 0 } },
        source,
        true
      );
      activeControllerSource = null;
    }

    async function nudgeJoint(name, delta) {
      const nextValue = valueOrDefault(name) + delta;
      await sendJointTarget(name, nextValue);
    }

    function applyDeadzone(value, deadzone = GAMEPAD_AXIS_DEADZONE) {
      if (Math.abs(value) < deadzone) {
        return 0;
      }
      const sign = Math.sign(value);
      const normalized = (Math.abs(value) - deadzone) / (1 - deadzone);
      return sign * normalized;
    }

    function applyPrecisionCurve(value, exponent = 1.9) {
      if (!value) {
        return 0;
      }
      return Math.sign(value) * Math.pow(Math.abs(value), exponent);
    }

    function buttonPressed(button) {
      if (!button) {
        return false;
      }
      if (typeof button === "object") {
        return Boolean(button.pressed) || Number(button.value || 0) > GAMEPAD_BUTTON_THRESHOLD;
      }
      return Boolean(button);
    }

    function connectedGamepads() {
      if (!navigator.getGamepads) {
        return [];
      }
      return Array.from(navigator.getGamepads()).filter((pad) => pad && pad.connected);
    }

    function gamepadActivityScore(pad) {
      const axisScore = (pad.axes || []).reduce((sum, value) => sum + Math.abs(Number(value || 0)), 0);
      const buttonScore = (pad.buttons || []).reduce((sum, button) => {
        if (!button) {
          return sum;
        }
        if (typeof button === "object") {
          return sum + (button.pressed ? 1 : 0) + Number(button.value || 0);
        }
        return sum + (button ? 1 : 0);
      }, 0);
      return axisScore + buttonScore;
    }

    function describeGamepadTransport(pad) {
      const id = String(pad.id || "").toLowerCase();
      if (id.includes("bluetooth") || id.includes("wireless")) {
        return "Bluetooth";
      }
      if (id.includes("usb") || id.includes("wired")) {
        return "USB";
      }
      if (pad.mapping === "standard") {
        return "standard mapping";
      }
      return "generic mapping";
    }

    function pickGamepad() {
      const pads = connectedGamepads();
      if (!pads.length) {
        preferredGamepadIndex = null;
        return null;
      }

      if (preferredGamepadIndex !== null) {
        const preferredPad = pads.find((pad) => pad.index === preferredGamepadIndex);
        if (preferredPad) {
          return preferredPad;
        }
      }

      pads.sort((left, right) => {
        const mappingDelta = Number(right.mapping === "standard") - Number(left.mapping === "standard");
        if (mappingDelta !== 0) {
          return mappingDelta;
        }
        const activityDelta = gamepadActivityScore(right) - gamepadActivityScore(left);
        if (activityDelta !== 0) {
          return activityDelta;
        }
        return right.index - left.index;
      });

      preferredGamepadIndex = pads[0].index;
      return pads[0];
    }

    function readAxis(pad, indexes) {
      for (const index of indexes) {
        const value = pad.axes && Number(pad.axes[index]);
        if (Number.isFinite(value)) {
          return value;
        }
      }
      return 0;
    }

    function triggerAxisValue(pad, indexes) {
      const axisValue = readAxis(pad, indexes);
      if (!axisValue) {
        return 0;
      }
      return clamp((axisValue + 1) / 2, 0, 1);
    }

    function triggerValue(pad, buttonIndex, fallbackAxisIndexes) {
      const button = pad.buttons && pad.buttons[buttonIndex];
      if (button && typeof button === "object" && Number(button.value || 0) > GAMEPAD_TRIGGER_DEADZONE) {
        return Number(button.value || 0);
      }
      return triggerAxisValue(pad, fallbackAxisIndexes);
    }

    function clamp(value, min, max) {
      return Math.max(min, Math.min(max, value));
    }

    function jointLimits(name) {
      if (!latestState || !latestState.limits) {
        return { min: -180, max: 180 };
      }
      return latestState.limits[name] || { min: -180, max: 180 };
    }

    async function tickGamepad() {
      const pad = pickGamepad();

      if (!gamepadEnabled) {
        if (activeControllerSource === "PS4") {
          sendGamepadSnapshot({ axes: {}, buttons: {} }, "PS4", true);
          activeControllerSource = null;
        }
        if (!pad) {
          setGamepadStatus("Connect a PS4 controller to your computer, then press any button with this page focused.");
        }
      } else if (!pad) {
        if (activeControllerSource === "PS4") {
          sendGamepadSnapshot({ axes: {}, buttons: {} }, "PS4", true);
          activeControllerSource = null;
        }
        setGamepadStatus("Gamepad mode is enabled, but no controller is currently detected.");
      } else {
        setGamepadStatus(
          `Using controller: ${pad.id} (${describeGamepadTransport(pad)}, slot ${pad.index})`
        );

        const now = Date.now();
        if (now - lastGamepadSendAt >= GAMEPAD_COMMAND_RATE_MS && latestState && actualJoints) {
          const leftX = readAxis(pad, [0]);
          const leftY = readAxis(pad, [1]);
          const rightX = readAxis(pad, [2, 3, 4]);
          const rightY = readAxis(pad, [3, 4, 5]);
          const l2 = triggerValue(pad, 6, [4, 2]);
          const r2 = triggerValue(pad, 7, [5, 3]);
          const l1 = buttonPressed(pad.buttons[4]);
          const r1 = buttonPressed(pad.buttons[5]);
          const homePressed = buttonPressed(pad.buttons[0]);

          const speed = speedScalar(gamepadSpeed.value);
          const snapshot = {
            axes: {
              left_x: leftX * speed,
              left_y: leftY * speed,
              right_x: rightX * speed,
              right_y: rightY * speed,
            },
            buttons: {
              l1: l1 ? speed : 0,
              r1: r1 ? speed : 0,
              l2: l2 * speed,
              r2: r2 * speed,
            },
          };
          const hasActivity =
            Object.values(snapshot.axes).some((value) => Math.abs(Number(value || 0)) > 0.001)
            || Object.values(snapshot.buttons).some((value) => Math.abs(Number(value || 0)) > 0.001);

          if (hasActivity) {
            lastGamepadSendAt = now;
            sendGamepadSnapshot(snapshot, "PS4");
          } else if (homePressed) {
            if (activeControllerSource === "PS4") {
              sendGamepadSnapshot({ axes: {}, buttons: {} }, "PS4", true);
              activeControllerSource = null;
            }
            lastGamepadSendAt = now;
            try {
              await fetchJson("/api/home", { method: "POST", body: "{}" });
              setFlash("Moved arm to home pose.");
              await refresh();
            } catch (error) {
              setFlash(error.message, true);
            }
          } else if (activeControllerSource === "PS4") {
            lastGamepadSendAt = now;
            sendGamepadSnapshot({ axes: {}, buttons: {} }, "PS4", true);
            activeControllerSource = null;
          }
        }
      }

      requestAnimationFrame(tickGamepad);
    }

    function startGamepadLoopOnce() {
      if (gamepadLoopStarted) {
        return;
      }
      gamepadLoopStarted = true;
      requestAnimationFrame(tickGamepad);
    }

    async function refresh() {
      try {
        const data = await fetchJson("/api/status");
        renderState(data);
      } catch (error) {
        setFlash(error.message, true);
      }
    }

    document.getElementById("connectBtn").addEventListener("click", async () => {
      const response = await fetchJson("/api/connect", { method: "POST", body: "{}" });
      setFlash("Arm connected.");
      renderState(response.state || latestState || {});
    });

    document.getElementById("disconnectBtn").addEventListener("click", async () => {
      await sendGamepadSnapshot(
        { axes: { left_x: 0, left_y: 0, right_x: 0, right_y: 0 }, buttons: { l1: 0, r1: 0, l2: 0, r2: 0 } },
        "system",
        true
      );
      const response = await fetchJson("/api/disconnect", { method: "POST", body: "{}" });
      setFlash("Arm disconnected.");
      renderState(response.state || latestState || {});
    });

    document.getElementById("homeBtn").addEventListener("click", async () => {
      try {
        await fetchJson("/api/home", { method: "POST", body: "{}" });
        setFlash("Moved arm to home pose.");
      } catch (error) {
        setFlash(error.message, true);
      }
      await refresh();
    });

    document.getElementById("refreshBtn").addEventListener("click", refresh);
    gamepadToggleBtn.addEventListener("click", () => {
      if (latestState && latestState.safety && !latestState.safety.motion_allowed) {
        setFlash("Arm the system before enabling gamepad control.", true);
        return;
      }
      gamepadEnabled = !gamepadEnabled;
      gamepadToggleBtn.textContent = gamepadEnabled ? "Disable" : "Enable";
      if (gamepadEnabled) {
        setFlash("Gamepad control enabled.");
      } else {
        sendGamepadSnapshot({ axes: {}, buttons: {} }, "PS4", true);
        activeControllerSource = null;
        setFlash("Gamepad control disabled.");
      }
    });

    window.addEventListener("gamepadconnected", (event) => {
      preferredGamepadIndex = event.gamepad.index;
      setGamepadStatus(
        `Detected controller: ${event.gamepad.id} (${describeGamepadTransport(event.gamepad)}, slot ${event.gamepad.index})`
      );
      startGamepadLoopOnce();
    });

    window.addEventListener("gamepaddisconnected", (event) => {
      if (preferredGamepadIndex === event.gamepad.index) {
        preferredGamepadIndex = null;
      }
      sendGamepadSnapshot({ axes: {}, buttons: {} }, "PS4", true);
      activeControllerSource = null;
      setGamepadStatus("Controller disconnected.");
    });

    armBtn.addEventListener("click", async () => {
      await safetyAction("/api/safety/arm", "System armed.", { resetInputs: true, source: "arm" });
    });
    mobileArmBtn.addEventListener("click", async () => {
      await safetyAction("/api/safety/arm", "System armed.", { resetInputs: true, source: "arm" });
    });
    disarmBtn.addEventListener("click", async () => {
      await safetyAction("/api/safety/disarm", "System disarmed.", { resetInputs: true, source: "disarm" });
    });
    lockBtn.addEventListener("click", async () => {
      await safetyAction("/api/torque_on", "Servo torque enabled. Arm locked in place.", { resetInputs: true, source: "lock" });
    });
    unlockBtn.addEventListener("click", async () => {
      await safetyAction("/api/torque_off", "Servo torque disabled. Arm unlocked.", { resetInputs: true, source: "unlock" });
    });
    resetBtn.addEventListener("click", async () => {
      await safetyAction("/api/safety/reset", "Emergency stop cleared.", { resetInputs: true, source: "reset" });
    });
    estopBtn.addEventListener("click", async () => {
      await safetyAction("/api/safety/estop", "Emergency stop triggered.", { resetInputs: true, source: "estop" });
    });
    mobileStopBtn.addEventListener("click", async () => {
      await safetyAction("/api/safety/estop", "Emergency stop triggered.", { resetInputs: true, source: "estop" });
    });

    document.querySelectorAll(".stick-pad").forEach(bindStick);
    document.querySelectorAll(".controller-button[data-control]").forEach(bindControllerButton);
    startControllerLoop();

    document.getElementById("savePoseBtn").addEventListener("click", async () => {
      const name = document.getElementById("poseName").value;
      try {
        await fetchJson("/api/pose/save", {
          method: "POST",
          body: JSON.stringify({ name }),
        });
        setFlash(`Saved pose "${name}".`);
      } catch (error) {
        setFlash(error.message, true);
      }
      await refresh();
    });

    document.getElementById("loadPoseBtn").addEventListener("click", async () => {
      try {
        await fetchJson("/api/pose/load", {
          method: "POST",
          body: JSON.stringify({ name: poseSelect.value }),
        });
        setFlash(`Loaded pose "${poseSelect.value}".`);
      } catch (error) {
        setFlash(error.message, true);
      }
      await refresh();
    });

    document.getElementById("deletePoseBtn").addEventListener("click", async () => {
      try {
        await fetchJson("/api/pose/delete", {
          method: "POST",
          body: JSON.stringify({ name: poseSelect.value }),
        });
        setFlash(`Deleted pose "${poseSelect.value}".`);
      } catch (error) {
        setFlash(error.message, true);
      }
      await refresh();
    });

    document.getElementById("saveSequenceBtn").addEventListener("click", async () => {
      const name = document.getElementById("sequenceName").value;
      try {
        await fetchJson("/api/sequence/save", {
          method: "POST",
          body: JSON.stringify({ name, poses: selectedSequencePoses }),
        });
        setFlash(`Saved sequence "${name}" with ${selectedSequencePoses.length} poses.`);
      } catch (error) {
        setFlash(error.message, true);
      }
      await refresh();
    });

    document.getElementById("viewSequenceBtn").addEventListener("click", async () => {
      try {
        const response = await fetchJson("/api/sequence/load", {
          method: "POST",
          body: JSON.stringify({ name: sequenceSelect.value }),
        });
        const poses = response.sequence.poses || [];
        setFlash(`Sequence "${response.sequence.name}" -> ${poses.join(" -> ") || "empty"}`);
      } catch (error) {
        setFlash(error.message, true);
      }
    });

    document.getElementById("deleteSequenceBtn").addEventListener("click", async () => {
      try {
        await fetchJson("/api/sequence/delete", {
          method: "POST",
          body: JSON.stringify({ name: sequenceSelect.value }),
        });
        setFlash(`Deleted sequence "${sequenceSelect.value}".`);
      } catch (error) {
        setFlash(error.message, true);
      }
      await refresh();
    });

    recordBtn.addEventListener("click", async () => {
      try {
        await resetInteractiveInputs("recording");
        const response = await fetchJson("/api/recording/start", { method: "POST", body: "{}" });
        setFlash("Recording started. Move the arm freely by hand.");
        renderState(response.state || latestState || {});
      } catch (error) {
        setFlash(error.message, true);
      }
    });

    stopRecordBtn.addEventListener("click", async () => {
      try {
        await resetInteractiveInputs("recording");
        const response = await fetchJson("/api/recording/stop", { method: "POST", body: "{}" });
        setFlash("Recording stopped. Press Play to replay the captured movement.");
        renderState(response.state || latestState || {});
      } catch (error) {
        setFlash(error.message, true);
      }
    });

    playRecordBtn.addEventListener("click", async () => {
      try {
        await resetInteractiveInputs("playback");
        const response = await fetchJson("/api/recording/play", { method: "POST", body: "{}" });
        setFlash("Playback complete.");
        renderState(response.state || latestState || {});
      } catch (error) {
        setFlash(error.message, true);
      }
    });

    refresh();
    startGamepadLoopOnce();
    setInterval(() => {
      if (commandInFlight || Object.keys(commandQueue).length || activeSliderName) {
        return;
      }
      refresh();
    }, STATUS_POLL_MS);
  </script>
</body>
</html>
"""


if __name__ == "__main__":
    app.run(
        host=config["app"].get("host", "0.0.0.0"),
        port=int(config["app"].get("port", 7001)),
        debug=False,
    )
