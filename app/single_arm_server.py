from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Dict

from flask import Flask, jsonify, render_template_string, request

BASE_DIR = Path(__file__).resolve().parent.parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

from app.robot_arm import (
    build_adapter,
    build_joint_definitions,
    default_joint_positions,
    load_config,
)


DEFAULT_CONFIG = BASE_DIR / "config" / "config.example.yaml"
POSES_DIR = BASE_DIR / "data" / "poses"
SEQUENCES_DIR = BASE_DIR / "data" / "sequences"
POSES_DIR.mkdir(parents=True, exist_ok=True)
SEQUENCES_DIR.mkdir(parents=True, exist_ok=True)

app = Flask(__name__)

config_path = Path(os.environ.get("QUBIT_CONFIG", DEFAULT_CONFIG))
config = load_config(config_path)
joint_definitions = build_joint_definitions(config)
adapter = build_adapter(config)


def clamp_targets(raw_targets: Dict[str, float]) -> Dict[str, float]:
    clamped = {}
    for name, value in raw_targets.items():
        definition = joint_definitions.get(name)
        if definition is None:
            continue
        numeric_value = float(value)
        clamped[name] = max(definition.minimum, min(definition.maximum, numeric_value))
    return clamped


def pose_path(name: str) -> Path:
    safe_name = "".join(ch for ch in name.lower() if ch.isalnum() or ch in {"-", "_"})
    if not safe_name:
        raise ValueError("Pose name must include letters or numbers.")
    return POSES_DIR / f"{safe_name}.json"


def current_payload():
    state = adapter.get_state()
    return {
        "robot": {
            "name": config["robot"]["name"],
            "driver": config["robot"]["driver"],
            "connected": state["connected"],
        },
        "joints": state["joints"],
        "limits": {
            name: {
                "min": definition.minimum,
                "max": definition.maximum,
                "default": definition.default,
            }
            for name, definition in joint_definitions.items()
        },
        "poses": sorted(path.stem for path in POSES_DIR.glob("*.json")),
        "sequences": sorted(path.stem for path in SEQUENCES_DIR.glob("*.json")),
    }


def error_response(message: str, status_code: int = 400):
    return jsonify({"ok": False, "error": message}), status_code


def sequence_path(name: str) -> Path:
    safe_name = "".join(ch for ch in name.lower() if ch.isalnum() or ch in {"-", "_"})
    if not safe_name:
        raise ValueError("Sequence name must include letters or numbers.")
    return SEQUENCES_DIR / f"{safe_name}.json"


@app.get("/")
def index():
    return render_template_string(HTML, robot_name=config["robot"]["name"])


@app.get("/api/health")
def health():
    return jsonify({"ok": True, "config_path": str(config_path)})


@app.post("/api/connect")
def connect():
    connected = adapter.connect()
    return jsonify({"ok": connected, "state": current_payload()})


@app.post("/api/disconnect")
def disconnect():
    adapter.disconnect()
    return jsonify({"ok": True, "state": current_payload()})


@app.get("/api/status")
def status():
    return jsonify(current_payload())


@app.post("/api/joints")
def move_joints():
    payload = request.get_json(silent=True) or {}
    targets = clamp_targets(payload.get("targets", {}))
    try:
        joints = adapter.move_joints(targets)
    except Exception as exc:
        return error_response(str(exc), 409)
    return jsonify({"ok": True, "joints": joints})


@app.post("/api/torque_off")
def torque_off():
    adapter.disable_torque()
    return jsonify({"ok": True, "state": current_payload()})


@app.post("/api/pose/save")
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
def load_pose():
    payload = request.get_json(silent=True) or {}
    name = payload.get("name", "").strip()
    try:
        path = pose_path(name)
    except ValueError as exc:
        return error_response(str(exc))
    if not path.exists():
        return error_response(f"Pose '{name}' does not exist.", 404)
    with path.open("r", encoding="utf-8") as handle:
        pose = json.load(handle)
    try:
        joints = adapter.move_joints(clamp_targets(pose["joints"]))
    except Exception as exc:
        return error_response(str(exc), 409)
    return jsonify({"ok": True, "joints": joints, "state": current_payload()})


@app.post("/api/pose/delete")
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
def home():
    defaults = default_joint_positions(joint_definitions)
    try:
        joints = adapter.move_joints(defaults)
    except Exception:
        # Mock mode and disconnected real hardware should still let the UI reset.
        reset = getattr(adapter, "reset_to_defaults", None)
        if callable(reset):
            joints = reset()
        else:
            return error_response("Unable to move to home while robot is disconnected.", 409)
    return jsonify({"ok": True, "joints": joints, "state": current_payload()})


@app.post("/api/sequence/save")
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


@app.post("/api/sequence/delete")
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
    @media (max-width: 860px) {
      .hero {
        grid-template-columns: 1fr;
      }
      .layout {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>
<body>
  <div class="shell">
    <section class="hero">
      <div class="hero-copy">
        <div class="eyebrow">Manual Control Interface</div>
        <h1>{{ robot_name }}</h1>
        <div class="subhead">
          A local control console for testing one robotic arm, tuning input feel, and validating the operator workflow before you spend money on hardware.
        </div>
      </div>
      <div class="hero-meta">
        <div class="meta-kicker">System Overview</div>
        <div class="status-grid">
          <div class="status-tile">
            <span class="status-label">Mode</span>
            <span class="status-value">Mock</span>
          </div>
          <div class="status-tile">
            <span class="status-label">Input</span>
            <span class="status-value">Web + PS4</span>
          </div>
          <div class="status-tile">
            <span class="status-label">Arm Count</span>
            <span class="status-value">1</span>
          </div>
          <div class="status-tile">
            <span class="status-label">Focus</span>
            <span class="status-value">Manual</span>
          </div>
        </div>
        <div class="hero-note">
          Design goal: feel like a high-end control rig now, then map cleanly onto a real USB or serial-connected arm later.
        </div>
      </div>
    </section>

    <section class="layout">
      <div class="card">
        <div class="toolbar">
          <div class="control-title">
            <span>Joint Control</span>
            <span>Direct Manipulation</span>
          </div>
          <div>
            <button id="connectBtn">Connect</button>
            <button class="secondary" id="disconnectBtn">Disconnect</button>
            <button class="secondary" id="homeBtn">Home</button>
            <button class="secondary" id="refreshBtn">Refresh</button>
          </div>
        </div>
        <div class="stack">
          <section class="joint">
            <div class="toolbar">
              <strong>PS4 Controller Test</strong>
              <button class="secondary" id="gamepadToggleBtn">Enable</button>
            </div>
            <div class="hint" id="gamepadStatus" style="margin-top: 10px;">
              Connect a PS4 controller to your computer, then press any button with this page focused.
            </div>
            <div class="toolbar" style="margin-top: 12px;">
              <label for="gamepadSpeed"><strong>Gamepad Speed</strong></label>
              <select id="gamepadSpeed">
                <option value="2">Slow</option>
                <option value="4" selected>Medium</option>
                <option value="7">Fast</option>
              </select>
            </div>
            <div class="hint" style="margin-top: 12px;">
              Left stick: shoulder pan and shoulder lift. Right stick: wrist roll and elbow. L2/R2: wrist flex. D-pad up/down: gripper.
            </div>
          </section>
          <div class="toolbar">
            <div>
              <strong>Step Size</strong>
            </div>
            <select id="stepSize">
              <option value="1">1 degree</option>
              <option value="5" selected>5 degrees</option>
              <option value="10">10 degrees</option>
            </select>
          </div>
        </div>
        <div class="controls" id="jointControls"></div>
      </div>

      <div class="card">
        <div class="control-title">
          <span>Arm Preview</span>
          <span>Live Telemetry View</span>
        </div>
        <div id="flashBox" class="status flash-status">Ready.</div>
        <div class="preview-shell">
          <canvas id="armPreview" width="360" height="280" style="width: 100%; border-radius: 18px; background: transparent; border: 0;"></canvas>
        </div>
        <div class="hud-line"></div>
        <div class="pose-row">
          <input id="poseName" placeholder="pose name">
          <button id="savePoseBtn">Save Pose</button>
        </div>
        <div class="pose-row">
          <select id="poseSelect"></select>
          <button class="secondary" id="loadPoseBtn">Load Pose</button>
          <button class="secondary" id="deletePoseBtn">Delete Pose</button>
        </div>
        <div class="pill-list" id="sequencePoseList"></div>
        <div class="sequence-row">
          <input id="sequenceName" placeholder="sequence name">
          <button id="saveSequenceBtn">Save Sequence</button>
        </div>
        <div class="sequence-row">
          <select id="sequenceSelect"></select>
          <button class="secondary" id="viewSequenceBtn">View Sequence</button>
          <button class="secondary" id="deleteSequenceBtn">Delete Sequence</button>
        </div>
        <div class="panel-section control-title">
          <span>State</span>
          <span>Raw System Snapshot</span>
        </div>
        <div class="status" id="statusBox">Loading...</div>
      </div>
    </section>
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
    const sequencePoseList = document.getElementById("sequencePoseList");
    const sequenceSelect = document.getElementById("sequenceSelect");
    const previewCanvas = document.getElementById("armPreview");
    const previewContext = previewCanvas.getContext("2d");
    let latestState = null;
    let selectedSequencePoses = [];
    let gamepadEnabled = false;
    let lastGamepadSendAt = 0;
    let gamepadLoopStarted = false;

    async function fetchJson(url, options = {}) {
      const response = await fetch(url, {
        headers: { "Content-Type": "application/json" },
        ...options,
      });
      const data = await response.json();
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

    function drawArmPreview(joints) {
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

    function renderState(data) {
      latestState = data;
      statusBox.textContent = JSON.stringify(data, null, 2);
      const limits = data.limits || {};
      const joints = data.joints || {};
      jointControls.innerHTML = "";
      drawArmPreview(joints);

      Object.entries(limits).forEach(([name, definition]) => {
        const section = document.createElement("section");
        section.className = "joint";
        const label = document.createElement("div");
        label.className = "joint-header";
        label.innerHTML = `<strong>${name}</strong><span>${joints[name]}</span>`;

        const slider = document.createElement("input");
        slider.type = "range";
        slider.min = definition.min;
        slider.max = definition.max;
        slider.step = "1";
        slider.value = joints[name];
        slider.addEventListener("input", () => {
          label.innerHTML = `<strong>${name}</strong><span>${slider.value}</span>`;
        });
        slider.addEventListener("change", async () => {
          await sendJointTarget(name, Number(slider.value));
        });

        const nudgeRow = document.createElement("div");
        nudgeRow.className = "toolbar";
        nudgeRow.style.marginTop = "10px";

        const minusButton = document.createElement("button");
        minusButton.className = "secondary";
        minusButton.textContent = `-${stepSize.value}`;
        minusButton.addEventListener("click", async () => {
          await nudgeJoint(name, -Number(stepSize.value));
        });

        const plusButton = document.createElement("button");
        plusButton.className = "secondary";
        plusButton.textContent = `+${stepSize.value}`;
        plusButton.addEventListener("click", async () => {
          await nudgeJoint(name, Number(stepSize.value));
        });

        stepSize.addEventListener("change", () => {
          minusButton.textContent = `-${stepSize.value}`;
          plusButton.textContent = `+${stepSize.value}`;
        });

        nudgeRow.appendChild(minusButton);
        nudgeRow.appendChild(plusButton);

        section.appendChild(label);
        section.appendChild(slider);
        section.appendChild(nudgeRow);
        jointControls.appendChild(section);
      });

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
        pill.textContent = selectedSequencePoses.includes(poseName) ? `${poseName} ✓` : poseName;
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

    async function sendJointTarget(name, value) {
      try {
        await fetchJson("/api/joints", {
          method: "POST",
          body: JSON.stringify({ targets: { [name]: value } }),
        });
        setFlash(`Moved ${name} to ${value}.`);
        await refresh();
      } catch (error) {
        setFlash(error.message, true);
      }
    }

    async function sendJointTargets(targets, flashMessage = null) {
      try {
        await fetchJson("/api/joints", {
          method: "POST",
          body: JSON.stringify({ targets }),
        });
        if (flashMessage) {
          setFlash(flashMessage);
        }
        await refresh();
      } catch (error) {
        setFlash(error.message, true);
      }
    }

    async function nudgeJoint(name, delta) {
      const joints = (latestState && latestState.joints) || {};
      const nextValue = Number(joints[name] || 0) + delta;
      await sendJointTarget(name, nextValue);
    }

    function applyDeadzone(value, deadzone = 0.18) {
      if (Math.abs(value) < deadzone) {
        return 0;
      }
      return value;
    }

    function buttonPressed(button) {
      if (!button) {
        return false;
      }
      if (typeof button === "object") {
        return Boolean(button.pressed) || button.value > 0.5;
      }
      return Boolean(button);
    }

    async function tickGamepad() {
      const pads = navigator.getGamepads ? navigator.getGamepads() : [];
      const pad = pads && pads[0];

      if (!gamepadEnabled) {
        if (!pad) {
          setGamepadStatus("Connect a PS4 controller to your computer, then press any button with this page focused.");
        }
      } else if (!pad) {
        setGamepadStatus("Gamepad mode is enabled, but no controller is currently detected.");
      } else {
        setGamepadStatus(`Using controller: ${pad.id}`);

        const now = Date.now();
        if (now - lastGamepadSendAt >= 90 && latestState && latestState.joints) {
          const joints = latestState.joints;
          const speed = Number(gamepadSpeed.value);

          const leftX = applyDeadzone(pad.axes[0] || 0);
          const leftY = applyDeadzone(pad.axes[1] || 0);
          const rightX = applyDeadzone(pad.axes[2] || 0);
          const rightY = applyDeadzone(pad.axes[3] || 0);
          const l2 = pad.buttons[6] ? pad.buttons[6].value : 0;
          const r2 = pad.buttons[7] ? pad.buttons[7].value : 0;
          const dpadUp = buttonPressed(pad.buttons[12]);
          const dpadDown = buttonPressed(pad.buttons[13]);
          const homePressed = buttonPressed(pad.buttons[0]);

          const targets = {};
          if (leftX) {
            targets.shoulder_pan = Number(joints.shoulder_pan) + leftX * speed;
          }
          if (leftY) {
            targets.shoulder_lift = Number(joints.shoulder_lift) - leftY * speed;
          }
          if (rightX) {
            targets.wrist_roll = Number(joints.wrist_roll) + rightX * speed;
          }
          if (rightY) {
            targets.elbow_flex = Number(joints.elbow_flex) - rightY * speed;
          }
          if (l2 || r2) {
            targets.wrist_flex = Number(joints.wrist_flex) + (r2 - l2) * speed;
          }
          if (dpadUp || dpadDown) {
            const gripperDelta = dpadUp ? speed : -speed;
            targets.gripper = Number(joints.gripper) + gripperDelta;
          }

          if (Object.keys(targets).length > 0) {
            lastGamepadSendAt = now;
            await sendJointTargets(targets);
          } else if (homePressed) {
            lastGamepadSendAt = now;
            try {
              await fetchJson("/api/home", { method: "POST", body: "{}" });
              setFlash("Moved arm to home pose.");
              await refresh();
            } catch (error) {
              setFlash(error.message, true);
            }
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
      await fetchJson("/api/connect", { method: "POST", body: "{}" });
      setFlash("Arm connected.");
      await refresh();
    });

    document.getElementById("disconnectBtn").addEventListener("click", async () => {
      await fetchJson("/api/disconnect", { method: "POST", body: "{}" });
      setFlash("Arm disconnected.");
      await refresh();
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
      gamepadEnabled = !gamepadEnabled;
      gamepadToggleBtn.textContent = gamepadEnabled ? "Disable" : "Enable";
      if (gamepadEnabled) {
        setFlash("Gamepad control enabled.");
      } else {
        setFlash("Gamepad control disabled.");
      }
    });

    window.addEventListener("gamepadconnected", (event) => {
      setGamepadStatus(`Detected controller: ${event.gamepad.id}`);
      startGamepadLoopOnce();
    });

    window.addEventListener("gamepaddisconnected", () => {
      setGamepadStatus("Controller disconnected.");
    });

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

    refresh();
    startGamepadLoopOnce();
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
