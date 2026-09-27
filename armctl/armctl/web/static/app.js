"use strict";

const POLL_MS = 50;        // telemetry refresh
const SEND_MS = 33;        // slider -> server, coalesced
const PAD_MS = 33;         // gamepad -> server
const GRACE_MS = 450;      // keep telemetry off a slider this long after a touch
const THUMB_PX = 15;

const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];
const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

let telemetry = null;
let poses = [];
let recording = { recording: false, playing: false, frames: 0 };

// --- slider ownership -------------------------------------------------------
// The single rule that stops snapback: telemetry may not write to a slider the
// user is touching, or touched within GRACE_MS. Nothing else guards this.
const dragging = new Set();
const touchedAt = new Map();

const pending = new Map();
let sendTimer = null;

function owned(name) {
  return dragging.has(name) || (performance.now() - (touchedAt.get(name) ?? -1e9)) < GRACE_MS;
}

// --- transport --------------------------------------------------------------

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  let body = {};
  try { body = await response.json(); } catch { /* empty body is fine */ }
  if (!response.ok || body.ok === false) {
    throw new Error(body.error || `${response.status} ${response.statusText}`);
  }
  return body;
}

function queueGoal(name, value) {
  pending.set(name, value);
  if (sendTimer) return;
  sendTimer = setTimeout(async () => {
    sendTimer = null;
    if (!pending.size) return;
    const joints = Object.fromEntries(pending);
    pending.clear();
    try {
      await api("/api/goal", { method: "POST", body: JSON.stringify({ joints, source: "slider" }) });
    } catch (error) {
      toast(error.message, true);
    }
  }, SEND_MS);
}

// --- rendering --------------------------------------------------------------

function pill(el, cls, text) {
  el.className = `pill ${cls}`;
  $("span", el).textContent = text;
}

function render(data) {
  telemetry = data;
  const motion = data.motion_allowed;

  pill($("#linkPill"), data.connected ? "good" : "bad",
       data.connected ? "Bus online" : "Bus offline");
  pill($("#armPill"), data.estopped ? "bad" : motion ? "good" : "warn",
       data.estopped ? "E-STOP" : motion ? "Armed" : "Disarmed");

  const volts = data.supply_volts;
  const voltPill = $("#voltPill");
  voltPill.hidden = volts === null || volts === undefined;
  if (!voltPill.hidden) {
    pill(voltPill, data.undervolt ? "bad" : "good", `${volts.toFixed(1)} V`);
    voltPill.title = data.undervolt
      ? `Below the configured floor for ${data.nominal_volts} V servos. Low `
        + "voltage causes overload faults with nothing actually jammed."
      : `Servo supply voltage (nominal ${data.nominal_volts} V)`;
  }

  $("#loopStat").textContent =
    `${data.loop.actual_hz.toFixed(0)} Hz · ${data.loop.tick_ms.toFixed(1)} ms`;
  $("#statusLine").textContent = data.status || "";

  $("#btnArm").disabled = motion || data.estopped || !data.connected;
  $("#btnDisarm").disabled = !motion;
  $("#btnHome").disabled = !motion;
  $("#btnPlay").disabled = !motion || !recording.frames;
  $("#btnReset").hidden = !data.estopped;

  for (const joint of data.joints) {
    const row = $(`.joint[data-joint="${joint.name}"]`);
    if (!row) continue;

    const slider = $("[data-slider]", row);
    slider.disabled = !motion;
    if (!owned(joint.name)) slider.value = joint.goal;

    const span = joint.hi - joint.lo;
    const place = (el, value) => {
      const pct = clamp((value - joint.lo) / span, 0, 1);
      el.style.left = `calc(${THUMB_PX / 2}px + ${pct} * (100% - ${THUMB_PX}px))`;
    };
    if (joint.observed !== null) place($("[data-ghost]", row), joint.observed);
    $("[data-ghost]", row).hidden = joint.observed === null;
    place($("[data-home]", row), Number(slider.dataset.home ?? joint.goal));

    const shown = joint.observed !== null ? joint.observed : joint.setpoint;
    $("[data-value]", row).textContent = shown.toFixed(1);

    const lag = $("[data-lag]", row);
    const gap = joint.error === null ? 0 : Math.abs(joint.error);
    lag.textContent = gap >= 0.5 ? ` Δ${gap.toFixed(1)}` : "";
    lag.classList.toggle("hot", gap >= 5);

    // A released joint is the loud case — it is deliberately not under torque.
    // Advisory bits (voltage sag) are shown quietly so they stop reading as
    // failures; they no longer stop the arm.
    const fault = $("[data-fault]", row);
    if (joint.frozen) {
      fault.hidden = false;
      fault.textContent = "OUT OF RANGE · easing back";
    } else if (joint.relieved) {
      fault.hidden = false;
      fault.textContent = "RELEASED · stalled";
    } else if (joint.faults.length) {
      fault.hidden = false;
      fault.textContent = joint.faults.join(", ");
    } else if (joint.warnings.length) {
      fault.hidden = false;
      fault.textContent = joint.warnings.join(", ");
    } else {
      fault.hidden = true;
    }
    fault.classList.toggle("soft",
      !joint.relieved && !joint.frozen && !joint.faults.length);

    const slider2 = $("[data-slider]", row);
    if (joint.relieved) slider2.disabled = true;
    row.classList.toggle("stale", !joint.readable);
  }

  const unreleased = data.torque_release_failed || [];
  const alarm = $("#alarm");
  alarm.hidden = unreleased.length === 0;
  if (unreleased.length) {
    alarm.textContent =
      `TORQUE NOT CONFIRMED OFF: ${unreleased.join(", ")} — the arm may still be `
      + "powered and holding. Cut power at the supply. Retrying every second.";
  }

  $("#btnRecover").hidden = !data.joints.some((j) => j.relieved);
  renderCalibration(data.calibration);

  renderDiagnostics(data);
}

function renderDiagnostics(data) {
  const unreadable = data.joints.filter((j) => !j.readable).map((j) => j.name);
  const faulted = data.joints.filter((j) => j.faults.length).map((j) => j.name);
  const errors = data.joints.reduce((sum, j) => sum + j.read_errors, 0);
  const worst = data.joints.reduce(
    (max, j) => Math.max(max, j.error === null ? 0 : Math.abs(j.error)), 0);

  const rows = [
    ["Control loop", `${data.loop.actual_hz.toFixed(1)} / ${data.loop.target_hz} Hz`],
    ["Tick time", `${data.loop.tick_ms.toFixed(2)} ms`],
    ["Sync writes", data.loop.writes.toLocaleString()],
    ["Active source", data.source],
    ["Worst tracking gap", `${worst.toFixed(2)}°`],
    ["Supply voltage", data.supply_volts === null || data.supply_volts === undefined
      ? "unknown"
      : `${data.supply_volts.toFixed(1)} V / ${data.nominal_volts} V nominal`
        + `${data.undervolt ? "  ** LOW **" : ""}`],
    ["Hottest servo", (() => {
      const temps = data.joints.map((j) => j.temp_c).filter((v) => v !== null);
      return temps.length ? `${Math.max(...temps)} °C` : "unknown";
    })()],
    ["Peak servo effort", (() => {
      const loads = data.joints.map((j) => j.load).filter((v) => v !== null);
      if (!loads.length) return "unknown";
      const peak = Math.max(...loads.map(Math.abs));
      return `${Math.round(peak / 10)}%${peak >= 900 ? "  ** AT LIMIT **" : ""}`;
    })()],
    ["Read errors", errors],
    ["Unreadable", unreadable.length ? unreadable.join(", ") : "none"],
    ["Faults", faulted.length ? faulted.join(", ") : "none"],
  ];
  if (data.last_error) rows.push(["Last error", data.last_error]);

  $("#diag").innerHTML = rows
    .map(([k, v]) => `<div class="stat"><span>${k}</span><b>${escapeHtml(String(v))}</b></div>`)
    .join("");
}

function escapeHtml(text) {
  return text.replace(/[&<>"']/g, (c) =>
    ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
}

let toastTimer = null;
function toast(message, isError = false) {
  const el = $("#toast");
  el.textContent = message;
  el.className = `toast show${isError ? " err" : ""}`;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove("show"), 3200);
}

// --- polling ----------------------------------------------------------------

async function poll() {
  try {
    render(await api("/api/telemetry"));
  } catch (error) {
    if (String(error.message).startsWith("401")) return location.reload();
    pill($("#linkPill"), "bad", "No server");
  }
  setTimeout(poll, POLL_MS);
}

async function refreshPoses() {
  try {
    poses = (await api("/api/poses")).poses;
  } catch { return; }
  const list = $("#poseList");
  if (!poses.length) {
    list.innerHTML = '<div class="empty">No poses saved.</div>';
    return;
  }
  list.innerHTML = poses
    .map((name) => `<div class="item"><span class="nm">${escapeHtml(name)}</span>
      <button class="btn sm" data-go="${escapeHtml(name)}">Go</button>
      <button class="btn sm danger" data-del="${escapeHtml(name)}">✕</button></div>`)
    .join("");
}

async function refreshRecording() {
  try {
    recording = await api("/api/record");
  } catch { return; }
  $("#recStat").textContent =
    recording.recording ? "recording…" : recording.playing ? "playing…" : `${recording.frames} frames`;
  $("#btnRec").textContent = recording.recording ? "Stop" : "Record";
  $("#btnPlay").textContent = recording.playing ? "Stop" : "Play";
}

// --- gamepad ----------------------------------------------------------------

const padUi = {
  nubL: $("#nubL"), nubR: $("#nubR"),
  barL1: $("#barL1"), barR1: $("#barR1"), barL2: $("#barL2"), barR2: $("#barR2"),
};
let padOn = false;
let padSentAt = 0;
let padWasActive = false;

function readPad() {
  const pad = [...(navigator.getGamepads?.() ?? [])].find(Boolean);
  if (!pad) return null;
  const axis = (i) => (pad.axes.length > i ? pad.axes[i] : 0);
  const button = (i) => (pad.buttons.length > i ? pad.buttons[i].value : 0);
  return {
    axes: { left_x: axis(0), left_y: axis(1), right_x: axis(2), right_y: axis(3) },
    buttons: { l1: button(4), r1: button(5), l2: button(6), r2: button(7) },
  };
}

function drawPad(axes, buttons) {
  padUi.nubL.style.transform =
    `translate(-50%,-50%) translate(${(axes.left_x || 0) * 26}px, ${(axes.left_y || 0) * 26}px)`;
  padUi.nubR.style.transform =
    `translate(-50%,-50%) translate(${(axes.right_x || 0) * 26}px, ${(axes.right_y || 0) * 26}px)`;
  for (const key of ["l1", "r1", "l2", "r2"]) {
    padUi[`bar${key.toUpperCase()}`].style.width = `${Math.abs(buttons[key] || 0) * 100}%`;
  }
}

/* The pad plugged into the host is read server-side and never reaches this
 * page as input, so without this poll the panel looks dead even while the pad
 * is driving the arm. Mirror it here so "is my controller working?" is
 * answerable by looking at the screen. */
async function pollWiredPad() {
  if (padOn) return;
  let pad;
  try { pad = await api("/api/pad"); } catch { return; }

  if (!pad.available) {
    $("#padHint").textContent = pad.reason || "Wired pad reader is off.";
    return;
  }
  if (!pad.connected) {
    $("#padHint").textContent =
      pad.error || "No controller on the host. Plug one into the laptop, or tick Browser pad.";
    drawPad({}, {});
    return;
  }
  drawPad(pad.axes || {}, pad.buttons || {});
  $("#padHint").textContent =
    `${pad.device} on the host${pad.active ? " · driving" : " · idle"}`;
}

function padLoop() {
  requestAnimationFrame(padLoop);
  if (!padOn) return;

  const snapshot = readPad();
  if (!snapshot) {
    $("#padHint").textContent = "No gamepad seen by this browser. Press a button on it.";
    return;
  }

  const { axes, buttons } = snapshot;
  drawPad(axes, buttons);

  const active = Object.values(axes).some((v) => Math.abs(v) > 0.12)
    || Object.values(buttons).some((v) => v > 0.05);
  const now = performance.now();
  if (now - padSentAt < PAD_MS) return;
  if (!active && !padWasActive) return;
  padSentAt = now;
  padWasActive = active;

  if (!telemetry?.motion_allowed) return;
  fetch("/api/pad", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ axes, buttons, source: "browser_pad" }),
    keepalive: true,
  }).catch(() => {});
}

// --- wiring -----------------------------------------------------------------

function wireSliders() {
  for (const row of $$(".joint")) {
    const name = row.dataset.joint;
    const slider = $("[data-slider]", row);
    slider.dataset.home = slider.value;

    slider.addEventListener("pointerdown", () => dragging.add(name));
    slider.addEventListener("input", () => {
      touchedAt.set(name, performance.now());
      $("[data-value]", row).textContent = Number(slider.value).toFixed(1);
      queueGoal(name, Number(slider.value));
    });
    const release = () => {
      dragging.delete(name);
      touchedAt.set(name, performance.now());
    };
    slider.addEventListener("pointerup", release);
    slider.addEventListener("pointercancel", release);
    slider.addEventListener("blur", release);
  }
}

function action(selector, path, options = {}) {
  $(selector).addEventListener("click", async () => {
    const button = $(selector);
    button.disabled = true;
    try {
      await api(path, { method: "POST" });
      if (options.after) await options.after();
      if (options.message) toast(options.message);
    } catch (error) {
      toast(error.message, true);
    } finally {
      button.disabled = false;
    }
  });
}

const MIN_SWEEP_STEPS = 120;  // must match server.py

function renderCalibration(cal) {
  const running = Boolean(cal && cal.active);
  $("#btnCalStart").hidden = running;
  $("#btnCalSave").hidden = !running;
  $("#btnCalCancel").hidden = !running;

  if (!running) {
    $("#calStat").textContent = "not running";
    $("#calTable").innerHTML = "";
    return;
  }

  const rows = Object.values(cal.joints);
  const ready = rows.filter((r) => r.swept >= MIN_SWEEP_STEPS).length;
  $("#calStat").textContent = `${ready}/${rows.length} joints swept`;
  $("#calTable").innerHTML = rows.map((r) => {
    const done = r.swept >= MIN_SWEEP_STEPS;
    return `<div class="stat"><span>${r.label}</span><b style="color:${
      done ? "var(--good)" : "var(--dim)"
    }">${r.min}–${r.max} · ${r.swept} steps${done ? " ✓" : ""}</b></div>`;
  }).join("");
}

function wireControls() {
  action("#btnArm", "/api/arm", { message: "Armed." });
  action("#btnDisarm", "/api/disarm", { message: "Disarmed." });
  action("#btnEstop", "/api/estop", { message: "Emergency stop engaged." });
  action("#btnReset", "/api/reset", { message: "E-stop cleared." });
  action("#btnRelax", "/api/relax", { message: "Torque off — arm is free." });
  action("#btnHome", "/api/home", { message: "Returning home." });
  action("#btnRecover", "/api/recover", { message: "Released joints brought back." });
  action("#btnCalStart", "/api/calibrate/start",
         { message: "Torque off — sweep every joint through its full travel." });
  action("#btnCalCancel", "/api/calibrate/cancel", { message: "Calibration discarded." });

  $("#btnCalSave").addEventListener("click", async () => {
    try {
      const result = await api("/api/calibrate/save", { method: "POST" });
      const names = Object.keys(result.calibrated || {});
      toast(`Saved travel for ${names.length} joints. Restart armctl to apply.`);
    } catch (error) { toast(error.message, true); }
  });

  $("#btnRec").addEventListener("click", async () => {
    try {
      await api(recording.recording ? "/api/record/stop" : "/api/record/start", { method: "POST" });
      await refreshRecording();
    } catch (error) { toast(error.message, true); }
  });

  $("#btnPlay").addEventListener("click", async () => {
    try {
      await api(recording.playing ? "/api/play/stop" : "/api/play", { method: "POST" });
      await refreshRecording();
    } catch (error) { toast(error.message, true); }
  });

  $("#btnSavePose").addEventListener("click", async () => {
    const name = prompt("Pose name");
    if (!name) return;
    try {
      await api("/api/poses", { method: "POST", body: JSON.stringify({ name }) });
      await refreshPoses();
      toast(`Saved ${name}.`);
    } catch (error) { toast(error.message, true); }
  });

  $("#poseList").addEventListener("click", async (event) => {
    const go = event.target.dataset.go;
    const del = event.target.dataset.del;
    try {
      if (go) {
        await api(`/api/poses/${encodeURIComponent(go)}/go`, { method: "POST" });
        toast(`Moving to ${go}.`);
      } else if (del) {
        await api(`/api/poses/${encodeURIComponent(del)}`, { method: "DELETE" });
        await refreshPoses();
      }
    } catch (error) { toast(error.message, true); }
  });

  $("#padEnable").addEventListener("change", (event) => {
    padOn = event.target.checked;
    if (!padOn) padWasActive = false;
  });
}

wireSliders();
wireControls();
poll();
padLoop();
refreshPoses();
refreshRecording();
setInterval(refreshRecording, 1000);
setInterval(pollWiredPad, 150);
pollWiredPad();
