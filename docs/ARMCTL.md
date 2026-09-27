# armctl: official robot arm code

**armctl is the official, current codebase for this robot arm.** Treat it as the
source of truth for control code, calibration, and documentation. Everything
else in this repository (`app/`, `scripts/robot_control_server.py`, and the
configs under `config/`) is legacy and kept for reference only.

## Why armctl replaced the old stack

The first codebase (`app/single_arm_server.py`, served on port 7001) was built
with ChatGPT/Codex and never produced a reliable arm. armctl is a rewrite that
does. It has its own calibration per servo, a fixed-rate control loop, and a
test suite.

| | armctl (official) | legacy stack |
|---|---|---|
| Entry point | `python armctl/run.py` | `python app/single_arm_server.py` |
| Port | 7002 | 7001 |
| Config | `armctl/config/arm.yaml` | `config/config.yaml` |
| Servo driver | `feetech` | `scservo_bus` |
| Health check | `GET /healthz` | none |

Both stacks talk to the arm over the same serial port, so **only one can run at
a time**.

## Running armctl

From the folder that contains `armctl/`:

```powershell
python armctl\run.py
```

To restart automatically after a crash, run it under the supervisor script from
inside `armctl/`:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File scripts\run_supervised.ps1
```

Use `scripts\stop.ps1` to stop it cleanly. armctl always boots **disarmed**, so
a restart never moves the arm. To expose it through the Cloudflare tunnel, use
`scripts\start.ps1 -Public` (with `cloudflared` on your PATH).

Check that it's up with `GET /healthz`. It reports status, the armed state, and
the control loop rate (around 100 Hz).

## Login

Set credentials with the `ARMCTL_USERNAME`, `ARMCTL_PASSWORD` and
`ARMCTL_SECRET_KEY` environment variables. Don't rely on the defaults in
`arm.yaml`, especially when the dashboard is reachable from the internet.

## Calibration notes

- Every joint's servo range is set with `step_lo` / `step_hi` (plus `invert`)
  in `arm.yaml`. It's measured on the assembled arm, not computed.
- `wrist_flex` (servo 4) is calibrated to `step_lo: 1777`, `step_hi: 4061`,
  inverted, over -90..90 deg. That upper limit is only 34 steps below the
  encoder's 4095 → 0 wrap. Any overshoot past it reads as a position near 0,
  which shows up as sudden out-of-range values. The elbow had the same problem
  and was fixed with an encoder homing offset. The wrist will need the same fix.
- Stall/overload torque cuts have been seen on `elbow_flex`, `shoulder_lift` and
  `shoulder_pan`. If joints drop out, lower their `max_accel` / `max_vel`.
