# armctl

Joint-space control for a six-servo SO-101-compatible arm. Rewrite of the
original stack, built around one idea: **the serial port has exactly one
owner, and there is exactly one place where motion is shaped.**

```bash
pip install -r requirements.txt
python run.py --sim     # no hardware needed
python run.py           # real arm on COM4
```

Then open `http://127.0.0.1:7002` and sign in with the credentials in
`config/arm.yaml`.

## Why the old stack jittered

The problems were architectural, not tuning. All four are gone by construction
here, and three have regression tests in `tests/test_motion.py`.

**Serial contention.** The old 40 Hz control loop called `adapter.get_state()`
every tick, which did six sequential blocking `ReadPos` round trips, each
retrying with `time.sleep`. `/api/status` polling and *every slider POST* called
`current_payload()`, which took the same `_io_lock` and did the same six reads.
The control loop, the UI poller and the sliders all queued on one serial mutex,
so the "40 Hz" loop actually ran at an irregular 5–30 Hz. On top of that the
vendor SDK's `LATENCY_TIMER` is 50 ms, so a single dropped byte stalled the
caller for two whole control periods.

*Now:* only the control thread touches the bus. HTTP handlers read a snapshot
the loop publishes and drop values into a mailbox — both O(1), neither can
block the other. Writes go out as one broadcast sync-write packet for all six
servos with no status returns; reads are round-robin, one servo per tick, never
retried inline. `LATENCY_TIMER` is lowered to 8 ms. Measured: **100.5 Hz, 0.16
ms per tick.**

**Rate limiting applied twice, from different baselines.** `MotionFilter`
limited from the hardware readback, then `SCServoBusAdapter._apply_motion_policy`
limited again using `elapsed = now - last_command_at`. That timestamp went stale
whenever a duplicate write was suppressed (the dedup path `continue`d before
updating it), so `allowed_delta` grew without bound and the next command
lurched. Stall, lurch, stall.

*Now:* one velocity- and acceleration-limited generator in `profile.py`, and
nothing downstream re-limits anything.

**No anti-windup.** The velocity integrator advanced `desired` with no bound
relative to the real position. Under load the servo lagged, `desired` ran away,
and releasing the stick left the arm still travelling toward a target far ahead
of it.

*Now:* the setpoint may never lead the last servo reading by more than
`lead_limit`, and velocity is zeroed when that clamp engages.

**Release bounce.** Pinning the goal to the current position while a stick is
held looks right but isn't: an acceleration-limited axis cannot stop instantly,
so it sails past its own goal on release and gets dragged backwards — measured
at **14.4° of reverse travel at 88°/s**. That alone explains a lot of "the
response doesn't make sense".

*Now:* while a stick is held the goal tracks the axis's *stopping point*, so the
handover to goal-seeking on release is continuous. Measured reverse travel:
**0.005°**, well under one raw servo step.

## Architecture

```
gamepad ─┐
sliders ─┼─► mailbox ─► control thread (100 Hz) ─► sync write ─► servos 1-6
poses  ──┘   (lock)        │                                        │
                           │        ┌── round-robin read ───────────┘
                           ▼        ▼
                      telemetry snapshot ──► HTTP ──► browser
```

| Module | Responsibility |
| --- | --- |
| `joints.py` | Joint specs, logical ↔ raw-step mapping. Pure. |
| `profile.py` | Trajectory generation. Pure. The entire motion model. |
| `inputs.py` | Gamepad snapshot → normalized velocity intent. Pure. |
| `bus.py` | Serial I/O. Single-threaded by assertion. `SimBus` mirrors it. |
| `controller.py` | The 100 Hz loop. The only serial owner. |
| `server.py` | Flask. Touches no hardware. |
| `pad_service.py` | Wired USB pad, read in-process. |

The three pure modules hold all the logic worth testing, and they are tested
without a robot, a socket or a thread.

### Motion model

Every input reduces to a velocity request:

- a **stick** gives velocity directly (`intent × teleop_vel`);
- a **slider, pose or playback frame** gives a position, converted by
  `velocity_toward` — the fastest approach speed that can still brake to a stop
  exactly on the goal.

One generator then integrates: acceleration limit → velocity limit → integrate
→ joint-limit clamp → anti-windup clamp. That is the whole thing.

`velocity_toward` uses the exact *discrete* braking bound rather than the
textbook `sqrt(2ad)`. The continuous form is slightly too fast once time is
quantised: the axis crosses the goal, brakes, crosses back, and chatters there
forever. Deriving the bound for the way `step_axis` actually integrates gives
`v ≤ (−a·dt + √((a·dt)² + 8ad)) / 2`, which converges cleanly.

### State

Three values per joint, each with one meaning:

| | |
| --- | --- |
| `goal` | where the active input wants the joint |
| `setpoint` | what the generator is streaming to the servo right now |
| `observed` | what the servo last reported |

The old stack had four overlapping server-side states plus two more in the
browser, and the browser ones existed to work around slider snapback.

### Sliders

Telemetry may not write to a slider the user is touching, or touched within
450 ms. That single rule replaces `uiJoints` / `pendingTargets` entirely.
Slider changes are coalesced at 30 Hz into one POST that returns without
touching hardware.

## Safety

`Relax` (torque off, arm movable by hand) and `Disarm` (loop refuses motion,
servos still holding) are different states, and the UI says which is which.
Arming re-reads every servo and pulls the setpoint to the real position first,
so it can never jump. Any exception in the control loop auto-disarms. Servo
status bytes are decoded every tick and a fault disarms immediately.

## Configuration

`config/arm.yaml`. Calibration (`step_lo` / `step_hi` / `invert`) is carried
over unchanged from the original stack, so it runs on the same arm without
recalibrating.

The tuning knobs that matter:

| Key | Meaning |
| --- | --- |
| `max_accel` | The jitter knob. Lower = softer, higher = snappier. |
| `teleop_vel` | Speed at full stick deflection. |
| `lead_limit` | Anti-windup budget. Size as `max_vel × 0.15 s`. |
| `goal_speed` / `goal_accel` | Both `0`. See below. |

`goal_speed: 0` and `goal_accel: 0` put the servo in "go to this step as fast as
you can" mode. The old config set per-joint profiles (`command_speed: 160`,
`command_acc: 24`), which made each servo run its own trapezoidal profile toward
a goal that was replaced 25 ms later — it never finished accelerating, which is
felt directly as stutter. The host owns the trajectory; the servo must not
second-guess it.

## Deployment

`cloudflare/cloudflared.yml` defines a tunnel separate from the original
stack's, so the two can coexist on disk. Setup instructions are in its header.

```powershell
.\scripts\start.ps1 -Public
```

**Only one process can hold COM4.** The old stack and this one cannot run at the
same time; `start.ps1` warns if it sees the old server running. Use `--sim` to
work on the UI while the arm is busy elsewhere.

## Not implemented

No inverse kinematics, no Cartesian control, no collision model, no path
planning. Six independent joint targets, same as before.

## Tests

```bash
python -m pytest tests -q
```
