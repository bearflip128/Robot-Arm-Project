"""The control loop. The only thread that touches the serial bus.

Every tick does the same bounded work: drain the command mailbox, turn intent
into a velocity request, integrate one trajectory step, write one sync packet,
read one servo, publish one telemetry snapshot. No HTTP request can ever block
it, and it can never block an HTTP request.
"""

from __future__ import annotations

import ctypes
import logging
import sys
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Any, Callable

from .bus import BusError, FeetechBus
from .inputs import PadSnapshot, intent_from_pad
from .joints import JointSpec
from .profile import AxisState, settle_state, step_axis, stopping_point, velocity_toward

log = logging.getLogger("armctl.control")

# How far outside its commandable range a joint may rest and still be armed.
# Beyond this, arming would drive it back hard enough to stall the servo.
ARM_RANGE_TOLERANCE_STEPS = 25

# Calibration sampling guards. A joint is read at roughly control_hz / joint
# count (~16 Hz), and a hand sweep moves a few hundred steps per second, so a
# jump this large between samples is an encoder wrap or a corrupt read, never
# real motion.
CAL_MAX_JUMP_STEPS = 300
CAL_RESEED_SAMPLES = 8

# How fast a joint found outside its configured range is walked back in,
# in raw steps per control tick. At 100 Hz this is ~200 steps/s: brisk enough
# to rejoin range in about a second, gentle enough that an obstructed joint
# leans rather than slams while stall detection catches it.
RETURN_STEPS_PER_TICK = 2

# Steps the return command may lead the servo by before it stops advancing.
# Bounds how hard an obstructed joint is pushed while still being enough to
# clear the servo's position deadband and actually produce motion.
RETURN_MAX_LEAD = 25

# Overshoot into the range so deadband settling still lands inside it.
RETURN_SETTLE_MARGIN = 12

# How often to re-attempt a torque-off that could not be confirmed.
RELEASE_RETRY_S = 1.0


@dataclass
class Command:
    run: Callable[["Controller"], Any]
    done: threading.Event = field(default_factory=threading.Event)
    result: Any = None
    error: str | None = None


class Controller:
    def __init__(self, specs: dict[str, JointSpec], bus: FeetechBus, *,
                 control_hz: float = 100.0, intent_timeout: float = 0.25) -> None:
        self.specs = specs
        self.bus = bus
        self.dt_nominal = 1.0 / max(20.0, control_hz)
        self.intent_timeout = intent_timeout

        self._axes = {name: settle_state(spec.home) for name, spec in specs.items()}
        self._goal = {name: spec.home for name, spec in specs.items()}

        self._mailbox = threading.Lock()
        self._queue: list[Command] = []
        self._intent: dict[str, float] = {}
        self._intent_at = 0.0
        self._intent_source = "none"

        self.armed = False
        self.estopped = False
        self.status = "Disarmed. Arm to enable motion."

        self._release_pending: set[str] = set()
        self._release_retry_at = 0.0
        self._stranded: list[str] = []
        self._return: dict[str, float] = {}
        self._cal: dict[str, list[int] | None] | None = None
        self._cal_last: dict[str, int] = {}
        self._cal_outliers: dict[str, int] = {}
        self._snapshot: dict[str, Any] = {}
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._loop_ms = 0.0
        self._actual_hz = 0.0
        self._writes = 0
        self._last_error: str | None = None

    # -- public API, called from HTTP threads -------------------------------

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="armctl-control")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def submit(self, fn: Callable[["Controller"], Any], timeout: float = 2.0) -> Any:
        """Run `fn` on the control thread and wait for it. Used for anything
        that touches hardware (arm, torque, home)."""
        command = Command(run=fn)
        with self._mailbox:
            self._queue.append(command)
        if not command.done.wait(timeout):
            raise TimeoutError("Control loop did not respond in time.")
        if command.error:
            raise RuntimeError(command.error)
        return command.result

    def set_pad(self, pad: PadSnapshot, source: str) -> None:
        """Teleop input. Latest value wins; never blocks, never queues."""
        intent = intent_from_pad(pad, self.specs)
        with self._mailbox:
            self._intent = intent
            self._intent_at = time.monotonic()
            self._intent_source = source

    def set_goal(self, targets: dict[str, float], source: str = "ui") -> dict[str, float]:
        """Absolute joint targets from a slider, pose or playback frame."""
        accepted = {}
        with self._mailbox:
            for name, value in targets.items():
                spec = self.specs.get(name)
                if spec is None:
                    continue
                accepted[name] = self._goal[name] = spec.clamp(float(value))
            # An absolute move overrides any stale stick intent on those joints.
            for name in accepted:
                self._intent[name] = 0.0
            self._intent_source = source
        return accepted

    def snapshot(self) -> dict[str, Any]:
        return self._snapshot

    # -- hardware operations, run on the control thread ---------------------

    def do_arm(self) -> None:
        if self.estopped:
            raise RuntimeError("Emergency stop is active. Reset it first.")
        # Clear stale stall latches first: a joint the operator has since freed
        # by hand must not keep blocking the arm.
        for name in self.specs:
            self.bus.clear_relief(name)
        self._resync_to_hardware()

        thermal = self.bus.thermal()
        if thermal:
            raise RuntimeError(
                f"Servo is overheating: {_describe(thermal)}. Let it cool before arming."
            )

        # A joint resting outside its commandable window means the configured
        # travel is narrower than the arm's real travel. That is a calibration
        # problem, not a reason to refuse to move: the joint is usually free,
        # and blocking Arm strands the operator with no way to drive it back.
        #
        # So this is a warning, not a veto. Such joints are eased back into
        # range at a reduced speed by _advance(), and if one genuinely is
        # jammed the stall detection relieves it on its own.
        outside = []
        for name, spec in self.specs.items():
            step = self.bus.health[name].step
            if step is None or spec.step_in_range(step):
                continue
            gap = step - spec.cmd_step_lo if step < spec.cmd_step_lo else step - spec.cmd_step_hi
            if abs(gap) > ARM_RANGE_TOLERANCE_STEPS:
                outside.append(f"{spec.label} ({abs(gap)} steps)")

        self._release_pending.clear()
        self.bus.set_torque(list(self.specs), True)
        self.armed = True
        if outside:
            self.status = (
                f"Armed. Easing back into range: {', '.join(outside)}. "
                "Recalibrate to widen the configured travel."
            )
            log.warning("Armed with joints outside range: %s", ", ".join(outside))
        else:
            self.status = "Armed. Motion enabled."

    # -- calibration -------------------------------------------------------

    def do_calibrate_start(self) -> None:
        """Torque off and start recording the travel of every joint by hand."""
        self.armed = False
        self.bus.set_torque(list(self.specs), False)
        for name in self.specs:
            self.bus.clear_relief(name)
        seeded = {}
        for name in self.specs:
            step = self.bus.read_one(name)
            seeded[name] = [step, step] if step is not None else None
        self._cal = seeded
        self._cal_last = {n: v[0] for n, v in seeded.items() if v}
        self._cal_outliers = {}
        self.status = "Calibrating — move every joint slowly through its full travel by hand."

    def do_calibrate_stop(self) -> dict[str, list[int]]:
        result = {n: v for n, v in (self._cal or {}).items() if v}
        self._cal = None
        self.status = "Calibration captured."
        return result

    def calibration(self) -> dict[str, Any] | None:
        if self._cal is None:
            return None
        return {
            "active": True,
            "joints": {
                name: {
                    "label": self.specs[name].label,
                    "min": span[0], "max": span[1],
                    "swept": span[1] - span[0],
                    "current": self.bus.health[name].step,
                }
                for name, span in self._cal.items() if span
            },
        }

    def _sample_calibration(self) -> None:
        """Extend each joint's recorded travel, rejecting discontinuities.

        A raw min/max over ReadPos is not safe. The encoder wraps: crossing
        zero makes a reading jump 4095 -> 0, and an unfiltered min/max then
        records the full 0..4095 span as if the joint had swept everything.
        That produced calibrations covering 98% of the encoder for joints that
        physically move a third of it.

        A joint moved by hand cannot travel far between samples, so any jump
        larger than CAL_MAX_JUMP_STEPS is a wrap or a corrupt read and is
        dropped. Several consistent readings in a row re-seed the filter, so a
        genuine re-grab is not ignored forever.
        """
        if self._cal is None:
            return
        for name in self.specs:
            step = self.bus.health[name].step
            if step is None:
                continue

            last = self._cal_last.get(name)
            if last is not None and abs(step - last) > CAL_MAX_JUMP_STEPS:
                self._cal_outliers[name] = self._cal_outliers.get(name, 0) + 1
                if self._cal_outliers[name] < CAL_RESEED_SAMPLES:
                    continue  # treat as a glitch, keep the existing track
                # Persistently somewhere else: accept the new position as the
                # truth, but do not stretch the span across the gap.
                self._cal_last[name] = step
                self._cal_outliers[name] = 0
                continue

            self._cal_outliers[name] = 0
            self._cal_last[name] = step
            span = self._cal.get(name)
            if span is None:
                self._cal[name] = [step, step]
            else:
                span[0] = min(span[0], step)
                span[1] = max(span[1], step)

    def do_recover(self) -> None:
        """Re-enable torque on joints that were released after a stall.

        Separate from Arm so the operator can free a jammed joint by hand and
        bring just that joint back without cycling the whole arm.
        """
        relieved = [n for n in self.specs if self.bus.health[n].relieved]
        if not relieved:
            self.status = "Nothing to recover."
            return
        for name in relieved:
            self.bus.clear_relief(name)
            position = self.bus.read_one(name)
            if position is not None:
                logical = self.specs[name].from_step(position)
                self._axes[name] = settle_state(logical)
                self._goal[name] = self.specs[name].clamp(logical)
        if self.armed:
            self.bus.set_torque(relieved, True)
        self.status = f"Recovered: {', '.join(self.specs[n].label for n in relieved)}."

    def do_disarm(self, reason: str = "Disarmed.") -> None:
        self.armed = False
        self._resync_to_hardware()
        self.status = reason

    def do_estop(self) -> None:
        """Stop motion and cut torque, and never claim success without proof.

        The previous version logged a failed torque-off as a warning and still
        reported "EMERGENCY STOP. Torque off." That happened twice in real use
        during a bus fault: every write failed, all six servos stayed powered,
        and the dashboard told the operator the arm was safe. An e-stop that
        can lie is worse than no e-stop, so a torque-off that cannot be
        confirmed is now a loud, latched alarm that keeps retrying.
        """
        self.armed = False
        self.estopped = True
        self._resync_to_hardware()

        self._release_pending = set(self.bus.release(list(self.specs)))
        if self._release_pending:
            labels = ", ".join(sorted(self.specs[n].label for n in self._release_pending))
            self.status = (
                f"EMERGENCY STOP — TORQUE NOT CONFIRMED OFF on {labels}. "
                "The arm may still be powered and holding. Cut power at the supply."
            )
            log.error("%s", self.status)
        else:
            self.status = "EMERGENCY STOP. Torque confirmed off on all joints."

    def do_reset(self) -> None:
        self.estopped = False
        self.armed = False
        self._resync_to_hardware()
        self.status = "E-stop cleared. Arm when ready."

    def do_relax(self) -> None:
        """Torque off so the arm can be posed by hand (teach mode)."""
        self.armed = False
        self._release_pending = set(self.bus.release(list(self.specs)))
        self._resync_to_hardware()
        if self._release_pending:
            labels = ", ".join(sorted(self.specs[n].label for n in self._release_pending))
            self.status = (f"Relax FAILED to confirm torque off on {labels}. "
                           "Those joints may still be powered.")
            log.error("%s", self.status)
        else:
            self.status = "Relaxed. Torque confirmed off — the arm can be moved by hand."

    def _resync_to_hardware(self) -> None:
        """Pull setpoint and goal to where the servos actually are.

        Called on every safety transition so arming can never produce a jump:
        the first commanded position after arming is the current position.
        """
        for name in self.specs:
            self.bus.read_one(name)
        observed = self.bus.observed()
        for name, spec in self.specs.items():
            position = observed.get(name, self._axes[name].pos)
            self._axes[name] = settle_state(position)
            self._goal[name] = spec.clamp(position)
        with self._mailbox:
            self._intent = {}
            self._intent_at = 0.0

    # -- the loop ----------------------------------------------------------

    def _run(self) -> None:
        with _fine_grained_timers():
            self._loop_forever()

    def _loop_forever(self) -> None:
        try:
            self.bus.open()
            self.bus.set_torque(list(self.specs), False)
            self._resync_to_hardware()
            self.status = "Connected with torque off. Arm to enable motion."
        except Exception as exc:
            self._last_error = str(exc)
            self.status = f"Hardware unavailable: {exc}"
            log.error("Bus open failed: %s", exc)

        previous = time.perf_counter()
        deadline = previous
        while not self._stop.is_set():
            now = time.perf_counter()
            dt = min(max(now - previous, 1e-4), self.dt_nominal * 4)
            previous = now

            self._tick(dt)

            self._loop_ms = (time.perf_counter() - now) * 1000.0
            # Smoothed, because the instantaneous 1/dt of a catch-up tick is
            # meaningless noise and makes the diagnostics unreadable.
            self._actual_hz += (1.0 / dt - self._actual_hz) * 0.05

            deadline += self.dt_nominal
            sleep = deadline - time.perf_counter()
            if sleep < -self.dt_nominal:
                # Fell a whole period behind. Drop the missed ticks instead of
                # running a burst of zero-length catch-up iterations.
                deadline = time.perf_counter() + self.dt_nominal
                sleep = self.dt_nominal
            self._stop.wait(max(0.0, sleep))

        try:
            self.bus.set_torque(list(self.specs), False)
        except Exception:
            pass
        self.bus.close()

    def _tick(self, dt: float) -> None:
        self._drain_queue()

        try:
            if self.bus.connected:
                if self.armed and not self.estopped:
                    self._advance(dt)
                else:
                    self._coast()
                if self._cal is not None:
                    # Sample every joint while calibrating; round-robin is
                    # too sparse to follow a hand sweep accurately.
                    self.bus.read_all()
                else:
                    self.bus.read_next()
                self._sample_calibration()
                self._check_faults()
                self._retry_release()
            self._last_error = None
        except Exception as exc:
            self._last_error = str(exc)
            if self.armed:
                self.armed = False
                self.status = f"Motion stopped: {exc}"
                log.error("Auto-disarmed: %s", exc)

        self._publish()

    def _advance(self, dt: float) -> None:
        with self._mailbox:
            fresh = (time.monotonic() - self._intent_at) <= self.intent_timeout
            intent = dict(self._intent) if fresh else {}

        observed = self.bus.observed()
        steps: dict[str, int] = {}
        stranded: list[str] = []
        for name, spec in self.specs.items():
            # A relieved joint has had its torque cut after stalling. Keep its
            # setpoint parked on the servo's real position and send it nothing,
            # so it cannot be driven back into whatever it was straining on.
            if self.bus.health[name].relieved:
                if name in observed:
                    self._axes[name] = settle_state(observed[name])
                    self._goal[name] = spec.clamp(observed[name])
                continue

            # A joint resting outside its commandable window cannot be
            # expressed as a setpoint: to_step() clamps into the window and
            # from_step() clamps too, so setpoint and observed both read as the
            # limit, the tracking error is zero, and anti-windup never sees the
            # gap. Commanding the boundary at full authority just leans on the
            # joint until it reports overload.
            #
            # Walk it back in step space instead, a few steps per tick. That is
            # slow enough not to strain anything if the joint is obstructed —
            # and the stall detection still covers that case — while a free
            # joint simply eases into range and resumes normal control.
            actual = self.bus.health[name].step
            if actual is not None and not spec.step_in_range(actual):
                # Aim a little way INSIDE the boundary, not at it. The servo
                # has a position deadband of a step or two, so a command that
                # lands exactly on the edge settles just outside it — the joint
                # then reads out-of-range forever, one step short of done.
                target = (spec.cmd_step_lo + RETURN_SETTLE_MARGIN
                          if actual < spec.cmd_step_lo
                          else spec.cmd_step_hi - RETURN_SETTLE_MARGIN)

                # The cursor must persist across ticks. Deriving the command
                # from the live reading each tick (actual +/- a few steps) can
                # never accumulate: a couple of steps sits inside the servo's
                # position deadband, so it does not move, the reading does not
                # change, and the same command is reissued forever.
                cursor = self._return.get(name)
                if cursor is None:
                    cursor = float(actual)

                # Anti-windup, in step space: stop advancing once the command
                # is leading the servo by more than RETURN_MAX_LEAD. A free
                # joint follows and keeps progressing; an obstructed one gets a
                # bounded nudge rather than the full weight of the controller.
                if abs(cursor - actual) < RETURN_MAX_LEAD:
                    cursor = (min(target, cursor + RETURN_STEPS_PER_TICK)
                              if cursor < target
                              else max(target, cursor - RETURN_STEPS_PER_TICK))

                self._return[name] = cursor
                steps[name] = int(round(cursor))
                # Park the setpoint on reality so the trajectory generator
                # resumes cleanly once back in range — but do NOT touch the
                # goal. Overwriting it here silently cancelled the operator's
                # command: a joint that sagged out of range while moving toward
                # a target had that target replaced by wherever it had drooped
                # to, so it made one attempt and then never tried again. That
                # is what "the joint does not move at all" actually was.
                self._axes[name] = settle_state(spec.from_step(actual))
                stranded.append(name)
                continue
            self._return.pop(name, None)

            stick = intent.get(name, 0.0)
            if stick:
                target_vel = stick * spec.teleop_vel
            else:
                target_vel = velocity_toward(self._axes[name].pos, self._goal[name], spec, dt)

            self._axes[name] = step_axis(
                self._axes[name], target_vel, dt, spec, observed.get(name)
            )
            if stick:
                # Under stick control the goal tracks where this axis would
                # coast to if it started braking now, so the handover back to
                # goal-seeking on release is continuous.
                self._goal[name] = stopping_point(self._axes[name], spec, dt)
            steps[name] = spec.to_step(self._axes[name].pos)

        self.bus.write_positions(steps)
        self._writes += 1
        if self._stranded and not stranded and self.status.startswith("Armed. Easing"):
            self.status = "Armed. Motion enabled."
        self._stranded = stranded

    def _coast(self) -> None:
        """Disarmed: keep the setpoint glued to the servos so nothing jumps."""
        for name, spec in self.specs.items():
            health = self.bus.health[name]
            if health.step is None:
                continue
            position = spec.from_step(health.step)
            self._axes[name] = settle_state(position)
            self._goal[name] = spec.clamp(position)

    def _retry_release(self) -> None:
        """Keep trying to release joints whose torque-off was never confirmed.

        Throttled: on a failing bus each attempt costs a read timeout, and
        hammering it every tick would bury the control loop.
        """
        if not self._release_pending:
            return
        now = time.monotonic()
        if now < self._release_retry_at:
            return
        self._release_retry_at = now + RELEASE_RETRY_S
        still = set(self.bus.release(sorted(self._release_pending), attempts=1))
        if still != self._release_pending:
            freed = sorted(self._release_pending - still)
            if freed:
                log.warning("Torque now confirmed off on: %s", ", ".join(freed))
        self._release_pending = still
        if not still:
            self.status = "Torque confirmed off on all joints."

    def _check_faults(self) -> None:
        """Respond to servo faults in proportion to what they actually mean.

        Overheat is the only thing that stops the whole arm. A stalled joint is
        relieved on its own and everything else keeps working. Voltage and
        angle bits are advisory and never interrupt motion — they were the
        reason the arm disarmed itself every few seconds.
        """
        if not self.armed:
            return

        thermal = self.bus.thermal()
        if thermal:
            self.armed = False
            self.status = f"Disarmed — servo overheating: {_describe(thermal)}. Let it cool."
            log.error("%s", self.status)
            return

        for name in self.bus.stalled():
            if not self.bus.health[name].relieved:
                self.bus.relieve(name)
                volts = self.bus.supply_volts()
                if self.bus.undervolted():
                    # Do not blame the mechanism for a power problem. Below
                    # spec these servos cannot hold a gravity-loaded joint, and
                    # report overload while nothing is jammed at all.
                    self.status = (
                        f"{self.specs[name].label} could not hold at {volts:.1f} V, "
                        f"below the {self.bus.undervolt_warn:.1f} V floor for "
                        f"{self.bus.nominal_volts:.1f} V servos. Power delivery, not a jam."
                    )
                else:
                    self.status = (
                        f"{self.specs[name].label} stalled and was released; "
                        "the other joints still work. Free it, then press Recover."
                    )

    def _drain_queue(self) -> None:
        with self._mailbox:
            pending, self._queue = self._queue, []
        for command in pending:
            try:
                command.result = command.run(self)
            except Exception as exc:
                command.error = str(exc)
            finally:
                command.done.set()

    def _publish(self) -> None:
        observed = self.bus.observed()
        joints = []
        for name, spec in self.specs.items():
            health = self.bus.health[name]
            actual = observed.get(name)
            setpoint = self._axes[name]
            joints.append({
                "name": name,
                "label": spec.label,
                "lo": spec.lo,
                "hi": spec.hi,
                "goal": round(self._goal[name], 3),
                "setpoint": round(setpoint.pos, 3),
                "velocity": round(setpoint.vel, 2),
                "observed": None if actual is None else round(actual, 3),
                "error": None if actual is None else round(setpoint.pos - actual, 3),
                "step": health.step,
                "servo_id": spec.servo_id,
                "torque": health.torque,
                # Only debounced hard faults are "faults"; voltage/angle bits
                # are advisory and shown separately so they read as noise, not
                # as something that stopped the arm.
                "faults": health.hard_flags if health.blocking else [],
                "warnings": health.warnings,
                "relieved": health.relieved,
                "frozen": name in self._stranded,
                "volts": health.volts,
                "temp_c": health.temp_c,
                "load": health.load,
                "in_range": health.step is None or spec.step_in_range(health.step),
                "readable": health.readable,
                "read_errors": health.read_errors,
            })

        self._snapshot = {
            "time": time.time(),
            "connected": self.bus.connected,
            "armed": self.armed,
            "estopped": self.estopped,
            "motion_allowed": self.armed and not self.estopped,
            "status": self.status,
            "calibration": self.calibration(),
            "torque_release_failed": sorted(self._release_pending),
            "supply_volts": self.bus.supply_volts(),
            "undervolt": self.bus.undervolted(),
            "nominal_volts": self.bus.nominal_volts,
            "source": self._intent_source,
            "last_error": self._last_error,
            "loop": {
                "target_hz": round(1.0 / self.dt_nominal, 1),
                "actual_hz": round(self._actual_hz, 1),
                "tick_ms": round(self._loop_ms, 2),
                "writes": self._writes,
            },
            "joints": joints,
        }


def _describe(faults: dict[str, list[str]]) -> str:
    return "; ".join(f"{name} ({', '.join(flags)})" for name, flags in faults.items())


@contextmanager
def _fine_grained_timers():
    """Ask Windows for 1 ms timer resolution for the life of the control loop.

    The default scheduling tick is ~15.6 ms, so an unaided sleep can only pace a
    loop at about 64 Hz — it overshoots every period and then runs a zero-length
    catch-up tick. Without this the loop cannot hold 100 Hz at all.
    """
    winmm = None
    if sys.platform == "win32":
        try:
            winmm = ctypes.WinDLL("winmm")
            winmm.timeBeginPeriod(1)
        except Exception:
            winmm = None
            log.warning("Could not raise timer resolution; loop rate may be uneven.")
    try:
        yield
    finally:
        if winmm is not None:
            winmm.timeEndPeriod(1)
