"""Feetech SCS/STS serial bus.

Owned by exactly one thread — the control loop. Nothing else may touch it, so
there is no lock on the hot path and HTTP requests can never stall the loop.

Two properties make the loop cheap enough to run at 100 Hz:

* writes go out as ONE broadcast sync-write packet for all six servos, with no
  status returns, instead of six request/response round trips;
* reads are round-robin, one servo per tick, and never retry inline.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field

from .joints import JointSpec

log = logging.getLogger("armctl.bus")

STATUS_FLAGS = ((1, "voltage"), (2, "angle"), (4, "overheat"), (8, "overcurrent"), (32, "overload"))

# Not every status bit means "stop the robot".
#
# STALL: the joint is straining. Relieve that joint, keep the rest of the arm
# alive — killing the whole session on a single stalled gripper is what made
# teleop unusable.
# THERMAL: genuine danger to the hardware. Stop everything.
# Anything else (voltage, angle) is advisory. A voltage bit is a supply-sag
# symptom that sets while several servos accelerate together and clears on its
# own; disarming on it means the arm quits every few seconds under normal use.
STALL_FAULTS = frozenset({"overload", "overcurrent"})
THERMAL_FAULTS = frozenset({"overheat"})
HARD_FAULTS = STALL_FAULTS | THERMAL_FAULTS

# Consecutive bad samples before a hard fault is believed. Reads are
# round-robin, so a joint is sampled at roughly control_hz / joint count —
# about 16 Hz here, making this ~190 ms. Long enough to ignore a single noisy
# status byte, short enough to protect the gearbox.
FAULT_DEBOUNCE = 3

# Servo SRAM registers for vitals.
PRESENT_LOAD_L = 60
PRESENT_VOLTAGE = 62
PRESENT_TEMPERATURE = 63
VITALS_EVERY_N_READS = 6

# Default only. The real values come from config, because STS3215 ships in 5 V,
# 7.4 V and 12 V variants and the EEPROM limits are family-wide firmware
# defaults that do NOT identify which one you have. Hardcoding 7.4 here meant
# flagging a correctly-powered 5 V arm as undervolted.
DEFAULT_NOMINAL_VOLTS = 5.0
DEFAULT_UNDERVOLT_WARN = 4.5

# Present-load is reported on a +/-1000 scale. A servo pinned near the top of
# it is giving everything it has. This matters because a servo can sit at 100%
# effort and fail to move WITHOUT ever setting its overload error bit — the
# status byte stayed clean while the elbow drew full torque and still could
# not lift. Watching load catches the stalls the error byte misses.
LOAD_SATURATED = 900
LOAD_STALL_SAMPLES = 3


def _signed(value: int, bits: int) -> int:
    sign = 1 << bits
    return -(value & (sign - 1)) if value & sign else value


def decode_status(code: int) -> list[str]:
    code = int(code)
    flags = [name for bit, name in STATUS_FLAGS if code & bit]
    unknown = code & ~sum(bit for bit, _ in STATUS_FLAGS)
    if unknown:
        flags.append(f"unknown_0x{unknown:02x}")
    return flags


@dataclass
class ServoHealth:
    servo_id: int
    step: int | None = None
    torque: bool | None = None
    status_code: int = 0
    flags: list[str] = field(default_factory=list)
    read_errors: int = 0
    last_read_at: float | None = None
    last_error: str | None = None
    hard_streak: int = 0
    relieved: bool = False  # torque cut on this joint after a stall
    volts: float | None = None
    temp_c: int | None = None
    load: int | None = None
    load_streak: int = 0

    @property
    def faulted(self) -> bool:
        return bool(self.status_code)

    @property
    def hard_flags(self) -> list[str]:
        return [f for f in self.flags if f in HARD_FAULTS]

    @property
    def warnings(self) -> list[str]:
        return [f for f in self.flags if f not in HARD_FAULTS]

    @property
    def blocking(self) -> bool:
        """A hard fault that has persisted long enough to be believed."""
        return bool(self.hard_flags) and self.hard_streak >= FAULT_DEBOUNCE

    @property
    def straining(self) -> bool:
        """Pinned at full torque for long enough to count as a stall."""
        return self.load_streak >= LOAD_STALL_SAMPLES

    @property
    def stalled(self) -> bool:
        hard_stall = self.blocking and any(f in STALL_FAULTS for f in self.hard_flags)
        return hard_stall or self.straining

    @property
    def thermal(self) -> bool:
        return self.blocking and any(f in THERMAL_FAULTS for f in self.hard_flags)

    @property
    def readable(self) -> bool:
        return self.step is not None


class BusError(RuntimeError):
    pass


class FeetechBus:
    def __init__(self, specs: dict[str, JointSpec], port: str, baud: int = 1_000_000,
                 latency_ms: int = 8, goal_speed: int = 0, goal_accel: int = 0,
                 nominal_volts: float = DEFAULT_NOMINAL_VOLTS,
                 undervolt_warn: float = DEFAULT_UNDERVOLT_WARN) -> None:
        self._specs = specs
        self._port_name = port
        self._baud = baud
        self._latency_ms = latency_ms
        self._goal_speed = goal_speed
        self._goal_accel = goal_accel
        self.nominal_volts = nominal_volts
        self.undervolt_warn = undervolt_warn
        self._port = None
        self._packet = None
        self._owner: int | None = None
        self._order = list(specs)
        self._cursor = 0
        self._vitals_countdown = 0
        self._vitals_cursor = 0
        self.connected = False
        self.health = {name: ServoHealth(servo_id=spec.servo_id) for name, spec in specs.items()}

    # -- lifecycle ---------------------------------------------------------

    def open(self) -> None:
        from scservo_sdk import PortHandler, port_handler, sms_sts

        # The SDK's default 50 ms latency slack means a single dropped byte
        # stalls the caller for 50 ms. At 100 Hz that is four missed ticks, so
        # cap it well under one control period.
        port_handler.LATENCY_TIMER = self._latency_ms

        handle = PortHandler(self._port_name)
        if not handle.openPort():
            raise BusError(f"Could not open {self._port_name}.")
        if not handle.setBaudRate(self._baud):
            handle.closePort()
            raise BusError(f"Could not set baud {self._baud} on {self._port_name}.")

        self._port = handle
        self._packet = sms_sts(handle)
        self._owner = threading.get_ident()
        self.connected = True

        for name in self._order:
            self.read_one(name)
        if not any(h.readable for h in self.health.values()):
            self.close()
            raise BusError(
                f"Opened {self._port_name} but no servo answered. Check power, wiring and port."
            )
        self.refresh_torque()

    def close(self) -> None:
        if self._port is not None:
            self._port.closePort()
        self._port = None
        self._packet = None
        self.connected = False
        for health in self.health.values():
            health.torque = None

    def _check_owner(self) -> None:
        if self._owner is not None and threading.get_ident() != self._owner:
            raise BusError("Serial bus may only be used from the control thread.")

    # -- writes ------------------------------------------------------------

    def write_positions(self, steps: dict[str, int]) -> None:
        """Send every joint in a single broadcast packet. No status returns."""
        self._check_owner()
        if not self.connected or self._packet is None:
            raise BusError("Bus is not connected.")
        if not steps:
            return

        group = self._packet.groupSyncWrite
        for name, step in steps.items():
            self._packet.SyncWritePosEx(
                self._specs[name].servo_id, int(step), self._goal_speed, self._goal_accel
            )
        result = group.txPacket()
        group.clearParam()

        from scservo_sdk import COMM_SUCCESS

        if result != COMM_SUCCESS:
            raise BusError(f"Sync write failed (comm {result}).")

    def set_torque(self, names: list[str], enabled: bool) -> None:
        self._check_owner()
        if not self.connected or self._packet is None:
            raise BusError("Bus is not connected.")

        from scservo_sdk import COMM_SUCCESS, SMS_STS_TORQUE_ENABLE

        failed = []
        for name in names:
            result, error = self._packet.write1ByteTxRx(
                self._specs[name].servo_id, SMS_STS_TORQUE_ENABLE, 1 if enabled else 0
            )
            if result != COMM_SUCCESS:
                self.health[name].last_error = f"torque_comm:{result}"
                failed.append(name)
                continue
            self.health[name].torque = enabled
            self._record_status(name, error)
        if failed:
            raise BusError(f"Torque {'on' if enabled else 'off'} failed for: {', '.join(failed)}.")

    def release(self, names: list[str], attempts: int = 3) -> list[str]:
        """Cut torque and CONFIRM it went off. Returns joints NOT confirmed.

        set_torque() only reports whether the write was acknowledged. During a
        bus fault every write can fail and the caller still believed the arm was
        safe — an e-stop reported "torque off" while all six servos stayed
        powered. So this retries, then reads the torque bit back. A joint whose
        state cannot be read counts as NOT released: unknown is never safe.
        """
        pending = list(names)
        for _ in range(max(1, attempts)):
            try:
                self.set_torque(pending, False)
            except BusError:
                pass  # verify below decides, not the write result
            try:
                self.refresh_torque()
            except BusError:
                pass
            pending = [n for n in pending if self.health[n].torque is not False]
            if not pending:
                return []
            time.sleep(0.02)
        log.error("Torque NOT confirmed off for: %s", ", ".join(pending))
        return pending

    def refresh_torque(self) -> None:
        self._check_owner()
        if not self.connected or self._packet is None:
            return

        from scservo_sdk import COMM_SUCCESS, SMS_STS_TORQUE_ENABLE

        for name, spec in self._specs.items():
            value, result, error = self._packet.read1ByteTxRx(spec.servo_id, SMS_STS_TORQUE_ENABLE)
            if result == COMM_SUCCESS:
                self.health[name].torque = bool(value)
                self._record_status(name, error)

    # -- reads -------------------------------------------------------------

    def read_next(self) -> str | None:
        """Read exactly one servo, round-robin. Bounded cost per control tick."""
        if not self._order:
            return None
        name = self._order[self._cursor % len(self._order)]
        self._cursor += 1
        self.read_one(name)
        # Vitals change slowly and cost three extra round trips, so sample one
        # joint's worth every few reads. This walks its OWN cursor: driving it
        # off `name` would alias, because the vitals interval and the joint
        # count are both 6 and every sample would land on the same servo.
        self._vitals_countdown -= 1
        if self._vitals_countdown <= 0:
            self._vitals_countdown = VITALS_EVERY_N_READS
            self.read_vitals(self._order[self._vitals_cursor % len(self._order)])
            self._vitals_cursor += 1
        return name

    def read_all(self) -> None:
        """Read every servo this tick.

        Used during calibration. Round-robin gives each joint only
        control_hz / joint count samples per second, and a brisk hand sweep
        then moves hundreds of steps between samples — indistinguishable from
        an encoder wrap, so the continuity filter discards it and the captured
        travel comes out short. A read costs well under a millisecond and
        nothing is being written while calibrating, so sample all of them.
        """
        for name in self._order:
            self.read_one(name)

    def read_vitals(self, name: str) -> None:
        """Supply voltage, temperature and load for one servo."""
        self._check_owner()
        if not self.connected or self._packet is None:
            return

        from scservo_sdk import COMM_SUCCESS

        servo_id = self._specs[name].servo_id
        health = self.health[name]
        volts, result, _ = self._packet.read1ByteTxRx(servo_id, PRESENT_VOLTAGE)
        if result == COMM_SUCCESS:
            health.volts = volts / 10.0
        temp, result, _ = self._packet.read1ByteTxRx(servo_id, PRESENT_TEMPERATURE)
        if result == COMM_SUCCESS:
            health.temp_c = int(temp)
        load, result, _ = self._packet.read2ByteTxRx(servo_id, PRESENT_LOAD_L)
        if result == COMM_SUCCESS:
            health.load = _signed(load, 10)
            if abs(health.load) >= LOAD_SATURATED:
                health.load_streak += 1
            else:
                health.load_streak = 0

    def undervolted(self) -> bool:
        volts = self.supply_volts()
        return volts is not None and volts < self.undervolt_warn

    def supply_volts(self) -> float | None:
        """Lowest voltage any servo is reporting."""
        readings = [h.volts for h in self.health.values() if h.volts is not None]
        return min(readings) if readings else None

    def read_one(self, name: str) -> int | None:
        self._check_owner()
        if not self.connected or self._packet is None:
            return None

        from scservo_sdk import COMM_SUCCESS

        health = self.health[name]
        step, result, error = self._packet.ReadPos(self._specs[name].servo_id)
        if result != COMM_SUCCESS:
            # No inline retry: a retry costs another latency window inside the
            # control loop. The next round-robin pass picks the joint up again.
            health.read_errors += 1
            health.last_error = f"read_comm:{result}"
            return None
        health.step = int(step)
        health.last_read_at = time.time()
        health.last_error = None
        self._record_status(name, error)
        return health.step

    def _record_status(self, name: str, code: int) -> None:
        health = self.health[name]
        code = int(code or 0)
        health.status_code = code
        health.flags = decode_status(code)
        if health.hard_flags:
            health.hard_streak += 1
        else:
            health.hard_streak = 0

    # -- introspection -----------------------------------------------------

    def observed(self) -> dict[str, float]:
        return {
            name: self._specs[name].from_step(health.step)
            for name, health in self.health.items()
            if health.step is not None
        }

    def faults(self) -> dict[str, list[str]]:
        """Hard faults only, and only once debounced. Motion reacts to these."""
        return {name: h.hard_flags for name, h in self.health.items() if h.blocking}

    def stalled(self) -> dict[str, list[str]]:
        return {name: h.hard_flags for name, h in self.health.items() if h.stalled}

    def thermal(self) -> dict[str, list[str]]:
        return {name: h.hard_flags for name, h in self.health.items() if h.thermal}

    def warnings(self) -> dict[str, list[str]]:
        """Advisory bits (voltage sag, angle sensor). Surfaced, never acted on."""
        return {name: h.warnings for name, h in self.health.items() if h.warnings}

    def relieve(self, name: str) -> None:
        """Cut torque on one stalled joint so it stops fighting its stop."""
        health = self.health[name]
        if health.relieved:
            return
        try:
            self.set_torque([name], False)
        except BusError as exc:
            log.warning("Could not relieve %s: %s", name, exc)
            return
        health.relieved = True
        log.error("Relieved %s (%s): torque cut on this joint only.",
                  name, ", ".join(health.hard_flags) or "stall")

    def clear_relief(self, name: str) -> None:
        self.health[name].relieved = False
        self.health[name].hard_streak = 0
        self.health[name].load_streak = 0


class SimBus(FeetechBus):
    """Same interface, no hardware. Models servo lag so tuning transfers."""

    def __init__(self, specs: dict[str, JointSpec], lag: float = 0.06, **_: object) -> None:
        self._specs = specs
        self._order = list(specs)
        self._cursor = 0
        self._vitals_countdown = 0
        self._vitals_cursor = 0
        self.nominal_volts = DEFAULT_NOMINAL_VOLTS
        self.undervolt_warn = DEFAULT_UNDERVOLT_WARN
        self._owner = None
        self.connected = False
        self.health = {name: ServoHealth(servo_id=s.servo_id) for name, s in specs.items()}
        self._lag = lag
        self._goal: dict[str, float] = {}
        self._actual: dict[str, float] = {}
        self._last = time.perf_counter()

    def open(self) -> None:
        self.connected = True
        self._owner = threading.get_ident()
        for name, spec in self._specs.items():
            self._goal[name] = spec.home
            self._actual[name] = spec.home
            self.health[name].step = spec.to_step(spec.home)
            self.health[name].torque = False

    def close(self) -> None:
        self.connected = False

    def write_positions(self, steps: dict[str, int]) -> None:
        for name, step in steps.items():
            self._goal[name] = self._specs[name].from_step(int(step))
        self._advance()

    def set_torque(self, names: list[str], enabled: bool) -> None:
        for name in names:
            self.health[name].torque = enabled

    def release(self, names: list[str], attempts: int = 3) -> list[str]:
        """Cut torque and CONFIRM it went off. Returns joints NOT confirmed.

        set_torque() only reports whether the write was acknowledged. During a
        bus fault every write can fail and the caller still believed the arm was
        safe — an e-stop reported "torque off" while all six servos stayed
        powered. So this retries, then reads the torque bit back. A joint whose
        state cannot be read counts as NOT released: unknown is never safe.
        """
        pending = list(names)
        for _ in range(max(1, attempts)):
            try:
                self.set_torque(pending, False)
            except BusError:
                pass  # verify below decides, not the write result
            try:
                self.refresh_torque()
            except BusError:
                pass
            pending = [n for n in pending if self.health[n].torque is not False]
            if not pending:
                return []
            time.sleep(0.02)
        log.error("Torque NOT confirmed off for: %s", ", ".join(pending))
        return pending

    def refresh_torque(self) -> None:
        return None

    def read_one(self, name: str) -> int | None:
        self._advance()
        spec = self._specs[name]
        self.health[name].step = spec.to_step(self._actual[name])
        self.health[name].last_read_at = time.time()
        return self.health[name].step

    def _advance(self) -> None:
        now = time.perf_counter()
        dt, self._last = now - self._last, now
        alpha = 1.0 - pow(2.71828, -max(0.0, dt) / max(1e-3, self._lag))
        for name in self._specs:
            actual = self._actual.get(name, 0.0)
            self._actual[name] = actual + (self._goal.get(name, actual) - actual) * alpha
