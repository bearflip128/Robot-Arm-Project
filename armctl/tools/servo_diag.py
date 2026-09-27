"""Direct servo diagnostic. Read-only: never writes a goal or torque bit.

Answers three questions the app cannot:
  1. What range does each servo itself think it has (EEPROM angle limits)?
  2. Are the error bits real and stable, or noise / latched?
  3. What is the actual load, current, voltage and temperature right now?
"""
import sys, time
from collections import Counter

from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS, port_handler

port_handler.LATENCY_TIMER = 8

# Register map (SMS/STS)
MIN_ANGLE_L, MAX_ANGLE_L = 9, 11
OFS_L, MODE = 31, 33
TORQUE_ENABLE = 40
PRESENT_POSITION_L = 56
PRESENT_LOAD_L, PRESENT_VOLTAGE, PRESENT_TEMPERATURE = 60, 62, 63
PRESENT_CURRENT_L = 69

FLAGS = ((1, "voltage"), (2, "angle"), (4, "overheat"), (8, "overcurrent"), (32, "overload"))
JOINTS = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
          4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}


def decode(code):
    names = [n for bit, n in FLAGS if code & bit]
    rest = code & ~sum(b for b, _ in FLAGS)
    if rest:
        names.append(f"0x{rest:02x}")
    return names or ["-"]


def signed(value, bits=15):
    sign = 1 << bits
    return -(value & (sign - 1)) if value & sign else value


def main(port_name="COM4"):
    port = PortHandler(port_name)
    if not port.openPort():
        raise SystemExit(f"cannot open {port_name}")
    if not port.setBaudRate(1_000_000):
        raise SystemExit("cannot set baud")
    ph = sms_sts(port)

    print("=" * 78)
    print("STATIC CONFIG (from each servo's own EEPROM)")
    print("=" * 78)
    print(f"{'joint':15}{'id':>3}{'servo min':>11}{'servo max':>11}{'ofs':>7}{'mode':>6}{'torque':>8}")
    limits = {}
    for sid, name in JOINTS.items():
        lo, r1, _ = ph.read2ByteTxRx(sid, MIN_ANGLE_L)
        hi, r2, _ = ph.read2ByteTxRx(sid, MAX_ANGLE_L)
        ofs, r3, _ = ph.read2ByteTxRx(sid, OFS_L)
        mode, r4, _ = ph.read1ByteTxRx(sid, MODE)
        tq, r5, _ = ph.read1ByteTxRx(sid, TORQUE_ENABLE)
        if COMM_SUCCESS not in (r1, r2, r3, r4, r5) or r1 != COMM_SUCCESS:
            print(f"{name:15}{sid:>3}   <unreadable>")
            continue
        limits[name] = (lo, hi)
        print(f"{name:15}{sid:>3}{lo:>11}{hi:>11}{signed(ofs, 11):>7}{mode:>6}{bool(tq)!s:>8}")

    print()
    print("=" * 78)
    print("LIVE SAMPLING  (3 s, watching for flag stability)")
    print("=" * 78)

    seen = {n: Counter() for n in JOINTS.values()}
    pos_range = {n: [10**9, -10**9] for n in JOINTS.values()}
    last = {}
    samples = {n: 0 for n in JOINTS.values()}
    detail = {}

    deadline = time.time() + 3.0
    while time.time() < deadline:
        for sid, name in JOINTS.items():
            pos, res, err = ph.ReadPos(sid)
            if res != COMM_SUCCESS:
                seen[name]["<read failed>"] += 1
                continue
            samples[name] += 1
            for flag in decode(err):
                seen[name][flag] += 1
            pos_range[name][0] = min(pos_range[name][0], pos)
            pos_range[name][1] = max(pos_range[name][1], pos)
            last[name] = pos

    for sid, name in JOINTS.items():
        load, r1, _ = ph.read2ByteTxRx(sid, PRESENT_LOAD_L)
        volt, r2, _ = ph.read1ByteTxRx(sid, PRESENT_VOLTAGE)
        temp, r3, _ = ph.read1ByteTxRx(sid, PRESENT_TEMPERATURE)
        cur, r4, _ = ph.read2ByteTxRx(sid, PRESENT_CURRENT_L)
        detail[name] = (
            signed(load, 10) if r1 == COMM_SUCCESS else None,
            volt / 10 if r2 == COMM_SUCCESS else None,
            temp if r3 == COMM_SUCCESS else None,
            signed(cur, 15) if r4 == COMM_SUCCESS else None,
        )

    print(f"{'joint':15}{'pos':>6}{'jitter':>8}{'load':>7}{'volt':>6}{'temp':>6}{'mA':>7}  flags seen (count/total)")
    for name in JOINTS.values():
        load, volt, temp, cur = detail.get(name, (None,) * 4)
        lo, hi = pos_range[name]
        jitter = (hi - lo) if samples[name] else 0
        flags = " ".join(f"{k}:{v}" for k, v in seen[name].items())
        print(f"{name:15}{last.get(name, 0):>6}{jitter:>8}"
              f"{str(load):>7}{str(volt):>6}{str(temp):>6}{str(cur):>7}  {flags}/{samples[name]}")

    print()
    print("=" * 78)
    print("SERVO EEPROM LIMITS vs armctl config")
    print("=" * 78)
    cfg = {
        "shoulder_pan": (379, 2726), "shoulder_lift": (729, 3061),
        "elbow_flex": (770, 2578), "wrist_flex": (456, 1800),
        "wrist_roll": (725, 2569), "gripper": (1258, 3820),
    }
    print(f"{'joint':15}{'servo says':>16}{'config says':>16}   verdict")
    for name, (clo, chi) in cfg.items():
        if name not in limits:
            continue
        slo, shi = limits[name]
        if slo == 0 and shi in (0, 4095):
            verdict = "servo limits DISABLED (free 0-4095)"
        elif clo < slo or chi > shi:
            verdict = "config EXCEEDS servo limits <-- will stall"
        else:
            verdict = "config inside servo limits"
        print(f"{name:15}{f'{slo}-{shi}':>16}{f'{clo}-{chi}':>16}   {verdict}")

    port.closePort()


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "COM4")
