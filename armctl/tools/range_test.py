"""Measure a joint's real, usable travel by talking straight to the servo.

None of armctl's control code runs here — no trajectory generator, no range
clamping, no stall relief. That matters: if the app only ever commands a small
move, the joint looks stuck for a software reason, and no amount of staring at
the hardware will show it. This walks the servo out in small increments and
records where it stops following its own command.

It is deliberately cautious: small steps, a load ceiling, a bounded total
sweep, and torque off on every exit path including exceptions.

    python tools/range_test.py COM4 4          # both directions
    python tools/range_test.py COM4 4 up       # one direction only
"""
import sys
import time

from scservo_sdk import COMM_SUCCESS, PortHandler, port_handler, sms_sts

port_handler.LATENCY_TIMER = 8

TORQUE_ENABLE = 40
PRESENT_LOAD_L, PRESENT_VOLTAGE = 60, 62
JOINTS = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
          4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}

STEP = 15            # command increment
SETTLE_S = 0.18      # time allowed to follow each increment
STUCK_STEPS = 45     # command leading actual by more than this = not following
STUCK_STRIKES = 3    # consecutive failures before calling it a limit
LOAD_CEILING = 750   # back off rather than strain
MAX_INCREMENTS = 180 # hard bound on total travel (~2700 steps)


def signed(value, bits=10):
    sign = 1 << bits
    return -(value & (sign - 1)) if value & sign else value


def sweep(ph, servo_id, direction, label):
    start, result, _ = ph.ReadPos(servo_id)
    if result != COMM_SUCCESS:
        print(f"  {label}: cannot read servo")
        return None

    print(f"  {label}: from {start}")
    commanded = start
    last_actual = start
    strikes = 0
    worst_load = 0
    stalled_at = None

    for _ in range(MAX_INCREMENTS):
        commanded += direction * STEP
        if not 0 <= commanded <= 4095:
            print(f"    reached encoder bound at {commanded}")
            break

        ph.WritePosEx(servo_id, commanded, 0, 0)
        time.sleep(SETTLE_S)

        actual, result, _ = ph.ReadPos(servo_id)
        if result != COMM_SUCCESS:
            continue
        load, lres, _ = ph.read2ByteTxRx(servo_id, PRESENT_LOAD_L)
        load = abs(signed(load)) if lres == COMM_SUCCESS else 0
        worst_load = max(worst_load, load)

        if load >= LOAD_CEILING:
            print(f"    load ceiling {load} at step {actual}")
            stalled_at = actual
            break

        if abs(commanded - actual) > STUCK_STEPS:
            strikes += 1
            if strikes >= STUCK_STRIKES:
                print(f"    stopped following at step {actual} "
                      f"(commanded {commanded}, load {load})")
                stalled_at = actual
                break
        else:
            strikes = 0
        last_actual = actual

    end = stalled_at if stalled_at is not None else last_actual
    volts, vres, _ = ph.read1ByteTxRx(servo_id, PRESENT_VOLTAGE)
    print(f"    travelled {abs(end - start)} steps -> {end}"
          f"   peak load {worst_load}   {volts / 10 if vres == COMM_SUCCESS else '?'} V")
    return end


def main(port_name="COM4", servo_id=4, which="both"):
    port = PortHandler(port_name)
    if not port.openPort() or not port.setBaudRate(1_000_000):
        raise SystemExit(f"cannot open {port_name}")
    ph = sms_sts(port)

    name = JOINTS.get(servo_id, f"servo{servo_id}")
    print(f"\nrange test: {name} (servo {servo_id})\n")

    try:
        ph.write1ByteTxRx(servo_id, TORQUE_ENABLE, 1)
        low = high = None
        if which in ("both", "down"):
            low = sweep(ph, servo_id, -1, "downward")
        if which in ("both", "up"):
            high = sweep(ph, servo_id, +1, "upward")
    finally:
        ph.write1ByteTxRx(servo_id, TORQUE_ENABLE, 0)

    if low is not None and high is not None:
        lo, hi = min(low, high), max(low, high)
        print(f"\n  measured usable travel: {lo} .. {hi}  ({hi - lo} steps)")
        print(f"  suggested config:  step_lo: {lo + 20}   step_hi: {hi - 20}")
    port.closePort()


if __name__ == "__main__":
    args = sys.argv[1:]
    main(args[0] if args else "COM4",
         int(args[1]) if len(args) > 1 else 4,
         args[2] if len(args) > 2 else "both")
