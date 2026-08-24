"""Locate voltage drop under load.

Drives ONE servo a short distance and samples every servo's reported supply
voltage during the strain. Interpreting the result:

  * every servo sags together      -> the supply itself, or the wiring between
                                      the supply and the first servo
  * servos late in the chain sag
    more than early ones           -> resistance in the daisy-chain cables
  * nothing sags but load pins
    at 1000                        -> mechanical: the joint is jammed

Read-only apart from a short, bounded move on the joint you name. Torque is
switched off again on the way out, including on error.
"""
import sys
import time

from scservo_sdk import COMM_SUCCESS, PortHandler, port_handler, sms_sts

port_handler.LATENCY_TIMER = 8

TORQUE_ENABLE = 40
PRESENT_LOAD_L, PRESENT_VOLTAGE = 60, 62
JOINTS = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
          4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}
MOVE_STEPS = 120        # short, bounded nudge
SAMPLE_SECONDS = 2.0


def signed(value, bits=10):
    sign = 1 << bits
    return -(value & (sign - 1)) if value & sign else value


def main(port_name="COM4", servo_id=6, direction=-1):
    port = PortHandler(port_name)
    if not port.openPort() or not port.setBaudRate(1_000_000):
        raise SystemExit(f"cannot open {port_name}")
    ph = sms_sts(port)

    name = JOINTS.get(servo_id, f"servo{servo_id}")
    start, result, _ = ph.ReadPos(servo_id)
    if result != COMM_SUCCESS:
        raise SystemExit(f"cannot read servo {servo_id}")
    target = start + direction * MOVE_STEPS
    print(f"probing {name} (id {servo_id}): {start} -> {target}\n")

    idle = {}
    for sid in JOINTS:
        volts, res, _ = ph.read1ByteTxRx(sid, PRESENT_VOLTAGE)
        idle[sid] = volts / 10 if res == COMM_SUCCESS else None

    try:
        ph.write1ByteTxRx(servo_id, TORQUE_ENABLE, 1)
        ph.WritePosEx(servo_id, target, 0, 0)

        worst = {sid: 99.0 for sid in JOINTS}
        peak_load = 0
        deadline = time.time() + SAMPLE_SECONDS
        while time.time() < deadline:
            for sid in JOINTS:
                volts, res, _ = ph.read1ByteTxRx(sid, PRESENT_VOLTAGE)
                if res == COMM_SUCCESS and volts:
                    worst[sid] = min(worst[sid], volts / 10)
            load, res, _ = ph.read2ByteTxRx(servo_id, PRESENT_LOAD_L)
            if res == COMM_SUCCESS:
                peak_load = max(peak_load, abs(signed(load)))

        end, _, _ = ph.ReadPos(servo_id)
    finally:
        ph.write1ByteTxRx(servo_id, TORQUE_ENABLE, 0)

    print(f"{'servo':>6}  {'joint':15}{'idle V':>8}{'loaded V':>10}{'drop':>8}")
    for sid, joint in JOINTS.items():
        lo = worst[sid] if worst[sid] < 99 else None
        drop = None if (lo is None or idle[sid] is None) else idle[sid] - lo
        print(f"{sid:>6}  {joint:15}{str(idle[sid]):>8}{str(lo):>10}"
              f"{('%.1f' % drop) if drop is not None else '-':>8}")

    moved = abs(end - start)
    print(f"\nmoved {moved} of {MOVE_STEPS} steps, peak load {peak_load}/1000")
    drops = [idle[s] - worst[s] for s in JOINTS
             if worst[s] < 99 and idle[s] is not None]
    if drops and max(drops) > 0.3:
        spread = max(drops) - min(drops)
        print("verdict:", "uneven -> resistance in the servo daisy-chain"
              if spread > 0.3 else
              "uniform -> the supply or its lead cannot hold up under current")
    elif peak_load > 800:
        print("verdict: full torque with no voltage sag -> mechanical resistance")
    else:
        print("verdict: healthy")
    port.closePort()


if __name__ == "__main__":
    args = sys.argv[1:]
    main(args[0] if args else "COM4",
         int(args[1]) if len(args) > 1 else 6,
         int(args[2]) if len(args) > 2 else -1)
