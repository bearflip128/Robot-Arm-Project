"""Move a servo's encoder zero out of its usable travel.

A joint whose travel straddles the encoder rollover reports the same step
number at two different physical angles, which makes position ambiguous, any
linear step->angle mapping invalid, and calibration record a bogus full-range
span. Feetech servos leave the zero wherever the horn happened to land at
assembly, so this is luck of the draw per joint.

This shifts the zero into the arc the joint never reaches, using the OFS
register (addr 31). It is an EEPROM write, and it is reversible: pass
--offset 0 to restore.

The sign convention is measured, not assumed — the direction of OFS differs
between firmware revisions, and guessing it wrong doubles the problem instead
of fixing it.

    python tools/set_home_offset.py COM4 3 --show
    python tools/set_home_offset.py COM4 3 --offset 1792
"""
import argparse
import time

from scservo_sdk import COMM_SUCCESS, PortHandler, port_handler, sms_sts

port_handler.LATENCY_TIMER = 8
OFS_L, TORQUE, LOCK = 31, 40, 55
ENCODER = 4096


def read_pos(ph, sid):
    pos, result, _ = ph.ReadPos(sid)
    return pos if result == COMM_SUCCESS else None


def read_ofs(ph, sid):
    raw, result, _ = ph.read2ByteTxRx(sid, OFS_L)
    return ph.scs_tohost(raw, 11) if result == COMM_SUCCESS else None


def write_ofs(ph, sid, value):
    ph.unLockEprom(sid)
    time.sleep(0.05)
    result, error = ph.write2ByteTxRx(sid, OFS_L, ph.scs_toscs(int(value), 11))
    time.sleep(0.05)
    ph.LockEprom(sid)
    time.sleep(0.05)
    return result == COMM_SUCCESS


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM4")
    ap.add_argument("servo", nargs="?", type=int, default=3)
    ap.add_argument("--offset", type=int)
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    port = PortHandler(args.port)
    if not port.openPort() or not port.setBaudRate(1_000_000):
        raise SystemExit(f"cannot open {args.port}")
    ph = sms_sts(port)
    sid = args.servo

    # Never adjust the zero of a servo that is holding: the position it is
    # driving toward would shift under it.
    ph.write1ByteTxRx(sid, TORQUE, 0)
    time.sleep(0.2)

    before_ofs = read_ofs(ph, sid)
    before_pos = read_pos(ph, sid)
    print(f"servo {sid}: position {before_pos}, offset {before_ofs}")

    if args.show or args.offset is None:
        port.closePort()
        return

    # Measure which way OFS moves the reading before trusting it.
    probe = (before_ofs or 0) + 200
    if not write_ofs(ph, sid, probe):
        port.closePort()
        raise SystemExit("offset write failed (comm error)")
    time.sleep(0.2)
    probe_pos = read_pos(ph, sid)
    delta = ((probe_pos - before_pos + ENCODER // 2) % ENCODER) - ENCODER // 2
    sign = 1 if delta > 0 else -1
    print(f"  probe: offset +200 moved the reading {delta:+d} "
          f"-> reported = raw {'+' if sign > 0 else '-'} offset")

    # Apply the requested shift in whichever direction actually achieves it.
    target = (before_ofs or 0) + sign * args.offset
    target = max(-2047, min(2047, target))
    if not write_ofs(ph, sid, target):
        write_ofs(ph, sid, before_ofs or 0)
        port.closePort()
        raise SystemExit("offset write failed; restored original")

    time.sleep(0.2)
    after_ofs = read_ofs(ph, sid)
    after_pos = read_pos(ph, sid)
    moved = ((after_pos - before_pos + ENCODER // 2) % ENCODER) - ENCODER // 2
    print(f"  offset now {after_ofs}, position {before_pos} -> {after_pos} ({moved:+d})")
    print(f"\nrestore with:  python tools/set_home_offset.py {args.port} {sid} "
          f"--offset {-(sign * args.offset)}")
    port.closePort()


if __name__ == "__main__":
    main()
