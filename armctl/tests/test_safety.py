"""E-stop must never claim the arm is safe without proof.

Regression for a real incident: during a bus fault every torque-off write
failed, the controller logged a warning and still reported "EMERGENCY STOP.
Torque off", and all six servos stayed powered while the dashboard said
disarmed. An e-stop that can lie is worse than none.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from armctl.bus import BusError, SimBus
from armctl.controller import Controller
from armctl.joints import JointSpec


def build():
    specs = {
        "a": JointSpec(name="a", label="A", servo_id=1, lo=-90, hi=90, home=0,
                       step_lo=1000, step_hi=3000),
        "b": JointSpec(name="b", label="B", servo_id=2, lo=-90, hi=90, home=0,
                       step_lo=1000, step_hi=3000),
    }
    bus = SimBus(specs)
    bus.open()
    return Controller(specs, bus), bus


class DeafBus(SimBus):
    """Acknowledges nothing: writes fail and torque state never reads back."""

    def set_torque(self, names, enabled):
        raise BusError("bus is down")

    def refresh_torque(self):
        for health in self.health.values():
            health.torque = None      # unknown, not False


def test_release_confirms_torque_actually_went_off():
    controller, bus = build()
    bus.set_torque(["a", "b"], True)
    assert bus.release(["a", "b"]) == []
    assert all(h.torque is False for h in bus.health.values())


def test_release_reports_joints_it_could_not_confirm():
    specs = {"a": JointSpec(name="a", label="A", servo_id=1, lo=-90, hi=90,
                            home=0, step_lo=1000, step_hi=3000)}
    bus = DeafBus(specs)
    bus.open()
    assert bus.release(["a"], attempts=2) == ["a"]


def test_unknown_torque_state_counts_as_not_released():
    """Unknown is never safe: a joint we cannot read must not read as off."""
    specs = {"a": JointSpec(name="a", label="A", servo_id=1, lo=-90, hi=90,
                            home=0, step_lo=1000, step_hi=3000)}
    bus = DeafBus(specs)
    bus.open()
    bus.health["a"].torque = None
    assert bus.release(["a"], attempts=1) == ["a"]


def test_estop_does_not_claim_success_when_release_fails():
    specs = {"a": JointSpec(name="a", label="A", servo_id=1, lo=-90, hi=90,
                            home=0, step_lo=1000, step_hi=3000)}
    bus = DeafBus(specs)
    bus.open()
    controller = Controller(specs, bus)

    controller.do_estop()

    assert controller.estopped and not controller.armed
    assert controller._release_pending == {"a"}
    assert "NOT CONFIRMED OFF" in controller.status
    assert "Torque off" not in controller.status


def test_estop_reports_confirmed_when_release_succeeds():
    controller, bus = build()
    bus.set_torque(["a", "b"], True)
    controller.do_estop()
    assert controller._release_pending == set()
    assert "confirmed off" in controller.status.lower()


def test_failed_release_is_retried_until_it_succeeds():
    controller, bus = build()
    controller._release_pending = {"a"}
    controller._release_retry_at = 0.0
    controller._retry_release()
    assert controller._release_pending == set()
