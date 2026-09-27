import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from armctl.inputs import PadSnapshot, intent_from_pad, shape
from armctl.joints import Binding, JointSpec
from armctl.profile import (AxisState, settle_state, step_axis, stopping_point,
                            velocity_toward)


def spec(**overrides) -> JointSpec:
    base = dict(
        name="j", label="J", servo_id=1, lo=-90.0, hi=90.0, home=0.0,
        step_lo=0, step_hi=4095, max_vel=100.0, max_accel=400.0,
        teleop_vel=80.0, lead_limit=10.0,
    )
    return JointSpec(**{**base, **overrides})


# --- mapping ---------------------------------------------------------------

def test_step_mapping_round_trips():
    s = spec(step_lo=379, step_hi=2726, lo=-150, hi=150)
    for value in (-150, -75, 0, 75, 150):
        assert s.from_step(s.to_step(value)) == pytest.approx(value, abs=0.3)


def test_invert_reverses_travel():
    normal = spec(step_lo=0, step_hi=1000)
    flipped = spec(step_lo=0, step_hi=1000, invert=True)
    assert normal.to_step(90) == flipped.to_step(-90)


def test_margin_steps_shrink_usable_range():
    s = spec(step_lo=0, step_hi=1000, margin_steps=100)
    assert s.to_step(-90) == 100
    assert s.to_step(90) == 900


def test_empty_range_is_rejected():
    with pytest.raises(ValueError):
        spec(lo=10.0, hi=10.0)
    with pytest.raises(ValueError):
        spec(step_lo=0, step_hi=100, margin_steps=60)


# --- trajectory ------------------------------------------------------------

def test_acceleration_is_limited():
    """A stick snapped to full deflection must ramp, not step."""
    s = spec(max_accel=400.0)
    state = settle_state(0.0)
    state = step_axis(state, s.max_vel, 0.01, s)
    assert state.vel == pytest.approx(4.0)  # 400 deg/s^2 * 0.01 s


def test_velocity_is_capped():
    s = spec(max_vel=50.0, max_accel=10_000.0)
    state = step_axis(settle_state(0.0), 999.0, 0.05, s)
    assert state.vel == pytest.approx(50.0)


def test_joint_limit_stops_and_kills_velocity():
    s = spec(lo=-10.0, hi=10.0, max_accel=1e6)
    state = settle_state(9.0)
    for _ in range(50):
        state = step_axis(state, s.max_vel, 0.02, s)
    assert state.pos == pytest.approx(10.0)
    assert state.vel == 0.0


def test_setpoint_cannot_wind_up_past_observed():
    """The regression that made the old arm keep moving after stick release."""
    s = spec(lead_limit=5.0, max_accel=1e6)
    state = settle_state(0.0)
    observed = 0.0  # servo is stalled and never moves
    for _ in range(200):
        state = step_axis(state, s.max_vel, 0.01, s, observed=observed)
    assert state.pos <= 5.0 + 1e-6
    assert state.vel == 0.0


def test_goal_approach_decelerates_into_position():
    s = spec(max_vel=100.0, max_accel=400.0)
    state = settle_state(0.0)
    for _ in range(2000):
        state = step_axis(state, velocity_toward(state.pos, 45.0, s, 0.01), 0.01, s)
    assert state.pos == pytest.approx(45.0, abs=0.05)
    assert abs(state.vel) < 1.0  # arrived at rest, no hunting


def test_velocity_toward_tends_to_the_continuous_limit():
    """As dt -> 0 the discrete bound must recover the textbook sqrt(2*a*d)."""
    s = spec(max_vel=1000.0, max_accel=100.0)
    assert velocity_toward(0.0, 2.0, s, 1e-9) == pytest.approx(math.sqrt(400.0))
    assert velocity_toward(2.0, 0.0, s, 1e-9) == pytest.approx(-math.sqrt(400.0))
    assert velocity_toward(5.0, 5.0, s, 0.01) == 0.0


def test_velocity_toward_is_stoppable_in_discrete_steps():
    """The commanded speed must fit this step plus the entire braking ramp.

    A continuous sqrt(2*a*d) fails this: it is slightly too fast to stop on
    the goal once time is quantised, which is what makes an axis chatter.
    """
    s = spec(max_vel=1000.0, max_accel=400.0)
    dt = 0.01
    for distance in (0.02, 0.05, 0.5, 2.0, 30.0):
        v = velocity_toward(0.0, distance, s, dt)
        this_step = v * dt
        braking = v**2 / (2 * s.max_accel) - v * dt / 2
        assert this_step + braking <= distance + 1e-9


def test_approach_never_steps_past_the_goal():
    s = spec(max_vel=1000.0, max_accel=1e6)
    assert velocity_toward(0.0, 0.01, s, 0.01) == pytest.approx(1.0)  # exactly reaches it


def test_no_overshoot_from_full_speed():
    s = spec(lo=-90, hi=90, max_vel=100.0, max_accel=200.0)
    state = AxisState(pos=0.0, vel=100.0)
    positions = []
    for _ in range(1000):
        state = step_axis(state, velocity_toward(state.pos, 40.0, s, 0.005), 0.005, s)
        positions.append(state.pos)
    assert max(positions) <= 40.5


# --- input shaping ---------------------------------------------------------

def test_deadzone_rescales_from_zero():
    binding = Binding(kind="axis", axis="left_x", deadzone=0.2, expo=1.0)
    assert shape(0.2, binding) == 0.0
    assert shape(0.21, binding) == pytest.approx(0.0125, abs=1e-3)  # starts at zero
    assert shape(1.0, binding) == pytest.approx(1.0)


def test_expo_softens_the_centre():
    binding = Binding(kind="axis", axis="left_x", deadzone=0.0, expo=2.0)
    assert shape(0.5, binding) == pytest.approx(0.25)
    assert shape(1.0, binding) == pytest.approx(1.0)


def test_sign_flips_direction():
    binding = Binding(kind="axis", axis="left_y", deadzone=0.0, expo=1.0, sign=-1.0)
    assert shape(0.5, binding) == pytest.approx(-0.5)


def test_trigger_pair_is_differential():
    specs = {"pan": spec(name="pan", binding=Binding(
        kind="trigger_pair", negative="l2", positive="r2", deadzone=0.0, expo=1.0))}
    pad = PadSnapshot(axes={}, buttons={"l2": 0.0, "r2": 1.0})
    assert intent_from_pad(pad, specs)["pan"] == pytest.approx(1.0)
    both = PadSnapshot(axes={}, buttons={"l2": 1.0, "r2": 1.0})
    assert intent_from_pad(both, specs)["pan"] == 0.0


def test_garbage_input_is_neutral():
    pad = PadSnapshot.parse({"axes": {"left_x": float("nan"), "left_y": "oops"}})
    assert pad.axes["left_x"] == 0.0
    assert pad.axes["left_y"] == 0.0
    assert not pad.active


def test_stick_release_does_not_bounce_back():
    """Releasing a stick must coast to a stop, not swing backwards.

    Parking the goal at the current position while the stick is held makes an
    acceleration-limited axis sail past it on release and get dragged back --
    14 degrees of it at 88 deg/s for these limits, which reads as the arm
    lurching the wrong way. Parking it at the stopping point instead leaves
    0.005 degrees, well under one raw servo step.
    """
    s = spec(max_vel=130.0, max_accel=560.0, lo=-150.0, hi=150.0)
    state = settle_state(0.0)
    dt = 0.01

    goal = 0.0
    for _ in range(60):  # stick held to full deflection
        state = step_axis(state, s.max_vel, dt, s)
        goal = stopping_point(state, s, dt)

    released_at = state.pos
    peak = state.pos
    worst_reverse = 0.0
    for _ in range(400):  # stick released; goal-seeking takes over
        state = step_axis(state, velocity_toward(state.pos, goal, s, dt), dt, s)
        peak = max(peak, state.pos)
        worst_reverse = min(worst_reverse, state.vel)

    assert state.pos > released_at              # coasted forward, as it should
    assert peak - state.pos < 0.05              # settled where it stopped
    assert worst_reverse > -1.0                 # no perceptible reversal


def test_stopping_point_respects_joint_limits():
    s = spec(lo=-10.0, hi=10.0, max_accel=100.0)
    assert stopping_point(AxisState(pos=9.0, vel=100.0), s, 0.01) == 10.0
    assert stopping_point(AxisState(pos=0.0, vel=0.0), s, 0.01) == 0.0
