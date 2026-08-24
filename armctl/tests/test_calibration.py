"""Calibration capture must not be fooled by encoder wrap.

A raw min/max over ReadPos recorded 98% of the encoder for joints that
physically move a third of it, because crossing encoder zero makes the reading
jump 4095 -> 0. Saving that remapped every logical degree onto roughly twice
the steps it should and drove joints into their mechanical stops.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from armctl.bus import SimBus
from armctl.controller import CAL_MAX_JUMP_STEPS, Controller
from armctl.joints import JointSpec


def build(**overrides):
    spec = JointSpec(name="j", label="J", servo_id=1, lo=-90.0, hi=90.0, home=0.0,
                     step_lo=1000, step_hi=3000, **overrides)
    specs = {"j": spec}
    controller = Controller(specs, SimBus(specs))
    controller._cal = {"j": [2000, 2000]}
    controller._cal_last = {"j": 2000}
    controller._cal_outliers = {}
    return controller


def feed(controller, steps):
    for step in steps:
        controller.bus.health["j"].step = step
        controller._sample_calibration()
    return controller._cal["j"]


def test_smooth_sweep_is_captured():
    controller = build()
    span = feed(controller, range(2000, 2400, 20))
    assert span == [2000, 2380]


def test_encoder_wrap_is_rejected():
    """The exact failure: a 4095 -> 0 jump must not widen the span."""
    controller = build()
    span = feed(controller, [2000, 2050, 2100, 4095, 0, 5, 2150, 2200])
    assert span[0] >= 2000, "wrap to 0 must not become the minimum"
    assert span[1] <= 2200, "wrap to 4095 must not become the maximum"


def test_isolated_corrupt_read_is_ignored():
    controller = build()
    span = feed(controller, [2000, 2020, 17, 2040, 2060])
    assert span == [2000, 2060]


def test_filter_reseeds_after_a_sustained_move():
    """A real re-grab elsewhere must not be ignored forever."""
    controller = build()
    far = 2000 + CAL_MAX_JUMP_STEPS * 2
    feed(controller, [far] * 12)
    feed(controller, [far + 10, far + 20])
    assert controller._cal_last["j"] >= far, "filter never re-synced"


def test_jump_threshold_boundary():
    controller = build()
    span = feed(controller, [2000 + CAL_MAX_JUMP_STEPS - 1])
    assert span[1] == 2000 + CAL_MAX_JUMP_STEPS - 1
