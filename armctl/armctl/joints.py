"""Joint specifications and the logical <-> raw-step mapping.

Pure functions only. No hardware, no threads, no I/O.
"""

from __future__ import annotations

from dataclasses import dataclass, field


def clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


@dataclass(frozen=True)
class Binding:
    """How one gamepad control drives one joint."""

    kind: str = "none"  # axis | trigger_pair | button_pair | none
    axis: str | None = None
    negative: str | None = None
    positive: str | None = None
    deadzone: float = 0.10
    expo: float = 1.6
    sign: float = 1.0


@dataclass(frozen=True)
class JointSpec:
    name: str
    label: str
    servo_id: int
    lo: float
    hi: float
    home: float
    step_lo: int
    step_hi: int
    invert: bool = False
    margin_steps: int = 0

    max_vel: float = 120.0  # deg/s ceiling for any motion
    max_accel: float = 400.0  # deg/s^2 — the knob that removes stick-snap jerk
    teleop_vel: float = 90.0  # deg/s at full stick deflection
    lead_limit: float = 12.0  # max degrees the setpoint may lead the servo

    binding: Binding = field(default_factory=Binding)

    @property
    def cmd_step_lo(self) -> int:
        return self.step_lo + self.margin_steps

    @property
    def cmd_step_hi(self) -> int:
        return self.step_hi - self.margin_steps

    def __post_init__(self) -> None:
        if self.hi <= self.lo:
            raise ValueError(f"{self.name}: logical range is empty ({self.lo}..{self.hi})")
        if self.cmd_step_hi <= self.cmd_step_lo:
            raise ValueError(f"{self.name}: margin_steps leaves no usable servo range")

    def to_step(self, value: float) -> int:
        """Logical units -> raw step.

        The logical range maps across the FULL calibrated span, and the
        endpoint margin is applied afterwards purely as a clamp. Folding the
        margin into the mapping instead (as this used to) shifts every position
        inward, so a joint resting near a calibrated endpoint is suddenly
        outside the commandable window and gets driven hard against its
        mechanical stop — which reads as an overload fault.
        """
        unit = (clamp(value, self.lo, self.hi) - self.lo) / (self.hi - self.lo)
        if self.invert:
            unit = 1.0 - unit
        step = self.step_lo + unit * (self.step_hi - self.step_lo)
        return int(round(clamp(step, self.cmd_step_lo, self.cmd_step_hi)))

    def from_step(self, step: int) -> float:
        span = self.step_hi - self.step_lo
        unit = clamp((step - self.step_lo) / span, 0.0, 1.0)
        if self.invert:
            unit = 1.0 - unit
        return self.lo + unit * (self.hi - self.lo)

    def step_in_range(self, step: int) -> bool:
        return self.cmd_step_lo <= step <= self.cmd_step_hi

    def clamp(self, value: float) -> float:
        return clamp(value, self.lo, self.hi)
