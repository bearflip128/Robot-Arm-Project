"""Trajectory generation.

Every input source — teleop stick, slider, pose, playback — is reduced to a
velocity request, then integrated by one generator. That is the whole motion
model. There is no second rate limiter anywhere downstream.

Pure functions only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .joints import JointSpec, clamp


# Fraction of a joint's acceleration limit that trajectory planning is allowed
# to count on. The remainder is headroom so a planned ramp is always achievable.
BRAKING_RESERVE = 0.92


@dataclass(frozen=True)
class AxisState:
    """The setpoint we are streaming to the servo, and how fast it is moving."""

    pos: float
    vel: float = 0.0


def velocity_toward(pos: float, goal: float, spec: JointSpec, dt: float) -> float:
    """Velocity that closes on `goal` and can still brake to a stop in time.

    The textbook answer is sqrt(2*a*d), the fastest speed from which constant
    deceleration `a` still stops exactly at the goal. That is a continuous-time
    result, and integrating it in discrete steps overshoots: the axis crosses
    the goal, brakes, crosses back, and chatters there forever.

    Integrating the way `step_axis` does — new velocity first, then move —
    an axis at velocity v covers v*dt this step and a further
    v**2/(2a) - v*dt/2 while braking. Requiring that total to fit inside the
    remaining distance d gives

        v**2 + a*dt*v - 2*a*d <= 0
        v <= (-a*dt + sqrt((a*dt)**2 + 8*a*d)) / 2

    which is the exact discrete bound, and tends to sqrt(2*a*d) as dt -> 0.
    The |error|/dt term is a final-step guard so the last move lands on the
    goal rather than a hair past it.
    """
    error = goal - pos
    if error == 0.0:
        return 0.0
    distance = abs(error)
    accel = spec.max_accel
    braking = (-accel * dt + math.sqrt((accel * dt) ** 2 + 8.0 * accel * distance)) / 2.0
    reachable = distance / max(dt, 1e-6)
    return math.copysign(min(spec.max_vel, braking, reachable), error)


def step_axis(
    state: AxisState,
    target_vel: float,
    dt: float,
    spec: JointSpec,
    observed: float | None = None,
) -> AxisState:
    """Advance one axis by `dt` toward `target_vel`."""
    vel = clamp(target_vel, -spec.max_vel, spec.max_vel)

    # Acceleration limit. This is what turns a stick snap into a smooth ramp.
    dv = clamp(vel - state.vel, -spec.max_accel * dt, spec.max_accel * dt)
    vel = state.vel + dv

    pos = state.pos + vel * dt

    # Stop dead at a joint limit rather than accumulating velocity into a wall.
    if pos <= spec.lo:
        pos, vel = spec.lo, max(vel, 0.0)
    elif pos >= spec.hi:
        pos, vel = spec.hi, min(vel, 0.0)

    # Anti-windup: the setpoint may never run further than `lead_limit` ahead of
    # where the servo actually is. Without this the setpoint races away while the
    # arm lags under load, and the arm keeps travelling after the stick is
    # released. Velocity is zeroed on clamp so the integrator cannot wind up.
    if observed is not None:
        bounded = clamp(pos, observed - spec.lead_limit, observed + spec.lead_limit)
        if bounded != pos:
            pos, vel = bounded, 0.0

    return AxisState(pos=pos, vel=vel)


def settle_state(position: float) -> AxisState:
    """A state parked at `position` with no residual velocity."""
    return AxisState(pos=position, vel=0.0)


def stopping_point(state: AxisState, spec: JointSpec, dt: float) -> float:
    """Where the axis comes to rest if it starts braking now.

    Used as the goal while a stick is held. An acceleration-limited axis cannot
    stop on the spot, so parking the goal at the current position means the axis
    sails past it the moment input stops and then gets dragged backwards — the
    arm visibly bounces. Parking it at the braking distance instead makes
    release a smooth deceleration with no reversal.

    This is the exact inverse of `velocity_toward`: solving that function's
    bound for distance given velocity gives v**2/(2a) + v*dt/2. The continuous
    v**2/(2a) alone is a shade short and leaves a small backwards twitch.

    The distance is computed against slightly less than the real acceleration
    limit. Planning to use 100% of available braking leaves no headroom: every
    step of the release ramp then needs the full `max_accel * dt`, and the
    moment rounding demands a hair more than that the axis creeps past the goal
    and has to come back. Reserving a few percent keeps the whole ramp strictly
    inside what the axis can actually do, so release never reverses.
    """
    authority = spec.max_accel * BRAKING_RESERVE
    speed = abs(state.vel)
    distance = speed * speed / (2.0 * authority) + speed * dt / 2.0
    return clamp(state.pos + math.copysign(distance, state.vel), spec.lo, spec.hi)
