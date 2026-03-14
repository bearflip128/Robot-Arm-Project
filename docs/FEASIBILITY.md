# Feasibility for a one-arm web-controlled build

## Goal

Evaluate whether building a single-arm robot with a browser-based control panel is worth pursuing before ordering hardware.

## Why this is feasible

- The original repository already proves the author used a local Flask web UI to control robot hardware.
- The arm control problem is smaller than the full original vision because you only need one arm, not two synchronized arms.
- A mock-mode dashboard lets us validate the exact operator workflow before you buy parts.

## What we can validate without spending money

- whether the browser UI feels usable for manual joint control
- whether saved poses and simple repeatable positions are enough for your use case
- whether the software architecture should stay Flask-based or later move to a richer frontend
- whether a single-arm product is more practical than copying the full dual-arm desk robot

## What we cannot validate yet

- real torque and payload limits
- actual servo noise, heat, and speed
- cable management and power stability
- how well the printed parts handle repeated load

## Recommended path

1. finish the browser-based single-arm controller in mock mode
2. decide whether the interaction model is good enough to justify a physical prototype
3. choose hardware only after the software flow feels right
4. wire the first real arm using the existing adapter layer
5. add calibration, homing, and hard safety limits before regular use

## Current prototype focus

The fastest path to your end state is:

- one arm
- manual joint control
- named poses
- browser dashboard
- mock mode first, hardware second
