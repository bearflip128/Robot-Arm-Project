# Qubit project reality and direction

## What is in this repository today

- A large `README.md` that describes a broader robot platform.
- One substantial Flask app in `scripts/robot_control_server.py`.
- Media assets for emotes, sounds, and promo GIFs.
- STL and 3MF files for printed parts.

## What is not in the repository today

- No `requirements.txt` or `pyproject.toml`.
- No checked-in config folder even though the README references one.
- No separate dashboard, diagnostics, calibration, or playback scripts matching the README table.

## What the existing Python app really does

`scripts/robot_control_server.py` is an all-in-one animation editor for two SO-100 style follower arms. It contains:

- dual-arm state management
- keyframe recording and playback
- sequence chaining
- a camera stream endpoint
- a large inline HTML dashboard

That makes it a useful reference, but not yet a clean product foundation for a single-arm web app.

## New single-arm foundation added here

- `requirements.txt`
- `config/config.example.yaml`
- `app/robot_arm.py`
- `app/single_arm_server.py`

This new scaffold keeps the original files untouched while giving us a simpler path:

1. run the dashboard in mock mode
2. wire in one real arm
3. add safety limits and homing
4. expand the UI around your printed robot body and behaviors
