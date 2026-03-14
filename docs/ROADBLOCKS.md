# Potential roadblocks

## Near-term software roadblocks

- The current app is a Flask server with inline HTML, which is fast for prototyping but will get harder to maintain as the UI grows.
- Manual joint sliders prove the control model, but they do not yet simulate real servo latency, backlash, or calibration drift.
- The mock preview is only a rough 2D visualization, so a sequence that looks fine in the browser may still be awkward on real hardware.

## Hardware and integration roadblocks

- The biggest unknown is whether the arm you eventually choose has enough payload and stiffness for the end-effector or tool you want.
- Real arms need careful homing, joint limits, and emergency-stop behavior; browser control alone is not enough for safe operation.
- Power delivery is easy to underestimate. Brownouts, USB serial instability, and noisy servo power are common failure points.
- Printed parts can flex or creep under repeated load, especially around shoulder joints and wrist mounts.

## Cost and sourcing roadblocks

- The original repo documents a broader dual-arm build, but the actual checked-in code is thinner than the README implies, so some integration work will fall on us.
- Servo and controller availability can change, and some recommended parts may ship slowly or arrive with slightly different firmware or gearing.
- A “cheap enough to test” arm may be too weak or imprecise, while a robust arm can raise the budget quickly.

## Product roadblocks

- A manual-control web UI may be enough for demos, but not enough for useful repeatable tasks without calibration and saved workflows.
- If you eventually want polished public sharing on GitHub, we will need to separate prototype code from hardware-specific secrets and machine-local assumptions.
- Once real hardware is connected, debugging gets slower because every software change can require physical retesting.

## What to watch first

If your goal is to decide whether this project is worth spending money on, the first three questions to answer are:

1. Does the browser control flow feel good enough to use repeatedly?
2. Is a single arm enough for the tasks you actually care about?
3. Can we keep the hardware target simple enough that safety and power do not dominate the project?
