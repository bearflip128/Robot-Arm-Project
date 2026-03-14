# Physical integration for a one-arm build

## Short answer

No, you do not automatically need a Raspberry Pi just because the original project used one.

For a one-arm build without the head, LED face, or onboard autonomy, the simplest first physical setup is:

- your computer runs the web app
- your PS4 controller talks to your computer over Bluetooth
- your computer talks to the arm controller over USB or serial
- the arm has its own power supply

That means a Raspberry Pi is optional, not required.

## When you would want a Raspberry Pi

A Raspberry Pi becomes useful if you want the robot to be more standalone:

- the web app should run on the robot instead of your desktop
- the robot should boot and be controllable without your main computer
- you later add cameras, LEDs, local behaviors, or always-on scripts
- you want a compact onboard computer mounted into the robot body

## Practical integration path

### Phase 1: no hardware

- run the Flask app in mock mode
- test browser controls and PS4 controller mapping
- decide whether the UX feels worth building

### Phase 2: one real arm, no Pi

- buy one compatible arm and controller board
- keep the same web app on your computer
- swap the adapter from `mock` to a real driver
- connect the arm over USB/serial
- power the arm from its own supply

### Phase 3: optional Pi

- move the same app to a Raspberry Pi only if you want standalone operation
- pair controller either to the Pi or keep controller input on a client browser

## Biggest physical unknowns

- whether the arm hardware you choose exposes a control interface we can drive cleanly from Python
- whether the controller board is stable on Windows over USB
- whether the arm has enough stiffness and repeatability for what you want to do
- whether the power supply and cable routing are reliable under motion

## Current recommendation

Because you are still deciding on feasibility, I would not buy a Raspberry Pi first.

I would only buy a Pi after we prove:

1. the browser controls feel right
2. a single arm is enough
3. the chosen arm hardware is worth integrating
