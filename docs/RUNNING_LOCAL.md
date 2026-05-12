# Running the single-arm prototype locally

## After restarting your terminal

Use these commands from the repository root:

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python app\single_arm_server.py
```

Then open:

```text
http://127.0.0.1:7001
```

## Notes

- The default config uses `mock` mode, so no hardware is required.
- For SO-101-equivalent hardware, prefer the LeRobot-compatible configuration fields in [config.yaml](/C:/Users/natej/OneDrive/Desktop/qubit/config/config.yaml).
- The live app now uses a unified backend control loop:
  - UI sliders set desired joint targets
  - PS4 sends raw controller state to the server
  - playback updates the same desired target layer
  - one fixed-rate loop writes the final filtered command frame to hardware
- Joint bindings, safe ranges, controller mappings, and teleop rates are centralized in [config.yaml](/C:/Users/natej/OneDrive/Desktop/qubit/config/config.yaml).
