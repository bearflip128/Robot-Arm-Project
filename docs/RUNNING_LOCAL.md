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
- Later, switch `robot.driver` in `config/config.example.yaml` to `lerobot_so100` when you have compatible hardware and dependencies.
