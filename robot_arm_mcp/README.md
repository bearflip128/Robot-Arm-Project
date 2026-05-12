# Robot Arm MCP Server

This project is a minimal MCP server that sits in front of an existing robot control HTTP API.

It does **not** implement low-level robot logic. Instead, it:

- exposes a small set of safe MCP tools and resources
- validates inputs
- checks robot state before motion
- calls the existing robot API with HTTP
- returns structured JSON responses

## What this server does

The MCP layer is a thin adapter between an MCP-compatible LLM client and your robot API.

It exposes:

- tools for status, saved positions, safe movement, demo sequences, and stop
- resources for current status, saved positions, and static safety rules

## Install dependencies

```bash
pip install -r requirements.txt
```

## Run locally

1. Copy `.env.example` to `.env`
2. Set `ROBOT_API_BASE` to your existing robot control API
3. If your robot app is protected with a bearer token, set `ROBOT_API_KEY`
4. Start the MCP server

```bash
python server.py
```

By default, the server runs with FastMCP using `streamable-http` transport.
In this repo, it is configured to listen on `127.0.0.1:8000` and serve the MCP endpoint at `/`.
For Cloudflare exposure, it also allows the public host `mcp.natewalinder.com`.

If you are on Windows in this repo, you can also start it with:

```powershell
.\start_local.ps1
```

## Required environment variables

- `ROBOT_API_BASE`
  Base URL for the existing robot API, for example `http://localhost:7001`
- `ROBOT_API_KEY`
  Optional bearer token used for outbound API requests

Optional:

- `ROBOT_API_TIMEOUT`
  Timeout in seconds for outbound robot API requests
- `MCP_TRANSPORT`
  MCP transport mode. Defaults to `streamable-http`

## Current project wiring

For this repo's current setup:

1. The Flask robot app should expose `https://robot.natewalinder.com`
2. The robot app host should have `ROBOT_API_KEY` set in its environment
3. `robot_arm_mcp/.env` should use the same token value
4. The robot app must be connected and armed before movement tools will succeed

Example `.env`:

```dotenv
ROBOT_API_BASE=https://robot.natewalinder.com
ROBOT_API_KEY=replace-with-the-same-token-used-by-the-robot-app
ROBOT_API_TIMEOUT=10
MCP_HOST=127.0.0.1
MCP_PORT=8000
MCP_STREAMABLE_HTTP_PATH=/
MCP_ALLOWED_HOSTS=127.0.0.1:*,localhost:*,mcp.natewalinder.com
MCP_ALLOWED_ORIGINS=http://127.0.0.1:*,http://localhost:*,https://mcp.natewalinder.com
MCP_TRANSPORT=streamable-http
```

## MCP server vs. robot control API

The underlying robot control API is the system that actually knows how to talk to the robot.

This MCP server does **not** do any of the following:

- low-level servo control
- inverse kinematics
- freeform XYZ motion
- vision or object detection

The MCP server only performs validation and translation.

For the current `robot.natewalinder.com` app in this project, the MCP server is intended to talk to a thin compatibility layer on the robot app that exposes:

- `GET /status`
- `GET /positions`
- `POST /move/saved`
- `POST /sequence/run`
- `POST /stop`

Those endpoints map onto the existing Flask robot UI routes and keep low-level robot logic out of the MCP server.

## Tool to API mapping

| MCP tool | Robot API call |
|---|---|
| `get_robot_status()` | `GET /status` |
| `list_saved_positions()` | `GET /positions` |
| `move_to_saved_position(position_name)` | `GET /status` then `POST /move/saved` |
| `run_demo_sequence(sequence_name)` | `POST /sequence/run` |
| `stop_robot()` | `POST /stop` |

Example payloads:

```json
{ "position_name": "home" }
```

```json
{ "sequence_name": "wave_hello" }
```

```json
{}
```

Example compatibility responses:

```json
{
  "online": true,
  "busy": false,
  "fault": false,
  "motion_allowed": true,
  "lock_reason": "System armed. Live motion commands are enabled.",
  "robot": {
    "name": "Robot Arm",
    "driver": "mock",
    "connected": true
  }
}
```

```json
{
  "positions": ["dropoff", "home", "pickup", "wave_start"],
  "allowed_sequences": ["demo_pickup", "reset_safe", "wave_hello"]
}
```

## Safety model in V1

- Only predefined positions are allowed
- Only predefined demo sequences are allowed
- Movement is rejected if robot state is incomplete
- Movement is rejected if the robot is offline
- Movement is rejected if the robot is busy
- Movement is rejected if the robot is in fault state
- Stop remains available at all times

## Future extensions

- gripper tools
- camera/object detection tools
- parameterized movement
- Raspberry Pi deployment
