from __future__ import annotations

import json
import os
from typing import Any

from dotenv import load_dotenv
from mcp.server.fastmcp import FastMCP
from mcp.server.fastmcp.server import TransportSecuritySettings

from robot_client import RobotAPIError, create_robot_client_from_env


load_dotenv()

ALLOWED_SAVED_POSITIONS = {
    "home",
    "wave_start",
    "pickup",
    "dropoff",
}

ALLOWED_DEMO_SEQUENCES = {
    "wave_hello",
    "demo_pickup",
    "reset_safe",
}

SAFETY_RULES = [
    "Only predefined positions are allowed in V1.",
    "Reject movement if robot state is unknown.",
    "Reject movement while robot is busy.",
    "Stop immediately on fault.",
    "Return to home before shutdown.",
]

allowed_hosts = [
    host.strip()
    for host in os.getenv("MCP_ALLOWED_HOSTS", "127.0.0.1:*,localhost:*,mcp.natewalinder.com").split(",")
    if host.strip()
]
allowed_origins = [
    origin.strip()
    for origin in os.getenv(
        "MCP_ALLOWED_ORIGINS",
        "http://127.0.0.1:*,http://localhost:*,https://mcp.natewalinder.com",
    ).split(",")
    if origin.strip()
]

mcp = FastMCP(
    name="Robot Arm MCP Server",
    instructions=(
        "Thin MCP adapter for a robot arm HTTP API. "
        "This server validates inputs, checks safety state, and forwards requests."
    ),
    host=os.getenv("MCP_HOST", "127.0.0.1"),
    port=int(os.getenv("MCP_PORT", "8000")),
    streamable_http_path=os.getenv("MCP_STREAMABLE_HTTP_PATH", "/"),
    transport_security=TransportSecuritySettings(
        enable_dns_rebinding_protection=True,
        allowed_hosts=allowed_hosts,
        allowed_origins=allowed_origins,
    ),
    json_response=True,
)

robot_client = create_robot_client_from_env()


def success(data: Any) -> dict[str, Any]:
    return {"ok": True, "data": data}


def failure(code: str, message: str, details: Any | None = None) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "ok": False,
        "error": {
            "code": code,
            "message": message,
        },
    }
    if details is not None:
        payload["error"]["details"] = details
    return payload


async def safe_status() -> dict[str, Any]:
    return await robot_client.get_status()


def validate_position_name(position_name: str) -> dict[str, Any] | None:
    if position_name in ALLOWED_SAVED_POSITIONS:
        return None
    return failure(
        "invalid_position",
        f"'{position_name}' is not an allowed saved position.",
        {"allowed_positions": sorted(ALLOWED_SAVED_POSITIONS)},
    )


def validate_sequence_name(sequence_name: str) -> dict[str, Any] | None:
    if sequence_name in ALLOWED_DEMO_SEQUENCES:
        return None
    return failure(
        "invalid_sequence",
        f"'{sequence_name}' is not an allowed demo sequence.",
        {"allowed_sequences": sorted(ALLOWED_DEMO_SEQUENCES)},
    )


def validate_move_preconditions(status: dict[str, Any]) -> dict[str, Any] | None:
    required_keys = ("online", "busy", "fault")
    missing = [key for key in required_keys if key not in status]
    if missing:
        return failure(
            "unknown_robot_state",
            "Robot state is incomplete. Movement rejected.",
            {"missing_fields": missing, "status": status},
        )

    if not status["online"]:
        return failure(
            "robot_offline",
            "Robot is offline. Movement rejected.",
            {"status": status},
        )
    if status["busy"]:
        return failure(
            "robot_busy",
            "Robot is currently busy. Movement rejected.",
            {"status": status},
        )
    if status["fault"]:
        return failure(
            "robot_fault",
            "Robot is in fault state. Movement rejected.",
            {"status": status},
        )

    return None


@mcp.tool()
async def get_robot_status() -> dict[str, Any]:
    """Return the current robot status."""
    try:
        return success(await robot_client.get_status())
    except RobotAPIError as exc:
        return failure(exc.code, exc.message, exc.details)


@mcp.tool()
async def list_saved_positions() -> dict[str, Any]:
    """Return available saved positions from the robot API."""
    try:
        positions = await robot_client.get_positions()
        return success(
            {
                "allowed_positions_v1": sorted(ALLOWED_SAVED_POSITIONS),
                "api_positions": positions,
            }
        )
    except RobotAPIError as exc:
        return failure(exc.code, exc.message, exc.details)


@mcp.tool()
async def move_to_saved_position(position_name: str) -> dict[str, Any]:
    """Move the robot to a predefined saved position."""
    invalid = validate_position_name(position_name)
    if invalid:
        return invalid

    try:
        status = await safe_status()
        blocked = validate_move_preconditions(status)
        if blocked:
            return blocked

        result = await robot_client.move_to_saved_position(position_name)
        return success(
            {
                "position_name": position_name,
                "status_checked": status,
                "robot_response": result,
            }
        )
    except RobotAPIError as exc:
        return failure(exc.code, exc.message, exc.details)


@mcp.tool()
async def run_demo_sequence(sequence_name: str) -> dict[str, Any]:
    """Run a predefined demo sequence."""
    invalid = validate_sequence_name(sequence_name)
    if invalid:
        return invalid

    try:
        result = await robot_client.run_demo_sequence(sequence_name)
        return success(
            {
                "sequence_name": sequence_name,
                "robot_response": result,
            }
        )
    except RobotAPIError as exc:
        return failure(exc.code, exc.message, exc.details)


@mcp.tool()
async def stop_robot() -> dict[str, Any]:
    """Send an immediate stop command."""
    try:
        return success(await robot_client.stop_robot())
    except RobotAPIError as exc:
        return failure(exc.code, exc.message, exc.details)


@mcp.resource("robot://status")
async def robot_status_resource() -> str:
    try:
        payload = await robot_client.get_status()
    except RobotAPIError as exc:
        payload = failure(exc.code, exc.message, exc.details)
    return json.dumps(payload, indent=2)


@mcp.resource("robot://saved_positions")
async def robot_saved_positions_resource() -> str:
    try:
        positions = await robot_client.get_positions()
        payload = {
            "allowed_positions_v1": sorted(ALLOWED_SAVED_POSITIONS),
            "api_positions": positions,
        }
    except RobotAPIError as exc:
        payload = failure(exc.code, exc.message, exc.details)
    return json.dumps(payload, indent=2)


@mcp.resource("robot://safety_rules")
def robot_safety_rules_resource() -> str:
    return "\n".join(f"{index}. {rule}" for index, rule in enumerate(SAFETY_RULES, start=1))


def main() -> None:
    transport = os.getenv("MCP_TRANSPORT", "streamable-http")
    mcp.run(transport=transport)


if __name__ == "__main__":
    main()
