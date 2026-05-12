from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any

import httpx
from dotenv import load_dotenv


load_dotenv()


@dataclass
class RobotAPIError(Exception):
    code: str
    message: str
    details: Any | None = None

    def __str__(self) -> str:
        return self.message


class RobotClient:
    def __init__(
        self,
        base_url: str,
        api_key: str | None = None,
        timeout_seconds: float = 10.0,
    ) -> None:
        if not base_url:
            raise ValueError("ROBOT_API_BASE is required.")

        headers: dict[str, str] = {"Accept": "application/json"}
        if api_key:
            headers["Authorization"] = f"Bearer {api_key}"

        self._client = httpx.AsyncClient(
            base_url=base_url.rstrip("/"),
            headers=headers,
            timeout=httpx.Timeout(timeout_seconds),
        )

    async def get(self, path: str) -> Any:
        return await self._request("GET", path)

    async def post(self, path: str, payload: dict[str, Any]) -> Any:
        return await self._request("POST", path, json=payload)

    async def _request(self, method: str, path: str, **kwargs: Any) -> Any:
        try:
            response = await self._client.request(method, path, **kwargs)
        except httpx.TimeoutException as exc:
            raise RobotAPIError(
                code="robot_api_timeout",
                message="Timed out while calling the robot API.",
                details={"path": path, "method": method},
            ) from exc
        except httpx.HTTPError as exc:
            raise RobotAPIError(
                code="robot_api_unreachable",
                message="Unable to reach the robot API.",
                details={"path": path, "method": method},
            ) from exc

        if response.status_code < 200 or response.status_code >= 300:
            details: Any
            try:
                details = response.json()
            except ValueError:
                details = response.text
            raise RobotAPIError(
                code="robot_api_error",
                message=f"Robot API returned HTTP {response.status_code}.",
                details={
                    "path": path,
                    "method": method,
                    "status_code": response.status_code,
                    "response": details,
                },
            )

        if not response.content:
            return {}

        try:
            return response.json()
        except ValueError:
            return {"raw_response": response.text}

    async def get_status(self) -> dict[str, Any]:
        payload = await self.get("/status")
        if not isinstance(payload, dict):
            raise RobotAPIError(
                code="invalid_status_payload",
                message="Robot /status response was not a JSON object.",
                details={"response": payload},
            )
        return payload

    async def get_positions(self) -> Any:
        return await self.get("/positions")

    async def move_to_saved_position(self, position_name: str) -> Any:
        return await self.post("/move/saved", {"position_name": position_name})

    async def run_demo_sequence(self, sequence_name: str) -> Any:
        return await self.post("/sequence/run", {"sequence_name": sequence_name})

    async def stop_robot(self) -> Any:
        return await self.post("/stop", {})


def create_robot_client_from_env() -> RobotClient:
    return RobotClient(
        base_url=os.getenv("ROBOT_API_BASE", ""),
        api_key=os.getenv("ROBOT_API_KEY"),
        timeout_seconds=float(os.getenv("ROBOT_API_TIMEOUT", "10")),
    )
