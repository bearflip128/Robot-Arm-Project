from __future__ import annotations

from .command_model import (
    AbsoluteJointTargetCommand,
    ControllerStateCommand,
    PlaybackFrameCommand,
)
from .controller_mapping import PS4ControllerMapper, PS4Snapshot


class UIInputAdapter:
    def absolute_targets(self, targets: dict[str, float], source: str = "ui") -> AbsoluteJointTargetCommand:
        return AbsoluteJointTargetCommand(targets=dict(targets), source=source)  # type: ignore[arg-type]


class PS4InputAdapter:
    def __init__(self, mapper: PS4ControllerMapper) -> None:
        self._mapper = mapper

    def from_snapshot(
        self,
        axes: dict[str, float],
        buttons: dict[str, float],
        source: str = "PS4",
    ) -> tuple[ControllerStateCommand, dict[str, float]]:
        command = ControllerStateCommand(axes=dict(axes), buttons=dict(buttons), source=source)  # type: ignore[arg-type]
        intents = self._mapper.map_snapshot(PS4Snapshot(axes=command.axes, buttons=command.buttons))
        return command, intents


class PlaybackInputAdapter:
    def frame(
        self,
        joints: dict[str, float],
        timestamp_s: float,
        source: str = "playback",
    ) -> PlaybackFrameCommand:
        return PlaybackFrameCommand(joints=dict(joints), timestamp_s=float(timestamp_s), source=source)  # type: ignore[arg-type]
