from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal


CommandSource = Literal[
    "slider",
    "ui",
    "virtual",
    "PS4",
    "controller",
    "recording",
    "playback",
    "pose",
    "sequence",
    "home",
    "system",
]


@dataclass(frozen=True)
class AbsoluteJointTargetCommand:
    targets: dict[str, float]
    source: CommandSource = "ui"


@dataclass(frozen=True)
class ControllerStateCommand:
    axes: dict[str, float] = field(default_factory=dict)
    buttons: dict[str, float] = field(default_factory=dict)
    source: CommandSource = "PS4"


@dataclass(frozen=True)
class PlaybackFrameCommand:
    joints: dict[str, float]
    timestamp_s: float
    source: CommandSource = "playback"
