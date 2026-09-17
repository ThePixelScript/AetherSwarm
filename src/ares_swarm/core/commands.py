"""Command definitions for AetherSwarm simulation."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple

from .enums import RejectionCode


@dataclass(frozen=True)
class Command:
    """Base command class with common attributes."""
    source_tick: int
    uav_id: str


@dataclass(frozen=True)
class AssignTaskCommand(Command):
    """Assign a task to a UAV."""
    task_id: str


@dataclass(frozen=True)
class ReleaseTaskCommand(Command):
    """Release a task from a UAV."""
    task_id: str
    reason: str = "DEFERRED"


@dataclass(frozen=True)
class SetTargetPositionCommand(Command):
    """Set a new target position for a UAV."""
    target_position: Tuple[float, float]
    speed: float = 10.0


@dataclass(frozen=True)
class StartRTHCommand(Command):
    """Initiate Return-to-Home for a UAV."""
    pass


@dataclass(frozen=True)
class ProgressTaskCommand(Command):
    """Update task progress for a UAV."""
    task_id: str
    delta_progress: float


@dataclass(frozen=True)
class StepPhysicsCommand(Command):
    """Apply physics step to a UAV."""
    new_position_xy: Tuple[float, float]
    new_velocity_xy: Tuple[float, float]
    delta_energy: float