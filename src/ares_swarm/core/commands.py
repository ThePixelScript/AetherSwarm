"""Command definitions for AetherSwarm simulation."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class Command:
    """Base command class carrying authoritative source tick and target entity ID."""
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
    """Set a new spatial target position for a UAV."""
    target_position: Tuple[float, float]
    speed: float = 10.0


@dataclass(frozen=True)
class StartRTHCommand(Command):
    """Initiate Return-to-Home for a UAV."""
    pass


@dataclass(frozen=True)
class CompleteRTHCommand(Command):
    """Complete Return-to-Home upon arrival at GCS landing threshold."""
    pass


@dataclass(frozen=True)
class ProgressTaskCommand(Command):
    """Update ongoing service progress for an assigned task."""
    task_id: str
    delta_progress: float


@dataclass(frozen=True)
class StepPhysicsCommand(Command):
    """Apply integrated kinematic position, velocity, and energy consumption."""
    new_position_xy: Tuple[float, float]
    new_velocity_xy: Tuple[float, float]
    delta_energy: float
