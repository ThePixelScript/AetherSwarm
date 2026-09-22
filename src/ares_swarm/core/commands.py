"""Command definitions for AetherSwarm simulation."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

from .enums import FailureState


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
class BeginLandingCommand(Command):
    """Transition UAV to final approach landing state."""
    pass


@dataclass(frozen=True)
class CompleteRTHCommand(Command):
    """Complete Return-to-Home upon arrival at GCS landing threshold."""
    pass


@dataclass(frozen=True)
class StartRechargeCommand(Command):
    """Initiate battery recharge on the ground/staging pad."""
    recharge_duration_s: float = 300.0


@dataclass(frozen=True)
class CompleteRechargeCommand(Command):
    """Complete battery recharge, restoring capacity and marking UAV READY."""
    pass


@dataclass(frozen=True)
class FailUAVCommand(Command):
    """Inject simulated hardware/communication failure into a UAV."""
    failure_state: FailureState = FailureState.FAILED
    reason: str = ""


@dataclass(frozen=True)
class RecoverUAVCommand(Command):
    """Restore a failed/degraded UAV back to operational status."""
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
