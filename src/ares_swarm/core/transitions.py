"""Accepted mutations, distinct from advisory ActionProposal objects."""
from dataclasses import dataclass
from .validation import Validated, nonnegative, require
from .models import Vector2D, Velocity2D, LinkState
from .enums import UAVRole, TaskStatus, HandoverState, RTHState, FailureStatus

@dataclass(frozen=True, slots=True, kw_only=True)
class StateTransition(Validated):
    timestamp: float
    reason: str
    source: str = "simulation_engine"
    correlation_id: str | None = None
    expected_revision: int | None = None

    @property
    def transition_type(self) -> str:
        return type(self).__name__

    def __post_init__(self) -> None:
        super(StateTransition, self).__post_init__()
        nonnegative(self.timestamp)
        require(bool(self.reason.strip()) and bool(self.source.strip()), "reason/source required")
        require(self.expected_revision is None or self.expected_revision >= 0, "invalid revision")

@dataclass(frozen=True, slots=True, kw_only=True)
class MoveUAV(StateTransition):
    uav_id: str
    position: Vector2D
    velocity: Velocity2D = Velocity2D()
    heading: float = 0.0

@dataclass(frozen=True, slots=True, kw_only=True)
class ChangeRole(StateTransition):
    uav_id: str
    role: UAVRole

@dataclass(frozen=True, slots=True, kw_only=True)
class AssignTask(StateTransition):
    uav_id: str
    task_id: str

@dataclass(frozen=True, slots=True, kw_only=True)
class UnassignTask(StateTransition):
    uav_id: str

@dataclass(frozen=True, slots=True, kw_only=True)
class UpdateBattery(StateTransition):
    uav_id: str
    battery_pct: float
    energy_remaining: float
    estimated_rth_energy: float
    safety_reserve: float

@dataclass(frozen=True, slots=True, kw_only=True)
class UpdateConnectivity(StateTransition):
    uav_id: str
    connected_to_gcs: bool
    route_to_gcs: tuple[str, ...] = ()

@dataclass(frozen=True, slots=True, kw_only=True)
class UpdateLink(StateTransition):
    link: LinkState

@dataclass(frozen=True, slots=True, kw_only=True)
class ChangeTaskStatus(StateTransition):
    task_id: str
    status: TaskStatus

@dataclass(frozen=True, slots=True, kw_only=True)
class MarkUAVFailed(StateTransition):
    uav_id: str
    failure_status: FailureStatus = FailureStatus.FAILED

@dataclass(frozen=True, slots=True, kw_only=True)
class SetRTHState(StateTransition):
    uav_id: str
    rth_state: RTHState

@dataclass(frozen=True, slots=True, kw_only=True)
class UpdateHandoverState(StateTransition):
    uav_id: str
    handover_state: HandoverState

@dataclass(frozen=True, slots=True, kw_only=True)
class AdvanceTime(StateTransition):
    """Timestamp is the new simulation time; no dynamics are executed."""
