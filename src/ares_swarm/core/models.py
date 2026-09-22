from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping, Optional, Tuple

from .constants import EPSILON
from .enums import FailureState, HandoverState, Role, RTHState, SortieState, TaskStatus, TelemetryStatus


@dataclass(frozen=True)
class UAVState:
    id: str
    active: bool = True
    failure_state: FailureState = FailureState.NORMAL
    position_xy: Tuple[float, float] = (0.0, 0.0)
    velocity_xy: Tuple[float, float] = (0.0, 0.0)
    altitude_layer: int = 1
    role: Role = Role.IDLE
    assigned_task_id: Optional[str] = None
    target_position: Optional[Tuple[float, float]] = None
    waypoint_queue: Tuple[Tuple[float, float], ...] = ()
    battery_capacity: float = 10000.0
    battery_energy: float = 10000.0
    rth_state: RTHState = RTHState.NONE
    handover_state: HandoverState = HandoverState.NONE
    sortie_state: SortieState = SortieState.READY
    recharge_start_time: Optional[float] = None
    recharge_duration_s: float = 0.0
    sortie_count: int = 0
    hop_count: int = 0
    neighbor_link_states: Mapping[str, float] = field(default_factory=lambda: MappingProxyType({}))
    role_lock_until: float = 0.0
    assignment_lock_until: float = 0.0
    cooldown_until: float = 0.0
    last_heartbeat_time: float = 0.0

    @property
    def battery_percent(self) -> float:
        if self.battery_capacity <= EPSILON:
            return 0.0
        return (self.battery_energy / self.battery_capacity) * 100.0


@dataclass(frozen=True)
class TaskState:
    id: str
    position_xy: Tuple[float, float]
    priority: int
    created_time: float = 0.0
    deadline: float = 0.0
    service_duration: float = 0.0
    service_progress: float = 0.0
    status: TaskStatus = TaskStatus.PENDING
    assigned_uav_id: Optional[str] = None
    emergency_flag: bool = False
    unreachable_reason: Optional[str] = None


@dataclass(frozen=True)
class StateSnapshot:
    simulation_tick: int
    simulation_time: float
    state_version: int
    uavs: Mapping[str, UAVState] = field(default_factory=lambda: MappingProxyType({}))
    tasks: Mapping[str, TaskState] = field(default_factory=lambda: MappingProxyType({}))
    gcs_position: Tuple[float, float] = (0.0, 0.0)

    def __post_init__(self) -> None:
        if not isinstance(self.uavs, MappingProxyType):
            object.__setattr__(self, "uavs", MappingProxyType(dict(self.uavs)))
        if not isinstance(self.tasks, MappingProxyType):
            object.__setattr__(self, "tasks", MappingProxyType(dict(self.tasks)))

    def get_uav(self, uav_id: str) -> Optional[UAVState]:
        return self.uavs.get(uav_id)

    def get_task(self, task_id: str) -> Optional[TaskState]:
        return self.tasks.get(task_id)

    def to_dict(self) -> dict:
        return {
            "simulation_tick": self.simulation_tick,
            "simulation_time": self.simulation_time,
            "state_version": self.state_version,
            "uavs": {
                uav_id: {
                    "id": uav.id,
                    "active": uav.active,
                    "failure_state": uav.failure_state.value if hasattr(uav.failure_state, "value") else str(uav.failure_state),
                    "position_xy": uav.position_xy,
                    "velocity_xy": uav.velocity_xy,
                    "altitude_layer": uav.altitude_layer,
                    "role": uav.role.value if hasattr(uav.role, "value") else str(uav.role),
                    "assigned_task_id": uav.assigned_task_id,
                    "target_position": uav.target_position,
                    "waypoint_queue": list(uav.waypoint_queue),
                    "battery_capacity": uav.battery_capacity,
                    "battery_energy": uav.battery_energy,
                    "battery_percent": uav.battery_percent,
                    "rth_state": uav.rth_state.value if hasattr(uav.rth_state, "value") else str(uav.rth_state),
                    "hop_count": uav.hop_count,
                    "role_lock_until": uav.role_lock_until,
                    "assignment_lock_until": uav.assignment_lock_until,
                    "cooldown_until": uav.cooldown_until,
                }
                for uav_id, uav in self.uavs.items()
            },
            "tasks": {
                task_id: {
                    "id": task.id,
                    "position_xy": task.position_xy,
                    "priority": task.priority,
                    "created_time": task.created_time,
                    "deadline": task.deadline,
                    "service_duration": task.service_duration,
                    "service_progress": task.service_progress,
                    "status": task.status.value if hasattr(task.status, "value") else str(task.status),
                    "assigned_uav_id": task.assigned_uav_id,
                    "emergency_flag": task.emergency_flag,
                }
                for task_id, task in self.tasks.items()
            },
            "gcs_position": self.gcs_position,
        }


@dataclass(frozen=True)
class TelemetryReport:
    """Immutable telemetry report recording detection and delivery of a POI."""
    task_id: str
    detecting_uav_id: str
    t_detect: float
    t_report_generated: float
    t_gcs_received: Optional[float] = None
    hop_count: Optional[int] = None
    route: Optional[Tuple[str, ...]] = None
    reporting_latency_s: Optional[float] = None
    status: TelemetryStatus = TelemetryStatus.PENDING

    def to_dict(self) -> dict:
        return {
            "task_id": self.task_id,
            "detecting_uav_id": self.detecting_uav_id,
            "t_detect": self.t_detect,
            "t_report_generated": self.t_report_generated,
            "t_gcs_received": self.t_gcs_received,
            "hop_count": self.hop_count,
            "route": list(self.route) if self.route else None,
            "reporting_latency_s": self.reporting_latency_s,
            "status": self.status.value if hasattr(self.status, "value") else str(self.status),
        }
