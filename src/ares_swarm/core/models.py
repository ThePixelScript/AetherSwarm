"""Immutable SI-unit domain models; consistency is separate from safety policy."""
from dataclasses import dataclass, field
from typing import Any, Mapping
from .enums import (
    UAVRole, FailureStatus, RecoveryState, TaskStatus, TaskType, HandoverState, RTHState,
)
from .validation import Validated, require, nonnegative, freeze

@dataclass(frozen=True, slots=True)
class Vector2D(Validated):
    x: float
    y: float

@dataclass(frozen=True, slots=True)
class Velocity2D(Validated):
    x: float = 0.0
    y: float = 0.0

@dataclass(frozen=True, slots=True)
class GeoFence(Validated):
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def __post_init__(self) -> None:
        super(GeoFence, self).__post_init__()
        require(self.min_x < self.max_x and self.min_y < self.max_y, "invalid geofence bounds")

@dataclass(frozen=True, slots=True)
class GCSState(Validated):
    id: str
    position: Vector2D

    def __post_init__(self) -> None:
        super(GCSState, self).__post_init__()
        require(bool(self.id.strip()), "empty GCS id")

@dataclass(frozen=True, slots=True)
class UAVState(Validated):
    id: str
    position: Vector2D
    active: bool = True
    failure_status: FailureStatus = FailureStatus.HEALTHY
    velocity: Velocity2D = field(default_factory=Velocity2D)
    heading: float = 0.0
    altitude_layer: int = 0
    role: UAVRole = UAVRole.IDLE
    assigned_task_id: str | None = None
    target_position: Vector2D | None = None
    battery_pct: float = 100.0
    energy_remaining: float = 100.0
    estimated_rth_energy: float = 0.0
    safety_reserve: float = 10.0
    connected_to_gcs: bool = False
    parent_relay_id: str | None = None
    route_to_gcs: tuple[str, ...] = ()
    hop_count: int = 0
    role_lock_until: float = 0.0
    assignment_lock_until: float = 0.0
    cooldown_until: float = 0.0
    handover_state: HandoverState = HandoverState.NONE
    rth_state: RTHState = RTHState.NONE
    last_heartbeat: float = 0.0
    last_role_change: float = 0.0
    last_assignment_change: float = 0.0

    def __post_init__(self) -> None:
        super(UAVState, self).__post_init__()
        require(bool(self.id.strip()), "empty UAV id")
        require(0 <= self.battery_pct <= 100, "battery percentage out of bounds")
        require(0 <= self.heading < 360, "heading must be degrees in [0,360)")
        nonnegative(self.energy_remaining, self.estimated_rth_energy, self.safety_reserve,
                    self.altitude_layer, self.hop_count, self.role_lock_until,
                    self.assignment_lock_until, self.cooldown_until, self.last_heartbeat,
                    self.last_role_change, self.last_assignment_change)
        require(not (self.active and self.failure_status in (FailureStatus.FAILED, FailureStatus.LOST)),
                "failed/lost UAV must be inactive")
        if self.rth_state is RTHState.RETURNING:
            require(self.role is UAVRole.RETURN_TO_HOME and self.assigned_task_id is None,
                    "returning UAV requires RTH role and no task")

@dataclass(frozen=True, slots=True)
class TaskState(Validated):
    id: str
    position: Vector2D
    priority: float = 1.0
    task_type: TaskType = TaskType.SURVEY
    status: TaskStatus = TaskStatus.PENDING
    assigned_uav_id: str | None = None
    created_time: float = 0.0
    started_time: float | None = None
    completed_time: float | None = None
    deadline: float | None = None
    service_duration: float = 1.0
    is_emergency: bool = False

    def __post_init__(self) -> None:
        super(TaskState, self).__post_init__()
        require(bool(self.id.strip()), "empty task id")
        require(self.priority > 0, "priority must be positive")
        nonnegative(self.created_time, self.service_duration)
        for t in (self.started_time, self.completed_time, self.deadline):
            require(t is None or t >= self.created_time, "task timestamp precedes creation")
        require(self.completed_time is None or self.started_time is not None,
                "completion requires start time")
        if self.completed_time is not None:
            require(self.completed_time >= self.started_time, "completion precedes start")
        require((self.status is TaskStatus.COMPLETED) == (self.completed_time is not None),
                "completion status/time mismatch")
        if self.status is TaskStatus.IN_PROGRESS:
            require(self.started_time is not None, "in-progress task requires start")
        if self.status in (TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS):
            require(self.assigned_uav_id is not None, "active task requires assignee")
        else:
            require(self.assigned_uav_id is None, "inactive task cannot have assignee")

@dataclass(frozen=True, slots=True)
class LinkState(Validated):
    source_id: str
    target_id: str
    distance: float = 0.0
    rssi: float | None = None
    snr: float | None = None
    packet_loss_probability: float = 0.0
    estimated_pdr: float = 1.0
    latency_ms: float = 0.0
    etx: float = 1.0
    link_quality: float = 1.0
    active: bool = True
    last_updated: float = 0.0

    def __post_init__(self) -> None:
        super(LinkState, self).__post_init__()
        require(bool(self.source_id) and bool(self.target_id) and self.source_id != self.target_id,
                "invalid link endpoints")
        nonnegative(self.distance, self.latency_ms, self.last_updated)
        require(self.etx >= 1, "ETX must be >= 1")
        require(all(0 <= v <= 1 for v in (
            self.packet_loss_probability, self.estimated_pdr, self.link_quality)), "invalid probability")

@dataclass(frozen=True, slots=True)
class NetworkState(Validated):
    links: tuple[LinkState, ...] = ()
    recovery_state: RecoveryState = RecoveryState.NORMAL

@dataclass(frozen=True, slots=True)
class MissionState(Validated):
    id: str = "mission"
    started: bool = False
    ended: bool = False

    def __post_init__(self) -> None:
        super(MissionState, self).__post_init__()
        require(bool(self.id.strip()), "empty mission id")
        require(not self.ended or self.started, "ended mission must have started")

@dataclass(frozen=True, slots=True)
class SwarmState(Validated):
    gcs: GCSState
    uavs: tuple[UAVState, ...] = ()
    tasks: tuple[TaskState, ...] = ()
    simulation_time: float = 0.0
    network: NetworkState = field(default_factory=NetworkState)
    mission: MissionState = field(default_factory=MissionState)
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        super(SwarmState, self).__post_init__()
        object.__setattr__(self, "metadata", freeze(self.metadata))
        nonnegative(self.simulation_time)
        us, ts = {u.id: u for u in self.uavs}, {t.id: t for t in self.tasks}
        require(len(us) == len(self.uavs), "duplicate UAV ids")
        require(len(ts) == len(self.tasks), "duplicate task ids")
        require(self.gcs.id not in us, "GCS/UAV id collision")
        nodes = set(us) | {self.gcs.id}
        edges = set()
        for link in self.network.links:
            require(link.source_id in nodes and link.target_id in nodes, "unknown link endpoint")
            key = (link.source_id, link.target_id)
            require(key not in edges, "duplicate directed link")
            edges.add(key)
            require(link.last_updated <= self.simulation_time, "future link timestamp")
        for u in self.uavs:
            if u.assigned_task_id is not None:
                require(u.assigned_task_id in ts, "unknown task reference")
                require(ts[u.assigned_task_id].assigned_uav_id == u.id, "assignment not reciprocal")
                require(u.active, "inactive UAV assigned")
            if u.connected_to_gcs:
                route = u.route_to_gcs
                require(len(route) >= 2 and route[0] == u.id and route[-1] == self.gcs.id,
                        "invalid route endpoints")
                require(len(set(route)) == len(route) and set(route) <= nodes, "invalid route nodes")
                require(u.hop_count == len(route)-1 and u.parent_relay_id == route[1],
                        "route metadata mismatch")
                require(u.active and all(n == self.gcs.id or us[n].active for n in route),
                        "route uses inactive node")
            else:
                require(not u.route_to_gcs and u.hop_count == 0 and u.parent_relay_id is None,
                        "disconnected UAV has route")
            require(max(u.last_heartbeat, u.last_role_change, u.last_assignment_change)
                    <= self.simulation_time, "future UAV timestamp")
        for t in self.tasks:
            if t.assigned_uav_id is not None:
                require(t.assigned_uav_id in us, "unknown UAV reference")
                require(us[t.assigned_uav_id].assigned_task_id == t.id, "assignment not reciprocal")
            require(all(v is None or v <= self.simulation_time for v in
                        (t.created_time, t.started_time, t.completed_time)), "future task timestamp")
