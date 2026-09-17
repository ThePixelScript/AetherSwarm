"""Atomic consistency validation and capability-protected state ownership."""
from dataclasses import replace
from .models import SwarmState, UAVState
from .snapshot import StateSnapshot
from .transitions import (
    StateTransition, AdvanceTime, MoveUAV, ChangeRole, AssignTask, UnassignTask,
    UpdateBattery, UpdateConnectivity, UpdateLink, ChangeTaskStatus,
    MarkUAVFailed, SetRTHState, UpdateHandoverState,
)
from .enums import FailureStatus, TaskStatus, UAVRole, RTHState, HandoverState
from .exceptions import ModelError, TransitionError, AuthorizationError

class StateStore:
    """Own state. The composition root retains its writer key for the future engine.

    This capability boundary prevents accidental writes, not hostile Python reflection.
    Never pass the key or full store to analysis/planning modules.
    """
    def __init__(self, initial_state: SwarmState, *, writer_key: object) -> None:
        if writer_key is None or not isinstance(initial_state, SwarmState):
            raise ModelError("validated state and non-None writer key required")
        self.__state = initial_state
        self.__initial = initial_state
        self.__writer_key = writer_key
        self.__revision = 0

    def get_snapshot(self) -> StateSnapshot:
        """Return deeply immutable state at the current monotonic revision."""
        return StateSnapshot(self.__state, self.__revision)

    def validate_transition(self, transition: StateTransition) -> None:
        """Validate without committing. Raise TransitionError on any inconsistency."""
        self._candidate(transition)

    def commit_transition(self, transition: StateTransition, *, writer_key: object) -> StateSnapshot:
        """Atomically commit one valid transition with the engine's capability."""
        self._authorize(writer_key)
        candidate = self._candidate(transition)
        self.__state = candidate
        self.__revision += 1
        return self.get_snapshot()

    def reset(self, state: SwarmState | None = None, *, writer_key: object) -> StateSnapshot:
        """Explicit run reset; time may rewind, revision never does."""
        self._authorize(writer_key)
        if state is not None and not isinstance(state, SwarmState):
            raise ModelError("reset requires SwarmState")
        self.__state = self.__initial if state is None else state
        self.__revision += 1
        return self.get_snapshot()

    def _authorize(self, key: object) -> None:
        if key is not self.__writer_key:
            raise AuthorizationError("only the engine writer capability can commit/reset")

    def _candidate(self, t: StateTransition) -> SwarmState:
        if not isinstance(t, StateTransition):
            raise TransitionError("expected accepted StateTransition, not proposal")
        if t.expected_revision is not None and t.expected_revision != self.__revision:
            raise TransitionError("stale revision")
        if t.timestamp < self.__state.simulation_time:
            raise TransitionError("time cannot move backward")
        if not isinstance(t, AdvanceTime) and t.timestamp != self.__state.simulation_time:
            raise TransitionError("advance time explicitly before a state mutation")
        try:
            return self._reduce(t)
        except (ModelError, KeyError, TypeError, ValueError) as exc:
            raise TransitionError(str(exc)) from exc

    def _reduce(self, t: StateTransition) -> SwarmState:
        state = self.__state
        uavs = {u.id: u for u in state.uavs}
        tasks = {task.id: task for task in state.tasks}
        network = state.network

        def available(uid: str) -> UAVState:
            u = uavs[uid]
            if not u.active:
                raise ModelError("inactive UAV cannot receive normal commands")
            return u

        def detach(uid: str) -> None:
            u = uavs[uid]
            if u.assigned_task_id is None:
                raise ModelError("UAV has no task")
            task = tasks[u.assigned_task_id]
            tasks[task.id] = replace(task, assigned_uav_id=None, status=TaskStatus.PENDING,
                                     started_time=None)
            uavs[uid] = replace(u, assigned_task_id=None, target_position=None,
                                last_assignment_change=t.timestamp)

        if type(t) is AdvanceTime:
            return replace(state, simulation_time=t.timestamp)
        if type(t) is MoveUAV:
            u = available(t.uav_id)
            uavs[u.id] = replace(u, position=t.position, velocity=t.velocity, heading=t.heading)
        elif type(t) is ChangeRole:
            u = available(t.uav_id)
            if u.rth_state is RTHState.RETURNING and t.role is not UAVRole.RETURN_TO_HOME:
                raise ModelError("returning UAV cannot change to a mission role")
            uavs[u.id] = replace(u, role=t.role, last_role_change=t.timestamp)
        elif type(t) is AssignTask:
            u, task = available(t.uav_id), tasks[t.task_id]
            if u.assigned_task_id is not None or task.status is not TaskStatus.PENDING:
                raise ModelError("assignment requires free UAV and pending task")
            if u.role is UAVRole.RETURN_TO_HOME or u.rth_state is not RTHState.NONE:
                raise ModelError("RTH UAV cannot accept task")
            tasks[task.id] = replace(task, status=TaskStatus.ASSIGNED, assigned_uav_id=u.id)
            uavs[u.id] = replace(u, assigned_task_id=task.id, target_position=task.position,
                                last_assignment_change=t.timestamp)
        elif type(t) is UnassignTask:
            detach(t.uav_id)
        elif type(t) is UpdateBattery:
            u = uavs[t.uav_id]
            uavs[u.id] = replace(u, battery_pct=t.battery_pct, energy_remaining=t.energy_remaining,
                                estimated_rth_energy=t.estimated_rth_energy,
                                safety_reserve=t.safety_reserve)
        elif type(t) is UpdateConnectivity:
            u = uavs[t.uav_id]
            route = t.route_to_gcs
            uavs[u.id] = replace(u, connected_to_gcs=t.connected_to_gcs, route_to_gcs=route,
                                hop_count=max(0, len(route)-1),
                                parent_relay_id=route[1] if len(route) > 1 else None)
        elif type(t) is UpdateLink:
            links = {(x.source_id, x.target_id): x for x in network.links}
            links[(t.link.source_id, t.link.target_id)] = t.link
            network = replace(network, links=tuple(links[k] for k in sorted(links)))
        elif type(t) is ChangeTaskStatus:
            task = tasks[t.task_id]
            allowed = {
                TaskStatus.PENDING: (TaskStatus.FAILED, TaskStatus.CANCELLED),
                TaskStatus.ASSIGNED: (TaskStatus.IN_PROGRESS, TaskStatus.FAILED, TaskStatus.CANCELLED),
                TaskStatus.IN_PROGRESS: (TaskStatus.COMPLETED, TaskStatus.FAILED, TaskStatus.CANCELLED),
            }
            if t.status not in allowed.get(task.status, ()):
                raise ModelError("illegal task lifecycle transition")
            if t.status is TaskStatus.IN_PROGRESS:
                tasks[task.id] = replace(task, status=t.status, started_time=t.timestamp)
            else:
                if task.assigned_uav_id is not None:
                    detach(task.assigned_uav_id)
                tasks[task.id] = replace(task, status=t.status, assigned_uav_id=None,
                                         completed_time=t.timestamp if t.status is TaskStatus.COMPLETED else None)
        elif type(t) is MarkUAVFailed:
            if t.failure_status not in (FailureStatus.FAILED, FailureStatus.LOST):
                raise ModelError("failure transition requires FAILED or LOST")
            u = uavs[t.uav_id]
            if u.assigned_task_id is not None:
                detach(u.id)
            uavs[u.id] = replace(uavs[u.id], active=False, failure_status=t.failure_status)
            # Referential cleanup only: no fault detection, routing or reassignment algorithm.
            for uid, other in tuple(uavs.items()):
                if uid == u.id or u.id in other.route_to_gcs:
                    uavs[uid] = replace(other, connected_to_gcs=False, route_to_gcs=(),
                                        hop_count=0, parent_relay_id=None)
            network = replace(network, links=tuple(
                replace(link, active=False) if u.id in (link.source_id, link.target_id) else link
                for link in network.links))
        elif type(t) is SetRTHState:
            u = available(t.uav_id)
            allowed = {RTHState.NONE: (RTHState.REQUESTED,),
                       RTHState.REQUESTED: (RTHState.RETURNING, RTHState.NONE),
                       RTHState.RETURNING: (RTHState.ARRIVED,),
                       RTHState.ARRIVED: (RTHState.NONE,)}
            if t.rth_state not in allowed[u.rth_state]:
                raise ModelError("illegal RTH lifecycle transition")
            if t.rth_state is RTHState.RETURNING and u.assigned_task_id is not None:
                raise ModelError("unassign task before starting RTH")
            uavs[u.id] = replace(u, rth_state=t.rth_state,
                                last_role_change=t.timestamp if t.rth_state is RTHState.RETURNING
                                and u.role is not UAVRole.RETURN_TO_HOME else u.last_role_change,
                                role=UAVRole.RETURN_TO_HOME if t.rth_state is RTHState.RETURNING else u.role)
        elif type(t) is UpdateHandoverState:
            u = available(t.uav_id)
            allowed = {
                HandoverState.NONE: (HandoverState.PENDING,),
                HandoverState.PENDING: (HandoverState.PREPARING, HandoverState.ABORTED),
                HandoverState.PREPARING: (HandoverState.VERIFYING, HandoverState.ABORTED),
                HandoverState.VERIFYING: (HandoverState.COMPLETED, HandoverState.ABORTED),
                HandoverState.COMPLETED: (HandoverState.NONE,),
                HandoverState.ABORTED: (HandoverState.NONE,),
            }
            if t.handover_state not in allowed[u.handover_state]:
                raise ModelError("illegal handover lifecycle transition")
            uavs[u.id] = replace(u, handover_state=t.handover_state)
        else:
            raise ModelError("unsupported transition type")
        return replace(state, uavs=tuple(uavs.values()), tasks=tuple(tasks.values()), network=network)
