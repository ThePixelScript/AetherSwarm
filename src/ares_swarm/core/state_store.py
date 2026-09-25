"""Authoritative StateStore implementation for AetherSwarm."""
from __future__ import annotations

from dataclasses import replace
from types import MappingProxyType
from typing import Dict, List, Optional, Sequence, Tuple

from .commands import (
    AssignTaskCommand,
    Command,
    CompleteRTHCommand,
    FailUAVCommand,
    RecoverUAVCommand,
    ProgressTaskCommand,
    ReleaseTaskCommand,
    SetTargetPositionCommand,
    AssignRelayRoleCommand,
    StartRTHCommand,
    StepPhysicsCommand,
)
from .constants import EPSILON
from .enums import EventType, FailureState, RejectionCode, Role, RTHState, TaskStatus
from .events import CommandRejection, DomainEvent, StateTransitionResult
from .models import StateSnapshot, TaskState, UAVState


class StateStore:
    """Authoritative mutable state owner for swarm simulation."""

    def __init__(self, initial_snapshot: Optional[StateSnapshot] = None) -> None:
        self._uavs: Dict[str, UAVState] = {}
        self._tasks: Dict[str, TaskState] = {}
        self._simulation_tick: int = 0
        self._simulation_time: float = 0.0
        self._state_version: int = 0
        self._gcs_position: Tuple[float, float] = (0.0, 0.0)

        if initial_snapshot:
            self._uavs = {u.id: u for u in initial_snapshot.uavs.values()}
            self._tasks = {t.id: t for t in initial_snapshot.tasks.values()}
            self._simulation_tick = initial_snapshot.simulation_tick
            self._simulation_time = initial_snapshot.simulation_time
            self._state_version = initial_snapshot.state_version
            self._gcs_position = initial_snapshot.gcs_position

    def snapshot(self) -> StateSnapshot:
        """Return an immutable point-in-time snapshot of current state."""
        return StateSnapshot(
            simulation_tick=self._simulation_tick,
            simulation_time=self._simulation_time,
            state_version=self._state_version,
            uavs=MappingProxyType(dict(self._uavs)),
            tasks=MappingProxyType(dict(self._tasks)),
            gcs_position=self._gcs_position,
        )

    def set_simulation_clock(self, tick: int, sim_time: float) -> None:
        """Advance simulation clock. Never clears entities."""
        if tick < self._simulation_tick or sim_time < self._simulation_time:
            raise ValueError(
                f"Time cannot move backwards: tick {tick} < {self._simulation_tick} or time {sim_time} < {self._simulation_time}"
            )
        self._simulation_tick = tick
        self._simulation_time = sim_time

    def apply(self, commands: Sequence[Command]) -> StateTransitionResult:
        """Apply commands with partitioned per-command transactional atomicity."""
        sorted_commands = sorted(commands, key=lambda c: (c.uav_id, type(c).__name__))

        applied: List[Command] = []
        rejected: List[CommandRejection] = []
        events: List[DomainEvent] = []

        for cmd in sorted_commands:
            if cmd.source_tick != self._simulation_tick:
                rejected.append(
                    CommandRejection(
                        command=cmd,
                        code=RejectionCode.STALE_TICK_PROPOSAL,
                        reason=f"Command tick {cmd.source_tick} != store tick {self._simulation_tick}",
                    )
                )
                continue

            uav = self._uavs.get(cmd.uav_id)
            if not uav:
                rejected.append(
                    CommandRejection(
                        command=cmd,
                        code=RejectionCode.ENTITY_NOT_FOUND,
                        reason=f"UAV {cmd.uav_id} not found",
                    )
                )
                continue

            staged_uavs: Dict[str, UAVState] = {}
            staged_tasks: Dict[str, TaskState] = {}
            staged_events: List[DomainEvent] = []
            rejection: Optional[CommandRejection] = None

            if isinstance(cmd, AssignTaskCommand):
                task = self._tasks.get(cmd.task_id)
                if not task:
                    rejection = CommandRejection(cmd, RejectionCode.ENTITY_NOT_FOUND, f"Task {cmd.task_id} not found")
                elif not uav.active or uav.rth_state != RTHState.NONE or uav.failure_state == FailureState.FAILED:
                    rejection = CommandRejection(cmd, RejectionCode.ILLEGAL_LIFECYCLE_TRANSITION, "UAV unavailable")
                elif uav.assignment_lock_until > self._simulation_time:
                    rejection = CommandRejection(cmd, RejectionCode.LOCKED_ASSIGNMENT, "UAV assignment locked")
                elif task.status not in (TaskStatus.PENDING, TaskStatus.DEFERRED):
                    rejection = CommandRejection(cmd, RejectionCode.ILLEGAL_LIFECYCLE_TRANSITION, f"Task in status {task.status}")
                elif task.assigned_uav_id is not None and task.assigned_uav_id != uav.id:
                    rejection = CommandRejection(cmd, RejectionCode.CONFLICTING_COMMAND, "Task assigned to another UAV")
                else:
                    staged_tasks[task.id] = replace(task, status=TaskStatus.ASSIGNED, assigned_uav_id=uav.id)
                    staged_uavs[uav.id] = replace(uav, assigned_task_id=task.id, target_position=task.position_xy)
                    staged_events.append(DomainEvent.create(
                        simulation_tick=self._simulation_tick,
                        simulation_time=self._simulation_time,
                        event_type=EventType.TASK_ASSIGNED,
                        entity_id=uav.id,
                        payload={"task_id": task.id},
                        sequence=len(events) + len(staged_events),
                    ))

            elif isinstance(cmd, ReleaseTaskCommand):
                task = self._tasks.get(cmd.task_id)
                if not task or task.assigned_uav_id != uav.id:
                    rejection = CommandRejection(cmd, RejectionCode.CONFLICTING_COMMAND, "UAV does not own this task")
                else:
                    staged_tasks[task.id] = replace(task, status=TaskStatus.DEFERRED, assigned_uav_id=None)
                    staged_uavs[uav.id] = replace(uav, assigned_task_id=None, target_position=None)
                    staged_events.append(DomainEvent.create(
                        simulation_tick=self._simulation_tick,
                        simulation_time=self._simulation_time,
                        event_type=EventType.TASK_DEFERRED,
                        entity_id=task.id,
                        payload={"uav_id": uav.id, "reason": cmd.reason},
                        sequence=len(events) + len(staged_events),
                    ))

            elif isinstance(cmd, StartRTHCommand):
                if uav.assigned_task_id and uav.assigned_task_id in self._tasks:
                    old_task = self._tasks[uav.assigned_task_id]
                    staged_tasks[old_task.id] = replace(old_task, status=TaskStatus.DEFERRED, assigned_uav_id=None)
                staged_uavs[uav.id] = replace(
                    uav,
                    rth_state=RTHState.ACTIVE,
                    target_position=self._gcs_position,
                    assigned_task_id=None,
                )
                staged_events.append(DomainEvent.create(
                    simulation_tick=self._simulation_tick,
                    simulation_time=self._simulation_time,
                    event_type=EventType.RTH_TRIGGERED,
                    entity_id=uav.id,
                    payload={},
                    sequence=len(events) + len(staged_events),
                ))

            elif isinstance(cmd, CompleteRTHCommand):
                if uav.rth_state != RTHState.ACTIVE:
                    rejection = CommandRejection(
                        cmd,
                        RejectionCode.ILLEGAL_LIFECYCLE_TRANSITION,
                        f"UAV {uav.id} is in RTHState {uav.rth_state}, expected ACTIVE",
                    )
                else:
                    staged_uavs[uav.id] = replace(
                        uav,
                        rth_state=RTHState.COMPLETE,
                        role=Role.IDLE,
                        velocity_xy=(0.0, 0.0),
                        target_position=None,
                        active=False,
                    )
                    staged_events.append(DomainEvent.create(
                        simulation_tick=self._simulation_tick,
                        simulation_time=self._simulation_time,
                        event_type=EventType.UAV_LANDED,
                        entity_id=uav.id,
                        payload={"final_energy": uav.battery_energy},
                        sequence=len(events) + len(staged_events),
                    ))

            elif isinstance(cmd, FailUAVCommand):
                target_failure = cmd.failure_state
                if uav.failure_state == target_failure:
                    rejection = CommandRejection(
                        cmd,
                        RejectionCode.ILLEGAL_LIFECYCLE_TRANSITION,
                        f"UAV {uav.id} already in failure state {target_failure}",
                    )
                else:
                    new_active = False if target_failure == FailureState.FAILED else uav.active
                    # If UAV drops into FAILED state, drop any assigned task
                    if target_failure == FailureState.FAILED and uav.assigned_task_id and uav.assigned_task_id in self._tasks:
                        old_task = self._tasks[uav.assigned_task_id]
                        staged_tasks[old_task.id] = replace(old_task, status=TaskStatus.DEFERRED, assigned_uav_id=None)
                        staged_uavs[uav.id] = replace(
                            uav,
                            failure_state=target_failure,
                            active=new_active,
                            assigned_task_id=None,
                            velocity_xy=(0.0, 0.0),
                            target_position=None,
                        )
                    else:
                        staged_uavs[uav.id] = replace(
                            uav,
                            failure_state=target_failure,
                            active=new_active,
                        )
                    staged_events.append(DomainEvent.create(
                        simulation_tick=self._simulation_tick,
                        simulation_time=self._simulation_time,
                        event_type=EventType.UAV_FAILED,
                        entity_id=uav.id,
                        payload={"failure_state": target_failure.value, "reason": cmd.reason},
                        sequence=len(events) + len(staged_events),
                    ))

            elif isinstance(cmd, RecoverUAVCommand):
                if uav.failure_state == FailureState.NORMAL and uav.active:
                    rejection = CommandRejection(
                        cmd,
                        RejectionCode.ILLEGAL_LIFECYCLE_TRANSITION,
                        f"UAV {uav.id} is already NORMAL and active",
                    )
                else:
                    staged_uavs[uav.id] = replace(
                        uav,
                        failure_state=FailureState.NORMAL,
                        active=True,
                        role=Role.IDLE,
                    )
                    staged_events.append(DomainEvent.create(
                        simulation_tick=self._simulation_tick,
                        simulation_time=self._simulation_time,
                        event_type=EventType.UAV_ACTIVATED,
                        entity_id=uav.id,
                        payload={"uav_id": uav.id},
                        sequence=len(events) + len(staged_events),
                    ))

            elif isinstance(cmd, SetTargetPositionCommand):
                staged_uavs[uav.id] = replace(uav, target_position=cmd.target_position)

            elif isinstance(cmd, StepPhysicsCommand):
                new_energy = max(0.0, uav.battery_energy - cmd.delta_energy)
                staged_uavs[uav.id] = replace(
                    uav,
                    position_xy=cmd.new_position_xy,
                    velocity_xy=cmd.new_velocity_xy,
                    battery_energy=new_energy,
                )

            elif isinstance(cmd, ProgressTaskCommand):
                task = self._tasks.get(cmd.task_id)
                if not task or task.assigned_uav_id != uav.id:
                    rejection = CommandRejection(cmd, RejectionCode.CONFLICTING_COMMAND, "UAV does not own this task")
                else:
                    new_progress = task.service_progress + cmd.delta_progress
                    is_complete = (new_progress >= task.service_duration - EPSILON)
                    new_status = TaskStatus.COMPLETE if is_complete else TaskStatus.IN_PROGRESS

                    staged_tasks[task.id] = replace(task, service_progress=new_progress, status=new_status)
                    if is_complete:
                        staged_tasks[task.id] = replace(staged_tasks[task.id], assigned_uav_id=None)
                        staged_uavs[uav.id] = replace(uav, assigned_task_id=None)
                        staged_events.append(DomainEvent.create(
                            simulation_tick=self._simulation_tick,
                            simulation_time=self._simulation_time,
                            event_type=EventType.TASK_COMPLETED,
                            entity_id=task.id,
                            payload={"uav_id": uav.id},
                            sequence=len(events) + len(staged_events),
                        ))
                    else:
                        staged_events.append(DomainEvent.create(
                            simulation_tick=self._simulation_tick,
                            simulation_time=self._simulation_time,
                            event_type=EventType.TASK_PROGRESS,
                            entity_id=task.id,
                            payload={"progress": new_progress, "duration": task.service_duration},
                            sequence=len(events) + len(staged_events),
                        ))

            elif isinstance(cmd, AssignRelayRoleCommand):
                staged_uavs[uav.id] = replace(
                    uav,
                    role=Role.RELAY,
                )
                staged_events.append(DomainEvent.create(
                    simulation_tick=self._simulation_tick,
                    simulation_time=self._simulation_time,
                    event_type=EventType.UAV_ACTIVATED,
                    entity_id=uav.id,
                    payload={"role": "RELAY"},
                    sequence=len(events) + len(staged_events),
                ))

            else:
                rejection = CommandRejection(
                    cmd,
                    RejectionCode.UNSUPPORTED_COMMAND,
                    f"Unsupported command type: {type(cmd).__name__}"
                )

            if rejection:
                rejected.append(rejection)
            else:
                self._uavs.update(staged_uavs)
                self._tasks.update(staged_tasks)
                events.extend(staged_events)
                applied.append(cmd)

        prev_version = self._state_version
        if applied:
            self._state_version += 1

        return StateTransitionResult(
            previous_version=prev_version,
            new_version=self._state_version,
            simulation_tick=self._simulation_tick,
            applied_commands=tuple(applied),
            rejected_commands=tuple(rejected),
            emitted_events=tuple(events),
        )
