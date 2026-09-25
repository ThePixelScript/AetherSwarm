"""A0 Autonomy Adapter bridging authoritative StateSnapshot to A0Allocator."""
from __future__ import annotations
import inspect
from typing import Any, List
from ares_swarm.core.commands import AssignTaskCommand
from ares_swarm.core.enums import RTHState, TaskStatus, Role
from ares_swarm.core.models import StateSnapshot

class A0AutonomyAdapter:
    def __init__(self, allocator: Any, ingress_coordinator: Any = None) -> None:
        self.allocator = allocator
        self.ingress_coordinator = ingress_coordinator

    def plan(self, snapshot: StateSnapshot, network_analysis: Any = None) -> List[Any]:
        if self.ingress_coordinator and not self.ingress_coordinator.deployment_complete:
            return self.ingress_coordinator.plan(snapshot, network_analysis)

        eligible_uavs = [
            u for u in sorted(snapshot.uavs.values(), key=lambda x: x.id)
            if u.active and u.assigned_task_id is None and u.rth_state == RTHState.NONE and u.role != Role.RELAY
        ]
        pending_tasks = [
            t for t in sorted(snapshot.tasks.values(), key=lambda x: x.id)
            if t.status in (TaskStatus.PENDING, TaskStatus.DEFERRED)
        ]

        if not eligible_uavs or not pending_tasks:
            return []

        kwargs: dict[str, Any] = {"simulation_time": snapshot.simulation_time}

        # Explicit capability detection (preserving cbb9776 design principle)
        accepts_net = getattr(self.allocator, "accepts_network_analysis", None)
        if accepts_net is None:
            try:
                sig = inspect.signature(self.allocator.allocate)
                accepts_net = (
                    "network_analysis" in sig.parameters
                    or any(p.kind == inspect.Parameter.VAR_KEYWORD for p in sig.parameters.values())
                )
            except (ValueError, TypeError):
                accepts_net = False

        if accepts_net:
            kwargs["network_analysis"] = network_analysis

        accepts_snap = getattr(self.allocator, "accepts_snapshot", None)
        if accepts_snap is None:
            try:
                sig = inspect.signature(self.allocator.allocate)
                accepts_snap = (
                    "snapshot" in sig.parameters
                    or any(p.kind == inspect.Parameter.VAR_KEYWORD for p in sig.parameters.values())
                )
            except (ValueError, TypeError):
                accepts_snap = False

        if accepts_snap:
            kwargs["snapshot"] = snapshot

        allocation_result = self.allocator.allocate(
            eligible_uavs,
            pending_tasks,
            **kwargs,
        )
        commands: List[AssignTaskCommand] = []

        for assignment in sorted(allocation_result.assignments, key=lambda a: a.uav_id):
            commands.append(
                AssignTaskCommand(
                    source_tick=snapshot.simulation_tick,
                    uav_id=assignment.uav_id,
                    task_id=assignment.task_id,
                )
            )
        return commands
