"""A0 Autonomy Adapter bridging authoritative StateSnapshot to A0Allocator."""
from __future__ import annotations
from typing import Any, List
from ares_swarm.core.commands import AssignTaskCommand
from ares_swarm.core.enums import RTHState, TaskStatus
from ares_swarm.core.models import StateSnapshot

class A0AutonomyAdapter:
    def __init__(self, allocator: Any) -> None:
        self.allocator = allocator

    def plan(self, snapshot: StateSnapshot, network_analysis: Any = None) -> List[AssignTaskCommand]:
        eligible_uavs = [
            u for u in sorted(snapshot.uavs.values(), key=lambda x: x.id)
            if u.active and u.assigned_task_id is None and u.rth_state == RTHState.NONE
        ]
        pending_tasks = [
            t for t in sorted(snapshot.tasks.values(), key=lambda x: x.id)
            if t.status in (TaskStatus.PENDING, TaskStatus.DEFERRED)
        ]

        if not eligible_uavs or not pending_tasks:
            return []

        try:
            allocation_result = self.allocator.allocate(
                eligible_uavs,
                pending_tasks,
                simulation_time=snapshot.simulation_time,
                network_analysis=network_analysis,
            )
        except TypeError:
            allocation_result = self.allocator.allocate(
                eligible_uavs,
                pending_tasks,
                simulation_time=snapshot.simulation_time,
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
