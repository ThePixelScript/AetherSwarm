"""A0 Autonomy Adapter bridging authoritative StateSnapshot to A0Allocator."""
from __future__ import annotations
from typing import Any, List
from ares_swarm.core.commands import AssignTaskCommand
from ares_swarm.core.enums import RTHState, TaskStatus
from ares_swarm.core.models import StateSnapshot

class A0AutonomyAdapter:
    def __init__(self, allocator: Any) -> None:
        self.allocator = allocator

    def plan(self, snapshot: StateSnapshot) -> List[AssignTaskCommand]:
        eligible_uavs = [
            u for u in sorted(snapshot.uavs.values(), key=lambda x: x.id)
            if u.active and u.assigned_task_id is None and u.rth_state == RTHState.NONE
        ]
        pending_tasks = [
            t for t in sorted(snapshot.tasks.values(), key=lambda x: x.id)
            if t.status == TaskStatus.PENDING
        ]

        if not eligible_uavs or not pending_tasks:
            return []

        assignments = self.allocator.allocate(eligible_uavs, pending_tasks)
        commands: List[AssignTaskCommand] = []

        for uav_id in sorted(assignments.keys()):
            task_id = assignments[uav_id]
            commands.append(
                AssignTaskCommand(
                    source_tick=snapshot.simulation_tick,
                    uav_id=uav_id,
                    task_id=task_id,
                )
            )
        return commands
