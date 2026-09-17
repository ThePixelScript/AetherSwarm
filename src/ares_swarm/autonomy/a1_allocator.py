import math
from typing import Any, Sequence, Mapping

from .task_allocator import (
    A0TaskAllocator,
    UtilityScore,
    AllocationResult,
    TaskAssignment,
    
)

class A1CommunicationAwareAllocator(A0TaskAllocator):
    """M1 Task Allocator incorporating NetworkAnalysis."""
    
    def __init__(self, config: Any = None):
        super().__init__(config=config)
        # Default M1 configuration values if not provided
        self.disconnected_penalty = 10000.0
        self.critical_node_penalty = 50.0

    def compute_utility_with_network(
        self,
        uav: Any,
        task: Any,
        network_analysis: Any
    ) -> UtilityScore:
        # Get baseline score from A0
        base_score = self.compute_utility(uav, task)
        
        if not network_analysis:
            return base_score
            
        u_id = str(getattr(uav, "id", ""))
        
        # Determine network risks based on M1 rules
        is_disconnected = u_id in getattr(network_analysis, "disconnected_uav_ids", ())
        is_critical = u_id in getattr(network_analysis, "articulation_points", ())
        
        network_risk_term = 0.0
        if is_disconnected:
            network_risk_term += self.disconnected_penalty
        if is_critical:
            network_risk_term += self.critical_node_penalty
            
        total = (
            base_score.priority_term
            - base_score.travel_cost_term
            - base_score.energy_risk_term
            + base_score.connectivity_term
            - network_risk_term
            - base_score.switching_cost_term
        )
        
        return UtilityScore(
            total=total,
            priority_term=base_score.priority_term,
            travel_cost_term=base_score.travel_cost_term,
            energy_risk_term=base_score.energy_risk_term,
            connectivity_term=base_score.connectivity_term,
            network_risk_term=network_risk_term,
            switching_cost_term=base_score.switching_cost_term,
        )

    def allocate(
        self,
        snapshot_or_uavs: Any = None,
        tasks: Sequence[Any] | None = None,
        *,
        uavs: Sequence[Any] | None = None,
        simulation_time: float | None = None,
        snapshot_revision: int = 0,
        network_analysis: Any = None,
    ) -> AllocationResult:
        """Deterministically allocate pending tasks to feasible UAVs."""
        # Parse inputs gracefully without requiring concrete core types
        if uavs is not None and tasks is not None:
            raw_uavs = list(uavs)
            raw_tasks = list(tasks)
            sim_time = simulation_time if simulation_time is not None else 0.0
            revision = snapshot_revision
        elif tasks is not None:
            raw_uavs = list(snapshot_or_uavs) if snapshot_or_uavs is not None else []
            raw_tasks = list(tasks)
            sim_time = simulation_time if simulation_time is not None else 0.0
            revision = snapshot_revision
        elif snapshot_or_uavs is not None and hasattr(snapshot_or_uavs, "state"):
            state = snapshot_or_uavs.state
            uavs_obj = getattr(state, "uavs", ())
            tasks_obj = getattr(state, "tasks", ())
            raw_uavs = list(uavs_obj.values() if isinstance(uavs_obj, Mapping) else uavs_obj)
            raw_tasks = list(tasks_obj.values() if isinstance(tasks_obj, Mapping) else tasks_obj)
            sim_time = simulation_time if simulation_time is not None else float(getattr(state, "simulation_time", 0.0))
            revision = snapshot_revision or int(getattr(snapshot_or_uavs, "revision", getattr(snapshot_or_uavs, "state_version", 0)))
        elif snapshot_or_uavs is not None and isinstance(snapshot_or_uavs, dict):
            uavs_obj = snapshot_or_uavs.get("uavs", ())
            tasks_obj = snapshot_or_uavs.get("tasks", ())
            raw_uavs = list(uavs_obj.values() if isinstance(uavs_obj, Mapping) else uavs_obj)
            raw_tasks = list(tasks_obj.values() if isinstance(tasks_obj, Mapping) else tasks_obj)
            sim_time = simulation_time if simulation_time is not None else float(snapshot_or_uavs.get("simulation_time", 0.0))
            revision = snapshot_revision or int(snapshot_or_uavs.get("revision", snapshot_or_uavs.get("state_version", 0)))
        elif snapshot_or_uavs is not None:
            uavs_obj = getattr(snapshot_or_uavs, "uavs", ())
            tasks_obj = getattr(snapshot_or_uavs, "tasks", ())
            raw_uavs = list(uavs_obj.values() if isinstance(uavs_obj, Mapping) else uavs_obj)
            raw_tasks = list(tasks_obj.values() if isinstance(tasks_obj, Mapping) else tasks_obj)
            sim_time = simulation_time if simulation_time is not None else float(getattr(snapshot_or_uavs, "simulation_time", 0.0))
            revision = snapshot_revision or int(getattr(snapshot_or_uavs, "revision", getattr(snapshot_or_uavs, "state_version", 0)))
        else:
            raw_uavs = []
            raw_tasks = []
            sim_time = simulation_time if simulation_time is not None else 0.0
            revision = snapshot_revision

        feasible_tasks: list[Any] = []
        infeasible_tasks: dict[str, str] = {}
        for t in raw_tasks:
            t_id = str(getattr(t, "id", ""))
            ok, reason = self.is_task_feasible(t, sim_time)
            if ok:
                feasible_tasks.append(t)
            else:
                infeasible_tasks[t_id] = reason

        feasible_uavs: list[Any] = []
        infeasible_uavs: dict[str, str] = {}
        for u in raw_uavs:
            u_id = str(getattr(u, "id", ""))
            ok, reason = self.is_uav_feasible(u, sim_time)
            if ok:
                feasible_uavs.append(u)
            else:
                infeasible_uavs[u_id] = reason

        feasible_tasks.sort(
            key=lambda t: (
                -float(getattr(t, "priority", 1.0)),
                0 if (getattr(t, "emergency_flag", False) or getattr(t, "is_emergency", False)) else 1,
                str(getattr(t, "id", "")),
            )
        )

        available_uavs: dict[str, Any] = {str(u.id): u for u in feasible_uavs}

        assignments: list[TaskAssignment] = []
        unassigned_tasks: list[str] = []

        for task in feasible_tasks:
            t_id = str(task.id)
            if not available_uavs:
                unassigned_tasks.append(t_id)
                continue

            scored_candidates: list[tuple[float, str, Any, UtilityScore]] = []
            for u_id, uav in available_uavs.items():
                score = self.compute_utility_with_network(uav, task, network_analysis)
                scored_candidates.append((round(score.total, 8), u_id, uav, score))

            if not scored_candidates:
                unassigned_tasks.append(t_id)
                continue

            scored_candidates.sort(key=lambda item: (-item[0], item[1]))

            best_score, best_uav_id, best_uav, breakdown = scored_candidates[0]

            assignments.append(
                TaskAssignment(
                    uav_id=best_uav_id,
                    task_id=t_id,
                    score=breakdown.total,
                    score_breakdown=breakdown,
                )
            )
            del available_uavs[best_uav_id]

        unassigned_uavs = sorted(available_uavs.keys())

        return AllocationResult(
            assignments=tuple(assignments),
            unassigned_tasks=tuple(unassigned_tasks),
            unassigned_uavs=tuple(unassigned_uavs),
            infeasible_uavs=infeasible_uavs,
            infeasible_tasks=infeasible_tasks,
            snapshot_revision=revision,
        )

    def plan(
        self,
        snapshot: Any,
        network_analysis: Any = None,
    ) -> list[Any]:
        result = self.allocate(snapshot, network_analysis=network_analysis)
        return result.to_action_proposals()
