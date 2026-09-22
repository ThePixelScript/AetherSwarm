"""Deterministic baseline task allocator (A0) for ARES-Swarm autonomy.

Stage-1 CPU-only deterministic task assignment based on multi-factor utility
scoring and deterministic tie-breaking.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Mapping, Protocol, Sequence, runtime_checkable


@runtime_checkable
class PositionProtocol(Protocol):
    """Protocol for 2D position objects."""
    x: float
    y: float


@dataclass(frozen=True, slots=True)
class AllocationWeights:
    """Weights for task allocation utility scoring.

    Utility formula:
        Utility(i, j) = wP * priority
                        - wT * travel_cost
                        - wE * energy_risk
                        + wC * connectivity_contribution
                        - wN * network_risk
                        - wS * switching_cost

    In A0, only terms with authoritative data from shared interfaces are used.
    Connectivity contribution (wC) and network risk (wN) are reserved for A1-A5
    and default to 0.0 in A0 baseline to avoid fabricating communication data.
    """
    wP: float = 1.0       # Weight for task priority
    wT: float = 0.001     # Weight for travel distance (in meters)
    wE: float = 0.1       # Weight for energy depletion risk [0, 1]
    wC: float = 0.0       # Connectivity contribution weight (reserved for future A1+)
    wN: float = 0.0       # Network risk weight (reserved for future A2+)
    wS: float = 1.0       # Task switching penalty weight


@dataclass(frozen=True, slots=True)
class TaskAllocatorConfig:
    """Configuration for deterministic A0 task allocation."""
    weights: AllocationWeights = field(default_factory=AllocationWeights)
    min_battery_pct: float = 15.0         # Minimum battery % required to accept tasks
    min_safety_reserve_wh: float = 10.0   # Fallback reserve energy (Wh) if unspecified
    allow_reassignment: bool = False       # Future-compatible flag; A0 allocates pending tasks without preemption
    eligible_roles: tuple[str, ...] = ("IDLE", "SCOUT", "EMERGENCY_SCOUT")


@dataclass(frozen=True, slots=True)
class UtilityScore:
    """Breakdown of calculated utility for a UAV-task candidate pair."""
    total: float
    priority_term: float
    travel_cost_term: float
    energy_risk_term: float
    connectivity_term: float
    network_risk_term: float
    switching_cost_term: float


@dataclass(frozen=True, slots=True)
class TaskAssignment:
    """Immutable assignment decision pairing a UAV with a task."""
    uav_id: str
    task_id: str
    score: float
    score_breakdown: UtilityScore | None = None


@dataclass(frozen=True, slots=True)
class TaskActionProposal:
    """Proposal emitted to the simulation engine, conforming to ActionProposal schema."""
    id: str
    snapshot_revision: int
    intent: str
    target: str
    reason: str
    source: str
    parameters: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class AllocationResult:
    """Immutable output from an allocation pass."""
    assignments: tuple[TaskAssignment, ...]
    unassigned_tasks: tuple[str, ...]
    unassigned_uavs: tuple[str, ...]
    infeasible_uavs: Mapping[str, str] = field(default_factory=dict)
    infeasible_tasks: Mapping[str, str] = field(default_factory=dict)
    snapshot_revision: int = 0

    def to_action_proposals(self) -> list[TaskActionProposal]:
        """Convert assignments into simulation-engine-compatible ActionProposals."""
        proposals: list[TaskActionProposal] = []
        for a in self.assignments:
            proposals.append(
                TaskActionProposal(
                    id=f"proposal-assign-{self.snapshot_revision}-{a.uav_id}-{a.task_id}",
                    snapshot_revision=self.snapshot_revision,
                    intent="ASSIGN_TASK",
                    target=a.uav_id,
                    reason=f"A0 task assignment with utility {a.score:.4f}",
                    source="task_allocator",
                    parameters={"task_id": a.task_id, "score": a.score},
                )
            )
        return proposals


def _extract_position(pos: Any) -> tuple[float, float]:
    """Extract (x, y) coordinates from a Vector2D-like object or sequence."""
    if hasattr(pos, "x") and hasattr(pos, "y"):
        return float(pos.x), float(pos.y)
    if isinstance(pos, (list, tuple)) and len(pos) >= 2:
        return float(pos[0]), float(pos[1])
    raise ValueError(f"Cannot extract 2D position coordinates from: {pos!r}")


def _euclidean_distance(p1: tuple[float, float], p2: tuple[float, float]) -> float:
    """Compute 2D Euclidean distance between two coordinate tuples."""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


class A0TaskAllocator:
    """Baseline deterministic task allocator (A0).

    Operates strictly on read-only snapshots and returns typed immutable plans.
    Uses greedy priority-first matching with deterministic tie-breaking:
    - Tasks are ordered by descending priority, descending emergency status, ascending ID.
    - UAV candidates are scored via the multi-factor utility function.
    - Equal utility scores are broken deterministically by ascending UAV ID.
    - No random choices, no GNN models, no simulation state mutation.
    """

    def __init__(self, config: TaskAllocatorConfig | None = None) -> None:
        self.config = config or TaskAllocatorConfig()

    def is_uav_feasible(
        self,
        uav: Any,
        simulation_time: float = 0.0,
    ) -> tuple[bool, str]:
        """Check whether a UAV is eligible for task assignment."""
        # Active status
        if not getattr(uav, "active", True):
            return False, "UAV is inactive"

        # Failure status
        failure = getattr(uav, "failure_state", getattr(uav, "failure_status", "NORMAL"))
        failure_str = (failure.value if hasattr(failure, "value") else str(failure)).upper().split(".")[-1]
        if failure_str in ("FAILED", "LOST"):
            return False, f"UAV failure status is {failure_str}"

        # Return to Home (RTH) status
        rth_state_obj = getattr(uav, "rth_state", "NONE")
        rth_state = (rth_state_obj.value if hasattr(rth_state_obj, "value") else str(rth_state_obj)).upper().split(".")[-1]
        if rth_state in ("REQUIRED", "ACTIVE", "COMPLETE", "RETURNING", "REQUESTED"):
            return False, f"UAV is in RTH state {rth_state}"

        # Sortie State eligibility
        sortie_state_obj = getattr(uav, "sortie_state", None)
        if sortie_state_obj is not None:
            s_val = (sortie_state_obj.value if hasattr(sortie_state_obj, "value") else str(sortie_state_obj)).upper().split(".")[-1]
            if s_val in ("RECHARGING", "RTH", "LANDING", "LANDED"):
                return False, f"UAV is in sortie state {s_val}"

        # Role eligibility
        role = str(getattr(uav, "role", "IDLE")).upper()
        if role in ("UAVROLE.RETURN_TO_HOME", "RETURN_TO_HOME"):
            return False, "UAV is returning to home"

        clean_role = role.split(".")[-1]
        eligible_clean = [r.split(".")[-1].upper() for r in self.config.eligible_roles]
        if clean_role not in eligible_clean:
            return False, f"UAV role '{role}' is not in eligible roles"

        # Time locks and cooldowns
        role_lock = float(getattr(uav, "role_lock_until", 0.0))
        assignment_lock = float(getattr(uav, "assignment_lock_until", 0.0))
        cooldown = float(getattr(uav, "cooldown_until", 0.0))
        max_lock = max(role_lock, assignment_lock, cooldown)
        if simulation_time < max_lock:
            return False, f"UAV locked or cooling down until t={max_lock:.1f} (sim_time={simulation_time:.1f})"

        # Pre-existing assignment lock
        assigned_task = getattr(uav, "assigned_task_id", None)
        if not self.config.allow_reassignment and assigned_task is not None:
            return False, f"UAV already assigned to task '{assigned_task}'"

        # Energy feasibility checks when telemetry is present
        energy_remaining = getattr(uav, "battery_energy", getattr(uav, "energy_remaining", None))
        estimated_rth = getattr(uav, "estimated_rth_energy", None)
        safety_reserve = getattr(uav, "safety_reserve", self.config.min_safety_reserve_wh)
        if energy_remaining is not None and estimated_rth is not None:
            available_energy = float(energy_remaining) - float(estimated_rth) - float(safety_reserve)
            if available_energy <= 0.0:
                return False, (
                    f"Insufficient energy: remaining {float(energy_remaining):.1f}Wh "
                    f"<= RTH {float(estimated_rth):.1f}Wh + reserve {float(safety_reserve):.1f}Wh"
                )

        battery_pct = getattr(uav, "battery_percent", getattr(uav, "battery_pct", None))
        if battery_pct is not None and float(battery_pct) <= self.config.min_battery_pct:
            return False, f"Battery level {float(battery_pct):.1f}% below minimum threshold {self.config.min_battery_pct:.1f}%"

        return True, ""

    def is_task_feasible(
        self,
        task: Any,
        simulation_time: float = 0.0,
    ) -> tuple[bool, str]:
        """Check whether a task is eligible for allocation."""
        # Lifecycle status
        status_obj = getattr(task, "status", "PENDING")
        status = (status_obj.value if hasattr(status_obj, "value") else str(status_obj)).upper().split(".")[-1]
        if status in ("COMPLETED", "COMPLETE", "FAILED", "CANCELLED", "UNREACHABLE"):
            return False, f"Task is in terminal state '{status}'"
        if status in ("ASSIGNED", "IN_PROGRESS"):
            assignee = getattr(task, "assigned_uav_id", None)
            return False, f"Task is already active (status='{status}', assigned_to='{assignee}')"

        # Task deadline
        deadline = getattr(task, "deadline", None)
        if deadline is not None and float(deadline) > 0.0 and simulation_time >= float(deadline):
            return False, f"Task deadline {float(deadline):.1f}s expired at t={simulation_time:.1f}s"

        return True, ""

    def compute_utility(
        self,
        uav: Any,
        task: Any,
    ) -> UtilityScore:
        """Compute the multi-factor utility score for assigning task to UAV.

        Formula:
            Utility(i, j) = wP*priority - wT*travel_cost - wE*energy_risk
                            + wC*connectivity_contribution - wN*network_risk
                            - wS*switching_cost
        """
        w = self.config.weights

        # 1. Priority term
        priority = float(getattr(task, "priority", 1.0))
        priority_term = w.wP * priority

        # 2. Travel cost term (Euclidean distance)
        uav_pos_val = getattr(uav, "position_xy", getattr(uav, "position", None))
        task_pos_val = getattr(task, "position_xy", getattr(task, "position", None))
        uav_pos = _extract_position(uav_pos_val)
        task_pos = _extract_position(task_pos_val)
        travel_cost = _euclidean_distance(uav_pos, task_pos)
        travel_cost_term = w.wT * travel_cost

        # 3. Energy risk term [0.0, 1.0] when data is available
        battery_pct = getattr(uav, "battery_percent", getattr(uav, "battery_pct", None))
        energy_remaining = getattr(uav, "battery_energy", getattr(uav, "energy_remaining", None))
        if battery_pct is not None:
            energy_risk = max(0.0, min(1.0, (100.0 - float(battery_pct)) / 100.0))
        elif energy_remaining is not None:
            capacity = float(getattr(uav, "battery_capacity", 100.0))
            energy_risk = max(0.0, min(1.0, (capacity - float(energy_remaining)) / max(1e-6, capacity)))
        else:
            energy_risk = 0.0
        energy_risk_term = w.wE * energy_risk

        # 4. Connectivity contribution (0.0 in A0 baseline, no relay logic)
        connectivity_contribution = 0.0
        connectivity_term = w.wC * connectivity_contribution

        # 5. Network risk (0.0 in A0 baseline, no predictive/GNN logic)
        network_risk = 0.0
        network_risk_term = w.wN * network_risk

        # 6. Switching cost (penalty if UAV is already committed elsewhere)
        assigned_task_id = getattr(uav, "assigned_task_id", None)
        is_switching = 1.0 if (assigned_task_id is not None and assigned_task_id != task.id) else 0.0
        switching_cost_term = w.wS * is_switching

        total = (
            priority_term
            - travel_cost_term
            - energy_risk_term
            + connectivity_term
            - network_risk_term
            - switching_cost_term
        )

        return UtilityScore(
            total=total,
            priority_term=priority_term,
            travel_cost_term=travel_cost_term,
            energy_risk_term=energy_risk_term,
            connectivity_term=connectivity_term,
            network_risk_term=network_risk_term,
            switching_cost_term=switching_cost_term,
        )

    def allocate(
        self,
        snapshot_or_uavs: Any = None,
        tasks: Sequence[Any] | None = None,
        *,
        uavs: Sequence[Any] | None = None,
        simulation_time: float | None = None,
        snapshot_revision: int = 0,
    ) -> AllocationResult:
        """Deterministically allocate pending tasks to feasible UAVs.

        Supports both direct collections (uavs, tasks) and snapshot objects.
        """
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
            # StateSnapshot protocol
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

        # Filter and record task feasibility
        feasible_tasks: list[Any] = []
        infeasible_tasks: dict[str, str] = {}
        for t in raw_tasks:
            t_id = str(getattr(t, "id", ""))
            ok, reason = self.is_task_feasible(t, sim_time)
            if ok:
                feasible_tasks.append(t)
            else:
                infeasible_tasks[t_id] = reason

        # Filter and record UAV feasibility
        feasible_uavs: list[Any] = []
        infeasible_uavs: dict[str, str] = {}
        for u in raw_uavs:
            u_id = str(getattr(u, "id", ""))
            ok, reason = self.is_uav_feasible(u, sim_time)
            if ok:
                feasible_uavs.append(u)
            else:
                infeasible_uavs[u_id] = reason

        # Deterministic ordering for tasks:
        # 1. Priority descending (-priority)
        # 2. Emergency status descending (True before False)
        # 3. Task ID ascending (lexicographical string sort)
        feasible_tasks.sort(
            key=lambda t: (
                -float(getattr(t, "priority", 1.0)),
                0 if (getattr(t, "emergency_flag", False) or getattr(t, "is_emergency", False)) else 1,
                str(getattr(t, "id", "")),
            )
        )

        # Available UAV candidate pool mapped by ID
        available_uavs: dict[str, Any] = {str(u.id): u for u in feasible_uavs}

        assignments: list[TaskAssignment] = []
        unassigned_tasks: list[str] = []

        # Deterministic greedy priority-first matching
        for task in feasible_tasks:
            t_id = str(task.id)
            if not available_uavs:
                unassigned_tasks.append(t_id)
                continue

            # Evaluate utility across all remaining feasible candidates
            scored_candidates: list[tuple[float, str, Any, UtilityScore]] = []
            for u_id, uav in available_uavs.items():
                score = self.compute_utility(uav, task)
                # Round score to 8 decimals to prevent platform floating-point epsilon noise
                scored_candidates.append((round(score.total, 8), u_id, uav, score))

            if not scored_candidates:
                unassigned_tasks.append(t_id)
                continue

            # Deterministic tie-breaking:
            # 1. Score descending (-rounded_score)
            # 2. UAV ID ascending (lexicographical)
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
            # Remove assigned UAV from pool (single-task allocation)
            del available_uavs[best_uav_id]

        # Remaining UAVs that received no task
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
    ) -> list[TaskActionProposal]:
        """Autonomous planner interface implementation.

        Conforms to AutonomyPlanner protocol. Network analysis is accepted for
        pipeline compatibility but unused in A0 baseline.
        """
        result = self.allocate(snapshot)
        return result.to_action_proposals()


# Compatibility alias
TaskAllocator = A0TaskAllocator
