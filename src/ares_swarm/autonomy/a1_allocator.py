"""Deterministic communication-aware task allocator (A1) for AetherSwarm.

Stage-1 communication-aware task assignment incorporating real NetworkAnalysis signals
(reachability, hop count, end-to-end route PDR) on top of the deterministic A0 baseline.

A1 v1 ARCHITECTURE & CONTRACTS:
- Inherits all operational and task feasibility gates from A0TaskAllocator without code duplication.
- Preserves A0 task ordering (descending priority, emergency status, ascending ID).
- Modifies candidate utility scoring using a bounded, deterministic communication factor:
    A1Utility(i, j) = BaseMissionUtility(i, j) * CommunicationFactor(i)
- Production mode rejects disconnected destinations and loss of connected peers.
- Accepted moves accumulate in a hypothetical batch snapshot; Gamma stays read-only.
- The communication floor applies to scoring, not a relaxation of feasibility.
- Endpoint checks do not prove continuous trajectory connectivity or relay recovery.
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

from ..communication.analysis import BaselineCommunicationAnalyzer
from ..communication.channel import LinkCondition
from ..communication.config import CommunicationConfig
from .task_allocator import (
    A0TaskAllocator,
    AllocationResult,
    AllocationWeights,
    TaskActionProposal,
    TaskAllocatorConfig,
    UtilityScore,
)


@dataclass(frozen=True, slots=True)
class A1AllocatorConfig(TaskAllocatorConfig):
    """Configuration for deterministic A1 communication-aware task allocation."""
    min_comm_factor: float = 0.2    # Non-zero floor for demoting disconnected UAVs (prevents deadlock)
    hop_decay: float = 0.85         # Hop attenuation decay factor per hop beyond 1
    default_pdr: float = 1.0        # Default PDR when edge data is unmeasured
    destination_aware: bool = True  # Evaluate communication quality at candidate destination


class A1TaskAllocator(A0TaskAllocator):
    """Deterministic communication-aware task allocator (A1).

    Inherits all operational and task feasibility gates from A0TaskAllocator.
    Applies bounded communication modifiers to candidate utility scoring based on
    authoritative NetworkAnalysis signals.
    """
    accepts_network_analysis: bool = True
    accepts_snapshot: bool = True

    def __init__(
        self,
        config: A1AllocatorConfig | TaskAllocatorConfig | None = None,
        comm_analyzer: Any = None,
    ) -> None:
        super().__init__(config=config or A1AllocatorConfig())
        self._current_network_analysis: Any = None
        self._current_snapshot: Any = None
        self._comm_analyzer: Any = comm_analyzer

    def compute_communication_factor(
        self,
        uav: Any,
        network_analysis: Any,
    ) -> float:
        """Compute deterministic bounded communication factor in [min_comm_factor, 1.0].

        Signals consumed:
        - Current reachability (connected_uav_ids vs disconnected_uav_ids)
        - Current hop count (hop_counts or route length)
        - Current end-to-end route PDR (route_pdr_to_gcs or derived from edge_metrics along routes_to_gcs)

        Disconnected UAVs are demoted to min_comm_factor, not globally rejected.
        """
        if network_analysis is None:
            return 1.0

        u_id = str(getattr(uav, "id", ""))
        min_factor = getattr(self.config, "min_comm_factor", 0.2)
        hop_decay = getattr(self.config, "hop_decay", 0.85)
        default_pdr = getattr(self.config, "default_pdr", 1.0)

        # 1. Reachability check
        connected_ids = getattr(network_analysis, "connected_uav_ids", ())
        disconnected_ids = getattr(network_analysis, "disconnected_uav_ids", ())

        # Disconnected UAVs are demoted to min_factor
        if u_id in disconnected_ids:
            return float(min_factor)
        if connected_ids and u_id not in connected_ids:
            return float(min_factor)

        # Check route presence
        routes_to_gcs = getattr(network_analysis, "routes_to_gcs", {})
        route = routes_to_gcs.get(u_id) if isinstance(routes_to_gcs, Mapping) else None
        if route is None and u_id not in connected_ids:
            return float(min_factor)

        # 2. Hop count and attenuation
        hop_counts = getattr(network_analysis, "hop_counts", {})
        hops = hop_counts.get(u_id) if isinstance(hop_counts, Mapping) else None
        if hops is None:
            hops = len(route) - 1 if route else 1
        hops = max(1, int(hops))

        hop_factor = float(hop_decay ** (hops - 1))

        # 3. End-to-end route PDR
        # Check if Gamma route_pdr_to_gcs mapping is available
        route_pdr_map = getattr(network_analysis, "route_pdr_to_gcs", None)
        if route_pdr_map and isinstance(route_pdr_map, Mapping) and u_id in route_pdr_map:
            val = route_pdr_map[u_id]
            route_pdr = float(val) if val is not None else default_pdr
        elif route and len(route) >= 2:
            # Derive e2e PDR from edge metrics along the route
            edge_metrics = getattr(network_analysis, "edge_metrics", ())
            pdr_by_pair: dict[tuple[str, str], float] = {}
            for link in edge_metrics:
                pair = (min(link.source_id, link.target_id), max(link.source_id, link.target_id))
                pdr_by_pair[pair] = float(getattr(link, "estimated_pdr", default_pdr))

            prod_pdr = 1.0
            for a, b in zip(route, route[1:]):
                pair = (min(a, b), max(a, b))
                prod_pdr *= pdr_by_pair.get(pair, default_pdr)
            route_pdr = prod_pdr
        else:
            route_pdr = default_pdr

        route_pdr = max(0.0, min(1.0, route_pdr))

        # 4. Combine factors with non-zero floor
        raw_comm = route_pdr * hop_factor
        return float(max(min_factor, min(1.0, raw_comm)))

    def _infer_comm_config_and_conditions(
        self,
        net: Any,
    ) -> tuple[CommunicationConfig, dict[tuple[str, str], LinkCondition]]:
        """Infer underlying CommunicationConfig and link conditions from edge metrics."""
        default_config = CommunicationConfig(max_range=100.0, base_latency=5.0, packet_loss=0.0)
        if net is None:
            return default_config, {}

        edge_metrics = getattr(net, "edge_metrics", ())
        if not edge_metrics:
            return default_config, {}

        inferred_ranges: list[float] = []
        inferred_latencies: list[float] = []
        inferred_losses: list[float] = []
        conditions: dict[tuple[str, str], LinkCondition] = {}

        for link in edge_metrics:
            q = float(getattr(link, "link_quality", 1.0))
            d = float(getattr(link, "distance", 0.0))
            lat = float(getattr(link, "latency_ms", 5.0))
            pdr = float(getattr(link, "estimated_pdr", 1.0))

            if q < 1.0 and d > 1e-3:
                denom = (1.0 / q) - 1.0
                if denom > 1e-6:
                    inferred_ranges.append(d / denom)
                    inferred_latencies.append(lat * q)
                    if q > 1e-6:
                        inferred_losses.append(max(0.0, min(1.0, 1.0 - (pdr / q))))

        max_range = inferred_ranges[0] if inferred_ranges else 100.0
        base_latency = inferred_latencies[0] if inferred_latencies else 5.0
        packet_loss = inferred_losses[0] if inferred_losses else 0.0

        for link in edge_metrics:
            q = float(getattr(link, "link_quality", 1.0))
            d = float(getattr(link, "distance", 0.0))
            pdr = float(getattr(link, "estimated_pdr", 1.0))
            lat = float(getattr(link, "latency_ms", base_latency))
            expected_q = 1.0 / (1.0 + d / max_range) if max_range > 0 else 1.0
            expected_pdr = (1.0 - packet_loss) * expected_q
            expected_lat = base_latency * (1.0 + d / max_range)
            pair = (min(link.source_id, link.target_id), max(link.source_id, link.target_id))

            if abs(pdr - expected_pdr) > 1e-3 or abs(lat - expected_lat) > 1e-3:
                loss_override = 1.0 - pdr
                lat_penalty = max(0.0, lat - expected_lat)
                conditions[pair] = LinkCondition(
                    packet_loss_override=loss_override,
                    latency_penalty_ms=lat_penalty,
                )

        config = CommunicationConfig(
            max_range=round(max_range, 2),
            base_latency=round(base_latency, 2),
            packet_loss=round(packet_loss, 4),
        )
        return config, conditions

    def _get_comm_analyzer(self, net: Any) -> Any:
        if self._comm_analyzer is not None:
            return self._comm_analyzer
        config, conditions = self._infer_comm_config_and_conditions(net)
        return BaselineCommunicationAnalyzer(config=config, conditions=conditions)

    def _evaluate_swarm_connectivity_preservation(
        self,
        uav: Any,
        task: Any,
        snapshot: Any,
        net: Any,
    ) -> tuple[bool, Any]:
        """Evaluate if moving the candidate to the task destination preserves GCS connectivity."""
        u_id = str(getattr(uav, "id", ""))
        task_pos = getattr(task, "position_xy", getattr(task, "position", None))
        if task_pos is None or not hasattr(snapshot, "uavs") or u_id not in snapshot.uavs:
            return True, net

        analyzer = self._get_comm_analyzer(net)
        dest_uavs = dict(snapshot.uavs)
        curr_uav_state = dest_uavs[u_id]
        dest_uavs[u_id] = dataclasses.replace(curr_uav_state, position_xy=tuple(task_pos))
        dest_snap = dataclasses.replace(snapshot, uavs=dest_uavs)

        dest_net = analyzer.analyze(dest_snap)

        before_connected = set(getattr(net, "connected_uav_ids", ()))
        after_connected = set(getattr(dest_net, "connected_uav_ids", ()))

        # CASE A: Candidate has no route to GCS at destination
        if u_id not in after_connected:
            return False, dest_net

        # CASE B: Candidate move disconnects previously-connected peers
        protected_peers = before_connected - {u_id}
        lost_peers = protected_peers - after_connected

        if lost_peers:
            return False, dest_net

        return True, dest_net

    def compute_utility(
        self,
        uav: Any,
        task: Any,
        network_analysis: Any = None,
        snapshot: Any = None,
    ) -> UtilityScore:
        """Compute communication-aware multi-factor utility score.

        A1Utility(i, j) = BaseMissionUtility(i, j) * CommunicationFactor(i)
        When destination_aware is enabled and snapshot is available, evaluates
        communication factor at candidate task destination.
        Preserves UtilityScore term consistency.
        """
        base_score = super().compute_utility(uav, task)

        net = network_analysis if network_analysis is not None else self._current_network_analysis
        if net is None:
            return base_score

        snap = snapshot if snapshot is not None else self._current_snapshot
        dest_aware = getattr(self.config, "destination_aware", True)

        if dest_aware and snap is not None and hasattr(snap, "uavs") and hasattr(snap, "gcs_position"):
            feasible, dest_net = self._evaluate_swarm_connectivity_preservation(uav, task, snap, net)
            if not feasible:
                return dataclasses.replace(base_score, total=float("-inf"))
            comm_factor = self.compute_communication_factor(uav, dest_net)
        else:
            comm_factor = self.compute_communication_factor(uav, net)

        if abs(comm_factor - 1.0) < 1e-9:
            return base_score

        base_total = base_score.total
        if base_total >= 0.0:
            final_total = base_total * comm_factor
        else:
            # Demote negative utility appropriately (lower comm_factor -> more negative score)
            final_total = base_total * (2.0 - comm_factor)

        # Record network term while keeping total mathematically identical
        comm_delta = final_total - base_total
        if comm_delta >= 0.0:
            conn_term = comm_delta
            net_risk_term = 0.0
        else:
            conn_term = 0.0
            net_risk_term = -comm_delta

        return UtilityScore(
            total=final_total,
            priority_term=base_score.priority_term,
            travel_cost_term=base_score.travel_cost_term,
            energy_risk_term=base_score.energy_risk_term,
            connectivity_term=conn_term,
            network_risk_term=net_risk_term,
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
        snapshot: Any = None,
    ) -> AllocationResult:
        """Allocate a hypothetical batch without leaking per-call scoring context."""
        previous_snapshot = self._current_snapshot
        previous_network = self._current_network_analysis
        try:
            return self._allocate_batch(
                snapshot_or_uavs, tasks, uavs=uavs,
                simulation_time=simulation_time, snapshot_revision=snapshot_revision,
                network_analysis=network_analysis, snapshot=snapshot,
            )
        finally:
            self._current_snapshot = previous_snapshot
            self._current_network_analysis = previous_network

    def _allocate_batch(
        self,
        snapshot_or_uavs: Any = None,
        tasks: Sequence[Any] | None = None,
        *,
        uavs: Sequence[Any] | None = None,
        simulation_time: float | None = None,
        snapshot_revision: int = 0,
        network_analysis: Any = None,
        snapshot: Any = None,
    ) -> AllocationResult:
        """Deterministically allocate pending tasks using A1 connectivity-aware scoring.
        
        Evaluates batch candidates sequentially, mutating a hypothetical working snapshot
        to prevent simultaneous assignments from breaking the communication graph.
        """
        from typing import Sequence, Any, Mapping
        from ares_swarm.autonomy.task_allocator import AllocationResult, TaskAssignment
        import dataclasses
        
        # Parse inputs
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

        # Check feasibility
        feasible_tasks: list[Any] = []
        infeasible_tasks: dict[str, str] = {}
        for t in raw_tasks:
            t_id = str(getattr(t, "id", ""))
            ok, reason = super().is_task_feasible(t, sim_time)
            if ok:
                feasible_tasks.append(t)
            else:
                infeasible_tasks[t_id] = reason

        feasible_uavs: list[Any] = []
        infeasible_uavs: dict[str, str] = {}
        for u in raw_uavs:
            u_id = str(getattr(u, "id", ""))
            ok, reason = super().is_uav_feasible(u, sim_time)
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

        available_uavs: dict[str, Any] = {str(u.id): u for u in sorted(feasible_uavs, key=lambda u: str(u.id))}
        assignments: list[TaskAssignment] = []
        unassigned_tasks: list[str] = []

        # Setup working state for accumulated batch evaluation
        working_snapshot = snapshot if snapshot is not None else (snapshot_or_uavs if hasattr(snapshot_or_uavs, "uavs") else None)
        working_net = network_analysis
        analyzer = self._get_comm_analyzer(working_net) if working_net else None

        # Sequential evaluation
        for task in feasible_tasks:
            t_id = str(getattr(task, "id", ""))
            if not available_uavs:
                unassigned_tasks.append(t_id)
                continue

            best_uav_id = None
            best_score = None
            best_utility = float("-inf")
            
            self._current_snapshot = working_snapshot
            self._current_network_analysis = working_net

            for uav in available_uavs.values():
                score = self.compute_utility(uav, task, network_analysis=working_net, snapshot=working_snapshot)
                if score.total > best_utility:
                    best_utility = score.total
                    best_uav_id = str(uav.id)
                    best_score = score
                elif score.total == best_utility and best_uav_id is not None:
                    if str(uav.id) < best_uav_id:
                        best_uav_id = str(uav.id)
                        best_score = score

            self._current_snapshot = None
            self._current_network_analysis = None

            # A0 has no zero-utility rejection threshold. Preserve that contract;
            # topology-infeasible (-inf) candidates never acquire best_uav_id.
            if best_uav_id is not None and best_utility >= float(getattr(self.config, "min_utility_threshold", float("-inf"))):
                assignments.append(
                    TaskAssignment(
                        uav_id=best_uav_id,
                        task_id=t_id,
                        score=best_score.total,
                        score_breakdown=best_score,
                    )
                )
                del available_uavs[best_uav_id]
                
                # Apply hypothetical move
                if working_snapshot is not None and analyzer is not None:
                    task_pos = getattr(task, "position_xy", getattr(task, "position", None))
                    if task_pos is not None and hasattr(working_snapshot, "uavs") and best_uav_id in working_snapshot.uavs:
                        dest_uavs = dict(working_snapshot.uavs)
                        curr_uav_state = dest_uavs[best_uav_id]
                        dest_uavs[best_uav_id] = dataclasses.replace(curr_uav_state, position_xy=tuple(task_pos))
                        working_snapshot = dataclasses.replace(working_snapshot, uavs=dest_uavs)
                        working_net = analyzer.analyze(working_snapshot)
            else:
                unassigned_tasks.append(t_id)

        return AllocationResult(
            assignments=tuple(assignments),
            unassigned_tasks=tuple(unassigned_tasks),
            unassigned_uavs=tuple(available_uavs.keys()),
            infeasible_tasks=infeasible_tasks,
            infeasible_uavs=infeasible_uavs,
            snapshot_revision=revision,
        )

    def plan(
        self,
        snapshot: Any,
        network_analysis: Any = None,
    ) -> list[TaskActionProposal]:
        """Autonomous planner interface implementation conforming to AutonomyPlanner protocol."""
        result = self.allocate(snapshot, network_analysis=network_analysis, snapshot=snapshot)
        return result.to_action_proposals()


# Compatibility aliases
A1CommunicationAwareAllocator = A1TaskAllocator
