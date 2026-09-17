"""Deterministic communication-aware task allocator (A1) for ARES-Swarm autonomy.

Stage-1 communication-aware task assignment incorporating real NetworkAnalysis signals
(reachability, hop count, end-to-end route PDR) on top of the deterministic A0 baseline.

A1 v1 ARCHITECTURE & CONTRACTS:
- Inherits all operational and task feasibility gates from A0TaskAllocator without code duplication.
- Preserves A0 task ordering (descending priority, emergency status, ascending ID).
- Modifies candidate utility scoring using a bounded, deterministic communication factor:
    A1Utility(i, j) = BaseMissionUtility(i, j) * CommunicationFactor(i)
- Demotes disconnected UAVs to a configurable floor (min_comm_factor) rather than rejecting them,
  preventing mission deadlock when the swarm is partitioned.
- Explicit A1 v1 limitation: evaluates communication based strictly on current UAV position and
  current route to GCS. Does not simulate future connectivity at the task destination.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

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


class A1TaskAllocator(A0TaskAllocator):
    """Deterministic communication-aware task allocator (A1).

    Inherits all operational and task feasibility gates from A0TaskAllocator.
    Applies bounded communication modifiers to candidate utility scoring based on
    authoritative NetworkAnalysis signals.
    """

    def __init__(self, config: A1AllocatorConfig | TaskAllocatorConfig | None = None) -> None:
        super().__init__(config=config or A1AllocatorConfig())
        self._current_network_analysis: Any = None

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

    def compute_utility(
        self,
        uav: Any,
        task: Any,
        network_analysis: Any = None,
    ) -> UtilityScore:
        """Compute communication-aware multi-factor utility score.

        A1Utility(i, j) = BaseMissionUtility(i, j) * CommunicationFactor(i)
        Preserves UtilityScore term consistency.
        """
        base_score = super().compute_utility(uav, task)

        net = network_analysis if network_analysis is not None else self._current_network_analysis
        if net is None:
            return base_score

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
    ) -> AllocationResult:
        """Deterministically allocate pending tasks using A0 matching with A1 scoring."""
        prev_net = self._current_network_analysis
        self._current_network_analysis = network_analysis
        try:
            return super().allocate(
                snapshot_or_uavs,
                tasks,
                uavs=uavs,
                simulation_time=simulation_time,
                snapshot_revision=snapshot_revision,
            )
        finally:
            self._current_network_analysis = prev_net

    def plan(
        self,
        snapshot: Any,
        network_analysis: Any = None,
    ) -> list[TaskActionProposal]:
        """Autonomous planner interface implementation conforming to AutonomyPlanner protocol."""
        result = self.allocate(snapshot, network_analysis=network_analysis)
        return result.to_action_proposals()


# Compatibility aliases
A1CommunicationAwareAllocator = A1TaskAllocator
