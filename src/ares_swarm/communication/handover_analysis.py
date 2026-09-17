"""M3 pure communication analysis helpers for relay dependency and handover readiness."""
from dataclasses import dataclass, replace
from types import MappingProxyType
from typing import Tuple, Mapping
import networkx as nx

from ..core.models import StateSnapshot
from ..interfaces.communication import NetworkAnalysis, CommunicationAnalyzer
from .validation import Validated, require
from .connectivity import analyze_connectivity


@dataclass(frozen=True)
class RelayDependencyAnalysis:
    relay_id: str
    is_articulation_point: bool
    dependent_uav_ids: Tuple[str, ...]
    affected_route_uav_ids: Tuple[str, ...]
    disconnected_if_removed: Tuple[str, ...]
    protected_uav_ids: Tuple[str, ...]


@dataclass(frozen=True)
class ReplacementCandidateEvidence:
    candidate_id: str
    target_position: Tuple[float, float]
    protected_uav_ids: Tuple[str, ...]
    reachable_protected_uav_ids: Tuple[str, ...]
    unreachable_protected_uav_ids: Tuple[str, ...]
    all_protected_reachable: bool
    shortest_routes: Mapping[str, Tuple[str, ...] | None]
    reliable_routes: Mapping[str, Tuple[str, ...] | None]
    hop_counts: Mapping[str, int | None]
    feasible: bool


@dataclass(frozen=True)
class HandoverVerification:
    protected_uav_ids: Tuple[str, ...]
    reachable_uav_ids: Tuple[str, ...]
    unreachable_uav_ids: Tuple[str, ...]
    all_protected_reachable: bool
    shortest_routes: Mapping[str, Tuple[str, ...] | None]
    reliable_routes: Mapping[str, Tuple[str, ...] | None]


def analyze_relay_dependency(
    graph: nx.Graph,
    relay_id: str,
    network_analysis: NetworkAnalysis,
    gcs_id: str = "gcs"
) -> RelayDependencyAnalysis:
    """Evaluate topological and routing dependency on a specific relay UAV.
    
    Does NOT mutate the input graph.
    """
    require(relay_id in graph, "Relay ID not in graph")
    
    # 1. Affected Routes
    # Which UAVs have shortest-hop or reliable routes that traverse the relay?
    affected_route_uav_ids_set = set()
    for uid in network_analysis.connected_uav_ids:
        if uid == relay_id:
            continue
        route_sh = network_analysis.routes_to_gcs.get(uid)
        route_rel = network_analysis.reliable_routes_to_gcs.get(uid)
        
        if route_sh and relay_id in route_sh:
            affected_route_uav_ids_set.add(uid)
        elif route_rel and relay_id in route_rel:
            affected_route_uav_ids_set.add(uid)
            
    affected_route_uav_ids = tuple(sorted(affected_route_uav_ids_set))
    
    # 2. Dependency / Disconnection analysis
    # Simulate removal on a copied graph
    test_graph = graph.copy()
    test_graph.remove_node(relay_id)
    
    # Use existing analyze_connectivity to find new reachability
    new_connectivity = analyze_connectivity(test_graph, gcs_id)
    
    # Dependent UAVs: were connected, but are now disconnected
    originally_connected = set(network_analysis.connected_uav_ids) - {relay_id}
    newly_disconnected = originally_connected - set(new_connectivity.connected_uav_ids)
    dependent_uav_ids = tuple(sorted(newly_disconnected))
    
    is_articulation = relay_id in network_analysis.articulation_points
    
    # Protected UAVs: union of dependent and affected. We must protect any UAV
    # whose route is disturbed, but especially those that disconnect.
    protected_uav_ids = tuple(sorted(affected_route_uav_ids_set | newly_disconnected))
    
    return RelayDependencyAnalysis(
        relay_id=relay_id,
        is_articulation_point=is_articulation,
        dependent_uav_ids=dependent_uav_ids,
        affected_route_uav_ids=affected_route_uav_ids,
        disconnected_if_removed=dependent_uav_ids, # synonymous in this context
        protected_uav_ids=protected_uav_ids
    )


def evaluate_replacement_candidate(
    snapshot: StateSnapshot,
    analyzer: CommunicationAnalyzer,
    candidate_id: str,
    target_position: Tuple[float, float],
    protected_uav_ids: Tuple[str, ...]
) -> ReplacementCandidateEvidence:
    """Hypothetically move candidate_id to target_position and evaluate reachability.
    
    Does NOT mutate the snapshot. Uses replace to construct a hypothetical snapshot.
    """
    require(candidate_id in snapshot.uavs, "Candidate ID not in snapshot")
    
    # Construct hypothetical snapshot
    uav = snapshot.uavs[candidate_id]
    hypothetical_uav = replace(uav, position_xy=target_position)
    
    # We must use a dict to construct the new MappingProxyType
    new_uavs = dict(snapshot.uavs)
    new_uavs[candidate_id] = hypothetical_uav
    
    hypothetical_snapshot = replace(snapshot, uavs=MappingProxyType(new_uavs))
    
    # Run communication analyzer
    analysis = analyzer.analyze(hypothetical_snapshot)
    
    # Evaluate protected UAVs
    protected_set = set(protected_uav_ids)
    connected_set = set(analysis.connected_uav_ids)
    
    reachable_protected = protected_set & connected_set
    unreachable_protected = protected_set - connected_set
    
    feasible = len(unreachable_protected) == 0
    
    return ReplacementCandidateEvidence(
        candidate_id=candidate_id,
        target_position=target_position,
        protected_uav_ids=protected_uav_ids,
        reachable_protected_uav_ids=tuple(sorted(reachable_protected)),
        unreachable_protected_uav_ids=tuple(sorted(unreachable_protected)),
        all_protected_reachable=feasible,
        shortest_routes=analysis.routes_to_gcs,
        reliable_routes=analysis.reliable_routes_to_gcs,
        hop_counts=analysis.hop_counts,
        feasible=feasible
    )


def verify_handover_connectivity(
    network_analysis: NetworkAnalysis,
    protected_uav_ids: Tuple[str, ...]
) -> HandoverVerification:
    """Verify if actual post-movement NetworkAnalysis provides connectivity to protected UAVs."""
    
    protected_set = set(protected_uav_ids)
    connected_set = set(network_analysis.connected_uav_ids)
    
    reachable = protected_set & connected_set
    unreachable = protected_set - connected_set
    
    all_reachable = len(unreachable) == 0
    
    return HandoverVerification(
        protected_uav_ids=protected_uav_ids,
        reachable_uav_ids=tuple(sorted(reachable)),
        unreachable_uav_ids=tuple(sorted(unreachable)),
        all_protected_reachable=all_reachable,
        shortest_routes=network_analysis.routes_to_gcs,
        reliable_routes=network_analysis.reliable_routes_to_gcs
    )
