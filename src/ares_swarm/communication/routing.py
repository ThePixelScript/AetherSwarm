"""M0 shortest-hop routing and M2 reliability-aware weighted routing."""
from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping
import networkx as nx

from .graph import validate_graph


def shortest_hop_routes(graph: nx.Graph, gcs_id: str) -> Mapping[str, tuple[str, ...] | None]:
    """Return the lexicographically smallest minimum-hop UAV-to-GCS paths."""
    validate_graph(graph, gcs_id)
    distances = nx.single_source_shortest_path_length(graph, gcs_id)
    next_hop = {
        uid: min(neighbor for neighbor in graph[uid]
                 if distances.get(neighbor) == distances[uid]-1)
        for uid in sorted(distances) if uid != gcs_id
    }
    routes = {}
    for uid in sorted(graph):
        if uid == gcs_id:
            continue
        if uid not in distances:
            routes[uid] = None
            continue
        path = [uid]
        while path[-1] != gcs_id:
            path.append(next_hop[path[-1]])
        routes[uid] = tuple(path)
    return MappingProxyType(routes)


@dataclass(frozen=True)
class RoutingWeights:
    """Weights for M2 reliability-aware routing. Cost must be minimized."""
    w_etx: float = 0.50
    w_latency: float = 0.20
    w_energy: float = 0.15
    w_instability: float = 0.15


def normalize_link_cost(link_data: dict, weights: RoutingWeights) -> float:
    """Compute deterministic normalized link cost [0, 1+].

    - ETX_norm: 1.0 - estimated_pdr (probability of failure).
    - latency_norm: min(1.0, latency_ms / 50.0).
    - energy_risk_norm: 0.0 (Energy risk not available on M0 static links).
    - instability_norm: 1.0 - link_quality (Static proxy for dynamic instability).
    """
    pdr = link_data.get("estimated_pdr", 0.0)
    latency = link_data.get("latency_ms", 0.0)
    quality = link_data.get("link_quality", 1.0)

    etx_norm = 1.0 - pdr
    latency_norm = min(1.0, latency / 50.0)
    energy_norm = 0.0
    instability_norm = 1.0 - quality

    return (
        weights.w_etx * etx_norm
        + weights.w_latency * latency_norm
        + weights.w_energy * energy_norm
        + weights.w_instability * instability_norm
    )


def reliability_aware_routes(
    graph: nx.Graph, gcs_id: str, weights: RoutingWeights | None = None
) -> Mapping[str, tuple[str, ...] | None]:
    """Compute optimal reliable routes using configurable weighted metrics.

    Uses Dijkstra's algorithm to find minimum-cost paths to GCS.
    Tie-breaking is strictly deterministic (lexicographical node order).
    """
    validate_graph(graph, gcs_id)
    if weights is None:
        weights = RoutingWeights()

    # Create a directed graph to incorporate deterministic tie-breaking.
    # We want paths TO the GCS. We compute single-source shortest paths FROM GCS
    # on the reversed edges (which are symmetric in cost but we add a tie-breaker).
    # Since networkx Dijkstra tie-breaking isn't strictly guaranteed by node names
    # internally without a custom queue, we will implement a deterministic Dijkstra.

    # We compute shortest paths FROM gcs_id to all other nodes.
    import heapq

    distances = {gcs_id: 0.0}
    # Predecessor mapping: uid -> best_next_hop (towards GCS)
    next_hop = {}
    
    # Priority queue: (cost, node_id)
    pq = [(0.0, gcs_id)]
    
    while pq:
        current_cost, u = heapq.heappop(pq)
        
        if current_cost > distances.get(u, float('inf')):
            continue
            
        # To guarantee deterministic tie-breaking, sort neighbors.
        for v in sorted(graph.neighbors(u)):
            edge_data = graph.get_edge_data(u, v)
            link_cost = normalize_link_cost(edge_data, weights)
            new_cost = current_cost + link_cost
            
            # Since we search FROM GCS TO UAV, v is a UAV (or intermediate)
            # and u is its next_hop towards GCS.
            # We want to minimize new_cost.
            # If new_cost == existing cost, we tie-break by choosing the lexicographically
            # smaller next_hop `u`.
            old_cost = distances.get(v, float('inf'))
            if new_cost < old_cost - 1e-9:
                distances[v] = new_cost
                next_hop[v] = u
                heapq.heappush(pq, (new_cost, v))
            elif abs(new_cost - old_cost) <= 1e-9:
                # Tie-breaker: choose smaller next_hop (u)
                if u < next_hop[v]:
                    next_hop[v] = u
                    # No need to push to PQ again, cost is identical.
                    
    routes = {}
    for uid in sorted(graph):
        if uid == gcs_id:
            continue
        if uid not in distances:
            routes[uid] = None
            continue
            
        path = [uid]
        while path[-1] != gcs_id:
            path.append(next_hop[path[-1]])
        routes[uid] = tuple(path)
        
    return MappingProxyType(routes)
