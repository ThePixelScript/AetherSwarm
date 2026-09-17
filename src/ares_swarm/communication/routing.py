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
    """Weights and normalization bounds for M2 reliability-aware routing. Cost must be minimized."""
    w_etx: float = 0.50
    w_latency: float = 0.20
    w_energy: float = 0.15
    w_instability: float = 0.15
    
    # Normalization bounds
    etx_max: float = 10.0
    latency_max_ms: float = 50.0


def normalize_link_cost(link_data: dict, weights: RoutingWeights) -> float:
    """Compute deterministic normalized link cost [0, 1].

    - ETX_norm: clip((ETX - 1) / (ETX_MAX - 1), 0, 1) using actual link.etx.
    - latency_norm: min(1.0, latency_ms / LATENCY_MAX_MS).
    - energy_risk_norm: 0.0 (Energy term reserved for later integration; no fabricated energy signal).
    - instability_proxy_norm: 1.0 - link_quality (This is an instantaneous deterministic proxy based on current link quality, not a temporal instability estimator).
    """
    etx = link_data.get("etx", 1.0)
    latency = link_data.get("latency_ms", 0.0)
    quality = link_data.get("link_quality", 1.0)

    # 1. ETX normalization
    # ETX >= 1.0. ETX=1 means perfect delivery.
    if weights.etx_max <= 1.0:
        etx_norm = 1.0 if etx > 1.0 else 0.0
    else:
        raw_etx_norm = (etx - 1.0) / (weights.etx_max - 1.0)
        etx_norm = max(0.0, min(1.0, raw_etx_norm))

    # 2. Latency normalization
    if weights.latency_max_ms <= 0.0:
        latency_norm = 1.0 if latency > 0.0 else 0.0
    else:
        latency_norm = min(1.0, latency / weights.latency_max_ms)

    # 3. Energy normalization
    # Energy term reserved for later integration; no fabricated energy signal.
    energy_norm = 0.0

    # 4. Instability proxy normalization
    # This is an instantaneous deterministic proxy based on current link quality, not a temporal instability estimator.
    instability_proxy_norm = 1.0 - quality

    return (
        weights.w_etx * etx_norm
        + weights.w_latency * latency_norm
        + weights.w_energy * energy_norm
        + weights.w_instability * instability_proxy_norm
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
            old_cost = distances.get(v, float('inf'))
            if new_cost < old_cost - 1e-9:
                distances[v] = new_cost
                next_hop[v] = u
                heapq.heappush(pq, (new_cost, v))
            elif abs(new_cost - old_cost) <= 1e-9:
                # Tie-breaker: choose smaller next_hop (u)
                if u < next_hop[v]:
                    next_hop[v] = u
                    
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
