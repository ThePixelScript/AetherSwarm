"""M0 shortest-hop routing; no reliability or energy weighting."""
from types import MappingProxyType
from typing import Mapping
import networkx as nx

from .graph import validate_graph


def shortest_hop_routes(graph: nx.Graph, gcs_id: str) -> Mapping[str, tuple[str, ...] | None]:
    """Return the lexicographically smallest minimum-hop UAV-to-GCS paths.

    One BFS gives distances to GCS. For each reachable node, choose the smallest
    neighbor with distance exactly one less. Distances strictly decrease, so paths
    are loop-free; this choice minimizes the path in the requested source direction.
    """
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
