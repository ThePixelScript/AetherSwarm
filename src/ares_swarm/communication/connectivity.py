"""Reachability and structural topology evidence, without command generation."""
from dataclasses import dataclass
from typing import Mapping
import networkx as nx

from ..core.validation import Validated, freeze
from .graph import validate_graph


@dataclass(frozen=True, slots=True)
class ConnectivityResult(Validated):
    """Immutable intermediate analysis; active UAVs only, excluding GCS from IDs."""
    connected_uav_ids: tuple[str, ...]
    disconnected_uav_ids: tuple[str, ...]
    components: tuple[tuple[str, ...], ...]
    hop_counts: Mapping[str, int | None]
    articulation_points: tuple[str, ...]
    network_health: Mapping[str, int | float | None]

    def __post_init__(self) -> None:
        super(ConnectivityResult, self).__post_init__()
        object.__setattr__(self, "hop_counts", freeze(self.hop_counts))
        object.__setattr__(self, "network_health", freeze(self.network_health))


def analyze_connectivity(graph: nx.Graph, gcs_id: str) -> ConnectivityResult:
    """Analyze a simple undirected GCS/UAV graph with explicitly sorted outputs."""
    validate_graph(graph, gcs_id)
    distances = nx.single_source_shortest_path_length(graph, gcs_id)
    uav_ids = tuple(uid for uid in sorted(graph) if uid != gcs_id)
    connected = tuple(uid for uid in uav_ids if uid in distances)
    disconnected = tuple(uid for uid in uav_ids if uid not in distances)
    components = tuple(sorted(tuple(sorted(c)) for c in nx.connected_components(graph)))
    points = tuple(sorted(nx.articulation_points(graph)))
    hops = {uid: distances.get(uid) for uid in uav_ids}
    connected_hops = [distances[uid] for uid in connected]
    health = {
        "active_uav_count": len(uav_ids),
        "connected_uav_count": len(connected),
        "disconnected_uav_count": len(disconnected),
        "connectivity_ratio": len(connected) / len(uav_ids) if uav_ids else None,
        "component_count": len(components),
        "disconnected_component_count": sum(gcs_id not in c for c in components),
        "largest_component_size": max(map(len, components)),
        "average_hop_count": sum(connected_hops)/len(connected_hops) if connected_hops else None,
        "maximum_hop_count": max(connected_hops) if connected_hops else None,
        "articulation_point_count": len(points),
    }
    return ConnectivityResult(connected, disconnected, components, hops, points, health)
