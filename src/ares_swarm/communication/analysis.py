"""Compose the M0 pipeline behind the existing CommunicationAnalyzer protocol."""
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping, Sequence

from .config import CommunicationConfig
from .models import NetworkState, LinkState
from ..core.models import StateSnapshot
from .validation import Validated, require
from ..interfaces.communication import NetworkAnalysis
from .channel import ChannelModel, LinkCondition
from .graph import build_network_graph, normalize_conditions
from .connectivity import analyze_connectivity
from .routing import shortest_hop_routes, reliability_aware_routes, RoutingWeights
from .scenario import CommunicationCondition, resolve_conditions


@dataclass(frozen=True, slots=True)
class BaselineCommunicationAnalyzer(Validated):
    """Stateless analyzer with copied immutable configuration/condition inputs."""
    config: CommunicationConfig
    conditions: Mapping[tuple[str, str], LinkCondition] = field(default_factory=dict)
    scenario_conditions: Sequence[CommunicationCondition] = ()
    routing_weights: RoutingWeights = field(default_factory=RoutingWeights)

    def __post_init__(self) -> None:
        object.__setattr__(self, "conditions", MappingProxyType(normalize_conditions(self.conditions)))
        object.__setattr__(self, "scenario_conditions", tuple(self.scenario_conditions))

    def analyze(self, snapshot: StateSnapshot) -> NetworkAnalysis:
        """Rebuild topology and report it without modifying any authoritative state."""

        # Merge scenario-driven conditions for the current simulation time
        resolved = resolve_conditions(snapshot.simulation_time, self.scenario_conditions)
        active_conditions = dict(self.conditions)

        for pair, cond in resolved.items():
            active_conditions[pair] = cond

        active_conditions = normalize_conditions(active_conditions)

        graph = build_network_graph(snapshot, ChannelModel(self.config), conditions=active_conditions)
        gcs_id = "gcs"
        connectivity = analyze_connectivity(graph, gcs_id)

        # M0 shortest hop routes
        routes = shortest_hop_routes(graph, gcs_id)

        # M2 reliability-aware weighted routes (optional)
        reliable_routes = reliability_aware_routes(graph, gcs_id, self.routing_weights)

        links = tuple(sorted((data["link"] for _,_,data in graph.edges(data=True)),
                             key=lambda link: (link.source_id, link.target_id)))

        reliable_hop_counts = {
            uid: len(r) - 1 if r is not None else None
            for uid, r in reliable_routes.items()
        }

        route_pdrs = {
            uid: route_pdr(links, r)
            for uid, r in routes.items()
        }

        return NetworkAnalysis(
            snapshot_revision=snapshot.state_version,
            network=NetworkState(links=links),
            connected_uav_ids=connectivity.connected_uav_ids,
            gcs_id=gcs_id,
            simulation_time=snapshot.simulation_time,
            disconnected_uav_ids=connectivity.disconnected_uav_ids,
            components=connectivity.components,
            routes_to_gcs=routes,
            hop_counts=connectivity.hop_counts,
            route_pdr_to_gcs=route_pdrs,
            reliable_routes_to_gcs=reliable_routes,
            reliable_hop_counts=reliable_hop_counts,
            articulation_points=connectivity.articulation_points,
            network_health=connectivity.network_health,
        )

def route_pdr(edges: tuple[LinkState, ...], route: tuple[str, ...] | None) -> float | None:
    """Calculate the end-to-end PDR for a specific route based on derived edge metrics.

    - No route (None) -> 0.0
    - Missing/unusable link in route -> None
    """
    if route is None:
        return 0.0

    pdr = 1.0
    for a, b in zip(route, route[1:]):
        pair = tuple(sorted((a, b)))
        link = next((l for l in edges if l.source_id == pair[0] and l.target_id == pair[1]), None)
        if link is None:
            return None
        pdr *= link.estimated_pdr

    return pdr


def route_latency_ms(analysis: NetworkAnalysis, route: tuple[str, ...] | None) -> float | None:
    """Calculate the end-to-end latency sum for a specific route based on derived edge metrics.

    - Valid multi-hop route -> summed latency
    - Single-node route (e.g., GCS only) -> 0.0
    - Route is None or empty -> None
    - Missing/unusable link in route -> None
    """
    if not route:
        return None
    if len(route) == 1:
        return 0.0

    total_ms = 0.0
    for a, b in zip(route, route[1:]):
        pair = tuple(sorted((a, b)))
        link = next((l for l in analysis.edge_metrics if l.source_id == pair[0] and l.target_id == pair[1]), None)
        if link is None:
            return None
        total_ms += link.latency_ms

    return total_ms
