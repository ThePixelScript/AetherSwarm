"""Compose the M0 pipeline behind the existing CommunicationAnalyzer protocol."""
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping

from .config import CommunicationConfig
from .models import NetworkState
from ..core.models import StateSnapshot
from .validation import Validated
from ..interfaces.communication import NetworkAnalysis
from .channel import ChannelModel, LinkCondition
from .graph import build_network_graph, normalize_conditions
from .connectivity import analyze_connectivity
from .routing import shortest_hop_routes, reliability_aware_routes, RoutingWeights


@dataclass(frozen=True, slots=True)
class BaselineCommunicationAnalyzer(Validated):
    """Stateless analyzer with copied immutable configuration/condition inputs."""
    config: CommunicationConfig
    conditions: Mapping[tuple[str, str], LinkCondition] = field(default_factory=dict)
    routing_weights: RoutingWeights = field(default_factory=RoutingWeights)

    def __post_init__(self) -> None:
        object.__setattr__(self, "conditions", MappingProxyType(normalize_conditions(self.conditions)))

    def analyze(self, snapshot: StateSnapshot) -> NetworkAnalysis:
        """Rebuild topology and report it without modifying any authoritative state."""
        graph = build_network_graph(snapshot, ChannelModel(self.config), conditions=self.conditions)
        gcs_id = "gcs"
        connectivity = analyze_connectivity(graph, gcs_id)
        
        # M0 shortest hop routes
        routes = shortest_hop_routes(graph, gcs_id)
        
        # M2 reliability-aware weighted routes
        reliable_routes = reliability_aware_routes(graph, gcs_id, self.routing_weights)
        
        links = tuple(sorted((data["link"] for _,_,data in graph.edges(data=True)),
                             key=lambda link: (link.source_id, link.target_id)))
                             
        reliable_hop_counts = {
            uid: len(r) - 1 if r is not None else None
            for uid, r in reliable_routes.items()
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
            reliable_routes_to_gcs=reliable_routes,
            reliable_hop_counts=reliable_hop_counts,
            articulation_points=connectivity.articulation_points,
            network_health=connectivity.network_health,
        )
