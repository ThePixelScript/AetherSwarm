"""Compose the M0 pipeline behind the existing CommunicationAnalyzer protocol."""
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping

from ..core.config import CommunicationConfig
from ..core.models import NetworkState
from ..core.snapshot import StateSnapshot
from ..core.validation import Validated
from ..interfaces.communication import NetworkAnalysis
from .channel import ChannelModel, LinkCondition
from .graph import build_network_graph, normalize_conditions
from .connectivity import analyze_connectivity
from .routing import shortest_hop_routes


@dataclass(frozen=True, slots=True)
class BaselineCommunicationAnalyzer(Validated):
    """Stateless analyzer with copied immutable configuration/condition inputs."""
    config: CommunicationConfig
    conditions: Mapping[tuple[str, str], LinkCondition] = field(default_factory=dict)

    def __post_init__(self) -> None:
        super(BaselineCommunicationAnalyzer, self).__post_init__()
        object.__setattr__(self, "conditions", MappingProxyType(normalize_conditions(self.conditions)))

    def analyze(self, snapshot: StateSnapshot) -> NetworkAnalysis:
        """Rebuild topology and report it without modifying any authoritative state."""
        graph = build_network_graph(snapshot, ChannelModel(self.config), conditions=self.conditions)
        gcs_id = snapshot.state.gcs.id
        connectivity = analyze_connectivity(graph, gcs_id)
        routes = shortest_hop_routes(graph, gcs_id)
        links = tuple(sorted((data["link"] for _,_,data in graph.edges(data=True)),
                             key=lambda link: (link.source_id, link.target_id)))
        return NetworkAnalysis(
            snapshot_revision=snapshot.revision,
            network=NetworkState(links=links, recovery_state=snapshot.state.network.recovery_state),
            connected_uav_ids=connectivity.connected_uav_ids,
            gcs_id=gcs_id,
            simulation_time=snapshot.state.simulation_time,
            disconnected_uav_ids=connectivity.disconnected_uav_ids,
            components=connectivity.components,
            routes_to_gcs=routes,
            hop_counts=connectivity.hop_counts,
            articulation_points=connectivity.articulation_points,
            network_health=connectivity.network_health,
        )
