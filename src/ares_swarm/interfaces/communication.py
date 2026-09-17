"""Shared communication contract: immutable derived evidence, never commands."""
from dataclasses import dataclass, field
from typing import Mapping, Protocol

from ..communication.models import NetworkState, LinkState
from ..core.models import StateSnapshot
from ..communication.validation import Validated, nonnegative, freeze, require


@dataclass(frozen=True, slots=True)
class NetworkAnalysis(Validated):
    """Phase-1 constructor preserved; M0 analyzers always populate the full view.

    gcs_id=None denotes a legacy partial result, not a complete topology analysis.
    Failed/inactive UAVs are absent from both reachable and disconnected sets.
    """
    snapshot_revision: int
    network: NetworkState
    connected_uav_ids: tuple[str, ...] = ()
    gcs_id: str | None = None
    simulation_time: float = 0.0
    disconnected_uav_ids: tuple[str, ...] = ()
    components: tuple[tuple[str, ...], ...] = ()
    routes_to_gcs: Mapping[str, tuple[str, ...] | None] = field(default_factory=dict)
    hop_counts: Mapping[str, int | None] = field(default_factory=dict)
    reliable_routes_to_gcs: Mapping[str, tuple[str, ...] | None] = field(default_factory=dict)
    reliable_hop_counts: Mapping[str, int | None] = field(default_factory=dict)
    articulation_points: tuple[str, ...] = ()
    network_health: Mapping[str, int | float | None] = field(default_factory=dict)

    def __post_init__(self) -> None:
        super(NetworkAnalysis, self).__post_init__()
        nonnegative(self.snapshot_revision, self.simulation_time)
        for name in ("routes_to_gcs", "hop_counts", "reliable_routes_to_gcs", "reliable_hop_counts", "network_health"):
            object.__setattr__(self, name, freeze(dict(sorted(getattr(self, name).items()))))
        if self.gcs_id is None:
            return  # Keep the existing Phase-1 partial constructor/JSON readable.
        require(bool(self.gcs_id.strip()), "GCS ID required")
        for ids in (self.connected_uav_ids, self.disconnected_uav_ids, self.articulation_points):
            require(ids == tuple(sorted(set(ids))), "analysis IDs must be sorted and unique")
        connected, disconnected = set(self.connected_uav_ids), set(self.disconnected_uav_ids)
        require(not connected & disconnected, "reachable/disconnected overlap")
        active = connected | disconnected
        require(self.gcs_id not in active, "GCS is not a UAV")
        require(self.components == tuple(sorted(self.components)), "components must be sorted")
        flattened = tuple(uid for component in self.components for uid in component)
        nodes = set(flattened)
        component_index = {uid: index for index, component in enumerate(self.components)
                           for uid in component}
        for component in self.components:
            require(bool(component) and component == tuple(sorted(set(component))),
                    "component IDs must be nonempty, sorted and unique")
        require(len(flattened) == len(nodes)
                and nodes == active | {self.gcs_id}, "invalid component coverage")
        gcs_component = next(c for c in self.components if self.gcs_id in c)
        require(set(gcs_component)-{self.gcs_id} == connected, "component/reachability mismatch")
        require(set(self.routes_to_gcs) == active and set(self.hop_counts) == active,
                "route/hop keys must cover all active UAVs")
                
        has_reliable = len(self.reliable_routes_to_gcs) > 0
        if has_reliable:
            require(set(self.reliable_routes_to_gcs) == active and set(self.reliable_hop_counts) == active,
                    "reliable route/hop keys must cover all active UAVs")
                    
        links = self.network.links
        edge_keys = tuple((link.source_id, link.target_id) for link in links)
        require(edge_keys == tuple(sorted(set(edge_keys))), "edge metrics must be sorted and unique")
        edges = set(edge_keys)
        for link in links:
            require(link.source_id < link.target_id and link.active and link.estimated_pdr > 0,
                    "M0 edge metrics must represent canonical usable links")
            require(link.source_id in nodes and link.target_id in nodes,
                    "edge references unknown node")
            require(component_index[link.source_id] == component_index[link.target_id],
                    "edge crosses declared components")
                    
        for uid in sorted(active):
            route, hops = self.routes_to_gcs[uid], self.hop_counts[uid]
            if uid in disconnected:
                require(route is None and hops is None, "disconnected UAV cannot have route/hops")
                if has_reliable:
                    require(self.reliable_routes_to_gcs[uid] is None and self.reliable_hop_counts[uid] is None, 
                            "disconnected UAV cannot have reliable route/hops")
            else:
                require(route is not None and len(route) >= 2 and route[0] == uid
                        and route[-1] == self.gcs_id, "routes must run UAV to GCS")
                require(len(set(route)) == len(route) and set(route) <= active | {self.gcs_id},
                        "invalid route nodes")
                require(hops == len(route)-1, "route/hop mismatch")
                require(all(tuple(sorted((a,b))) in edges for a,b in zip(route,route[1:])),
                        "route references missing edge")
                        
                if has_reliable:
                    r_route, r_hops = self.reliable_routes_to_gcs[uid], self.reliable_hop_counts[uid]
                    require(r_route is not None and len(r_route) >= 2 and r_route[0] == uid
                            and r_route[-1] == self.gcs_id, "reliable routes must run UAV to GCS")
                    require(len(set(r_route)) == len(r_route) and set(r_route) <= active | {self.gcs_id},
                            "invalid reliable route nodes")
                    require(r_hops == len(r_route)-1, "reliable route/hop mismatch")
                    require(all(tuple(sorted((a,b))) in edges for a,b in zip(r_route,r_route[1:])),
                            "reliable route references missing edge")
                            
        require(set(self.articulation_points) <= nodes, "unknown articulation node")

    @property
    def reachable_uav_ids(self) -> tuple[str, ...]:
        """Consumer-friendly alias of the original connected_uav_ids field."""
        return self.connected_uav_ids

    @property
    def edge_metrics(self) -> tuple[LinkState, ...]:
        """Single canonical source of edge metrics; no duplicated mutable graph."""
        return self.network.links


class CommunicationAnalyzer(Protocol):
    def analyze(self, snapshot: StateSnapshot) -> NetworkAnalysis:
        """Return derived observations for exactly this snapshot revision."""
        ...
