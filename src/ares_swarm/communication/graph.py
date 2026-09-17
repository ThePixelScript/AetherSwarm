"""Fresh undirected NetworkX topology derived exclusively from a snapshot."""
from dataclasses import fields
from itertools import combinations
from typing import Mapping
import networkx as nx

from ..core.snapshot import StateSnapshot
from ..core.validation import require
from .channel import ChannelModel, LinkCondition, node_available


def validate_graph(graph: nx.Graph, gcs_id: str) -> None:
    """Reject graph forms outside the M0 undirected simple string-ID contract."""
    require(isinstance(graph, nx.Graph) and not graph.is_directed()
            and not graph.is_multigraph(), "M0 requires a simple undirected graph")
    require(all(type(n) is str and bool(n.strip()) for n in graph), "invalid graph node ID")
    require(gcs_id in graph, "GCS missing from graph")
    require(nx.number_of_selfloops(graph) == 0, "self-loops are not supported")


def normalize_conditions(
    conditions: Mapping[tuple[str, str], LinkCondition] | None,
) -> dict[tuple[str, str], LinkCondition]:
    """Copy pair inputs into canonical order; reject ambiguous reversed duplicates."""
    result = {}
    for pair, condition in (conditions.items() if conditions is not None else ()):
        require(isinstance(pair, tuple) and len(pair) == 2
                and all(type(n) is str and bool(n.strip()) for n in pair)
                and pair[0] != pair[1], "condition key must be a distinct string-ID pair")
        require(isinstance(condition, LinkCondition), "invalid condition value")
        key = tuple(sorted(pair))
        require(key not in result, "duplicate unordered condition pair")
        result[key] = condition
    return dict(sorted(result.items()))


def build_network_graph(
    snapshot: StateSnapshot,
    channel: ChannelModel,
    *,
    conditions: Mapping[tuple[str, str], LinkCondition] | None = None,
) -> nx.Graph:
    """Build GCS + available UAVs; each edge carries canonical LinkState metrics.

    Conditions describe this evaluation only; this function does not execute events.
    Snapshot.network and stored UAV connectivity flags are not topology inputs.
    """
    require(isinstance(snapshot, StateSnapshot), "expected StateSnapshot")
    require(isinstance(channel, ChannelModel), "expected ChannelModel")
    state = snapshot.state
    all_nodes = {state.gcs.id: state.gcs, **{u.id: u for u in state.uavs}}
    pair_conditions = normalize_conditions(conditions)
    require(all(a in all_nodes and b in all_nodes for a,b in pair_conditions),
            "condition references unknown node")
    nodes = {uid: node for uid, node in sorted(all_nodes.items()) if node_available(node)}
    graph = nx.Graph(gcs_id=state.gcs.id, snapshot_revision=snapshot.revision,
                     simulation_time=state.simulation_time)
    for uid, node in nodes.items():
        graph.add_node(uid, kind="GCS" if uid == state.gcs.id else "UAV",
                       position=node.position)
    for a,b in combinations(nodes, 2):
        evaluation = channel.evaluate(
            nodes[a], nodes[b], simulation_time=state.simulation_time,
            condition=pair_conditions.get((a,b)))
        if evaluation.usable:
            link = evaluation.link
            metrics = {field.name: getattr(link, field.name) for field in fields(link)}
            graph.add_edge(a,b, **metrics, link=link, range_feasible=True)
    # Topology is read-only; attribute dictionaries remain local derived NetworkX data.
    return nx.freeze(graph)
