from dataclasses import replace
import networkx as nx
import pytest
from ares_swarm.communication.graph import build_network_graph
from ares_swarm.communication.channel import LinkCondition, ChannelModel
from ares_swarm.communication.config import CommunicationConfig

import copy

def signature(graph):
    return list(graph.nodes(data=True)), list(graph.edges(data=True)), dict(graph.graph)

def test_gcs_only(snapshot_factory, channel):
    graph = build_network_graph(snapshot_factory(),channel)
    assert list(graph) == ["gcs"]
    assert graph.number_of_edges() == 0

@pytest.mark.parametrize("count", [1,2,3])
def test_direct_and_chain(snapshot_factory, channel, count):
    snapshot = snapshot_factory(tuple((f"u{i}",10*i,0) for i in range(1,count+1)))
    graph = build_network_graph(snapshot,channel)
    assert graph.number_of_edges() == count
    assert nx.shortest_path_length(graph,f"u{count}","gcs") == count
    for a,b,data in graph.edges(data=True):
        assert data["link"].source_id == min(a,b)
        assert data["estimated_pdr"] == 0.5
        assert data["etx"] == 2
        assert data["latency_ms"] == 10
        assert data["active"] and data["range_feasible"]
        assert data["last_updated"] == snapshot.simulation_time

def test_partition_and_failed_intermediate(snapshot_factory,channel):
    positions = (("u1",10,0),("u2",20,0),("u3",100,0),("u4",110,0))
    graph = build_network_graph(snapshot_factory(positions),channel)
    assert nx.number_connected_components(graph) == 2
    failed = build_network_graph(snapshot_factory(positions,failed=("u1",)),channel)
    assert "u1" not in failed and not nx.has_path(failed,"u2","gcs")
    assert nx.number_connected_components(failed) == 3

def test_inactive_excluded(snapshot_factory,channel):
    graph = build_network_graph(snapshot_factory((("u1",1,0),),inactive=("u1",)),channel)
    assert list(graph) == ["gcs"]

def test_rebuild_order_and_snapshot_unchanged(snapshot_factory,channel):
    positions=(("u2",20,0),("u1",10,0))
    snapshot=snapshot_factory(positions)
    before=repr(snapshot)
    first=build_network_graph(snapshot,channel)
    assert signature(first) == signature(build_network_graph(snapshot,channel))
    assert signature(first) == signature(build_network_graph(snapshot_factory(positions,reverse=True),channel))
    assert repr(snapshot) == before
    with pytest.raises(nx.NetworkXError):
        first.remove_node("gcs")
    first.nodes["u1"]["position"] = "local-only"
    first.edges["gcs","u1"]["etx"] = 99
    assert repr(snapshot) == before
    assert build_network_graph(snapshot,channel).edges["gcs","u1"]["etx"] == 2

def test_condition_hooks(snapshot_factory,channel):
    snapshot=snapshot_factory((("u1",10,0),))
    degraded=build_network_graph(snapshot,channel,
                                conditions={("u1","gcs"):LinkCondition(0.5,7)})
    assert degraded.edges["gcs","u1"]["estimated_pdr"] == 0.25
    assert degraded.edges["gcs","u1"]["latency_ms"] == 17
    outage=build_network_graph(snapshot,channel,
                              conditions={("gcs","u1"):LinkCondition(outage=True)})
    assert outage.number_of_edges() == 0

def test_zero_pdr_excluded(snapshot_factory):
    graph=build_network_graph(snapshot_factory((("u1",1,0),)),
                              ChannelModel(CommunicationConfig(10,5,1)))
    assert graph.number_of_edges() == 0

@pytest.mark.parametrize("conditions", [
    {("gcs","missing"):LinkCondition()},
    {("gcs","u1"):LinkCondition(), ("u1","gcs"):LinkCondition()},
    {("gcs","gcs"):LinkCondition()},
])
def test_invalid_conditions(snapshot_factory,channel,conditions):
    with pytest.raises(ValueError):
        build_network_graph(snapshot_factory((("u1",1,0),)),channel,conditions=conditions)
