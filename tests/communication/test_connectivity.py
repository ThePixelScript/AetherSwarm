import networkx as nx
import pytest
from ares_swarm.communication.connectivity import analyze_connectivity
from ares_swarm.communication.graph import build_network_graph
from ares_swarm.core.exceptions import ModelError

def graph(edges=(), nodes=()):
    result=nx.Graph()
    result.add_node("gcs")
    result.add_nodes_from(nodes)
    result.add_edges_from(edges)
    return result

def test_all_connected_and_hops():
    result=analyze_connectivity(graph([("gcs","u1"),("u1","u2"),("u2","u3")]),"gcs")
    assert result.connected_uav_ids == ("u1","u2","u3")
    assert result.disconnected_uav_ids == ()
    assert result.hop_counts == {"u1":1,"u2":2,"u3":3}
    assert result.components == (("gcs","u1","u2","u3"),)
    assert result.articulation_points == ("u1","u2")
    assert result.network_health["connectivity_ratio"] == 1
    assert result.network_health["average_hop_count"] == 2
    assert result.network_health["maximum_hop_count"] == 3

def test_multiple_components_and_disconnected_node():
    result=analyze_connectivity(graph([("gcs","u1"),("u2","u3")],["u4"]),"gcs")
    assert result.connected_uav_ids == ("u1",)
    assert result.disconnected_uav_ids == ("u2","u3","u4")
    assert result.components == (("gcs","u1"),("u2","u3"),("u4",))
    assert result.hop_counts == {"u1":1,"u2":None,"u3":None,"u4":None}
    assert result.network_health["disconnected_component_count"] == 2
    assert result.network_health["connectivity_ratio"] == 0.25

def test_no_articulation():
    result=analyze_connectivity(graph([("gcs","u1"),("gcs","u2"),("u1","u2")]),"gcs")
    assert result.articulation_points == ()

def test_gcs_can_be_articulation():
    result=analyze_connectivity(graph([("gcs","u1"),("gcs","u2")]),"gcs")
    assert result.articulation_points == ("gcs",)

def test_gcs_only():
    result=analyze_connectivity(graph(),"gcs")
    assert result.components == (("gcs",),)
    assert result.network_health["connectivity_ratio"] is None
    assert result.network_health["average_hop_count"] is None
    assert result.network_health["largest_component_size"] == 1
    assert result.hop_counts == {}

def test_none_connected():
    result=analyze_connectivity(graph(nodes=["u1"]),"gcs")
    assert result.network_health["connectivity_ratio"] == 0
    assert result.network_health["average_hop_count"] is None

def test_topology_after_failure(snapshot_factory,channel):
    positions=(("u1",10,0),("u2",20,0))
    before=analyze_connectivity(build_network_graph(snapshot_factory(positions),channel),"gcs")
    after=analyze_connectivity(build_network_graph(
        snapshot_factory(positions,failed=("u1",)),channel),"gcs")
    assert before.articulation_points == ("u1",)
    assert after.disconnected_uav_ids == ("u2",)
    assert after.components == (("gcs",),("u2",))
    assert after.hop_counts == {"u2":None}

def test_order_independence_and_immutability():
    edges=[("gcs","u1"),("u1","u2"),("u3","u4")]
    a=analyze_connectivity(graph(edges),"gcs")
    b=analyze_connectivity(graph(reversed(edges)),"gcs")
    assert a == b
    with pytest.raises(TypeError):
        a.hop_counts["u1"]=99
    with pytest.raises(TypeError):
        a.network_health["connectivity_ratio"]=9

@pytest.mark.parametrize("bad", [nx.Graph(), nx.DiGraph([("gcs","u")]),
                                nx.MultiGraph([("gcs","u")]),
                                nx.Graph([("gcs","gcs")])])
def test_invalid_graph(bad):
    with pytest.raises(ModelError):
        analyze_connectivity(bad,"gcs")
