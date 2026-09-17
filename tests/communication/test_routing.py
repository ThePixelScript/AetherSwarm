import itertools
import networkx as nx
import pytest
from ares_swarm.communication.routing import shortest_hop_routes
from ares_swarm.communication.graph import build_network_graph
from ares_swarm.core.exceptions import ModelError

def make(edges=(), nodes=()):
    graph=nx.Graph()
    graph.add_node("gcs")
    graph.add_nodes_from(nodes)
    graph.add_edges_from(edges)
    return graph

def test_direct():
    assert shortest_hop_routes(make([("u1","gcs")]),"gcs") == {"u1":("u1","gcs")}

def test_multihop_minimum():
    graph=make([("u","a"),("a","b"),("b","gcs"),("u","z"),("z","gcs")])
    assert shortest_hop_routes(graph,"gcs")["u"] == ("u","z","gcs")

def test_equal_hop_insertion_order():
    edges=[("u","b"),("b","gcs"),("u","a"),("a","gcs")]
    for ordered in itertools.permutations(edges):
        routes=shortest_hop_routes(make(ordered),"gcs")
        assert routes["u"] == ("u","a","gcs")

def test_source_direction_lexicographic_not_reversed_gcs_bfs():
    # GCS-first lexicographic traversal would choose gcs,c,b,u; u-first chooses a.
    graph=make([("u","a"),("a","z"),("z","gcs"),
                ("u","b"),("b","c"),("c","gcs")])
    assert shortest_hop_routes(graph,"gcs")["u"] == ("u","a","z","gcs")

def test_metrics_do_not_change_shortest_hop():
    graph=make([("u","gcs"),("u","a"),("a","gcs")])
    graph.edges["u","gcs"].update(etx=1000,latency_ms=1000)
    assert shortest_hop_routes(graph,"gcs")["u"] == ("u","gcs")

def test_disconnected_and_gcs_only():
    assert shortest_hop_routes(make(nodes=["u"]),"gcs") == {"u":None}
    assert shortest_hop_routes(make(),"gcs") == {}

def test_failed_relay(snapshot_factory,channel):
    positions=(("u1",10,0),("u2",20,0))
    graph=build_network_graph(snapshot_factory(positions,failed=("u1",)),channel)
    assert shortest_hop_routes(graph,"gcs") == {"u2":None}

def test_repeat_and_immutability():
    graph=make([("gcs","a"),("a","b")])
    before=list(graph.edges)
    first=shortest_hop_routes(graph,"gcs")
    assert first == shortest_hop_routes(graph,"gcs")
    assert list(graph.edges) == before
    with pytest.raises(TypeError):
        first["a"]=None

def test_exhaustive_small_graph_oracle():
    # All 64 simple graphs on four named nodes, compared with all shortest paths.
    nodes=("gcs","a","b","u")
    pairs=tuple(itertools.combinations(nodes,2))
    for mask in range(1 << len(pairs)):
        graph=make([edge for bit,edge in enumerate(pairs) if mask & (1 << bit)],nodes)
        routes=shortest_hop_routes(graph,"gcs")
        for uid in nodes[1:]:
            expected=min(tuple(p) for p in nx.all_shortest_paths(graph,uid,"gcs"))                 if nx.has_path(graph,uid,"gcs") else None
            assert routes[uid] == expected

def test_invalid_directed_graph():
    with pytest.raises(ModelError):
        shortest_hop_routes(nx.DiGraph([("u","gcs")]),"gcs")
