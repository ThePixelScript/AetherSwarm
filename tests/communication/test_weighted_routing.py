import pytest
import networkx as nx

from ares_swarm.communication.routing import reliability_aware_routes, shortest_hop_routes, RoutingWeights, normalize_link_cost

def make_graph(edges=(), nodes=()):
    graph = nx.Graph()
    graph.add_node("gcs")
    graph.add_nodes_from(nodes)
    for u, v, data in edges:
        # Fill in default LinkState data expected by normalize_link_cost if not provided
        base_data = {
            "etx": 1.0,
            "latency_ms": 10.0,
            "link_quality": 1.0
        }
        base_data.update(data)
        graph.add_edge(u, v, **base_data)
    return graph

def test_etx_normalization():
    weights = RoutingWeights(etx_max=10.0, w_etx=1.0, w_latency=0.0, w_energy=0.0, w_instability=0.0)
    
    # 1. ETX=1 produces minimum ETX penalty (0.0)
    assert normalize_link_cost({"etx": 1.0}, weights) == 0.0
    
    # 2. higher ETX produces higher normalized penalty
    cost_2 = normalize_link_cost({"etx": 2.0}, weights)
    cost_5 = normalize_link_cost({"etx": 5.5}, weights)
    assert 0.0 < cost_2 < cost_5 < 1.0
    assert abs(cost_2 - (2.0 - 1.0)/(10.0 - 1.0)) < 1e-6
    
    # 3. ETX normalization saturates at configured maximum
    assert normalize_link_cost({"etx": 10.0}, weights) == 1.0
    assert normalize_link_cost({"etx": 100.0}, weights) == 1.0

def test_latency_and_instability_bounds():
    weights = RoutingWeights(latency_max_ms=50.0, w_etx=0.0, w_latency=1.0, w_instability=1.0, w_energy=0.0)
    
    # latency bound
    assert normalize_link_cost({"latency_ms": 100.0, "link_quality": 1.0}, weights) == 1.0
    # instability proxy bound (1.0 - link_quality)
    assert normalize_link_cost({"latency_ms": 0.0, "link_quality": 0.0}, weights) == 1.0
    # negative quality (if ever happened, should clip? the current formula doesn't clip, but link_quality is [0,1] bounded by contract)

def test_equal_quality_multihop_alternatives():
    # 1. equal-quality multi-hop alternatives (should use tie-breaker)
    graph = make_graph([
        ("u", "b", {}),
        ("b", "gcs", {}),
        ("u", "a", {}),
        ("a", "gcs", {})
    ])
    routes = reliability_aware_routes(graph, "gcs")
    assert routes["u"] == ("u", "a", "gcs") # tie-breaker chooses 'a' over 'b'
    
def test_short_route_poor_etx_vs_longer_better_etx():
    # 5. longer reliable route can beat short lossy route
    # short route with poor ETX vs longer route with better ETX
    graph = make_graph([
        ("u", "gcs", {"etx": 8.0}), 
        ("u", "a", {"etx": 1.0}),
        ("a", "gcs", {"etx": 1.0})
    ])
    routes = reliability_aware_routes(graph, "gcs")
    assert routes["u"] == ("u", "a", "gcs")
    
    # 6. shortest-hop baseline remains unchanged
    assert shortest_hop_routes(graph, "gcs")["u"] == ("u", "gcs")
    
def test_short_route_high_latency():
    # 7. short route with high latency
    graph = make_graph([
        ("u", "gcs", {"latency_ms": 100.0}), # Latency norm = 1.0
        ("u", "a", {"latency_ms": 10.0}), 
        ("a", "gcs", {"latency_ms": 10.0}) 
    ])
    routes = reliability_aware_routes(graph, "gcs")
    assert routes["u"] == ("u", "a", "gcs")
    assert shortest_hop_routes(graph, "gcs")["u"] == ("u", "gcs")
    
def test_unstable_degraded_route():
    # 8. unstable/degraded route
    graph = make_graph([
        ("u", "gcs", {"link_quality": 0.1}), 
        ("u", "a", {"link_quality": 1.0}), 
        ("a", "gcs", {"link_quality": 1.0}) 
    ])
    routes = reliability_aware_routes(graph, "gcs")
    assert routes["u"] == ("u", "a", "gcs")
    
def test_temporary_outage():
    # 4. unusable/PDR=0 link is excluded (ETX=infinity, saturates at 1)
    graph = make_graph([
        ("u", "gcs", {"etx": float('inf')}),
        ("u", "a", {"etx": 1.0}),
        ("a", "gcs", {"etx": 1.0})
    ])
    routes = reliability_aware_routes(graph, "gcs")
    assert routes["u"] == ("u", "a", "gcs")

def test_equal_weighted_cost_tie():
    # 9. equal weighted cost still tie-breaks deterministically
    graph = make_graph([
        ("u", "c", {}),
        ("c", "gcs", {}),
        ("u", "b", {}),
        ("b", "gcs", {})
    ])
    routes = reliability_aware_routes(graph, "gcs")
    assert routes["u"] == ("u", "b", "gcs") # tie-breaker chooses 'b' over 'c'
    
def test_disconnected_node():
    graph = make_graph(nodes=["u"])
    routes = reliability_aware_routes(graph, "gcs")
    assert routes["u"] is None
    
def test_failed_intermediate_node():
    graph = make_graph([
        ("u", "a", {})
    ])
    routes = reliability_aware_routes(graph, "gcs")
    assert routes["u"] is None

def test_repeated_identical_evaluation():
    # 10. repeated evaluation returns identical routes
    graph = make_graph([
        ("u", "gcs", {"etx": 5.0}),
        ("u", "a", {"etx": 1.0}),
        ("a", "gcs", {"etx": 1.0})
    ])
    routes1 = reliability_aware_routes(graph, "gcs")
    routes2 = reliability_aware_routes(graph, "gcs")
    assert routes1 == routes2
    assert routes1["u"] == ("u", "a", "gcs")
