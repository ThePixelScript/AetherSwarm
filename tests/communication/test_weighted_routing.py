import pytest
import networkx as nx

from ares_swarm.communication.routing import reliability_aware_routes, shortest_hop_routes, RoutingWeights

def make_graph(edges=(), nodes=()):
    graph = nx.Graph()
    graph.add_node("gcs")
    graph.add_nodes_from(nodes)
    for u, v, data in edges:
        # Fill in default LinkState data expected by normalize_link_cost if not provided
        base_data = {
            "estimated_pdr": 1.0,
            "latency_ms": 10.0,
            "link_quality": 1.0
        }
        base_data.update(data)
        graph.add_edge(u, v, **base_data)
    return graph

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
    
def test_short_route_poor_pdr_vs_longer_better_pdr():
    # 2 & 3. short route with poor PDR vs longer route with better PDR
    graph = make_graph([
        ("u", "gcs", {"estimated_pdr": 0.5}), # Short route, PDR=0.5. ETX_norm = 0.5.
        ("u", "a", {"estimated_pdr": 1.0}),
        ("a", "gcs", {"estimated_pdr": 1.0})
    ])
    # weights: w_etx=0.50. Short cost = 0.5 * 0.5 = 0.25 (plus latency)
    # Long cost = 0 (for etx) (plus latency)
    # Default latency=10ms. latency_norm = 10/50 = 0.2. w_latency=0.2. Latency cost = 0.04.
    # Short route cost = 0.25 (etx) + 0.04 (lat) = 0.29.
    # Long route cost = 0.0 (etx) + 0.04 (lat) + 0.0 (etx) + 0.04 (lat) = 0.08.
    # Long route is better!
    routes = reliability_aware_routes(graph, "gcs")
    assert routes["u"] == ("u", "a", "gcs")
    # Shortest hop route would still pick u-gcs
    assert shortest_hop_routes(graph, "gcs")["u"] == ("u", "gcs")
    
def test_short_route_high_latency():
    # 4. short route with high latency
    graph = make_graph([
        ("u", "gcs", {"latency_ms": 100.0}), # Latency norm = 1.0. cost = 0.2
        ("u", "a", {"latency_ms": 10.0}), # Latency norm = 0.2. cost = 0.04
        ("a", "gcs", {"latency_ms": 10.0}) # cost = 0.04. Total = 0.08.
    ])
    routes = reliability_aware_routes(graph, "gcs")
    assert routes["u"] == ("u", "a", "gcs")
    assert shortest_hop_routes(graph, "gcs")["u"] == ("u", "gcs")
    
def test_unstable_degraded_route():
    # 5. unstable/degraded route
    graph = make_graph([
        ("u", "gcs", {"link_quality": 0.1}), # Instability norm = 0.9. cost = 0.15 * 0.9 = 0.135
        ("u", "a", {"link_quality": 1.0}), # cost = 0
        ("a", "gcs", {"link_quality": 1.0}) # cost = 0
    ])
    # Total for short route = 0.135 + 0.04(lat) = 0.175.
    # Total for long route = 0.08.
    routes = reliability_aware_routes(graph, "gcs")
    assert routes["u"] == ("u", "a", "gcs")
    
def test_temporary_outage():
    # 6. temporary outage (PDR=0 or missing edge)
    # If PDR is 0, ETX norm is 1.0. Cost is 0.5 (etx) + latency.
    graph = make_graph([
        ("u", "gcs", {"estimated_pdr": 0.0}), # Outage simulated by PDR=0
        ("u", "a", {"estimated_pdr": 1.0}),
        ("a", "gcs", {"estimated_pdr": 1.0})
    ])
    routes = reliability_aware_routes(graph, "gcs")
    assert routes["u"] == ("u", "a", "gcs")

def test_equal_weighted_cost_tie():
    # 7. equal weighted cost tie
    graph = make_graph([
        ("u", "c", {}),
        ("c", "gcs", {}),
        ("u", "b", {}),
        ("b", "gcs", {})
    ])
    routes = reliability_aware_routes(graph, "gcs")
    assert routes["u"] == ("u", "b", "gcs") # tie-breaker chooses 'b' over 'c'
    
def test_disconnected_node():
    # 8. disconnected node
    graph = make_graph(nodes=["u"])
    routes = reliability_aware_routes(graph, "gcs")
    assert routes["u"] is None
    
def test_failed_intermediate_node():
    # 9. failed intermediate node (node missing)
    graph = make_graph([
        ("u", "a", {})
    ])
    routes = reliability_aware_routes(graph, "gcs")
    assert routes["u"] is None

def test_repeated_identical_evaluation():
    # 10. repeated identical evaluation
    graph = make_graph([
        ("u", "gcs", {"estimated_pdr": 0.5}),
        ("u", "a", {"estimated_pdr": 1.0}),
        ("a", "gcs", {"estimated_pdr": 1.0})
    ])
    routes1 = reliability_aware_routes(graph, "gcs")
    routes2 = reliability_aware_routes(graph, "gcs")
    assert routes1 == routes2
    assert routes1["u"] == ("u", "a", "gcs")
