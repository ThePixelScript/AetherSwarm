"""Regression coverage for optional M2 and loop-free zero-cost routing."""
from itertools import combinations

import networkx as nx
import pytest

from ares_swarm.communication import analysis as analysis_module
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer, route_pdr
from ares_swarm.communication.channel import LinkCondition
from ares_swarm.communication.config import CommunicationConfig
from ares_swarm.communication.routing import reliability_aware_routes, shortest_hop_routes
from ares_swarm.core.state_store import StateStore


def zero_graph(edges, nodes=()):
    graph = nx.Graph()
    graph.add_nodes_from(('gcs', *nodes))
    for a, b in edges:
        graph.add_edge(a, b, etx=1.0, latency_ms=0.0, link_quality=1.0)
    return graph


@pytest.mark.parametrize('enabled', [False, True])
def test_zero_cost_analyzer_and_state_unchanged(snapshot_factory, enabled):
    store = StateStore(snapshot_factory((('u1', 0, 0),)))
    before = store.snapshot()
    analyzer = BaselineCommunicationAnalyzer(
        CommunicationConfig(max_range=100, base_latency=0, packet_loss=0),
        enable_reliability_routing=enabled,
    )
    result = analyzer.analyze(before)
    assert result.routes_to_gcs == {'u1': ('u1', 'gcs')}
    assert result.hop_counts == {'u1': 1}
    assert result.route_pdr_to_gcs == {'u1': 1.0}
    assert result.reliable_routes_to_gcs == ({'u1': ('u1', 'gcs')} if enabled else {})
    assert result.reliable_hop_counts == ({'u1': 1} if enabled else {})
    assert analyzer.analyze(before) == result
    assert store.snapshot() == before


def test_default_never_calls_m2(snapshot_factory, monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('M2 must not execute')
    monkeypatch.setattr(analysis_module, 'reliability_aware_routes', forbidden)
    result = BaselineCommunicationAnalyzer(
        CommunicationConfig(base_latency=0)
    ).analyze(snapshot_factory((('u1', 0, 0),)))
    assert result.routes_to_gcs['u1'] == ('u1', 'gcs')
    assert not result.reliable_routes_to_gcs


def test_zero_cost_chain():
    graph = zero_graph([('u2', 'u1'), ('u1', 'gcs')], nodes=('lost',))
    routes = reliability_aware_routes(graph, 'gcs')
    assert routes['u2'] == ('u2', 'u1', 'gcs')
    assert routes['lost'] is None


def test_zero_cost_cycle_lexicographic_and_order_independent():
    edges = [('u', 'b'), ('b', 'gcs'), ('u', 'a'), ('a', 'gcs'), ('a', 'b')]
    first = reliability_aware_routes(zero_graph(edges), 'gcs')
    assert first['u'] == ('u', 'a', 'b', 'gcs')
    assert first == reliability_aware_routes(zero_graph(reversed(edges)), 'gcs')
    for route in first.values():
        assert len(route) == len(set(route))
    assert shortest_hop_routes(zero_graph(edges), 'gcs')['u'] == ('u', 'a', 'gcs')


def test_zero_cost_small_graph_oracle():
    pairs = list(combinations(('gcs', 'a', 'b', 'u'), 2))
    for mask in range(1 << len(pairs)):
        graph = zero_graph([p for i, p in enumerate(pairs) if mask & (1 << i)], ('a', 'b', 'u'))
        routes = reliability_aware_routes(graph, 'gcs')
        for uid in ('a', 'b', 'u'):
            paths = [tuple(p) for p in nx.all_simple_paths(graph, uid, 'gcs')]
            assert routes[uid] == (min(paths) if paths else None)


def test_pdr_stays_on_r0_when_m2_selects_different_route(snapshot_factory):
    snapshot = snapshot_factory((('u', 20, 0), ('a', 10, 0)))
    kwargs = dict(
        config=CommunicationConfig(max_range=100),
        conditions={('gcs', 'u'): LinkCondition(packet_loss_override=0.99)},
    )
    baseline = BaselineCommunicationAnalyzer(**kwargs).analyze(snapshot)
    weighted = BaselineCommunicationAnalyzer(**kwargs, enable_reliability_routing=True).analyze(snapshot)
    assert weighted.routes_to_gcs['u'] == ('u', 'gcs')
    assert weighted.reliable_routes_to_gcs['u'] == ('u', 'a', 'gcs')
    assert weighted.routes_to_gcs == baseline.routes_to_gcs
    assert weighted.hop_counts == baseline.hop_counts
    assert weighted.route_pdr_to_gcs == baseline.route_pdr_to_gcs
    assert weighted.route_pdr_to_gcs['u'] == pytest.approx(0.01)
    assert route_pdr(weighted.edge_metrics, None) == 0.0
    assert route_pdr(weighted.edge_metrics, ('missing', 'gcs')) is None
