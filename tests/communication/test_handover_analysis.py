import pytest
from types import MappingProxyType
import networkx as nx

from ares_swarm.core.models import StateSnapshot, UAVState
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.communication.config import CommunicationConfig
from ares_swarm.communication.handover_analysis import (
    analyze_relay_dependency,
    evaluate_replacement_candidate,
    verify_handover_connectivity
)

def make_snapshot(uavs, gcs_pos=(0.0, 0.0)):
    return StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({u.id: u for u in uavs}),
        tasks=MappingProxyType({}),
        gcs_position=gcs_pos
    )

def make_uav(u_id, x, y, active=True):
    return UAVState(id=u_id, position_xy=(x, y), active=active)

@pytest.fixture
def analyzer():
    return BaselineCommunicationAnalyzer(
        config=CommunicationConfig(max_range=15.0, base_latency=5.0, packet_loss=0.0)
    )

def test_analyze_simple_chain(analyzer):
    # A. simple chain: GCS(0,0) — R1(10,0) — U2(20,0)
    r1 = make_uav("r1", 10.0, 0.0)
    u2 = make_uav("u2", 20.0, 0.0)
    
    snap = make_snapshot([r1, u2])
    na = analyzer.analyze(snap)
    
    from ares_swarm.communication.graph import build_network_graph
    from ares_swarm.communication.channel import ChannelModel
    graph = build_network_graph(snap, ChannelModel(analyzer.config))
    
    dep = analyze_relay_dependency(graph, "r1", na)
    
    assert dep.relay_id == "r1"
    assert dep.is_articulation_point is True
    assert dep.dependent_uav_ids == ("u2",)
    assert dep.disconnected_if_removed == ("u2",)
    assert dep.affected_route_uav_ids == ("u2",)
    assert dep.protected_uav_ids == ("u2",)

def test_analyze_branching_topology(analyzer):
    # B. branching topology: GCS(0,0) — R1(10,0) — U2(20,0) / U3(20, 10)
    r1 = make_uav("r1", 10.0, 0.0)
    u2 = make_uav("u2", 20.0, 0.0)
    u3 = make_uav("u3", 20.0, 10.0)
    
    snap = make_snapshot([r1, u2, u3])
    na = analyzer.analyze(snap)
    
    from ares_swarm.communication.graph import build_network_graph
    from ares_swarm.communication.channel import ChannelModel
    graph = build_network_graph(snap, ChannelModel(analyzer.config))
    
    dep = analyze_relay_dependency(graph, "r1", na)
    assert dep.dependent_uav_ids == ("u2", "u3")
    assert dep.protected_uav_ids == ("u2", "u3")
    assert dep.is_articulation_point is True

def test_analyze_non_critical_uav(analyzer):
    # C. non-critical UAV
    r1 = make_uav("r1", 10.0, 0.0)
    u2 = make_uav("u2", 10.0, 10.0) # Directly connected to GCS (distance 14.1)
    
    snap = make_snapshot([r1, u2])
    na = analyzer.analyze(snap)
    
    from ares_swarm.communication.graph import build_network_graph
    from ares_swarm.communication.channel import ChannelModel
    graph = build_network_graph(snap, ChannelModel(analyzer.config))
    
    dep = analyze_relay_dependency(graph, "u2", na)
    assert dep.dependent_uav_ids == ()
    assert dep.affected_route_uav_ids == ()
    assert dep.is_articulation_point is False

def test_analyze_route_dependency(analyzer):
    # D. route dependency
    r1 = make_uav("r1", 10.0, 0.0)
    r2 = make_uav("r2", 20.0, 0.0)
    u3 = make_uav("u3", 30.0, 0.0)
    
    snap = make_snapshot([r1, r2, u3])
    na = analyzer.analyze(snap)
    
    from ares_swarm.communication.graph import build_network_graph
    from ares_swarm.communication.channel import ChannelModel
    graph = build_network_graph(snap, ChannelModel(analyzer.config))
    
    dep = analyze_relay_dependency(graph, "r2", na)
    assert dep.affected_route_uav_ids == ("u3",)

def test_hypothetical_successful_replacement(analyzer):
    # E. hypothetical successful replacement
    # We remove r1 from snapshot by making it inactive so u2 is disconnected.
    r1 = make_uav("r1", 10.0, 0.0, active=False)
    u2 = make_uav("u2", 20.0, 0.0)
    c3 = make_uav("c3", 100.0, 0.0) # Disconnected candidate
    
    snap = make_snapshot([r1, u2, c3])
    
    # Move c3 to (10, 0)
    ev = evaluate_replacement_candidate(snap, analyzer, "c3", (10.0, 0.0), protected_uav_ids=("u2",))
    
    assert ev.feasible is True
    assert ev.all_protected_reachable is True
    assert ev.reachable_protected_uav_ids == ("u2",)
    assert ev.unreachable_protected_uav_ids == ()

def test_hypothetical_failed_replacement(analyzer):
    # F. hypothetical failed replacement
    r1 = make_uav("r1", 10.0, 0.0, active=False)
    u2 = make_uav("u2", 20.0, 0.0)
    c3 = make_uav("c3", 100.0, 0.0)
    
    snap = make_snapshot([r1, u2, c3])
    
    # Move c3 to (50, 0), which is out of range of GCS (15) and u2 (20).
    ev = evaluate_replacement_candidate(snap, analyzer, "c3", (50.0, 0.0), protected_uav_ids=("u2",))
    
    assert ev.feasible is False
    assert ev.reachable_protected_uav_ids == ()
    assert ev.unreachable_protected_uav_ids == ("u2",)

def test_actual_verification_success(analyzer):
    # G. actual verification
    u2 = make_uav("u2", 20.0, 0.0)
    c3 = make_uav("c3", 10.0, 0.0) # now acts as relay
    
    snap = make_snapshot([u2, c3])
    na = analyzer.analyze(snap)
    
    verif = verify_handover_connectivity(na, ("u2",))
    assert verif.all_protected_reachable is True
    assert verif.reachable_uav_ids == ("u2",)

def test_actual_verification_partial(analyzer):
    # H. partial restoration
    u2 = make_uav("u2", 20.0, 0.0)
    u3 = make_uav("u3", 100.0, 0.0) # Still disconnected
    c3 = make_uav("c3", 10.0, 0.0) 
    
    snap = make_snapshot([u2, u3, c3])
    na = analyzer.analyze(snap)
    
    verif = verify_handover_connectivity(na, ("u2", "u3"))
    assert verif.all_protected_reachable is False
    assert verif.reachable_uav_ids == ("u2",)
    assert verif.unreachable_uav_ids == ("u3",)

def test_immutability(analyzer):
    # I. immutability
    r1 = make_uav("r1", 10.0, 0.0)
    u2 = make_uav("u2", 20.0, 0.0)
    snap = make_snapshot([r1, u2])
    
    from ares_swarm.communication.graph import build_network_graph
    from ares_swarm.communication.channel import ChannelModel
    graph = build_network_graph(snap, ChannelModel(analyzer.config))
    
    na = analyzer.analyze(snap)
    
    nodes_before = list(graph.nodes)
    
    analyze_relay_dependency(graph, "r1", na)
    
    # Graph should be unmodified
    assert list(graph.nodes) == nodes_before

def test_determinism(analyzer):
    # J. determinism
    r1 = make_uav("r1", 10.0, 0.0, active=False)
    u2 = make_uav("u2", 20.0, 0.0)
    c3 = make_uav("c3", 100.0, 0.0)
    snap = make_snapshot([r1, u2, c3])
    
    ev1 = evaluate_replacement_candidate(snap, analyzer, "c3", (10.0, 0.0), protected_uav_ids=("u2",))
    ev2 = evaluate_replacement_candidate(snap, analyzer, "c3", (10.0, 0.0), protected_uav_ids=("u2",))
    
    assert ev1 == ev2
