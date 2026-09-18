import pytest
from dataclasses import replace
from ares_swarm.communication.analysis import route_pdr, BaselineCommunicationAnalyzer
from ares_swarm.communication.models import LinkState
from ares_swarm.communication.config import CommunicationConfig
from ares_swarm.core.models import StateSnapshot, UAVState
from ares_swarm.communication.channel import LinkCondition

def test_route_pdr_helper_one_hop():
    # 1. ONE-HOP ROUTE
    edges = (
        LinkState(source_id="gcs", target_id="u1", packet_loss_probability=0.2, estimated_pdr=0.8),
    )
    assert route_pdr(edges, ("u1", "gcs")) == 0.8

def test_route_pdr_helper_multi_hop():
    # 2. MULTI-HOP ROUTE
    edges = (
        LinkState(source_id="u1", target_id="u2", packet_loss_probability=0.1, estimated_pdr=0.9),
        LinkState(source_id="gcs", target_id="u1", packet_loss_probability=0.2, estimated_pdr=0.8),
    )
    assert route_pdr(edges, ("u2", "u1", "gcs")) == pytest.approx(0.72)

def test_route_pdr_helper_disconnected():
    # 3. DISCONNECTED UAV
    edges = (
        LinkState(source_id="gcs", target_id="u1", packet_loss_probability=0.2, estimated_pdr=0.8),
    )
    assert route_pdr(edges, None) == 0.0

def test_route_pdr_helper_missing_edge():
    # 4. MISSING EDGE
    edges = (
        LinkState(source_id="gcs", target_id="u1", packet_loss_probability=0.2, estimated_pdr=0.8),
    )
    assert route_pdr(edges, ("u2", "u1", "gcs")) is None

def test_route_pdr_helper_perfect_route():
    # Optional regression
    edges = (
        LinkState(source_id="gcs", target_id="u1", packet_loss_probability=0.0, estimated_pdr=1.0),
    )
    assert route_pdr(edges, ("u1", "gcs")) == 1.0

def test_route_pdr_helper_zero_pdr_route():
    # Optional regression (edge with 0 pdr)
    edges = (
        LinkState(source_id="u1", target_id="u2", packet_loss_probability=1.0, estimated_pdr=0.0, etx=1.0),
        LinkState(source_id="gcs", target_id="u1", packet_loss_probability=0.0, estimated_pdr=1.0),
    )
    assert route_pdr(edges, ("u2", "u1", "gcs")) == 0.0

@pytest.fixture
def snapshot_factory():
    def _factory(uavs: tuple = (), gcs_pos: tuple = (0.0, 0.0), sim_time: float = 0.0):
        uav_dict = {}
        for u in uavs:
            uid, x, y = u
            uav_dict[uid] = UAVState(id=uid, position_xy=(x, y))
        return StateSnapshot(
            simulation_tick=int(sim_time * 10),
            simulation_time=sim_time,
            state_version=1,
            uavs=uav_dict,
            gcs_position=gcs_pos
        )
    return _factory

def test_determinism_and_network_analysis_integration(snapshot_factory):
    # 5. DETERMINISM
    config = CommunicationConfig(max_range=100.0, base_latency=5.0, packet_loss=0.1)
    analyzer = BaselineCommunicationAnalyzer(config)
    
    snap = snapshot_factory((("u1", 10.0, 0.0), ("u2", 20.0, 0.0)))
    
    res1 = analyzer.analyze(snap)
    res2 = analyzer.analyze(snap)
    
    # Must be identical mapping
    assert res1.route_pdr_to_gcs == res2.route_pdr_to_gcs
    
    # Check values
    assert "u1" in res1.route_pdr_to_gcs
    assert "u2" in res1.route_pdr_to_gcs
    assert res1.route_pdr_to_gcs["u1"] > 0.0
    assert res1.route_pdr_to_gcs["u2"] > 0.0
    assert res1.route_pdr_to_gcs["u1"] == pytest.approx(res1.edge_metrics[0].estimated_pdr)
