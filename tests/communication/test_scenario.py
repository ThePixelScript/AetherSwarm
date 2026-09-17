import pytest
from dataclasses import replace
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer, route_latency_ms
from ares_swarm.communication.config import CommunicationConfig
from ares_swarm.communication.scenario import CommunicationCondition
from ares_swarm.core.models import StateSnapshot, UAVState


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


def test_outage_begins_and_recovers(snapshot_factory):
    # u1 at (10,0), gcs at (0,0). max range 100.
    conds = [
        CommunicationCondition("u1", "gcs", start_time_s=10.0, end_time_s=20.0, outage=True)
    ]
    analyzer = BaselineCommunicationAnalyzer(CommunicationConfig(max_range=100.0, base_latency=5.0), scenario_conditions=conds)
    
    # Before
    snap_before = snapshot_factory((("u1", 10.0, 0.0),), sim_time=5.0)
    res_before = analyzer.analyze(snap_before)
    assert "u1" in res_before.connected_uav_ids
    assert len(res_before.edge_metrics) == 1
    
    # During
    snap_during = replace(snap_before, simulation_time=15.0)
    res_during = analyzer.analyze(snap_during)
    assert "u1" in res_during.disconnected_uav_ids
    assert len(res_during.edge_metrics) == 0
    
    # After (Recovery)
    snap_after = replace(snap_before, simulation_time=25.0)
    res_after = analyzer.analyze(snap_after)
    assert "u1" in res_after.connected_uav_ids
    assert len(res_after.edge_metrics) == 1


def test_degradation_begins(snapshot_factory):
    conds = [
        CommunicationCondition("u1", "gcs", start_time_s=10.0, end_time_s=20.0, quality_multiplier=0.5)
    ]
    analyzer = BaselineCommunicationAnalyzer(CommunicationConfig(max_range=100.0, base_latency=5.0, packet_loss=0.0), scenario_conditions=conds)
    
    snap_before = snapshot_factory((("u1", 10.0, 0.0),), sim_time=5.0)
    res_before = analyzer.analyze(snap_before)
    etx_before = res_before.edge_metrics[0].etx
    
    snap_during = replace(snap_before, simulation_time=15.0)
    res_during = analyzer.analyze(snap_during)
    etx_during = res_during.edge_metrics[0].etx
    
    assert etx_during > etx_before
    assert res_during.edge_metrics[0].link_quality < res_before.edge_metrics[0].link_quality


def test_latency_degradation(snapshot_factory):
    conds = [
        CommunicationCondition("u1", "gcs", start_time_s=10.0, end_time_s=20.0, latency_penalty_ms=50.0)
    ]
    analyzer = BaselineCommunicationAnalyzer(CommunicationConfig(max_range=100.0, base_latency=5.0), scenario_conditions=conds)
    
    snap_before = snapshot_factory((("u1", 10.0, 0.0),), sim_time=5.0)
    snap_during = replace(snap_before, simulation_time=15.0)
    
    lat_before = analyzer.analyze(snap_before).edge_metrics[0].latency_ms
    lat_during = analyzer.analyze(snap_during).edge_metrics[0].latency_ms
    
    assert lat_during == lat_before + 50.0


def test_packet_loss_condition(snapshot_factory):
    conds = [
        CommunicationCondition("u1", "gcs", start_time_s=10.0, end_time_s=20.0, packet_loss_override=0.8)
    ]
    analyzer = BaselineCommunicationAnalyzer(CommunicationConfig(max_range=100.0, base_latency=5.0, packet_loss=0.0), scenario_conditions=conds)
    
    snap_during = snapshot_factory((("u1", 10.0, 0.0),), sim_time=15.0)
    res = analyzer.analyze(snap_during)
    
    assert res.edge_metrics[0].packet_loss_probability == 0.8
    assert res.edge_metrics[0].estimated_pdr == pytest.approx(0.2)


def test_route_impact(snapshot_factory):
    # u2 -> u1 -> gcs
    conds = [
        CommunicationCondition("u1", "u2", start_time_s=10.0, end_time_s=20.0, outage=True)
    ]
    analyzer = BaselineCommunicationAnalyzer(CommunicationConfig(max_range=100.0), scenario_conditions=conds)
    
    snap = snapshot_factory((("u1", 75.0, 0.0), ("u2", 150.0, 0.0)), sim_time=15.0)
    res = analyzer.analyze(snap)
    
    assert "u1" in res.connected_uav_ids
    assert "u2" in res.disconnected_uav_ids
    assert res.routes_to_gcs["u2"] is None


def test_repeated_analysis(snapshot_factory):
    conds = [
        CommunicationCondition("u1", "gcs", start_time_s=10.0, end_time_s=20.0, outage=True)
    ]
    analyzer = BaselineCommunicationAnalyzer(CommunicationConfig(max_range=100.0), scenario_conditions=conds)
    snap = snapshot_factory((("u1", 10.0, 0.0),), sim_time=15.0)
    
    res1 = analyzer.analyze(snap)
    res2 = analyzer.analyze(snap)
    
    assert res1 == res2


def test_pair_ordering(snapshot_factory):
    conds = [
        CommunicationCondition("gcs", "u1", start_time_s=10.0, end_time_s=20.0, outage=True)
    ]
    analyzer = BaselineCommunicationAnalyzer(CommunicationConfig(max_range=100.0), scenario_conditions=conds)
    snap = snapshot_factory((("u1", 10.0, 0.0),), sim_time=15.0)
    
    res = analyzer.analyze(snap)
    assert "u1" in res.disconnected_uav_ids


def test_route_latency_helper(snapshot_factory):
    analyzer = BaselineCommunicationAnalyzer(CommunicationConfig(max_range=100.0, base_latency=10.0))
    snap = snapshot_factory((("u1", 75.0, 0.0), ("u2", 150.0, 0.0)))
    res = analyzer.analyze(snap)
    
    route = res.routes_to_gcs["u2"]
    latency = route_latency_ms(res, route)
    
    assert latency is not None
    # u2 -> u1 -> gcs: 2 hops. Each hop is ~10ms + distance factor
    assert latency > 20.0


def test_no_cumulative_state(snapshot_factory):
    conds = [
        CommunicationCondition("u1", "gcs", start_time_s=10.0, end_time_s=20.0, outage=True)
    ]
    analyzer = BaselineCommunicationAnalyzer(CommunicationConfig(max_range=100.0), scenario_conditions=conds)
    snap1 = snapshot_factory((("u1", 10.0, 0.0),), sim_time=15.0)
    snap2 = snapshot_factory((("u1", 10.0, 0.0),), sim_time=25.0)
    
    res1 = analyzer.analyze(snap1)
    res2 = analyzer.analyze(snap2)
    
    assert "u1" in res1.disconnected_uav_ids
    assert "u1" in res2.connected_uav_ids
    
    # Ensure res1 remains untouched
    assert "u1" in res1.disconnected_uav_ids
