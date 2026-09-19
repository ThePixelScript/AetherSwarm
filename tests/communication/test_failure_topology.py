import pytest
from ares_swarm.core.models import StateSnapshot, UAVState
from ares_swarm.core.enums import FailureState
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.communication.config import CommunicationConfig

@pytest.fixture
def snapshot_factory():
    def _factory(uavs: tuple = (), gcs_pos: tuple = (0.0, 0.0), sim_time: float = 0.0):
        uav_dict = {}
        for u in uavs:
            uid, x, y, active, failure_state = u
            uav_dict[uid] = UAVState(id=uid, position_xy=(x, y), active=active, failure_state=failure_state)
        return StateSnapshot(
            simulation_tick=int(sim_time * 10),
            simulation_time=sim_time,
            state_version=1,
            uavs=uav_dict,
            gcs_position=gcs_pos
        )
    return _factory

def test_failure_topology(snapshot_factory):
    # u2 -> u1 -> gcs
    # u1 is at (50, 0), u2 is at (100, 0)
    config = CommunicationConfig(max_range=75.0, base_latency=10.0, packet_loss=0.0)
    analyzer = BaselineCommunicationAnalyzer(config)

    # BEFORE FAILURE
    snap_before = snapshot_factory((
        ("u1", 50.0, 0.0, True, FailureState.NORMAL),
        ("u2", 100.0, 0.0, True, FailureState.NORMAL)
    ))
    res_before = analyzer.analyze(snap_before)
    
    assert "u1" in res_before.connected_uav_ids
    assert "u2" in res_before.connected_uav_ids
    assert res_before.routes_to_gcs["u2"] == ("u2", "u1", "gcs")
    assert res_before.route_pdr_to_gcs["u2"] > 0

    # POST-FAILURE
    snap_after = snapshot_factory((
        ("u1", 50.0, 0.0, False, FailureState.FAILED),
        ("u2", 100.0, 0.0, True, FailureState.NORMAL)
    ))
    res_after = analyzer.analyze(snap_after)

    # u1 should be completely excluded from the network analysis!
    assert "u1" not in res_after.connected_uav_ids
    assert "u1" not in res_after.disconnected_uav_ids
    assert "u1" not in res_after.routes_to_gcs
    
    # u2 should be disconnected because u1 (its relay) is gone and it's 100m away (max range 75m)
    assert "u2" in res_after.disconnected_uav_ids
    assert res_after.routes_to_gcs["u2"] is None
    assert res_after.route_pdr_to_gcs["u2"] == 0.0

def test_alternate_route(snapshot_factory):
    # u2 -> u1 -> gcs OR u2 -> u3 -> gcs
    config = CommunicationConfig(max_range=75.0, base_latency=10.0, packet_loss=0.0)
    analyzer = BaselineCommunicationAnalyzer(config)

    # POST-FAILURE of u1
    snap_after = snapshot_factory((
        ("u1", 50.0, 0.0, False, FailureState.FAILED),
        ("u2", 100.0, 0.0, True, FailureState.NORMAL),
        ("u3", 50.0, 50.0, True, FailureState.NORMAL)
    ))
    res_after = analyzer.analyze(snap_after)

    assert "u1" not in res_after.routes_to_gcs
    
    # u2 should now route through u3
    assert "u2" in res_after.connected_uav_ids
    assert res_after.routes_to_gcs["u2"] == ("u2", "u3", "gcs")
    assert res_after.route_pdr_to_gcs["u2"] > 0

def test_recovery(snapshot_factory):
    config = CommunicationConfig(max_range=75.0, base_latency=10.0, packet_loss=0.0)
    analyzer = BaselineCommunicationAnalyzer(config)

    # RECOVERY of u1
    snap_recovery = snapshot_factory((
        ("u1", 50.0, 0.0, True, FailureState.NORMAL),
        ("u2", 100.0, 0.0, True, FailureState.NORMAL)
    ))
    # Make a copy to check mutation
    snap_copy = snapshot_factory((
        ("u1", 50.0, 0.0, True, FailureState.NORMAL),
        ("u2", 100.0, 0.0, True, FailureState.NORMAL)
    ))
    
    res_recovery = analyzer.analyze(snap_recovery)
    
    assert "u1" in res_recovery.connected_uav_ids
    assert "u2" in res_recovery.connected_uav_ids
    assert res_recovery.routes_to_gcs["u2"] == ("u2", "u1", "gcs")
    
    # Assert no mutation
    assert snap_recovery == snap_copy

def test_determinism(snapshot_factory):
    config = CommunicationConfig(max_range=75.0, base_latency=10.0, packet_loss=0.0)
    analyzer = BaselineCommunicationAnalyzer(config)

    snap = snapshot_factory((
        ("u1", 50.0, 0.0, False, FailureState.FAILED),
        ("u2", 100.0, 0.0, True, FailureState.NORMAL),
        ("u3", 50.0, 50.0, True, FailureState.NORMAL)
    ))

    res1 = analyzer.analyze(snap)
    res2 = analyzer.analyze(snap)

    assert res1.connected_uav_ids == res2.connected_uav_ids
    assert res1.disconnected_uav_ids == res2.disconnected_uav_ids
    assert res1.components == res2.components
    assert res1.routes_to_gcs == res2.routes_to_gcs
    assert res1.hop_counts == res2.hop_counts
    assert res1.route_pdr_to_gcs == res2.route_pdr_to_gcs
    assert res1.network.links == res2.network.links
