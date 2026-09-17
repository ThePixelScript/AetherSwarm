from dataclasses import replace
import pytest

from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.communication.channel import LinkCondition
from ares_swarm.communication.config import CommunicationConfig
from ares_swarm.core.enums import FailureState
from ares_swarm.core.models import StateSnapshot, UAVState
from ares_swarm.core.state_store import StateStore
from ares_swarm.interfaces.communication import NetworkAnalysis

def analyzer():
    return BaselineCommunicationAnalyzer(CommunicationConfig(10, 5, 0))

def test_chain_then_failed_relay(snapshot_factory):
    snapshot = snapshot_factory((("u1", 10, 0), ("u2", 20, 0)))
    store = StateStore(snapshot)
    service = analyzer()
    
    before = store.snapshot()
    result = service.analyze(before)
    
    assert result.reachable_uav_ids == ("u1", "u2")
    assert result.routes_to_gcs["u2"] == ("u2", "u1", "gcs")
    assert result.hop_counts["u2"] == 2
    assert result.articulation_points == ("u1",)
    assert result.network_health["average_hop_count"] == 1.5
    
    # Simulate a failed relay by modifying the snapshot and loading into a new store
    failed_u1 = replace(before.uavs["u1"], active=False, failure_state=FailureState.FAILED)
    after_snap = replace(before, uavs={**before.uavs, "u1": failed_u1}, state_version=before.state_version + 1)
    
    failed_result = service.analyze(after_snap)
    assert failed_result.snapshot_revision == after_snap.state_version
    assert failed_result.disconnected_uav_ids == ("u2",)
    assert failed_result.routes_to_gcs == {"u2": None}
    assert failed_result.hop_counts == {"u2": None}
    assert failed_result.edge_metrics == ()

def test_conditions_copied_and_repeatable(snapshot_factory):
    supplied = {("u1", "gcs"): LinkCondition(outage=True)}
    service = BaselineCommunicationAnalyzer(CommunicationConfig(10, 5, 0), supplied)
    supplied.clear()
    snapshot = snapshot_factory((("u1", 10, 0), ("u2", 20, 0)))
    assert service.analyze(snapshot).disconnected_uav_ids == ("u1", "u2")

def test_gcs_only_contract(snapshot_factory):
    result = analyzer().analyze(snapshot_factory())
    assert result.routes_to_gcs == {}
    assert result.network_health["connectivity_ratio"] is None

def test_simulation_time_not_wall_clock(snapshot_factory):
    service = analyzer()
    snapshot = snapshot_factory((("u1", 10, 0),))
    later = replace(snapshot, simulation_time=5, state_version=1)
    result = service.analyze(later)
    assert result.simulation_time == 5
    assert all(link.last_updated == 5 for link in result.edge_metrics)
