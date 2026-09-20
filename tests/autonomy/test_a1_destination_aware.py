"""Focused unit tests for A1 destination-aware communication candidate evaluation."""
import pytest
from types import MappingProxyType

from ares_swarm.autonomy.a1_allocator import A1AllocatorConfig, A1TaskAllocator
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.communication.config import CommunicationConfig
from ares_swarm.core.enums import FailureState, Role, RTHState, TaskStatus
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState


def _make_snapshot(uavs: list[UAVState], tasks: list[TaskState], gcs_pos=(-50.0, 500.0)) -> StateSnapshot:
    return StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=1,
        uavs=MappingProxyType({u.id: u for u in uavs}),
        tasks=MappingProxyType({t.id: t for t in tasks}),
        gcs_position=gcs_pos,
    )


def test_destination_aware_demotes_gateway_departure():
    """Verify destination-aware evaluation prevents the sole GCS gateway UAV from abandoning its position.

    Scenario setup:
    - GCS at (-50, 500), comm_range = 100m.
    - uav_3 at (50, 500): dist to GCS = 100m <= 100m (sole gateway).
    - uav_1 at (50, 420): dist to uav_3 = 80m <= 100m (routes via uav_3).
    - Task at (80, 420):
      * If uav_3 moves to (80, 420): dist to GCS = 152.6m > 100m -> disconnected!
      * If uav_1 moves to (80, 420): dist to uav_3 at (50, 500) = 85.4m <= 100m -> connected via uav_3!
    """
    uav_1 = UAVState(id="uav_1", position_xy=(50.0, 420.0))
    uav_3 = UAVState(id="uav_3", position_xy=(50.0, 500.0))
    task = TaskState(id="poi_01", position_xy=(80.0, 420.0), priority=5)

    snap = _make_snapshot([uav_1, uav_3], [task])
    comm_config = CommunicationConfig(max_range=100.0, base_latency=5.0)
    analyzer = BaselineCommunicationAnalyzer(config=comm_config)
    net = analyzer.analyze(snap)

    assert "uav_3" in net.connected_uav_ids
    assert "uav_1" in net.connected_uav_ids

    # 1. Without destination awareness (legacy A1):
    # uav_3 is 1 hop to GCS with higher PDR than uav_1 (2 hops).
    legacy_allocator = A1TaskAllocator(A1AllocatorConfig(destination_aware=False), comm_analyzer=analyzer)
    legacy_res = legacy_allocator.allocate(snap, network_analysis=net, snapshot=snap)
    assert legacy_res.assignments[0].uav_id == "uav_3"

    # 2. With destination awareness (new A1):
    # uav_3 evaluated at destination (80, 420) is disconnected from GCS and demoted to min_comm_factor.
    # uav_1 evaluated at destination (80, 420) remains connected through uav_3 (which stays at 50, 500).
    dest_allocator = A1TaskAllocator(A1AllocatorConfig(destination_aware=True), comm_analyzer=analyzer)
    dest_res = dest_allocator.allocate(snap, network_analysis=net, snapshot=snap)
    assert dest_res.assignments[0].uav_id == "uav_1"


def test_destination_aware_utility_scoring():
    """Verify compute_utility directly reflects destination network topology when snapshot provided."""
    uav_1 = UAVState(id="uav_1", position_xy=(50.0, 420.0))
    uav_3 = UAVState(id="uav_3", position_xy=(50.0, 500.0))
    task = TaskState(id="poi_01", position_xy=(80.0, 420.0), priority=5)

    snap = _make_snapshot([uav_1, uav_3], [task])
    comm_config = CommunicationConfig(max_range=100.0)
    analyzer = BaselineCommunicationAnalyzer(config=comm_config)
    net = analyzer.analyze(snap)

    allocator = A1TaskAllocator(comm_analyzer=analyzer)

    # Calling compute_utility with snapshot evaluates at destination
    score_u1_dest = allocator.compute_utility(uav_1, task, network_analysis=net, snapshot=snap)
    score_u3_dest = allocator.compute_utility(uav_3, task, network_analysis=net, snapshot=snap)

    # uav_1 should score higher than uav_3 because uav_3 becomes disconnected at destination
    assert score_u1_dest.total > score_u3_dest.total
    # uav_3 should have a significant network penalty (network_risk_term > 0)
    assert score_u3_dest.network_risk_term > score_u1_dest.network_risk_term


def test_destination_aware_preserves_disconnected_candidate_feasibility():
    """Verify that even when destination is disconnected, candidate is demoted rather than rejected."""
    # Both UAVs will be disconnected at task location
    uav_1 = UAVState(id="uav_1", position_xy=(50.0, 500.0))
    task_far = TaskState(id="poi_far", position_xy=(500.0, 500.0), priority=5)

    snap = _make_snapshot([uav_1], [task_far])
    comm_config = CommunicationConfig(max_range=100.0)
    analyzer = BaselineCommunicationAnalyzer(config=comm_config)
    net = analyzer.analyze(snap)

    allocator = A1TaskAllocator(comm_analyzer=analyzer)
    res = allocator.allocate(snap, network_analysis=net, snapshot=snap)

    # Task is still assigned (no deadlock!), with comm_factor = min_comm_factor
    assert len(res.assignments) == 1
    assert res.assignments[0].uav_id == "uav_1"


def test_a0_adapter_explicit_capability_detection():
    """Verify A0AutonomyAdapter explicitly passes snapshot to A1 without swallow of TypeErrors."""
    from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter

    uav_1 = UAVState(id="uav_1", position_xy=(50.0, 420.0))
    uav_3 = UAVState(id="uav_3", position_xy=(50.0, 500.0))
    task = TaskState(id="poi_01", position_xy=(80.0, 420.0), priority=5)

    snap = _make_snapshot([uav_1, uav_3], [task])
    comm_config = CommunicationConfig(max_range=100.0)
    analyzer = BaselineCommunicationAnalyzer(config=comm_config)
    net = analyzer.analyze(snap)

    allocator = A1TaskAllocator(comm_analyzer=analyzer)
    adapter = A0AutonomyAdapter(allocator=allocator)

    # Calling plan via A0AutonomyAdapter should pass snapshot to A1 and select uav_1
    cmds = adapter.plan(snap, network_analysis=net)
    assert len(cmds) == 1
    assert cmds[0].uav_id == "uav_1"


def test_a0_adapter_does_not_swallow_internal_type_error():
    """Verify internal TypeError inside allocator is NOT swallowed as a capability mismatch."""
    from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter

    class BuggyAllocator:
        accepts_snapshot = True
        accepts_network_analysis = True

        def allocate(self, uavs, tasks, **kwargs):
            # Internal implementation bug raising TypeError
            return None + 42

    uav_1 = UAVState(id="uav_1", position_xy=(50.0, 500.0))
    task = TaskState(id="task_1", position_xy=(50.0, 500.0), priority=5)
    snap = _make_snapshot([uav_1], [task])

    adapter = A0AutonomyAdapter(allocator=BuggyAllocator())
    with pytest.raises(TypeError, match="unsupported operand type"):
        adapter.plan(snap)

