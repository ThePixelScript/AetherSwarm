"""Focused unit tests for A1 destination-aware communication candidate evaluation."""
import pytest
from types import MappingProxyType

from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
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

    Generic topology setup:
    - GCS at (-50, 500), comm_range = 100m.
    - uav_gateway at (50, 500): dist to GCS = 100m <= 100m (sole gateway).
    - uav_leaf at (50, 420): dist to gateway = 80m <= 100m (routes via gateway).
    - task_remote at (80, 420):
      * If gateway moves to (80, 420): dist to GCS = 152.6m > 100m -> disconnected!
      * If leaf moves to (80, 420): dist to gateway at (50, 500) = 85.4m <= 100m -> connected via gateway!
    """
    uav_leaf = UAVState(id="uav_leaf", position_xy=(50.0, 420.0))
    uav_gateway = UAVState(id="uav_gateway", position_xy=(50.0, 500.0))
    task_remote = TaskState(id="task_remote", position_xy=(80.0, 420.0), priority=5)

    snap = _make_snapshot([uav_leaf, uav_gateway], [task_remote])
    comm_config = CommunicationConfig(max_range=100.0, base_latency=5.0)
    analyzer = BaselineCommunicationAnalyzer(config=comm_config)
    net = analyzer.analyze(snap)

    assert "uav_gateway" in net.connected_uav_ids
    assert "uav_leaf" in net.connected_uav_ids

    # 1. Without destination awareness (legacy A1):
    # uav_gateway has 1 hop to GCS with higher PDR than uav_leaf (2 hops), so it wins the task.
    legacy_allocator = A1TaskAllocator(A1AllocatorConfig(destination_aware=False), comm_analyzer=analyzer)
    legacy_res = legacy_allocator.allocate(snap, network_analysis=net, snapshot=snap)
    assert legacy_res.assignments[0].uav_id == "uav_gateway"

    # 2. With destination awareness (new A1):
    # uav_gateway evaluated at destination (80, 420) is disconnected from GCS and demoted to min_comm_factor.
    # uav_leaf evaluated at destination (80, 420) remains connected through uav_gateway (which stays at 50, 500).
    dest_allocator = A1TaskAllocator(A1AllocatorConfig(destination_aware=True), comm_analyzer=analyzer)
    dest_res = dest_allocator.allocate(snap, network_analysis=net, snapshot=snap)
    assert dest_res.assignments[0].uav_id == "uav_leaf"


def test_destination_aware_utility_scoring():
    """Verify compute_utility directly reflects destination network topology when snapshot provided."""
    uav_leaf = UAVState(id="uav_leaf", position_xy=(50.0, 420.0))
    uav_gateway = UAVState(id="uav_gateway", position_xy=(50.0, 500.0))
    task_remote = TaskState(id="task_remote", position_xy=(80.0, 420.0), priority=5)

    snap = _make_snapshot([uav_leaf, uav_gateway], [task_remote])
    comm_config = CommunicationConfig(max_range=100.0)
    analyzer = BaselineCommunicationAnalyzer(config=comm_config)
    net = analyzer.analyze(snap)

    allocator = A1TaskAllocator(comm_analyzer=analyzer)

    # Calling compute_utility with snapshot evaluates at destination
    score_leaf_dest = allocator.compute_utility(uav_leaf, task_remote, network_analysis=net, snapshot=snap)
    score_gateway_dest = allocator.compute_utility(uav_gateway, task_remote, network_analysis=net, snapshot=snap)

    # uav_leaf should score higher than uav_gateway because uav_gateway becomes disconnected at destination
    assert score_leaf_dest.total > score_gateway_dest.total
    # uav_gateway should receive a network penalty
    assert score_gateway_dest.total == float("-inf")


def test_destination_aware_rejects_disconnected_candidate_feasibility():
    """Verify that when destination is disconnected, candidate is rejected."""
    uav = UAVState(id="uav_single", position_xy=(50.0, 500.0))
    task_far = TaskState(id="task_far", position_xy=(500.0, 500.0), priority=5)

    snap = _make_snapshot([uav], [task_far])
    comm_config = CommunicationConfig(max_range=100.0)
    analyzer = BaselineCommunicationAnalyzer(config=comm_config)
    net = analyzer.analyze(snap)

    allocator = A1TaskAllocator(comm_analyzer=analyzer)
    res = allocator.allocate(snap, network_analysis=net, snapshot=snap)

    assert len(res.assignments) == 0
    assert "task_far" in res.unassigned_tasks


def test_a0_adapter_explicit_capability_detection():
    """Verify A0AutonomyAdapter explicitly passes snapshot to A1 without swallow of TypeErrors."""
    uav_leaf = UAVState(id="uav_leaf", position_xy=(50.0, 420.0))
    uav_gateway = UAVState(id="uav_gateway", position_xy=(50.0, 500.0))
    task_remote = TaskState(id="task_remote", position_xy=(80.0, 420.0), priority=5)

    snap = _make_snapshot([uav_leaf, uav_gateway], [task_remote])
    comm_config = CommunicationConfig(max_range=100.0)
    analyzer = BaselineCommunicationAnalyzer(config=comm_config)
    net = analyzer.analyze(snap)

    allocator = A1TaskAllocator(comm_analyzer=analyzer)
    adapter = A0AutonomyAdapter(allocator=allocator)

    # Calling plan via A0AutonomyAdapter should pass snapshot to A1 and select uav_leaf
    cmds = adapter.plan(snap, network_analysis=net)
    assert len(cmds) == 1
    assert cmds[0].uav_id == "uav_leaf"


def test_a0_adapter_does_not_swallow_internal_type_error():
    """Verify internal TypeError inside allocator is NOT swallowed as a capability mismatch."""
    class BuggyAllocator:
        accepts_snapshot = True
        accepts_network_analysis = True

        def allocate(self, uavs, tasks, **kwargs):
            # Internal implementation bug raising TypeError
            return None + 42

    uav = UAVState(id="uav_test", position_xy=(50.0, 500.0))
    task = TaskState(id="task_test", position_xy=(50.0, 500.0), priority=5)
    snap = _make_snapshot([uav], [task])

    adapter = A0AutonomyAdapter(allocator=BuggyAllocator())
    with pytest.raises(TypeError, match="unsupported operand type"):
        adapter.plan(snap)
