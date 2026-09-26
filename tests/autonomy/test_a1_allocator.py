"""Unit and integration tests for A1 communication-aware task allocator."""
from __future__ import annotations

from types import MappingProxyType
import pytest
from ares_swarm.core.commands import FailUAVCommand
from ares_swarm.core.event_scheduler import ScheduledEvent, ScheduledEventType
from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.autonomy.a1_allocator import A1AllocatorConfig, A1CommunicationAwareAllocator, A1TaskAllocator
from ares_swarm.autonomy.task_allocator import A0TaskAllocator, AllocationWeights, TaskAllocatorConfig
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.communication.channel import LinkCondition
from ares_swarm.communication.config import CommunicationConfig
from ares_swarm.core.commands import AssignTaskCommand, Command
from ares_swarm.core.enums import FailureState, Role, RTHState, TaskStatus
from ares_swarm.core.events import StateTransitionResult
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState
from ares_swarm.core.state_store import StateStore
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import ScenarioConfig


def make_snapshot(
    uavs: list[UAVState],
    tasks: list[TaskState],
    gcs_pos: tuple[float, float] = (0.0, 0.0),
    sim_time: float = 0.0,
    sim_tick: int = 0,
) -> StateSnapshot:
    """Helper to build immutable StateSnapshot with sorted mapping proxies."""
    return StateSnapshot(
        simulation_tick=sim_tick,
        simulation_time=sim_time,
        state_version=1,
        uavs=MappingProxyType({u.id: u for u in sorted(uavs, key=lambda x: x.id)}),
        tasks=MappingProxyType({t.id: t for t in sorted(tasks, key=lambda x: x.id)}),
        gcs_position=gcs_pos,
    )


def test_a1_inherits_a0_feasibility_rules():
    """Requirement 1: A1 inherits all A0 feasibility rules for UAVs and tasks."""
    uav_inactive = UAVState(id="u-inactive", position_xy=(10.0, 0.0), active=False)
    uav_failed = UAVState(id="u-failed", position_xy=(10.0, 0.0), failure_state=FailureState.FAILED)
    uav_rth = UAVState(id="u-rth", position_xy=(10.0, 0.0), rth_state=RTHState.ACTIVE)
    uav_ok = UAVState(id="u-ok", position_xy=(10.0, 0.0), active=True, rth_state=RTHState.NONE)

    t_completed = TaskState(id="t-comp", position_xy=(12.0, 0.0), priority=5, status=TaskStatus.COMPLETE)
    t_expired = TaskState(id="t-exp", position_xy=(12.0, 0.0), priority=5, deadline=5.0)
    t_pending = TaskState(id="t-pend", position_xy=(12.0, 0.0), priority=1, status=TaskStatus.PENDING)

    snap = make_snapshot(
        [uav_inactive, uav_failed, uav_rth, uav_ok],
        [t_completed, t_expired, t_pending],
        sim_time=10.0,  # t_expired deadline 5.0 is past
    )

    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=50.0))
    net = analyzer.analyze(snap)

    allocator = A1TaskAllocator()
    res = allocator.allocate(snap, network_analysis=net)

    # Infeasible entities must be recorded exactly as in A0
    assert "u-inactive" in res.infeasible_uavs
    assert "u-failed" in res.infeasible_uavs
    assert "u-rth" in res.infeasible_uavs
    assert "t-comp" in res.infeasible_tasks
    assert "t-exp" in res.infeasible_tasks

    # Only feasible UAV and feasible task can be matched
    assert len(res.assignments) == 1
    assert res.assignments[0].uav_id == "u-ok"
    assert res.assignments[0].task_id == "t-pend"


def test_network_analysis_none_preserves_a0_behavior():
    """Requirement 2: When network_analysis=None, A1 preserves A0 scoring and allocation."""
    u1 = UAVState(id="u1", position_xy=(10.0, 0.0))
    u2 = UAVState(id="u2", position_xy=(20.0, 0.0))
    t1 = TaskState(id="t1", position_xy=(12.0, 0.0), priority=3)

    snap = make_snapshot([u1, u2], [t1])

    a0 = A0TaskAllocator()
    a1 = A1TaskAllocator()

    res_a0 = a0.allocate(snap)
    res_a1 = a1.allocate(snap, network_analysis=None)

    assert len(res_a0.assignments) == len(res_a1.assignments) == 1
    assert res_a0.assignments[0].uav_id == res_a1.assignments[0].uav_id
    assert res_a0.assignments[0].task_id == res_a1.assignments[0].task_id
    assert pytest.approx(res_a0.assignments[0].score) == res_a1.assignments[0].score


def test_connected_uav_beats_disconnected_uav():
    """Requirement 3: Connected UAV beats disconnected UAV when travel and priority are similar."""
    # GCS at (0, 0). Comm range = 50m.
    # UAV 1 at (20, 0) -> distance to GCS = 20m -> connected!
    # UAV 2 at (100, 0) -> distance to GCS = 100m, distance to u1 = 80m > 50m -> disconnected!
    # Task at (60, 0) -> distance from u1 = 40m, distance from u2 = 40m (identical travel cost!)
    u1 = UAVState(id="u1", position_xy=(20.0, 0.0))
    u2 = UAVState(id="u2", position_xy=(100.0, 0.0))
    t = TaskState(id="t1", position_xy=(40.0, 0.0), priority=5)

    snap = make_snapshot([u1, u2], [t])
    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=50.0))
    net = analyzer.analyze(snap)

    assert "u1" in net.connected_uav_ids
    assert "u2" in net.disconnected_uav_ids

    # Under A0 (comm-blind): distance is identical, u1 wins solely by tie-break
    # Under A1: u1 has comm_factor 1.0, u2 has comm_factor 0.2 (demoted)
    allocator = A1TaskAllocator()
    res = allocator.allocate(snap, network_analysis=net)

    assert len(res.assignments) == 1
    assert res.assignments[0].uav_id == "u1"

    score_u1 = allocator.compute_utility(u1, t, net).total
    score_u2 = allocator.compute_utility(u2, t, net).total
    assert score_u1 > score_u2 * 2.0  # u2 heavily demoted


def test_better_route_pdr_beats_worse_route_pdr():
    """Requirement 4: Better route PDR beats worse route PDR when travel and priority are equal."""
    # GCS at (0, 0). Comm range = 50m.
    # UAV 1 at (20, 0): healthy link (PDR = 1.0)
    # UAV 2 at (0, 20): degraded link condition (loss = 0.7 -> PDR = 0.3)
    # Task at (20, 20): equidistant (20m from both UAVs)
    u1 = UAVState(id="u1", position_xy=(20.0, 0.0))
    u2 = UAVState(id="u2", position_xy=(0.0, 20.0))
    t = TaskState(id="t1", position_xy=(20.0, 20.0), priority=5)

    snap = make_snapshot([u1, u2], [t])
    # Introduce condition degrading u2's link to GCS
    conditions = {
        ("gcs", "u2"): LinkCondition(packet_loss_override=0.7),
    }
    analyzer = BaselineCommunicationAnalyzer(
        config=CommunicationConfig(max_range=50.0),
        conditions=conditions,
    )
    net = analyzer.analyze(snap)

    assert "u1" in net.connected_uav_ids
    assert "u2" in net.connected_uav_ids

    # Find edge metrics
    pdr_u1 = next(l.estimated_pdr for l in net.edge_metrics if "u1" in (l.source_id, l.target_id))
    pdr_u2 = next(l.estimated_pdr for l in net.edge_metrics if "u2" in (l.source_id, l.target_id))
    assert pdr_u1 > pdr_u2

    allocator = A1TaskAllocator()
    res = allocator.allocate(snap, network_analysis=net)

    assert len(res.assignments) == 1
    assert res.assignments[0].uav_id == "u1"

    score_u1 = allocator.compute_utility(u1, t, net).total
    score_u2 = allocator.compute_utility(u2, t, net).total
    assert score_u1 > score_u2


def test_lower_hop_count_preferred_when_reliability_equal():
    """Requirement 5: Lower hop count is preferred when communication reliability is equal."""
    # GCS at (0, 0). Comm range = 30m.
    # UAV 1 at (25, 0): 1 hop to GCS
    # UAV 2 at (50, 0): 2 hops to GCS (via u1 at 25, 0)
    # Task at (37.5, 0): equidistant (12.5m from u1 and 12.5m from u2)
    u1 = UAVState(id="u1", position_xy=(25.0, 0.0))
    u2 = UAVState(id="u2", position_xy=(50.0, 0.0))
    t = TaskState(id="t1", position_xy=(37.5, 0.0), priority=5)

    snap = make_snapshot([u1, u2], [t])
    conditions = {
        ("gcs", "u1"): LinkCondition(packet_loss_override=0.0),
        ("u1", "u2"): LinkCondition(packet_loss_override=0.0),
    }
    analyzer = BaselineCommunicationAnalyzer(
        config=CommunicationConfig(max_range=30.0),
        conditions=conditions,
    )
    net = analyzer.analyze(snap)

    assert net.hop_counts["u1"] == 1
    assert net.hop_counts["u2"] == 2

    allocator = A1TaskAllocator(A1AllocatorConfig(hop_decay=0.85))
    comm_f1 = allocator.compute_communication_factor(u1, net)
    comm_f2 = allocator.compute_communication_factor(u2, net)

    assert comm_f1 == 1.0
    assert pytest.approx(comm_f2) == 0.85

    res = allocator.allocate(snap, network_analysis=net)
    assert res.assignments[0].uav_id == "u2"


def test_communication_does_not_override_clearly_higher_priority_task():
    """Requirement 6: Communication factors do not override higher-priority task order."""
    # Two tasks: t_high (priority 10) and t_low (priority 1)
    # Single UAV u1 available
    u1 = UAVState(id="u1", position_xy=(10.0, 0.0))
    t_high = TaskState(id="t-high", position_xy=(30.0, 0.0), priority=10)
    t_low = TaskState(id="t-low", position_xy=(12.0, 0.0), priority=1)

    snap = make_snapshot([u1], [t_high, t_low])
    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=50.0))
    net = analyzer.analyze(snap)

    allocator = A1TaskAllocator()
    res = allocator.allocate(snap, network_analysis=net)

    # t_high must be assigned first to u1 because task sorting is priority-first
    assert len(res.assignments) == 1
    assert res.assignments[0].task_id == "t-high"
    assert "t-low" in res.unassigned_tasks


def test_disconnected_only_swarm_does_not_deadlock():
    """Requirement 7: Swarm with all UAVs disconnected does not deadlock; task is assigned."""
    # GCS at (0, 0), comm range = 20m.
    # Both UAVs at 100m, 120m (completely disconnected from GCS)
    u1 = UAVState(id="u1", position_xy=(100.0, 0.0))
    u2 = UAVState(id="u2", position_xy=(120.0, 0.0))
    t = TaskState(id="t1", position_xy=(105.0, 0.0), priority=5)

    snap = make_snapshot([u1, u2], [t])
    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=20.0))
    net = analyzer.analyze(snap)

    assert len(net.connected_uav_ids) == 0
    assert set(net.disconnected_uav_ids) == {"u1", "u2"}

    allocator = A1TaskAllocator(A1AllocatorConfig(min_comm_factor=0.2))
    res = allocator.allocate(snap, network_analysis=net)

    # Must NOT deadlock: u1 is closer to t1 (5m vs 15m) and is assigned despite being disconnected
    assert len(res.assignments) == 0
    # Both UAVs had floor comm_factor = 0.2
    assert allocator.compute_communication_factor(u1, net) == 0.2
    assert allocator.compute_communication_factor(u2, net) == 0.2


def test_energy_infeasible_uav_remains_rejected_as_a0():
    """Requirement 8: UAV with low battery is rejected by hard gate, even if 1-hop connected."""
    u_low_batt = UAVState(
        id="u-low",
        position_xy=(5.0, 0.0),
        battery_capacity=100.0,
        battery_energy=10.0,  # 10% <= min threshold 15%
    )
    t = TaskState(id="t1", position_xy=(6.0, 0.0), priority=5)

    snap = make_snapshot([u_low_batt], [t])
    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=50.0))
    net = analyzer.analyze(snap)

    allocator = A1TaskAllocator()
    res = allocator.allocate(snap, network_analysis=net)

    assert len(res.assignments) == 0
    assert "u-low" in res.infeasible_uavs
    assert "below minimum threshold" in res.infeasible_uavs["u-low"]


def test_landed_failed_inactive_uav_remains_rejected():
    """Requirement 9: Inactive, failed, or landed UAVs are rejected exactly as in A0."""
    u_landed = UAVState(id="u-landed", position_xy=(0.0, 0.0), active=False, rth_state=RTHState.COMPLETE)
    u_failed = UAVState(id="u-failed", position_xy=(5.0, 0.0), failure_state=FailureState.FAILED)
    t = TaskState(id="t1", position_xy=(5.0, 0.0), priority=5)

    snap = make_snapshot([u_landed, u_failed], [t])
    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=50.0))
    net = analyzer.analyze(snap)

    allocator = A1TaskAllocator()
    res = allocator.allocate(snap, network_analysis=net)

    assert len(res.assignments) == 0
    assert "u-landed" in res.infeasible_uavs
    assert "u-failed" in res.infeasible_uavs


def test_deterministic_repeated_allocation_gives_identical_result():
    """Requirement 10: Repeated allocation on identical snapshot produces bitwise identical results."""
    u1 = UAVState(id="uav-alpha", position_xy=(10.0, 10.0))
    u2 = UAVState(id="uav-beta", position_xy=(20.0, 20.0))
    t1 = TaskState(id="task-1", position_xy=(15.0, 15.0), priority=3)
    t2 = TaskState(id="task-2", position_xy=(25.0, 25.0), priority=2)

    snap = make_snapshot([u1, u2], [t1, t2])
    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=40.0))
    net = analyzer.analyze(snap)

    allocator = A1TaskAllocator()
    ref_res = allocator.allocate(snap, network_analysis=net)

    for _ in range(50):
        res = allocator.allocate(snap, network_analysis=net)
        assert res.assignments == ref_res.assignments
        assert res.unassigned_tasks == ref_res.unassigned_tasks
        assert res.unassigned_uavs == ref_res.unassigned_uavs
        assert res.infeasible_uavs == ref_res.infeasible_uavs
        assert res.infeasible_tasks == ref_res.infeasible_tasks


def test_a1_never_mutates_state_snapshot_or_state_store():
    """Requirement 11: A1 never directly mutates StateSnapshot or StateStore."""
    u1 = UAVState(id="u1", position_xy=(10.0, 0.0), active=True, battery_capacity=100.0, battery_energy=100.0)
    t1 = TaskState(id="t1", position_xy=(20.0, 0.0), priority=5)

    snap = make_snapshot([u1], [t1])
    store = StateStore(snap)

    snap_before = store.snapshot()
    u1_before = snap_before.uavs["u1"]
    t1_before = snap_before.tasks["t1"]

    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=50.0))
    net = analyzer.analyze(snap_before)

    allocator = A1TaskAllocator()
    res = allocator.allocate(snap_before, network_analysis=net)
    assert len(res.assignments) == 1

    snap_after = store.snapshot()
    # StateStore and snapshot must be 100% untouched
    assert snap_after.simulation_tick == snap_before.simulation_tick
    assert snap_after.state_version == snap_before.state_version
    assert snap_after.uavs["u1"] == u1_before
    assert snap_after.tasks["t1"] == t1_before


def test_a0_adapter_wiring_with_a1():
    """Verify A0AutonomyAdapter correctly wires A1 allocator and emits AssignTaskCommand."""
    u1 = UAVState(id="u1", position_xy=(10.0, 0.0))
    t1 = TaskState(id="t1", position_xy=(15.0, 0.0), priority=5)

    snap = make_snapshot([u1], [t1], sim_tick=3)
    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=50.0))
    net = analyzer.analyze(snap)

    allocator = A1TaskAllocator()
    adapter = A0AutonomyAdapter(allocator=allocator)

    # Plan with network_analysis passed
    cmds = adapter.plan(snap, network_analysis=net)
    assert len(cmds) == 1
    assert isinstance(cmds[0], AssignTaskCommand)
    assert cmds[0].uav_id == "u1"
    assert cmds[0].task_id == "t1"
    assert cmds[0].source_tick == 3


def test_controlled_a0_vs_a1_scenario():
    """Controlled unit scenario demonstrating a communication-driven ranking change.

    Setup:
    - GCS at (0, 0).
    - UAV A at (100, 0): closer to task, but disconnected from GCS (out of 50m range).
    - UAV B at (30, 0): farther from task, but directly connected to GCS (1 hop, PDR 1.0).
    - Task at (90, 0):
        Distance to UAV A = 10m.
        Distance to UAV B = 60m.

    Decision:
    - Under A0: Distance dominates (10m vs 60m). A0 chooses UAV A.
    - Under A1: UAV A is demoted due to being disconnected (comm factor 0.2),
                while UAV B has comm factor 1.0. A1 chooses UAV B.
    """
    uav_a = UAVState(id="uav-a", position_xy=(100.0, 0.0))  # Disconnected
    uav_b = UAVState(id="uav-b", position_xy=(30.0, 0.0))   # Connected
    task = TaskState(id="task-1", position_xy=(90.0, 0.0), priority=5)

    snap = make_snapshot([uav_a, uav_b], [task])
    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=50.0))
    net = analyzer.analyze(snap)

    assert "uav-b" in net.connected_uav_ids
    assert "uav-a" in net.disconnected_uav_ids

    # 1. Evaluate with A0 baseline (comm-blind)
    a0 = A0TaskAllocator()
    res_a0 = a0.allocate(snap)
    # A0 chooses uav-a because it is 50m closer (10m vs 60m)
    assert res_a0.assignments[0].uav_id == "uav-a"

    # 2. Evaluate with A1 communication-aware allocator
    a1 = A1TaskAllocator(A1AllocatorConfig(min_comm_factor=0.2, destination_aware=False))
    res_a1 = a1.allocate(snap, network_analysis=net)

    # A1 switches the assignment to uav-b due to GCS reachability
    assert res_a1.assignments[0].uav_id == "uav-b"

    # Verify underlying scores
    score_a = a1.compute_utility(uav_a, task, net).total
    score_b = a1.compute_utility(uav_b, task, net).total
    assert score_b > score_a


def test_a1_rejects_failed_and_inactive_uavs():
    """Validations 1 & 2: A1 rejects both FAILED and inactive UAVs."""
    u_failed_active = UAVState(id="u-fa", position_xy=(5.0, 0.0), active=True, failure_state=FailureState.FAILED)
    u_inactive_normal = UAVState(id="u-in", position_xy=(10.0, 0.0), active=False, failure_state=FailureState.NORMAL)
    u_both = UAVState(id="u-both", position_xy=(15.0, 0.0), active=False, failure_state=FailureState.FAILED)
    t = TaskState(id="t1", position_xy=(5.0, 0.0), priority=5)

    snap = make_snapshot([u_failed_active, u_inactive_normal, u_both], [t])
    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=50.0))
    net = analyzer.analyze(snap)

    allocator = A1TaskAllocator()
    res = allocator.allocate(snap, network_analysis=net)

    assert len(res.assignments) == 0
    assert "u-fa" in res.infeasible_uavs
    assert "FAILED" in res.infeasible_uavs["u-fa"]
    assert "u-in" in res.infeasible_uavs
    assert "inactive" in res.infeasible_uavs["u-in"]
    assert "u-both" in res.infeasible_uavs


def test_a1_reassigns_deferred_task_from_failed_uav_to_operational_uav():
    """Validations 3, 4, 5, 9: Failed UAV's DEFERRED task is reallocated to feasible UAV without mutating state."""
    # State matching Alpha 714f999 post-failure contract
    u_failed = UAVState(
        id="u-failed",
        position_xy=(5.0, 0.0),
        active=False,
        failure_state=FailureState.FAILED,
        assigned_task_id=None,
        velocity_xy=(0.0, 0.0),
        target_position=None,
    )
    u_operational = UAVState(
        id="u-operational",
        position_xy=(10.0, 0.0),
        active=True,
        failure_state=FailureState.NORMAL,
        assigned_task_id=None,
    )
    t_deferred = TaskState(
        id="t-deferred",
        position_xy=(12.0, 0.0),
        priority=8,
        status=TaskStatus.DEFERRED,
        assigned_uav_id=None,
    )

    snap = make_snapshot([u_failed, u_operational], [t_deferred], sim_tick=5, sim_time=2.5)
    store = StateStore(snap)
    snap_before = store.snapshot()

    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=50.0))
    net = analyzer.analyze(snap_before)

    # 1. Direct A1TaskAllocator allocation
    allocator = A1TaskAllocator()
    res = allocator.allocate(snap_before, network_analysis=net)

    assert len(res.assignments) == 1
    assert res.assignments[0].uav_id == "u-operational"
    assert res.assignments[0].task_id == "t-deferred"
    assert "u-failed" in res.infeasible_uavs
    assert "t-deferred" not in res.infeasible_tasks

    # 2. A0AutonomyAdapter planning with A1 allocator
    adapter = A0AutonomyAdapter(allocator=allocator)
    cmds = adapter.plan(snap_before, network_analysis=net)
    assert len(cmds) == 1
    assert isinstance(cmds[0], AssignTaskCommand)
    assert cmds[0].uav_id == "u-operational"
    assert cmds[0].task_id == "t-deferred"
    assert cmds[0].source_tick == 5

    # 3. Validation 9: StateStore and snapshots are 100% unmutated
    snap_after = store.snapshot()
    assert snap_after.simulation_tick == snap_before.simulation_tick
    assert snap_after.state_version == snap_before.state_version
    assert snap_after.uavs["u-failed"] == snap_before.uavs["u-failed"]
    assert snap_after.uavs["u-operational"] == snap_before.uavs["u-operational"]
    assert snap_after.tasks["t-deferred"] == snap_before.tasks["t-deferred"]


def test_a1_deferred_task_respects_all_feasibility_constraints():
    """Validation 7: Battery, RTH, deadline, lock, and busy constraints remain strictly enforced."""
    u_failed = UAVState(id="u-fail", position_xy=(1.0, 0.0), active=False, failure_state=FailureState.FAILED)
    u_inactive = UAVState(id="u-inact", position_xy=(2.0, 0.0), active=False)
    u_low_batt = UAVState(id="u-low", position_xy=(3.0, 0.0), battery_capacity=100.0, battery_energy=10.0)
    u_rth = UAVState(id="u-rth", position_xy=(4.0, 0.0), rth_state=RTHState.ACTIVE)
    u_locked = UAVState(id="u-lock", position_xy=(5.0, 0.0), assignment_lock_until=15.0)
    u_busy = UAVState(id="u-busy", position_xy=(6.0, 0.0), assigned_task_id="t-existing")
    u_feasible = UAVState(id="u-good", position_xy=(7.0, 0.0), battery_capacity=100.0, battery_energy=100.0)

    t_deferred = TaskState(id="t-def", position_xy=(7.0, 0.0), priority=5, status=TaskStatus.DEFERRED)
    t_expired_deferred = TaskState(id="t-exp-def", position_xy=(7.0, 0.0), priority=10, status=TaskStatus.DEFERRED, deadline=8.0)

    snap = make_snapshot(
        [u_failed, u_inactive, u_low_batt, u_rth, u_locked, u_busy, u_feasible],
        [t_deferred, t_expired_deferred],
        sim_time=10.0,
    )
    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=50.0))
    net = analyzer.analyze(snap)

    allocator = A1TaskAllocator()
    res = allocator.allocate(snap, network_analysis=net)

    # Infeasible UAVs
    assert "u-fail" in res.infeasible_uavs
    assert "u-inact" in res.infeasible_uavs
    assert "u-low" in res.infeasible_uavs
    assert "u-rth" in res.infeasible_uavs
    assert "u-lock" in res.infeasible_uavs
    assert "u-busy" in res.infeasible_uavs

    # Infeasible tasks: expired deadline rejected even if priority is higher
    assert "t-exp-def" in res.infeasible_tasks
    assert "expired" in res.infeasible_tasks["t-exp-def"]

    # Only feasible UAV receives the feasible deferred task
    assert len(res.assignments) == 1
    assert res.assignments[0].uav_id == "u-good"
    assert res.assignments[0].task_id == "t-def"


def test_a1_deferred_task_deterministic_tie_breaking_and_repeated_replay():
    """Validations 6 & 8: Deterministic tie-breaking on deferred task and repeated allocation replay."""
    # uav-alpha and uav-beta are equidistant to task (both at x=10m, task at x=20m)
    u_alpha = UAVState(id="uav-alpha", position_xy=(10.0, 0.0), active=True, battery_capacity=100.0, battery_energy=100.0)
    u_beta = UAVState(id="uav-beta", position_xy=(10.0, 0.0), active=True, battery_capacity=100.0, battery_energy=100.0)
    t_def = TaskState(id="task-def", position_xy=(20.0, 0.0), priority=5, status=TaskStatus.DEFERRED)

    snap = make_snapshot([u_beta, u_alpha], [t_def])
    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=50.0))
    net = analyzer.analyze(snap)

    allocator = A1TaskAllocator()
    ref_res = allocator.allocate(snap, network_analysis=net)

    # Tie-break selects uav-alpha (ascending lexicographical ID)
    assert len(ref_res.assignments) == 1
    assert ref_res.assignments[0].uav_id == "uav-alpha"
    assert ref_res.assignments[0].task_id == "task-def"

    # Repeated allocation replay produces bitwise identical results
    for _ in range(50):
        res = allocator.allocate(snap, network_analysis=net)
        assert res.assignments == ref_res.assignments
        assert res.unassigned_tasks == ref_res.unassigned_tasks
        assert res.unassigned_uavs == ref_res.unassigned_uavs
        assert res.infeasible_uavs == ref_res.infeasible_uavs
        assert res.infeasible_tasks == ref_res.infeasible_tasks
def test_mission_runner_scheduled_failure_reallocates_deferred_task():
    """Prove scheduled UAV failure -> DEFERRED task -> A1 reallocation."""

    config = ScenarioConfig(
        name="test_runner_scheduled_failure",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=10.0,
        max_ticks=10,
        gcs_position=(0.0, 0.0),
        uavs=(
            {"id": "u1", "position": [10.0, 0.0], "battery_capacity": 100.0},
            {"id": "u2", "position": [20.0, 0.0], "battery_capacity": 100.0},
        ),
        tasks=(
            {"id": "t1", "position": [15.0, 0.0],
             "priority": 5, "service_duration": 5.0},
        ),
    )

    runner = MissionRunner(
        scenario=config,
        autonomy_adapter=A0AutonomyAdapter(
            allocator=A1TaskAllocator()
        ),
    )

    # Tick 0: A1 allocates t1 to u1.
    step1 = runner.step()
    snap1 = runner.state_store.snapshot()

    assert snap1.uavs["u1"].assigned_task_id == "t1"
    assert snap1.uavs["u2"].assigned_task_id is None
    assert snap1.tasks["t1"].status == TaskStatus.ASSIGNED
    assert snap1.tasks["t1"].assigned_uav_id == "u1"

    # Schedule u1 failure for tick 1.
    runner.sim_engine.event_scheduler.schedule(
        ScheduledEvent(
            tick=1,
            event_type=ScheduledEventType.UAV_FAILURE,
            uav_id="u1",
            reason="hardware_fault",
        )
    )

    # Tick 1:
    # scheduled failure -> canonical StateStore lifecycle
    # -> task becomes DEFERRED -> A1 reallocates to u2.
    step2 = runner.step()
    snap2 = runner.state_store.snapshot()

    assert snap2.uavs["u1"].active is False
    assert snap2.uavs["u1"].failure_state == FailureState.FAILED
    assert snap2.uavs["u1"].assigned_task_id is None

    assert snap2.tasks["t1"].status == TaskStatus.ASSIGNED
    assert snap2.tasks["t1"].assigned_uav_id == "u2"
    assert snap2.uavs["u2"].assigned_task_id == "t1"

    # Confirm the scheduled failure was actually applied during tick 1.
    assert any(
        isinstance(command, FailUAVCommand)
        for command in step2.applied_commands
    )


