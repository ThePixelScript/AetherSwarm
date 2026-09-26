"""Focused regression test suite for atomic task-chain formation and synchronized group movement.

Covers the 10 authoritative scenarios:
1. test_atomic_chain_one_surveyor_one_relay
   - 1 surveyor + 1 relay assigned in same tick with synchronized targets.
2. test_atomic_chain_one_surveyor_multiple_relays
   - 1 surveyor + multiple relays (all K assigned in same tick atomically).
3. test_atomic_chain_shared_trunk
   - Shared-trunk tasks with reference counting in relay_dependent_surveyors.
4. test_atomic_chain_two_independent_tasks_concurrent
   - Two independent tasks concurrently allocating disjoint fleets.
5. test_atomic_chain_relay_unavailable_aborts_entire_allocation
   - Relay becoming unavailable during planning -> clean deferral, 0 dispatched.
6. test_atomic_chain_repeated_planner_ticks_while_forming
   - Repeated planner ticks while forming -> no reassignment, no target spamming.
7. test_atomic_chain_task_completion_and_teardown
   - Task completion and relay teardown -> clean release of non-shared relays.
8. test_atomic_chain_rth_recharge_interaction
   - RTH/recharge candidates rejected for new chains.
9. test_atomic_chain_active_surveyor_protection
   - Active surveyor is never drafted as a relay candidate.
10. test_atomic_chain_no_partial_chain_movement_regression
   - Partial chain is never formed; 0 movement commands emitted.
"""
from __future__ import annotations

import math
from typing import Optional

import pytest

from ares_swarm.autonomy.connectivity_planner import (
    ConnectivityAwarePlanner,
    ConnectivityAwarePlannerConfig,
    compute_multihop_stations,
)
from ares_swarm.autonomy.relay_manager import (
    ChainStatus,
    DynamicRelayManager,
    RelayChain,
)
from ares_swarm.core.commands import (
    AssignRelayRoleCommand,
    AssignTaskCommand,
    ReleaseRelayRoleCommand,
    SetTargetPositionCommand,
)
from ares_swarm.core.enums import (
    FailureState,
    Role,
    RTHState,
    SortieState,
    TaskStatus,
)
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState


def make_test_uav(
    uav_id: str,
    position: tuple[float, float] = (-75.0, 500.0),
    role: Role = Role.IDLE,
    battery: float = 1000.0,
    active: bool = True,
    failure_state: FailureState = FailureState.NORMAL,
    rth_state: RTHState = RTHState.NONE,
    sortie_state: SortieState = SortieState.ACTIVE,
    assigned_task_id: Optional[str] = None,
    target_position: Optional[tuple[float, float]] = None,
) -> UAVState:
    return UAVState(
        id=uav_id,
        position_xy=position,
        velocity_xy=(0.0, 0.0),
        battery_energy=battery,
        battery_capacity=1000.0,
        role=role,
        active=active,
        failure_state=failure_state,
        rth_state=rth_state,
        sortie_state=sortie_state,
        assigned_task_id=assigned_task_id,
        target_position=target_position,
    )


def test_atomic_chain_one_surveyor_one_relay():
    """Scenario 1: 1 surveyor + 1 relay assigned in same tick."""
    rm = DynamicRelayManager()
    planner = ConnectivityAwarePlanner(relay_manager=rm, comm_range=100.0)

    # Task at 175m from GCS (-75.0, 500.0) -> requires 1 relay
    task = TaskState(
        id="task_1",
        position_xy=(100.0, 500.0),
        priority=1,
        service_duration=10.0,
        status=TaskStatus.PENDING,
    )
    uavs = {
        "uav_1": make_test_uav("uav_1", position=(-75.0, 500.0)),
        "uav_2": make_test_uav("uav_2", position=(-75.0, 500.0)),
    }
    snapshot = StateSnapshot(
        uavs=uavs,
        tasks={"task_1": task},
        gcs_position=(-75.0, 500.0),
        simulation_tick=10,
        simulation_time=10.0,
        state_version=1,
    )

    cmds = planner.plan(snapshot)

    task_cmds = [c for c in cmds if isinstance(c, AssignTaskCommand)]
    relay_cmds = [c for c in cmds if isinstance(c, AssignRelayRoleCommand)]
    target_cmds = [c for c in cmds if isinstance(c, SetTargetPositionCommand)]

    # Exactly 1 surveyor and 1 relay assigned in the same tick
    assert len(task_cmds) == 1
    assert len(relay_cmds) == 1
    surv_id = task_cmds[0].uav_id
    relay_id = relay_cmds[0].uav_id
    assert surv_id != relay_id
    assert {surv_id, relay_id} == {"uav_1", "uav_2"}

    # Both UAVs receive target position commands in this same tick
    target_uav_ids = {c.uav_id for c in target_cmds}
    assert surv_id in target_uav_ids
    assert relay_id in target_uav_ids

    # Authoritative RelayChain is registered with FORMING status
    chain = rm.chains.get("chain_task_1")
    assert chain is not None
    assert chain.status == ChainStatus.FORMING
    assert chain.surveyor_id == surv_id
    assert chain.relay_ids == [relay_id]
    assert len(chain.station_positions) == 1


def test_atomic_chain_one_surveyor_multiple_relays():
    """Scenario 2: 1 surveyor + multiple relays (all K assigned in same tick)."""
    rm = DynamicRelayManager()
    planner = ConnectivityAwarePlanner(relay_manager=rm, comm_range=100.0)

    # Task at 255m from GCS -> requires K=2 relays
    task = TaskState(
        id="task_multi",
        position_xy=(180.0, 500.0),
        priority=1,
        service_duration=10.0,
        status=TaskStatus.PENDING,
    )
    uavs = {
        "uav_1": make_test_uav("uav_1", position=(-75.0, 500.0)),
        "uav_2": make_test_uav("uav_2", position=(-75.0, 500.0)),
        "uav_3": make_test_uav("uav_3", position=(-75.0, 500.0)),
    }
    snapshot = StateSnapshot(
        uavs=uavs,
        tasks={"task_multi": task},
        gcs_position=(-75.0, 500.0),
        simulation_tick=5,
        simulation_time=5.0,
        state_version=1,
    )

    cmds = planner.plan(snapshot)

    task_cmds = [c for c in cmds if isinstance(c, AssignTaskCommand)]
    relay_cmds = [c for c in cmds if isinstance(c, AssignRelayRoleCommand)]
    target_cmds = [c for c in cmds if isinstance(c, SetTargetPositionCommand)]

    # 1 surveyor + 2 relays assigned atomically in one planning tick
    assert len(task_cmds) == 1
    assert len(relay_cmds) == 2
    assigned_uavs = {task_cmds[0].uav_id} | {c.uav_id for c in relay_cmds}
    assert len(assigned_uavs) == 3
    assert assigned_uavs == {"uav_1", "uav_2", "uav_3"}

    # All 3 UAVs receive movement commands in the same tick
    target_uav_ids = {c.uav_id for c in target_cmds}
    assert assigned_uavs == target_uav_ids

    chain = rm.chains.get("chain_task_multi")
    assert chain is not None
    assert chain.status == ChainStatus.FORMING
    assert len(chain.relay_ids) == 2
    assert len(chain.station_positions) == 2


def test_atomic_chain_shared_trunk():
    """Scenario 3: Shared-trunk tasks with reference counting in relay_dependent_surveyors."""
    rm = DynamicRelayManager()

    # Chain A: Trunk (r1) -> Branch (r2) -> Surveyor A
    chain_a = rm.register_chain(
        chain_id="chain_a",
        surveyor_id="surv_a",
        relay_ids=["r1", "r2"],
        station_positions=[(0.0, 500.0), (100.0, 500.0)],
        task_id="task_a",
    )
    # Chain B: Trunk (r1) -> Branch (r3) -> Surveyor B
    chain_b = rm.register_chain(
        chain_id="chain_b",
        surveyor_id="surv_b",
        relay_ids=["r1", "r3"],
        station_positions=[(0.0, 500.0), (100.0, 600.0)],
        task_id="task_b",
    )

    # Reference counting: r1 shared by both, r2 dedicated to A, r3 dedicated to B
    assert rm.relay_dependent_surveyors["r1"] == {"surv_a", "surv_b"}
    assert rm.relay_dependent_surveyors["r2"] == {"surv_a"}
    assert rm.relay_dependent_surveyors["r3"] == {"surv_b"}

    # Teardown Chain A: only r2 should be released; r1 must remain active for Surveyor B
    cmds_a = rm.teardown_chain("chain_a")
    released_a = [c.uav_id for c in cmds_a if isinstance(c, ReleaseRelayRoleCommand)]
    assert "r2" in released_a
    assert "r1" not in released_a
    assert rm.relay_dependent_surveyors["r1"] == {"surv_b"}

    # Teardown Chain B: r1 and r3 should now be released
    cmds_b = rm.teardown_chain("chain_b")
    released_b = [c.uav_id for c in cmds_b if isinstance(c, ReleaseRelayRoleCommand)]
    assert "r1" in released_b
    assert "r3" in released_b
    assert len(rm.relay_dependent_surveyors) == 0


def test_atomic_chain_two_independent_tasks_concurrent():
    """Scenario 4: Two independent tasks concurrently allocating disjoint fleets."""
    rm = DynamicRelayManager()
    planner = ConnectivityAwarePlanner(relay_manager=rm, comm_range=100.0)

    # Two tasks, each requiring 1 surveyor + 1 relay
    task_1 = TaskState(id="task_1", position_xy=(100.0, 450.0), priority=2, service_duration=10.0, status=TaskStatus.PENDING)
    task_2 = TaskState(id="task_2", position_xy=(100.0, 550.0), priority=1, service_duration=10.0, status=TaskStatus.PENDING)

    uavs = {
        f"uav_{i}": make_test_uav(f"uav_{i}", position=(-75.0, 500.0))
        for i in range(1, 5)
    }
    snapshot = StateSnapshot(
        uavs=uavs,
        tasks={"task_1": task_1, "task_2": task_2},
        gcs_position=(-75.0, 500.0),
        simulation_tick=1,
        simulation_time=1.0,
        state_version=1,
    )

    cmds = planner.plan(snapshot)

    task_cmds = [c for c in cmds if isinstance(c, AssignTaskCommand)]
    relay_cmds = [c for c in cmds if isinstance(c, AssignRelayRoleCommand)]

    # Both tasks assigned in this single planning tick
    assert len(task_cmds) == 2
    assert len(relay_cmds) == 2

    assigned_tasks = {c.task_id for c in task_cmds}
    assert assigned_tasks == {"task_1", "task_2"}

    # All 4 assigned UAVs must be completely disjoint
    all_assigned_uavs = [c.uav_id for c in task_cmds] + [c.uav_id for c in relay_cmds]
    assert len(all_assigned_uavs) == 4
    assert len(set(all_assigned_uavs)) == 4


def test_atomic_chain_relay_unavailable_aborts_entire_allocation():
    """Scenario 5: Relay becoming unavailable during planning -> clean deferral, 0 dispatched."""
    rm = DynamicRelayManager()
    planner = ConnectivityAwarePlanner(relay_manager=rm, comm_range=100.0)

    # Task requires 1 surveyor + 1 relay (2 UAVs)
    task = TaskState(id="task_far", position_xy=(100.0, 500.0), priority=1, service_duration=10.0, status=TaskStatus.PENDING)

    # Only 1 UAV available in the entire fleet
    uavs = {"uav_1": make_test_uav("uav_1", position=(-75.0, 500.0))}
    snapshot = StateSnapshot(
        uavs=uavs,
        tasks={"task_far": task},
        gcs_position=(-75.0, 500.0),
        simulation_tick=1,
        simulation_time=1.0,
        state_version=1,
    )

    cmds = planner.plan(snapshot)

    # Clean deferral: 0 UAVs dispatched, 0 commands emitted
    assert len(cmds) == 0
    assert planner.connectivity_deferred_tasks >= 1
    assert planner.tasks_deferred_insufficient_relays >= 1
    assert len(rm.chains) == 0


def test_atomic_chain_repeated_planner_ticks_while_forming():
    """Scenario 6: Repeated planner ticks while forming -> no reassignment, no target spamming."""
    rm = DynamicRelayManager()
    planner = ConnectivityAwarePlanner(relay_manager=rm, comm_range=100.0)

    # Pre-configure an active chain in FORMING status
    station_pos = (12.5, 500.0)
    chain = RelayChain(
        chain_id="chain_t1",
        task_id="t1",
        surveyor_id="uav_1",
        relay_ids=["uav_2"],
        station_positions=[station_pos],
        created_tick=0,
        created_time=0.0,
        status=ChainStatus.FORMING,
    )
    rm.register_chain(chain)

    # Compute expected holding target for surveyor during forming
    dx = 100.0 - station_pos[0]
    dy = 500.0 - station_pos[1]
    d_outer = math.hypot(dx, dy)
    safe_dist = max(0.0, d_outer - 45.0)
    nx, ny = dx / d_outer, dy / d_outer
    expected_holding = (round(station_pos[0] + nx * safe_dist, 2), round(station_pos[1] + ny * safe_dist, 2))

    task = TaskState(id="t1", position_xy=(100.0, 500.0), priority=1, service_duration=10.0, status=TaskStatus.IN_PROGRESS, assigned_uav_id="uav_1")
    uav_1 = make_test_uav("uav_1", position=(-50.0, 500.0), role=Role.SURVEYOR, assigned_task_id="t1", target_position=expected_holding)
    uav_2 = make_test_uav("uav_2", position=(-60.0, 500.0), role=Role.RELAY, target_position=station_pos)

    snapshot = StateSnapshot(
        uavs={"uav_1": uav_1, "uav_2": uav_2},
        tasks={"t1": task},
        gcs_position=(-75.0, 500.0),
        simulation_tick=1,
        simulation_time=1.0,
        state_version=2,
    )

    # plan() should not re-plan the task since it is IN_PROGRESS
    plan_cmds = planner.plan(snapshot)
    assert len(plan_cmds) == 0

    # monitor_active_tasks() should not spam target commands when target hasn't changed
    monitor_cmds = planner.monitor_active_tasks(snapshot)
    assert len(monitor_cmds) == 0
    assert chain.status == ChainStatus.FORMING


def test_atomic_chain_task_completion_and_teardown():
    """Scenario 7: Task completion and relay teardown -> clean release of non-shared relays."""
    rm = DynamicRelayManager()

    chain = RelayChain(
        chain_id="chain_t1",
        task_id="t1",
        surveyor_id="uav_1",
        relay_ids=["uav_2"],
        station_positions=[(12.5, 500.0)],
        created_tick=0,
        created_time=0.0,
        status=ChainStatus.ACTIVE,
    )
    rm.register_chain(chain)

    # Calling teardown_chain (as triggered on task completion)
    cmds = rm.teardown_chain("chain_t1")

    # Dedicated relay uav_2 is cleanly released
    release_cmds = [c for c in cmds if isinstance(c, ReleaseRelayRoleCommand)]
    assert len(release_cmds) == 1
    assert release_cmds[0].uav_id == "uav_2"
    assert release_cmds[0].next_role == Role.IDLE

    # Clean release: uav_2 also receives target command to return toward staging
    assert any(isinstance(c, SetTargetPositionCommand) and c.uav_id == "uav_2" for c in cmds)

    # Chain marked TEARDOWN and removed from tracking
    assert chain.status == ChainStatus.TEARDOWN
    assert "uav_2" not in rm.relay_positions
    assert "uav_1" not in rm.surveyor_to_chain

    # Also verify automatic teardown via step() when surveyor task completes (assigned_task_id is None)
    chain2 = RelayChain(
        chain_id="chain_t2",
        task_id="t2",
        surveyor_id="uav_3",
        relay_ids=["uav_4"],
        station_positions=[(12.5, 500.0)],
        created_tick=0,
        created_time=0.0,
        status=ChainStatus.ACTIVE,
    )
    rm.register_chain(chain2)
    uav_3 = make_test_uav("uav_3", position=(-75.0, 500.0), role=Role.IDLE, assigned_task_id=None)
    uav_4 = make_test_uav("uav_4", position=(12.5, 500.0), role=Role.RELAY)
    snap2 = StateSnapshot(
        uavs={"uav_3": uav_3, "uav_4": uav_4},
        gcs_position=(-75.0, 500.0),
        simulation_tick=21,
        simulation_time=21.0,
        state_version=6,
    )
    step_cmds = rm.step(snap2)
    step_release_cmds = [c for c in step_cmds if isinstance(c, ReleaseRelayRoleCommand) and c.uav_id == "uav_4"]
    assert len(step_release_cmds) == 1
    assert "uav_4" not in rm.relay_positions


def test_atomic_chain_rth_recharge_interaction():
    """Scenario 8: RTH/recharge candidates rejected for new chains."""
    rm = DynamicRelayManager()
    planner = ConnectivityAwarePlanner(relay_manager=rm, comm_range=100.0)

    task = TaskState(id="t_rth", position_xy=(100.0, 500.0), priority=1, service_duration=10.0, status=TaskStatus.PENDING)

    uavs = {
        "uav_1": make_test_uav("uav_1", position=(-75.0, 500.0)),  # IDLE, valid surveyor
        "uav_2": make_test_uav("uav_2", position=(-75.0, 500.0), rth_state=RTHState.ACTIVE, sortie_state=SortieState.RTH),
        "uav_3": make_test_uav("uav_3", position=(-75.0, 500.0), sortie_state=SortieState.RECHARGING),
        "uav_4": make_test_uav("uav_4", position=(-75.0, 500.0), sortie_state=SortieState.LANDED),
        "uav_5": make_test_uav("uav_5", position=(-75.0, 500.0), failure_state=FailureState.FAILED),
    }
    snapshot = StateSnapshot(
        uavs=uavs,
        tasks={"t_rth": task},
        gcs_position=(-75.0, 500.0),
        simulation_tick=1,
        simulation_time=1.0,
        state_version=1,
    )

    cmds = planner.plan(snapshot)

    # Task requires 1 surveyor + 1 relay. Only 1 valid UAV (uav_1) exists.
    # UAVs 2-5 in RTH/recharging/landed/failed state must NOT be selected.
    assert len(cmds) == 0
    assert len(rm.chains) == 0
    assert planner.connectivity_deferred_tasks >= 1


def test_atomic_chain_active_surveyor_protection():
    """Scenario 9: Active surveyor is never drafted as a relay candidate."""
    rm = DynamicRelayManager()
    planner = ConnectivityAwarePlanner(relay_manager=rm, comm_range=100.0)

    # uav_1 is actively servicing task_1
    uav_1 = make_test_uav("uav_1", position=(100.0, 500.0), role=Role.SURVEYOR, assigned_task_id="task_1")
    # uav_2 is idle
    uav_2 = make_test_uav("uav_2", position=(-75.0, 500.0), role=Role.IDLE)

    tasks = {
        "task_1": TaskState(id="task_1", position_xy=(100.0, 500.0), priority=1, service_duration=10.0, status=TaskStatus.IN_PROGRESS, assigned_uav_id="uav_1"),
        "task_2": TaskState(id="task_2", position_xy=(100.0, 600.0), priority=2, service_duration=10.0, status=TaskStatus.PENDING),
    }
    snapshot = StateSnapshot(
        uavs={"uav_1": uav_1, "uav_2": uav_2},
        tasks=tasks,
        gcs_position=(-75.0, 500.0),
        simulation_tick=5,
        simulation_time=5.0,
        state_version=1,
    )

    # Candidate selection directly verifies active surveyor is skipped
    cand_relay = rm.select_relay_candidate(
        snapshot=snapshot,
        target_uav_id="uav_2",
        relay_position=(12.5, 550.0),
    )
    assert cand_relay != "uav_1"

    # Task 2 requires 2 UAVs (1 surveyor + 1 relay). Since uav_1 is protected and cannot be a relay,
    # task_2 must be deferred rather than drafting uav_1.
    cmds = planner.plan(snapshot)
    relay_cmds = [c for c in cmds if isinstance(c, AssignRelayRoleCommand)]
    assert not any(c.uav_id == "uav_1" for c in relay_cmds)


def test_atomic_chain_no_partial_chain_movement_regression():
    """Scenario 10: Partial chain is never formed; 0 movement commands emitted."""
    rm = DynamicRelayManager()
    # Explicitly test that even if allow_partial_chains was configured True,
    # atomic chain enforcement strictly prevents deploying partial chains
    planner = ConnectivityAwarePlanner(relay_manager=rm, comm_range=100.0, allow_partial_chains=True)

    # Distant task requiring 2 relays (K=2) + 1 surveyor = 3 UAVs
    task = TaskState(
        id="task_far",
        position_xy=(180.0, 500.0),
        priority=1,
        service_duration=10.0,
        status=TaskStatus.PENDING,
    )

    # Only 2 UAVs available (1 surveyor candidate + 1 relay candidate, missing 1 relay)
    uavs = {
        "uav_1": make_test_uav("uav_1", position=(-75.0, 500.0)),
        "uav_2": make_test_uav("uav_2", position=(-75.0, 500.0)),
    }
    snapshot = StateSnapshot(
        uavs=uavs,
        tasks={"task_far": task},
        gcs_position=(-75.0, 500.0),
        simulation_tick=0,
        simulation_time=0.0,
        state_version=1,
    )

    # Feasibility check must return False due to atomic requirement
    res = planner.check_task_connectivity_feasibility(task=task, uav=uavs["uav_1"], snapshot=snapshot)
    assert res.feasible is False
    assert "insufficient relays" in res.reason.lower()

    # plan() must emit 0 commands: no partial chain movement, no premature departure
    cmds = planner.plan(snapshot)
    assert len(cmds) == 0
    move_cmds = [c for c in cmds if isinstance(c, SetTargetPositionCommand)]
    assert len(move_cmds) == 0
    assert len(rm.chains) == 0
