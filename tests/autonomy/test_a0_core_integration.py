"""Integration tests for A0 Task Allocator and Alpha Core architecture."""
from __future__ import annotations

from types import MappingProxyType
import pytest

from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.autonomy.task_allocator import A0TaskAllocator
from ares_swarm.core.commands import AssignTaskCommand
from ares_swarm.core.enums import (
    EventType,
    FailureState,
    Role,
    RTHState,
    TaskStatus,
)
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState
from ares_swarm.core.state_store import StateStore


def test_state_snapshot_to_a0_adapter_to_assign_task_command():
    """StateSnapshot -> A0 adapter produces valid AssignTaskCommand with source_tick."""
    uav = UAVState(id="uav-1", position_xy=(0.0, 0.0), battery_capacity=100.0, battery_energy=100.0)
    task = TaskState(id="task-1", position_xy=(10.0, 10.0), priority=5)
    snapshot = StateSnapshot(
        simulation_tick=7,
        simulation_time=14.0,
        state_version=3,
        uavs=MappingProxyType({"uav-1": uav}),
        tasks=MappingProxyType({"task-1": task}),
    )

    adapter = A0AutonomyAdapter(A0TaskAllocator())
    commands = adapter.plan(snapshot)

    assert len(commands) == 1
    cmd = commands[0]
    assert isinstance(cmd, AssignTaskCommand)
    assert cmd.source_tick == snapshot.simulation_tick == 7
    assert cmd.uav_id == "uav-1"
    assert cmd.task_id == "task-1"


def test_multiple_deterministic_assignments_and_state_store_apply():
    """Multiple assignments must be deterministically ordered, applied by StateStore,

    and correctly update entity states, state_version, and emitted domain events.
    """
    u1 = UAVState(id="uav-1", position_xy=(0.0, 0.0))
    u2 = UAVState(id="uav-2", position_xy=(100.0, 100.0))
    u3 = UAVState(id="uav-3", position_xy=(200.0, 200.0))

    t1 = TaskState(id="task-1", position_xy=(5.0, 5.0), priority=3)
    t2 = TaskState(id="task-2", position_xy=(105.0, 105.0), priority=2)
    t3 = TaskState(id="task-3", position_xy=(205.0, 205.0), priority=1)

    snapshot = StateSnapshot(
        simulation_tick=1,
        simulation_time=1.0,
        state_version=1,
        uavs=MappingProxyType({"uav-1": u1, "uav-2": u2, "uav-3": u3}),
        tasks=MappingProxyType({"task-1": t1, "task-2": t2, "task-3": t3}),
    )

    adapter = A0AutonomyAdapter(A0TaskAllocator())
    commands = adapter.plan(snapshot)

    assert len(commands) == 3
    # Deterministic UAV ordering
    assert [c.uav_id for c in commands] == ["uav-1", "uav-2", "uav-3"]
    assert [c.task_id for c in commands] == ["task-1", "task-2", "task-3"]

    store = StateStore(snapshot)
    res = store.apply(commands)

    assert len(res.applied_commands) == 3
    assert len(res.rejected_commands) == 0
    assert res.previous_version == 1
    assert res.new_version == 2
    assert len(res.emitted_events) == 3

    for event in res.emitted_events:
        assert event.event_type == EventType.TASK_ASSIGNED
        assert event.simulation_tick == 1

    updated = store.snapshot()
    assert updated.uavs["uav-1"].assigned_task_id == "task-1"
    assert updated.uavs["uav-1"].target_position == (5.0, 5.0)
    assert updated.tasks["task-1"].status == TaskStatus.ASSIGNED
    assert updated.tasks["task-1"].assigned_uav_id == "uav-1"

    assert updated.uavs["uav-2"].assigned_task_id == "task-2"
    assert updated.uavs["uav-2"].target_position == (105.0, 105.0)
    assert updated.tasks["task-2"].status == TaskStatus.ASSIGNED
    assert updated.tasks["task-2"].assigned_uav_id == "uav-2"

    assert updated.uavs["uav-3"].assigned_task_id == "task-3"
    assert updated.uavs["uav-3"].target_position == (205.0, 205.0)
    assert updated.tasks["task-3"].status == TaskStatus.ASSIGNED
    assert updated.tasks["task-3"].assigned_uav_id == "uav-3"


def test_low_battery_rejected():
    """UAV with battery_percent below threshold must be rejected."""
    # battery_percent = (1000 / 10000) * 100 = 10% <= min threshold 15%
    uav_low = UAVState(
        id="uav-low",
        position_xy=(0.0, 0.0),
        battery_capacity=10000.0,
        battery_energy=1000.0,
    )
    task = TaskState(id="task-1", position_xy=(10.0, 10.0), priority=5)
    snapshot = StateSnapshot(
        simulation_tick=1,
        simulation_time=0.0,
        state_version=1,
        uavs=MappingProxyType({"uav-low": uav_low}),
        tasks=MappingProxyType({"task-1": task}),
    )

    allocator = A0TaskAllocator()
    adapter = A0AutonomyAdapter(allocator)

    commands = adapter.plan(snapshot)
    assert len(commands) == 0

    alloc_result = allocator.allocate(snapshot)
    assert len(alloc_result.assignments) == 0
    assert "uav-low" in alloc_result.infeasible_uavs
    assert "below minimum threshold" in alloc_result.infeasible_uavs["uav-low"]


def test_failed_uav_rejected():
    """UAV with failure_state == FailureState.FAILED must be rejected."""
    uav_failed = UAVState(
        id="uav-failed",
        position_xy=(0.0, 0.0),
        failure_state=FailureState.FAILED,
    )
    task = TaskState(id="task-1", position_xy=(10.0, 10.0), priority=5)
    snapshot = StateSnapshot(
        simulation_tick=1,
        simulation_time=0.0,
        state_version=1,
        uavs=MappingProxyType({"uav-failed": uav_failed}),
        tasks=MappingProxyType({"task-1": task}),
    )

    allocator = A0TaskAllocator()
    adapter = A0AutonomyAdapter(allocator)

    commands = adapter.plan(snapshot)
    assert len(commands) == 0

    alloc_result = allocator.allocate(snapshot)
    assert len(alloc_result.assignments) == 0
    assert "uav-failed" in alloc_result.infeasible_uavs
    assert "FAILED" in alloc_result.infeasible_uavs["uav-failed"]


def test_deadline_handling():
    """Default deadline=0.0 remains eligible; expired positive deadline is rejected."""
    uav = UAVState(id="uav-1", position_xy=(0.0, 0.0))

    # task with default deadline 0.0 (unconstrained)
    task_default = TaskState(id="task-default", position_xy=(10.0, 0.0), priority=2, deadline=0.0)
    # task with expired positive deadline (deadline 5.0 <= sim_time 10.0)
    task_expired = TaskState(id="task-expired", position_xy=(10.0, 0.0), priority=10, deadline=5.0)
    # task with unexpired positive deadline (deadline 50.0 > sim_time 10.0)
    task_future = TaskState(id="task-future", position_xy=(10.0, 0.0), priority=5, deadline=50.0)

    snapshot = StateSnapshot(
        simulation_tick=5,
        simulation_time=10.0,
        state_version=1,
        uavs=MappingProxyType({"uav-1": uav}),
        tasks=MappingProxyType({
            "task-default": task_default,
            "task-expired": task_expired,
            "task-future": task_future,
        }),
    )

    allocator = A0TaskAllocator()
    adapter = A0AutonomyAdapter(allocator)

    alloc_result = allocator.allocate(snapshot)
    assert "task-expired" in alloc_result.infeasible_tasks
    assert "expired" in alloc_result.infeasible_tasks["task-expired"]
    assert "task-default" not in alloc_result.infeasible_tasks
    assert "task-future" not in alloc_result.infeasible_tasks

    # Between task-future (priority 5) and task-default (priority 2), task-future is selected
    commands = adapter.plan(snapshot)
    assert len(commands) == 1
    assert commands[0].task_id == "task-future"

    # Now verify task-default (deadline=0.0) is allocated when alone
    snap_default_only = StateSnapshot(
        simulation_tick=5,
        simulation_time=10.0,
        state_version=1,
        uavs=MappingProxyType({"uav-1": uav}),
        tasks=MappingProxyType({"task-default": task_default}),
    )
    cmds_default = adapter.plan(snap_default_only)
    assert len(cmds_default) == 1
    assert cmds_default[0].task_id == "task-default"


def test_nonzero_simulation_time_reaches_allocator():
    """Non-zero simulation_time must be propagated from StateSnapshot to allocate()."""
    # UAV has cooldown_until=20.0
    uav_cooldown = UAVState(id="uav-1", position_xy=(0.0, 0.0), cooldown_until=20.0)
    task = TaskState(id="task-1", position_xy=(10.0, 0.0), priority=1)

    adapter = A0AutonomyAdapter(A0TaskAllocator())

    # At t=15.0, cooldown is active -> no assignment
    snap_t15 = StateSnapshot(
        simulation_tick=30,
        simulation_time=15.0,
        state_version=1,
        uavs=MappingProxyType({"uav-1": uav_cooldown}),
        tasks=MappingProxyType({"task-1": task}),
    )
    cmds_t15 = adapter.plan(snap_t15)
    assert len(cmds_t15) == 0

    # At t=25.0, cooldown has elapsed -> assignment succeeds with correct tick
    snap_t25 = StateSnapshot(
        simulation_tick=50,
        simulation_time=25.0,
        state_version=2,
        uavs=MappingProxyType({"uav-1": uav_cooldown}),
        tasks=MappingProxyType({"task-1": task}),
    )
    cmds_t25 = adapter.plan(snap_t25)
    assert len(cmds_t25) == 1
    assert cmds_t25[0].uav_id == "uav-1"
    assert cmds_t25[0].task_id == "task-1"
    assert cmds_t25[0].source_tick == 50


def test_repeated_identical_snapshot_produces_identical_commands():
    """A0 must produce identical command output across repeated evaluations on the same snapshot."""
    uav1 = UAVState(id="uav-a", position_xy=(0.0, 10.0))
    uav2 = UAVState(id="uav-b", position_xy=(10.0, 0.0))
    t1 = TaskState(id="task-x", position_xy=(5.0, 5.0), priority=2)
    t2 = TaskState(id="task-y", position_xy=(15.0, 15.0), priority=2)

    snapshot = StateSnapshot(
        simulation_tick=12,
        simulation_time=6.0,
        state_version=4,
        uavs=MappingProxyType({"uav-a": uav1, "uav-b": uav2}),
        tasks=MappingProxyType({"task-x": t1, "task-y": t2}),
    )

    adapter = A0AutonomyAdapter(A0TaskAllocator())
    reference_cmds = adapter.plan(snapshot)
    assert len(reference_cmds) == 2

    for _ in range(10):
        cmds = adapter.plan(snapshot)
        assert cmds == reference_cmds


def test_emergency_flag_precedence_with_core_models():
    """Emergency task takes precedence over non-emergency task of same priority."""
    uav = UAVState(id="uav-1", position_xy=(0.0, 0.0))
    task_normal = TaskState(id="task-norm", position_xy=(10.0, 0.0), priority=5, emergency_flag=False)
    task_emer = TaskState(id="task-emer", position_xy=(20.0, 0.0), priority=5, emergency_flag=True)

    snapshot = StateSnapshot(
        simulation_tick=1,
        simulation_time=0.0,
        state_version=1,
        uavs=MappingProxyType({"uav-1": uav}),
        tasks=MappingProxyType({"task-norm": task_normal, "task-emer": task_emer}),
    )

    adapter = A0AutonomyAdapter(A0TaskAllocator())
    cmds = adapter.plan(snapshot)
    assert len(cmds) == 1
    assert cmds[0].task_id == "task-emer"


def test_rth_and_terminal_task_states_rejected():
    """UAVs in RTH and tasks in terminal states (COMPLETE, UNREACHABLE) must not be assigned."""
    uav_rth_active = UAVState(id="u-rth-act", position_xy=(0.0, 0.0), rth_state=RTHState.ACTIVE)
    uav_rth_req = UAVState(id="u-rth-req", position_xy=(0.0, 0.0), rth_state=RTHState.REQUIRED)
    uav_rth_comp = UAVState(id="u-rth-comp", position_xy=(0.0, 0.0), rth_state=RTHState.COMPLETE)
    uav_ok = UAVState(id="u-ok", position_xy=(0.0, 0.0), rth_state=RTHState.NONE)

    t_complete = TaskState(id="t-comp", position_xy=(5.0, 5.0), priority=5, status=TaskStatus.COMPLETE)
    t_unreachable = TaskState(id="t-unreach", position_xy=(5.0, 5.0), priority=5, status=TaskStatus.UNREACHABLE)
    t_pending = TaskState(id="t-pend", position_xy=(5.0, 5.0), priority=1, status=TaskStatus.PENDING)

    snapshot = StateSnapshot(
        simulation_tick=1,
        simulation_time=0.0,
        state_version=1,
        uavs=MappingProxyType({
            "u-rth-act": uav_rth_active,
            "u-rth-req": uav_rth_req,
            "u-rth-comp": uav_rth_comp,
            "u-ok": uav_ok,
        }),
        tasks=MappingProxyType({
            "t-comp": t_complete,
            "t-unreach": t_unreachable,
            "t-pend": t_pending,
        }),
    )

    adapter = A0AutonomyAdapter(A0TaskAllocator())
    cmds = adapter.plan(snapshot)
    assert len(cmds) == 1
    assert cmds[0].uav_id == "u-ok"
    assert cmds[0].task_id == "t-pend"
