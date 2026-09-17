import pytest
from types import MappingProxyType
from ares_swarm.core.state_store import StateStore
from ares_swarm.core.models import UAVState, TaskState, StateSnapshot
from ares_swarm.core.enums import TaskStatus, RejectionCode, EventType
from ares_swarm.core.commands import AssignTaskCommand, ProgressTaskCommand

def test_state_store_atomic_rollback():
    uav = UAVState(id="u1", active=True)
    task = TaskState(
        id="t1",
        position_xy=(0.0, 0.0),
        priority=1,
        created_time=0.0,
        deadline=100.0,
        service_duration=10.0,
        status=TaskStatus.COMPLETE,
    )
    snap = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav}),
        tasks=MappingProxyType({"t1": task}),
    )
    store = StateStore(snap)

    cmd = AssignTaskCommand(source_tick=0, uav_id="u1", task_id="t1")
    result = store.apply([cmd])

    assert len(result.rejected_commands) == 1
    assert result.rejected_commands[0].code == RejectionCode.ILLEGAL_LIFECYCLE_TRANSITION
    assert store.snapshot().uavs["u1"].assigned_task_id is None

def test_state_store_stale_tick():
    uav = UAVState(id="u1", active=True)
    snap = StateSnapshot(
        simulation_tick=1,
        simulation_time=1.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav}),
        tasks=MappingProxyType({}),
    )
    store = StateStore(snap)

    cmd = AssignTaskCommand(source_tick=0, uav_id="u1", task_id="t1")
    result = store.apply([cmd])

    assert len(result.rejected_commands) == 1
    assert result.rejected_commands[0].code == RejectionCode.STALE_TICK_PROPOSAL

def test_task_completion():
    uav = UAVState(id="u1", active=True, assigned_task_id="t1")
    task = TaskState(
        id="t1",
        position_xy=(0.0, 0.0),
        priority=1,
        created_time=0.0,
        deadline=100.0,
        service_duration=10.0,
        service_progress=0.0,
        status=TaskStatus.IN_PROGRESS,
        assigned_uav_id="u1",
    )
    snap = StateSnapshot(
        simulation_tick=1,
        simulation_time=1.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav}),
        tasks=MappingProxyType({"t1": task}),
    )
    store = StateStore(snap)

    cmd = ProgressTaskCommand(source_tick=1, uav_id="u1", task_id="t1", delta_progress=10.0)
    result = store.apply([cmd])

    assert len(result.applied_commands) == 1
    snap_after = store.snapshot()
    assert snap_after.tasks["t1"].status == TaskStatus.COMPLETE
    assert snap_after.tasks["t1"].assigned_uav_id is None
    assert snap_after.uavs["u1"].assigned_task_id is None
