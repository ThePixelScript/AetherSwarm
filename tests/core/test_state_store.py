import pytest
from types import MappingProxyType
from ares_swarm.core.state_store import StateStore
from ares_swarm.core.models import UAVState, TaskState, StateSnapshot
from ares_swarm.core.enums import Role, RTHState, FailureState, TaskStatus, RejectionCode, EventType
from ares_swarm.core.commands import (
    AssignTaskCommand,
    ProgressTaskCommand,
    StartRTHCommand,
    CompleteRTHCommand,
    FailUAVCommand,
    RecoverUAVCommand,
)

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

def test_rth_full_lifecycle():
    uav = UAVState(id="u1", active=True, assigned_task_id="t1", rth_state=RTHState.NONE)
    task = TaskState(
        id="t1",
        position_xy=(10.0, 10.0),
        priority=1,
        status=TaskStatus.ASSIGNED,
        assigned_uav_id="u1",
    )
    snap = StateSnapshot(
        simulation_tick=2,
        simulation_time=2.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav}),
        tasks=MappingProxyType({"t1": task}),
        gcs_position=(0.0, 0.0),
    )
    store = StateStore(snap)

    # 1. Trigger RTH
    r1 = store.apply([StartRTHCommand(source_tick=2, uav_id="u1")])
    assert len(r1.applied_commands) == 1
    snap1 = store.snapshot()
    assert snap1.uavs["u1"].rth_state == RTHState.ACTIVE
    assert snap1.uavs["u1"].assigned_task_id is None
    assert snap1.uavs["u1"].target_position == (0.0, 0.0)
    assert snap1.tasks["t1"].status == TaskStatus.DEFERRED
    assert snap1.tasks["t1"].assigned_uav_id is None

    # 2. Complete RTH arrival
    r2 = store.apply([CompleteRTHCommand(source_tick=2, uav_id="u1")])
    assert len(r2.applied_commands) == 1
    snap2 = store.snapshot()
    uav_final = snap2.uavs["u1"]
    assert uav_final.rth_state == RTHState.COMPLETE
    assert uav_final.active is False
    assert uav_final.role == Role.IDLE
    assert uav_final.target_position is None
    assert any(e.event_type == EventType.UAV_LANDED for e in r2.emitted_events)


def test_uav_failure_and_recovery_lifecycle():
    uav = UAVState(id="u1", active=True, assigned_task_id="t1", failure_state=FailureState.NORMAL)
    task = TaskState(
        id="t1",
        position_xy=(10.0, 10.0),
        priority=1,
        status=TaskStatus.ASSIGNED,
        assigned_uav_id="u1",
    )
    snap = StateSnapshot(
        simulation_tick=3,
        simulation_time=3.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav}),
        tasks=MappingProxyType({"t1": task}),
    )
    store = StateStore(snap)

    # 1. Inject failure
    r1 = store.apply([FailUAVCommand(source_tick=3, uav_id="u1", failure_state=FailureState.FAILED, reason="motor_stall")])
    assert len(r1.applied_commands) == 1
    snap1 = store.snapshot()
    assert snap1.uavs["u1"].failure_state == FailureState.FAILED
    assert snap1.uavs["u1"].active is False
    assert snap1.uavs["u1"].assigned_task_id is None
    assert snap1.tasks["t1"].status == TaskStatus.DEFERRED
    assert snap1.tasks["t1"].assigned_uav_id is None
    assert any(e.event_type == EventType.UAV_FAILED for e in r1.emitted_events)

    # 2. Recover UAV
    r2 = store.apply([RecoverUAVCommand(source_tick=3, uav_id="u1")])
    assert len(r2.applied_commands) == 1
    snap2 = store.snapshot()
    assert snap2.uavs["u1"].failure_state == FailureState.NORMAL
    assert snap2.uavs["u1"].active is True
    assert snap2.uavs["u1"].role == Role.IDLE
    assert any(e.event_type == EventType.UAV_ACTIVATED for e in r2.emitted_events)
