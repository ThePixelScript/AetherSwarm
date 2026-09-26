"""Unit and integration tests for multi-surveyor concurrent scheduling and active surveyor protection."""
from __future__ import annotations

import pytest

from ares_swarm.autonomy.connectivity_planner import ConnectivityAwarePlanner
from ares_swarm.autonomy.relay_manager import DynamicRelayManager
from ares_swarm.core.enums import FailureState, Role, RTHState, SortieState, TaskStatus
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState


def make_test_uav(
    uav_id: str,
    position: tuple[float, float] = (-75.0, 500.0),
    role: Role = Role.IDLE,
    battery: float = 1000.0,
    active: bool = True,
) -> UAVState:
    return UAVState(
        id=uav_id,
        position_xy=position,
        velocity_xy=(0.0, 0.0),
        battery_energy=battery,
        battery_capacity=1000.0,
        role=role,
        active=active,
        failure_state=FailureState.NORMAL,
        rth_state=RTHState.NONE,
        sortie_state=SortieState.ACTIVE,
    )


def test_concurrent_multi_surveyor_assignment():
    """Verify ConnectivityAwarePlanner assigns multiple surveyors concurrently when capacity allows."""
    rm = DynamicRelayManager()
    planner = ConnectivityAwarePlanner(relay_manager=rm)

    uavs = {
        "uav_1": make_test_uav("uav_1", position=(-75.0, 500.0)),
        "uav_2": make_test_uav("uav_2", position=(-75.0, 500.0)),
    }
    tasks = {
        "task_1": TaskState(id="task_1", position_xy=(10.0, 500.0), priority=1, service_duration=10.0, status=TaskStatus.PENDING),
        "task_2": TaskState(id="task_2", position_xy=(15.0, 500.0), priority=1, service_duration=10.0, status=TaskStatus.PENDING),
    }

    snapshot = StateSnapshot(uavs=uavs, tasks=tasks, gcs_position=(-75.0, 500.0), simulation_tick=0, simulation_time=0.0, state_version=1)

    cmds = planner.plan(snapshot)

    # Should emit AssignTaskCommand for both task_1 and task_2 using distinct surveyors uav_1 and uav_2
    assigned_tasks = [c.task_id for c in cmds if c.__class__.__name__ == "AssignTaskCommand"]
    assigned_uavs = [c.uav_id for c in cmds if c.__class__.__name__ == "AssignTaskCommand"]

    assert "task_1" in assigned_tasks
    assert "task_2" in assigned_tasks
    assert len(set(assigned_uavs)) == 2


def test_active_surveyor_protected_from_relay_theft():
    """Verify an active surveyor working on a task cannot be selected as a relay for another task."""
    rm = DynamicRelayManager()
    planner = ConnectivityAwarePlanner(relay_manager=rm)

    # uav_1 is actively servicing task_1
    surv_active = make_test_uav("uav_1", position=(100.0, 500.0), role=Role.SURVEYOR)
    surv_active = UAVState(
        id="uav_1",
        position_xy=(100.0, 500.0),
        velocity_xy=(0.0, 0.0),
        battery_energy=1000.0,
        battery_capacity=1000.0,
        role=Role.SURVEYOR,
        active=True,
        failure_state=FailureState.NORMAL,
        rth_state=RTHState.NONE,
        sortie_state=SortieState.ACTIVE,
        assigned_task_id="task_1",
    )
    # uav_2 is idle
    uav_idle = make_test_uav("uav_2", position=(-75.0, 500.0), role=Role.IDLE)

    uavs = {"uav_1": surv_active, "uav_2": uav_idle}
    tasks = {
        "task_1": TaskState(id="task_1", position_xy=(100.0, 500.0), priority=1, service_duration=10.0, status=TaskStatus.IN_PROGRESS, assigned_uav_id="uav_1"),
        "task_2": TaskState(id="task_2", position_xy=(200.0, 500.0), priority=2, service_duration=10.0, status=TaskStatus.PENDING),
    }

    snapshot = StateSnapshot(uavs=uavs, tasks=tasks, gcs_position=(-75.0, 500.0), simulation_tick=0, simulation_time=0.0, state_version=1)

    # uav_2 tries to plan for task_2 (which requires relays)
    # uav_1 is protected from being stolen as a relay
    res = planner.check_task_connectivity_feasibility(task=tasks["task_2"], uav=uav_idle, snapshot=snapshot)
    assert surv_active.id not in res.relay_uav_ids
