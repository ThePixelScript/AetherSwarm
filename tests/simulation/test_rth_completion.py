"""Unit and integration tests for canonical RTH completion lifecycle in MissionRunner."""
import pytest
from ares_swarm.core.enums import EventType, Role, RTHState, TaskStatus
from ares_swarm.core.commands import CompleteRTHCommand, StartRTHCommand
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import ScenarioConfig


def test_rth_completion_lifecycle_in_mission_runner():
    """Verify complete RTH lifecycle in MissionRunner:
    StartRTH -> move toward GCS -> arrival -> CompleteRTHCommand -> StateStore updates -> UAV_LANDED event.
    """
    config = ScenarioConfig(
        name="test_rth_lifecycle",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=10.0,
        max_ticks=10,
        gcs_position=(0.0, 0.0),
        uavs=(
            {
                "id": "u1",
                "position": [10.0, 0.0],
                "battery_capacity": 100.0,
                "battery_energy": 100.0,
                "role": "IDLE",
                "active": True,
            },
        ),
        tasks=(
            {"id": "t1", "position": [50.0, 0.0], "priority": 1, "service_duration": 1.0},
        ),
        enable_auto_rth=False,
    )

    runner = MissionRunner(config)

    # Initial state: UAV at (10, 0), active, RTHState.NONE
    snap0 = runner.state_store.snapshot()
    assert snap0.uavs["u1"].rth_state == RTHState.NONE
    assert snap0.uavs["u1"].active is True
    assert snap0.uavs["u1"].position_xy == (10.0, 0.0)

    # Trigger RTH via StateStore
    res_rth = runner.state_store.apply([StartRTHCommand(source_tick=0, uav_id="u1")])
    assert len(res_rth.applied_commands) == 1
    snap_after_rth = runner.state_store.snapshot()
    assert snap_after_rth.uavs["u1"].rth_state == RTHState.ACTIVE
    assert snap_after_rth.uavs["u1"].target_position == (0.0, 0.0)
    assert snap_after_rth.uavs["u1"].active is True

    # Step 1: UAV flies from (10, 0) towards (0, 0) at 5 m/s -> reaches (5, 0)
    step1 = runner.step()
    u1_s1 = step1.snapshot.uavs["u1"]
    assert u1_s1.position_xy == (5.0, 0.0)
    assert u1_s1.rth_state == RTHState.ACTIVE
    assert u1_s1.active is True
    assert not any(e.event_type == EventType.UAV_LANDED for e in step1.events)

    # Step 2: UAV flies from (5, 0) to (0, 0) -> arrives at GCS!
    # CompleteRTHCommand should be emitted and applied during this step
    step2 = runner.step()
    u1_s2 = step2.snapshot.uavs["u1"]
    assert u1_s2.position_xy == (0.0, 0.0)
    assert u1_s2.velocity_xy == (0.0, 0.0)
    assert u1_s2.rth_state == RTHState.COMPLETE
    assert u1_s2.active is False
    assert u1_s2.role == Role.IDLE
    assert u1_s2.target_position is None

    # Check UAV_LANDED event
    landed_events = [e for e in step2.events if e.event_type == EventType.UAV_LANDED]
    assert len(landed_events) == 1
    assert landed_events[0].entity_id == "u1"
    assert "final_energy" in landed_events[0].payload
    assert landed_events[0].payload["final_energy"] == u1_s2.battery_energy

    # Check CompleteRTHCommand was in applied_commands
    complete_cmds = [c for c in step2.applied_commands if isinstance(c, CompleteRTHCommand)]
    assert len(complete_cmds) == 1
    assert complete_cmds[0].uav_id == "u1"
    assert complete_cmds[0].source_tick == 1  # current_tick when step2 began

    # Step 3: Step simulation again -> Landed UAV must NOT move or be assigned
    battery_before_s3 = u1_s2.battery_energy
    step3 = runner.step()
    u1_s3 = step3.snapshot.uavs["u1"]
    assert u1_s3.position_xy == (0.0, 0.0)
    assert u1_s3.velocity_xy == (0.0, 0.0)
    assert u1_s3.active is False
    assert u1_s3.rth_state == RTHState.COMPLETE
    assert u1_s3.target_position is None
    assert u1_s3.assigned_task_id is None
    # No energy consumed while landed
    assert u1_s3.battery_energy == battery_before_s3
    # No additional commands applied for landed UAV
    assert not any(c.uav_id == "u1" for c in step3.applied_commands)


def test_auto_rth_triggered_when_all_tasks_complete():
    """Verify auto-RTH triggers for active UAVs when all tasks are COMPLETE."""
    config = ScenarioConfig(
        name="test_auto_rth_complete",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=10.0,
        max_ticks=10,
        gcs_position=(0.0, 0.0),
        uavs=(
            {"id": "u1", "position": [10.0, 0.0], "battery_capacity": 100.0, "battery_energy": 100.0, "role": "SURVEYOR", "active": True},
            {"id": "u2", "position": [30.0, 0.0], "battery_capacity": 100.0, "battery_energy": 100.0, "role": "RELAY", "active": True},
        ),
        tasks=(
            {"id": "t1", "position": [10.0, 0.0], "priority": 1, "service_duration": 1.0, "spawn_time": 0.0},
        ),
        enable_auto_rth=True,
    )
    runner = MissionRunner(config)
    from ares_swarm.core.commands import AssignTaskCommand
    runner.state_store.apply([
        AssignTaskCommand(source_tick=0, uav_id="u1", task_id="t1"),
    ])
    # Step 1: u1 at (10, 0) completes task t1
    step1 = runner.step()
    # Both u1 and u2 should transition to RTHState.ACTIVE
    snap1 = step1.snapshot
    assert snap1.tasks["t1"].status == TaskStatus.COMPLETE
    assert snap1.uavs["u1"].rth_state == RTHState.ACTIVE
    assert snap1.uavs["u2"].rth_state == RTHState.ACTIVE


def test_auto_rth_triggered_when_tasks_complete_and_unreachable():
    """Verify auto-RTH triggers when tasks are a mix of COMPLETE and UNREACHABLE."""
    config = ScenarioConfig(
        name="test_auto_rth_unreachable",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=10.0,
        max_ticks=10,
        gcs_position=(0.0, 0.0),
        uavs=(
            {"id": "u1", "position": [10.0, 0.0], "battery_capacity": 100.0, "battery_energy": 100.0, "role": "SURVEYOR", "active": True},
        ),
        tasks=(
            {"id": "t1", "position": [10.0, 0.0], "priority": 1, "service_duration": 1.0, "spawn_time": 0.0},
            {"id": "t2", "position": [900.0, 0.0], "priority": 1, "service_duration": 1.0, "spawn_time": 0.0},
        ),
        enable_auto_rth=True,
    )
    runner = MissionRunner(config)
    from dataclasses import replace
    from ares_swarm.core.commands import AssignTaskCommand
    runner.state_store.apply([
        AssignTaskCommand(source_tick=0, uav_id="u1", task_id="t1"),
    ])
    # Directly mark t2 as UNREACHABLE
    runner.state_store._tasks["t2"] = replace(runner.state_store._tasks["t2"], status=TaskStatus.UNREACHABLE)

    step1 = runner.step()
    snap1 = step1.snapshot
    assert snap1.tasks["t1"].status == TaskStatus.COMPLETE
    assert snap1.tasks["t2"].status == TaskStatus.UNREACHABLE
    assert snap1.uavs["u1"].rth_state == RTHState.ACTIVE


def test_auto_rth_triggered_when_remaining_deferred_task_expired():
    """Verify auto-RTH triggers when all remaining tasks are DEFERRED with expired deadlines."""
    config = ScenarioConfig(
        name="test_auto_rth_expired_deferred",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=20.0,
        max_ticks=20,
        gcs_position=(0.0, 0.0),
        uavs=(
            {"id": "u1", "position": [10.0, 0.0], "battery_capacity": 100.0, "battery_energy": 100.0, "role": "SURVEYOR", "active": True},
        ),
        tasks=(
            {"id": "t1", "position": [10.0, 0.0], "priority": 1, "service_duration": 1.0, "spawn_time": 0.0},
            {"id": "t_def", "position": [80.0, 0.0], "priority": 1, "service_duration": 1.0, "spawn_time": 0.0, "deadline_offset": 5.0},
        ),
        enable_auto_rth=True,
    )
    noop_adapter = type("NoopAutonomy", (), {"plan": lambda self, snap, *args: []})()
    runner = MissionRunner(config, autonomy_adapter=noop_adapter)
    from dataclasses import replace
    from ares_swarm.core.commands import AssignTaskCommand
    # t1 assigned to u1, t_def deferred
    runner.state_store.apply([
        AssignTaskCommand(source_tick=0, uav_id="u1", task_id="t1"),
    ])
    runner.state_store._tasks["t_def"] = replace(runner.state_store._tasks["t_def"], status=TaskStatus.DEFERRED)

    # Step 1: t1 completes at t=1.0s, but t_def has deadline 5.0s > 1.0s, so t_def is still actionable -> auto-RTH should NOT trigger yet
    step1 = runner.step()
    assert step1.snapshot.tasks["t1"].status == TaskStatus.COMPLETE
    assert step1.snapshot.tasks["t_def"].status == TaskStatus.DEFERRED
    assert step1.snapshot.uavs["u1"].rth_state == RTHState.NONE

    # Advance time past t_def deadline (t >= 5.0s)
    for _ in range(5):
        runner.step()
    # At t=5.0s, t_def deadline is reached, auto-RTH should trigger
    snap_exp = runner.state_store.snapshot()
    assert snap_exp.simulation_time >= 5.0
    assert snap_exp.uavs["u1"].rth_state == RTHState.ACTIVE


def test_auto_rth_not_triggered_when_deferred_task_is_retryable():
    """Verify auto-RTH does NOT trigger when a DEFERRED task has no deadline or is before deadline."""
    config = ScenarioConfig(
        name="test_auto_rth_retryable",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=20.0,
        max_ticks=20,
        gcs_position=(0.0, 0.0),
        uavs=(
            {"id": "u1", "position": [10.0, 0.0], "battery_capacity": 100.0, "battery_energy": 100.0, "role": "SURVEYOR", "active": True},
        ),
        tasks=(
            {"id": "t1", "position": [10.0, 0.0], "priority": 1, "service_duration": 1.0, "spawn_time": 0.0},
            {"id": "t_retry", "position": [80.0, 0.0], "priority": 1, "service_duration": 1.0, "spawn_time": 0.0, "deadline_offset": 0.0},
        ),
        enable_auto_rth=True,
    )
    noop_adapter = type("NoopAutonomy", (), {"plan": lambda self, snap, *args: []})()
    runner = MissionRunner(config, autonomy_adapter=noop_adapter)
    from dataclasses import replace
    from ares_swarm.core.commands import AssignTaskCommand
    runner.state_store.apply([
        AssignTaskCommand(source_tick=0, uav_id="u1", task_id="t1"),
    ])
    runner.state_store._tasks["t_retry"] = replace(runner.state_store._tasks["t_retry"], status=TaskStatus.DEFERRED)
    step1 = runner.step()
    assert step1.snapshot.tasks["t1"].status == TaskStatus.COMPLETE
    assert step1.snapshot.tasks["t_retry"].status == TaskStatus.DEFERRED
    # t_retry has deadline 0.0 (no deadline), so it remains retryable and auto-RTH must not trigger
    assert step1.snapshot.uavs["u1"].rth_state == RTHState.NONE


def test_auto_rth_not_triggered_when_future_task_unspawned():
    """Verify auto-RTH does NOT trigger when future tasks have not yet spawned."""
    config = ScenarioConfig(
        name="test_auto_rth_future_task",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=20.0,
        max_ticks=20,
        gcs_position=(0.0, 0.0),
        uavs=(
            {"id": "u1", "position": [10.0, 0.0], "battery_capacity": 100.0, "battery_energy": 100.0, "role": "SURVEYOR", "active": True},
        ),
        tasks=(
            {"id": "t1", "position": [10.0, 0.0], "priority": 1, "service_duration": 1.0, "spawn_time": 0.0},
            {"id": "t_future", "position": [80.0, 0.0], "priority": 1, "service_duration": 1.0, "spawn_time": 10.0, "deadline_offset": 5.0},
        ),
        enable_auto_rth=True,
    )
    runner = MissionRunner(config)
    from ares_swarm.core.commands import AssignTaskCommand
    runner.state_store.apply([
        AssignTaskCommand(source_tick=0, uav_id="u1", task_id="t1"),
    ])
    step1 = runner.step()
    assert step1.snapshot.tasks["t1"].status == TaskStatus.COMPLETE
    assert step1.snapshot.tasks["t_future"].status == TaskStatus.PENDING
    # Future task has created_time 10.0 > 1.0, so auto-RTH must not trigger
    assert step1.snapshot.uavs["u1"].rth_state == RTHState.NONE


def test_auto_rth_disabled_flag_respected():
    """Verify auto-RTH does NOT trigger when enable_auto_rth=False even if all tasks complete."""
    config = ScenarioConfig(
        name="test_auto_rth_disabled",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=10.0,
        max_ticks=10,
        gcs_position=(0.0, 0.0),
        uavs=(
            {"id": "u1", "position": [10.0, 0.0], "battery_capacity": 100.0, "battery_energy": 100.0, "role": "SURVEYOR", "active": True},
        ),
        tasks=(
            {"id": "t1", "position": [10.0, 0.0], "priority": 1, "service_duration": 1.0, "spawn_time": 0.0},
        ),
        enable_auto_rth=False,
    )
    runner = MissionRunner(config)
    from ares_swarm.core.commands import AssignTaskCommand
    runner.state_store.apply([
        AssignTaskCommand(source_tick=0, uav_id="u1", task_id="t1"),
    ])
    step1 = runner.step()
    assert step1.snapshot.tasks["t1"].status == TaskStatus.COMPLETE
    assert step1.snapshot.uavs["u1"].rth_state == RTHState.NONE


def test_reachable_task_remains_after_task_completion():
    """Verify that when Task A completes, the relay chain is torn down, releasing relays to IDLE,
    so reachable Task B can be assigned and serviced rather than triggering early RTH.
    """
    config = ScenarioConfig(
        name="test_reachable_task_remains",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=100.0,
        max_ticks=100,
        gcs_position=(-75.0, 500.0),
        uavs=(
            {"id": "u1", "position": [-75.0, 500.0], "battery_capacity": 5000.0, "battery_energy": 5000.0, "role": "IDLE", "active": True},
            {"id": "u2", "position": [-75.0, 500.0], "battery_capacity": 5000.0, "battery_energy": 5000.0, "role": "IDLE", "active": True},
        ),
        tasks=(
            {"id": "t1", "position": [10.0, 500.0], "priority": 2, "service_duration": 1.0, "spawn_time": 0.0},
            {"id": "t2", "position": [20.0, 500.0], "priority": 1, "service_duration": 1.0, "spawn_time": 0.0},
        ),
        enable_auto_rth=True,
        enable_relay_manager=True,
        enable_connectivity_aware_planning=True,
    )
    runner = MissionRunner(config)
    result = runner.run()
    snap = result.final_snapshot
    assert snap.tasks["t1"].status == TaskStatus.COMPLETE
    assert snap.tasks["t2"].status == TaskStatus.COMPLETE


def test_rth_must_not_start_while_retryable_work_exists():
    """Verify auto-RTH is NOT triggered when reachable pending tasks remain to be serviced."""
    config = ScenarioConfig(
        name="test_no_rth_with_retryable_work",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=100.0,
        max_ticks=100,
        gcs_position=(-75.0, 500.0),
        uavs=(
            {"id": "u1", "position": [-75.0, 500.0], "battery_capacity": 5000.0, "battery_energy": 5000.0, "role": "IDLE", "active": True},
            {"id": "u2", "position": [-75.0, 500.0], "battery_capacity": 5000.0, "battery_energy": 5000.0, "role": "IDLE", "active": True},
        ),
        tasks=(
            {"id": "t1", "position": [10.0, 500.0], "priority": 2, "service_duration": 1.0, "spawn_time": 0.0},
            {"id": "t2", "position": [30.0, 500.0], "priority": 1, "service_duration": 1.0, "spawn_time": 0.0},
        ),
        enable_auto_rth=True,
        enable_relay_manager=True,
        enable_connectivity_aware_planning=True,
    )
    runner = MissionRunner(config)
    for _ in range(30):
        res = runner.step()
        snap = res.snapshot
        if snap.tasks["t2"].status == TaskStatus.PENDING:
            assert all(u.rth_state == RTHState.NONE for u in snap.uavs.values() if u.active)
