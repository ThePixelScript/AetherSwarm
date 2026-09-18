from types import MappingProxyType
from ares_swarm.core.enums import FailureState, TaskStatus, RTHState
from ares_swarm.core.models import StateSnapshot, UAVState, TaskState
from ares_swarm.core.state_store import StateStore
from ares_swarm.core.simulator import SimulationEngine


def test_clock_starts_at_zero():
    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({}),
        tasks=MappingProxyType({}),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store)

    current = store.snapshot()

    assert current.simulation_tick == 0
    assert current.simulation_time == 0.0


def test_clock_advances_deterministically():
    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({}),
        tasks=MappingProxyType({}),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store, dt=1.0)

    engine.advance_tick()
    engine.advance_tick()

    current = store.snapshot()

    assert current.simulation_tick == 2
    assert current.simulation_time == 2.0


def test_invalid_dt_rejected():
    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({}),
        tasks=MappingProxyType({}),
    )

    store = StateStore(snapshot)

    try:
        SimulationEngine(store, dt=0)
        assert False
    except ValueError:
        assert True
def test_clock_synchronizes_with_state_store():
    snapshot = StateSnapshot(
        simulation_tick=5,
        simulation_time=10.0,
        state_version=0,
        uavs=MappingProxyType({}),
        tasks=MappingProxyType({}),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store, dt=2.0)

    engine.advance_tick()

    current = store.snapshot()

    assert current.simulation_tick == 6
    assert current.simulation_time == 12.0
def test_step_uav_updates_position_velocity_and_battery():
    uav = UAVState(
        id="u1",
        position_xy=(0.0, 0.0),
        battery_energy=100.0,
    )

    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav}),
        tasks=MappingProxyType({}),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store, dt=1.0)

    result = engine.step_uav(
    uav_id="u1",
    target=(10.0, 0.0),
    speed=5.0,
)

    assert len(result.applied_commands) == 1

    updated = store.snapshot().uavs["u1"]

    assert updated.position_xy == (5.0, 0.0)
    assert updated.velocity_xy == (5.0, 0.0)
    assert updated.battery_energy == 96.5
def test_start_rth_updates_uav_state():
    uav = UAVState(
        id="u1",
        position_xy=(10.0, 20.0),
        battery_energy=50.0,
    )

    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav}),
        tasks=MappingProxyType({}),
        gcs_position=(0.0, 0.0),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store)

    result = engine.start_rth("u1")

    assert len(result.applied_commands) == 1

    updated = store.snapshot().uavs["u1"]

    assert updated.rth_state.value == "ACTIVE"
    assert updated.target_position == (0.0, 0.0)
    assert updated.assigned_task_id is None
def test_rth_moves_uav_toward_gcs():
    uav = UAVState(
        id="u1",
        position_xy=(10.0, 0.0),
        velocity_xy=(0.0, 0.0),
        target_position=None,
        battery_capacity=100.0,
        battery_energy=50.0,
    )

    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs={"u1": uav},
        tasks={},
        gcs_position=(0.0, 0.0),
    )

    store = StateStore(initial_snapshot=snapshot)
    engine = SimulationEngine(store, dt=1.0)

    engine.start_rth("u1")
    engine.step_swarm(speed=5.0)

    updated = store.snapshot().uavs["u1"]

    assert updated.rth_state == RTHState.ACTIVE
    assert updated.position_xy == (5.0, 0.0)
    assert updated.target_position == (0.0, 0.0)
def test_rth_snaps_uav_to_gcs_on_arrival():
    uav = UAVState(
        id="u1",
        position_xy=(4.0, 0.0),
        velocity_xy=(0.0, 0.0),
        target_position=None,
        battery_capacity=100.0,
        battery_energy=50.0,
    )

    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs={"u1": uav},
        tasks={},
        gcs_position=(0.0, 0.0),
    )

    store = StateStore(initial_snapshot=snapshot)
    engine = SimulationEngine(store, dt=1.0)

    engine.start_rth("u1")
    engine.step_swarm(speed=5.0)

    updated = store.snapshot().uavs["u1"]

    assert updated.rth_state == RTHState.ACTIVE
    assert updated.position_xy == (0.0, 0.0)
    assert updated.velocity_xy == (0.0, 0.0)
def test_step_swarm_updates_multiple_uavs():
    uav1 = UAVState(
        id="u1",
        position_xy=(0.0, 0.0),
        target_position=(10.0, 0.0),
        battery_energy=100.0,
    )

    uav2 = UAVState(
        id="u2",
        position_xy=(0.0, 0.0),
        target_position=(0.0, 10.0),
        battery_energy=100.0,
    )

    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav1, "u2": uav2}),
        tasks=MappingProxyType({}),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store)

    engine.step_swarm()

    updated = store.snapshot()

    assert updated.uavs["u1"].position_xy == (5.0, 0.0)
    assert updated.uavs["u2"].position_xy == (0.0, 5.0)
def test_step_swarm_uses_deterministic_uav_order():
    uav2 = UAVState(
        id="u2",
        position_xy=(0.0, 0.0),
        target_position=(10.0, 0.0),
        battery_energy=100.0,
    )

    uav1 = UAVState(
        id="u1",
        position_xy=(0.0, 0.0),
        target_position=(0.0, 10.0),
        battery_energy=100.0,
    )

    snapshot = StateSnapshot(
        simulation_tick=5,
        simulation_time=5.0,
        state_version=0,
        uavs=MappingProxyType({"u2": uav2, "u1": uav1}),
        tasks=MappingProxyType({}),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store)

    result = engine.step_swarm()

    assert [cmd.uav_id for cmd in result.applied_commands] == ["u1", "u2"]
def test_step_swarm_uses_one_state_store_apply_call(monkeypatch):
    uav1 = UAVState(
        id="u1",
        position_xy=(0.0, 0.0),
        target_position=(10.0, 0.0),
        battery_energy=100.0,
    )

    uav2 = UAVState(
        id="u2",
        position_xy=(0.0, 0.0),
        target_position=(0.0, 10.0),
        battery_energy=100.0,
    )

    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav1, "u2": uav2}),
        tasks=MappingProxyType({}),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store)

    calls = []
    original_apply = store.apply

    def tracked_apply(commands):
        calls.append(list(commands))
        return original_apply(commands)

    monkeypatch.setattr(store, "apply", tracked_apply)

    engine.step_swarm()

    assert len(calls) == 1
    assert len(calls[0]) == 2
def test_step_swarm_uses_snapshot_tick():
    uav = UAVState(
        id="u1",
        position_xy=(0.0, 0.0),
        target_position=(10.0, 0.0),
        battery_energy=100.0,
    )

    snapshot = StateSnapshot(
        simulation_tick=7,
        simulation_time=7.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav}),
        tasks=MappingProxyType({}),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store)

    result = engine.step_swarm()

    assert result.applied_commands[0].source_tick == 7


def test_step_swarm_enforces_max_speed():
    uav = UAVState(
        id="u1",
        position_xy=(0.0, 0.0),
        target_position=(20.0, 0.0),
        battery_energy=100.0,
    )

    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav}),
        tasks=MappingProxyType({}),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store)

    try:
        engine.step_swarm(speed=6.0)
        assert False
    except ValueError:
        assert True
def test_step_swarm_updates_velocity_and_battery_through_state_store():
    uav = UAVState(
        id="u1",
        position_xy=(0.0, 0.0),
        target_position=(10.0, 0.0),
        battery_energy=100.0,
    )

    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav}),
        tasks=MappingProxyType({}),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store)

    engine.step_swarm()

    updated = store.snapshot().uavs["u1"]

    assert updated.position_xy == (5.0, 0.0)
    assert updated.velocity_xy == (5.0, 0.0)
    assert updated.battery_energy == 96.5
def test_step_swarm_does_not_make_battery_negative():
    uav = UAVState(
        id="u1",
        position_xy=(0.0, 0.0),
        target_position=(10.0, 0.0),
        battery_energy=1.0,
    )

    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav}),
        tasks=MappingProxyType({}),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store)

    engine.step_swarm()

    updated = store.snapshot().uavs["u1"]

    assert updated.battery_energy == 0.0
def test_step_swarm_skips_inactive_uav():
    uav = UAVState(
        id="u1",
        position_xy=(0.0, 0.0),
        target_position=(10.0, 0.0),
        battery_energy=100.0,
        active=False,
    )

    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav}),
        tasks=MappingProxyType({}),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store)

    result = engine.step_swarm()

    assert result.applied_commands == ()
    assert store.snapshot().uavs["u1"].position_xy == (0.0, 0.0)
def test_step_swarm_skips_failed_uav():
    uav = UAVState(
        id="u1",
        position_xy=(0.0, 0.0),
        target_position=(10.0, 0.0),
        battery_energy=100.0,
        failure_state=FailureState.FAILED,
    )

    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav}),
        tasks=MappingProxyType({}),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store)

    result = engine.step_swarm()

    assert result.applied_commands == ()
    assert store.snapshot().uavs["u1"].position_xy == (0.0, 0.0)
def test_step_swarm_steps_active_uav():
    uav = UAVState(
        id="u1",
        position_xy=(0.0, 0.0),
        target_position=(10.0, 0.0),
        battery_energy=100.0,
        active=True,
    )

    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav}),
        tasks=MappingProxyType({}),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store)

    result = engine.step_swarm()

    assert len(result.applied_commands) == 1
    assert store.snapshot().uavs["u1"].position_xy == (5.0, 0.0)
def test_step_swarm_progresses_task_on_arrival():
    uav = UAVState(
        id="u1",
        position_xy=(0.0, 0.0),
        target_position=(5.0, 0.0),
        assigned_task_id="t1",
        battery_energy=100.0,
    )

    task = TaskState(
        id="t1",
        position_xy=(5.0, 0.0),
        priority=1,
        service_duration=10.0,
        status=TaskStatus.IN_PROGRESS,
        assigned_uav_id="u1",
    )

    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav}),
        tasks=MappingProxyType({"t1": task}),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store, dt=1.0)

    physics_result = engine.step_swarm()

    assert len(physics_result.applied_commands) == 1
    assert store.snapshot().uavs["u1"].position_xy == (5.0, 0.0)

    progress_result = engine.progress_arrived_tasks()

    assert len(progress_result.applied_commands) == 1
    assert progress_result.applied_commands[0].task_id == "t1"

    updated = store.snapshot()
    assert updated.tasks["t1"].service_progress == 1.0
def test_task_service_progresses_until_completion():
    uav = UAVState(
        id="u1",
        position_xy=(5.0, 0.0),
        assigned_task_id="t1",
        battery_energy=100.0,
    )

    task = TaskState(
        id="t1",
        position_xy=(5.0, 0.0),
        priority=1,
        service_duration=3.0,
        service_progress=0.0,
        status=TaskStatus.IN_PROGRESS,
        assigned_uav_id="u1",
    )

    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav}),
        tasks=MappingProxyType({"t1": task}),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store, dt=1.0)

    engine.progress_arrived_tasks()
    assert store.snapshot().tasks["t1"].service_progress == 1.0
    assert store.snapshot().tasks["t1"].status == TaskStatus.IN_PROGRESS

    engine.advance_tick()
    engine.progress_arrived_tasks()
    assert store.snapshot().tasks["t1"].service_progress == 2.0
    assert store.snapshot().tasks["t1"].status == TaskStatus.IN_PROGRESS

    engine.advance_tick()
    engine.progress_arrived_tasks()

    updated = store.snapshot()
    assert updated.tasks["t1"].service_progress == 3.0
    assert updated.tasks["t1"].status == TaskStatus.COMPLETE
    assert updated.tasks["t1"].assigned_uav_id is None
    assert updated.uavs["u1"].assigned_task_id is None
def test_completed_task_does_not_continue_moving_uav():
    uav = UAVState(
        id="u1",
        position_xy=(5.0, 0.0),
        target_position=(5.0, 0.0),
        assigned_task_id="t1",
        battery_energy=100.0,
    )

    task = TaskState(
        id="t1",
        position_xy=(5.0, 0.0),
        priority=1,
        service_duration=1.0,
        status=TaskStatus.IN_PROGRESS,
        assigned_uav_id="u1",
    )

    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({"u1": uav}),
        tasks=MappingProxyType({"t1": task}),
    )

    store = StateStore(snapshot)
    engine = SimulationEngine(store, dt=1.0)

    engine.progress_arrived_tasks()

    updated = store.snapshot()

    assert updated.tasks["t1"].status == TaskStatus.COMPLETE
    assert updated.uavs["u1"].assigned_task_id is None
def test_check_battery_rth_triggers_for_low_battery():
    uav = UAVState(
        id="u1",
        position_xy=(10.0, 0.0),
        battery_energy=5.0,
        target_position=(20.0, 0.0),
    )

    store = StateStore(
        StateSnapshot(
            simulation_tick=0,
            simulation_time=0.0,
            state_version=0,
            uavs={"u1": uav},
            gcs_position=(0.0, 0.0),
        )
    )

    engine = SimulationEngine(store)

    result = engine.check_battery_rth(rth_reserve=1.0)

    assert len(result.applied_commands) == 1
    assert store.snapshot().uavs["u1"].rth_state == RTHState.ACTIVE
    assert store.snapshot().uavs["u1"].target_position == (0.0, 0.0)


def test_check_battery_rth_does_not_trigger_with_sufficient_battery():
    uav = UAVState(
        id="u1",
        position_xy=(10.0, 0.0),
        battery_energy=100.0,
        target_position=(20.0, 0.0),
    )

    store = StateStore(
        StateSnapshot(
            simulation_tick=0,
            simulation_time=0.0,
            state_version=0,
            uavs={"u1": uav},
            gcs_position=(0.0, 0.0),
        )
    )

    engine = SimulationEngine(store)

    result = engine.check_battery_rth(rth_reserve=1.0)

    assert len(result.applied_commands) == 0
    assert store.snapshot().uavs["u1"].rth_state == RTHState.NONE