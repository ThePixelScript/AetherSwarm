from types import MappingProxyType

from ares_swarm.core.models import StateSnapshot, UAVState
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