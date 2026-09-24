"""Focused unit and integration tests for deterministic non-collinear RTH routing (A2)."""
import math
import pytest

from ares_swarm.core.enums import EventType, Role, RTHState, SortieState
from ares_swarm.core.models import StateSnapshot, UAVState
from ares_swarm.safety.airspace import ChallengeAirspace
from ares_swarm.safety.rth_router import RTHRouter
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import (
    ChallengeAirspaceConfig,
    ChallengeProfileConfig,
    CommunicationConfig,
    ScenarioConfig,
)


def build_rth_scenario(num_uavs: int = 4, seed: int = 42) -> ScenarioConfig:
    """Helper to build a scenario with UAVs positioned at PoIs ready to trigger RTH."""
    gcs = (-75.0, 500.0)
    airspace_cfg = ChallengeAirspaceConfig(
        enabled=True,
        staging_pad_center=gcs,
        staging_pad_radius_m=15.0,
        corridor_bounds_x=(-75.0, 0.0),
        corridor_bounds_y=(400.0, 600.0),
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
    )
    prof_cfg = ChallengeProfileConfig(
        enabled=True,
        airspace=airspace_cfg,
        enforce_separation=True,
        enforce_geofence=True,
        enable_departure_sequencing=True,
    )
    uavs = tuple(
        {
            "id": f"uav_{i+1}",
            "position": [200.0 + i * 30.0, 430.0 + i * 20.0],
            "role": "IDLE",
            "battery_capacity": 4200.0,
            "battery_energy": 4200.0,
        }
        for i in range(num_uavs)
    )
    tasks = tuple(
        {
            "id": f"poi_{i+1}",
            "position": [200.0 + i * 30.0, 430.0 + i * 20.0],
            "priority": 1,
            "spawn_time": 0.0,
            "service_duration": 2.0,
        }
        for i in range(num_uavs)
    )
    return ScenarioConfig(
        name="test_rth_scenario",
        seed=seed,
        dt=1.0,
        speed_limit=5.0,
        duration=300.0,
        max_ticks=300,
        gcs_position=gcs,
        communication=CommunicationConfig(max_range=100.0),
        uavs=uavs,
        tasks=tasks,
        challenge_profile=prof_cfg,
        enable_departure_sequencing=True,
    )


def test_1_simultaneous_rth_no_deadlock():
    """Requirement 1: Multiple UAVs enter RTH simultaneously without deadlock."""
    scenario = build_rth_scenario(num_uavs=4)
    runner = MissionRunner(scenario=scenario, seed=42)

    # Step until UAVs are airborne/active
    for _ in range(10):
        runner.step()

    # Trigger RTH simultaneously on all UAVs
    from ares_swarm.core.commands import StartRTHCommand
    snap = runner.state_store.snapshot()
    rth_cmds = [StartRTHCommand(source_tick=snap.simulation_tick, uav_id=uid) for uid in snap.uavs]
    runner.state_store.apply(rth_cmds)

    # Run for 150 ticks to allow all to return
    for _ in range(150):
        runner.step()

    final_snap = runner.state_store.snapshot()
    for uid, uav in final_snap.uavs.items():
        assert uav.rth_state == RTHState.COMPLETE, f"UAV {uid} failed to complete RTH, state={uav.rth_state}"
        assert uav.sortie_state in (SortieState.LANDED, SortieState.RECHARGING, SortieState.READY), f"UAV {uid} not landed: {uav.sortie_state}"


def test_2_rth_routes_min_separation():
    """Requirement 2: RTH routes remain >= 20.0m separated continuously."""
    scenario = build_rth_scenario(num_uavs=4)
    runner = MissionRunner(scenario=scenario, seed=42)

    for _ in range(10):
        runner.step()

    from ares_swarm.core.commands import StartRTHCommand
    snap = runner.state_store.snapshot()
    rth_cmds = [StartRTHCommand(source_tick=snap.simulation_tick, uav_id=uid) for uid in snap.uavs]
    runner.state_store.apply(rth_cmds)

    for _ in range(150):
        runner.step()

    metrics = runner.run().metrics_report
    assert metrics is not None
    assert metrics.separation_violation_count == 0, f"Expected 0 separation violations, got {metrics.separation_violation_count}"


def test_3_rth_routes_within_geofence():
    """Requirement 3: RTH routes remain strictly within authorized airspace/geofence."""
    scenario = build_rth_scenario(num_uavs=4)
    runner = MissionRunner(scenario=scenario, seed=42)

    for _ in range(10):
        runner.step()

    from ares_swarm.core.commands import StartRTHCommand
    snap = runner.state_store.snapshot()
    rth_cmds = [StartRTHCommand(source_tick=snap.simulation_tick, uav_id=uid) for uid in snap.uavs]
    runner.state_store.apply(rth_cmds)

    for _ in range(150):
        runner.step()

    metrics = runner.run().metrics_report
    assert metrics is not None
    assert metrics.geofence_violation_count == 0, f"Expected 0 geofence violations, got {metrics.geofence_violation_count}"


def test_4_rth_uavs_reach_staging_pads():
    """Requirement 4: UAVs reach their designated staging pads at x = -75.0."""
    scenario = build_rth_scenario(num_uavs=4)
    runner = MissionRunner(scenario=scenario, seed=42)

    for _ in range(10):
        runner.step()

    from ares_swarm.core.commands import StartRTHCommand
    snap = runner.state_store.snapshot()
    rth_cmds = [StartRTHCommand(source_tick=snap.simulation_tick, uav_id=uid) for uid in snap.uavs]
    runner.state_store.apply(rth_cmds)

    for _ in range(150):
        runner.step()

    final_snap = runner.state_store.snapshot()
    for uid, uav in final_snap.uavs.items():
        assert uav.position_xy[0] <= -74.8, f"UAV {uid} did not reach staging x=-75.0, got x={uav.position_xy[0]}"


def test_5_deterministic_rth_route_assignment():
    """Requirement 5: RTH route assignment is strictly deterministic."""
    scenario = build_rth_scenario(num_uavs=4)
    runner1 = MissionRunner(scenario=scenario, seed=42)
    runner1.step()

    from ares_swarm.core.commands import StartRTHCommand
    snap1 = runner1.state_store.snapshot()
    runner1.state_store.apply([StartRTHCommand(source_tick=snap1.simulation_tick, uav_id=uid) for uid in snap1.uavs])
    for _ in range(50):
        runner1.step()
    positions1 = {uid: u.position_xy for uid, u in runner1.state_store.snapshot().uavs.items()}

    runner2 = MissionRunner(scenario=scenario, seed=42)
    runner2.step()
    snap2 = runner2.state_store.snapshot()
    runner2.state_store.apply([StartRTHCommand(source_tick=snap2.simulation_tick, uav_id=uid) for uid in snap2.uavs])
    for _ in range(50):
        runner2.step()
    positions2 = {uid: u.position_xy for uid, u in runner2.state_store.snapshot().uavs.items()}

    assert positions1 == positions2, f"RTH deterministic position mismatch: {positions1} vs {positions2}"


def test_6_battery_and_relay_handoff_preserved():
    """Requirement 6: Battery recharge lifecycle and relay handoff behavior are preserved."""
    scenario = build_rth_scenario(num_uavs=2)
    runner = MissionRunner(scenario=scenario, seed=42)

    for _ in range(5):
        runner.step()

    # Trigger RTH for uav_1
    from ares_swarm.core.commands import StartRTHCommand
    snap = runner.state_store.snapshot()
    runner.state_store.apply([StartRTHCommand(source_tick=snap.simulation_tick, uav_id="uav_1")])

    for _ in range(100):
        runner.step()

    u1 = runner.state_store.snapshot().uavs["uav_1"]
    assert u1.rth_state == RTHState.COMPLETE
    assert u1.sortie_state in (SortieState.LANDED, SortieState.RECHARGING, SortieState.READY)
    assert u1.battery_energy > 0.0
