"""Focused unit and integration tests for deterministic ground departure sequencing (A1)."""
import math
import pytest

from ares_swarm.core.enums import EventType, RTHState, SortieState
from ares_swarm.core.models import StateSnapshot, UAVState
from ares_swarm.safety.airspace import ChallengeAirspace
from ares_swarm.safety.departure import DepartureSequencer, UAVDeparturePhase
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import (
    ChallengeAirspaceConfig,
    ChallengeProfileConfig,
    CommunicationConfig,
    DetectionPipelineConfig,
    ScenarioConfig,
)


def build_8uav_scenario(name: str = "test_8uav_departure", seed: int = 42) -> ScenarioConfig:
    """Helper to build a standard 8-UAV scenario staged at x=-75.0 with >=20m spacing."""
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
    # 8 UAVs staged at x = -75.0, y spaced by 20m (from 430 to 570, within staging line y in [420, 580])
    uavs = tuple(
        {
            "id": f"uav_{i+1}",
            "position": [-75.0, 430.0 + i * 20.0],
            "role": "IDLE",
            "battery_capacity": 4200.0,
            "battery_energy": 4200.0,
        }
        for i in range(8)
    )
    # 8 tasks far in the arena
    tasks = tuple(
        {
            "id": f"poi_{i+1}",
            "position": [200.0 + i * 50.0, 430.0 + i * 20.0],
            "priority": 1,
            "spawn_time": 0.0,
            "service_duration": 2.0,
        }
        for i in range(8)
    )
    return ScenarioConfig(
        name=name,
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


def test_1_8uav_staging_pairwise_separation():
    """Requirement 1: 8-UAV staging is >= 20m pairwise."""
    scenario = build_8uav_scenario()
    uavs = scenario.uavs
    assert len(uavs) == 8
    for i in range(len(uavs)):
        for j in range(i + 1, len(uavs)):
            pos_i = uavs[i]["position"]
            pos_j = uavs[j]["position"]
            dist = math.hypot(pos_i[0] - pos_j[0], pos_i[1] - pos_j[1])
            assert dist >= 20.0, f"UAV {uavs[i]['id']} and {uavs[j]['id']} pairwise distance {dist:.2f}m < 20m"


def test_2_uav1_deterministic_first_clearance():
    """Requirement 2: UAV_1 gets deterministic first clearance."""
    scenario = build_8uav_scenario()
    runner = MissionRunner(scenario=scenario, seed=42)
    # Step 1 tick
    runner.step()
    seq = runner.departure_sequencer
    assert seq is not None
    assert seq.active_departing_uav_id == "uav_1", f"Expected uav_1 first active, got {seq.active_departing_uav_id}"
    assert seq.uav_phases["uav_1"] == UAVDeparturePhase.TAXIING


def test_3_uav1_moves_instead_of_indefinite_hold():
    """Requirement 3: UAV_1 actually moves instead of indefinite HOLD."""
    scenario = build_8uav_scenario()
    runner = MissionRunner(scenario=scenario, seed=42)
    init_x = runner.state_store.snapshot().uavs["uav_1"].position_xy[0]
    for _ in range(5):
        runner.step()
    curr_x = runner.state_store.snapshot().uavs["uav_1"].position_xy[0]
    assert curr_x > init_x, f"UAV_1 failed to move: initial x={init_x}, current x={curr_x}"


def test_4_uav2_remains_staged_until_uav1_clears():
    """Requirement 4: UAV_2 remains staged until UAV_1 clears."""
    scenario = build_8uav_scenario()
    runner = MissionRunner(scenario=scenario, seed=42)
    # Step until UAV_1 is active but not yet cleared
    for _ in range(3):
        runner.step()

    seq = runner.departure_sequencer
    snap = runner.state_store.snapshot()
    uav1 = snap.uavs["uav_1"]
    uav2 = snap.uavs["uav_2"]

    # Verify uav_1 is moving east, uav_2 has not departed
    assert seq.active_departing_uav_id == "uav_1"
    assert seq.uav_phases["uav_2"] in (UAVDeparturePhase.STAGED, UAVDeparturePhase.QUEUED)
    assert uav2.position_xy[0] == -75.0, f"UAV_2 prematurely moved: x={uav2.position_xy[0]}"


def test_5_all_8_uavs_eventually_depart():
    """Requirement 5: All 8 UAVs eventually depart in a feasible scenario."""
    scenario = build_8uav_scenario()
    runner = MissionRunner(scenario=scenario, seed=42)
    result = runner.run()
    seq = runner.departure_sequencer
    assert seq is not None
    assert seq.total_departures_started == 8, f"Expected 8 started departures, got {seq.total_departures_started}"
    assert seq.total_departures_completed == 8, f"Expected 8 completed departures, got {seq.total_departures_completed}"


def test_6_zero_separation_violations_during_departure():
    """Requirement 6: Zero separation violations during departure."""
    scenario = build_8uav_scenario()
    runner = MissionRunner(scenario=scenario, seed=42)
    result = runner.run()
    metrics = result.metrics_report
    assert metrics is not None
    assert metrics.separation_violation_count == 0, f"Got {metrics.separation_violation_count} separation violations"
    seq = runner.departure_sequencer
    assert seq is not None
    assert seq.min_observed_departure_separation_m >= 20.0, f"Min departure separation was {seq.min_observed_departure_separation_m:.2f}m < 20m"


def test_7_zero_takeoff_location_violations():
    """Requirement 7: Zero takeoff-location violations (all depart from x=-75.0)."""
    scenario = build_8uav_scenario()
    runner = MissionRunner(scenario=scenario, seed=42)
    result = runner.run()
    seq = runner.departure_sequencer
    assert seq is not None
    for rec in seq.departure_records.values():
        assert rec.staging_position[0] == -75.0, f"UAV {rec.uav_id} staged at invalid x={rec.staging_position[0]}"


def test_8_zero_geofence_violations_during_taxi():
    """Requirement 8: Zero geofence violations during taxi."""
    scenario = build_8uav_scenario()
    runner = MissionRunner(scenario=scenario, seed=42)
    result = runner.run()
    metrics = result.metrics_report
    assert metrics is not None
    assert metrics.geofence_violation_count == 0, f"Got {metrics.geofence_violation_count} geofence violations"


def test_9_departure_order_is_deterministic():
    """Requirement 9: Departure order is deterministic."""
    scenario = build_8uav_scenario()
    runner1 = MissionRunner(scenario=scenario, seed=42)
    runner1.run()
    order1 = [e.entity_id for e in runner1.all_events if e.event_type == EventType.DEPARTURE_STARTED]

    runner2 = MissionRunner(scenario=scenario, seed=42)
    runner2.run()
    order2 = [e.entity_id for e in runner2.all_events if e.event_type == EventType.DEPARTURE_STARTED]

    assert len(order1) == 8
    assert order1 == order2, f"Departure order mismatch: {order1} vs {order2}"
