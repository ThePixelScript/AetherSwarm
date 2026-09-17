"""Unit tests for deterministic SafetyAssessor and RTH prevention."""
import pytest
from types import MappingProxyType

from ares_swarm.core.enums import FailureState, Role, RTHState, TaskStatus
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState
from ares_swarm.safety.safety_assessor import SafetyAssessor, SafetyReport, SafetyViolation


def test_geofence_violation_detection():
    assessor = SafetyAssessor(
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        gcs_position=(-50.0, 500.0),
    )

    # u1 is inside, u2 is outside geofence (x > 1000), u3 is outside (x < 0, but in RTH flying to GCS)
    u1 = UAVState(id="u1", position_xy=(500.0, 500.0), battery_capacity=100.0, battery_energy=100.0)
    u2 = UAVState(id="u2", position_xy=(1050.0, 500.0), battery_capacity=100.0, battery_energy=100.0)
    u3 = UAVState(id="u3", position_xy=(-20.0, 500.0), battery_capacity=100.0, battery_energy=100.0, rth_state=RTHState.ACTIVE)

    snap = StateSnapshot(
        simulation_tick=1,
        simulation_time=1.0,
        state_version=1,
        uavs=MappingProxyType({"u1": u1, "u2": u2, "u3": u3}),
        tasks=MappingProxyType({}),
        gcs_position=(-50.0, 500.0),
    )

    violations = assessor.assess_snapshot(snap)
    assert len(violations) == 1
    assert violations[0].violation_type == "GEOFENCE"
    assert violations[0].entity_ids == ("u2",)
    assert assessor.report.geofence_violations_count == 1


def test_inter_uav_separation_violation_detection():
    assessor = SafetyAssessor(
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        min_separation_m=20.0,
    )

    # u1 and u2 are separated by 10m (< 20m minimum constraint)
    u1 = UAVState(id="u1", position_xy=(100.0, 100.0), battery_capacity=100.0, battery_energy=100.0)
    u2 = UAVState(id="u2", position_xy=(110.0, 100.0), battery_capacity=100.0, battery_energy=100.0)
    u3 = UAVState(id="u3", position_xy=(200.0, 100.0), battery_capacity=100.0, battery_energy=100.0)

    snap = StateSnapshot(
        simulation_tick=5,
        simulation_time=5.0,
        state_version=5,
        uavs=MappingProxyType({"u1": u1, "u2": u2, "u3": u3}),
        tasks=MappingProxyType({}),
        gcs_position=(0.0, 0.0),
    )

    violations = assessor.assess_snapshot(snap)
    assert len(violations) == 1
    assert violations[0].violation_type == "SEPARATION"
    assert set(violations[0].entity_ids) == {"u1", "u2"}
    assert assessor.report.separation_violations_count == 1
    assert assessor.report.min_observed_separation_m == pytest.approx(10.0)


def test_battery_exhaustion_detection():
    assessor = SafetyAssessor(
        gcs_position=(0.0, 0.0),
    )

    # u1 has zero battery far from GCS (at (100, 100))
    u1 = UAVState(id="u1", position_xy=(100.0, 100.0), battery_capacity=100.0, battery_energy=0.0)
    # u2 has zero battery but is at GCS (0, 0)
    u2 = UAVState(id="u2", position_xy=(0.0, 0.0), battery_capacity=100.0, battery_energy=0.0)

    snap = StateSnapshot(
        simulation_tick=10,
        simulation_time=10.0,
        state_version=10,
        uavs=MappingProxyType({"u1": u1, "u2": u2}),
        tasks=MappingProxyType({}),
        gcs_position=(0.0, 0.0),
    )

    violations = assessor.assess_snapshot(snap)
    assert len(violations) == 1
    assert violations[0].violation_type == "BATTERY_EXHAUSTION"
    assert violations[0].entity_ids == ("u1",)
    assert assessor.report.battery_exhaustions_count == 1


def test_evaluate_rth_triggers():
    assessor = SafetyAssessor(
        gcs_position=(0.0, 0.0),
        rth_energy_buffer=1.2,
    )

    # UAV at (100, 0), dist = 100m.
    # At speed 5 m/s, return time = 20s.
    # Return energy = (1.0 * 20 + 0.5 * 100) * 1.2 = (20 + 50) * 1.2 = 84.0 Wh.
    # u1 has 80 Wh (< 84 Wh) -> battery trigger!
    u1 = UAVState(id="u1", position_xy=(100.0, 0.0), battery_capacity=100.0, battery_energy=80.0)
    # u2 has 95 Wh (> 84 Wh) -> no battery trigger
    u2 = UAVState(id="u2", position_xy=(100.0, 0.0), battery_capacity=100.0, battery_energy=95.0)

    snap = StateSnapshot(
        simulation_tick=50,
        simulation_time=50.0,
        state_version=50,
        uavs=MappingProxyType({"u1": u1, "u2": u2}),
        tasks=MappingProxyType({}),
        gcs_position=(0.0, 0.0),
    )

    cmds = assessor.evaluate_rth_triggers(
        snap,
        speed_limit=5.0,
        idle_rate=1.0,
        movement_rate=0.5,
        mission_duration=1000.0,
        enable_battery_rth=True,
        enable_time_rth=False,
    )
    assert len(cmds) == 1
    assert cmds[0].uav_id == "u1"

    # Now test time trigger: mission duration 70s, sim_time 50s => time_left 20s <= 20 * 1.2 = 24s
    cmds_time = assessor.evaluate_rth_triggers(
        snap,
        speed_limit=5.0,
        idle_rate=1.0,
        movement_rate=0.5,
        mission_duration=70.0,
        enable_battery_rth=False,
        enable_time_rth=True,
    )
    # Both u1 and u2 need to return before mission ends
    assert len(cmds_time) == 2
    assert {c.uav_id for c in cmds_time} == {"u1", "u2"}
