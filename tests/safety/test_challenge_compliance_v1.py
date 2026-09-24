"""Focused unit and integration tests for Challenge Compliance Layer V1.

Verifies:
A. Staging position accepted
B. Ingress corridor accepted
C. Mission arena accepted
D. Illegal position rejected
E. RTH corridor accepted
F. Landing outside staging region rejected
G. 1200s sortie tracking (takeoff, in-flight, touchdown)
H. Over-duration detection (FLIGHT_DURATION violation)
I. RTH triggered early enough (sortie limit - transit time - safety margin)
J. Legacy E1 behavior remains unchanged
K. Staged assigned-but-not-moving UAV is NOT airborne (takeoff requires motion)
L. Single-sortie policy rejects relaunch after landing (RELAUNCH_PROHIBITED)
M. End-to-end integration: takeoff -> flight -> dynamic RTH -> touchdown <= 1200s
"""
from types import MappingProxyType
import pytest

from ares_swarm.core.commands import AssignTaskCommand, SetTargetPositionCommand
from ares_swarm.core.enums import EventType, Role, RTHState, TaskStatus
from ares_swarm.core.models import StateSnapshot, UAVState
from ares_swarm.safety.airspace import ChallengeAirspace, FlightPhase
from ares_swarm.safety.safety_assessor import SafetyAssessor, SafetyReport, UAVFlightRecord
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import (
    ChallengeAirspaceConfig,
    ChallengeProfileConfig,
    ScenarioConfig,
    load_scenario,
)


def _make_snapshot(
    uavs: dict[str, UAVState],
    tick: int = 0,
    sim_time: float = 0.0,
    gcs: tuple[float, float] = (-75.0, 500.0),
) -> StateSnapshot:
    return StateSnapshot(
        simulation_tick=tick,
        simulation_time=sim_time,
        state_version=0,
        uavs=MappingProxyType(uavs),
        tasks=MappingProxyType({}),
        gcs_position=gcs,
    )


def test_a_staging_position_accepted():
    """Test A: Staging position at [-75.0, 500.0] is accepted without violations."""
    airspace = ChallengeAirspace(
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        staging_pad_center=(-75.0, 500.0),
        staging_pad_radius_m=15.0,
        corridor_bounds_x=(-75.0, 0.0),
        corridor_bounds_y=(400.0, 600.0),
    )
    valid, reason = airspace.validate_position((-75.0, 500.0), FlightPhase.STAGING)
    assert valid is True
    assert reason is None

    # Slightly offset on pad
    valid, reason = airspace.validate_position((-70.0, 505.0), FlightPhase.STAGING)
    assert valid is True


def test_b_ingress_corridor_accepted():
    """Test B: Ingress transit corridor [-75, 0] x [400, 600] is accepted."""
    airspace = ChallengeAirspace(corridor_bounds_y=(400.0, 600.0))
    valid, reason = airspace.validate_position((-30.0, 500.0), FlightPhase.INGRESS)
    assert valid is True
    assert reason is None


def test_c_mission_arena_accepted():
    """Test C: Primary arena [0, 1000] x [0, 1000] is accepted for MISSION phase."""
    airspace = ChallengeAirspace()
    valid, reason = airspace.validate_position((500.0, 500.0), FlightPhase.MISSION)
    assert valid is True
    assert reason is None


def test_d_illegal_position_rejected():
    """Test D: Positions outside authorized airspace union are rejected."""
    airspace = ChallengeAirspace(corridor_bounds_y=(400.0, 600.0))
    # Deep south outside corridor
    valid, reason = airspace.validate_position((-75.0, 200.0), FlightPhase.STAGING)
    assert valid is False
    assert "outside authorized airspace union" in reason

    # West behind pad
    valid, reason = airspace.validate_position((-120.0, 500.0), FlightPhase.INGRESS)
    assert valid is False

    # Mission phase leaving arena into corridor
    valid, reason = airspace.validate_position((-30.0, 500.0), FlightPhase.MISSION)
    assert valid is False
    assert "departed operational arena" in reason


def test_e_rth_corridor_accepted():
    """Test E: RTH/EGRESS phase is authorized inside the transit corridor."""
    airspace = ChallengeAirspace(corridor_bounds_y=(400.0, 600.0))
    valid, reason = airspace.validate_position((-40.0, 510.0), FlightPhase.EGRESS)
    assert valid is True
    assert reason is None


def test_f_landing_outside_staging_region_rejected():
    """Test F: Landing outside the designated staging pad triggers a violation."""
    airspace = ChallengeAirspace(
        staging_pad_center=(-75.0, 500.0),
        staging_pad_radius_m=15.0,
    )
    # Landing inside arena center
    valid, reason = airspace.validate_position((500.0, 500.0), FlightPhase.LANDED)
    assert valid is False
    assert "outside staging pad" in reason

    # Assessor records violation on touch down outside pad
    assessor = SafetyAssessor(
        gcs_position=(-75.0, 500.0),
        airspace=airspace,
    )
    u1 = UAVState(
        id="uav_1",
        position_xy=(500.0, 500.0),
        rth_state=RTHState.COMPLETE,
        active=False,
    )
    snap = _make_snapshot({"uav_1": u1}, tick=10, sim_time=10.0)
    # Pre-populate prior airborne state
    assessor.report.uav_flight_records["uav_1"] = UAVFlightRecord(
        uav_id="uav_1",
        is_airborne=True,
        takeoff_time=0.0,
    )

    violations = assessor.assess_snapshot(snap)
    landing_viols = [v for v in violations if v.violation_type == "LANDING_LOCATION"]
    assert len(landing_viols) == 1
    assert "outside authorized staging pad" in landing_viols[0].details


def test_g_1200s_sortie_tracking():
    """Test G: Accurate tracking of takeoff, in-flight duration, and landing."""
    assessor = SafetyAssessor(
        gcs_position=(-75.0, 500.0),
        airspace=ChallengeAirspace(),
    )
    # Staged at pad
    u1 = UAVState(id="uav_1", position_xy=(-75.0, 500.0), role=Role.IDLE, active=True)
    snap0 = _make_snapshot({"uav_1": u1}, tick=0, sim_time=0.0)
    assessor.assess_snapshot(snap0)
    assert assessor.report.uav_flight_records["uav_1"].is_airborne is False

    # Takeoff at tick 10 (sim_time = 10.0)
    u1_airborne = UAVState(id="uav_1", position_xy=(-50.0, 500.0), velocity_xy=(5.0, 0.0), role=Role.SCOUT, active=True)
    snap1 = _make_snapshot({"uav_1": u1_airborne}, tick=10, sim_time=10.0)
    assessor.assess_snapshot(snap1)
    rec = assessor.report.uav_flight_records["uav_1"]
    assert rec.is_airborne is True
    assert rec.takeoff_time == 10.0

    # In flight at tick 500 (sim_time = 500.0)
    snap2 = _make_snapshot({"uav_1": u1_airborne}, tick=500, sim_time=500.0)
    assessor.assess_snapshot(snap2)
    assert rec.current_sortie_duration_s == 490.0

    # Landing at tick 1000 (sim_time = 1000.0) at staging pad
    u1_landed = UAVState(id="uav_1", position_xy=(-75.0, 500.0), rth_state=RTHState.COMPLETE, active=False)
    snap3 = _make_snapshot({"uav_1": u1_landed}, tick=1000, sim_time=1000.0)
    assessor.assess_snapshot(snap3)
    assert rec.is_airborne is False
    assert rec.landing_time == 1000.0
    assert rec.current_sortie_duration_s == 990.0
    assert rec.cumulative_airborne_s == 990.0
    assert assessor.report.max_observed_sortie_duration_s == 990.0


def test_h_over_duration_detection():
    """Test H: Exceeding max_sortie_duration_s generates a FLIGHT_DURATION violation."""
    assessor = SafetyAssessor(
        gcs_position=(-75.0, 500.0),
        airspace=ChallengeAirspace(),
        max_sortie_duration_s=1200.0,
        enforce_sortie_limit=True,
    )
    # UAV with non-zero velocity in arena
    u1 = UAVState(id="uav_1", position_xy=(500.0, 500.0), velocity_xy=(5.0, 0.0), role=Role.SCOUT, active=True)
    # Takeoff at 0.0
    snap0 = _make_snapshot({"uav_1": u1}, tick=0, sim_time=0.0)
    assessor.assess_snapshot(snap0)

    # In flight at 1205.0s (> 1200.0s)
    snap1 = _make_snapshot({"uav_1": u1}, tick=1205, sim_time=1205.0)
    violations = assessor.assess_snapshot(snap1)
    duration_viols = [v for v in violations if v.violation_type == "FLIGHT_DURATION"]
    assert len(duration_viols) == 1
    assert "exceeded max sortie duration" in duration_viols[0].details
    assert assessor.report.flight_duration_violations_count == 1


def test_i_rth_triggered_early_enough_for_sortie_limit():
    """Test I: RTH triggered when remaining sortie <= required transit time + safety margin."""
    assessor = SafetyAssessor(
        gcs_position=(-75.0, 500.0),
        airspace=ChallengeAirspace(),
        max_sortie_duration_s=1200.0,
        rth_safety_margin_s=15.0,
        enforce_sortie_limit=True,
    )
    # UAV at (500.0, 500.0). Distance to GCS [-75, 500] is 575.0m.
    # At speed 5 m/s: transit_time = 575 / 5 = 115.0s.
    # Required time = 115.0 + 15.0 = 130.0s.
    # If sortie duration = 1060s, remaining = 140s > 130s -> No RTH.
    # If sortie duration = 1075s, remaining = 125s <= 130s -> RTH Triggered!
    u1 = UAVState(id="uav_1", position_xy=(500.0, 500.0), role=Role.SCOUT, active=True)
    assessor.report.uav_flight_records["uav_1"] = UAVFlightRecord(
        uav_id="uav_1",
        is_airborne=True,
        takeoff_time=0.0,
    )

    # At t = 1060s
    assessor.report.uav_flight_records["uav_1"].current_sortie_duration_s = 1060.0
    snap_safe = _make_snapshot({"uav_1": u1}, tick=1060, sim_time=1060.0)
    cmds_safe = assessor.evaluate_rth_triggers(
        snap_safe,
        speed_limit=5.0,
        mission_duration=2700.0,
        enable_sortie_rth=True,
    )
    assert len(cmds_safe) == 0

    # At t = 1075s
    assessor.report.uav_flight_records["uav_1"].current_sortie_duration_s = 1075.0
    snap_trigger = _make_snapshot({"uav_1": u1}, tick=1075, sim_time=1075.0)
    cmds_trigger = assessor.evaluate_rth_triggers(
        snap_trigger,
        speed_limit=5.0,
        mission_duration=2700.0,
        enable_sortie_rth=True,
    )
    assert len(cmds_trigger) == 1
    assert cmds_trigger[0].uav_id == "uav_1"


def test_j_legacy_e1_behavior_unchanged():
    """Test J: Frozen E1 benchmark maintains exact behavior and passes cleanly."""
    scenario = load_scenario("scenarios/poc_round1.yaml")
    assert scenario.challenge_profile.enabled is False

    runner = MissionRunner(scenario)
    result = runner.run()

    summary = result.to_dict()
    metrics = summary["metrics"]
    eval_metrics = summary["evaluation"]

    # E1 golden metrics
    assert metrics["tasks_total"] == 10
    assert metrics["tasks_completed"] == 10
    assert metrics["mission_completion_rate"] == 1.0
    assert eval_metrics["safety"]["geofence_violation_count"] == 0
    assert eval_metrics["safety"]["separation_violation_count"] == 0
    assert eval_metrics["safety"]["min_inter_uav_separation_m"] >= 20.0
    assert result.total_ticks <= 2700


def test_k_staged_assigned_uav_is_not_airborne():
    """Test K: A staged UAV assigned a task but with zero velocity on the pad is NOT airborne."""
    assessor = SafetyAssessor(
        gcs_position=(-75.0, 500.0),
        airspace=ChallengeAirspace(),
    )
    # Staged on pad with assigned task and SCOUT role, but stationary
    u1_staged = UAVState(
        id="uav_1",
        position_xy=(-75.0, 500.0),
        velocity_xy=(0.0, 0.0),
        role=Role.SCOUT,
        assigned_task_id="poi_01",
        active=True,
    )
    snap = _make_snapshot({"uav_1": u1_staged}, tick=5, sim_time=5.0)
    assessor.assess_snapshot(snap)

    rec = assessor.report.uav_flight_records["uav_1"]
    assert rec.is_airborne is False
    assert rec.takeoff_time is None
    assert rec.current_sortie_duration_s == 0.0

    # Now starts moving -> takeoff occurs
    u1_moving = UAVState(
        id="uav_1",
        position_xy=(-75.0, 500.0),
        velocity_xy=(5.0, 0.0),
        role=Role.SCOUT,
        assigned_task_id="poi_01",
        active=True,
    )
    snap2 = _make_snapshot({"uav_1": u1_moving}, tick=6, sim_time=6.0)
    assessor.assess_snapshot(snap2)

    assert rec.is_airborne is True
    assert rec.takeoff_time == 6.0


def test_l_single_sortie_policy_prohibits_relaunch():
    """Test L: Attempting a second sortie after landing generates RELAUNCH_PROHIBITED violation."""
    assessor = SafetyAssessor(
        gcs_position=(-75.0, 500.0),
        airspace=ChallengeAirspace(),
        enforce_single_sortie=True,
    )
    # UAV has landed
    u1_landed = UAVState(
        id="uav_1",
        position_xy=(-75.0, 500.0),
        rth_state=RTHState.COMPLETE,
        active=False,
    )
    snap0 = _make_snapshot({"uav_1": u1_landed}, tick=100, sim_time=100.0)
    assessor.report.uav_flight_records["uav_1"] = UAVFlightRecord(
        uav_id="uav_1",
        is_airborne=False,
        takeoff_time=10.0,
        landing_time=100.0,
        current_sortie_duration_s=90.0,
    )
    assessor.assess_snapshot(snap0)

    # Attempt second sortie: active with velocity
    u1_relaunch = UAVState(
        id="uav_1",
        position_xy=(-70.0, 500.0),
        velocity_xy=(5.0, 0.0),
        role=Role.SCOUT,
        active=True,
    )
    snap1 = _make_snapshot({"uav_1": u1_relaunch}, tick=105, sim_time=105.0)
    violations = assessor.assess_snapshot(snap1)

    relaunch_viols = [v for v in violations if v.violation_type == "RELAUNCH_PROHIBITED"]
    assert len(relaunch_viols) == 1
    assert "attempted secondary sortie" in relaunch_viols[0].details
    assert assessor.report.relaunch_violations_count == 1


def test_m_end_to_end_sortie_compliance_integration():
    """Test M: End-to-end mission integration test verifying actual transit, RTH trigger, and landing under 1200s."""
    # Create deterministic scenario:
    # 1 UAV at [-75, 500], target at [200, 500] (distance 275m).
    # Speed 5 m/s -> transit time = 55s.
    # Max sortie = 1200s, safety margin = 15s -> RTH trigger when remaining sortie <= 55 + 15 = 70s (i.e. at t=1130s).
    # Mission duration = 1500s.
    profile = ChallengeProfileConfig(
        enabled=True,
        max_sortie_duration_s=1200.0,
        rth_safety_margin_s=15.0,
        enforce_sortie_limit=True,
        enforce_single_sortie=True,
        airspace=ChallengeAirspaceConfig(
            enabled=True,
            staging_pad_center=(-75.0, 500.0),
            staging_pad_radius_m=15.0,
            corridor_bounds_x=(-75.0, 0.0),
            corridor_bounds_y=(400.0, 600.0),
            arena_bounds_x=(0.0, 1000.0),
            arena_bounds_y=(0.0, 1000.0),
        ),
    )
    config = ScenarioConfig(
        name="test_sortie_end_to_end",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=1300.0,
        max_ticks=1300,
        gcs_position=(-75.0, 500.0),
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        enable_auto_rth=True,
        return_by_mission_end=False,
        uavs=(
            {
                "id": "uav_1",
                "position": [-75.0, 500.0],
                "battery_capacity": 50000.0,
                "battery_energy": 50000.0,
                "role": "IDLE",
            },
        ),
        tasks=(
            {
                "id": "poi_01",
                "position": [200.0, 500.0],
                "priority": 1,
                "spawn_time": 0.0,
                "deadline_offset": 2000.0,
                "service_duration": 1000.0,
            },
        ),
        challenge_profile=profile,
    )

    runner = MissionRunner(config)
    result = runner.run()

    report = runner.safety_assessor.report
    rec = report.uav_flight_records["uav_1"]

    # 1. UAV took off
    assert rec.takeoff_time is not None
    assert rec.takeoff_time <= 5.0

    # 2. UAV landed before 1200s
    assert rec.landing_time is not None
    assert rec.landing_time <= 1200.0
    assert rec.is_airborne is False

    # 3. Sortie duration accurately equals touchdown - takeoff
    expected_duration = rec.landing_time - rec.takeoff_time
    assert abs(rec.current_sortie_duration_s - expected_duration) < 1e-4

    # 4. Landed safely at GCS
    final_uav = result.final_snapshot.uavs["uav_1"]
    assert final_uav.rth_state == RTHState.COMPLETE
    dist_gcs = ((final_uav.position_xy[0] - (-75.0))**2 + (final_uav.position_xy[1] - 500.0)**2)**0.5
    assert dist_gcs <= 15.0

    # 5. Zero safety violations
    assert report.flight_duration_violations_count == 0
    assert report.landing_violations_count == 0
    assert report.geofence_violations_count == 0
