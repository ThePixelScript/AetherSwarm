"""Comprehensive unit and integration tests for deterministic 2D geofence enforcement (Stage 1).

Tests A through L:
A. Seed 2026 corridor clipping elimination -> 0 geofence violations, 0 separation violations, 10/10 tasks, 0 battery exhaustions.
B. Normal central ingress -> direct route preserved, 0 unnecessary interventions.
C. Southern corridor ingress -> steered toward southern portal, continuous corridor containment.
D. Northern arena egress -> steered toward northern portal, prevents reflex corner clipping.
E. Southern arena egress -> steered toward southern portal, prevents reflex corner clipping.
F. Arena boundary overshoot -> continuous trajectory truncated cleanly at exterior boundary.
G. Corridor boundary overshoot -> continuous trajectory truncated cleanly at exterior boundary.
H. Staging pad containment -> circular staging pad boundary and clearance respected.
I. Continuous containment sweep -> sub-step sampling verifies no boundary breaches across full step.
J. Combined geofence + 20m separation -> simultaneous boundary & separation safety verified.
K. Disabled enforcement preserves baseline -> geofence enforcer absent when disabled.
L. Deterministic replay -> two independent executions produce bitwise identical results.
"""
from __future__ import annotations

from dataclasses import replace
import math
from types import MappingProxyType
import pytest

from ares_swarm.core.commands import StepPhysicsCommand
from ares_swarm.core.enums import EventType, Role, RTHState, TaskStatus
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState
from ares_swarm.safety.airspace import ChallengeAirspace, FlightPhase
from ares_swarm.safety.geofence import GeofenceEnforcer
from ares_swarm.safety.separation import SeparationEnforcer
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import load_scenario


def _make_airspace() -> ChallengeAirspace:
    return ChallengeAirspace(
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        corridor_bounds_x=(-100.0, 0.0),
        corridor_bounds_y=(400.0, 600.0),
        staging_pad_center=(-75.0, 500.0),
        staging_pad_radius_m=15.0,
        max_height=100.0,
    )


def _make_snapshot(
    uavs: dict[str, UAVState],
    tasks: dict[str, TaskState] | None = None,
    tick: int = 0,
    sim_time: float = 0.0,
    gcs: tuple[float, float] = (-75.0, 500.0),
) -> StateSnapshot:
    return StateSnapshot(
        simulation_tick=tick,
        simulation_time=sim_time,
        state_version=0,
        uavs=MappingProxyType(uavs),
        tasks=MappingProxyType(tasks or {}),
        gcs_position=gcs,
    )


# --- Test A: Seed 2026 Corridor Clipping Regression Test ---
def test_a_seed_2026_corridor_clipping_eliminated():
    """Test A: Challenge Seed 2026 with geofence enforcement eliminates corridor boundary violations."""
    scenario = load_scenario("results/random/random_seed_2026.yaml")
    # Enable both separation enforcement and geofence enforcement
    scenario = replace(
        scenario,
        challenge_profile=replace(
            scenario.challenge_profile,
            enforce_geofence=True,
            enforce_separation=True,
        ),
    )

    runner = MissionRunner(scenario=scenario, seed=2026)
    result = runner.run()
    metrics = result.metrics_report

    assert metrics is not None
    # Hard safety constraints
    assert metrics.geofence_violation_count == 0, f"Expected 0 geofence violations, got {metrics.geofence_violation_count}"
    assert metrics.separation_violation_count == 0, f"Expected 0 separation violations, got {metrics.separation_violation_count}"
    assert metrics.battery_exhaustion_count == 0, f"Expected 0 battery exhaustions, got {metrics.battery_exhaustion_count}"
    assert metrics.tasks_completed == 10, f"Expected 10/10 tasks, got {metrics.tasks_completed}"

    # Geofence interventions occurred and cleared boundaries
    assert metrics.geofence_interventions > 0, "Expected geofence interventions to steer clear of corridor wall"
    assert metrics.min_boundary_clearance_m is not None
    assert metrics.min_boundary_clearance_m >= 0.0, f"Min clearance should be non-negative: {metrics.min_boundary_clearance_m}"

    # All active UAVs return to staging pad
    airspace = runner.safety_assessor.airspace
    for uid, uav in result.final_snapshot.uavs.items():
        assert airspace.is_in_staging_pad(uav.position_xy), f"UAV {uid} not on staging pad: {uav.position_xy}"
        assert uav.battery_energy > 0.0, f"UAV {uid} depleted battery"


# --- Test B: Normal Central Ingress ---
def test_b_central_ingress_direct_route():
    """Test B: Straight-line ingress near corridor center incurs no interventions."""
    airspace = _make_airspace()
    enforcer = GeofenceEnforcer(airspace=airspace, margin_m=1.0)

    uav = UAVState(
        id="uav_center",
        position_xy=(-50.0, 500.0),
        target_position=(500.0, 500.0),
        battery_capacity=100.0,
        battery_energy=100.0,
    )

    p_eff, v_eff, event = enforcer.compute_effective_movement(
        uav=uav,
        configured_speed=15.0,
        dt=1.0,
        tick=1,
        sim_time=1.0,
    )

    # Crosses x=0 at y=500, perfectly inside [401, 599]
    assert event is None
    assert enforcer.total_interventions == 0
    assert p_eff == (-35.0, 500.0)
    assert v_eff == (15.0, 0.0)


# --- Test C: Southern Corridor Ingress Portal Steering ---
def test_c_southern_ingress_portal_steering():
    """Test C: Southern ingress trajectory clipping corridor wall is steered toward southern portal."""
    airspace = _make_airspace()
    enforcer = GeofenceEnforcer(airspace=airspace, margin_m=1.0)

    # Ingress from (-75, 420) toward (500, 200). Straight line intersects x=0 at y ~ 391.3 (< 400)
    uav = UAVState(
        id="uav_south",
        position_xy=(-75.0, 420.0),
        target_position=(500.0, 200.0),
        battery_capacity=100.0,
        battery_energy=100.0,
    )

    p_eff, v_eff, event = enforcer.compute_effective_movement(
        uav=uav,
        configured_speed=15.0,
        dt=1.0,
        tick=1,
        sim_time=1.0,
    )

    assert event is not None
    assert event.event_type == EventType.GEOFENCE_INTERVENTION
    assert event.payload["intervention_type"] == "INGRESS_PORTAL_SOUTH"
    # Position must remain inside corridor
    assert p_eff[0] < 0.0
    assert p_eff[1] >= 400.0
    assert airspace.is_in_corridor(p_eff)


# --- Test D: Northern Arena Egress Portal Steering ---
def test_d_northern_egress_portal_steering():
    """Test D: Northern egress from arena toward corridor is steered toward northern portal."""
    airspace = _make_airspace()
    enforcer = GeofenceEnforcer(airspace=airspace, margin_m=1.0)

    # From (50, 750) heading to (-75, 450). Intersection with x=0 is y = 630 > 600
    uav = UAVState(
        id="uav_egress_north",
        position_xy=(50.0, 750.0),
        target_position=(-75.0, 450.0),
        battery_capacity=100.0,
        battery_energy=100.0,
    )

    p_eff, v_eff, event = enforcer.compute_effective_movement(
        uav=uav,
        configured_speed=15.0,
        dt=1.0,
        tick=1,
        sim_time=1.0,
    )

    assert event is not None
    assert event.payload["intervention_type"] == "EGRESS_PORTAL_NORTH"
    # Target was steered toward (0.0, 599.0)
    assert p_eff[0] <= 50.0
    assert p_eff[1] <= 750.0
    assert airspace.is_in_arena(p_eff)


# --- Test E: Southern Arena Egress Portal Steering ---
def test_e_southern_egress_portal_steering():
    """Test E: Southern egress from arena toward corridor is steered toward southern portal."""
    airspace = _make_airspace()
    enforcer = GeofenceEnforcer(airspace=airspace, margin_m=1.0)

    # From (50, 250) heading to (-75, 550). Intersection with x=0 is y = 130 < 400
    uav = UAVState(
        id="uav_egress_south",
        position_xy=(50.0, 250.0),
        target_position=(-75.0, 550.0),
        battery_capacity=100.0,
        battery_energy=100.0,
    )

    p_eff, v_eff, event = enforcer.compute_effective_movement(
        uav=uav,
        configured_speed=15.0,
        dt=1.0,
        tick=1,
        sim_time=1.0,
    )

    assert event is not None
    assert event.payload["intervention_type"] == "EGRESS_PORTAL_SOUTH"
    assert airspace.is_in_arena(p_eff)


# --- Test F: Arena Boundary Overshoot ---
def test_f_arena_boundary_overshoot():
    """Test F: Target position beyond arena boundary is truncated continuously."""
    airspace = _make_airspace()
    enforcer = GeofenceEnforcer(airspace=airspace, margin_m=1.0)

    # UAV near eastern boundary heading east beyond 1000m
    uav = UAVState(
        id="uav_east",
        position_xy=(995.0, 500.0),
        target_position=(1050.0, 500.0),
        battery_capacity=100.0,
        battery_energy=100.0,
    )

    p_eff, v_eff, event = enforcer.compute_effective_movement(
        uav=uav,
        configured_speed=15.0,
        dt=1.0,
        tick=1,
        sim_time=1.0,
    )

    assert event is not None
    assert event.payload["intervention_type"] in ("BOUNDARY_TRUNCATE", "BOUNDARY_HOLD")
    assert p_eff[0] <= 1000.0
    assert abs(p_eff[0] - 1000.0) < 1e-6
    assert airspace.is_in_arena(p_eff)


# --- Test G: Corridor Boundary Overshoot ---
def test_g_corridor_boundary_overshoot():
    """Test G: Target position beyond corridor boundary is truncated cleanly."""
    airspace = _make_airspace()
    enforcer = GeofenceEnforcer(airspace=airspace, margin_m=1.0)

    # UAV near northern corridor boundary heading north beyond 600m
    uav = UAVState(
        id="uav_north_corridor",
        position_xy=(-50.0, 595.0),
        target_position=(-50.0, 620.0),
        battery_capacity=100.0,
        battery_energy=100.0,
    )

    p_eff, v_eff, event = enforcer.compute_effective_movement(
        uav=uav,
        configured_speed=15.0,
        dt=1.0,
        tick=1,
        sim_time=1.0,
    )

    assert event is not None
    assert p_eff[1] <= 600.0
    assert abs(p_eff[1] - 600.0) < 1e-6
    assert airspace.is_in_corridor(p_eff)


# --- Test H: Staging Pad Boundary Clearance ---
def test_h_staging_pad_containment():
    """Test H: Staging pad boundary clearance and circular expansion are respected."""
    airspace = _make_airspace()
    enforcer = GeofenceEnforcer(airspace=airspace, margin_m=1.0)

    # GCS position is (-75, 500)
    clearance_center = enforcer.compute_boundary_clearance((-75.0, 500.0))
    # Distance to north corridor wall (600 - 500 = 100), south (500 - 400 = 100), west (-75 - (-100) = 25)
    assert clearance_center == 25.0

    # At pad perimeter (-90, 500): clearance should be 10m to west boundary
    clearance_west = enforcer.compute_boundary_clearance((-90.0, 500.0))
    assert clearance_west == 10.0


# --- Test I: Continuous Containment Sweep ---
def test_i_continuous_containment_sweep():
    """Test I: Trajectory interpolation verifies no point breaches boundary across full step."""
    airspace = _make_airspace()
    enforcer = GeofenceEnforcer(airspace=airspace, margin_m=1.0)

    test_cases = [
        # Ingress from various corridor spots
        ((-80.0, 580.0), (300.0, 750.0)),
        ((-70.0, 410.0), (400.0, 200.0)),
        ((-10.0, 595.0), (500.0, 800.0)),
        # Egress from arena to corridor
        ((10.0, 650.0), (-75.0, 500.0)),
        ((20.0, 350.0), (-75.0, 500.0)),
        # Intra-corridor edge cases
        ((-50.0, 598.0), (-50.0, 650.0)),
        ((-50.0, 402.0), (-50.0, 350.0)),
    ]

    for p0, p_tgt in test_cases:
        uav = UAVState(
            id="uav_test",
            position_xy=p0,
            target_position=p_tgt,
            battery_capacity=100.0,
            battery_energy=100.0,
        )
        p1, v1, _ = enforcer.compute_effective_movement(
            uav=uav,
            configured_speed=15.0,
            dt=1.0,
            tick=1,
            sim_time=1.0,
        )

        # Sample 20 intermediate points along the step
        for step_idx in range(21):
            alpha = step_idx / 20.0
            p_interp = (p0[0] + alpha * (p1[0] - p0[0]), p0[1] + alpha * (p1[1] - p0[1]))
            assert airspace.is_in_authorized_union(p_interp), (
                f"Trajectory from {p0} to {p1} breached geofence at alpha={alpha}, point={p_interp}"
            )


# --- Test J: Combined Geofence and 20m Separation ---
def test_j_combined_geofence_and_separation():
    """Test J: Simultaneous geofence portal steering and 20m separation enforcement."""
    airspace = _make_airspace()
    geo_enforcer = GeofenceEnforcer(airspace=airspace, margin_m=1.0)
    sep_enforcer = SeparationEnforcer(min_separation_m=20.0, gcs_position=(-75.0, 500.0))

    # Two UAVs entering northern portal simultaneously, separated >= 20m initially (dist = 25m)
    u1 = UAVState(
        id="uav_1",
        position_xy=(-5.0, 595.0),
        target_position=(400.0, 750.0),
        battery_capacity=100.0,
        battery_energy=100.0,
        role=Role.SCOUT,
        assigned_task_id="t1",
    )
    u2 = UAVState(
        id="uav_2",
        position_xy=(-30.0, 595.0),
        target_position=(400.0, 750.0),
        battery_capacity=100.0,
        battery_energy=100.0,
        role=Role.SCOUT,
        assigned_task_id="t2",
    )

    snap = _make_snapshot(uavs={"uav_1": u1, "uav_2": u2})

    cmds, events = sep_enforcer.enforce_step(
        snapshot=snap,
        configured_speed=15.0,
        dt=1.0,
        idle_rate=0.01,
        movement_rate=0.05,
        geofence_enforcer=geo_enforcer,
    )

    cmd_map = {cmd.uav_id: cmd for cmd in cmds}
    p1 = cmd_map["uav_1"].new_position_xy
    p2 = cmd_map["uav_2"].new_position_xy

    # 1. Geofence containment: neither UAV breaches boundaries
    if p1[0] < 0.0:
        assert p1[1] <= 600.0
    if p2[0] < 0.0:
        assert p2[1] <= 600.0
    assert airspace.is_in_authorized_union(p1)
    assert airspace.is_in_authorized_union(p2)

    # 2. Separation: distance remains >= 20.0m
    dist = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
    assert dist >= 20.0 - 1e-4, f"Separation violated: {dist} < 20.0m"


# --- Test K: Disabled Enforcement Preserves Baseline ---
def test_k_disabled_preserves_baseline():
    """Test K: When enforce_geofence is False, geofence_enforcer is None."""
    scenario = load_scenario("results/random/random_seed_2026.yaml")
    # Separation True, Geofence False
    scenario = replace(
        scenario,
        challenge_profile=replace(
            scenario.challenge_profile,
            enforce_geofence=False,
            enforce_separation=True,
        ),
    )

    runner = MissionRunner(scenario=scenario, seed=2026)
    assert runner.geofence_enforcer is None
    # Run 5 ticks
    for _ in range(5):
        runner.step()
    res = runner.state_store.snapshot()
    assert res.simulation_tick == 5


# --- Test L: Deterministic Replay ---
def test_l_deterministic_replay():
    """Test L: Two independent runs with geofence enforcement produce bitwise identical results."""
    scenario = load_scenario("results/random/random_seed_2026.yaml")
    scenario = replace(
        scenario,
        challenge_profile=replace(
            scenario.challenge_profile,
            enforce_geofence=True,
            enforce_separation=True,
        ),
    )

    runner1 = MissionRunner(scenario=scenario, seed=2026)
    res1 = runner1.run(max_ticks=200)

    runner2 = MissionRunner(scenario=scenario, seed=2026)
    res2 = runner2.run(max_ticks=200)

    dict1 = res1.to_dict()
    dict2 = res2.to_dict()

    assert dict1["metrics"] == dict2["metrics"]
    assert dict1["evaluation"] == dict2["evaluation"]
    assert dict1["safety_interventions"] == dict2["safety_interventions"]
    assert dict1["geofence_interventions"] == dict2["geofence_interventions"]
    assert len(res1.all_events) == len(res2.all_events)
    for e1, e2 in zip(res1.all_events, res2.all_events):
        assert e1.event_type == e2.event_type
        assert e1.entity_id == e2.entity_id
        assert e1.payload == e2.payload
