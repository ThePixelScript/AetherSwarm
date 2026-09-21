"""Comprehensive unit and integration tests for deterministic 2D separation enforcement (Stage 1).

Tests A through K:
A. Clear parallel flight -> unchanged nominal step, 0 interventions.
B. Head-on trajectory -> movement truncated/held before 20m.
C. Continuous trajectory minimum >= 20.0m where endpoints are > 20m but crossing would be < 20m.
D. Deterministic priority ordering (service > transit > RTH > idle; remaining dist; uav_id).
E. 3+ UAV convergence -> all pairwise continuous distances >= 20.0m.
F. Landed UAV exemption -> landed UAV parked on GCS staging pad is exempt.
G. Failed-airborne UAV handling -> treated as static obstacle with 20m safety bubble.
H. Deterministic repeated runs -> bitwise identical results.
I. Disabled enforcement preserves E1 baseline -> 10/10 tasks, 0 violations.
J. Geofence + separation interaction -> holds without exiting airspace boundaries.
K. RTH convergence at GCS -> orderly in-trail queuing without separation violations.
"""
from __future__ import annotations

import math
from types import MappingProxyType
import pytest

from ares_swarm.core.commands import StepPhysicsCommand
from ares_swarm.core.constants import EPSILON
from ares_swarm.core.enums import EventType, FailureState, Role, RTHState, TaskStatus
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState
from ares_swarm.safety.airspace import ChallengeAirspace, FlightPhase
from ares_swarm.safety.safety_assessor import SafetyAssessor
from ares_swarm.safety.separation import SeparationEnforcer, min_continuous_separation
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import load_scenario


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


# --- Test A: Clear parallel flight ---
def test_a_clear_parallel_flight():
    """Test A: Parallel non-conflicting flight maintains nominal step with zero interventions."""
    enforcer = SeparationEnforcer(min_separation_m=20.0)

    u1 = UAVState(
        id="uav_1",
        position_xy=(100.0, 100.0),
        target_position=(115.0, 100.0),
        battery_capacity=100.0,
        battery_energy=100.0,
    )
    u2 = UAVState(
        id="uav_2",
        position_xy=(100.0, 150.0),
        target_position=(115.0, 150.0),
        battery_capacity=100.0,
        battery_energy=100.0,
    )
    snap = _make_snapshot({"uav_1": u1, "uav_2": u2})

    commands, events = enforcer.enforce_step(
        snapshot=snap,
        configured_speed=15.0,
        dt=1.0,
        idle_rate=0.1,
        movement_rate=1.0,
    )

    assert enforcer.total_interventions == 0
    assert len(events) == 0
    assert len(commands) == 2

    # Verify nominal positions reached
    cmd1 = next(c for c in commands if c.uav_id == "uav_1")
    cmd2 = next(c for c in commands if c.uav_id == "uav_2")
    assert cmd1.new_position_xy == pytest.approx((115.0, 100.0))
    assert cmd2.new_position_xy == pytest.approx((115.0, 150.0))


# --- Test B: Head-on trajectory ---
def test_b_head_on_trajectory():
    """Test B: Head-on collision trajectory is truncated/held before 20m separation violation."""
    enforcer = SeparationEnforcer(min_separation_m=20.0)

    # Initial distance = 30m. Head on: each wants to move 10m toward the other.
    # Unconstrained end distance would be 10m (< 20m).
    u1 = UAVState(
        id="uav_1",
        position_xy=(100.0, 100.0),
        target_position=(150.0, 100.0),
        battery_capacity=100.0,
        battery_energy=100.0,
    )
    u2 = UAVState(
        id="uav_2",
        position_xy=(130.0, 100.0),
        target_position=(80.0, 100.0),
        battery_capacity=100.0,
        battery_energy=100.0,
    )
    snap = _make_snapshot({"uav_1": u1, "uav_2": u2})

    commands, events = enforcer.enforce_step(
        snapshot=snap,
        configured_speed=10.0,
        dt=1.0,
        idle_rate=0.1,
        movement_rate=1.0,
    )

    assert enforcer.total_interventions > 0
    assert len(events) > 0

    cmd1 = next(c for c in commands if c.uav_id == "uav_1")
    cmd2 = next(c for c in commands if c.uav_id == "uav_2")

    # Check continuous separation across [0, 1.0]
    min_sep = min_continuous_separation(
        u1.position_xy, cmd1.new_velocity_xy,
        u2.position_xy, cmd2.new_velocity_xy,
        dt=1.0,
    )
    assert min_sep >= 20.0 - 1e-3


# --- Test C: Continuous trajectory minimum crossing ---
def test_c_continuous_trajectory_minimum_crossing():
    """Test C: Orthogonal crossing where endpoints are > 20m apart, but intermediate continuous distance < 20m."""
    enforcer = SeparationEnforcer(min_separation_m=20.0)

    # UAV 1 moves along x: (-30, 0) -> (+30, 0)
    # UAV 2 moves along y: (0, -30) -> (0, +30)
    # At t=0: dist = sqrt(30^2 + 30^2) = 42.43m > 20m
    # At t=1: dist = sqrt(30^2 + 30^2) = 42.43m > 20m
    # At t=0.5: crossing at (0, 0), dist = 0.0m!
    u1 = UAVState(
        id="uav_1",
        position_xy=(-30.0, 0.0),
        target_position=(30.0, 0.0),
        battery_capacity=100.0,
        battery_energy=100.0,
    )
    u2 = UAVState(
        id="uav_2",
        position_xy=(0.0, -30.0),
        target_position=(0.0, 30.0),
        battery_capacity=100.0,
        battery_energy=100.0,
    )
    snap = _make_snapshot({"uav_1": u1, "uav_2": u2})

    commands, events = enforcer.enforce_step(
        snapshot=snap,
        configured_speed=60.0,
        dt=1.0,
        idle_rate=0.1,
        movement_rate=1.0,
    )

    assert enforcer.total_interventions > 0

    cmd1 = next(c for c in commands if c.uav_id == "uav_1")
    cmd2 = next(c for c in commands if c.uav_id == "uav_2")

    min_sep = min_continuous_separation(
        u1.position_xy, cmd1.new_velocity_xy,
        u2.position_xy, cmd2.new_velocity_xy,
        dt=1.0,
    )
    assert min_sep >= 20.0 - 1e-3


# --- Test D: Deterministic priority ordering ---
def test_d_deterministic_priority_ordering():
    """Test D: Priority tier ordering (service > transit > RTH > idle; distance; uav_id)."""
    enforcer = SeparationEnforcer(min_separation_m=20.0, gcs_position=(-75.0, 500.0))

    task1 = TaskState(id="t1", position_xy=(500.0, 500.0), priority=1)
    tasks = {"t1": task1}

    # 1. Active task service: at task location <= 0.05m
    u_service = UAVState(id="u_srv", position_xy=(500.0, 500.0), assigned_task_id="t1", battery_capacity=100.0, battery_energy=100.0)
    key_srv = enforcer.get_priority_key(u_service, tasks)
    assert key_srv[0] == 0

    # 2. Transit to assigned task (rem_dist = 50m)
    u_transit1 = UAVState(id="u_tra1", position_xy=(450.0, 500.0), assigned_task_id="t1", battery_capacity=100.0, battery_energy=100.0)
    key_tra1 = enforcer.get_priority_key(u_transit1, tasks)
    assert key_tra1[0] == 1
    assert key_tra1[1] == pytest.approx(50.0)

    # 3. Transit to assigned task (rem_dist = 100m)
    u_transit2 = UAVState(id="u_tra2", position_xy=(400.0, 500.0), assigned_task_id="t1", battery_capacity=100.0, battery_energy=100.0)
    key_tra2 = enforcer.get_priority_key(u_transit2, tasks)
    assert key_tra2[0] == 1
    assert key_tra2[1] == pytest.approx(100.0)

    # 4. RTH active (rem_dist to GCS (-75, 500) = 75m)
    u_rth = UAVState(id="u_rth", position_xy=(0.0, 500.0), rth_state=RTHState.ACTIVE, battery_capacity=100.0, battery_energy=100.0)
    key_rth = enforcer.get_priority_key(u_rth, tasks)
    assert key_rth[0] == 2
    assert key_rth[1] == pytest.approx(75.0)

    # 5. Idle with target (rem_dist = 20m)
    u_idle = UAVState(id="u_idle", position_xy=(10.0, 10.0), target_position=(30.0, 10.0), battery_capacity=100.0, battery_energy=100.0)
    key_idle = enforcer.get_priority_key(u_idle, tasks)
    assert key_idle[0] == 3
    assert key_idle[1] == pytest.approx(20.0)

    # 6. Tie-breaking on uav_id
    u_tie_a = UAVState(id="uav_a", position_xy=(10.0, 10.0), target_position=(30.0, 10.0), battery_capacity=100.0, battery_energy=100.0)
    u_tie_b = UAVState(id="uav_b", position_xy=(10.0, 10.0), target_position=(30.0, 10.0), battery_capacity=100.0, battery_energy=100.0)
    key_tie_a = enforcer.get_priority_key(u_tie_a, tasks)
    key_tie_b = enforcer.get_priority_key(u_tie_b, tasks)
    assert key_tie_a < key_tie_b

    # Verify overall order
    ordered = sorted([u_idle, u_transit2, u_service, u_rth, u_transit1], key=lambda u: enforcer.get_priority_key(u, tasks))
    assert ordered == [u_service, u_transit1, u_transit2, u_rth, u_idle]


# --- Test E: 3+ UAV convergence ---
def test_e_multi_uav_convergence_3_plus():
    """Test E: Four UAVs converging from cardinal directions all maintain >= 20.0m continuous separation."""
    enforcer = SeparationEnforcer(min_separation_m=20.0)

    # 4 UAVs at distance 25m from center (500, 500)
    u_n = UAVState(id="u_n", position_xy=(500.0, 525.0), target_position=(500.0, 500.0), battery_capacity=100.0, battery_energy=100.0)
    u_s = UAVState(id="u_s", position_xy=(500.0, 475.0), target_position=(500.0, 500.0), battery_capacity=100.0, battery_energy=100.0)
    u_e = UAVState(id="u_e", position_xy=(525.0, 500.0), target_position=(500.0, 500.0), battery_capacity=100.0, battery_energy=100.0)
    u_w = UAVState(id="u_w", position_xy=(475.0, 500.0), target_position=(500.0, 500.0), battery_capacity=100.0, battery_energy=100.0)

    snap = _make_snapshot({"u_n": u_n, "u_s": u_s, "u_e": u_e, "u_w": u_w})

    commands, events = enforcer.enforce_step(
        snapshot=snap,
        configured_speed=15.0,
        dt=1.0,
        idle_rate=0.1,
        movement_rate=1.0,
    )

    cmd_map = {c.uav_id: c for c in commands}
    uav_map = snap.uavs

    # Verify every pair satisfies continuous separation >= 20.0m over [0, dt]
    ids = list(uav_map.keys())
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            id_i, id_j = ids[i], ids[j]
            min_sep = min_continuous_separation(
                uav_map[id_i].position_xy, cmd_map[id_i].new_velocity_xy,
                uav_map[id_j].position_xy, cmd_map[id_j].new_velocity_xy,
                dt=1.0,
            )
            assert min_sep >= 20.0 - 1e-3, f"Pair {id_i} and {id_j} violated separation: {min_sep:.3f}m"


# --- Test F: Landed UAV exemption ---
def test_f_landed_uav_exemption():
    """Test F: Landed UAV parked on GCS staging pad is exempt from separation checks."""
    enforcer = SeparationEnforcer(
        min_separation_m=20.0,
        gcs_position=(-75.0, 500.0),
        staging_pad_radius_m=15.0,
    )

    # u1 is landed at GCS staging pad (distance to GCS = 0.0m)
    u1 = UAVState(
        id="uav_1",
        position_xy=(-75.0, 500.0),
        rth_state=RTHState.COMPLETE,
        active=False,
        battery_capacity=100.0,
        battery_energy=50.0,
    )
    # u2 is airborne hovering/moving nearby on pad (distance to u1 = 8.0m < 20m)
    u2 = UAVState(
        id="uav_2",
        position_xy=(-75.0, 508.0),
        target_position=(-75.0, 500.0),
        rth_state=RTHState.ACTIVE,
        active=True,
        battery_capacity=100.0,
        battery_energy=50.0,
    )

    assert enforcer.is_landed_at_gcs(u1) is True
    assert enforcer.is_landed_at_gcs(u2) is False

    snap = _make_snapshot({"uav_1": u1, "uav_2": u2})

    commands, events = enforcer.enforce_step(
        snapshot=snap,
        configured_speed=5.0,
        dt=1.0,
        idle_rate=0.1,
        movement_rate=1.0,
    )

    # u2 must not be blocked by the landed u1
    cmd2 = next(c for c in commands if c.uav_id == "uav_2")
    assert cmd2.new_velocity_xy != (0.0, 0.0)
    assert enforcer.total_interventions == 0


# --- Test G: Failed airborne UAV handling ---
def test_g_failed_airborne_uav_handling():
    """Test G: In-flight failed UAV is treated as a static obstacle with a 20m safety bubble."""
    enforcer = SeparationEnforcer(min_separation_m=20.0)

    # u1 failed in-flight at (500, 500)
    u1 = UAVState(
        id="uav_1",
        position_xy=(500.0, 500.0),
        failure_state=FailureState.FAILED,
        active=False,
        battery_capacity=100.0,
        battery_energy=0.0,
    )
    # u2 active at (475, 500) heading to (525, 500) directly through u1
    u2 = UAVState(
        id="uav_2",
        position_xy=(475.0, 500.0),
        target_position=(525.0, 500.0),
        active=True,
        battery_capacity=100.0,
        battery_energy=100.0,
    )

    snap = _make_snapshot({"uav_1": u1, "uav_2": u2})

    commands, events = enforcer.enforce_step(
        snapshot=snap,
        configured_speed=15.0,
        dt=1.0,
        idle_rate=0.1,
        movement_rate=1.0,
    )

    assert enforcer.total_interventions > 0
    cmd2 = next(c for c in commands if c.uav_id == "uav_2")

    min_sep = min_continuous_separation(
        u2.position_xy, cmd2.new_velocity_xy,
        u1.position_xy, (0.0, 0.0),
        dt=1.0,
    )
    assert min_sep >= 20.0 - 1e-3


# --- Test H: Deterministic repeated runs ---
def test_h_deterministic_repeated_runs():
    """Test H: Repeated runs produce bitwise identical StepPhysicsCommands and intervention logs."""
    def run_simulation():
        enforcer = SeparationEnforcer(min_separation_m=20.0)
        u1 = UAVState(id="u1", position_xy=(100.0, 100.0), target_position=(140.0, 100.0), battery_capacity=100.0, battery_energy=100.0)
        u2 = UAVState(id="u2", position_xy=(130.0, 100.0), target_position=(90.0, 100.0), battery_capacity=100.0, battery_energy=100.0)
        u3 = UAVState(id="u3", position_xy=(115.0, 120.0), target_position=(115.0, 80.0), battery_capacity=100.0, battery_energy=100.0)
        snap = _make_snapshot({"u1": u1, "u2": u2, "u3": u3})

        history = []
        for tick in range(5):
            cmds, evts = enforcer.enforce_step(
                snapshot=snap,
                configured_speed=10.0,
                dt=1.0,
                idle_rate=0.1,
                movement_rate=1.0,
            )
            step_record = [(c.uav_id, c.new_position_xy, c.new_velocity_xy, c.delta_energy) for c in cmds]
            history.append((step_record, len(evts)))

            # Update snap for next tick
            updated_uavs = {}
            for c in cmds:
                old = snap.uavs[c.uav_id]
                updated_uavs[c.uav_id] = UAVState(
                    id=c.uav_id,
                    position_xy=c.new_position_xy,
                    target_position=old.target_position,
                    battery_capacity=old.battery_capacity,
                    battery_energy=old.battery_energy - c.delta_energy,
                )
            snap = _make_snapshot(updated_uavs, tick=tick + 1, sim_time=float(tick + 1))
        return history

    run1 = run_simulation()
    run2 = run_simulation()
    assert run1 == run2


# --- Test I: Disabled enforcement preserves E1 baseline ---
def test_i_disabled_enforcement_preserves_e1():
    """Test I: When enforce_separation is False (default), standard E1 scenario completes 10/10 without regression."""
    scenario = load_scenario("scenarios/poc_round1.yaml")
    assert scenario.challenge_profile.enforce_separation is False

    runner = MissionRunner(scenario)
    result = runner.run()

    res_dict = result.to_dict()
    assert res_dict["metrics"]["tasks_completed"] == 10
    assert res_dict["metrics"]["tasks_total"] == 10
    assert res_dict["metrics"]["tasks_expired"] == 0
    assert result.safety_report.separation_violations_count == 0
    assert result.safety_report.geofence_violations_count == 0
    assert result.separation_enforcer is None
    if result.metrics_report is not None:
        assert result.metrics_report.separation_violation_count == 0
        assert result.metrics_report.geofence_violation_count == 0
        assert result.metrics_report.separation_interventions == 0


# --- Test J: Geofence and separation interaction ---
def test_j_geofence_separation_interaction():
    """Test J: Separation enforcement obeys geofence boundaries and holds if no safe movement exists inside airspace."""
    airspace = ChallengeAirspace(
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
    )
    enforcer = SeparationEnforcer(min_separation_m=20.0)

    # u1 is at (1.0, 500.0), facing west towards (-10.0, 500.0) which is outside arena
    u1 = UAVState(
        id="uav_1",
        position_xy=(1.0, 500.0),
        target_position=(-10.0, 500.0),
        battery_capacity=100.0,
        battery_energy=100.0,
    )
    snap = _make_snapshot({"uav_1": u1})

    flight_phases = {"uav_1": FlightPhase.MISSION}

    commands, events = enforcer.enforce_step(
        snapshot=snap,
        configured_speed=10.0,
        dt=1.0,
        idle_rate=0.1,
        movement_rate=1.0,
        airspace=airspace,
        flight_phases=flight_phases,
    )

    cmd1 = commands[0]
    # Candidate step would exit geofence -> must be held at current position
    assert cmd1.new_position_xy == (1.0, 500.0)
    assert cmd1.new_velocity_xy == (0.0, 0.0)


# --- Test K: RTH convergence at GCS ---
def test_k_rth_in_trail_queuing_at_gcs():
    """Test K: Multiple UAVs returning to GCS form an in-trail queue without separation violations."""
    enforcer = SeparationEnforcer(
        min_separation_m=20.0,
        gcs_position=(-75.0, 500.0),
        staging_pad_radius_m=15.0,
    )

    # u1 is ahead at (-30.0, 500.0), u2 is trailing at (-5.0, 500.0)
    u1 = UAVState(
        id="uav_1",
        position_xy=(-30.0, 500.0),
        target_position=(-75.0, 500.0),
        rth_state=RTHState.ACTIVE,
        battery_capacity=100.0,
        battery_energy=50.0,
    )
    u2 = UAVState(
        id="uav_2",
        position_xy=(-5.0, 500.0),
        target_position=(-75.0, 500.0),
        rth_state=RTHState.ACTIVE,
        battery_capacity=100.0,
        battery_energy=50.0,
    )
    snap = _make_snapshot({"uav_1": u1, "uav_2": u2})

    # Step physics over several timesteps
    dt = 1.0
    speed = 10.0
    assessor = SafetyAssessor(
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        gcs_position=(-75.0, 500.0),
        min_separation_m=20.0,
    )

    for step in range(8):
        # Assess pre-step snapshot for violations
        violations = assessor.assess_snapshot(snap)
        sep_violations = [v for v in violations if v.violation_type == "SEPARATION"]
        assert len(sep_violations) == 0, f"Step {step} had separation violation: {sep_violations}"

        commands, events = enforcer.enforce_step(
            snapshot=snap,
            configured_speed=speed,
            dt=dt,
            idle_rate=0.1,
            movement_rate=1.0,
        )

        cmd_map = {c.uav_id: c for c in commands}

        # Check continuous separation between active airborne commands if both moving
        if "uav_1" in cmd_map and "uav_2" in cmd_map:
            min_sep = min_continuous_separation(
                snap.uavs["uav_1"].position_xy, cmd_map["uav_1"].new_velocity_xy,
                snap.uavs["uav_2"].position_xy, cmd_map["uav_2"].new_velocity_xy,
                dt=dt,
            )
            assert min_sep >= 20.0 - 1e-3, f"Step {step} continuous separation violated: {min_sep}m"

        # Update snapshot preserving all UAVs
        new_uavs = dict(snap.uavs)
        for c in commands:
            old = snap.uavs[c.uav_id]
            dist_gcs = math.hypot(c.new_position_xy[0] - (-75.0), c.new_position_xy[1] - 500.0)
            rth_state = RTHState.COMPLETE if dist_gcs <= 1.0 else RTHState.ACTIVE
            active = rth_state != RTHState.COMPLETE
            new_uavs[c.uav_id] = UAVState(
                id=c.uav_id,
                position_xy=c.new_position_xy,
                target_position=(-75.0, 500.0) if active else None,
                rth_state=rth_state,
                active=active,
                battery_capacity=old.battery_capacity,
                battery_energy=old.battery_energy - c.delta_energy,
            )
        snap = _make_snapshot(new_uavs, tick=step + 1, sim_time=float(step + 1))

    # Finally assess the last state
    violations = assessor.assess_snapshot(snap)
    sep_violations = [v for v in violations if v.violation_type == "SEPARATION"]
    assert len(sep_violations) == 0
