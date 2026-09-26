"""Deterministic tests for Phase 4: Connectivity-Aware Mission Planning.

Tests:
1. Feasible POI assignment (direct and relay-assisted)
2. Infeasible POI assignment (out of reach or insufficient endurance)
3. Task deferral due to connectivity infeasibility
4. Relay creation enabling a previously infeasible task
5. Relay handoff while task is active
6. Relay RTH triggering communication replan
7. Relay failure triggering communication replan
8. Reporting deadline satisfaction after reconfiguration
9. Deterministic repeatability
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

import pytest

from ares_swarm.autonomy.connectivity_planner import (
    ConnectivityAwarePlanner,
    ConnectivityAwarePlannerConfig,
)
from ares_swarm.autonomy.relay_manager import DynamicRelayManager
from ares_swarm.core.commands import (
    AssignRelayRoleCommand,
    AssignTaskCommand,
    FailUAVCommand,
    HandoffRelayCommand,
    ReleaseTaskCommand,
    SetTargetPositionCommand,
    StartRTHCommand,
)
from ares_swarm.core.enums import (
    EventType,
    FailureState,
    Role,
    RTHState,
    SortieState,
    TaskStatus,
    TelemetryStatus,
)
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import (
    ChallengeAirspaceConfig,
    ChallengeProfileConfig,
    CommunicationConfig,
    DetectionPipelineConfig,
    ScenarioConfig,
)


def make_test_scenario(
    name: str = "conn_plan_test",
    duration: float = 300.0,
    uavs: tuple = (),
    tasks: tuple = (),
    comm_range: float = 100.0,
    enable_conn_planning: bool = True,
) -> ScenarioConfig:
    """Build a deterministic ScenarioConfig for connectivity testing."""
    gcs = (-75.0, 500.0)
    airspace = ChallengeAirspaceConfig(
        enabled=True,
        staging_pad_center=gcs,
        staging_pad_radius_m=15.0,
        corridor_bounds_x=(-75.0, 0.0),
        corridor_bounds_y=(400.0, 600.0),
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        max_height=100.0,
    )
    detect_pipe = DetectionPipelineConfig(
        enabled=True,
        sensor_fov_radius_m=40.0,
        reporting_deadline_s=10.0,
        processing_delay_s=0.0,
    )
    prof = ChallengeProfileConfig(
        enabled=True,
        max_sortie_duration_s=1200.0,
        rth_safety_margin_s=15.0,
        recharge_duration_s=300.0,
        enforce_sortie_limit=True,
        enforce_single_sortie=False,
        enforce_separation=False,
        enforce_geofence=False,
        airspace=airspace,
        detection_pipeline=detect_pipe,
        enable_relay_manager=True,
        enable_connectivity_aware_planning=enable_conn_planning,
    )
    return ScenarioConfig(
        name=name,
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=duration,
        max_ticks=int(duration),
        gcs_position=gcs,
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        min_separation_m=20.0,
        communication=CommunicationConfig(max_range=comm_range, base_latency=5.0, packet_loss=0.0),
        battery_idle_rate=1.0,
        battery_movement_rate=0.5,
        enable_auto_rth=True,
        return_by_mission_end=True,
        enable_relay_manager=True,
        enable_connectivity_aware_planning=enable_conn_planning,
        uavs=uavs,
        tasks=tasks,
        challenge_profile=prof,
    )


def test_feasible_poi_assignment():
    """Verify that a task within direct GCS link is deemed feasible and assigned."""
    planner = ConnectivityAwarePlanner(comm_range=100.0)
    # POI at (10, 500): distance to GCS (-75, 500) is 85m <= 100m direct range
    task = TaskState(id="poi_close", position_xy=(10.0, 500.0), priority=1, service_duration=10.0)
    uav = UAVState(id="uav_1", position_xy=(-75.0, 500.0), role=Role.IDLE, active=True, battery_energy=5000.0)
    snap = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs={"uav_1": uav},
        tasks={"poi_close": task},
        gcs_position=(-75.0, 500.0),
    )

    res = planner.check_task_connectivity_feasibility(task, uav, snap)
    assert res.feasible is True
    assert res.relay_needed is False

    cmds = planner.plan(snap)
    assert len(cmds) == 1
    assert isinstance(cmds[0], AssignTaskCommand)
    assert cmds[0].task_id == "poi_close"
    assert cmds[0].uav_id == "uav_1"
    assert planner.connectivity_feasible_assignments == 1
    assert planner.connectivity_rejected_assignments == 0


def test_infeasible_poi_assignment():
    """Verify that a task deep in the arena beyond relay reach is rejected."""
    planner = ConnectivityAwarePlanner(comm_range=100.0)
    # POI at (800, 800): distance to GCS is ~925m >> 200m (single relay coverage)
    task = TaskState(id="poi_deep", position_xy=(800.0, 800.0), priority=1, service_duration=10.0)
    uav = UAVState(id="uav_1", position_xy=(-75.0, 500.0), role=Role.IDLE, active=True, battery_energy=5000.0)
    snap = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs={"uav_1": uav},
        tasks={"poi_deep": task},
        gcs_position=(-75.0, 500.0),
    )

    res = planner.check_task_connectivity_feasibility(task, uav, snap)
    assert res.feasible is False
    assert "exceeds" in res.reason.lower()


def test_task_deferral_due_to_connectivity():
    """Verify that when no candidate UAV can connect to a POI, the task is deferred."""
    planner = ConnectivityAwarePlanner(comm_range=100.0)
    task = TaskState(id="poi_unreachable", position_xy=(800.0, 800.0), priority=1, service_duration=10.0)
    uav = UAVState(id="uav_1", position_xy=(-75.0, 500.0), role=Role.IDLE, active=True, battery_energy=5000.0)
    snap = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs={"uav_1": uav},
        tasks={"poi_unreachable": task},
        gcs_position=(-75.0, 500.0),
    )

    cmds = planner.plan(snap)
    assert len(cmds) == 0, "Must not assign unreachable task"
    assert planner.connectivity_rejected_assignments >= 1
    assert planner.connectivity_deferred_tasks >= 1


def test_relay_creation_enabling_previously_infeasible_task():
    """Verify that an otherwise disconnected task triggers dynamic relay deployment."""
    planner = ConnectivityAwarePlanner(comm_range=100.0)
    # POI at (100, 500): distance to GCS is 175m > 100m direct range
    task = TaskState(id="poi_mid", position_xy=(100.0, 500.0), priority=1, service_duration=10.0)
    uav_surveyor = UAVState(id="uav_a", position_xy=(-75.0, 500.0), role=Role.IDLE, active=True, battery_energy=5000.0)
    uav_relay_cand = UAVState(id="uav_b", position_xy=(-75.0, 500.0), role=Role.IDLE, active=True, battery_energy=5000.0)

    snap = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs={"uav_a": uav_surveyor, "uav_b": uav_relay_cand},
        tasks={"poi_mid": task},
        gcs_position=(-75.0, 500.0),
    )

    cmds = planner.plan(snap)
    assert len(cmds) >= 2, "Must emit relay assignment and task assignment"

    relay_cmds = [c for c in cmds if isinstance(c, AssignRelayRoleCommand)]
    task_cmds = [c for c in cmds if isinstance(c, AssignTaskCommand)]

    assert len(relay_cmds) == 1
    assert len(task_cmds) == 1
    assert planner.relay_required_for_assignment == 1
    assert planner.connectivity_feasible_assignments == 1

    # Check relay target position is midpoint
    relay_cmd = relay_cmds[0]
    expected_pos = (12.5, 500.0)
    assert relay_cmd.target_position == expected_pos


def test_relay_handoff_while_task_is_active():
    """Verify that when a relay approaches RTH, preemptive handoff preserves task continuity."""
    sc = make_test_scenario(
        name="handoff_while_active",
        duration=250.0,
        uavs=(
            {"id": "uav_a", "position": [100.0, 500.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "SURVEYOR"},
            # UAV B has low battery so it triggers RTH mid-mission
            {"id": "uav_b", "position": [12.5, 500.0], "battery_capacity": 180.0, "battery_energy": 180.0, "role": "RELAY"},
            {"id": "uav_c", "position": [-75.0, 500.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "IDLE"},
        ),
        tasks=(
            {"id": "poi_1", "position": [100.0, 500.0], "priority": 1, "service_duration": 100.0},
        ),
    )

    runner = MissionRunner(scenario=sc, seed=42)
    runner.relay_manager.surveyor_to_relay["uav_a"] = "uav_b"
    runner.relay_manager.relay_to_surveyor["uav_b"] = "uav_a"
    runner.relay_manager.relay_positions["uav_b"] = (12.5, 500.0)

    res = runner.run()
    m = res.metrics_report

    assert m.relay_handoffs >= 1
    assert m.tasks_completed == 1
    assert m.connectivity_availability > 0.95
    assert m.connected_time_after_handoff > 0.0


def test_relay_rth_triggering_replan():
    """Verify that when an active relay RTH cannot be replaced, task is deferred and replan occurs."""
    sc = make_test_scenario(
        name="relay_rth_replan",
        duration=100.0,
        uavs=(
            # Surveyor A servicing task at (100, 500)
            {"id": "uav_a", "position": [100.0, 500.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "SURVEYOR"},
            # Relay B with very low battery, no other UAV exists
            {"id": "uav_b", "position": [12.5, 500.0], "battery_capacity": 50.0, "battery_energy": 50.0, "role": "RELAY"},
        ),
        tasks=(
            {"id": "poi_1", "position": [100.0, 500.0], "priority": 1, "service_duration": 120.0},
        ),
    )

    runner = MissionRunner(scenario=sc, seed=42)
    runner.relay_manager.surveyor_to_relay["uav_a"] = "uav_b"
    runner.relay_manager.relay_to_surveyor["uav_b"] = "uav_a"
    runner.relay_manager.relay_positions["uav_b"] = (12.5, 500.0)

    # Pre-assign task to uav_a
    runner.state_store.apply([
        AssignTaskCommand(source_tick=0, uav_id="uav_a", task_id="poi_1")
    ])

    res = runner.run()
    m = res.metrics_report

    # Replan must be triggered because relay departed without replacement
    replan_events = [e for e in res.all_events if e.event_type == EventType.COMMUNICATION_REPLAN]
    assert len(replan_events) >= 1 or m.communication_induced_replans >= 1

    # Task was safely deferred
    t = res.final_snapshot.tasks["poi_1"]
    assert t.status in (TaskStatus.DEFERRED, TaskStatus.PENDING)


def test_relay_failure_triggering_replan():
    """Verify that hardware failure of active relay without replacement triggers task replanning."""
    sc = make_test_scenario(
        name="relay_failure_replan",
        duration=60.0,
        uavs=(
            {"id": "uav_a", "position": [100.0, 500.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "SURVEYOR"},
            {"id": "uav_b", "position": [12.5, 500.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "RELAY"},
        ),
        tasks=(
            {"id": "poi_1", "position": [100.0, 500.0], "priority": 1, "service_duration": 120.0},
        ),
    )

    runner = MissionRunner(scenario=sc, seed=42)
    runner.relay_manager.surveyor_to_relay["uav_a"] = "uav_b"
    runner.relay_manager.relay_to_surveyor["uav_b"] = "uav_a"
    runner.relay_manager.relay_positions["uav_b"] = (12.5, 500.0)

    runner.state_store.apply([
        AssignTaskCommand(source_tick=0, uav_id="uav_a", task_id="poi_1")
    ])

    # Run for 5 ticks, then inject hardware failure into Relay B
    for _ in range(5):
        runner.step()

    runner.state_store.apply([
        FailUAVCommand(source_tick=5, uav_id="uav_b", failure_state=FailureState.FAILED, reason="Engine flameout")
    ])

    # Run remaining ticks
    for _ in range(20):
        runner.step()

    # Replan must be recorded
    replan_events = [e for e in runner.all_events if e.event_type == EventType.COMMUNICATION_REPLAN]
    assert len(replan_events) >= 1

    t = runner.state_store.snapshot().tasks["poi_1"]
    assert t.status == TaskStatus.DEFERRED


def test_reporting_deadline_after_reconfiguration():
    """Verify that detection report is delivered to GCS within 10s deadline after relay reconfiguration."""
    sc = make_test_scenario(
        name="reporting_after_reconfig",
        duration=120.0,
        uavs=(
            {"id": "uav_a", "position": [100.0, 500.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "SURVEYOR"},
            {"id": "uav_b", "position": [12.5, 500.0], "battery_capacity": 180.0, "battery_energy": 180.0, "role": "RELAY"},
            {"id": "uav_c", "position": [-75.0, 500.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "IDLE"},
        ),
        tasks=(
            {"id": "poi_1", "position": [100.0, 500.0], "priority": 1, "service_duration": 50.0},
        ),
    )

    runner = MissionRunner(scenario=sc, seed=42)
    runner.relay_manager.surveyor_to_relay["uav_a"] = "uav_b"
    runner.relay_manager.relay_to_surveyor["uav_b"] = "uav_a"
    runner.relay_manager.relay_positions["uav_b"] = (12.5, 500.0)

    res = runner.run()
    m = res.metrics_report

    assert m.total_detections >= 1
    assert m.reporting_deadline_success >= 1
    assert m.reporting_deadline_failure == 0
    assert m.reporting_compliance_ratio == 1.0


def test_deterministic_repeatability():
    """Verify that running the connectivity-aware scenario twice produces identical bitwise event logs."""
    sc = make_test_scenario(
        name="determinism_test",
        duration=60.0,
        uavs=(
            {"id": "uav_a", "position": [10.0, 500.0], "battery_capacity": 5000.0, "battery_energy": 5000.0, "role": "IDLE"},
            {"id": "uav_b", "position": [-75.0, 500.0], "battery_capacity": 5000.0, "battery_energy": 5000.0, "role": "IDLE"},
        ),
        tasks=(
            {"id": "poi_1", "position": [10.0, 500.0], "priority": 1, "service_duration": 20.0},
        ),
    )

    runner1 = MissionRunner(scenario=sc, seed=42)
    res1 = runner1.run()

    runner2 = MissionRunner(scenario=sc, seed=42)
    res2 = runner2.run()

    assert res1.total_ticks == res2.total_ticks
    assert res1.simulation_time == res2.simulation_time

    evs1 = [(e.simulation_tick, e.event_type.value, e.entity_id) for e in res1.all_events]
    evs2 = [(e.simulation_tick, e.event_type.value, e.entity_id) for e in res2.all_events]
    assert evs1 == evs2


def test_sensor_fov_configuration_and_multihop_stations():
    """Verify sensor FOV radius parameter reduces effective distance in multihop station calculation."""
    from ares_swarm.autonomy.connectivity_planner import compute_multihop_stations

    gcs = (0.0, 500.0)
    target = (300.0, 500.0)
    eff_range = 95.0

    # 1. Default FOV (0m): distance = 300m -> math.ceil(300 / 95) = 4 hops -> 3 relays
    h0, k0, st0 = compute_multihop_stations(gcs, target, effective_range=eff_range, sensor_fov_radius_m=0.0)
    assert h0 == 4
    assert k0 == 3
    assert len(st0) == 3

    # 2. Research FOV (40m): surveyor distance = 300 - 40 = 260m -> math.ceil(260 / 95) = 3 hops -> 2 relays
    h40, k40, st40 = compute_multihop_stations(gcs, target, effective_range=eff_range, sensor_fov_radius_m=40.0)
    assert h40 == 3
    assert k40 == 2
    assert len(st40) == 2

    # 3. Planner config propagation
    config_default = ConnectivityAwarePlannerConfig()
    assert config_default.sensor_fov_radius_m == 0.0

    planner_fov = ConnectivityAwarePlanner(sensor_fov_radius_m=40.0)
    assert planner_fov.config.sensor_fov_radius_m == 40.0
