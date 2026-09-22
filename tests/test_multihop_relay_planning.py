"""Deterministic test suite for Phase 5B: Multi-Hop Relay-Chain Planning.

Covers the 10 authoritative implementation requirements:
1. test_single_relay_backward_compatibility (D = 175m -> H=2, K=1, station at (12.5, 500))
2. test_two_relay_chain_formation (D = 255m -> H=3, K=2, stations at (10, 500) and (95, 500))
3. test_three_relay_chain_formation (D = 350m -> H=4, K=3, stations computed correctly)
4. test_airborne_relay_reuse (airborne candidate near station prioritized)
5. test_insufficient_relays_clean_deferral (atomic candidate check, 0 UAVs dispatched)
6. test_multihop_intermediate_link_handoff (localized link replacement in active chain)
7. test_multihop_link_failure_recovery (localized failure replacement in active chain)
8. test_multihop_unrecoverable_break_replan (degraded chain triggers task deferral and clean teardown)
9. test_multihop_reporting_deadline_compliance (end-to-end 3-hop delivery within 10s deadline)
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

import pytest

from ares_swarm.autonomy.connectivity_planner import (
    ConnectivityAwarePlanner,
    ConnectivityAwarePlannerConfig,
    compute_multihop_stations,
)
from ares_swarm.autonomy.relay_manager import (
    ChainStatus,
    DynamicRelayManager,
    RelayChain,
    RelayManagementConfig,
)
from ares_swarm.core.commands import (
    AssignRelayRoleCommand,
    AssignTaskCommand,
    Command,
    FailUAVCommand,
    HandoffRelayCommand,
    ReleaseRelayRoleCommand,
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


def make_multihop_scenario(
    name: str = "multihop_test",
    duration: float = 120.0,
    uavs: tuple = (),
    tasks: tuple = (),
    comm_range: float = 100.0,
) -> ScenarioConfig:
    """Build a deterministic ScenarioConfig for multi-hop testing."""
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
        enable_connectivity_aware_planning=True,
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
        enable_connectivity_aware_planning=True,
        uavs=uavs,
        tasks=tasks,
        challenge_profile=prof,
    )


def test_single_relay_backward_compatibility():
    """Requirement 1: Verify single-relay backward compatibility for D=175m -> H=2, K=1, station at (12.5, 500)."""
    gcs = (-75.0, 500.0)
    poi_pos = (100.0, 500.0)  # Distance = 175.0m
    h_min, k_min, stations = compute_multihop_stations(gcs, poi_pos, effective_range=95.0)

    # Geometry check
    assert h_min == 2
    assert k_min == 1
    assert len(stations) == 1
    assert stations[0] == (12.5, 500.0)

    planner = ConnectivityAwarePlanner(comm_range=100.0)
    task = TaskState(id="poi_1", position_xy=poi_pos, priority=1, service_duration=10.0)
    uav_surveyor = UAVState(id="uav_s", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)
    uav_relay = UAVState(id="uav_r", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)

    snap = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs={"uav_s": uav_surveyor, "uav_r": uav_relay},
        tasks={"poi_1": task},
        gcs_position=gcs,
    )

    res = planner.check_task_connectivity_feasibility(task, uav_surveyor, snap)
    assert res.feasible is True
    assert res.relay_needed is True
    assert res.min_relay_count == 1
    assert res.hop_count == 2
    assert list(res.relay_positions) == [(12.5, 500.0)]

    cmds = planner.plan(snap)
    assert len(cmds) >= 2
    relay_cmds = [c for c in cmds if isinstance(c, AssignRelayRoleCommand)]
    task_cmds = [c for c in cmds if isinstance(c, AssignTaskCommand)]

    assert len(relay_cmds) == 1
    assert relay_cmds[0].target_position == (12.5, 500.0)
    assert len(task_cmds) == 1
    assert task_cmds[0].task_id == "poi_1"
    assert planner.connectivity_feasible_assignments == 1
    assert planner.max_hop_count == 2


def test_two_relay_chain_formation():
    """Requirement 2: Verify 2-relay chain formation for D=255m -> H=3, K=2, stations at (10, 500) and (95, 500)."""
    gcs = (-75.0, 500.0)
    poi_pos = (180.0, 500.0)  # Distance = 255.0m: H = ceil(255/95) = 3, K = 2
    h_min, k_min, stations = compute_multihop_stations(gcs, poi_pos, effective_range=95.0)

    assert h_min == 3
    assert k_min == 2
    assert len(stations) == 2
    assert stations[0] == (10.0, 500.0)
    assert stations[1] == (95.0, 500.0)

    planner = ConnectivityAwarePlanner(comm_range=100.0)
    task = TaskState(id="poi_2", position_xy=poi_pos, priority=1, service_duration=10.0)
    uav_s = UAVState(id="uav_s", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)
    uav_r1 = UAVState(id="uav_r1", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)
    uav_r2 = UAVState(id="uav_r2", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)

    snap = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs={"uav_s": uav_s, "uav_r1": uav_r1, "uav_r2": uav_r2},
        tasks={"poi_2": task},
        gcs_position=gcs,
    )

    res = planner.check_task_connectivity_feasibility(task, uav_s, snap)
    assert res.feasible is True
    assert res.relay_needed is True
    assert res.min_relay_count == 2
    assert res.hop_count == 3
    assert list(res.relay_positions) == [(10.0, 500.0), (95.0, 500.0)]
    assert len(res.relay_uav_ids) == 2

    cmds = planner.plan(snap)
    relay_cmds = [c for c in cmds if isinstance(c, AssignRelayRoleCommand)]
    task_cmds = [c for c in cmds if isinstance(c, AssignTaskCommand)]

    assert len(relay_cmds) == 2
    positions = [c.target_position for c in relay_cmds]
    assert (10.0, 500.0) in positions
    assert (95.0, 500.0) in positions
    assert len(task_cmds) == 1
    assert planner.connectivity_feasible_assignments == 1
    assert planner.max_hop_count == 3
    assert len(planner.relay_manager.chains) == 1


def test_three_relay_chain_formation():
    """Requirement 3: Verify 3-relay chain formation for D=350m -> H=4, K=3, stations computed correctly."""
    gcs = (-75.0, 500.0)
    poi_pos = (275.0, 500.0)  # Distance = 350.0m: H = ceil(350/95) = 4, K = 3
    h_min, k_min, stations = compute_multihop_stations(gcs, poi_pos, effective_range=95.0)

    assert h_min == 4
    assert k_min == 3
    assert len(stations) == 3
    assert stations[0] == (12.5, 500.0)
    assert stations[1] == (100.0, 500.0)
    assert stations[2] == (187.5, 500.0)

    # Link segments all <= 95m
    all_points = [gcs] + list(stations) + [poi_pos]
    for p1, p2 in zip(all_points[:-1], all_points[1:]):
        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        assert dist <= 95.0 + 1e-6

    planner = ConnectivityAwarePlanner(comm_range=100.0)
    task = TaskState(id="poi_3", position_xy=poi_pos, priority=1, service_duration=10.0)
    uav_s = UAVState(id="uav_s", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)
    uav_r1 = UAVState(id="uav_r1", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)
    uav_r2 = UAVState(id="uav_r2", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)
    uav_r3 = UAVState(id="uav_r3", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)

    snap = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs={"uav_s": uav_s, "uav_r1": uav_r1, "uav_r2": uav_r2, "uav_r3": uav_r3},
        tasks={"poi_3": task},
        gcs_position=gcs,
    )

    res = planner.check_task_connectivity_feasibility(task, uav_s, snap)
    assert res.feasible is True
    assert res.min_relay_count == 3
    assert res.hop_count == 4
    assert list(res.relay_positions) == list(stations)
    assert len(res.relay_uav_ids) == 3

    cmds = planner.plan(snap)
    relay_cmds = [c for c in cmds if isinstance(c, AssignRelayRoleCommand)]
    assert len(relay_cmds) == 3
    assert planner.max_hop_count == 4


def test_airborne_relay_reuse():
    """Requirement 4: Verify airborne candidate already near intermediate station is prioritized over ground UAV."""
    gcs = (-75.0, 500.0)
    station = (10.0, 500.0)

    relay_mgr = DynamicRelayManager()
    # Ground candidate at staging pad (-75, 500)
    uav_ground = UAVState(id="uav_ground", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0, sortie_state=SortieState.READY)
    # Airborne candidate at (12, 500) within 25m reuse tolerance of station (10, 500)
    uav_airborne = UAVState(id="uav_airborne", position_xy=(12.0, 500.0), role=Role.IDLE, active=True, battery_energy=5000.0, sortie_state=SortieState.ACTIVE)

    snap = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs={"uav_ground": uav_ground, "uav_airborne": uav_airborne},
        tasks={},
        gcs_position=gcs,
    )

    # Candidate selection should prioritize the airborne UAV already at the station
    chosen = relay_mgr.select_relay_candidate(
        snapshot=snap,
        target_uav_id="uav_s",
        relay_position=station,
    )
    assert chosen == "uav_airborne"


def test_insufficient_relays_clean_deferral():
    """Requirement 5: Verify clean task deferral and zero UAV dispatches when insufficient relays exist."""
    gcs = (-75.0, 500.0)
    poi_pos = (275.0, 500.0)  # Requires 3 relays + 1 surveyor = 4 UAVs
    planner = ConnectivityAwarePlanner(comm_range=100.0)
    task = TaskState(id="poi_heavy", position_xy=poi_pos, priority=1, service_duration=10.0)

    # Only 3 UAVs provided (1 surveyor candidate + 2 relay candidates, but 3 relays are needed)
    uav_s = UAVState(id="uav_s", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)
    uav_r1 = UAVState(id="uav_r1", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)
    uav_r2 = UAVState(id="uav_r2", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)

    snap = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs={"uav_s": uav_s, "uav_r1": uav_r1, "uav_r2": uav_r2},
        tasks={"poi_heavy": task},
        gcs_position=gcs,
    )

    cmds = planner.plan(snap)
    # Zero UAVs dispatched
    assert len(cmds) == 0
    assert planner.connectivity_deferred_tasks >= 1
    assert planner.tasks_deferred_insufficient_relays == 1
    assert planner.connectivity_feasible_assignments == 0


def test_multihop_intermediate_link_handoff():
    """Requirement 6: In an active 3-hop chain, intermediate relay reaches RTH margin -> replaced without breaking chain."""
    gcs = (-75.0, 500.0)
    station_1 = (10.0, 500.0)
    station_2 = (95.0, 500.0)
    surv_pos = (180.0, 500.0)

    relay_mgr = DynamicRelayManager()
    # Register active chain: R1 at station_1, R2 at station_2, surveyor at surv_pos
    chain = relay_mgr.register_chain(
        chain_id="chain_test",
        surveyor_id="uav_s",
        relay_ids=["uav_r1", "uav_r2"],
        station_positions=[station_1, station_2],
    )

    # uav_r1 has very low battery (will trigger RTH handoff)
    uav_r1 = UAVState(id="uav_r1", position_xy=station_1, role=Role.RELAY, active=True, battery_energy=50.0)
    # uav_r2 is healthy
    uav_r2 = UAVState(id="uav_r2", position_xy=station_2, role=Role.RELAY, active=True, battery_energy=5000.0)
    # uav_s is active surveyor
    uav_s = UAVState(id="uav_s", position_xy=surv_pos, role=Role.SURVEYOR, active=True, battery_energy=5000.0, assigned_task_id="poi_test")
    # Fresh candidate at GCS
    uav_rep = UAVState(id="uav_rep", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)

    snap = StateSnapshot(
        simulation_tick=10,
        simulation_time=10.0,
        state_version=1,
        uavs={"uav_s": uav_s, "uav_r1": uav_r1, "uav_r2": uav_r2, "uav_rep": uav_rep},
        tasks={"poi_test": TaskState(id="poi_test", position_xy=surv_pos, priority=1, status=TaskStatus.IN_PROGRESS, assigned_uav_id="uav_s")},
        gcs_position=gcs,
    )

    cmds = relay_mgr.step(snapshot=snap, dt=1.0)

    # 1. Dispatch tick: uav_rep dispatched toward station_1, uav_r1 remains active at station_1 (make-before-break)
    assert relay_mgr.relay_chain_handoffs == 0
    assert chain.status == ChainStatus.HANDOFF
    assert chain.relay_ids == ["uav_r1", "uav_r2"]
    assert "uav_r1" in relay_mgr.relay_to_chain
    assert 0 in chain.pending_handoffs

    assign_cmds = [c for c in cmds if isinstance(c, AssignRelayRoleCommand)]
    assert len(assign_cmds) == 1
    assert assign_cmds[0].uav_id == "uav_rep"
    assert assign_cmds[0].target_position == station_1

    target_cmds = [c for c in cmds if isinstance(c, SetTargetPositionCommand)]
    assert any(c.uav_id == "uav_rep" and c.target_position == station_1 for c in target_cmds)

    # No release or RTH commands for incumbent uav_r1 yet
    assert not any(isinstance(c, ReleaseRelayRoleCommand) and c.uav_id == "uav_r1" for c in cmds)
    assert not any(isinstance(c, StartRTHCommand) and c.uav_id == "uav_r1" for c in cmds)

    # 2. Simulate arrival at station_1 on tick 11
    from dataclasses import replace
    uav_rep_at_station = replace(uav_rep, position_xy=station_1, role=Role.RELAY)
    snap2 = replace(
        snap,
        simulation_tick=11,
        simulation_time=11.0,
        uavs={"uav_s": uav_s, "uav_r1": uav_r1, "uav_r2": uav_r2, "uav_rep": uav_rep_at_station},
    )

    cmds2 = relay_mgr.step(snapshot=snap2, dt=1.0)

    # Localized handoff completed atomically upon arrival and verification
    assert relay_mgr.relay_chain_handoffs == 1
    assert chain.status == ChainStatus.ACTIVE
    # R1 was replaced by uav_rep; R2 remains in place
    assert chain.relay_ids == ["uav_rep", "uav_r2"]
    assert relay_mgr.relay_to_chain.get("uav_rep") == "chain_test"
    assert "uav_r1" not in relay_mgr.relay_to_chain

    # Commands emitted for replacement swap, release, and RTH
    handoff_cmds = [c for c in cmds2 if isinstance(c, HandoffRelayCommand)]
    assert len(handoff_cmds) == 1
    assert handoff_cmds[0].replacement_uav_id == "uav_rep"
    assert handoff_cmds[0].target_position == station_1

    release_cmds = [c for c in cmds2 if isinstance(c, ReleaseRelayRoleCommand)]
    assert len(release_cmds) == 1
    assert release_cmds[0].uav_id == "uav_r1"

    rth_cmds = [c for c in cmds2 if isinstance(c, StartRTHCommand)]
    assert len(rth_cmds) == 1
    assert rth_cmds[0].uav_id == "uav_r1"



def test_multihop_link_failure_recovery():
    """Requirement 7: In an active 3-hop chain, intermediate relay fails -> replaced if candidate available."""
    gcs = (-75.0, 500.0)
    station_1 = (10.0, 500.0)
    station_2 = (95.0, 500.0)
    surv_pos = (180.0, 500.0)

    relay_mgr = DynamicRelayManager()
    chain = relay_mgr.register_chain(
        chain_id="chain_test",
        surveyor_id="uav_s",
        relay_ids=["uav_r1", "uav_r2"],
        station_positions=[station_1, station_2],
    )

    # uav_r2 experiences hardware failure
    uav_r1 = UAVState(id="uav_r1", position_xy=station_1, role=Role.RELAY, active=True, battery_energy=5000.0)
    uav_r2_failed = UAVState(id="uav_r2", position_xy=station_2, role=Role.RELAY, active=False, failure_state=FailureState.FAILED, battery_energy=5000.0)
    uav_s = UAVState(id="uav_s", position_xy=surv_pos, role=Role.SURVEYOR, active=True, battery_energy=5000.0, assigned_task_id="poi_test")
    uav_rep = UAVState(id="uav_rep", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)

    snap = StateSnapshot(
        simulation_tick=15,
        simulation_time=15.0,
        state_version=1,
        uavs={"uav_s": uav_s, "uav_r1": uav_r1, "uav_r2": uav_r2_failed, "uav_rep": uav_rep},
        tasks={"poi_test": TaskState(id="poi_test", position_xy=surv_pos, priority=1, status=TaskStatus.IN_PROGRESS, assigned_uav_id="uav_s")},
        gcs_position=gcs,
    )

    cmds = relay_mgr.step(snapshot=snap, dt=1.0)

    assert relay_mgr.relay_chain_failures == 1
    assert relay_mgr.relay_chain_recoveries == 1
    assert chain.status == ChainStatus.ACTIVE
    assert chain.relay_ids == ["uav_r1", "uav_rep"]
    assert relay_mgr.relay_to_chain.get("uav_rep") == "chain_test"


def test_multihop_unrecoverable_break_replan():
    """Requirement 8: Intermediate relay fails with no replacement candidate -> task deferred, surveyor recalled, chain cleanly torn down."""
    gcs = (-75.0, 500.0)
    station_1 = (10.0, 500.0)
    station_2 = (95.0, 500.0)
    surv_pos = (180.0, 500.0)

    planner = ConnectivityAwarePlanner(comm_range=100.0)
    relay_mgr = planner.relay_manager
    chain = relay_mgr.register_chain(
        chain_id="chain_test",
        surveyor_id="uav_s",
        relay_ids=["uav_r1", "uav_r2"],
        station_positions=[station_1, station_2],
    )

    # uav_r2 fails, and NO other UAV exists to replace it
    uav_r1 = UAVState(id="uav_r1", position_xy=station_1, role=Role.RELAY, active=True, battery_energy=5000.0)
    uav_r2_failed = UAVState(id="uav_r2", position_xy=station_2, role=Role.RELAY, active=False, failure_state=FailureState.FAILED, battery_energy=5000.0)
    uav_s = UAVState(id="uav_s", position_xy=surv_pos, role=Role.SURVEYOR, active=True, battery_energy=5000.0, assigned_task_id="poi_test")

    snap = StateSnapshot(
        simulation_tick=20,
        simulation_time=20.0,
        state_version=1,
        uavs={"uav_s": uav_s, "uav_r1": uav_r1, "uav_r2": uav_r2_failed},
        tasks={"poi_test": TaskState(id="poi_test", position_xy=surv_pos, priority=1, status=TaskStatus.IN_PROGRESS, assigned_uav_id="uav_s")},
        gcs_position=gcs,
    )

    # Step relay manager -> mark chain degraded
    relay_mgr.step(snapshot=snap, dt=1.0)
    assert chain.status == ChainStatus.DEGRADED

    # Step monitor_active_tasks in planner -> trigger replan & teardown
    replanning_cmds = planner.monitor_active_tasks(snapshot=snap, dt=1.0)

    assert planner.communication_induced_replans >= 1
    # Surveyor task released
    rel_task_cmds = [c for c in replanning_cmds if isinstance(c, ReleaseTaskCommand)]
    assert len(rel_task_cmds) == 1
    assert rel_task_cmds[0].task_id == "poi_test"

    # Surviving healthy relay uav_r1 released
    rel_relay_cmds = [c for c in replanning_cmds if isinstance(c, ReleaseRelayRoleCommand)]
    assert any(c.uav_id == "uav_r1" for c in rel_relay_cmds)

    # Chain unregistered cleanly
    assert "chain_test" not in relay_mgr.chains


def test_multihop_reporting_deadline_compliance():
    """Requirement 9: Detection report is delivered over a 3-hop relay chain within 10s deadline."""
    gcs = (-75.0, 500.0)
    poi_pos = (180.0, 500.0)
    station_1 = (10.0, 500.0)
    station_2 = (95.0, 500.0)

    # 3-hop chain: GCS (-75, 500) <-> R1 (10, 500) <-> R2 (95, 500) <-> S (180, 500)
    # All link segments are exactly 85m <= 100m comm range
    sc = make_multihop_scenario(
        name="multihop_reporting_test",
        duration=60.0,
        uavs=(
            {"id": "uav_s", "position": [180.0, 500.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "SURVEYOR"},
            {"id": "uav_r1", "position": [10.0, 500.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "RELAY"},
            {"id": "uav_r2", "position": [95.0, 500.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "RELAY"},
        ),
        tasks=(
            {"id": "poi_1", "position": [180.0, 500.0], "priority": 1, "service_duration": 40.0},
        ),
    )

    runner = MissionRunner(scenario=sc, seed=42)
    # Register multi-hop chain
    runner.relay_manager.register_chain(
        chain_id="chain_active",
        surveyor_id="uav_s",
        relay_ids=["uav_r1", "uav_r2"],
        station_positions=[station_1, station_2],
    )
    runner.state_store.apply([
        AssignTaskCommand(source_tick=0, uav_id="uav_s", task_id="poi_1")
    ])

    res = runner.run()
    m = res.metrics_report

    assert m.total_detections >= 1
    assert m.reports_delivered >= 1
    assert m.reporting_deadline_success >= 1
    assert m.reporting_deadline_failure == 0
    assert m.reporting_compliance_ratio == 1.0
    # Must report 3 hops over the 2-relay chain
    assert m.max_hop_count == 3
    assert m.mean_hop_count == 3.0
