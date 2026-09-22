"""Deterministic integration tests for Phase 5B.1: Relay Handoff Continuity.

Verifies make-before-break handoff semantics in multi-hop relay chains:
1. test_handoff_replacement_in_transit_preserves_incumbent:
   Incumbent remains assigned and operational at station during transit;
   chain status is HANDOFF; no premature RTH; telemetry route intact through incumbent.
2. test_handoff_arrival_and_atomic_swap:
   Replacement arrives within 1.0m, adjacent link connectivity is verified,
   atomic role swap occurs, incumbent is released and commanded RTH, metrics incremented.
3. test_handoff_replacement_failure_preserves_incumbent:
   Replacement UAV fails during transit; incumbent is preserved; handoff retries
   or transitions to DEGRADED without premature incumbent teardown.
4. test_handoff_telemetry_continuity_across_all_ticks:
   Surveyor generates telemetry on every tick through handoff; zero packet loss;
   continuous delivery through incumbent then replacement.
5. test_single_relay_backward_compatibility:
   Legacy single-relay handoff paths without RelayChain continue to function
   without regression.
"""
from __future__ import annotations

import math
from dataclasses import replace
from typing import Dict, List, Optional, Tuple

import pytest

from ares_swarm.autonomy.relay_manager import (
    ChainStatus,
    DynamicRelayManager,
    RelayChain,
    RelayManagementConfig,
)
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.core.commands import (
    AssignRelayRoleCommand,
    Command,
    HandoffRelayCommand,
    ReleaseRelayRoleCommand,
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
from ares_swarm.telemetry.manager import DetectionManager


def make_test_scenario(
    name: str = "single_relay_compat",
    duration: float = 200.0,
    uavs: tuple = (),
    tasks: tuple = (),
    comm_range: float = 100.0,
) -> ScenarioConfig:
    """Build a deterministic ScenarioConfig for relay handoff testing."""
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
    comm = CommunicationConfig(
        max_range=comm_range,
        base_latency=5.0,
        packet_loss=0.0,
    )
    profile = ChallengeProfileConfig(
        enabled=True,
        max_sortie_duration_s=1200.0,
        rth_safety_margin_s=15.0,
        recharge_duration_s=60.0,
        enforce_sortie_limit=True,
        enforce_single_sortie=False,
        airspace=airspace,
        detection_pipeline=detect_pipe,
        enable_relay_manager=True,
    )
    return ScenarioConfig(
        name=name,
        duration=duration,
        max_ticks=int(duration),
        dt=1.0,
        gcs_position=gcs,
        speed_limit=5.0,
        battery_idle_rate=1.0,
        battery_movement_rate=0.5,
        enable_auto_rth=True,
        challenge_profile=profile,
        communication=comm,
        enable_relay_manager=True,
        uavs=uavs,
        tasks=tasks,
    )


def _make_snapshot(
    tick: int,
    sim_time: float,
    uavs: Dict[str, UAVState],
    gcs: Tuple[float, float] = (-75.0, 500.0),
    task_pos: Tuple[float, float] = (180.0, 500.0),
    tasks: Optional[Dict[str, TaskState]] = None,
) -> StateSnapshot:
    """Helper to build a consistent StateSnapshot."""
    if tasks is None:
        tasks = {
            "poi_1": TaskState(
                id="poi_1",
                position_xy=task_pos,
                priority=1,
                status=TaskStatus.IN_PROGRESS,
                assigned_uav_id="uav_s",
            )
        }
    return StateSnapshot(
        simulation_tick=tick,
        simulation_time=sim_time,
        state_version=tick,
        uavs=uavs,
        tasks=tasks,
        gcs_position=gcs,
    )




def test_handoff_replacement_in_transit_preserves_incumbent():
    """Test 1: Incumbent remains in role at station while replacement is in transit."""
    gcs = (-75.0, 500.0)
    st_1 = (10.0, 500.0)
    st_2 = (95.0, 500.0)
    surv_pos = (180.0, 500.0)

    relay_mgr = DynamicRelayManager()
    chain = relay_mgr.register_chain(
        chain_id="chain_test",
        surveyor_id="uav_s",
        relay_ids=["uav_r1", "uav_r2"],
        station_positions=[st_1, st_2],
    )

    # uav_r1 has low battery (will trigger Condition B)
    uav_r1 = UAVState(id="uav_r1", position_xy=st_1, role=Role.RELAY, active=True, battery_energy=50.0)
    uav_r2 = UAVState(id="uav_r2", position_xy=st_2, role=Role.RELAY, active=True, battery_energy=5000.0)
    uav_s = UAVState(id="uav_s", position_xy=surv_pos, role=Role.SURVEYOR, active=True, battery_energy=5000.0, assigned_task_id="poi_1")
    uav_rep = UAVState(id="uav_rep", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)

    snap = _make_snapshot(
        tick=10,
        sim_time=10.0,
        uavs={"uav_s": uav_s, "uav_r1": uav_r1, "uav_r2": uav_r2, "uav_rep": uav_rep},
        gcs=gcs,
        task_pos=surv_pos,
    )

    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=100.0))
    net_analysis = analyzer.analyze(snap)

    cmds = relay_mgr.step(snapshot=snap, network_analysis=net_analysis, dt=1.0)

    # Verify dispatch occurs without breaking chain
    assert relay_mgr.relay_chain_handoffs == 0, "Handoff count must not increment upon dispatch"
    assert chain.status == ChainStatus.HANDOFF, "Chain must transition to HANDOFF state"
    assert chain.relay_ids == ["uav_r1", "uav_r2"], "Incumbent must remain in chain during transit"
    assert "uav_r1" in relay_mgr.relay_to_chain, "Incumbent mapping must be preserved"
    assert 0 in chain.pending_handoffs, "Station 0 must be tracked in pending_handoffs"

    handoff = chain.pending_handoffs[0]
    assert handoff["incumbent_id"] == "uav_r1"
    assert handoff["replacement_id"] == "uav_rep"
    assert handoff["station_position"] == st_1
    assert handoff["status"] == "IN_PROGRESS"

    # Replacement dispatch commands emitted
    assign_cmds = [c for c in cmds if isinstance(c, AssignRelayRoleCommand) and c.uav_id == "uav_rep"]
    assert len(assign_cmds) == 1
    assert assign_cmds[0].target_position == st_1

    target_cmds = [c for c in cmds if isinstance(c, SetTargetPositionCommand) and c.uav_id == "uav_rep"]
    assert len(target_cmds) == 1
    assert target_cmds[0].target_position == st_1

    # Incumbent MUST NOT receive release or RTH commands
    assert not any(isinstance(c, ReleaseRelayRoleCommand) and c.uav_id == "uav_r1" for c in cmds)
    assert not any(isinstance(c, StartRTHCommand) and c.uav_id == "uav_r1" for c in cmds)

    # Communication route to GCS is intact through incumbent
    assert net_analysis.routes_to_gcs.get("uav_s") == ("uav_s", "uav_r2", "uav_r1", "gcs")


def test_handoff_arrival_and_atomic_swap():
    """Test 2: When replacement arrives at station and verifies links, atomic swap occurs."""
    gcs = (-75.0, 500.0)
    st_1 = (10.0, 500.0)
    st_2 = (95.0, 500.0)
    surv_pos = (180.0, 500.0)

    relay_mgr = DynamicRelayManager()
    chain = relay_mgr.register_chain(
        chain_id="chain_test",
        surveyor_id="uav_s",
        relay_ids=["uav_r1", "uav_r2"],
        station_positions=[st_1, st_2],
    )

    uav_r1 = UAVState(id="uav_r1", position_xy=st_1, role=Role.RELAY, active=True, battery_energy=50.0)
    uav_r2 = UAVState(id="uav_r2", position_xy=st_2, role=Role.RELAY, active=True, battery_energy=5000.0)
    uav_s = UAVState(id="uav_s", position_xy=surv_pos, role=Role.SURVEYOR, active=True, battery_energy=5000.0, assigned_task_id="poi_1")
    uav_rep = UAVState(id="uav_rep", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)

    snap1 = _make_snapshot(10, 10.0, {"uav_s": uav_s, "uav_r1": uav_r1, "uav_r2": uav_r2, "uav_rep": uav_rep}, gcs, surv_pos)
    relay_mgr.step(snapshot=snap1, dt=1.0)
    assert 0 in chain.pending_handoffs

    # Intermediate tick: replacement is mid-flight at (-30.0, 500.0) (40m away from st_1)
    uav_rep_transit = replace(uav_rep, position_xy=(-30.0, 500.0), role=Role.RELAY)
    snap2 = _make_snapshot(15, 15.0, {"uav_s": uav_s, "uav_r1": uav_r1, "uav_r2": uav_r2, "uav_rep": uav_rep_transit}, gcs, surv_pos)
    cmds2 = relay_mgr.step(snapshot=snap2, dt=1.0)

    # In transit: no swap, no release
    assert relay_mgr.relay_chain_handoffs == 0
    assert chain.status == ChainStatus.HANDOFF
    assert chain.relay_ids == ["uav_r1", "uav_r2"]
    assert not any(isinstance(c, HandoffRelayCommand) for c in cmds2)

    # Final tick: replacement arrives at st_1 (10.0, 500.0)
    uav_rep_arrived = replace(uav_rep, position_xy=st_1, role=Role.RELAY)
    snap3 = _make_snapshot(20, 20.0, {"uav_s": uav_s, "uav_r1": uav_r1, "uav_r2": uav_r2, "uav_rep": uav_rep_arrived}, gcs, surv_pos)
    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=100.0))
    net_analysis3 = analyzer.analyze(snap3)

    cmds3 = relay_mgr.step(snapshot=snap3, network_analysis=net_analysis3, dt=1.0)

    # Atomic swap completed!
    assert relay_mgr.relay_chain_handoffs == 1
    assert relay_mgr.relay_handoffs == 1
    assert chain.status == ChainStatus.ACTIVE
    assert 0 not in chain.pending_handoffs
    assert chain.relay_ids == ["uav_rep", "uav_r2"]
    assert relay_mgr.relay_to_chain["uav_rep"] == "chain_test"
    assert "uav_r1" not in relay_mgr.relay_to_chain

    # Commands emitted for swap, release, and RTH
    handoff_cmds = [c for c in cmds3 if isinstance(c, HandoffRelayCommand)]
    assert len(handoff_cmds) == 1
    assert handoff_cmds[0].replacement_uav_id == "uav_rep"
    assert handoff_cmds[0].uav_id == "uav_r1"

    release_cmds = [c for c in cmds3 if isinstance(c, ReleaseRelayRoleCommand)]
    assert len(release_cmds) == 1
    assert release_cmds[0].uav_id == "uav_r1"

    rth_cmds = [c for c in cmds3 if isinstance(c, StartRTHCommand)]
    assert len(rth_cmds) == 1
    assert rth_cmds[0].uav_id == "uav_r1"


def test_handoff_replacement_failure_preserves_incumbent():
    """Test 3: If replacement UAV fails during transit, incumbent is preserved and retried."""
    gcs = (-75.0, 500.0)
    st_1 = (10.0, 500.0)
    st_2 = (95.0, 500.0)
    surv_pos = (180.0, 500.0)

    relay_mgr = DynamicRelayManager()
    chain = relay_mgr.register_chain(
        chain_id="chain_test",
        surveyor_id="uav_s",
        relay_ids=["uav_r1", "uav_r2"],
        station_positions=[st_1, st_2],
    )

    uav_r1 = UAVState(id="uav_r1", position_xy=st_1, role=Role.RELAY, active=True, battery_energy=50.0)
    uav_r2 = UAVState(id="uav_r2", position_xy=st_2, role=Role.RELAY, active=True, battery_energy=5000.0)
    uav_s = UAVState(id="uav_s", position_xy=surv_pos, role=Role.SURVEYOR, active=True, battery_energy=5000.0, assigned_task_id="poi_1")
    uav_rep1 = UAVState(id="uav_rep1", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)
    uav_rep2 = UAVState(id="uav_rep2", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)

    # Step 1: Dispatch rep1
    snap1 = _make_snapshot(10, 10.0, {
        "uav_s": uav_s, "uav_r1": uav_r1, "uav_r2": uav_r2,
        "uav_rep1": uav_rep1, "uav_rep2": uav_rep2,
    }, gcs, surv_pos)
    relay_mgr.step(snapshot=snap1, dt=1.0)
    assert chain.pending_handoffs[0]["replacement_id"] == "uav_rep1"

    # Step 2: rep1 fails during transit!
    uav_rep1_failed = replace(uav_rep1, active=False, failure_state=FailureState.FAILED)
    snap2 = _make_snapshot(11, 11.0, {
        "uav_s": uav_s, "uav_r1": uav_r1, "uav_r2": uav_r2,
        "uav_rep1": uav_rep1_failed, "uav_rep2": uav_rep2,
    }, gcs, surv_pos)

    cmds2 = relay_mgr.step(snapshot=snap2, dt=1.0)

    # Incumbent R1 was NOT released!
    assert chain.relay_ids == ["uav_r1", "uav_r2"]
    assert "uav_r1" in relay_mgr.relay_to_chain
    assert not any(isinstance(c, ReleaseRelayRoleCommand) and c.uav_id == "uav_r1" for c in cmds2)

    # rep2 was selected and dispatched to replace rep1
    assert 0 in chain.pending_handoffs
    assert chain.pending_handoffs[0]["replacement_id"] == "uav_rep2"
    assert chain.status == ChainStatus.HANDOFF
    assert any(isinstance(c, AssignRelayRoleCommand) and c.uav_id == "uav_rep2" for c in cmds2)


def test_handoff_telemetry_continuity_across_all_ticks():
    """Test 4: Surveyor telemetry is delivered continuously across handoff ticks with zero packet loss."""
    gcs = (-75.0, 500.0)
    st_1 = (10.0, 500.0)
    st_2 = (95.0, 500.0)
    surv_pos = (180.0, 500.0)

    relay_mgr = DynamicRelayManager()
    chain = relay_mgr.register_chain(
        chain_id="chain_test",
        surveyor_id="uav_s",
        relay_ids=["uav_r1", "uav_r2"],
        station_positions=[st_1, st_2],
    )

    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=100.0))
    telem_mgr = DetectionManager(config=DetectionPipelineConfig(enabled=True, reporting_deadline_s=10.0))

    # Initial state
    uav_r1 = UAVState(id="uav_r1", position_xy=st_1, role=Role.RELAY, active=True, battery_energy=50.0)
    uav_r2 = UAVState(id="uav_r2", position_xy=st_2, role=Role.RELAY, active=True, battery_energy=5000.0)
    uav_s = UAVState(id="uav_s", position_xy=surv_pos, role=Role.SURVEYOR, active=True, battery_energy=5000.0, assigned_task_id="poi_1")
    uav_rep = UAVState(id="uav_rep", position_xy=gcs, role=Role.IDLE, active=True, battery_energy=5000.0)

    # 1. Tick 10: Dispatch tick
    tasks1 = {"poi_1": TaskState(id="poi_1", position_xy=surv_pos, priority=1, status=TaskStatus.IN_PROGRESS, assigned_uav_id="uav_s", created_time=10.0)}
    snap1 = _make_snapshot(10, 10.0, {"uav_s": uav_s, "uav_r1": uav_r1, "uav_r2": uav_r2, "uav_rep": uav_rep}, tasks=tasks1, gcs=gcs, task_pos=surv_pos)
    net1 = analyzer.analyze(snap1)
    relay_mgr.step(snapshot=snap1, network_analysis=net1, dt=1.0)

    # Surveyor detects POI and queues telemetry report
    telem_mgr.step_perception(snap1)
    events1 = telem_mgr.step_telemetry(snap1, net1)
    assert len(events1) == 1
    assert events1[0].event_type == EventType.TELEMETRY_DELIVERED
    assert telem_mgr.authoritative_reports["poi_1"].status == TelemetryStatus.DELIVERED
    assert telem_mgr.authoritative_reports["poi_1"].route == ("uav_s", "uav_r2", "uav_r1", "gcs")

    # 2. Tick 15: In-transit tick (replacement halfway between GCS and station)
    uav_rep_mid = replace(uav_rep, position_xy=(-30.0, 500.0), role=Role.RELAY)
    tasks2 = {
        "poi_1": TaskState(id="poi_1", position_xy=surv_pos, priority=1, status=TaskStatus.COMPLETE, assigned_uav_id="uav_s", created_time=10.0),
        "poi_2": TaskState(id="poi_2", position_xy=surv_pos, priority=1, status=TaskStatus.IN_PROGRESS, assigned_uav_id="uav_s", created_time=15.0),
    }
    snap2 = _make_snapshot(15, 15.0, {"uav_s": uav_s, "uav_r1": uav_r1, "uav_r2": uav_r2, "uav_rep": uav_rep_mid}, tasks=tasks2, gcs=gcs, task_pos=surv_pos)
    net2 = analyzer.analyze(snap2)
    relay_mgr.step(snapshot=snap2, network_analysis=net2, dt=1.0)

    telem_mgr.step_perception(snap2)
    events2 = telem_mgr.step_telemetry(snap2, net2)
    assert len(events2) == 1
    assert events2[0].event_type == EventType.TELEMETRY_DELIVERED
    assert telem_mgr.authoritative_reports["poi_2"].status == TelemetryStatus.DELIVERED
    assert telem_mgr.authoritative_reports["poi_2"].route == ("uav_s", "uav_r2", "uav_r1", "gcs")

    # 3. Tick 20: Arrival and atomic swap tick
    uav_rep_arr = replace(uav_rep, position_xy=st_1, role=Role.RELAY)
    tasks3 = {
        "poi_1": TaskState(id="poi_1", position_xy=surv_pos, priority=1, status=TaskStatus.COMPLETE, assigned_uav_id="uav_s", created_time=10.0),
        "poi_2": TaskState(id="poi_2", position_xy=surv_pos, priority=1, status=TaskStatus.COMPLETE, assigned_uav_id="uav_s", created_time=15.0),
        "poi_3": TaskState(id="poi_3", position_xy=surv_pos, priority=1, status=TaskStatus.IN_PROGRESS, assigned_uav_id="uav_s", created_time=21.0),
    }
    snap3 = _make_snapshot(20, 20.0, {"uav_s": uav_s, "uav_r1": uav_r1, "uav_r2": uav_r2, "uav_rep": uav_rep_arr}, tasks=tasks3, gcs=gcs, task_pos=surv_pos)
    net3 = analyzer.analyze(snap3)
    relay_mgr.step(snapshot=snap3, network_analysis=net3, dt=1.0)

    # Post-swap snapshot: uav_r1 starts RTH towards GCS, uav_rep is now active station 1 relay
    uav_r1_rth = replace(uav_r1, position_xy=(-20.0, 500.0), role=Role.IDLE, rth_state=RTHState.ACTIVE)
    snap4 = _make_snapshot(21, 21.0, {"uav_s": uav_s, "uav_r1": uav_r1_rth, "uav_r2": uav_r2, "uav_rep": uav_rep_arr}, tasks=tasks3, gcs=gcs, task_pos=surv_pos)
    net4 = analyzer.analyze(snap4)

    telem_mgr.step_perception(snap4)
    events4 = telem_mgr.step_telemetry(snap4, net4)
    assert len(events4) == 1
    assert events4[0].event_type == EventType.TELEMETRY_DELIVERED
    assert telem_mgr.authoritative_reports["poi_3"].status == TelemetryStatus.DELIVERED
    assert telem_mgr.authoritative_reports["poi_3"].route == ("uav_s", "uav_r2", "uav_rep", "gcs")


    # Metrics summary: 3/3 delivered, 0 lost, 0 deadline exceeded
    metrics = telem_mgr.get_metrics()
    assert metrics["total_detections"] == 3
    assert metrics["reports_delivered"] == 3
    assert metrics["reports_deadline_exceeded"] == 0
    assert metrics["reporting_compliance_ratio"] == 1.0


def test_single_relay_backward_compatibility():
    """Test 5: Preserves legacy single-relay fallback handoff without regression."""
    sc = make_test_scenario(
        name="single_relay_compat",
        duration=120.0,
        uavs=(
            {"id": "uav_a", "position": [100.0, 500.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "SURVEYOR"},
            {"id": "uav_b", "position": [15.0, 500.0], "battery_capacity": 180.0, "battery_energy": 180.0, "role": "RELAY"},
            {"id": "uav_c", "position": [-75.0, 500.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "IDLE"},
        ),
        tasks=(
            {"id": "poi_1", "position": [100.0, 500.0], "priority": 1, "service_duration": 100.0},
        ),
    )


    runner = MissionRunner(scenario=sc, seed=42)
    runner.relay_manager.surveyor_to_relay["uav_a"] = "uav_b"
    runner.relay_manager.relay_to_surveyor["uav_b"] = "uav_a"
    runner.relay_manager.relay_positions["uav_b"] = (15.0, 500.0)

    result = runner.run()
    m = result.metrics_report

    assert m.relay_handoffs >= 1
    assert m.relay_releases >= 1
    assert m.connected_time_after_handoff > 0.0
