"""Deterministic unit and integration tests for Phase 3: Dynamic Relay Role Management."""
from __future__ import annotations

import math
from pathlib import Path
from types import MappingProxyType
import pytest

from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.autonomy.relay_manager import DynamicRelayManager, RelayManagementConfig
from ares_swarm.autonomy.task_allocator import A0TaskAllocator
from ares_swarm.core.commands import (
    AssignRelayRoleCommand,
    AssignTaskCommand,
    FailUAVCommand,
    HandoffRelayCommand,
    ReleaseRelayRoleCommand,
    StartRTHCommand,
)
from ares_swarm.core.enums import EventType, FailureState, Role, RTHState, SortieState, TaskStatus
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState
from ares_swarm.core.state_store import StateStore
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import (
    ChallengeAirspaceConfig,
    ChallengeProfileConfig,
    ScenarioConfig,
)


def make_relay_test_scenario(
    name: str = "relay_test",
    duration: float = 400.0,
    uavs: tuple[dict, ...] = (),
    tasks: tuple[dict, ...] = (),
    gcs_pos: tuple[float, float] = (-75.0, 500.0),
    enable_auto_rth: bool = True,
    enable_relay_manager: bool = True,
) -> ScenarioConfig:
    """Helper to build challenge-compliant scenario for relay management testing."""
    return ScenarioConfig(
        name=name,
        duration=duration,
        max_ticks=int(duration),
        dt=1.0,
        speed_limit=5.0,
        gcs_position=gcs_pos,
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        min_separation_m=20.0,
        battery_idle_rate=1.0,
        battery_movement_rate=0.5,
        enable_auto_rth=enable_auto_rth,
        enable_relay_manager=enable_relay_manager,
        challenge_profile=ChallengeProfileConfig(
            enabled=True,
            max_sortie_duration_s=1200.0,
            rth_safety_margin_s=15.0,
            recharge_duration_s=60.0,
            enforce_sortie_limit=True,
            enforce_single_sortie=False,
            enable_relay_manager=enable_relay_manager,
            airspace=ChallengeAirspaceConfig(
                enabled=True,
                staging_pad_center=gcs_pos,
                staging_pad_radius_m=15.0,
                corridor_bounds_x=(-75.0, 0.0),
                corridor_bounds_y=(400.0, 600.0),
                arena_bounds_x=(0.0, 1000.0),
                arena_bounds_y=(0.0, 1000.0),
            ),
        ),
        uavs=uavs,
        tasks=tasks,
    )


def test_surveyor_becoming_relay():
    """Verify operational role transition: SURVEYOR -> RELAY."""
    uav = UAVState(
        id="uav_1",
        position_xy=(50.0, 500.0),
        role=Role.SURVEYOR,
        assigned_task_id="task_1",
        sortie_state=SortieState.ACTIVE,
    )
    task = TaskState(id="task_1", position_xy=(100.0, 500.0), priority=1, status=TaskStatus.IN_PROGRESS, assigned_uav_id="uav_1")
    store = StateStore(StateSnapshot(simulation_tick=0, simulation_time=0.0, state_version=0, uavs={"uav_1": uav}, tasks={"task_1": task}))

    # Transition SURVEYOR -> RELAY
    res = store.apply([
        AssignRelayRoleCommand(source_tick=0, uav_id="uav_1", target_position=(25.0, 500.0), relay_for_uav_id="uav_2")
    ])
    assert len(res.rejected_commands) == 0

    snap = store.snapshot()
    u1 = snap.uavs["uav_1"]
    assert u1.role == Role.RELAY
    assert u1.relay_target_id == "uav_2"
    assert u1.target_position == (25.0, 500.0)
    assert u1.assigned_task_id is None

    # In-progress task must have been deferred back to allocator
    assert snap.tasks["task_1"].status == TaskStatus.DEFERRED
    assert snap.tasks["task_1"].assigned_uav_id is None

    # Events emitted: TASK_HANDOFF, TASK_DEFERRED, RELAY_ASSIGNED
    event_types = [e.event_type for e in res.emitted_events]
    assert EventType.TASK_HANDOFF in event_types
    assert EventType.TASK_DEFERRED in event_types
    assert EventType.RELAY_ASSIGNED in event_types


def test_relay_becoming_surveyor():
    """Verify operational role transition: RELAY -> SURVEYOR."""
    uav = UAVState(
        id="uav_1",
        position_xy=(25.0, 500.0),
        role=Role.RELAY,
        relay_target_id="uav_2",
        target_position=(25.0, 500.0),
        sortie_state=SortieState.ACTIVE,
    )
    task = TaskState(id="task_2", position_xy=(80.0, 500.0), priority=2)
    store = StateStore(StateSnapshot(simulation_tick=0, simulation_time=0.0, state_version=0, uavs={"uav_1": uav}, tasks={"task_2": task}))

    # 1. Release relay role to SURVEYOR
    res1 = store.apply([ReleaseRelayRoleCommand(source_tick=0, uav_id="uav_1", next_role=Role.SURVEYOR)])
    assert len(res1.rejected_commands) == 0
    snap1 = store.snapshot()
    assert snap1.uavs["uav_1"].role == Role.SURVEYOR
    assert snap1.uavs["uav_1"].relay_target_id is None
    assert any(e.event_type == EventType.RELAY_RELEASED for e in res1.emitted_events)

    # 2. Assigning task to RELAY directly transitions it to SURVEYOR
    uav_relay = UAVState(id="uav_relay", position_xy=(25.0, 500.0), role=Role.RELAY, active=True)
    store2 = StateStore(StateSnapshot(simulation_tick=0, simulation_time=0.0, state_version=0, uavs={"uav_relay": uav_relay}, tasks={"task_2": task}))
    res2 = store2.apply([AssignTaskCommand(source_tick=0, uav_id="uav_relay", task_id="task_2")])
    assert len(res2.rejected_commands) == 0
    snap2 = store2.snapshot()
    assert snap2.uavs["uav_relay"].role == Role.SURVEYOR
    assert any(e.event_type == EventType.RELAY_RELEASED for e in res2.emitted_events)
    assert any(e.event_type == EventType.TASK_ASSIGNED for e in res2.emitted_events)


def test_relay_loss_on_failure():
    """Verify that failure of an active relay triggers RELAY_LOST event."""
    uav = UAVState(id="uav_relay", position_xy=(25.0, 500.0), role=Role.RELAY, active=True)
    store = StateStore(StateSnapshot(simulation_tick=0, simulation_time=0.0, state_version=0, uavs={"uav_relay": uav}, tasks={}))

    res = store.apply([FailUAVCommand(source_tick=0, uav_id="uav_relay", failure_state=FailureState.FAILED, reason="MOTOR_ERROR")])
    assert len(res.rejected_commands) == 0
    ev_types = [e.event_type for e in res.emitted_events]
    assert EventType.UAV_FAILED in ev_types
    assert EventType.RELAY_LOST in ev_types

    lost_ev = next(e for e in res.emitted_events if e.event_type == EventType.RELAY_LOST)
    assert lost_ev.entity_id == "uav_relay"
    assert lost_ev.payload["reason"] == "MOTOR_ERROR"


def test_relay_replacement_and_recovery():
    """Verify that when a relay fails, DynamicRelayManager selects a replacement and records recovery."""
    rm = DynamicRelayManager()
    uav_surv = UAVState(id="uav_surv", position_xy=(100.0, 500.0), role=Role.SURVEYOR, active=True)
    uav_relay = UAVState(id="uav_relay", position_xy=(15.0, 500.0), role=Role.RELAY, active=False, failure_state=FailureState.FAILED)
    uav_backup = UAVState(id="uav_backup", position_xy=(-75.0, 500.0), role=Role.IDLE, active=True, battery_energy=5000.0)

    snap = StateSnapshot(
        simulation_tick=1,
        simulation_time=1.0,
        state_version=1,
        uavs={"uav_surv": uav_surv, "uav_relay": uav_relay, "uav_backup": uav_backup},
        tasks={},
        gcs_position=(-75.0, 500.0),
    )
    rm.surveyor_to_relay["uav_surv"] = "uav_relay"
    rm.relay_to_surveyor["uav_relay"] = "uav_surv"
    rm.relay_positions["uav_relay"] = (15.0, 500.0)

    cmds = rm.step(snap)
    assert rm.relay_losses >= 1
    assert rm.relay_recovery_successes >= 1

    assign_cmd = next((c for c in cmds if isinstance(c, AssignRelayRoleCommand)), None)
    assert assign_cmd is not None
    assert assign_cmd.uav_id == "uav_backup"
    assert assign_cmd.target_position == (15.0, 500.0)
    assert assign_cmd.relay_for_uav_id == "uav_surv"


def test_relay_rth_causing_handoff():
    """Verify that when a relay enters RTH, a deterministic relay handoff occurs."""
    uav_old = UAVState(id="uav_old", position_xy=(15.0, 500.0), role=Role.RELAY, relay_target_id="uav_surv", active=True)
    uav_new = UAVState(id="uav_new", position_xy=(-75.0, 500.0), role=Role.IDLE, active=True, battery_energy=5000.0)
    store = StateStore(StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs={"uav_old": uav_old, "uav_new": uav_new},
        tasks={},
        gcs_position=(-75.0, 500.0),
    ))

    # Apply HandoffRelayCommand
    res = store.apply([
        HandoffRelayCommand(
            source_tick=0,
            uav_id="uav_old",
            replacement_uav_id="uav_new",
            target_position=(15.0, 500.0),
            relay_for_uav_id="uav_surv",
        )
    ])
    assert len(res.rejected_commands) == 0

    snap = store.snapshot()
    assert snap.uavs["uav_new"].role == Role.RELAY
    assert snap.uavs["uav_new"].relay_target_id == "uav_surv"
    assert snap.uavs["uav_new"].target_position == (15.0, 500.0)

    ev_types = [e.event_type for e in res.emitted_events]
    assert EventType.RELAY_HANDOFF in ev_types
    assert EventType.RELAY_ASSIGNED in ev_types

    handoff_ev = next(e for e in res.emitted_events if e.event_type == EventType.RELAY_HANDOFF)
    assert handoff_ev.entity_id == "uav_old"
    assert handoff_ev.payload["new_relay_id"] == "uav_new"


def test_no_eligible_relay_available():
    """Verify that when no UAV satisfies endurance and RTH gates, no relay is selected (Requirement 5)."""
    rm = DynamicRelayManager()
    uav_surv = UAVState(id="uav_surv", position_xy=(120.0, 500.0), role=Role.SURVEYOR, active=True)
    # Candidate has critically low battery (< required transit + operational + return energy)
    uav_poor_battery = UAVState(id="uav_poor", position_xy=(-75.0, 500.0), role=Role.IDLE, active=True, battery_energy=10.0)
    # Candidate already in RTH
    uav_in_rth = UAVState(id="uav_rth", position_xy=(10.0, 500.0), role=Role.IDLE, active=True, rth_state=RTHState.ACTIVE)
    # Candidate in RECHARGING
    uav_recharging = UAVState(id="uav_rech", position_xy=(-75.0, 500.0), role=Role.IDLE, active=False, sortie_state=SortieState.RECHARGING)

    snap = StateSnapshot(
        simulation_tick=1,
        simulation_time=1.0,
        state_version=1,
        uavs={"uav_surv": uav_surv, "uav_poor": uav_poor_battery, "uav_rth": uav_in_rth, "uav_rech": uav_recharging},
        tasks={},
        gcs_position=(-75.0, 500.0),
    )

    cand = rm.select_relay_candidate(
        snapshot=snap,
        target_uav_id="uav_surv",
        relay_position=(15.0, 500.0),
    )
    assert cand is None, "Must return None when all candidates fail feasibility gates"


def test_deterministic_repeatability():
    """Verify that repeated runs of relay coordination produce identical deterministic results."""
    sc = make_relay_test_scenario(
        name="relay_determinism_test",
        duration=150.0,
        uavs=(
            {"id": "uav_a", "position": [100.0, 500.0], "battery_capacity": 5000.0, "battery_energy": 5000.0, "role": "SURVEYOR"},
            {"id": "uav_b", "position": [15.0, 500.0], "battery_capacity": 5000.0, "battery_energy": 5000.0, "role": "RELAY"},
            {"id": "uav_c", "position": [-75.0, 500.0], "battery_capacity": 5000.0, "battery_energy": 5000.0, "role": "IDLE"},
        ),
        tasks=(
            {"id": "task_1", "position": [100.0, 500.0], "priority": 1, "service_duration": 80.0},
        ),
    )

    runner1 = MissionRunner(scenario=sc, seed=42)
    res1 = runner1.run()

    runner2 = MissionRunner(scenario=sc, seed=42)
    res2 = runner2.run()

    # Identical tick counts and simulation times
    assert res1.total_ticks == res2.total_ticks
    assert res1.simulation_time == res2.simulation_time

    # Bitwise identical event stream
    events1 = [(e.simulation_tick, e.event_type.value, e.entity_id) for e in res1.all_events]
    events2 = [(e.simulation_tick, e.event_type.value, e.entity_id) for e in res2.all_events]
    assert events1 == events2


def test_integrated_relay_handoff_scenario():
    """Integrated scenario demonstrating:

    UAV A surveys -> UAV B is relay -> B must RTH -> UAV C takes relay responsibility ->
    route preserved/reconfigured -> mission continues to completion.
    """
    gcs = (-75.0, 500.0)
    sc = make_relay_test_scenario(
        name="integrated_relay_handoff",
        duration=300.0,
        uavs=(
            # UAV A: Surveyor stationed at (100, 500) (distance to GCS = 175m > 100m range)
            {"id": "uav_a", "position": [100.0, 500.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "SURVEYOR"},
            # UAV B: Relay stationed at (15, 500) (distance to GCS = 90m <= 100m; distance to A = 85m <= 100m)
            # Low battery capacity so it will trigger RTH mid-mission
            {"id": "uav_b", "position": [15.0, 500.0], "battery_capacity": 180.0, "battery_energy": 180.0, "role": "RELAY"},
            # UAV C: Standby replacement at GCS with full battery
            {"id": "uav_c", "position": [-75.0, 500.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "IDLE"},
        ),
        tasks=(
            {"id": "poi_1", "position": [100.0, 500.0], "priority": 1, "service_duration": 120.0},
        ),
    )

    runner = MissionRunner(scenario=sc, seed=42)
    # Set initial relay mapping
    runner.relay_manager.surveyor_to_relay["uav_a"] = "uav_b"
    runner.relay_manager.relay_to_surveyor["uav_b"] = "uav_a"
    runner.relay_manager.relay_positions["uav_b"] = (15.0, 500.0)

    result = runner.run()

    # 1. Verify relay handoff was recorded
    handoff_events = [e for e in result.all_events if e.event_type == EventType.RELAY_HANDOFF]
    assert len(handoff_events) >= 1, "Must emit RELAY_HANDOFF event"

    m = result.metrics_report
    assert m.relay_handoffs >= 1
    assert m.relay_releases >= 1

    # 2. Verify before/after connectivity around handoff (Requirement 13)
    assert m.connected_time_before_handoff > 0.0, f"Expected connected_time_before_handoff > 0, got {m.connected_time_before_handoff}"
    assert m.connected_time_after_handoff > 0.0, f"Expected connected_time_after_handoff > 0, got {m.connected_time_after_handoff}"
    assert m.network_reconfiguration_time_s is not None
    assert m.network_reconfiguration_time_s >= 0.0

    # 3. Verify UAV B safely returned to GCS without battery exhaustion
    assert m.battery_exhaustion_count == 0
    u_b = result.final_snapshot.uavs["uav_b"]
    assert u_b.battery_energy > 0.0
    assert u_b.rth_state == RTHState.COMPLETE or u_b.sortie_state in (SortieState.LANDED, SortieState.RECHARGING, SortieState.READY)

    # 4. Verify mission continued and task was completed
    assert result.final_snapshot.tasks["poi_1"].status == TaskStatus.COMPLETE
    assert m.tasks_completed == 1
    assert m.mission_completion_rate == 1.0
