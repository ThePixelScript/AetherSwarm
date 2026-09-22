"""Deterministic unit and integration tests for Phase 2: 20-minute Sortie Rotation and Handoff."""
from __future__ import annotations

import math
from pathlib import Path
import pytest

from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.autonomy.task_allocator import A0TaskAllocator
from ares_swarm.core.commands import (
    AssignTaskCommand,
    CompleteRechargeCommand,
    CompleteRTHCommand,
    StartRechargeCommand,
    StartRTHCommand,
)
from ares_swarm.core.enums import EventType, RTHState, Role, SortieState, TaskStatus
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState
from ares_swarm.core.state_store import StateStore
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import (
    ChallengeAirspaceConfig,
    ChallengeProfileConfig,
    ScenarioConfig,
)


def make_test_scenario(
    name: str = "sortie_test",
    duration: float = 600.0,
    uavs: tuple[dict, ...] = (),
    tasks: tuple[dict, ...] = (),
    recharge_duration_s: float = 60.0,
    enforce_sortie_limit: bool = True,
    enforce_single_sortie: bool = False,
    gcs_pos: tuple[float, float] = (-75.0, 500.0),
) -> ScenarioConfig:
    """Helper to build deterministic challenge-enabled scenario for rotation tests."""
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
        enable_auto_rth=True,
        challenge_profile=ChallengeProfileConfig(
            enabled=True,
            max_sortie_duration_s=1200.0,
            rth_safety_margin_s=15.0,
            recharge_duration_s=recharge_duration_s,
            enforce_sortie_limit=enforce_sortie_limit,
            enforce_single_sortie=enforce_single_sortie,
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


def test_explicit_sortie_states_and_transitions():
    """Verify explicit UAV sortie state machine: READY -> ACTIVE -> RTH -> LANDING -> LANDED -> RECHARGING -> READY."""
    uav = UAVState(id="uav_1", position_xy=(-75.0, 500.0), sortie_state=SortieState.READY)
    task = TaskState(id="task_1", position_xy=(100.0, 500.0), priority=1)
    store = StateStore(StateSnapshot(simulation_tick=0, simulation_time=0.0, state_version=0, uavs={"uav_1": uav}, tasks={"task_1": task}))

    # 1. READY -> ACTIVE on task assignment
    res1 = store.apply([AssignTaskCommand(source_tick=0, uav_id="uav_1", task_id="task_1")])
    assert len(res1.rejected_commands) == 0
    snap1 = store.snapshot()
    assert snap1.uavs["uav_1"].sortie_state == SortieState.ACTIVE

    # 2. ACTIVE -> RTH
    res2 = store.apply([StartRTHCommand(source_tick=0, uav_id="uav_1")])
    assert len(res2.rejected_commands) == 0
    snap2 = store.snapshot()
    assert snap2.uavs["uav_1"].sortie_state == SortieState.RTH

    # 3. RTH -> LANDING / LANDED on landing
    res3 = store.apply([CompleteRTHCommand(source_tick=0, uav_id="uav_1")])
    assert len(res3.rejected_commands) == 0
    snap3 = store.snapshot()
    assert snap3.uavs["uav_1"].sortie_state == SortieState.LANDED
    assert not snap3.uavs["uav_1"].active

    # 4. LANDED -> RECHARGING
    res4 = store.apply([StartRechargeCommand(source_tick=0, uav_id="uav_1", recharge_duration_s=60.0)])
    assert len(res4.rejected_commands) == 0
    snap4 = store.snapshot()
    assert snap4.uavs["uav_1"].sortie_state == SortieState.RECHARGING
    assert snap4.uavs["uav_1"].recharge_duration_s == 60.0

    # 5. RECHARGING -> READY
    res5 = store.apply([CompleteRechargeCommand(source_tick=0, uav_id="uav_1")])
    assert len(res5.rejected_commands) == 0
    snap5 = store.snapshot()
    assert snap5.uavs["uav_1"].sortie_state == SortieState.READY
    assert snap5.uavs["uav_1"].active
    assert snap5.uavs["uav_1"].battery_energy == snap5.uavs["uav_1"].battery_capacity


def test_1200s_sortie_enforcement():
    """Verify that continuous airborne sortie duration <= 1200 s is enforced."""
    gcs = (-75.0, 500.0)
    sc = make_test_scenario(
        name="sortie_limit_test",
        duration=1300.0,
        enforce_single_sortie=True,
        uavs=(
            {"id": "uav_1", "position": [100.0, 500.0], "battery_capacity": 50000.0, "battery_energy": 50000.0, "role": "IDLE"},
        ),
        tasks=(
            {"id": "task_1", "position": [200.0, 500.0], "priority": 1, "service_duration": 2000.0},
        ),
    )
    runner = MissionRunner(scenario=sc, autonomy_adapter=A0AutonomyAdapter(A0TaskAllocator()))
    result = runner.run()

    # Safety assessor must verify max continuous sortie <= 1200s
    safety_rep = runner.safety_assessor.report
    assert safety_rep.max_observed_sortie_duration_s <= 1200.0 + 1e-4
    assert len([v for v in safety_rep.violations if v.violation_type == "FLIGHT_DURATION"]) == 0
    snap = result.final_snapshot
    assert snap.uavs["uav_1"].sortie_state in (SortieState.LANDED, SortieState.RECHARGING, SortieState.READY)


def test_rth_before_battery_exhaustion():
    """Verify RTH triggers early enough to prevent uncommanded battery exhaustion."""
    gcs = (-75.0, 500.0)
    # Give just enough battery for a short mission and safe return
    sc = make_test_scenario(
        name="battery_rth_test",
        duration=400.0,
        uavs=(
            {"id": "uav_1", "position": [100.0, 500.0], "battery_capacity": 600.0, "battery_energy": 600.0, "role": "IDLE"},
        ),
        tasks=(
            {"id": "task_1", "position": [300.0, 500.0], "priority": 1, "service_duration": 500.0},
        ),
    )
    runner = MissionRunner(scenario=sc, autonomy_adapter=A0AutonomyAdapter(A0TaskAllocator()))
    result = runner.run()

    safety_rep = runner.safety_assessor.report
    assert safety_rep.battery_exhaustions_count == 0
    assert result.metrics_report.battery_exhaustion_count == 0
    # UAV must have safely returned to GCS
    u1 = result.final_snapshot.uavs["uav_1"]
    assert u1.battery_energy > 0.0


def test_task_handoff_and_reassignment():
    """Verify that when a UAV must RTH while servicing a task, the task is handed off and reassigned."""
    gcs = (-75.0, 500.0)
    sc = make_test_scenario(
        name="handoff_reassign_test",
        duration=500.0,
        recharge_duration_s=30.0,
        uavs=(
            # UAV 1 has low battery (will trigger RTH while servicing task)
            {"id": "uav_1", "position": [50.0, 500.0], "battery_capacity": 250.0, "battery_energy": 250.0, "role": "IDLE"},
            # UAV 2 is at GCS with plenty of battery (can take over)
            {"id": "uav_2", "position": [-75.0, 500.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "IDLE"},
        ),
        tasks=(
            {"id": "task_1", "position": [150.0, 500.0], "priority": 1, "service_duration": 80.0},
        ),
    )
    runner = MissionRunner(scenario=sc, autonomy_adapter=A0AutonomyAdapter(A0TaskAllocator()))
    result = runner.run()

    # Verify task handoff was recorded
    handoff_events = [e for e in result.all_events if e.event_type == EventType.TASK_HANDOFF]
    assert len(handoff_events) >= 1
    assert result.metrics_report.task_handoffs >= 1

    # Verify task was completed after reassignment
    final_task = result.final_snapshot.tasks["task_1"]
    assert final_task.status == TaskStatus.COMPLETE


def test_deterministic_recharge_and_redeployment():
    """Verify complete lifecycle: UAV leaves -> performs work -> RTH -> lands -> recharges -> redeploys."""
    gcs = (-75.0, 500.0)
    recharge_time = 40.0
    sc = make_test_scenario(
        name="full_rotation_test",
        duration=600.0,
        recharge_duration_s=recharge_time,
        enforce_single_sortie=False,
        uavs=(
            {"id": "uav_1", "position": [-75.0, 500.0], "battery_capacity": 380.0, "battery_energy": 380.0, "role": "IDLE"},
        ),
        tasks=(
            {"id": "task_1", "position": [100.0, 500.0], "priority": 2, "service_duration": 20.0},
            {"id": "task_2", "position": [150.0, 500.0], "priority": 1, "service_duration": 20.0},
        ),
    )
    runner = MissionRunner(scenario=sc, autonomy_adapter=A0AutonomyAdapter(A0TaskAllocator()))
    result = runner.run()

    # Check metrics
    m = result.metrics_report
    assert m.recharge_count >= 1
    assert m.sorties_completed >= 1
    assert m.RTH_count >= 1
    assert m.battery_exhaustion_count == 0

    # Flight records should show multiple sorties
    rec = runner.safety_assessor.report.uav_flight_records["uav_1"]
    assert rec.sortie_count >= 2


def test_multiple_simultaneous_returns_no_deadlock():
    """Verify that multiple returning UAVs do not remain indefinitely deadlocked at 20 m."""
    gcs = (-75.0, 500.0)
    sc = make_test_scenario(
        name="simul_return_deadlock_test",
        duration=400.0,
        uavs=(
            {"id": "uav_1", "position": [50.0, 480.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "IDLE"},
            {"id": "uav_2", "position": [60.0, 520.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "IDLE"},
            {"id": "uav_3", "position": [80.0, 500.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "IDLE"},
        ),
        tasks=(),
    )
    runner = MissionRunner(scenario=sc, autonomy_adapter=A0AutonomyAdapter(A0TaskAllocator()))
    # Trigger RTH on all 3 UAVs simultaneously
    runner.state_store.apply([
        StartRTHCommand(source_tick=0, uav_id="uav_1"),
        StartRTHCommand(source_tick=0, uav_id="uav_2"),
        StartRTHCommand(source_tick=0, uav_id="uav_3"),
    ])
    result = runner.run()

    # All 3 UAVs must successfully reach GCS and land without deadlock
    snap = result.final_snapshot
    for uid in ("uav_1", "uav_2", "uav_3"):
        u = snap.uavs[uid]
        assert u.rth_state == RTHState.COMPLETE or u.sortie_state in (SortieState.LANDED, SortieState.RECHARGING, SortieState.READY)
        d_gcs = math.hypot(u.position_xy[0] - gcs[0], u.position_xy[1] - gcs[1])
        assert d_gcs <= 15.0  # Safely inside staging pad radius

    # Verify zero landing deadlocks
    assert result.metrics_report.landing_deadlocks == 0


def test_deterministic_repeatability():
    """Verify that running the same rotation scenario twice produces identical results."""
    gcs = (-75.0, 500.0)
    sc1 = make_test_scenario(
        name="repeatability_test",
        duration=300.0,
        recharge_duration_s=30.0,
        uavs=(
            {"id": "uav_1", "position": [-75.0, 500.0], "battery_capacity": 800.0, "battery_energy": 800.0, "role": "IDLE"},
        ),
        tasks=(
            {"id": "task_1", "position": [100.0, 500.0], "priority": 1, "service_duration": 30.0},
        ),
    )
    sc2 = make_test_scenario(
        name="repeatability_test",
        duration=300.0,
        recharge_duration_s=30.0,
        uavs=(
            {"id": "uav_1", "position": [-75.0, 500.0], "battery_capacity": 800.0, "battery_energy": 800.0, "role": "IDLE"},
        ),
        tasks=(
            {"id": "task_1", "position": [100.0, 500.0], "priority": 1, "service_duration": 30.0},
        ),
    )
    runner1 = MissionRunner(scenario=sc1, autonomy_adapter=A0AutonomyAdapter(A0TaskAllocator()))
    res1 = runner1.run()

    runner2 = MissionRunner(scenario=sc2, autonomy_adapter=A0AutonomyAdapter(A0TaskAllocator()))
    res2 = runner2.run()

    # Compare event sequences
    events1 = [(e.simulation_tick, e.event_type.value, e.entity_id) for e in res1.all_events]
    events2 = [(e.simulation_tick, e.event_type.value, e.entity_id) for e in res2.all_events]
    assert events1 == events2

    # Compare metrics
    m1 = res1.metrics_report.to_dict()
    m2 = res2.metrics_report.to_dict()
    assert m1["sortie_rotation"] == m2["sortie_rotation"]
