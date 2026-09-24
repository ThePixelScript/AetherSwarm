import math
import pytest

from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.autonomy.connectivity_planner import ConnectivityAwarePlanner
from ares_swarm.autonomy.relay_manager import DynamicRelayManager
from ares_swarm.autonomy.task_allocator import A0TaskAllocator
from ares_swarm.core.commands import (
    AssignTaskCommand,
    BeginLandingCommand,
    CompleteRechargeCommand,
    CompleteRTHCommand,
    StartRechargeCommand,
    StartRTHCommand,
    StepPhysicsCommand,
)
from ares_swarm.core.enums import EventType, Role, RTHState, SortieState, TaskStatus
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState
from ares_swarm.core.state_store import StateStore
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import (
    ChallengeAirspaceConfig,
    ChallengeProfileConfig,
    ScenarioConfig,
)


def make_lifecycle_scenario(
    name: str = "lifecycle_scenario",
    duration: float = 600.0,
    uavs: tuple[dict, ...] = (),
    tasks: tuple[dict, ...] = (),
    recharge_duration_s: float = 60.0,
    enforce_single_sortie: bool = False,
    gcs_pos: tuple[float, float] = (-75.0, 500.0),
    enable_relay_manager: bool = False,
    enable_connectivity_aware_planning: bool = False,
) -> ScenarioConfig:
    """Helper to build deterministic challenge-enabled scenario for lifecycle tests."""
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
        recharge_duration_s=recharge_duration_s,
        enforce_single_sortie=enforce_single_sortie,
        enable_relay_manager=enable_relay_manager,
        enable_connectivity_aware_planning=enable_connectivity_aware_planning,
        challenge_profile=ChallengeProfileConfig(
            enabled=True,
            max_sortie_duration_s=1200.0,
            rth_safety_margin_s=15.0,
            recharge_duration_s=recharge_duration_s,
            enforce_sortie_limit=True,
            enforce_single_sortie=enforce_single_sortie,
            enable_relay_manager=enable_relay_manager,
            enable_connectivity_aware_planning=enable_connectivity_aware_planning,
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


def test_battery_drain_during_flight():
    """Verify that battery energy decreases deterministically during movement and idle hovering."""
    uav = UAVState(
        id="uav_1",
        position_xy=(0.0, 0.0),
        battery_capacity=1000.0,
        battery_energy=1000.0,
        sortie_state=SortieState.ACTIVE,
    )
    store = StateStore(StateSnapshot(simulation_tick=0, simulation_time=0.0, state_version=0, uavs={"uav_1": uav}))

    # Step physics with delta_energy = 15.0 (movement + idle)
    cmd = StepPhysicsCommand(
        source_tick=0,
        uav_id="uav_1",
        new_position_xy=(5.0, 0.0),
        new_velocity_xy=(5.0, 0.0),
        delta_energy=15.0,
    )
    res = store.apply([cmd])
    assert len(res.applied_commands) == 1
    snap = store.snapshot()
    assert snap.uavs["uav_1"].battery_energy == 985.0
    assert snap.uavs["uav_1"].position_xy == (5.0, 0.0)


def test_preemptive_rth_before_depletion():
    """Verify that RTH is triggered well before unsafe battery depletion."""
    gcs = (-75.0, 500.0)
    sc = make_lifecycle_scenario(
        name="test_preemptive_rth",
        duration=400.0,
        uavs=(
            {"id": "uav_1", "position": [100.0, 500.0], "battery_capacity": 500.0, "battery_energy": 500.0, "role": "IDLE"},
        ),
        tasks=(
            {"id": "task_1", "position": [300.0, 500.0], "priority": 1, "service_duration": 400.0},
        ),
    )
    runner = MissionRunner(scenario=sc, autonomy_adapter=A0AutonomyAdapter(A0TaskAllocator()))
    result = runner.run()

    # RTH must have triggered before battery hit 0
    u1 = result.final_snapshot.uavs["uav_1"]
    assert u1.battery_energy > 0.0
    assert result.metrics_report.battery_exhaustions == 0
    assert result.metrics_report.RTH_count >= 1
    assert any(e.event_type == EventType.RTH_TRIGGERED for e in result.all_events)


def test_linear_ground_recharge_in_simulation_time():
    """Verify that recharge occurs over time on the ground and does NOT instantly reset to 100% on landing."""
    gcs = (-75.0, 500.0)
    recharge_dur = 100.0
    sc = make_lifecycle_scenario(
        name="test_linear_recharge",
        duration=250.0,
        recharge_duration_s=recharge_dur,
        enforce_single_sortie=False,
        uavs=(
            {"id": "uav_1", "position": [-50.0, 500.0], "battery_capacity": 1000.0, "battery_energy": 120.0, "role": "IDLE"},
        ),
        tasks=(),  # No tasks: force immediate RTH/landing
    )

    runner = MissionRunner(scenario=sc, autonomy_adapter=A0AutonomyAdapter(A0TaskAllocator()))
    # Trigger RTH immediately
    runner.state_store.apply([StartRTHCommand(source_tick=0, uav_id="uav_1")])

    # Run for 200 ticks and inspect step history
    result = runner.run()

    # Find the tick when UAV_LANDED occurred
    landed_events = [e for e in result.all_events if e.event_type == EventType.UAV_LANDED]
    assert len(landed_events) >= 1
    land_time = landed_events[0].simulation_time

    # Find the tick when UAV_RECHARGING started
    recharge_events = [e for e in result.all_events if e.event_type == EventType.UAV_RECHARGING]
    assert len(recharge_events) >= 1
    rech_start_time = recharge_events[0].simulation_time

    # At landing tick, battery must NOT be fully restored
    landing_step = [s for s in result.step_history if abs(s.simulation_time - land_time) < 1e-4][0]
    uav_at_landing = landing_step.snapshot.uavs["uav_1"]
    assert uav_at_landing.battery_energy < uav_at_landing.battery_capacity * 0.99
    assert uav_at_landing.sortie_state in (SortieState.LANDED, SortieState.RECHARGING)

    # Midway through recharge, battery should be partially charged
    mid_time = rech_start_time + recharge_dur / 2.0
    mid_steps = [s for s in result.step_history if abs(s.simulation_time - mid_time) < 1.0]
    if mid_steps:
        mid_uav = mid_steps[0].snapshot.uavs["uav_1"]
        assert mid_uav.sortie_state == SortieState.RECHARGING
        assert not mid_uav.active
        # Energy should have increased above landing energy but not yet full capacity
        assert mid_uav.battery_energy > uav_at_landing.battery_energy
        assert mid_uav.battery_energy < uav_at_landing.battery_capacity

    # After full recharge duration has passed, battery must be 100% and state READY
    end_events = [e for e in result.all_events if e.event_type == EventType.UAV_RECHARGED]
    assert len(end_events) >= 1
    completion_time = end_events[0].simulation_time
    assert completion_time >= rech_start_time + recharge_dur - 1.0

    # At the exact step when recharge completed, battery was 100% capacity
    recharge_step = [s for s in result.step_history if abs(s.simulation_time - completion_time) < 1e-4][0]
    uav_recharged = recharge_step.snapshot.uavs["uav_1"]
    assert uav_recharged.battery_energy == uav_recharged.battery_capacity
    assert uav_recharged.sortie_state == SortieState.READY

    final_uav = result.final_snapshot.uavs["uav_1"]
    assert final_uav.sortie_state == SortieState.READY
    assert final_uav.active is True


def test_charging_uav_excluded_from_allocation():
    """Verify that a UAV in SortieState.RECHARGING or SortieState.LANDED is strictly excluded from task and relay allocations."""
    allocator = A0TaskAllocator()

    # 1. Recharging UAV
    uav_recharging = UAVState(
        id="uav_rech",
        position_xy=(-75.0, 500.0),
        sortie_state=SortieState.RECHARGING,
        active=False,
    )
    task = TaskState(id="task_1", position_xy=(100.0, 500.0), priority=1)

    feasible, reason = allocator.is_uav_feasible(uav_recharging)
    assert not feasible
    assert "inactive" in reason or "RECHARGING" in reason

    # 2. Landed UAV
    uav_landed = UAVState(
        id="uav_landed",
        position_xy=(-75.0, 500.0),
        sortie_state=SortieState.LANDED,
        active=False,
    )
    feasible, reason = allocator.is_uav_feasible(uav_landed)
    assert not feasible

    # 3. DynamicRelayManager candidate filtering
    relay_mgr = DynamicRelayManager()
    snap = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs={"uav_rech": uav_recharging, "uav_landed": uav_landed},
        tasks={"task_1": task},
        gcs_position=(-75.0, 500.0),
    )
    cand = relay_mgr.select_relay_candidate(
        snapshot=snap,
        target_uav_id="task_1",
        relay_position=(0.0, 500.0),
    )
    assert cand is None


def test_redeployment_after_recharge_completes():
    """Verify that once a UAV finishes recharging, it becomes READY and is deployed to pending tasks."""
    gcs = (-75.0, 500.0)
    recharge_dur = 40.0
    sc = make_lifecycle_scenario(
        name="test_redeployment",
        duration=600.0,
        recharge_duration_s=recharge_dur,
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

    # Check that both tasks were completed across multiple sorties
    t1 = result.final_snapshot.tasks["task_1"]
    t2 = result.final_snapshot.tasks["task_2"]
    assert t1.status == TaskStatus.COMPLETE
    assert t2.status == TaskStatus.COMPLETE

    # Verify multiple sorties completed
    assert result.metrics_report.recharge_count >= 1
    assert result.metrics_report.sorties_completed >= 1
    assert runner.safety_assessor.report.uav_flight_records["uav_1"].sortie_count >= 2


def test_task_handoff_on_surveyor_rth():
    """Verify that when a surveyor enters RTH, the active task is deferred, handed off, and reassigned."""
    gcs = (-75.0, 500.0)
    sc = make_lifecycle_scenario(
        name="test_task_handoff",
        duration=400.0,
        recharge_duration_s=20.0,
        enforce_single_sortie=False,
        uavs=(
            # UAV 1 low battery: will trigger RTH while working
            {"id": "uav_1", "position": [50.0, 500.0], "battery_capacity": 220.0, "battery_energy": 220.0, "role": "IDLE"},
            # UAV 2 high battery: at GCS ready to take over
            {"id": "uav_2", "position": [-75.0, 500.0], "battery_capacity": 5000.0, "battery_energy": 5000.0, "role": "IDLE"},
        ),
        tasks=(
            {"id": "task_1", "position": [150.0, 500.0], "priority": 1, "service_duration": 60.0},
        ),
    )
    runner = MissionRunner(scenario=sc, autonomy_adapter=A0AutonomyAdapter(A0TaskAllocator()))
    result = runner.run()

    # Verify task handoff events
    handoff_events = [e for e in result.all_events if e.event_type == EventType.TASK_HANDOFF]
    deferred_events = [e for e in result.all_events if e.event_type == EventType.TASK_DEFERRED]
    assert len(handoff_events) >= 1
    assert len(deferred_events) >= 1
    assert result.metrics_report.task_handoffs >= 1

    # Final task must be completed by UAV 2
    t1 = result.final_snapshot.tasks["task_1"]
    assert t1.status == TaskStatus.COMPLETE


def test_relay_make_before_break_handoff_on_rth():
    """Verify that an active relay facing RTH performs make-before-break handoff with replacement."""
    gcs = (-75.0, 500.0)
    sc = make_lifecycle_scenario(
        name="test_relay_handoff",
        duration=300.0,
        recharge_duration_s=30.0,
        enforce_single_sortie=False,
        enable_relay_manager=True,
        uavs=(
            # UAV A: Surveyor stationed at (100, 500) (distance to GCS = 175m > 100m range)
            {"id": "uav_a", "position": [100.0, 500.0], "battery_capacity": 10000.0, "battery_energy": 10000.0, "role": "SURVEYOR"},
            # UAV B: Relay stationed at (15, 500) (distance to GCS = 90m <= 100m; distance to A = 85m <= 100m)
            # Low battery capacity so it will trigger RTH mid-mission
            {"id": "uav_b", "position": [15.0, 500.0], "battery_capacity": 500.0, "battery_energy": 180.0, "role": "RELAY"},
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

    # Verify relay handoff was executed
    relay_handoff_events = [e for e in result.all_events if e.event_type == EventType.RELAY_HANDOFF]
    assert len(relay_handoff_events) >= 1 or runner.relay_manager.relay_handoffs >= 1
    # Incumbent relay returned safely without battery exhaustion
    assert result.metrics_report.battery_exhaustion_count == 0
    u_b = result.final_snapshot.uavs["uav_b"]
    assert u_b.battery_energy > 0.0


def test_zero_battery_exhaustion_multi_uav():
    """Verify that in a dense multi-UAV scenario with depleted batteries, no UAV ever runs out of charge."""
    gcs = (-75.0, 500.0)
    sc = make_lifecycle_scenario(
        name="test_zero_exhaustion",
        duration=300.0,
        recharge_duration_s=25.0,
        enforce_single_sortie=False,
        uavs=(
            {"id": "uav_1", "position": [40.0, 480.0], "battery_capacity": 300.0, "battery_energy": 300.0, "role": "IDLE"},
            {"id": "uav_2", "position": [50.0, 520.0], "battery_capacity": 320.0, "battery_energy": 320.0, "role": "IDLE"},
            {"id": "uav_3", "position": [60.0, 500.0], "battery_capacity": 350.0, "battery_energy": 350.0, "role": "IDLE"},
            {"id": "uav_4", "position": [-75.0, 500.0], "battery_capacity": 5000.0, "battery_energy": 5000.0, "role": "IDLE"},
        ),
        tasks=(
            {"id": "task_1", "position": [120.0, 500.0], "priority": 2, "service_duration": 40.0},
            {"id": "task_2", "position": [180.0, 500.0], "priority": 1, "service_duration": 40.0},
        ),
    )
    runner = MissionRunner(scenario=sc, autonomy_adapter=A0AutonomyAdapter(A0TaskAllocator()))
    result = runner.run()

    assert result.metrics_report.battery_exhaustion_count == 0
    assert result.metrics_report.battery_exhaustions == 0
    for uid, u in result.final_snapshot.uavs.items():
        assert u.battery_energy > 0.0


def test_comprehensive_metrics_report_fields():
    """Verify that all new battery and recharge lifecycle metrics are accurately populated in MissionMetricsReport."""
    gcs = (-75.0, 500.0)
    recharge_dur = 20.0
    sc = make_lifecycle_scenario(
        name="test_metrics_fields",
        duration=350.0,
        recharge_duration_s=recharge_dur,
        enforce_single_sortie=False,
        uavs=(
            {"id": "uav_1", "position": [40.0, 500.0], "battery_capacity": 250.0, "battery_energy": 250.0, "role": "IDLE"},
        ),
        tasks=(
            {"id": "task_1", "position": [80.0, 500.0], "priority": 1, "service_duration": 15.0},
            {"id": "task_2", "position": [100.0, 500.0], "priority": 1, "service_duration": 15.0},
        ),
    )
    runner = MissionRunner(scenario=sc, autonomy_adapter=A0AutonomyAdapter(A0TaskAllocator()))
    result = runner.run()
    m = result.metrics_report

    # Verify per-UAV initial and minimum battery
    assert "uav_1" in m.per_uav_initial_battery
    assert m.per_uav_initial_battery["uav_1"] == 250.0
    assert "uav_1" in m.per_uav_min_battery
    assert m.per_uav_min_battery["uav_1"] < 250.0

    # Verify RTH and landing times
    assert "uav_1" in m.per_uav_rth_trigger_times
    assert len(m.per_uav_rth_trigger_times["uav_1"]) >= 1
    assert "uav_1" in m.per_uav_landing_times
    assert len(m.per_uav_landing_times["uav_1"]) >= 1

    # Verify recharge tracking
    assert "uav_1" in m.per_uav_recharge_start_times
    assert len(m.per_uav_recharge_start_times["uav_1"]) >= 1
    assert m.per_uav_recharge_counts["uav_1"] >= 1
    assert m.total_recharge_duration_s >= recharge_dur - 1.0

    # Verify serialization in to_dict()
    d = m.to_dict()
    assert "sortie_rotation" in d
    sr = d["sortie_rotation"]
    assert "per_uav_initial_battery" in sr
    assert "per_uav_min_battery" in sr
    assert "per_uav_rth_trigger_times" in sr
    assert "per_uav_landing_times" in sr
    assert "per_uav_recharge_start_times" in sr
    assert "per_uav_recharge_completion_times" in sr
    assert "per_uav_recharge_counts" in sr
    assert "total_recharge_duration_s" in sr
    assert "relay_handoffs_caused_by_rth" in sr
