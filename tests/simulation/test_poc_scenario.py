"""Tests for organizer sample scenario poc_round1.yaml and MissionRunner PoC readiness."""
from pathlib import Path
import pytest

from ares_swarm.core.enums import TaskStatus
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import load_scenario


def test_poc_round1_scenario_loading():
    scenario_path = Path("scenarios/poc_round1.yaml")
    assert scenario_path.is_file(), "scenarios/poc_round1.yaml must exist"

    config = load_scenario(scenario_path)
    assert config.name == "poc_round1_mission"
    assert config.speed_limit == 5.0
    assert config.duration == 2700.0
    assert config.max_ticks == 2700
    assert config.arena_bounds_x == (0.0, 1000.0)
    assert config.arena_bounds_y == (0.0, 1000.0)
    assert config.max_height == 100.0
    assert config.min_separation_m == 20.0
    assert config.gcs_position[0] < 0.0  # GCS strictly outside arena
    assert config.communication.max_range == 100.0
    assert len(config.uavs) == 5
    assert len(config.tasks) == 10

    # Verify all initial UAVs are separated by >= 20m
    uav_positions = [tuple(u["position"]) for u in config.uavs]
    for i in range(len(uav_positions)):
        for j in range(i + 1, len(uav_positions)):
            dx = uav_positions[i][0] - uav_positions[j][0]
            dy = uav_positions[i][1] - uav_positions[j][1]
            dist = (dx**2 + dy**2) ** 0.5
            assert dist >= 20.0, f"UAVs {i} and {j} too close: {dist}m < 20m"


def test_poc_round1_execution_and_metrics(tmp_path):
    scenario_path = Path("scenarios/poc_round1.yaml")
    runner = MissionRunner(scenario_path, seed=42)

    # Run for 150 ticks to observe dynamic task appearance and execution
    result = runner.run(max_ticks=150)
    assert result.total_ticks == 150
    assert result.simulation_time == 150.0

    summary = result.to_dict()
    assert "metrics" in summary
    assert "evaluation" in summary
    assert "mission" in summary["evaluation"]
    assert "communication" in summary["evaluation"]
    assert "resilience" in summary["evaluation"]
    assert "safety" in summary["evaluation"]

    # Tasks that completed must have assigned_task_id == None and target_position cleared
    for u in result.final_snapshot.uavs.values():
        if u.assigned_task_id is None and u.rth_state.value == "NONE":
            # If idle and no task, target_position must be None
            assert u.target_position is None

    # Safety metrics should report no collisions (initial separation >= 20m)
    assert summary["evaluation"]["safety"]["collision_count"] == 0
    assert summary["evaluation"]["safety"]["min_inter_uav_separation_m"] >= 20.0

    # Save and verify JSON serialization
    json_path = tmp_path / "summary.json"
    result.save_json(json_path)
    assert json_path.is_file()


def test_poc_round1_deterministic_replay():
    scenario_path = Path("scenarios/poc_round1.yaml")

    runner1 = MissionRunner(scenario_path, seed=42)
    res1 = runner1.run(max_ticks=80)
    summary1 = res1.to_dict()

    runner2 = MissionRunner(scenario_path, seed=42)
    res2 = runner2.run(max_ticks=80)
    summary2 = res2.to_dict()

    assert summary1 == summary2

    # Step by step snapshots and event sequences must match exactly
    assert len(res1.all_events) == len(res2.all_events)
    for e1, e2 in zip(res1.all_events, res2.all_events):
        assert e1.event_type == e2.event_type
        assert e1.entity_id == e2.entity_id
        assert e1.simulation_tick == e2.simulation_tick
        assert e1.payload == e2.payload

    for s1, s2 in zip(res1.step_history, res2.step_history):
        assert s1.tick == s2.tick
        assert s1.simulation_time == s2.simulation_time
        assert s1.snapshot.state_version == s2.snapshot.state_version
        for uid in s1.snapshot.uavs:
            assert s1.snapshot.uavs[uid].position_xy == s2.snapshot.uavs[uid].position_xy
            assert s1.snapshot.uavs[uid].battery_energy == s2.snapshot.uavs[uid].battery_energy


def test_poc_round1_full_duration_execution():
    """Run the PoC scenario for its full configured 2700-second mission duration."""
    scenario_path = Path("scenarios/poc_round1.yaml")
    runner = MissionRunner(scenario_path, seed=42)

    result = runner.run()  # Defaults to scenario.max_ticks = 2700
    assert result.total_ticks == 2700
    assert result.simulation_time == 2700.0

    summary = result.to_dict()
    metrics = summary["metrics"]
    eval_metrics = summary["evaluation"]

    # Task metrics semantics
    assert metrics["tasks_total"] == 10
    assert metrics["tasks_spawned"] == 10
    assert metrics["tasks_completed"] == 10
    assert metrics["tasks_assigned"] == 0  # No tasks left merely assigned
    assert metrics["tasks_expired"] == 0
    assert metrics["mission_completion_rate"] == 1.0

    # Safety constraints & hard constraints
    assert eval_metrics["safety"]["collision_count"] == 0
    assert eval_metrics["safety"]["min_inter_uav_separation_m"] >= 20.0
    assert eval_metrics["safety"]["battery_exhaustion_count"] == 0
    assert eval_metrics["safety"]["geofence_violation_count"] == 0

    # All UAVs returned to GCS by mission end
    for uid, u in result.final_snapshot.uavs.items():
        dx = u.position_xy[0] - result.final_snapshot.gcs_position[0]
        dy = u.position_xy[1] - result.final_snapshot.gcs_position[1]
        dist_gcs = (dx**2 + dy**2) ** 0.5
        assert dist_gcs <= 1.0, f"UAV {uid} did not return to GCS: {dist_gcs:.2f}m away"
        assert u.battery_energy > 0.0, f"UAV {uid} exhausted battery"

    # Resilience metrics explicitly unsupported/not applicable in M0
    assert eval_metrics["resilience"]["status"] == "NOT_APPLICABLE_M0"
    assert eval_metrics["resilience"]["relay_reallocations"] is None
    assert eval_metrics["resilience"]["recovery_time_s"] is None
