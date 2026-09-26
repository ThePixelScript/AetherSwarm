from pathlib import Path
import tempfile

from ares_swarm.core.enums import TaskStatus
from ares_swarm.simulation.runner import MissionRunner, main
from ares_swarm.simulation.scenario import ScenarioConfig, load_scenario


def test_scenario_loading_from_dict_and_yaml():
    data = {
        "name": "test_scenario",
        "seed": 123,
        "dt": 0.5,
        "speed_limit": 4.0,
        "duration": 5.0,
        "gcs_position": [1.0, 2.0],
        "uavs": [
            {"id": "u1", "position": [0.0, 0.0], "battery_capacity": 50.0},
        ],
        "tasks": [
            {"id": "t1", "position": [2.0, 2.0], "priority": 1, "service_duration": 1.0},
        ],
    }

    # Dict loading
    sc_dict = load_scenario(data)
    assert sc_dict.name == "test_scenario"
    assert sc_dict.seed == 123
    assert sc_dict.dt == 0.5
    assert sc_dict.speed_limit == 4.0
    assert sc_dict.gcs_position == (1.0, 2.0)
    assert len(sc_dict.uavs) == 1
    assert len(sc_dict.tasks) == 1

    # YAML file loading
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        import yaml
        yaml.safe_dump(data, f)
        temp_path = f.name

    try:
        sc_yaml = load_scenario(temp_path)
        assert sc_yaml.name == "test_scenario"
        assert sc_yaml.seed == 123
        assert sc_yaml.dt == 0.5
    finally:
        Path(temp_path).unlink()


def test_mission_runner_deterministic_execution():
    config = ScenarioConfig(
        name="runner_test",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=6.0,
        max_ticks=6,
        gcs_position=(0.0, 0.0),
        uavs=(
            {"id": "u1", "position": [0.0, 0.0], "battery_capacity": 100.0},
            {"id": "u2", "position": [0.0, 0.0], "battery_capacity": 100.0},
            {"id": "u3", "position": [0.0, 0.0], "battery_capacity": 100.0},
        ),
        tasks=(
            {"id": "t1", "position": [0.0, 10.0], "priority": 2, "service_duration": 2.0},
            {"id": "t2", "position": [10.0, 0.0], "priority": 1, "service_duration": 2.0},
        ),
        enable_auto_rth=False,
    )

    runner = MissionRunner(config)
    result = runner.run()

    assert result.total_ticks == 6
    assert result.simulation_time == 6.0

    summary = result.to_dict()
    metrics = summary["metrics"]

    # Both tasks should be assigned and completed
    assert metrics["tasks_total"] == 2
    assert metrics["tasks_completed"] == 2
    assert summary["tasks"]["t1"]["status"] == "COMPLETE"
    assert summary["tasks"]["t2"]["status"] == "COMPLETE"

    # UAVs moved and consumed energy
    assert summary["uavs"]["u1"]["position"] == [0.0, 10.0]
    assert summary["uavs"]["u2"]["position"] == [10.0, 0.0]
    assert summary["uavs"]["u3"]["position"] == [0.0, 0.0]
    assert summary["uavs"]["u1"]["battery_energy"] < 100.0
    assert summary["uavs"]["u2"]["battery_energy"] < 100.0
    assert summary["uavs"]["u3"]["battery_energy"] == 94.0

    # Events were generated
    assert len(result.all_events) > 0


def test_mission_runner_exact_repeatability_across_runs():
    config = ScenarioConfig(
        name="repeatability_test",
        seed=999,
        dt=1.0,
        speed_limit=5.0,
        duration=5.0,
        max_ticks=5,
        gcs_position=(0.0, 0.0),
        uavs=(
            {"id": "u1", "position": [0.0, 0.0], "battery_capacity": 100.0},
            {"id": "u2", "position": [0.0, 0.0], "battery_capacity": 100.0},
            {"id": "u3", "position": [0.0, 0.0], "battery_capacity": 100.0},
        ),
        tasks=(
            {"id": "t1", "position": [0.0, 10.0], "priority": 2, "service_duration": 2.0},
            {"id": "t2", "position": [10.0, 0.0], "priority": 1, "service_duration": 2.0},
        ),
    )

    runner1 = MissionRunner(config, seed=999)
    res1 = runner1.run()

    runner2 = MissionRunner(config, seed=999)
    res2 = runner2.run()

    dict1 = res1.to_dict()
    dict2 = res2.to_dict()

    assert dict1 == dict2
    assert len(res1.step_history) == len(res2.step_history)
    for s1, s2 in zip(res1.step_history, res2.step_history):
        assert s1.tick == s2.tick
        assert s1.simulation_time == s2.simulation_time
        assert s1.snapshot.uavs["u1"].position_xy == s2.snapshot.uavs["u1"].position_xy
        assert s1.snapshot.uavs["u1"].battery_energy == s2.snapshot.uavs["u1"].battery_energy
        assert s1.network_analysis.connected_uav_ids == s2.network_analysis.connected_uav_ids


def test_mission_runner_safety_hook():
    config = ScenarioConfig(
        name="safety_hook_test",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=2.0,
        max_ticks=2,
    )

    hook_calls = []

    def mock_safety_hook(snapshot, net_analysis):
        hook_calls.append((snapshot.simulation_tick, net_analysis.gcs_id))

    runner = MissionRunner(config, safety_hook=mock_safety_hook)
    runner.run()

    assert len(hook_calls) == 2
    assert hook_calls[0] == (0, "gcs")
    assert hook_calls[1] == (1, "gcs")


def test_ares_simulate_cli(tmp_path):
    out_file = tmp_path / "cli_summary.json"
    code = main([
        "--scenario", "scenarios/basic.yaml",
        "--seed", "42",
        "--ticks", "4",
        "--output", str(out_file),
    ])

    assert code == 0
    assert out_file.is_file()

    import json
    with open(out_file, "r") as f:
        data = json.load(f)

    assert data["scenario_name"] == "basic_mission"
    assert data["total_ticks"] == 4
    assert data["seed"] == 42
    assert data["metrics"]["tasks_total"] == 2
    assert data["metrics"]["tasks_completed"] == 2
    assert data["metrics"]["tasks_assigned"] == 0
    assert data["metrics"]["mission_completion_rate"] == 1.0
