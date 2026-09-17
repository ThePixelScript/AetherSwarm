import pytest
from ares_swarm.core.config import load_config, load_scenario, SimulationConfig
from ares_swarm.core.exceptions import ConfigError, ModelError

def test_load(root):
    cfg = load_config(root/"configs/default.yaml")
    scenario = load_scenario(root/"scenarios/basic.yaml", cfg)
    assert cfg.simulation.seed == 42
    assert len(scenario.uavs) == 2

@pytest.mark.parametrize("old,new", [
    ("timestep: 0.1", "timestep: -1"),
    ("seed: 42", "seed: true"),
    ("seed: 42", "seed: '42'"),
    ("seed: 42", "seed: 42\n  unknown: 1"),
    ("seed: 42", "seed: 42\n  seed: 43"),
    ("packet_loss: 0.05", "packet_loss: 1.2"),
    ("max_range: 200.0", "max_range: 100.0"),
    ("battery_reserve: 10.0", "battery_reserve: 1000"),
    ("duration: 600.0", "duration: .nan"),
])
def test_invalid_config(root, tmp_path, old, new):
    path = tmp_path/"bad.yaml"
    path.write_text((root/"configs/default.yaml").read_text().replace(old,new))
    with pytest.raises(ConfigError):
        load_config(path)

def test_unsafe_yaml(tmp_path):
    path = tmp_path/"bad.yaml"
    path.write_text("!!python/object/apply:os.system ['echo unsafe']")
    with pytest.raises(ConfigError):
        load_config(path)

def test_scenario_count(root, tmp_path):
    cfg = load_config(root/"configs/default.yaml")
    path = tmp_path/"bad.yaml"
    path.write_text("name: empty\nuavs: []\n")
    with pytest.raises(ConfigError):
        load_scenario(path, cfg)

def test_unknown_event_target(root, tmp_path):
    cfg = load_config(root/"configs/default.yaml")
    path = tmp_path/"bad.yaml"
    text = (root/"scenarios/basic.yaml").read_text().replace(
        "event_type: MISSION_START", "event_type: UAV_FAILURE\n    target: missing")
    path.write_text(text)
    with pytest.raises(ConfigError):
        load_scenario(path, cfg)
