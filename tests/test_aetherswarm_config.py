"""Tests for AetherSwarm centralized configuration architecture."""
from __future__ import annotations

from pathlib import Path
import pytest
import yaml

from ares_swarm.config import (
    AetherSwarmConfig,
    ChallengeSimulationConfig,
    FeatureConfig,
    ScenarioGenConfig,
    WebotsPresentationConfig,
)
from ares_swarm.simulation.scenario import load_scenario


REPO_ROOT = Path(__file__).resolve().parent.parent


def test_challenge_simulation_config_defaults():
    """Verify Layer 1: Challenge/Simulation configuration defaults."""
    cfg = ChallengeSimulationConfig()
    assert cfg.arena_bounds_x == (0.0, 1000.0)
    assert cfg.arena_bounds_y == (0.0, 1000.0)
    assert cfg.max_altitude_m == 100.0
    assert cfg.speed_limit_mps == 5.0
    assert cfg.min_separation_m == 20.0
    assert cfg.comm_range_m == 100.0
    assert cfg.comm_base_latency_ms == 5.0
    assert cfg.staging_pad_center == (-75.0, 500.0)
    assert cfg.staging_pad_radius_m == 15.0
    assert cfg.corridor_bounds_x == (-75.0, 0.0)
    assert cfg.corridor_bounds_y == (450.0, 550.0)
    assert cfg.mission_duration_s == 2700.0
    assert cfg.max_sortie_duration_s == 1200.0
    assert cfg.rth_safety_margin_s == 15.0
    assert cfg.reporting_deadline_s == 10.0
    assert cfg.detection_fov_radius_m == 40.0
    assert cfg.processing_delay_s == 0.0


def test_feature_config_defaults():
    """Verify Layer 2: Feature configuration defaults."""
    cfg = FeatureConfig()
    assert cfg.enforce_separation is False
    assert cfg.enforce_geofence is False
    assert cfg.enforce_sortie_limit is True
    assert cfg.enforce_single_sortie is True
    assert cfg.enable_detection_pipeline is False
    assert cfg.enable_auto_rth is True
    assert cfg.return_by_mission_end is True


def test_scenario_gen_config_defaults():
    """Verify Layer 3: Scenario generation configuration defaults."""
    cfg = ScenarioGenConfig()
    assert cfg.seed == 42
    assert cfg.num_pois == 10
    assert cfg.x_range == (5.0, 995.0)
    assert cfg.y_range == (5.0, 995.0)
    assert cfg.min_spacing_m == 0.0
    assert cfg.margin_m == 0.0
    assert cfg.spawn_window_s == (0.0, 300.0)
    assert cfg.service_duration_s == 2.0
    assert cfg.deadline_offset_s == 10.0


def test_webots_presentation_config_defaults():
    """Verify Layer 4: Webots presentation configuration defaults."""
    cfg = WebotsPresentationConfig()
    assert cfg.sub_steps == 4
    assert cfg.sim_mode == "default"
    assert cfg.default_camera == "demo_presentation_cam"
    assert cfg.show_comm_mesh is True
    assert cfg.show_routes is True
    assert cfg.show_drop_lines is True
    assert cfg.show_hud is True
    assert cfg.show_event_banner is True
    assert cfg.auto_quit is False


def test_root_config_serialization_roundtrip():
    """Verify AetherSwarmConfig serialization to/from dictionary."""
    root = AetherSwarmConfig()
    d = root.to_dict()

    assert "challenge" in d
    assert "features" in d
    assert "scenario" in d
    assert "presentation" in d

    recovered = AetherSwarmConfig.from_dict(d)
    assert recovered == root


def test_root_config_overrides():
    """Verify override mechanism creates new immutable instances with requested modifications."""
    root = AetherSwarmConfig()
    overridden = root.with_overrides(
        challenge_overrides={"speed_limit_mps": 6.5, "min_separation_m": 25.0},
        feature_overrides={"enforce_separation": True},
        scenario_overrides={"num_pois": 12, "min_spacing_m": 35.0},
        presentation_overrides={"sub_steps": 8},
    )

    # Original remains untouched
    assert root.challenge.speed_limit_mps == 5.0
    assert root.features.enforce_separation is False
    assert root.scenario.num_pois == 10
    assert root.presentation.sub_steps == 4

    # Overridden has new values
    assert overridden.challenge.speed_limit_mps == 6.5
    assert overridden.challenge.min_separation_m == 25.0
    assert overridden.features.enforce_separation is True
    assert overridden.scenario.num_pois == 12
    assert overridden.scenario.min_spacing_m == 35.0
    assert overridden.presentation.sub_steps == 8


def test_load_scenario_integrates_typed_config():
    """Verify load_scenario seamlessly populates scenario.config with exact parity."""
    e1_path = REPO_ROOT / "scenarios" / "poc_round1.yaml"
    scenario = load_scenario(e1_path)

    assert hasattr(scenario, "config")
    cfg = scenario.config
    assert isinstance(cfg, AetherSwarmConfig)
    assert cfg.challenge.speed_limit_mps == scenario.speed_limit == 5.0
    assert cfg.challenge.min_separation_m == scenario.min_separation_m == 20.0
    assert cfg.challenge.mission_duration_s == scenario.duration == 2700.0
    assert cfg.challenge.comm_range_m == scenario.communication.max_range == 100.0


def test_load_scenario_with_explicit_config_section(tmp_path):
    """YAML scenario files may explicitly provide a config section that overrides defaults."""
    custom_yaml = tmp_path / "custom_config_scenario.yaml"
    data = {
        "name": "custom_override_mission",
        "seed": 100,
        "dt": 1.0,
        "speed_limit": 5.0,
        "duration": 500.0,
        "config": {
            "challenge": {
                "speed_limit_mps": 4.5,
                "min_separation_m": 30.0,
            },
            "features": {
                "enforce_separation": True,
                "enforce_geofence": True,
            },
            "scenario": {
                "num_pois": 8,
                "x_range": [10.0, 500.0],
                "y_range": [10.0, 500.0],
            },
            "presentation": {
                "sub_steps": 6,
                "sim_mode": "fast",
            },
        },
    }
    with open(custom_yaml, "w", encoding="utf-8") as f:
        yaml.dump(data, f)

    scen = load_scenario(custom_yaml)
    assert scen.config.challenge.speed_limit_mps == 4.5
    assert scen.config.challenge.min_separation_m == 30.0
    assert scen.config.features.enforce_separation is True
    assert scen.config.features.enforce_geofence is True
    assert scen.config.scenario.num_pois == 8
    assert scen.config.scenario.x_range == (10.0, 500.0)
    assert scen.config.scenario.y_range == (10.0, 500.0)
    assert scen.config.presentation.sub_steps == 6
    assert scen.config.presentation.sim_mode == "fast"
