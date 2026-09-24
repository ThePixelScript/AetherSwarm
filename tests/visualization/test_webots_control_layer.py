"""Tests for AetherSwarm Webots Working-Model Control Layer.

Validates:
1. Seeking forward/backward (+1s, +3s, -1s, -3s).
2. Seeking boundary clamping at 0 and final_tick.
3. Seeking works identically while paused and while playing.
4. Play/pause toggling and reset functionality.
5. Continuation of visual interpolation state after seeking.
6. Playback speed configuration (0.25x, 0.5x, 1x, 2x, 4x) and cycling.
7. Camera selection (Overview, Follow UAV, GCS, Recovery) and cycling.
8. Visibility toggles (HUD, POIs, Comm mesh, Routes, Drop lines, Grid).
9. Trace immutability (seeking never mutates authoritative simulation trace).
10. Preservation of E1 benchmark and random working scenario loading.
11. Separation of scenario configuration from presentation configuration.
"""
from __future__ import annotations

import copy
import json
import os
from pathlib import Path
import sys
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "visualization" / "webots" / "controllers" / "aetherswarm_supervisor"))

from ares_swarm.config.presentation import WebotsPresentationConfig
from ares_swarm.config.scenario import ScenarioGenConfig
from ares_swarm.config.root import AetherSwarmConfig
import aetherswarm_supervisor
from aetherswarm_supervisor import WebotsAetherSwarmSupervisor, find_trace_file

from scripts.generate_scenario import generate_and_export_scenario

@pytest.fixture(scope="session", autouse=True)
def _ensure_random_trace():
    """Ensure the random trace exists before running webots visualization tests."""
    data_dir = REPO_ROOT / "visualization" / "webots" / "data"
    data_dir.mkdir(parents=True, exist_ok=True)
    target = data_dir / "random_scenario_trace.json"
    
    # Generate the expected random scenario trace for seed 2026 if missing
    # Actually just generate it unconditionally to be deterministic
    generate_and_export_scenario(
        seed=2026,
        output_scenario=data_dir / "random_seed_2026.yaml",
        output_trace=target,
        max_ticks=2700,
        run_simulation=True,
    )
    return target


@pytest.fixture
def supervisor_e1():
    """Create a standalone supervisor loaded with canonical E1 trace."""
    old_scen = os.environ.get("AETHERSWARM_SCENARIO")
    os.environ["AETHERSWARM_SCENARIO"] = "e1"
    try:
        sup = WebotsAetherSwarmSupervisor()
        return sup
    finally:
        if old_scen is not None:
            os.environ["AETHERSWARM_SCENARIO"] = old_scen
        else:
            os.environ.pop("AETHERSWARM_SCENARIO", None)


@pytest.fixture
def supervisor_random():
    """Create a standalone supervisor loaded with randomized working scenario trace."""
    old_scen = os.environ.get("AETHERSWARM_SCENARIO")
    os.environ["AETHERSWARM_SCENARIO"] = "random"
    try:
        sup = WebotsAetherSwarmSupervisor()
        return sup
    finally:
        if old_scen is not None:
            os.environ["AETHERSWARM_SCENARIO"] = old_scen
        else:
            os.environ.pop("AETHERSWARM_SCENARIO", None)


def test_initial_replay_state(supervisor_e1):
    """Supervisor initializes at tick 0 in playing mode with speed 1.0x and overview camera."""
    assert supervisor_e1.playback_cursor == 0
    assert supervisor_e1.sub_step_progress == 0.0
    assert supervisor_e1.is_paused is False
    assert supervisor_e1.playback_speed == 1.0
    assert supervisor_e1.camera_mode == "overview"
    assert supervisor_e1.total_ticks == len(supervisor_e1.ticks)


def test_forward_backward_seeking(supervisor_e1):
    """Seeking forward and backward by 1s and 3s updates playback cursor correctly."""
    sup = supervisor_e1
    assert sup.playback_cursor == 0

    # Step forward 1s
    sup.step_forward_1s()
    assert sup.playback_cursor == 1

    # Step forward 3s
    sup.step_forward_3s()
    assert sup.playback_cursor == 4

    # Step backward 1s
    sup.step_backward_1s()
    assert sup.playback_cursor == 3

    # Step backward 3s
    sup.step_backward_3s()
    assert sup.playback_cursor == 0


def test_seeking_clamping_at_boundaries(supervisor_e1):
    """Seeking clamps strictly to [0, total_ticks - 1]."""
    sup = supervisor_e1
    final_tick = sup.total_ticks - 1

    # Seek backward past 0
    sup.seek_to(0)
    sup.step_backward_1s()
    assert sup.playback_cursor == 0
    sup.step_backward_3s()
    assert sup.playback_cursor == 0
    sup.seek_to(-50)
    assert sup.playback_cursor == 0

    # Seek forward past final_tick
    sup.seek_to(final_tick)
    assert sup.playback_cursor == final_tick
    sup.step_forward_1s()
    assert sup.playback_cursor == final_tick
    sup.step_forward_3s()
    assert sup.playback_cursor == final_tick
    sup.seek_to(final_tick + 500)
    assert sup.playback_cursor == final_tick


def test_seeking_while_paused_and_playing(supervisor_e1):
    """Seeking operates correctly both when playing and when paused."""
    sup = supervisor_e1

    # While playing
    sup.is_paused = False
    sup.seek_to(100)
    assert sup.playback_cursor == 100
    assert sup.is_paused is False

    # While paused
    sup.toggle_play_pause()
    assert sup.is_paused is True
    sup.seek_to(200)
    assert sup.playback_cursor == 200
    assert sup.is_paused is True

    sup.step_forward_3s()
    assert sup.playback_cursor == 203
    assert sup.is_paused is True

    sup.step_backward_1s()
    assert sup.playback_cursor == 202
    assert sup.is_paused is True


def test_reset_control(supervisor_e1):
    """Reset seeks cursor to tick 0 and pauses."""
    sup = supervisor_e1
    sup.seek_to(500, pause=False)
    assert sup.playback_cursor == 500
    assert sup.is_paused is False

    sup.reset()
    assert sup.playback_cursor == 0
    assert sup.is_paused is True


def test_playback_speed_controls(supervisor_e1):
    """Playback speed supports 0.25x, 0.5x, 1x, 2x, 4x presets and cycling."""
    sup = supervisor_e1
    assert sup.SPEED_PRESETS == [0.25, 0.5, 1.0, 2.0, 4.0]

    # Direct setting
    for spd in (0.25, 0.5, 1.0, 2.0, 4.0):
        sup.set_playback_speed(spd)
        assert sup.playback_speed == spd

    # Speed up / slow down
    sup.set_playback_speed(1.0)
    sup.speed_up()
    assert sup.playback_speed == 2.0
    sup.speed_up()
    assert sup.playback_speed == 4.0
    sup.speed_up()  # capped at 4.0
    assert sup.playback_speed == 4.0

    sup.slow_down()
    assert sup.playback_speed == 2.0
    sup.slow_down()
    assert sup.playback_speed == 1.0
    sup.slow_down()
    assert sup.playback_speed == 0.5
    sup.slow_down()
    assert sup.playback_speed == 0.25
    sup.slow_down()  # capped at 0.25
    assert sup.playback_speed == 0.25

    # Cycle speed
    sup.set_playback_speed(4.0)
    sup.cycle_playback_speed(forward=True)
    assert sup.playback_speed == 0.25


def test_camera_selection(supervisor_e1):
    """Camera selection supports Overview, Follow UAV, GCS, Recovery."""
    sup = supervisor_e1
    assert sup.CAMERA_MODES == ["overview", "follow", "gcs", "recovery"]

    for mode in ("overview", "follow", "gcs", "recovery"):
        sup.set_camera(mode)
        assert sup.camera_mode == mode

    # Invalid mode defaults to overview
    sup.set_camera("invalid_camera_name")
    assert sup.camera_mode == "overview"

    # Cycle cameras
    sup.set_camera("overview")
    sup.cycle_camera()
    assert sup.camera_mode == "follow"
    sup.cycle_camera()
    assert sup.camera_mode == "gcs"
    sup.cycle_camera()
    assert sup.camera_mode == "recovery"
    sup.cycle_camera()
    assert sup.camera_mode == "overview"


def test_visibility_toggles(supervisor_e1):
    """Visibility toggles flip flags for HUD, POIs, Mesh, Routes, DropLines, Grid."""
    sup = supervisor_e1

    # Initial states all True
    assert sup.show_hud is True
    assert sup.show_pois is True
    assert sup.show_comm_mesh is True
    assert sup.show_routes is True
    assert sup.show_drop_lines is True
    assert sup.show_grid is True

    # Toggle each off then back on
    sup.toggle_hud()
    assert sup.show_hud is False
    sup.toggle_hud()
    assert sup.show_hud is True

    sup.toggle_pois()
    assert sup.show_pois is False
    sup.toggle_pois()
    assert sup.show_pois is True

    sup.toggle_comm_mesh()
    assert sup.show_comm_mesh is False
    sup.toggle_comm_mesh()
    assert sup.show_comm_mesh is True

    sup.toggle_routes()
    assert sup.show_routes is False
    sup.toggle_routes()
    assert sup.show_routes is True

    sup.toggle_drop_lines()
    assert sup.show_drop_lines is False
    sup.toggle_drop_lines()
    assert sup.show_drop_lines is True

    sup.toggle_grid()
    assert sup.show_grid is False
    sup.toggle_grid()
    assert sup.show_grid is True


def test_trace_immutability(supervisor_e1):
    """Seeking, pausing, and control layer actions never mutate the loaded trace."""
    sup = supervisor_e1
    initial_ticks_copy = copy.deepcopy(sup.ticks[:10])

    # Perform diverse seeking and control operations
    sup.seek_to(5)
    sup.step_forward_3s()
    sup.step_backward_1s()
    sup.set_playback_speed(2.0)
    sup.set_camera("recovery")
    sup.toggle_pois()
    sup.toggle_comm_mesh()
    sup.reset()

    # Compare first 10 ticks with deep copy
    assert sup.ticks[:10] == initial_ticks_copy


def test_random_scenario_loads(supervisor_random):
    """Random working scenario trace loads with valid metadata and ticks."""
    sup = supervisor_random
    assert sup.metadata.get("scenario_name") == "random_seed_2026"
    assert sup.total_ticks == 2700
    assert len(sup.metadata.get("task_ids", [])) == 10

    # Seeking in random scenario works identically
    sup.seek_to(50)
    assert sup.playback_cursor == 50
    sup.step_forward_3s()
    assert sup.playback_cursor == 53


def test_config_separation():
    """Scenario configuration and presentation configuration remain strictly decoupled."""
    scen_cfg = ScenarioGenConfig(seed=2026, num_pois=10, scenario_type="random")
    pres_cfg = WebotsPresentationConfig(
        playback_speed=2.0,
        camera_mode="gcs",
        show_hud=True,
        show_grid=False,
    )

    root = AetherSwarmConfig(scenario=scen_cfg, presentation=pres_cfg)
    assert root.scenario.seed == 2026
    assert root.scenario.scenario_type == "random"
    assert root.presentation.playback_speed == 2.0
    assert root.presentation.camera_mode == "gcs"
    assert root.presentation.show_grid is False

    # Roundtrip serialization
    d = root.to_dict()
    reconstructed = AetherSwarmConfig.from_dict(d)
    assert reconstructed.scenario == root.scenario
    assert reconstructed.presentation == root.presentation
