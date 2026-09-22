"""Tests for DEMO-ONLY Randomized POI Scenario Generator.

Validates:
1. Deterministic reproducibility: same seed yields bit-identical POIs and spawn times.
2. Distinct seeds yield distinct spatial configurations.
3. Rejection sampling guarantees pairwise spacing >= min_spacing.
4. Operational arena boundaries and margin containment (no POI in corridor/staging x <= 0).
5. Deterministic tie-breaking on identical spawn times.
6. Authoritative consistency: exported trace task coordinates match scenario YAML coordinates.
7. Isolation: canonical E1 scenario and authoritative trace remain untouched.
"""
from __future__ import annotations

import math
from pathlib import Path
import sys

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from scripts.generate_random_demo import (
    sample_random_pois,
    generate_demo_scenario_dict,
    compute_poi_metrics,
    generate_and_export_demo,
)
from ares_swarm.simulation.scenario import load_scenario


def test_rejection_sampling_min_spacing():
    """All generated POIs must respect the configured minimum pairwise spacing."""
    min_spacing = 45.0
    tasks = sample_random_pois(seed=123, num_pois=10, min_spacing=min_spacing, margin=30.0)
    assert len(tasks) == 10

    for i in range(len(tasks)):
        for j in range(i + 1, len(tasks)):
            p1 = tasks[i]["position"]
            p2 = tasks[j]["position"]
            d = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
            assert d >= min_spacing - 1e-6, f"Pairwise spacing violation between {tasks[i]['id']} and {tasks[j]['id']}: {d:.2f} < {min_spacing}"


def test_rejection_sampling_arena_containment_and_margins():
    """POIs must strictly reside inside the operational arena away from margins and never in corridor."""
    margin = 40.0
    arena_size = 1000.0
    tasks = sample_random_pois(
        seed=999,
        num_pois=10,
        min_spacing=35.0,
        margin=margin,
        arena_size_x=arena_size,
        arena_size_y=arena_size,
    )
    assert len(tasks) == 10

    for t in tasks:
        x, y = t["position"]
        # Must be strictly positive (never inside corridor x <= 0)
        assert x >= margin, f"POI {t['id']} x={x} < margin {margin}"
        assert x <= arena_size - margin, f"POI {t['id']} x={x} > {arena_size - margin}"
        assert y >= margin, f"POI {t['id']} y={y} < margin {margin}"
        assert y <= arena_size - margin, f"POI {t['id']} y={y} > {arena_size - margin}"


def test_seed_reproducibility():
    """Identical seeds must produce identical POIs, positions, priorities, and spawn times."""
    tasks_a = sample_random_pois(seed=42, num_pois=10, min_spacing=40.0)
    tasks_b = sample_random_pois(seed=42, num_pois=10, min_spacing=40.0)
    assert tasks_a == tasks_b


def test_distinct_seeds_produce_different_configurations():
    """Different seeds must produce different spatial configurations."""
    tasks_1 = sample_random_pois(seed=42, num_pois=10, min_spacing=40.0)
    tasks_2 = sample_random_pois(seed=101, num_pois=10, min_spacing=40.0)
    pos_1 = [t["position"] for t in tasks_1]
    pos_2 = [t["position"] for t in tasks_2]
    assert pos_1 != pos_2


def test_deterministic_spawn_time_ordering():
    """Tasks must be ordered chronologically by spawn_time with id as tie-break."""
    tasks = sample_random_pois(seed=777, num_pois=10, min_spacing=30.0, spawn_window=(10.0, 50.0))
    for i in range(len(tasks) - 1):
        t1 = tasks[i]
        t2 = tasks[i + 1]
        assert (t1["spawn_time"], t1["id"]) <= (t2["spawn_time"], t2["id"])


def test_authoritative_consistency_scenario_and_trace(tmp_path):
    """Generated scenario YAML and simulation trace must agree exactly on task coordinates."""
    scen_path = tmp_path / "test_demo_scenario.yaml"
    trace_path = tmp_path / "test_demo_trace.json"

    res = generate_and_export_demo(
        seed=555,
        num_pois=10,
        min_spacing=40.0,
        output_scenario=scen_path,
        output_trace=trace_path,
        max_ticks=20,  # short test simulation
    )

    # Verify scenario loads correctly via core load_scenario
    cfg = load_scenario(scen_path)
    assert len(cfg.tasks) == 10
    assert cfg.min_separation_m == 20.0
    assert cfg.challenge_profile.enforce_separation is True
    assert cfg.challenge_profile.enforce_geofence is True

    # Read exported trace
    import json
    with open(trace_path, "r", encoding="utf-8") as f:
        trace_data = json.load(f)

    # Check trace task coordinates match scenario task coordinates
    scenario_positions = {t["id"]: t["position"] for t in res["tasks"]}
    trace_task_ids = trace_data["metadata"]["task_ids"]
    assert len(trace_task_ids) == 10

    tick_0_tasks = trace_data["ticks"][0]["tasks"]
    for tid, pos in scenario_positions.items():
        assert tid in tick_0_tasks
        trace_pos = tick_0_tasks[tid]["position"]
        assert pytest.approx(pos[0], abs=1e-3) == trace_pos[0]
        assert pytest.approx(pos[1], abs=1e-3) == trace_pos[1]


def test_frozen_e1_isolation():
    """Canonical E1 scenario and authoritative trace must never be modified by the demo generator."""
    e1_scen_path = REPO_ROOT / "scenarios" / "poc_round1.yaml"
    assert e1_scen_path.is_file()

    with open(e1_scen_path, "r", encoding="utf-8") as f:
        e1_data = yaml.safe_load(f)

    # E1 baseline grid POIs must remain exactly intact
    assert e1_data["name"] == "poc_round1_mission"
    assert e1_data["tasks"][0]["position"] == [80.0, 420.0]
    assert e1_data["tasks"][1]["position"] == [80.0, 460.0]
    assert e1_data["tasks"][2]["position"] == [80.0, 500.0]
    assert e1_data["tasks"][3]["position"] == [80.0, 540.0]
    assert e1_data["tasks"][4]["position"] == [80.0, 580.0]
    assert e1_data["tasks"][5]["position"] == [120.0, 420.0]
