"""Tests for DEMO-ONLY Randomized POI Scenario Generator.

Validates:
1. Default POI count: exactly 10 POIs generated.
2. Coordinate bounds: all generated POIs lie strictly within [5.0, 995.0] on both axes.
3. Absence of quadrant balancing: no forced sector coverage, intentional spread, or quadrant balance.
4. Deterministic reproducibility: same seed yields bit-identical POIs and spawn times.
5. Seed variability: distinct seeds yield distinct spatial configurations.
6. Direct uniform sampling: min_spacing_m=0 allows normal independent uniform placement without rejection.
7. Constrained placement: min_spacing_m>0 strictly enforces requested pairwise spacing constraint.
8. Deterministic tie-breaking on identical spawn times.
9. Authoritative consistency: exported trace task coordinates match scenario YAML coordinates.
10. Isolation: canonical E1 scenario and authoritative trace remain untouched.
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


def test_default_poi_count_is_ten():
    """Exactly 10 POIs remain the default."""
    tasks = sample_random_pois(seed=42)
    assert len(tasks) == 10


def test_pois_lie_within_arena_bounds():
    """All generated POIs must lie strictly within [5.0, 995.0] on both X and Y axes."""
    for s in (42, 101, 2026, 9999):
        tasks = sample_random_pois(seed=s)
        assert len(tasks) == 10
        for t in tasks:
            x, y = t["position"]
            assert 5.0 <= x <= 995.0, f"Seed {s}: POI {t['id']} x={x} out of [5.0, 995.0]"
            assert 5.0 <= y <= 995.0, f"Seed {s}: POI {t['id']} y={y} out of [5.0, 995.0]"


def test_no_quadrant_balancing_or_coverage_requirement():
    """Generator does NOT enforce quadrant balancing, sector coverage, or intentional spatial spreading.

    POIs are sampled randomly across the configured arena bounds; the generator does not enforce
    quadrant or regional distribution.
    """
    # Test across multiple seeds to verify no artificial balancing
    quadrant_counts = []
    for s in range(20):
        tasks = sample_random_pois(seed=s, num_pois=10, min_spacing=0.0)
        # Quadrants around center (500, 500)
        q1 = sum(1 for t in tasks if t["position"][0] >= 500.0 and t["position"][1] >= 500.0)
        q2 = sum(1 for t in tasks if t["position"][0] < 500.0 and t["position"][1] >= 500.0)
        q3 = sum(1 for t in tasks if t["position"][0] < 500.0 and t["position"][1] < 500.0)
        q4 = sum(1 for t in tasks if t["position"][0] >= 500.0 and t["position"][1] < 500.0)
        quadrant_counts.append((q1, q2, q3, q4))

    # Natural unconstrained uniform sampling will produce uneven distributions (e.g. 0 or >=5 in a single quadrant)
    has_uneven_distribution = any(any(count == 0 or count >= 5 for count in qc) for qc in quadrant_counts)
    assert has_uneven_distribution, "Expected natural uneven distribution without artificial quadrant balancing"


def test_same_seed_produces_bit_identical_scenario():
    """Same seed produces bit-for-bit identical scenario configuration and coordinates."""
    tasks_a = sample_random_pois(seed=2026)
    tasks_b = sample_random_pois(seed=2026)
    assert tasks_a == tasks_b


def test_different_seeds_produce_different_scenarios():
    """Different seeds produce different spatial configurations."""
    tasks_1 = sample_random_pois(seed=101)
    tasks_2 = sample_random_pois(seed=202)
    pos_1 = [t["position"] for t in tasks_1]
    pos_2 = [t["position"] for t in tasks_2]
    assert pos_1 != pos_2


def test_min_spacing_zero_allows_uniform_random_placement():
    """min_spacing_m=0 allows normal independent uniform random placement without spatial rejection."""
    tasks = sample_random_pois(seed=303, num_pois=10, min_spacing=0.0)
    assert len(tasks) == 10
    for t in tasks:
        x, y = t["position"]
        assert 5.0 <= x <= 995.0
        assert 5.0 <= y <= 995.0


def test_min_spacing_positive_enforces_spacing_constraint():
    """min_spacing_m > 0 strictly enforces the requested pairwise spacing constraint via rejection sampling."""
    min_spacing = 45.0
    tasks = sample_random_pois(seed=505, num_pois=10, min_spacing=min_spacing)
    assert len(tasks) == 10

    for i in range(len(tasks)):
        for j in range(i + 1, len(tasks)):
            p1 = tasks[i]["position"]
            p2 = tasks[j]["position"]
            d = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
            assert d >= min_spacing - 1e-6, f"Spacing violation between {tasks[i]['id']} and {tasks[j]['id']}: {d:.2f} < {min_spacing}"


def test_deterministic_spawn_time_ordering():
    """Tasks must be ordered chronologically by spawn_time with id as tie-break."""
    tasks = sample_random_pois(seed=777, num_pois=10, spawn_window=(10.0, 50.0))
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
        min_spacing=0.0,
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
