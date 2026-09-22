#!/usr/bin/env python3
"""Phase 4 Connectivity-Aware Planning: Randomized Robustness Validation (Seeds 2026, 42, 5001).

Executes deterministic randomized challenge scenarios generated via the authoritative
sample_random_pois generator across 3 distinct seeds:
  - BASELINE: connectivity-aware planning OFF, relay manager OFF
  - CONNECTIVITY-AWARE: connectivity-aware planning ON, DynamicRelayManager ON

All challenge/configuration parameters identical:
  - 100m comm range, 20m separation, 5 m/s speed limit
  - 1200s sortie limit, 2700s mission duration, 10s reporting deadline
  - 10 POIs sampled uniformly from [5.0, 995.0] x [5.0, 995.0]
  - No quadrant or sector balancing
  - 5 UAVs staged inside corridor / arena interface at x=50.0

Exports:
  results/phase4_randomized_comparison.json
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import sys
from typing import Any, Dict, List, Optional, Tuple

# Ensure src and scripts are on path
REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from generate_scenario import sample_random_pois
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import (
    ChallengeAirspaceConfig,
    ChallengeProfileConfig,
    CommunicationConfig,
    DetectionPipelineConfig,
    ScenarioConfig,
)

SEEDS = [2026, 42, 5001]
GCS_POSITION = (-50.0, 500.0)


def generate_seed_tasks(seed: int) -> List[Dict[str, Any]]:
    """Generate 10 deterministic random POIs for a given seed."""
    tasks = sample_random_pois(
        seed=seed,
        num_pois=10,
        min_spacing=0.0,
        x_range=(5.0, 995.0),
        y_range=(5.0, 995.0),
        spawn_window=(0.0, 300.0),
    )
    # Verification
    assert len(tasks) == 10, f"Seed {seed} generated {len(tasks)} POIs, expected 10"
    for t in tasks:
        x, y = t["position"]
        assert 5.0 <= x <= 995.0, f"Seed {seed}: POI {t['id']} x={x} out of bounds"
        assert 5.0 <= y <= 995.0, f"Seed {seed}: POI {t['id']} y={y} out of bounds"
        assert 0.0 <= t["spawn_time"] <= 300.0
    return tasks


def make_random_scenario(
    seed: int,
    tasks: List[Dict[str, Any]],
    enable_conn_planning: bool,
    enable_relay_mgr: bool,
    name_suffix: str = "",
) -> ScenarioConfig:
    """Build ScenarioConfig for a randomized seed and planning mode."""
    mode = "conn_aware" if enable_conn_planning else "baseline"
    scenario_name = f"phase4_random_{mode}_seed{seed}{name_suffix}"

    airspace = ChallengeAirspaceConfig(
        enabled=True,
        staging_pad_center=GCS_POSITION,
        staging_pad_radius_m=15.0,
        corridor_bounds_x=(-50.0, 0.0),
        corridor_bounds_y=(400.0, 600.0),
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        max_height=100.0,
    )
    detect_pipe = DetectionPipelineConfig(
        enabled=True,
        sensor_fov_radius_m=40.0,
        reporting_deadline_s=10.0,
        processing_delay_s=0.0,
    )
    prof = ChallengeProfileConfig(
        enabled=True,
        max_sortie_duration_s=1200.0,
        rth_safety_margin_s=15.0,
        recharge_duration_s=300.0,
        enforce_sortie_limit=True,
        enforce_single_sortie=False,
        enforce_separation=True,
        enforce_geofence=True,
        airspace=airspace,
        detection_pipeline=detect_pipe,
        enable_relay_manager=enable_relay_mgr,
        enable_connectivity_aware_planning=enable_conn_planning,
    )

    uavs = (
        {"id": "uav_1", "position": [50.0, 420.0], "battery_capacity": 4200.0, "battery_energy": 4200.0, "role": "IDLE"},
        {"id": "uav_2", "position": [50.0, 460.0], "battery_capacity": 4200.0, "battery_energy": 4200.0, "role": "IDLE"},
        {"id": "uav_3", "position": [50.0, 500.0], "battery_capacity": 4200.0, "battery_energy": 4200.0, "role": "IDLE"},
        {"id": "uav_4", "position": [50.0, 540.0], "battery_capacity": 4200.0, "battery_energy": 4200.0, "role": "IDLE"},
        {"id": "uav_5", "position": [50.0, 580.0], "battery_capacity": 4200.0, "battery_energy": 4200.0, "role": "IDLE"},
    )

    return ScenarioConfig(
        name=scenario_name,
        seed=seed,
        dt=1.0,
        speed_limit=5.0,
        duration=2700.0,
        max_ticks=2700,
        gcs_position=GCS_POSITION,
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        min_separation_m=20.0,
        communication=CommunicationConfig(max_range=100.0, base_latency=5.0, packet_loss=0.0),
        battery_idle_rate=1.0,
        battery_movement_rate=0.5,
        enable_auto_rth=True,
        return_by_mission_end=True,
        enable_relay_manager=enable_relay_mgr,
        enable_connectivity_aware_planning=enable_conn_planning,
        uavs=uavs,
        tasks=tuple(tasks),
        challenge_profile=prof,
    )


def extract_metrics(res) -> Dict[str, Any]:
    """Extract complete Phase 4 metrics report."""
    m = res.metrics_report
    return {
        # Mission
        "completed_pois": m.tasks_completed,
        "completion_rate": round(m.mission_completion_rate, 4),
        "completion_time": round(m.completion_time_s, 2),
        # Communication
        "reports_delivered": m.reports_delivered,
        "deadline_exceeded": m.reports_deadline_exceeded,
        "reporting_compliance": round(m.reporting_compliance_ratio, 4),
        "connectivity_availability": round(m.connectivity_availability, 4),
        "pdr": round(m.model_estimated_route_pdr, 4) if m.model_estimated_route_pdr is not None else None,
        "latency_ms": round(m.model_estimated_route_latency_ms, 2) if m.model_estimated_route_latency_ms is not None else None,
        # Planning
        "feasibility_checks": m.connectivity_feasibility_checks,
        "feasible_assignments": m.connectivity_feasible_assignments,
        "rejected_assignments": m.connectivity_rejected_assignments,
        "deferred_tasks": m.connectivity_deferred_tasks,
        "relay_required_assignments": m.relay_required_for_assignment,
        "relay_assignments": m.relay_assignments,
        "relay_handoffs": m.relay_handoffs,
        "communication_induced_replans": m.communication_induced_replans,
        "connectivity_preserved_during_task": round(m.connectivity_preserved_during_task, 2),
        # Safety / Endurance
        "separation_violations": m.separation_violation_count,
        "geofence_violations": m.geofence_violation_count,
        "battery_exhaustion": m.battery_exhaustion_count,
        "max_continuous_sortie_duration_s": round(m.max_continuous_sortie_duration_s, 1),
    }


def compute_geometry_summary(tasks: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Compute spatial distance statistics of POIs to GCS."""
    dists = [
        math.hypot(t["position"][0] - GCS_POSITION[0], t["position"][1] - GCS_POSITION[1])
        for t in tasks
    ]
    within_direct = sum(1 for d in dists if d <= 95.0)
    within_single_relay = sum(1 for d in dists if 95.0 < d <= 190.0)
    beyond_single_relay = sum(1 for d in dists if d > 190.0)

    return {
        "min_dist_to_gcs_m": round(min(dists), 1),
        "max_dist_to_gcs_m": round(max(dists), 1),
        "mean_dist_to_gcs_m": round(sum(dists) / len(dists), 1),
        "direct_reachable_count": within_direct,
        "single_relay_reachable_count": within_single_relay,
        "beyond_single_relay_count": beyond_single_relay,
        "pois": [
            {
                "id": t["id"],
                "position": t["position"],
                "priority": t["priority"],
                "spawn_time": t["spawn_time"],
                "distance_to_gcs_m": round(math.hypot(t["position"][0] - GCS_POSITION[0], t["position"][1] - GCS_POSITION[1]), 1),
            }
            for t in tasks
        ],
    }


def compute_aggregate(records: List[Dict[str, Any]], key: str) -> Dict[str, float]:
    """Compute mean, min, max for a given metric across records."""
    vals = [r[key] for r in records if r.get(key) is not None]
    if not vals:
        return {"mean": 0.0, "min": 0.0, "max": 0.0}
    return {
        "mean": round(sum(vals) / len(vals), 4),
        "min": round(min(vals), 4),
        "max": round(max(vals), 4),
    }


def run_randomized_comparison() -> Dict[str, Any]:
    """Run paired comparisons across all 3 seeds and compute aggregates."""
    print("=" * 80)
    print("PHASE 4: RANDOMIZED ROBUSTNESS VALIDATION (Seeds 2026, 42, 5001)")
    print("=" * 80)

    per_seed_results = []
    seed_tasks_map = {}

    # 1. Generate & verify distinct POI sets
    for seed in SEEDS:
        tasks = generate_seed_tasks(seed)
        seed_tasks_map[seed] = tasks
        geom = compute_geometry_summary(tasks)
        print(f"\n[SEED {seed}] 10 POIs generated:")
        print(f"  GCS distance range: {geom['min_dist_to_gcs_m']}m - {geom['max_dist_to_gcs_m']}m (mean: {geom['mean_dist_to_gcs_m']}m)")
        print(f"  Reachability: direct={geom['direct_reachable_count']}, single-relay={geom['single_relay_reachable_count']}, beyond-single-relay={geom['beyond_single_relay_count']}")

    # Verify seed variability
    pos_2026 = [t["position"] for t in seed_tasks_map[2026]]
    pos_42 = [t["position"] for t in seed_tasks_map[42]]
    pos_5001 = [t["position"] for t in seed_tasks_map[5001]]
    assert pos_2026 != pos_42 != pos_5001, "Error: POI positions must differ between seeds!"

    # 2. Run paired simulations
    for seed in SEEDS:
        tasks = seed_tasks_map[seed]
        geom = compute_geometry_summary(tasks)

        # Baseline run
        print(f"\n--- Running Seed {seed}: BASELINE ---")
        sc_base = make_random_scenario(seed, tasks, enable_conn_planning=False, enable_relay_mgr=False)
        res_base = MissionRunner(scenario=sc_base, seed=seed).run()
        m_base = extract_metrics(res_base)
        print(f"  Baseline: completed={m_base['completed_pois']}/10, compliance={m_base['reporting_compliance']}, availability={m_base['connectivity_availability']}")

        # Connectivity-aware run
        print(f"--- Running Seed {seed}: CONNECTIVITY-AWARE ---")
        sc_conn = make_random_scenario(seed, tasks, enable_conn_planning=True, enable_relay_mgr=True)
        res_conn = MissionRunner(scenario=sc_conn, seed=seed).run()
        m_conn = extract_metrics(res_conn)
        print(f"  Conn-Aware: completed={m_conn['completed_pois']}/10, compliance={m_conn['reporting_compliance']}, availability={m_conn['connectivity_availability']}")
        print(f"  Checks: {m_conn['feasibility_checks']}, Feasible: {m_conn['feasible_assignments']}, Rejected: {m_conn['rejected_assignments']}, Deferred: {m_conn['deferred_tasks']}, Relay-req: {m_conn['relay_required_assignments']}")

        per_seed_results.append({
            "seed": seed,
            "geometry": geom,
            "baseline": m_base,
            "connectivity_aware": m_conn,
        })

    # 3. Reproducibility test: re-run Seed 2026
    print("\n--- Running Seed 2026 REPRODUCIBILITY TEST ---")
    tasks_rep = generate_seed_tasks(2026)
    assert tasks_rep == seed_tasks_map[2026], "Seed 2026 failed identical POI reproduction!"
    sc_conn_rep = make_random_scenario(2026, tasks_rep, enable_conn_planning=True, enable_relay_mgr=True)
    res_conn_rep = MissionRunner(scenario=sc_conn_rep, seed=2026).run()
    m_conn_rep = extract_metrics(res_conn_rep)
    assert m_conn_rep == per_seed_results[0]["connectivity_aware"], "Seed 2026 failed bit-identical simulation reproducibility!"
    print("  Reproducibility confirmed: identical POIs and bit-identical metric results.")

    # 4. Compute aggregates
    baseline_records = [r["baseline"] for r in per_seed_results]
    conn_records = [r["connectivity_aware"] for r in per_seed_results]

    agg_keys = [
        "reporting_compliance",
        "connectivity_availability",
        "completed_pois",
        "pdr",
        "latency_ms",
        "relay_assignments",
        "communication_induced_replans",
    ]

    aggregates = {
        "baseline": {k: compute_aggregate(baseline_records, k) for k in agg_keys},
        "connectivity_aware": {k: compute_aggregate(conn_records, k) for k in agg_keys},
    }

    full_output = {
        "validation_type": "randomized_robustness_validation",
        "description": "Deterministic randomized challenge scenarios sampled uniformly across [5.0, 995.0]^2 arena. Demonstrates connectivity gating and single-relay limit behavior.",
        "seeds": SEEDS,
        "gcs_position": list(GCS_POSITION),
        "fleet_size": 5,
        "per_seed_results": per_seed_results,
        "aggregates": aggregates,
        "reproducibility_verified": True,
    }

    # 5. Print summary table
    print("\n" + "=" * 90)
    print("PHASE 4 RANDOMIZED COMPARISON TABLE")
    print("=" * 90)
    fmt_hdr = f"{'Metric':<32} | {'S2026 BASE':>10} {'S2026 CONN':>10} | {'S42 BASE':>10} {'S42 CONN':>10} | {'S5001 BASE':>10} {'S5001 CONN':>10}"
    print(fmt_hdr)
    print("-" * len(fmt_hdr))

    metrics_display = [
        ("completed_pois", "Completed POIs"),
        ("completion_rate", "Completion Rate"),
        ("completion_time", "Completion Time (s)"),
        ("reports_delivered", "Reports Delivered"),
        ("deadline_exceeded", "Deadline Exceeded"),
        ("reporting_compliance", "Reporting Compliance"),
        ("connectivity_availability", "Conn Availability"),
        ("pdr", "Route PDR"),
        ("latency_ms", "Route Latency (ms)"),
        ("feasibility_checks", "Feasibility Checks"),
        ("feasible_assignments", "Feasible Assignments"),
        ("rejected_assignments", "Rejected Assignments"),
        ("deferred_tasks", "Deferred Tasks"),
        ("relay_required_assignments", "Relay-Req Assignments"),
        ("relay_assignments", "Relay Assignments"),
        ("relay_handoffs", "Relay Handoffs"),
        ("communication_induced_replans", "Comm Replans"),
        ("connectivity_preserved_during_task", "Conn Preserved (s)"),
        ("separation_violations", "Separation Violations"),
        ("geofence_violations", "Geofence Violations"),
        ("battery_exhaustion", "Battery Exhaustions"),
        ("max_continuous_sortie_duration_s", "Max Sortie (s)"),
    ]

    for key, label in metrics_display:
        row = f"{label:<32} |"
        for r in per_seed_results:
            b_val = r["baseline"].get(key)
            c_val = r["connectivity_aware"].get(key)
            b_str = str(b_val) if b_val is not None else "N/A"
            c_str = str(c_val) if c_val is not None else "N/A"
            row += f" {b_str:>10} {c_str:>10} |"
        print(row)

    print("=" * 90)

    print("\n" + "=" * 80)
    print("AGGREGATE STATISTICS (Mean [Min, Max])")
    print("=" * 80)
    print(f"{'Metric':<32} | {'BASELINE':^20} | {'CONNECTIVITY-AWARE':^20}")
    print("-" * 80)
    for k in agg_keys:
        b = aggregates["baseline"][k]
        c = aggregates["connectivity_aware"][k]
        b_str = f"{b['mean']:.4f} [{b['min']}, {b['max']}]"
        c_str = f"{c['mean']:.4f} [{c['min']}, {c['max']}]"
        print(f"{k:<32} | {b_str:>20} | {c_str:>20}")
    print("=" * 80)

    # 6. Save JSON artifact
    out_dir = REPO_ROOT / "results"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_file = out_dir / "phase4_randomized_comparison.json"
    with open(out_file, "w", encoding="utf-8") as f:
        json.dump(full_output, f, indent=2)
    print(f"\n[INFO] Saved results to: {out_file}")

    return full_output


if __name__ == "__main__":
    run_randomized_comparison()
