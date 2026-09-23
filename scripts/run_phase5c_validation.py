#!/usr/bin/env python3
"""Deterministic Phase 5C Controlled Validation Experiment Runner."""
from __future__ import annotations

import json
from pathlib import Path
import sys

# Ensure src and scripts are on sys.path
REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from generate_scenario import sample_random_pois
from run_fleet_sweep import build_sweep_scenario
from ares_swarm.simulation.runner import MissionRunner
from dataclasses import replace

SEEDS = [2026, 42, 5001]
FLEET_SIZE = 8
DURATION = 300.0


def run_experiment(seed: int, multihop: bool) -> dict:
    # Sample 10 POIs using seed
    pois = sample_random_pois(seed=seed, num_pois=10)
    scenario = build_sweep_scenario(seed=seed, num_uavs=FLEET_SIZE, tasks=pois)
    
    # Configure duration and multihop flag
    scenario = replace(
        scenario,
        duration=DURATION,
        max_ticks=int(DURATION),
        challenge_profile=replace(
            scenario.challenge_profile,
            enable_connectivity_aware_planning=multihop,
            enable_relay_manager=multihop,
        )
    )
    
    runner = MissionRunner(scenario=scenario, seed=seed)
    result = runner.run()
    
    metrics = result.metrics_report
    assert metrics is not None
    
    return {
        "seed": seed,
        "multihop": multihop,
        "tasks_completed": metrics.tasks_completed,
        "tasks_total": metrics.tasks_total,
        "completion_rate": round(metrics.mission_completion_rate, 4),
        "geofence_violations": metrics.geofence_violation_count,
        "separation_violations": metrics.separation_violation_count,
        "battery_exhaustions": metrics.battery_exhaustion_count,
        "reports_delivered": metrics.reports_delivered,
        "reports_deadline_exceeded": metrics.reports_deadline_exceeded,
        "geofence_interventions": metrics.geofence_interventions,
    }


def main():
    print("=== RUNNING PHASE 5C CONTROLLED VALIDATION ===")
    results = []
    for seed in SEEDS:
        for multihop in [False, True]:
            res = run_experiment(seed, multihop)
            results.append(res)
            print(
                f"Seed {seed:4d} | MultiHop: {str(multihop):5s} | "
                f"Completed: {res['tasks_completed']}/{res['tasks_total']} | "
                f"Deadline Exceeded: {res['reports_deadline_exceeded']} | "
                f"Geofence Violations: {res['geofence_violations']} | "
                f"Separation Violations: {res['separation_violations']}"
            )
            
    summary_path = REPO_ROOT / "results" / "phase5c_validation_summary.json"
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    print(f"\nSaved summary to {summary_path}")


if __name__ == "__main__":
    main()
