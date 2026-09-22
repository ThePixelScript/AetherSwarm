#!/usr/bin/env python3
"""Phase 4 Connectivity-Aware Planning: Baseline vs. Connectivity-Aware 3-Seed Comparison.

Runs identical mission configurations across 3 seeds with:
  - BASELINE: no connectivity-aware planner, no relay manager
  - CONNECTIVITY-AWARE: connectivity-aware planner + DynamicRelayManager enabled

All other parameters identical:
  - 100m comm range, 20m separation, 5 m/s, 1200s sortie, 2700s mission, 10s deadline
"""
from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

# Ensure src is on path
REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ares_swarm.simulation.scenario import (
    ChallengeAirspaceConfig,
    ChallengeProfileConfig,
    CommunicationConfig,
    DetectionPipelineConfig,
    ScenarioConfig,
)
from ares_swarm.simulation.runner import MissionRunner

# ---------------------------------------------------------------------------
# Canonical scenario: matches poc_round1.yaml constraints but parameterised
# for seed and planning flags.
# ---------------------------------------------------------------------------

def make_scenario(
    seed: int,
    enable_conn_planning: bool,
    enable_relay_mgr: bool,
    name_suffix: str = "",
) -> ScenarioConfig:
    """Build a deterministic 2700s mission scenario.

    All constraint parameters are fixed per the specification:
      - comm_range=100m, separation=20m, speed=5m/s
      - 1200s sortie limit, 2700s mission, 10s reporting deadline
      - POIs match poc_round1 positions/priorities/spawn_times
      - GCS at (-50, 500), arena 1000x1000

    Note: POIs at x=80 are ~152m from GCS, x=120 are ~188m — beyond direct
    link but within single-relay coverage (2×95m = 190m effective). This
    makes relay deployment meaningful.
    """
    mode = "conn_aware" if enable_conn_planning else "baseline"
    scenario_name = f"phase4_{mode}_seed{seed}{name_suffix}"

    gcs = (-50.0, 500.0)
    airspace = ChallengeAirspaceConfig(
        enabled=True,
        staging_pad_center=gcs,
        staging_pad_radius_m=15.0,
        corridor_bounds_x=(-50.0, 50.0),
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

    tasks = (
        {"id": "poi_01", "position": [80.0, 420.0],  "priority": 3, "spawn_time": 0.0,   "deadline_offset": 10.0, "service_duration": 2.0},
        {"id": "poi_02", "position": [80.0, 460.0],  "priority": 2, "spawn_time": 30.0,  "deadline_offset": 10.0, "service_duration": 2.0},
        {"id": "poi_03", "position": [80.0, 500.0],  "priority": 3, "spawn_time": 60.0,  "deadline_offset": 10.0, "service_duration": 2.0},
        {"id": "poi_04", "position": [80.0, 540.0],  "priority": 1, "spawn_time": 90.0,  "deadline_offset": 10.0, "service_duration": 2.0},
        {"id": "poi_05", "position": [80.0, 580.0],  "priority": 2, "spawn_time": 120.0, "deadline_offset": 10.0, "service_duration": 2.0},
        {"id": "poi_06", "position": [120.0, 420.0], "priority": 3, "spawn_time": 180.0, "deadline_offset": 10.0, "service_duration": 2.0},
        {"id": "poi_07", "position": [120.0, 460.0], "priority": 2, "spawn_time": 240.0, "deadline_offset": 10.0, "service_duration": 2.0},
        {"id": "poi_08", "position": [120.0, 500.0], "priority": 1, "spawn_time": 300.0, "deadline_offset": 10.0, "service_duration": 2.0},
        {"id": "poi_09", "position": [120.0, 540.0], "priority": 3, "spawn_time": 360.0, "deadline_offset": 10.0, "service_duration": 2.0},
        {"id": "poi_10", "position": [120.0, 580.0], "priority": 2, "spawn_time": 420.0, "deadline_offset": 10.0, "service_duration": 2.0},
    )

    return ScenarioConfig(
        name=scenario_name,
        seed=seed,
        dt=1.0,
        speed_limit=5.0,
        duration=2700.0,
        max_ticks=2700,
        gcs_position=gcs,
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
        tasks=tasks,
        challenge_profile=prof,
    )


def extract_metrics(res, planner=None) -> Dict[str, Any]:
    """Extract all required metrics from a MissionResult."""
    m = res.metrics_report
    cp = res.connectivity_planning if hasattr(res, "connectivity_planning") else {}
    if cp is None:
        cp = {}

    return {
        "completed_pois": m.tasks_completed,
        "completion_rate": round(m.mission_completion_rate, 4),
        "reporting_delivered": m.reports_delivered,
        "reporting_deadline_exceeded": m.reports_deadline_exceeded,
        "reporting_compliance": round(m.reporting_compliance_ratio, 4),
        "connectivity_availability": round(m.connectivity_availability, 4),
        "pdr": round(m.model_estimated_route_pdr, 4) if m.model_estimated_route_pdr is not None else None,
        "latency_ms": round(m.model_estimated_route_latency_ms, 2) if m.model_estimated_route_latency_ms is not None else None,
        "relay_assignments": m.relay_assignments,
        "relay_handoffs": m.relay_handoffs,
        # Phase 4 planning metrics
        "connectivity_feasibility_checks": m.connectivity_feasibility_checks,
        "connectivity_feasible_assignments": m.connectivity_feasible_assignments,
        "connectivity_rejected_assignments": m.connectivity_rejected_assignments,
        "connectivity_deferred_tasks": m.connectivity_deferred_tasks,
        "relay_required_for_assignment": m.relay_required_for_assignment,
        "connectivity_preserved_during_task": round(m.connectivity_preserved_during_task, 2),
        "communication_induced_replans": m.communication_induced_replans,
        # Safety / endurance
        "max_continuous_sortie_duration_s": round(m.max_continuous_sortie_duration_s, 1),
        "battery_exhaustion_count": m.battery_exhaustion_count,
        "separation_violation_count": m.separation_violation_count,
        "geofence_violation_count": m.geofence_violation_count,
    }


def run_seed(seed: int) -> Dict[str, Any]:
    """Run baseline and connectivity-aware missions for a given seed."""
    print(f"\n{'='*60}")
    print(f"  SEED {seed}")
    print(f"{'='*60}")

    # BASELINE: no connectivity planner, no relay manager
    print(f"  [BASELINE] seed={seed} ...")
    sc_base = make_scenario(seed=seed, enable_conn_planning=False, enable_relay_mgr=False)
    runner_base = MissionRunner(scenario=sc_base, seed=seed)
    res_base = runner_base.run()
    metrics_base = extract_metrics(res_base)
    print(f"    completed={metrics_base['completed_pois']}/10  compliance={metrics_base['reporting_compliance']}")

    # CONNECTIVITY-AWARE: planner + relay manager enabled
    print(f"  [CONN-AWARE] seed={seed} ...")
    sc_conn = make_scenario(seed=seed, enable_conn_planning=True, enable_relay_mgr=True)
    runner_conn = MissionRunner(scenario=sc_conn, seed=seed)
    res_conn = runner_conn.run()
    metrics_conn = extract_metrics(res_conn)
    print(f"    completed={metrics_conn['completed_pois']}/10  compliance={metrics_conn['reporting_compliance']}")
    print(f"    feasibility_checks={metrics_conn['connectivity_feasibility_checks']}  "
          f"feasible_assignments={metrics_conn['connectivity_feasible_assignments']}  "
          f"relay_required={metrics_conn['relay_required_for_assignment']}")

    return {
        "seed": seed,
        "baseline": metrics_base,
        "connectivity_aware": metrics_conn,
    }


def print_comparison_table(results: List[Dict]) -> None:
    """Print a formatted side-by-side comparison table."""
    METRICS = [
        ("completed_pois",                    "Completed POIs"),
        ("completion_rate",                    "Completion Rate"),
        ("reporting_delivered",                "Reporting Delivered"),
        ("reporting_deadline_exceeded",        "Deadline Exceeded"),
        ("reporting_compliance",               "Reporting Compliance"),
        ("connectivity_availability",          "Connectivity Availability"),
        ("pdr",                                "PDR (model est.)"),
        ("latency_ms",                         "Latency ms (model est.)"),
        ("relay_assignments",                  "Relay Assignments"),
        ("relay_handoffs",                     "Relay Handoffs"),
        ("connectivity_feasibility_checks",    "Feasibility Checks"),
        ("connectivity_feasible_assignments",  "Feasible Assignments"),
        ("connectivity_rejected_assignments",  "Rejected Assignments"),
        ("connectivity_deferred_tasks",        "Deferred Tasks"),
        ("relay_required_for_assignment",      "Relay-Required Assignments"),
        ("connectivity_preserved_during_task", "Conn. Preserved (s)"),
        ("communication_induced_replans",      "Comm-Induced Replans"),
        ("max_continuous_sortie_duration_s",   "Max Sortie Duration (s)"),
        ("battery_exhaustion_count",           "Battery Exhaustion"),
        ("separation_violation_count",         "Safety Violations"),
        ("geofence_violation_count",           "Geofence Violations"),
    ]

    seeds = [r["seed"] for r in results]
    header = f"{'Metric':<38}" + "".join(
        f"  {'S'+str(s)+'/BASE':>10}  {'S'+str(s)+'/CONN':>10}" for s in seeds
    )
    print("\n" + "=" * len(header))
    print("PHASE 4: BASELINE vs CONNECTIVITY-AWARE COMPARISON")
    print("=" * len(header))
    print(header)
    print("-" * len(header))

    for key, label in METRICS:
        row = f"{label:<38}"
        for r in results:
            bv = r["baseline"].get(key)
            cv = r["connectivity_aware"].get(key)
            bstr = str(bv) if bv is not None else "N/A"
            cstr = str(cv) if cv is not None else "N/A"
            row += f"  {bstr:>10}  {cstr:>10}"
        print(row)

    print("=" * len(header))


def main() -> None:
    seeds = [2026, 42, 5001]
    all_results = []

    for seed in seeds:
        result = run_seed(seed)
        all_results.append(result)

    print_comparison_table(all_results)

    # Save JSON artifact
    output_path = REPO_ROOT / "docs" / "phase4_comparison_results.json"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(all_results, f, indent=2)
    print(f"\n[INFO] Raw results saved to: {output_path}")


if __name__ == "__main__":
    main()
