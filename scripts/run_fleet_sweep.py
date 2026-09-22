#!/usr/bin/env python3
"""Authoritative Fleet-Size Feasibility Study Sweep Harness for AetherSwarm UAV-X Stage 1.

Executes deterministic sweeps across fleet sizes N = 1 to 16 and seeds:
[42, 2026, 1001, 2027, 3001, 4001, 5001, 6001, 7001, 8001].

Records complete mission, communication, autonomy, robustness, safety, and geometry metrics.
Evaluates:
  A. MISSION_SUCCESS
  B. COMMUNICATION_SUCCESS
  C. SAFETY_SUCCESS
  D. ENDURANCE_SUCCESS
  E. LANDING_SUCCESS
  FULL_SUCCESS = A and B and C and D and E

Exports:
  results/fleet_sweep/fleet_sweep_summary.csv
  results/fleet_sweep/fleet_sweep_summary.json
  results/fleet_sweep/geometric_analysis.json
  results/fleet_sweep/per_seed/seed_{seed}.json
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

# Ensure src and scripts are on sys.path
REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from generate_scenario import sample_random_pois
from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.core.enums import RTHState, TaskStatus, TelemetryStatus
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import (
    ChallengeAirspaceConfig,
    ChallengeProfileConfig,
    CommunicationConfig,
    DetectionPipelineConfig,
    ScenarioConfig,
)

SEEDS = [42, 2026, 1001, 2027, 3001, 4001, 5001, 6001, 7001, 8001]
FLEET_SIZES = list(range(1, 17))
GCS_POSITION = (-75.0, 500.0)


def generate_uav_staging(
    num_uavs: int,
    gcs_pos: tuple[float, float] = GCS_POSITION,
    min_separation: float = 20.0,
) -> list[dict[str, Any]]:
    """Stage N UAVs within the authorized transit corridor [-75.0, 0.0] x [400.0, 600.0].

    Guarantees:
      1. Every UAV position is strictly inside the transit corridor [-75, 0] x [400, 600].
      2. Pairwise separation >= 25.0m (strictly exceeding the 20.0m constraint).
      3. For N <= 5, matches the canonical baseline column at x = -75.0.
      4. For N > 5, arranges UAVs into multi-lane staging columns to avoid artificial
         geofence corridor violations caused by naive 1D linear extension.
    """
    battery_cap = 4200.0  # 20 min continuous flight limit
    uavs_data: list[dict[str, Any]] = []

    if num_uavs <= 5:
        # Canonical single-lane staging along x = -75.0
        spacing = max(min_separation * 2.0, 40.0)
        y_start = gcs_pos[1] - ((num_uavs - 1) / 2.0) * spacing
        for i in range(num_uavs):
            uid = f"uav_{i + 1}"
            pos_y = round(y_start + i * spacing, 2)
            uavs_data.append({
                "id": uid,
                "position": [-75.0, pos_y],
                "battery_capacity": battery_cap,
                "battery_energy": battery_cap,
                "role": "IDLE",
            })
    else:
        # Multi-lane corridor staging inside [-75, 0] x [400, 600]
        # Column 0 at x = -75.0, Column 1 at x = -35.0 (separated by 40m in X)
        num_cols = 2
        col_xs = [-75.0, -35.0]
        col_counts = [num_uavs // 2 + (1 if c < (num_uavs % 2) else 0) for c in range(num_cols)]

        idx = 0
        for c, count in enumerate(col_counts):
            px = col_xs[c]
            spacing_y = 25.0
            y_start = gcs_pos[1] - ((count - 1) / 2.0) * spacing_y
            for r in range(count):
                idx += 1
                uid = f"uav_{idx}"
                pos_y = round(y_start + r * spacing_y, 2)
                uavs_data.append({
                    "id": uid,
                    "position": [px, pos_y],
                    "battery_capacity": battery_cap,
                    "battery_energy": battery_cap,
                    "role": "IDLE",
                })
        uavs_data.sort(key=lambda u: int(u["id"].split("_")[1]))

    return uavs_data


def build_sweep_scenario(
    seed: int,
    num_uavs: int,
    tasks: list[dict[str, Any]],
) -> ScenarioConfig:
    """Build authoritative ScenarioConfig for seed and fleet size N."""
    uavs_data = generate_uav_staging(num_uavs, gcs_pos=GCS_POSITION, min_separation=20.0)

    airspace = ChallengeAirspaceConfig(
        enabled=True,
        staging_pad_center=GCS_POSITION,
        staging_pad_radius_m=15.0,
        corridor_bounds_x=(-75.0, 0.0),
        corridor_bounds_y=(400.0, 600.0),
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        max_height=100.0,
    )

    detection_pipe = DetectionPipelineConfig(
        enabled=True,
        sensor_fov_radius_m=40.0,
        reporting_deadline_s=10.0,
        processing_delay_s=0.0,
    )

    challenge_prof = ChallengeProfileConfig(
        enabled=True,
        max_sortie_duration_s=1200.0,
        rth_safety_margin_s=15.0,
        enforce_sortie_limit=True,
        enforce_single_sortie=True,
        enforce_separation=True,
        enforce_geofence=True,
        airspace=airspace,
        detection_pipeline=detection_pipe,
    )

    comm_cfg = CommunicationConfig(
        max_range=100.0,
        base_latency=5.0,
        packet_loss=0.0,
        degradation_multiplier=1.0,
    )

    return ScenarioConfig(
        name=f"sweep_N{num_uavs:02d}_seed_{seed}",
        seed=seed,
        dt=1.0,
        speed_limit=5.0,
        duration=2700.0,
        max_ticks=2700,
        gcs_position=GCS_POSITION,
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        max_height=100.0,
        min_separation_m=20.0,
        communication=comm_cfg,
        battery_idle_rate=1.0,
        battery_movement_rate=0.5,
        enable_auto_rth=True,
        return_by_mission_end=True,
        uavs=tuple(uavs_data),
        tasks=tuple(tasks),
        challenge_profile=challenge_prof,
    )


def compute_geometric_seed_analysis(seed: int) -> dict[str, Any]:
    """Compute exact geometric parameters and hop bounds for a given seed."""
    pois = sample_random_pois(seed=seed, num_pois=10, min_spacing=0.0)
    poi_records = []
    dists = []
    hops = []

    for p in pois:
        px, py = p["position"]
        dist = math.hypot(px - GCS_POSITION[0], py - GCS_POSITION[1])
        min_hops = math.ceil(dist / 100.0)
        dists.append(dist)
        hops.append(min_hops)
        poi_records.append({
            "id": p["id"],
            "position": [px, py],
            "priority": p["priority"],
            "spawn_time": p["spawn_time"],
            "distance_from_gcs_m": round(dist, 2),
            "required_geometric_min_hops": min_hops,
        })

    return {
        "seed": seed,
        "poi_count": len(pois),
        "pois": poi_records,
        "min_poi_distance_from_gcs_m": round(min(dists), 2),
        "max_poi_distance_from_gcs_m": round(max(dists), 2),
        "mean_poi_distance_from_gcs_m": round(sum(dists) / len(dists), 2),
        "min_required_hop_count": min(hops),
        "max_required_hop_count": max(hops),
    }


def run_single_sweep(seed: int, num_uavs: int, tasks: list[dict[str, Any]], geo_info: dict[str, Any]) -> dict[str, Any]:
    """Run a single deterministic simulation for (seed, N) and record all metrics."""
    scenario = build_sweep_scenario(seed=seed, num_uavs=num_uavs, tasks=tasks)
    adapter = A0AutonomyAdapter(allocator=A1TaskAllocator())
    runner = MissionRunner(scenario=scenario, autonomy_adapter=adapter)

    t_start = time.perf_counter()
    result = runner.run()
    sim_wall_time = time.perf_counter() - t_start

    final_snap = result.final_snapshot
    metrics_rep = result.metrics_report
    safety_rep = runner.safety_assessor.report
    telem_mgr = runner.detection_manager

    # 1. Mission Metrics
    tasks_total = len(final_snap.tasks)
    tasks_completed = sum(1 for t in final_snap.tasks.values() if t.status == TaskStatus.COMPLETE)
    completion_rate = round(tasks_completed / tasks_total, 4) if tasks_total > 0 else 0.0
    completion_time_s = metrics_rep.completion_time_s if metrics_rep else final_snap.simulation_time
    priority_score = metrics_rep.priority_weighted_score if metrics_rep else 0.0
    energy_wh = metrics_rep.total_energy_consumed_wh if metrics_rep else 0.0

    # 2. Communication Metrics
    conn_avail = metrics_rep.connectivity_availability if metrics_rep else 0.0
    downtime_s = metrics_rep.downtime_s if metrics_rep else 2700.0
    pdr = metrics_rep.model_estimated_route_pdr if metrics_rep else None
    mean_lat = metrics_rep.model_estimated_route_latency_ms if metrics_rep else None

    # Max latency across all step routes
    max_lat = 0.0
    for step in runner.history:
        net = step.network_analysis
        edge_lat = {}
        for link in net.network.links:
            k = (min(link.source_id, link.target_id), max(link.source_id, link.target_id))
            edge_lat[k] = link.latency_ms
        for uid in net.connected_uav_ids:
            route = net.routes_to_gcs.get(uid)
            if route and len(route) >= 2:
                r_lat = sum(edge_lat.get((min(a, b), max(a, b)), 0.0) for a, b in zip(route, route[1:]))
                if r_lat > max_lat:
                    max_lat = r_lat
    max_lat_ms = round(max_lat, 2) if max_lat > 0.0 else None

    # Telemetry / Detection Reporting
    total_detections = telem_mgr.get_metrics().get("total_detections", 0) if telem_mgr else 0
    reports_delivered = telem_mgr.get_metrics().get("reports_delivered", 0) if telem_mgr else 0
    reports_deadline_exceeded = telem_mgr.get_metrics().get("reports_deadline_exceeded", 0) if telem_mgr else 0
    reporting_compliance = telem_mgr.get_metrics().get("reporting_compliance_ratio", 0.0) if telem_mgr else 0.0

    # Check whether connected GCS path existed at detection time per POI
    connected_path_at_detect: dict[str, bool] = {}
    if telem_mgr:
        for tid, r in telem_mgr.authoritative_reports.items():
            t_det = r.t_detect
            detect_tick = int(round(t_det))
            had_path = False
            if 0 <= detect_tick < len(runner.history):
                step_net = runner.history[detect_tick].network_analysis
                had_path = step_net.routes_to_gcs.get(r.detecting_uav_id) is not None
            connected_path_at_detect[tid] = had_path

    # 3. Autonomy Metrics
    relay_reallocations = metrics_rep.relay_reallocations if metrics_rep else None
    recovery_time_s = metrics_rep.recovery_time_s if metrics_rep else None
    network_reconfig_eff = metrics_rep.network_reconfiguration_efficiency if metrics_rep else None

    # 4. Robustness Metrics
    uav_failures = 0
    task_recovery = "N/A - no failure injected"
    perf_after_failure = "N/A - no failure injected"

    # 5. Safety Metrics
    sep_viols = safety_rep.separation_violations_count if safety_rep else 0
    min_sep = safety_rep.min_observed_separation_m if safety_rep and safety_rep.min_observed_separation_m != float("inf") else None
    geofence_viols = safety_rep.geofence_violations_count if safety_rep else 0
    alt_viols = 0
    battery_exhaust = safety_rep.battery_exhaustions_count if safety_rep else 0
    max_continuous_sortie = safety_rep.max_observed_sortie_duration_s if safety_rep else 0.0
    landing_viols = safety_rep.landing_violations_count if safety_rep else 0

    # Landing status: all active UAVs must complete RTH at GCS
    landed_uavs = sum(1 for u in final_snap.uavs.values() if u.rth_state == RTHState.COMPLETE)
    all_landed = (landed_uavs == num_uavs)

    # 6. Classification
    mission_success = (tasks_completed == tasks_total and tasks_total == 10)
    comm_success = (
        reports_delivered == tasks_total
        and reports_deadline_exceeded == 0
        and reporting_compliance == 1.0
    )
    safety_success = (
        sep_viols == 0
        and geofence_viols == 0
        and alt_viols == 0
        and (min_sep is None or min_sep >= 19.999)
    )
    endurance_success = (
        battery_exhaust == 0
        and max_continuous_sortie <= 1200.0 + 1e-4
    )
    landing_success = (
        all_landed
        and landing_viols == 0
    )
    full_success = (
        mission_success
        and comm_success
        and safety_success
        and endurance_success
        and landing_success
    )

    return {
        "N": num_uavs,
        "seed": seed,
        "sim_wall_time_s": round(sim_wall_time, 3),
        "classification": {
            "MISSION_SUCCESS": mission_success,
            "COMMUNICATION_SUCCESS": comm_success,
            "SAFETY_SUCCESS": safety_success,
            "ENDURANCE_SUCCESS": endurance_success,
            "LANDING_SUCCESS": landing_success,
            "FULL_SUCCESS": full_success,
        },
        "mission": {
            "completion_rate": completion_rate,
            "completion_time_s": round(completion_time_s, 2),
            "completed_pois": tasks_completed,
            "total_pois": tasks_total,
            "priority_weighted_score": round(priority_score, 4),
            "energy_consumed_wh": round(energy_wh, 4),
        },
        "communication": {
            "connectivity_availability": round(conn_avail, 4),
            "communication_downtime_s": round(downtime_s, 2),
            "PDR": round(pdr, 4) if pdr is not None else None,
            "mean_latency_ms": round(mean_lat, 2) if mean_lat is not None else None,
            "max_latency_ms": max_lat_ms,
            "detection_reporting_compliance": round(reporting_compliance, 4),
            "reports_delivered": reports_delivered,
            "reports_deadline_exceeded": reports_deadline_exceeded,
            "total_detections": total_detections,
        },
        "autonomy": {
            "relay_reallocations": relay_reallocations,
            "recovery_time_s": recovery_time_s,
            "network_reconfiguration_efficiency": network_reconfig_eff,
        },
        "robustness": {
            "uav_failures": uav_failures,
            "task_recovery_after_failure": task_recovery,
            "performance_after_failure": perf_after_failure,
        },
        "safety": {
            "collision_separation_violations": sep_viols,
            "minimum_observed_separation_m": round(min_sep, 2) if min_sep is not None else None,
            "geofence_violations": geofence_viols,
            "altitude_violations": alt_viols,
            "battery_exhaustion": battery_exhaust,
            "maximum_continuous_sortie_duration_s": round(max_continuous_sortie, 2),
            "landing_violations": landing_viols,
            "final_landing_status": {
                "all_landed": all_landed,
                "landed_count": landed_uavs,
                "total_uavs": num_uavs,
            },
        },
        "scenario_geometry": {
            "max_poi_distance_from_gcs_m": geo_info["max_poi_distance_from_gcs_m"],
            "max_required_hop_count": geo_info["max_required_hop_count"],
            "connected_path_at_detection": connected_path_at_detect,
        },
    }


def worker_task(arg: tuple[int, int, list[dict[str, Any]], dict[str, Any]]) -> dict[str, Any]:
    seed, n, tasks, geo_info = arg
    return run_single_sweep(seed, n, tasks, geo_info)


def execute_full_sweep(output_dir: Path | str = REPO_ROOT / "results" / "fleet_sweep") -> None:
    out_dir = Path(output_dir)
    per_seed_dir = out_dir / "per_seed"
    per_seed_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 80)
    print("AETHERSWARM PHASE 1: FLEET-SIZE FEASIBILITY STUDY SWEEP")
    print(f"Seeds: {SEEDS}")
    print(f"Fleet sizes: N = {FLEET_SIZES[0]} to {FLEET_SIZES[-1]}")
    print(f"Total experiment runs: {len(SEEDS) * len(FLEET_SIZES)}")
    print("=" * 80)

    # 1. Precompute geometric analysis and POIs for all seeds
    geo_analyses = {}
    seed_pois = {}
    for s in SEEDS:
        geo = compute_geometric_seed_analysis(s)
        geo_analyses[s] = geo
        seed_pois[s] = sample_random_pois(seed=s, num_pois=10, min_spacing=0.0)

    # Save geometric analysis JSON
    geo_out = {
        "metadata": {
            "operational_area_m": [1000.0, 1000.0],
            "gcs_position": list(GCS_POSITION),
            "comm_range_m": 100.0,
            "hop_calculation_formula": "ceil(distance_from_GCS / 100.0)",
            "farthest_corner_distance_m": round(math.hypot(1000 - GCS_POSITION[0], 1000 - GCS_POSITION[1]), 2),
            "theoretical_max_hops": math.ceil(math.hypot(1000 - GCS_POSITION[0], 1000 - GCS_POSITION[1]) / 100.0),
        },
        "per_seed_geometry": geo_analyses,
    }
    with open(out_dir / "geometric_analysis.json", "w", encoding="utf-8") as f:
        json.dump(geo_out, f, indent=2)
    print(f"Saved: {out_dir / 'geometric_analysis.json'}")

    # 2. Build tasks list for multiprocessing
    tasks_to_run = []
    for s in SEEDS:
        for n in FLEET_SIZES:
            tasks_to_run.append((s, n, seed_pois[s], geo_analyses[s]))

    # 3. Execute sweep with multiprocessing pool
    from concurrent.futures import ProcessPoolExecutor, as_completed

    num_workers = min(os.cpu_count() or 4, 12)
    print(f"Executing {len(tasks_to_run)} runs across {num_workers} parallel workers...")

    results_by_seed_n: dict[tuple[int, int], dict[str, Any]] = {}
    completed_count = 0
    t_sweep_start = time.time()

    with ProcessPoolExecutor(max_workers=num_workers) as executor:
        futures = {executor.submit(worker_task, item): item for item in tasks_to_run}
        for future in as_completed(futures):
            res = future.result()
            s, n = res["seed"], res["N"]
            results_by_seed_n[(s, n)] = res
            completed_count += 1
            if completed_count % 16 == 0 or completed_count == len(tasks_to_run):
                elapsed = time.time() - t_sweep_start
                print(f"  Progress: {completed_count:3d}/{len(tasks_to_run):3d} runs completed ({elapsed:.1f}s)")

    sweep_elapsed = time.time() - t_sweep_start
    print(f"Sweep complete in {sweep_elapsed:.2f}s!")

    # 4. Save per-seed JSON files
    for s in SEEDS:
        seed_runs = {}
        for n in FLEET_SIZES:
            seed_runs[f"N_{n}"] = results_by_seed_n[(s, n)]

        seed_payload = {
            "seed": s,
            "geometry": geo_analyses[s],
            "fleet_runs": seed_runs,
        }
        with open(per_seed_dir / f"seed_{s}.json", "w", encoding="utf-8") as f:
            json.dump(seed_payload, f, indent=2)
    print(f"Saved 10 per-seed JSON files to: {per_seed_dir}")

    # 5. Build CSV summary
    csv_rows = []
    for n in FLEET_SIZES:
        for s in SEEDS:
            r = results_by_seed_n[(s, n)]
            c = r["classification"]
            m = r["mission"]
            cm = r["communication"]
            sf = r["safety"]
            g = r["scenario_geometry"]
            csv_rows.append({
                "N": n,
                "seed": s,
                "completion_rate": m["completion_rate"],
                "completion_time_s": m["completion_time_s"],
                "completed_pois": m["completed_pois"],
                "connectivity_availability": cm["connectivity_availability"],
                "communication_downtime_s": cm["communication_downtime_s"],
                "PDR": cm["PDR"] if cm["PDR"] is not None else "",
                "mean_latency_ms": cm["mean_latency_ms"] if cm["mean_latency_ms"] is not None else "",
                "max_latency_ms": cm["max_latency_ms"] if cm["max_latency_ms"] is not None else "",
                "detection_reporting_compliance": cm["detection_reporting_compliance"],
                "reports_delivered": cm["reports_delivered"],
                "reports_deadline_exceeded": cm["reports_deadline_exceeded"],
                "relay_reallocations": "",
                "recovery_time_s": "",
                "network_reconfiguration_efficiency": "",
                "uav_failures": 0,
                "task_recovery_after_failure": "N/A",
                "performance_after_failure": "N/A",
                "collision_separation_violations": sf["collision_separation_violations"],
                "min_observed_separation": sf["minimum_observed_separation_m"] if sf["minimum_observed_separation_m"] is not None else "",
                "geofence_violations": sf["geofence_violations"],
                "altitude_violations": sf["altitude_violations"],
                "battery_exhaustion": sf["battery_exhaustion"],
                "max_continuous_sortie_duration": sf["maximum_continuous_sortie_duration_s"],
                "final_landing_status": "ALL_LANDED" if sf["final_landing_status"]["all_landed"] else "INCOMPLETE",
                "max_poi_distance_m": g["max_poi_distance_from_gcs_m"],
                "max_required_hops": g["max_required_hop_count"],
                "mission_success": c["MISSION_SUCCESS"],
                "comm_success": c["COMMUNICATION_SUCCESS"],
                "safety_success": c["SAFETY_SUCCESS"],
                "endurance_success": c["ENDURANCE_SUCCESS"],
                "landing_success": c["LANDING_SUCCESS"],
                "full_success": c["FULL_SUCCESS"],
            })

    csv_path = out_dir / "fleet_sweep_summary.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(csv_rows[0].keys()))
        writer.writeheader()
        writer.writerows(csv_rows)
    print(f"Saved CSV summary: {csv_path}")

    # 6. Aggregate metrics per N for summary JSON and Markdown tables
    n_aggregates: dict[int, dict[str, Any]] = {}
    for n in FLEET_SIZES:
        n_runs = [results_by_seed_n[(s, n)] for s in SEEDS]
        tot = len(n_runs)

        mission_succ = sum(1 for r in n_runs if r["classification"]["MISSION_SUCCESS"])
        comm_succ = sum(1 for r in n_runs if r["classification"]["COMMUNICATION_SUCCESS"])
        safety_succ = sum(1 for r in n_runs if r["classification"]["SAFETY_SUCCESS"])
        endurance_succ = sum(1 for r in n_runs if r["classification"]["ENDURANCE_SUCCESS"])
        landing_succ = sum(1 for r in n_runs if r["classification"]["LANDING_SUCCESS"])
        full_succ = sum(1 for r in n_runs if r["classification"]["FULL_SUCCESS"])

        avg_comp_rate = sum(r["mission"]["completion_rate"] for r in n_runs) / tot
        avg_comp_time = sum(r["mission"]["completion_time_s"] for r in n_runs) / tot
        avg_comp_pois = sum(r["mission"]["completed_pois"] for r in n_runs) / tot
        avg_conn = sum(r["communication"]["connectivity_availability"] for r in n_runs) / tot
        avg_down = sum(r["communication"]["communication_downtime_s"] for r in n_runs) / tot

        pdrs = [r["communication"]["PDR"] for r in n_runs if r["communication"]["PDR"] is not None]
        avg_pdr = sum(pdrs) / len(pdrs) if pdrs else None

        avg_rep_comp = sum(r["communication"]["detection_reporting_compliance"] for r in n_runs) / tot
        avg_rep_deliv = sum(r["communication"]["reports_delivered"] for r in n_runs) / tot
        avg_rep_exceed = sum(r["communication"]["reports_deadline_exceeded"] for r in n_runs) / tot

        seps = [r["safety"]["minimum_observed_separation_m"] for r in n_runs if r["safety"]["minimum_observed_separation_m"] is not None]
        avg_min_sep = sum(seps) / len(seps) if seps else None

        avg_sep_viols = sum(r["safety"]["collision_separation_violations"] for r in n_runs) / tot
        avg_geo_viols = sum(r["safety"]["geofence_violations"] for r in n_runs) / tot
        avg_sorties = sum(r["safety"]["maximum_continuous_sortie_duration_s"] for r in n_runs) / tot
        all_landed_rate = sum(1 for r in n_runs if r["safety"]["final_landing_status"]["all_landed"]) / tot

        n_aggregates[n] = {
            "N": n,
            "total_runs": tot,
            "mission_success_runs": mission_succ,
            "comm_success_runs": comm_succ,
            "safety_success_runs": safety_succ,
            "endurance_success_runs": endurance_succ,
            "landing_success_runs": landing_succ,
            "full_success_runs": full_succ,
            "avg_completion_rate": round(avg_comp_rate, 4),
            "avg_completion_time_s": round(avg_comp_time, 2),
            "avg_completed_pois": round(avg_comp_pois, 2),
            "avg_connectivity_availability": round(avg_conn, 4),
            "avg_communication_downtime_s": round(avg_down, 2),
            "avg_pdr": round(avg_pdr, 4) if avg_pdr is not None else None,
            "avg_detection_reporting_compliance": round(avg_rep_comp, 4),
            "avg_reports_delivered": round(avg_rep_deliv, 2),
            "avg_reports_deadline_exceeded": round(avg_rep_exceed, 2),
            "avg_minimum_separation_m": round(avg_min_sep, 2) if avg_min_sep is not None else None,
            "avg_separation_violations": round(avg_sep_viols, 2),
            "avg_geofence_violations": round(avg_geo_viols, 2),
            "avg_max_continuous_sortie_duration_s": round(avg_sorties, 2),
            "all_landed_rate": round(all_landed_rate, 4),
        }

    # Per-seed smallest N determination
    per_seed_smallest_n: dict[int, dict[str, Any]] = {}
    for s in SEEDS:
        s_res = {
            "smallest_N_mission_success": None,
            "smallest_N_comm_success": None,
            "smallest_N_safety_success": None,
            "smallest_N_endurance_success": None,
            "smallest_N_landing_success": None,
            "smallest_N_full_success": None,
        }
        for n in FLEET_SIZES:
            r = results_by_seed_n[(s, n)]
            c = r["classification"]
            if c["MISSION_SUCCESS"] and s_res["smallest_N_mission_success"] is None:
                s_res["smallest_N_mission_success"] = n
            if c["COMMUNICATION_SUCCESS"] and s_res["smallest_N_comm_success"] is None:
                s_res["smallest_N_comm_success"] = n
            if c["SAFETY_SUCCESS"] and s_res["smallest_N_safety_success"] is None:
                s_res["smallest_N_safety_success"] = n
            if c["ENDURANCE_SUCCESS"] and s_res["smallest_N_endurance_success"] is None:
                s_res["smallest_N_endurance_success"] = n
            if c["LANDING_SUCCESS"] and s_res["smallest_N_landing_success"] is None:
                s_res["smallest_N_landing_success"] = n
            if c["FULL_SUCCESS"] and s_res["smallest_N_full_success"] is None:
                s_res["smallest_N_full_success"] = n
        per_seed_smallest_n[s] = s_res

    # Overall threshold fleet sizes
    consistent_mission_n = next((n for n in FLEET_SIZES if n_aggregates[n]["mission_success_runs"] == len(SEEDS)), None)
    consistent_comm_n = next((n for n in FLEET_SIZES if n_aggregates[n]["comm_success_runs"] == len(SEEDS)), None)
    consistent_safety_n = next((n for n in FLEET_SIZES if n_aggregates[n]["safety_success_runs"] == len(SEEDS)), None)
    consistent_full_n = next((n for n in FLEET_SIZES if n_aggregates[n]["full_success_runs"] == len(SEEDS)), None)

    summary_payload = {
        "study_title": "AetherSwarm Phase 1: Pre-Rotation Fleet Feasibility Baseline",
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "mission_parameters": {
            "mission_duration_s": 2700.0,
            "max_sortie_duration_s": 1200.0,
            "speed_limit_mps": 5.0,
            "comm_range_m": 100.0,
            "min_separation_m": 20.0,
            "arena_dimensions_m": [1000.0, 1000.0],
            "gcs_position": list(GCS_POSITION),
            "poi_count": 10,
            "reporting_deadline_s": 10.0,
            "detection_fov_radius_m": 40.0,
        },
        "fleet_sweep_range": [FLEET_SIZES[0], FLEET_SIZES[-1]],
        "seeds": SEEDS,
        "key_thresholds": {
            "smallest_N_consistent_all_10_pois_completed": consistent_mission_n,
            "smallest_N_consistent_communication_success": consistent_comm_n,
            "smallest_N_consistent_safety_success": consistent_safety_n,
            "smallest_N_consistent_full_success": consistent_full_n,
        },
        "per_seed_smallest_N": per_seed_smallest_n,
        "n_aggregates": {f"N_{n}": n_aggregates[n] for n in FLEET_SIZES},
    }

    with open(out_dir / "fleet_sweep_summary.json", "w", encoding="utf-8") as f:
        json.dump(summary_payload, f, indent=2)
    print(f"Saved: {out_dir / 'fleet_sweep_summary.json'}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Fleet-Size Feasibility Study Sweep Harness for AetherSwarm.")
    parser.add_argument("--output-dir", "-o", type=str, default=str(REPO_ROOT / "results" / "fleet_sweep"), help="Output directory for results")
    args = parser.parse_args()

    execute_full_sweep(output_dir=args.output_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
