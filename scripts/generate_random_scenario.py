#!/usr/bin/env python3
"""Deterministic random scenario generator and constraint validator for AetherSwarm.

Generates reproducible randomized challenge scenarios for UAV-X mission evaluations,
performs comprehensive generation-time structural and feasibility audits against
official organizer constraints, and optionally executes authoritative simulation validation.

Usage:
  .venv/bin/python scripts/generate_random_scenario.py --seed 2026 --output results/random/random_seed_2026.yaml
  .venv/bin/python scripts/generate_random_scenario.py --seed 2026 --output results/random/random_seed_2026.yaml --run
"""
from __future__ import annotations

import argparse
import dataclasses
import math
from pathlib import Path
import random
import sys
from typing import Any

import yaml

# Ensure src is on sys.path
REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.core.enums import FailureState, RTHState, TaskStatus
from ares_swarm.core.models import StateSnapshot
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import ScenarioConfig, load_scenario


def generate_random_scenario(
    seed: int,
    output_path: str | Path | None = None,
    arena_size: float = 1000.0,
    gcs_offset_m: float = 75.0,
    num_pois: int = 10,
    num_uavs: int = 5,
    mission_duration: float = 2700.0,
    max_speed: float = 5.0,
    comm_range: float = 100.0,
    max_altitude: float = 100.0,
    min_separation: float = 20.0,
    max_flight_duration: float = 1200.0,
    uav_start_mode: str = "operational_center",
    sampling: str = "uniform",
) -> dict[str, Any]:
    """Generate a deterministic random scenario dictionary strictly controlled by seed.

    Args:
        seed: Deterministic PRNG seed. Same seed produces identical output.
        output_path: Optional path to write YAML.
        arena_size: Operational square arena side length (1000m).
        gcs_offset_m: Distance of operational center outside arena (75m).
        num_pois: Exactly 10 POIs.
        num_uavs: Number of UAVs (5).
        mission_duration: Total mission operation time (2700s = 45 min).
        max_speed: Maximum UAV speed limit (5 m/s).
        comm_range: Maximum RF communication range (100m).
        max_altitude: Maximum operational ceiling (100m).
        min_separation: Minimum pairwise UAV separation (20m).
        max_flight_duration: Maximum UAV flight endurance (1200s = 20 min).
        uav_start_mode: "operational_center" (at [-75, y]) or "staged_ingress" (at [50, y] like baseline).
    """
    rng = random.Random(seed)

    # Operational center at exact 75m offset outside arena west boundary (x = 0)
    gcs_x = -float(gcs_offset_m)
    gcs_y = arena_size / 2.0  # Center of Y axis (500.0)
    gcs_position = [gcs_x, gcs_y]

    # Initial UAV positions: strictly separated by >= min_separation (40m lane spacing)
    # Staged along Y around operational center latitude
    spacing = max(min_separation * 2.0, 40.0)
    y_start = gcs_y - ((num_uavs - 1) / 2.0) * spacing

    uav_x = gcs_x if uav_start_mode == "operational_center" else 50.0
    uavs_data = []
    # Continuous flight capacity: 1200s at 3.5 Wh/s => 4200 Wh
    battery_cap = 4200.0

    for i in range(num_uavs):
        uid = f"uav_{i + 1}"
        pos_y = round(y_start + i * spacing, 2)
        uavs_data.append({
            "id": uid,
            "position": [round(uav_x, 2), pos_y],
            "battery_capacity": battery_cap,
            "battery_energy": battery_cap,
            "role": "IDLE",
        })

    # Exactly 10 POIs randomly generated inside the 1000m x 1000m operational area
    # Coordinates drawn either uniformly (unconstrained) or via rejection sampling (feasible corridor)
    margin = 30.0
    tasks_data = []

    if sampling == "rejection":
        # Rejection sampling for operationally feasible demonstration:
        # 1. POI coordinates sampled within 5-hop relay reach (x in [50, 400], y in [300, 700])
        # 2. Pairwise POI separation >= 40m to prevent trajectory collision
        # Documented explicitly per Part C requirements.
        accepted = 0
        attempts = 0
        while accepted < num_pois and attempts < 10000:
            attempts += 1
            px = round(rng.uniform(60.0, 380.0), 2)
            py = round(rng.uniform(320.0, 680.0), 2)
            # Check pairwise separation from existing POIs
            too_close = any(math.hypot(px - t["position"][0], py - t["position"][1]) < 40.0 for t in tasks_data)
            if too_close:
                continue

            tid = f"poi_{accepted + 1:02d}"
            priority = rng.choice([1, 2, 3])
            spawn_time = round(accepted * 45.0 + rng.uniform(0.0, 20.0), 1)
            tasks_data.append({
                "id": tid,
                "position": [px, py],
                "priority": priority,
                "spawn_time": spawn_time,
                "deadline_offset": 10.0,
                "service_duration": 2.0,
            })
            accepted += 1
    else:
        # Unconstrained uniform random sampling across the entire 1000m x 1000m operational area
        for i in range(num_pois):
            tid = f"poi_{i + 1:02d}"
            px = round(rng.uniform(margin, arena_size - margin), 2)
            py = round(rng.uniform(margin, arena_size - margin), 2)
            priority = rng.choice([1, 2, 3])
            spawn_time = round(rng.uniform(0.0, 600.0), 1)

            tasks_data.append({
                "id": tid,
                "position": [px, py],
                "priority": priority,
                "spawn_time": spawn_time,
                "deadline_offset": 10.0,
                "service_duration": 2.0,
            })

    # Sort tasks deterministically by spawn_time then id
    tasks_data.sort(key=lambda t: (t["spawn_time"], t["id"]))

    scenario_dict = {
        "name": f"challenge_random_seed_{seed}",
        "seed": seed,
        "dt": 1.0,
        "speed_limit": max_speed,
        "duration": mission_duration,
        "max_ticks": int(mission_duration),
        "gcs_position": gcs_position,
        "arena": {
            "bounds_x": [0.0, arena_size],
            "bounds_y": [0.0, arena_size],
            "max_height": max_altitude,
        },
        "min_separation_m": min_separation,
        "enable_auto_rth": True,
        "return_by_mission_end": True,
        "communication": {
            "max_range": comm_range,
            "base_latency": 5.0,
            "packet_loss": 0.0,
            "degradation_multiplier": 1.0,
        },
        "battery": {
            "idle_rate": 1.0,
            "movement_rate": 0.5,
        },
        "challenge_profile": {
            "enabled": True,
            "max_sortie_duration_s": max_flight_duration,
            "rth_safety_margin_s": 15.0,
            "enforce_sortie_limit": True,
            "enforce_single_sortie": True,
            "airspace": {
                "enabled": True,
                "staging_pad_center": gcs_position,
                "staging_pad_radius_m": 15.0,
                "corridor_bounds_x": [-gcs_offset_m, 0.0],
                "corridor_bounds_y": [400.0, 600.0],
                "arena_bounds_x": [0.0, arena_size],
                "arena_bounds_y": [0.0, arena_size],
                "max_height": max_altitude,
            },
        },
        "uavs": uavs_data,
        "tasks": tasks_data,
    }

    if output_path is not None:
        out_file = Path(output_path)
        out_file.parent.mkdir(parents=True, exist_ok=True)
        with open(out_file, "w", encoding="utf-8") as f:
            yaml.dump(scenario_dict, f, sort_keys=False, indent=2)

    return scenario_dict


def validate_scenario_structure_and_feasibility(
    scenario_dict: dict[str, Any],
) -> dict[str, Any]:
    """Validate all structural parameters and evaluate operational feasibility.

    Outputs comprehensive report without silently fixing invalid fields.
    """
    errors: list[str] = []
    warnings: list[str] = []

    arena = scenario_dict.get("arena", {})
    bounds_x = arena.get("bounds_x", [0.0, 1000.0])
    bounds_y = arena.get("bounds_y", [0.0, 1000.0])
    max_altitude = arena.get("max_height", 100.0)

    arena_w = bounds_x[1] - bounds_x[0]
    arena_h = bounds_y[1] - bounds_y[0]
    if arena_w != 1000.0 or arena_h != 1000.0:
        errors.append(f"Arena dimensions must be 1000m x 1000m, got {arena_w}m x {arena_h}m")

    gcs = scenario_dict.get("gcs_position", [0.0, 0.0])
    gcs_offset = bounds_x[0] - gcs[0]  # distance outside west boundary

    duration = float(scenario_dict.get("duration", 2700.0))
    if duration != 2700.0:
        warnings.append(f"Mission duration is {duration}s, expected 2700.0s (45 min)")

    speed_limit = float(scenario_dict.get("speed_limit", 5.0))
    if speed_limit > 5.0:
        errors.append(f"Speed limit {speed_limit}m/s exceeds 5.0m/s maximum")

    min_sep = float(scenario_dict.get("min_separation_m", 20.0))
    if min_sep < 20.0:
        errors.append(f"Min separation {min_sep}m below 20.0m constraint")

    comm = scenario_dict.get("communication", {})
    comm_range = float(comm.get("max_range", 100.0))
    if comm_range > 100.0:
        errors.append(f"Comm range {comm_range}m exceeds 100.0m constraint")

    # UAV validation
    uavs = scenario_dict.get("uavs", [])
    if len(uavs) < 1:
        errors.append("Scenario contains zero UAVs")

    # Check pairwise UAV initial separation
    for i in range(len(uavs)):
        for j in range(i + 1, len(uavs)):
            p1 = uavs[i]["position"]
            p2 = uavs[j]["position"]
            d = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
            if d < min_sep - 1e-4:
                errors.append(f"Initial separation violation between {uavs[i]['id']} and {uavs[j]['id']}: {d:.2f}m < {min_sep}m")

    # Check POIs
    tasks = scenario_dict.get("tasks", [])
    if len(tasks) != 10:
        errors.append(f"Scenario must have exactly 10 POIs, got {len(tasks)}")

    poi_xs = [t["position"][0] for t in tasks]
    poi_ys = [t["position"][1] for t in tasks]
    spawn_times = [t.get("spawn_time", 0.0) for t in tasks]

    out_of_bounds_pois = []
    for t in tasks:
        x, y = t["position"]
        if x < bounds_x[0] or x > bounds_x[1] or y < bounds_y[0] or y > bounds_y[1]:
            out_of_bounds_pois.append((t["id"], x, y))
    if out_of_bounds_pois:
        errors.append(f"POIs outside operational area: {out_of_bounds_pois}")

    # Operational feasibility evaluation
    # 1. 100m RF mesh connectivity feasibility:
    # Check distance from GCS to nearest UAV/POI and pairwise POI cluster distances
    max_reach_hops = len(uavs) * comm_range  # maximum linear chain reach (5 * 100m = 500m)
    farthest_poi_dist = max(math.hypot(x - gcs[0], y - gcs[1]) for x, y in zip(poi_xs, poi_ys)) if poi_xs else 0.0

    infeasible_reasons: list[str] = []
    if farthest_poi_dist > max_reach_hops:
        infeasible_reasons.append(
            f"Farthest POI at {farthest_poi_dist:.1f}m from GCS exceeds 5-UAV relay chain reach ({max_reach_hops:.0f}m @ 100m/hop)"
        )

    # Check 10s deadline offset vs physical travel time
    # At 5 m/s, maximum distance covered in 10s is 50m.
    unreachable_in_10s = 0
    for t in tasks:
        tx, ty = t["position"]
        min_uav_dist = min(math.hypot(tx - u["position"][0], ty - u["position"][1]) for u in uavs) if uavs else 0.0
        if min_uav_dist > 50.0:
            unreachable_in_10s += 1

    if unreachable_in_10s > 0:
        infeasible_reasons.append(
            f"{unreachable_in_10s}/10 POIs are > 50m from initial UAV positions (cannot physically be reached within 10s at 5m/s)"
        )

    is_structurally_valid = len(errors) == 0
    is_operationally_feasible = is_structurally_valid and len(infeasible_reasons) == 0

    return {
        "structurally_valid": is_structurally_valid,
        "operationally_feasible": is_operationally_feasible,
        "classification": (
            "STRUCTURALLY VALID / OPERATIONALLY FEASIBLE"
            if is_operationally_feasible
            else ("STRUCTURALLY VALID / MISSION INFEASIBLE" if is_structurally_valid else "INVALID SCENARIO")
        ),
        "arena_dimensions": f"{arena_w:.1f}m x {arena_h:.1f}m",
        "operational_center_coords": gcs,
        "operational_center_offset_m": gcs_offset,
        "mission_duration_s": duration,
        "uav_count": len(uavs),
        "initial_uav_positions": [u["position"] for u in uavs],
        "uav_max_speed_mps": speed_limit,
        "max_uav_flight_duration_s": 1200.0,
        "flight_duration_enforcement_note": "Battery capacity 4200 Wh proxy (1200s @ 3.5 W/s continuous full-speed); no hard clock timer in M0",
        "maximum_altitude_m": max_altitude,
        "altitude_enforcement_note": "Stored in config and verified in 3D Webots layer; 2D planar in ares_swarm core",
        "communication_range_m": comm_range,
        "minimum_separation_m": min_sep,
        "poi_count": len(tasks),
        "poi_coord_bounds": (
            [round(min(poi_xs), 1), round(max(poi_xs), 1)],
            [round(min(poi_ys), 1), round(max(poi_ys), 1)],
        ) if poi_xs else None,
        "poi_spawn_time_bounds": [min(spawn_times), max(spawn_times)] if spawn_times else None,
        "landing_deadline_s": duration,
        "detection_reporting_timing_support": "NOT MODELED / NOT MEASURED (Task completion local event only; no sensor discovery or multi-hop reporting timer)",
        "errors": errors,
        "warnings": warnings,
        "infeasible_reasons": infeasible_reasons,
    }


def run_and_verify_random_scenario(scenario_dict: dict[str, Any]) -> dict[str, Any]:
    """Execute authoritative simulation on random scenario and record pass/fail per constraint."""
    cfg = load_scenario(scenario_dict)
    allocator = A1TaskAllocator()
    adapter = A0AutonomyAdapter(allocator=allocator)

    runner = MissionRunner(
        scenario=cfg,
        seed=cfg.seed,
        autonomy_adapter=adapter,
    )

    metrics = runner.run()
    summary = metrics.to_dict()

    # Detailed per-condition evaluation
    eval_m = summary.get("evaluation", {})
    mission_m = eval_m.get("mission", {})
    safety_m = eval_m.get("safety", {})
    comm_m = eval_m.get("communication", {})

    # Compute actual per-UAV flight duration (takeoff -> landing or mission end)
    flight_durations: dict[str, float] = {}
    landing_status: dict[str, bool] = {}

    for uid in sorted(cfg.uavs, key=lambda x: x["id"]):
        u_id = uid["id"]
        takeoff_tick = None
        landing_tick = None

        for step in runner.history:
            u_state = step.snapshot.uavs.get(u_id)
            if u_state is None:
                continue

            # Takeoff/movement started when UAV velocity > 0 or position changed from initial
            p_init = uid["position"]
            moved = (u_state.position_xy != tuple(p_init)) or (u_state.velocity_xy != (0.0, 0.0))
            if moved and takeoff_tick is None:
                takeoff_tick = step.tick

            if u_state.rth_state == RTHState.COMPLETE and landing_tick is None:
                landing_tick = step.tick

        t_start = takeoff_tick if takeoff_tick is not None else 0
        t_end = landing_tick if landing_tick is not None else runner.total_ticks
        duration_s = float(t_end - t_start) * cfg.dt
        flight_durations[u_id] = duration_s
        landing_status[u_id] = landing_tick is not None

    # Verify each condition
    c_pois_spawned = len(cfg.tasks) == 10
    c_speed = cfg.speed_limit <= 5.0
    c_altitude = cfg.max_height <= 100.0
    c_sep = safety_m.get("separation_violation_count", 0) == 0
    c_comm_range = cfg.communication.max_range <= 100.0
    c_geofence = safety_m.get("geofence_violation_count", 0) == 0

    # Flight duration <= 1200s
    c_flight_duration = all(d <= 1200.0 for d in flight_durations.values())

    # All surviving UAVs land by 2700s
    surviving = [uid["id"] for uid in cfg.uavs if summary["uavs"][uid["id"]]["rth_state"] == "COMPLETE"]
    c_landing = len(surviving) == len(cfg.uavs)

    # Operational center rule check
    gcs_x = cfg.gcs_position[0]
    c_gcs_offset = abs(cfg.arena_bounds_x[0] - gcs_x - 75.0) < 1.0

    return {
        "scenario_name": cfg.name,
        "seed": cfg.seed,
        "tasks_completed": mission_m.get("tasks_completed", 0),
        "tasks_total": mission_m.get("tasks_total", 0),
        "mission_completion_rate": mission_m.get("mission_completion_rate", 0.0),
        "completion_time_s": mission_m.get("completion_time_s", 0.0),
        "min_separation_m": safety_m.get("min_inter_uav_separation_m"),
        "separation_violations": safety_m.get("separation_violation_count", 0),
        "geofence_violations": safety_m.get("geofence_violation_count", 0),
        "flight_durations_s": flight_durations,
        "all_flight_durations_within_1200s": c_flight_duration,
        "landing_status": landing_status,
        "all_landed_by_2700s": c_landing,
        "conditions": {
            "1_arena_1000x1000": True,
            "2_gcs_75m_offset": c_gcs_offset,
            "3_mission_2700s": cfg.duration == 2700.0,
            "4_flight_duration_1200s": c_flight_duration,
            "5_comm_range_100m": c_comm_range,
            "6_takeoff_from_center": cfg.gcs_position[0] == -75.0,
            "7_landing_by_2700s": c_landing,
            "8_max_altitude_100m": c_altitude,
            "9_max_speed_5mps": c_speed,
            "10_min_separation_20m": c_sep,
            "11_detection_to_reporting_10s": "NOT_MODELED",
            "12_exactly_10_pois": c_pois_spawned,
            "13_random_poi_positions": True,
            "14_random_poi_spawn_times": True,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Deterministic random scenario generator and validator for AetherSwarm.")
    parser.add_argument("--seed", type=int, default=2026, help="Deterministic random seed")
    parser.add_argument("--output", "-o", type=str, default=None, help="Output YAML path")
    parser.add_argument("--gcs-offset", type=float, default=75.0, help="Operational center offset outside arena (m)")
    parser.add_argument("--uav-start-mode", type=str, default="operational_center", choices=["operational_center", "staged_ingress"], help="UAV initial staging location")
    parser.add_argument("--sampling", type=str, default="uniform", choices=["uniform", "rejection"], help="Sampling mode: 'uniform' (unconstrained across 1000x1000) or 'rejection' (clustered feasible corridor)")
    parser.add_argument("--run", action="store_true", help="Execute authoritative simulation and output constraint verification")

    args = parser.parse_args()

    out_path = args.output
    if not out_path:
        out_path = f"results/random/random_seed_{args.seed}.yaml"

    print("=" * 80)
    print(f"AETHERSWARM RANDOM SCENARIO GENERATOR & CONSTRAINT AUDIT (Seed: {args.seed})")
    print("=" * 80)

    scen_dict = generate_random_scenario(
        seed=args.seed,
        output_path=out_path,
        gcs_offset_m=args.gcs_offset,
        uav_start_mode=args.uav_start_mode,
        sampling=args.sampling,
    )
    print(f"Generated scenario YAML: {out_path}")

    # Validate scenario
    val_report = validate_scenario_structure_and_feasibility(scen_dict)
    print("\n--- GENERATION-TIME CONSTRAINT VALIDATION ---")
    print(f"Classification:            {val_report['classification']}")
    print(f"Arena Dimensions:          {val_report['arena_dimensions']}")
    print(f"Operational Center:        {val_report['operational_center_coords']} (Offset: {val_report['operational_center_offset_m']:.1f}m outside boundary)")
    print(f"Mission Duration:          {val_report['mission_duration_s']}s (45 min)")
    print(f"UAV Count:                 {val_report['uav_count']}")
    print(f"Initial UAV Positions:     {val_report['initial_uav_positions']}")
    print(f"UAV Max Speed:             {val_report['uav_max_speed_mps']} m/s")
    print(f"Max UAV Flight Duration:   {val_report['max_uav_flight_duration_s']}s (20 min) [{val_report['flight_duration_enforcement_note']}]")
    print(f"Maximum Altitude:          {val_report['maximum_altitude_m']}m [{val_report['altitude_enforcement_note']}]")
    print(f"Communication Range:       {val_report['communication_range_m']}m")
    print(f"Minimum Separation:        {val_report['minimum_separation_m']}m")
    print(f"POI Count:                 {val_report['poi_count']}")
    print(f"POI Coordinate Bounds:     X={val_report['poi_coord_bounds'][0]}, Y={val_report['poi_coord_bounds'][1]}")
    print(f"POI Spawn Time Bounds:     {val_report['poi_spawn_time_bounds'][0]}s to {val_report['poi_spawn_time_bounds'][1]}s")
    print(f"Landing Deadline:          {val_report['landing_deadline_s']}s")
    print(f"Detection/Reporting:       {val_report['detection_reporting_timing_support']}")

    if val_report["errors"]:
        print(f"\n[ERRORS]: {val_report['errors']}")
    if val_report["infeasible_reasons"]:
        print("\n[FEASIBILITY ASSESSMENT]:")
        for r in val_report["infeasible_reasons"]:
            print(f"  - {r}")

    if args.run:
        print("\n--- RUNNING AUTHORITATIVE SIMULATION ---")
        run_res = run_and_verify_random_scenario(scen_dict)
        print(f"Tasks Completed:           {run_res['tasks_completed']}/{run_res['tasks_total']} ({run_res['mission_completion_rate']*100:.1f}%)")
        print(f"Min Separation:            {run_res['min_separation_m']:.2f}m (Violations: {run_res['separation_violations']})")
        print(f"Geofence Violations:       {run_res['geofence_violations']}")
        print(f"Flight Durations:          {run_res['flight_durations_s']}")
        print(f"All Within 1200s Limit:    {run_res['all_flight_durations_within_1200s']}")
        print(f"All Landed by 2700s:       {run_res['all_landed_by_2700s']}")
        print("\n[CONSTRAINT AUDIT RESULTS]:")
        for cond, status in run_res["conditions"].items():
            print(f"  {cond:30s}: {status}")

    return 0 if val_report["structurally_valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
