#!/usr/bin/env python3
"""DEMO-ONLY Randomized POI Scenario and Webots Trace Generator for AetherSwarm.

================================================================================
CRITICAL ARCHITECTURAL DISTINCTION:
This tool is strictly DEMO / EXPERIMENTAL ONLY.
- It does NOT alter canonical benchmark scenarios (e.g. scenarios/poc_round1.yaml).
- It does NOT modify or replace the authoritative E1 evaluation trace.
- It does NOT alter core autonomy or safety enforcement semantics.
- Generated POI locations are authoritative task positions executed by
  MissionRunner / SimulationEngine and subsequently visualized in Webots.
- POIs are NEVER randomized inside the Webots controller; the Webots supervisor
  strictly replays the authoritative simulation trace.
================================================================================

Architecture Pipeline:
  Rejection Sampling (10 POIs, configurable min_spacing, arena margins)
  -> Deterministic Scenario YAML (scenarios/demo_random_seed_{seed}.yaml)
  -> Authoritative Simulation (MissionRunner with A1 Autonomy & Safety)
  -> Webots JSON Trace Export (visualization/webots/data/random_demo_trace.json)
  -> Faithful 3D Webots Visualization (aetherswarm_supervisor)

Usage:
  .venv/bin/python scripts/generate_random_demo.py --seed 42
  .venv/bin/python scripts/generate_random_demo.py --seed 101 --min-spacing 45.0
  .venv/bin/python scripts/generate_random_demo.py --seed 2026 --full-arena
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import random
import sys
from typing import Any

import yaml

# Ensure src and scripts are on sys.path
REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from export_webots_trace import export_trace


def sample_random_pois(
    seed: int,
    num_pois: int = 10,
    min_spacing: float = 40.0,
    margin: float = 30.0,
    arena_size_x: float = 1000.0,
    arena_size_y: float = 1000.0,
    x_range: tuple[float, float] | None = None,
    y_range: tuple[float, float] | None = None,
    spawn_window: tuple[float, float] = (0.0, 300.0),
    max_attempts: int = 20000,
) -> list[dict[str, Any]]:
    """Sample non-overlapping, non-grid POIs using rejection sampling.

    Guarantees:
      1. Exactly num_pois (10) generated.
      2. Pairwise 2D Euclidean distance >= min_spacing between all POIs.
      3. At least margin away from operational arena boundaries.
      4. Strictly inside the arena (x >= margin > 0, never in corridor/staging).
      5. Continuous coordinate distribution (no grid pattern or artificial clustering).
      6. Deterministic ordering when spawn times are identical (sorted by spawn_time, then id).
      7. Fully reproducible from seed.
    """
    rng = random.Random(seed)

    if x_range is not None:
        x_min, x_max = x_range
    else:
        # Default operational swarm demonstration corridor
        x_min, x_max = max(margin, 60.0), min(arena_size_x - margin, 420.0)

    if y_range is not None:
        y_min, y_max = y_range
    else:
        y_min, y_max = max(margin, 220.0), min(arena_size_y - margin, 780.0)

    # Clamp sampling bounds strictly to valid arena interior
    x_min = max(margin, min(x_min, arena_size_x - margin))
    x_max = max(x_min, min(x_max, arena_size_x - margin))
    y_min = max(margin, min(y_min, arena_size_y - margin))
    y_max = max(y_min, min(y_max, arena_size_y - margin))

    tasks: list[dict[str, Any]] = []
    attempts = 0

    while len(tasks) < num_pois and attempts < max_attempts:
        attempts += 1
        px = round(rng.uniform(x_min, x_max), 2)
        py = round(rng.uniform(y_min, y_max), 2)

        # Rejection: check pairwise distance against all previously accepted POIs
        too_close = any(
            math.hypot(px - t["position"][0], py - t["position"][1]) < min_spacing
            for t in tasks
        )
        if too_close:
            continue

        tid = f"poi_{len(tasks) + 1:02d}"
        priority = rng.choice([1, 2, 3])
        spawn_time = round(rng.uniform(spawn_window[0], spawn_window[1]), 1)

        tasks.append({
            "id": tid,
            "position": [px, py],
            "priority": priority,
            "spawn_time": spawn_time,
            "deadline_offset": 10.0,
            "service_duration": 2.0,
        })

    if len(tasks) < num_pois:
        raise RuntimeError(
            f"Failed to place {num_pois} POIs with min_spacing={min_spacing}m after {max_attempts} attempts. "
            f"Try decreasing min_spacing or expanding the sampling area."
        )

    # Deterministic tie-breaking: sort strictly by spawn_time then id
    tasks.sort(key=lambda t: (t["spawn_time"], t["id"]))
    return tasks


def generate_demo_scenario_dict(
    seed: int,
    tasks: list[dict[str, Any]],
    num_uavs: int = 5,
    arena_size: float = 1000.0,
    gcs_pos: tuple[float, float] = (-50.0, 500.0),
    mission_duration: float = 2700.0,
    speed_limit: float = 5.0,
    comm_range: float = 100.0,
    min_separation: float = 20.0,
) -> dict[str, Any]:
    """Build the scenario configuration dictionary for the random demo."""
    # Staging lane: 5 UAVs at x=50.0, spaced 40m apart along Y, matching Webots world base layout
    spacing = max(min_separation * 2.0, 40.0)
    y_start = gcs_pos[1] - ((num_uavs - 1) / 2.0) * spacing

    uavs_data = []
    battery_cap = 4200.0  # 20 min continuous flight limit

    for i in range(num_uavs):
        uid = f"uav_{i + 1}"
        pos_y = round(y_start + i * spacing, 2)
        uavs_data.append({
            "id": uid,
            "position": [50.0, pos_y],
            "battery_capacity": battery_cap,
            "battery_energy": battery_cap,
            "role": "IDLE",
        })

    return {
        "name": f"demo_random_seed_{seed}",
        "seed": seed,
        "dt": 1.0,
        "speed_limit": speed_limit,
        "duration": mission_duration,
        "max_ticks": int(mission_duration),
        "gcs_position": list(gcs_pos),
        "arena": {
            "bounds_x": [0.0, arena_size],
            "bounds_y": [0.0, arena_size],
            "max_height": 100.0,
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
            "enforce_separation": True,
            "enforce_geofence": True,
            "airspace": {
                "enabled": True,
                "staging_pad_center": list(gcs_pos),
                "staging_pad_radius_m": 15.0,
                "corridor_bounds_x": [float(gcs_pos[0]), 0.0],
                "corridor_bounds_y": [400.0, 600.0],
                "arena_bounds_x": [0.0, arena_size],
                "arena_bounds_y": [0.0, arena_size],
                "max_height": 100.0,
            },
        },
        "uavs": uavs_data,
        "tasks": tasks,
    }


def compute_poi_metrics(tasks: list[dict[str, Any]]) -> dict[str, Any]:
    """Compute spatial and temporal metrics for the generated POIs."""
    n = len(tasks)
    min_dist = float("inf")
    all_dists: list[float] = []

    for i in range(n):
        for j in range(i + 1, n):
            p1 = tasks[i]["position"]
            p2 = tasks[j]["position"]
            d = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
            all_dists.append(d)
            if d < min_dist:
                min_dist = d

    xs = [t["position"][0] for t in tasks]
    ys = [t["position"][1] for t in tasks]
    ts = [t["spawn_time"] for t in tasks]

    return {
        "poi_count": n,
        "min_pairwise_distance": round(min_dist, 2) if all_dists else 0.0,
        "avg_pairwise_distance": round(sum(all_dists) / len(all_dists), 2) if all_dists else 0.0,
        "x_bounds": [min(xs), max(xs)],
        "y_bounds": [min(ys), max(ys)],
        "spawn_time_bounds": [min(ts), max(ts)],
    }


def generate_and_export_demo(
    seed: int = 2026,
    num_pois: int = 10,
    min_spacing: float = 40.0,
    margin: float = 30.0,
    spawn_start: float = 0.0,
    spawn_end: float = 300.0,
    full_arena: bool = False,
    custom_x_range: tuple[float, float] | None = None,
    custom_y_range: tuple[float, float] | None = None,
    output_scenario: str | Path | None = None,
    output_trace: str | Path | None = None,
    max_ticks: int | None = None,
    run_simulation: bool = True,
) -> dict[str, Any]:
    """Execute the full demo generation and export pipeline deterministically."""
    # Determine sampling spatial bounds
    if full_arena:
        x_range = (margin, 1000.0 - margin)
        y_range = (margin, 1000.0 - margin)
    else:
        x_range = custom_x_range
        y_range = custom_y_range

    # 1. Sample POIs
    tasks = sample_random_pois(
        seed=seed,
        num_pois=num_pois,
        min_spacing=min_spacing,
        margin=margin,
        x_range=x_range,
        y_range=y_range,
        spawn_window=(spawn_start, spawn_end),
    )

    # 2. Build scenario dictionary
    scen_dict = generate_demo_scenario_dict(
        seed=seed,
        tasks=tasks,
        min_separation=20.0,
    )

    # 3. Write scenario YAML
    scen_path = Path(output_scenario) if output_scenario else REPO_ROOT / "scenarios" / f"demo_random_seed_{seed}.yaml"
    scen_path.parent.mkdir(parents=True, exist_ok=True)
    with open(scen_path, "w", encoding="utf-8") as f:
        # Prepend DEMO warning header
        f.write("# ==============================================================================\n")
        f.write(f"# DEMO-ONLY RANDOMIZED POI SCENARIO (Seed: {seed})\n")
        f.write("# EXPERIMENTAL / DEMO USE ONLY. NOT AN AUTHORITATIVE COMPETITION BENCHMARK.\n")
        f.write("# Canonical frozen scenario remains: scenarios/poc_round1.yaml\n")
        f.write("# ==============================================================================\n")
        yaml.dump(scen_dict, f, sort_keys=False, indent=2)

    poi_metrics = compute_poi_metrics(tasks)

    trace_path = None
    if run_simulation:
        # 4. Authoritative Simulation & Trace Export
        target_trace = Path(output_trace) if output_trace else REPO_ROOT / "visualization" / "webots" / "data" / "random_demo_trace.json"
        target_trace.parent.mkdir(parents=True, exist_ok=True)

        trace_path = export_trace(
            scenario_path=scen_path,
            output_path=target_trace,
            experiment="demo",
            seed=seed,
            a1=True,
            max_ticks=max_ticks,
        )

    return {
        "seed": seed,
        "scenario_path": scen_path,
        "trace_path": trace_path,
        "tasks": tasks,
        "poi_metrics": poi_metrics,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Deterministic DEMO-ONLY randomized POI scenario and Webots trace generator for AetherSwarm."
    )
    parser.add_argument("--seed", type=int, default=2026, help="Deterministic random seed (default: 2026)")
    parser.add_argument("--num-pois", type=int, default=10, help="Number of POIs to generate (default: 10)")
    parser.add_argument("--min-spacing", type=float, default=40.0, help="Minimum POI-to-POI 2D Euclidean distance in meters (default: 40.0)")
    parser.add_argument("--margin", type=float, default=30.0, help="Minimum margin from operational arena boundaries in meters (default: 30.0)")
    parser.add_argument("--spawn-start", type=float, default=0.0, help="Spawn window start time in seconds (default: 0.0)")
    parser.add_argument("--spawn-end", type=float, default=300.0, help="Spawn window end time in seconds (default: 300.0)")
    parser.add_argument("--full-arena", action="store_true", help="Sample across the full 1000m x 1000m arena rather than the multi-hop demonstration corridor")
    parser.add_argument("--x-range", nargs=2, type=float, default=None, metavar=("X_MIN", "X_MAX"), help="Custom X sampling bounds")
    parser.add_argument("--y-range", nargs=2, type=float, default=None, metavar=("Y_MIN", "Y_MAX"), help="Custom Y sampling bounds")
    parser.add_argument("--output-scenario", "-o", type=str, default=None, help="Output YAML scenario path")
    parser.add_argument("--output-trace", "-t", type=str, default=None, help="Output JSON trace path (default: visualization/webots/data/random_demo_trace.json)")
    parser.add_argument("--max-ticks", type=int, default=None, help="Maximum simulation ticks to run (default: full duration 2700)")
    parser.add_argument("--no-run", action="store_true", help="Generate scenario YAML only without running simulation or exporting trace")

    args = parser.parse_args()

    print("=" * 80)
    print(f"AETHERSWARM DEMO-ONLY RANDOMIZED POI GENERATOR (Seed: {args.seed})")
    print("NOTE: DEMO / EXPERIMENTAL ONLY. Canonical E1 benchmark is untouched.")
    print("=" * 80)

    res = generate_and_export_demo(
        seed=args.seed,
        num_pois=args.num_pois,
        min_spacing=args.min_spacing,
        margin=args.margin,
        spawn_start=args.spawn_start,
        spawn_end=args.spawn_end,
        full_arena=args.full_arena,
        custom_x_range=tuple(args.x_range) if args.x_range else None,
        custom_y_range=tuple(args.y_range) if args.y_range else None,
        output_scenario=args.output_scenario,
        output_trace=args.output_trace,
        max_ticks=args.max_ticks,
        run_simulation=not args.no_run,
    )

    metrics = res["poi_metrics"]
    print(f"\nScenario Generated:         {res['scenario_path']}")
    print(f"POIs Generated:             {metrics['poi_count']}")
    print(f"Min Pairwise POI Spacing:   {metrics['min_pairwise_distance']}m (Configured minimum: {args.min_spacing}m)")
    print(f"Avg Pairwise POI Spacing:   {metrics['avg_pairwise_distance']}m")
    print(f"POI Spatial Bounds:         X={metrics['x_bounds']}, Y={metrics['y_bounds']}")
    print(f"POI Spawn Window:           {metrics['spawn_time_bounds'][0]}s - {metrics['spawn_time_bounds'][1]}s")
    print("\nPOI Positions & Spawn Times:")
    for t in res["tasks"]:
        print(f"  {t['id']}: pos=({t['position'][0]:6.2f}, {t['position'][1]:6.2f})  spawn={t['spawn_time']:5.1f}s  priority={t['priority']}")

    if res["trace_path"]:
        print(f"\nAuthoritative Trace:        {res['trace_path']}")
        print("\nWebots Visualization Instructions:")
        print("  1. Launch Webots world: visualization/webots/worlds/uavx_round1.wbt")
        print("  2. Replay trace via supervisor:")
        print("     export AETHERSWARM_SCENARIO=random")
        print("     # or pass path directly: python aetherswarm_supervisor.py visualization/webots/data/random_demo_trace.json")

    return 0


if __name__ == "__main__":
    sys.exit(main())
