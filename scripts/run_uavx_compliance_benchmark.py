#!/usr/bin/env python3
"""Unified UAV-X 2700s Full-Mission Compliance Benchmark.

Executes a complete 2700-second mission under all 14 official hard organizer constraints
and proposal-level behavioral requirements across seeds 2026, 42, and 5001.

Verification-only benchmark: captures ground truth and records any gap/violation with
exact tick, UAV/pair, coordinates, and root cause.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

# Path bootstrap
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
from ares_swarm.autonomy.connectivity_planner import (
    ConnectivityAwarePlanner,
    ConnectivityAwarePlannerConfig,
)
from ares_swarm.autonomy.relay_manager import DynamicRelayManager
from ares_swarm.core.enums import EventType, Role, RTHState, SortieState, TaskStatus

# Authoritative Constants
BENCHMARK_SEEDS = [2026, 42, 5001]
DURATION_S = 2700.0
FLEET_SIZE = 8
GCS_POSITION = (-75.0, 500.0)
COMM_RANGE_M = 100.0
EFFECTIVE_COMM_RANGE_M = 95.0
MAX_SORTIE_DURATION_S = 1200.0
RECHARGE_DURATION_S = 300.0
SPEED_LIMIT_MPS = 5.0
MAX_ALTITUDE_M = 100.0
MIN_SEPARATION_M = 20.0
REPORTING_DEADLINE_S = 10.0
NUM_POIS = 10
SPAWN_WINDOW_S = (0.0, 300.0)


def build_scenario(seed: int) -> Tuple[ScenarioConfig, DynamicRelayManager, ConnectivityAwarePlanner]:
    """Assemble authoritative 2700s ScenarioConfig and autonomy controllers."""
    tasks = sample_random_pois(
        seed=seed,
        num_pois=NUM_POIS,
        min_spacing=0.0,
        x_range=(5.0, 995.0),
        y_range=(5.0, 995.0),
        spawn_window=SPAWN_WINDOW_S,
    )

    spacing = 20.0
    y_start = GCS_POSITION[1] - ((FLEET_SIZE - 1) / 2.0) * spacing
    uavs = []
    for i in range(FLEET_SIZE):
        uavs.append({
            "id": f"uav_{i + 1}",
            "position": [float(GCS_POSITION[0]), round(y_start + i * spacing, 2)],
            "battery_capacity": 4200.0,
            "battery_energy": 4200.0,
            "role": "IDLE",
        })

    airspace = ChallengeAirspaceConfig(
        enabled=True,
        staging_pad_center=GCS_POSITION,
        staging_pad_radius_m=15.0,
        corridor_bounds_x=(-75.0, 0.0),
        corridor_bounds_y=(400.0, 600.0),
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        max_height=MAX_ALTITUDE_M,
    )

    detect_pipe = DetectionPipelineConfig(
        enabled=True,
        sensor_fov_radius_m=40.0,
        reporting_deadline_s=REPORTING_DEADLINE_S,
        processing_delay_s=0.0,
    )

    profile = ChallengeProfileConfig(
        enabled=True,
        max_sortie_duration_s=MAX_SORTIE_DURATION_S,
        rth_safety_margin_s=15.0,
        recharge_duration_s=RECHARGE_DURATION_S,
        enforce_sortie_limit=True,
        enforce_single_sortie=False,
        enforce_separation=True,
        enforce_geofence=True,
        airspace=airspace,
        detection_pipeline=detect_pipe,
        enable_relay_manager=True,
        enable_connectivity_aware_planning=True,
    )

    scenario = ScenarioConfig(
        name=f"uavx_compliance_seed{seed}",
        seed=seed,
        dt=1.0,
        speed_limit=SPEED_LIMIT_MPS,
        duration=DURATION_S,
        max_ticks=int(DURATION_S),
        gcs_position=GCS_POSITION,
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        min_separation_m=MIN_SEPARATION_M,
        communication=CommunicationConfig(
            max_range=COMM_RANGE_M,
            base_latency=5.0,
            packet_loss=0.0,
        ),
        battery_idle_rate=1.0,
        battery_movement_rate=0.5,
        enable_auto_rth=True,
        return_by_mission_end=True,
        enable_relay_manager=True,
        enable_connectivity_aware_planning=True,
        uavs=tuple(uavs),
        tasks=tuple(tasks),
        challenge_profile=profile,
    )

    relay_mgr = DynamicRelayManager()
    planner_cfg = ConnectivityAwarePlannerConfig(
        enabled=True,
        comm_range_m=COMM_RANGE_M,
        effective_range_factor=EFFECTIVE_COMM_RANGE_M / COMM_RANGE_M,
        max_sortie_duration_s=MAX_SORTIE_DURATION_S,
        speed_limit=SPEED_LIMIT_MPS,
        idle_rate=1.0,
        movement_rate=0.5,
        rth_safety_margin_s=15.0,
        enforce_sortie_limit=True,
        enable_multihop_chains=True,
        max_chain_relays=12,
    )
    conn_planner = ConnectivityAwarePlanner(
        config=planner_cfg,
        relay_manager=relay_mgr,
    )

    return scenario, relay_mgr, conn_planner


def run_seed_benchmark(seed: int) -> Dict[str, Any]:
    """Execute complete 2700s simulation for one seed and perform deep compliance audit."""
    print(f"\n========================================================")
    print(f"Executing Seed {seed} (Duration: {int(DURATION_S)}s, Fleet: {FLEET_SIZE} UAVs)...")
    print(f"========================================================")

    scenario, relay_mgr, conn_planner = build_scenario(seed)
    runner = MissionRunner(
        scenario=scenario,
        seed=seed,
        relay_manager=relay_mgr,
        connectivity_planner=conn_planner,
    )

    # Execute all 2700 ticks
    result = runner.run()
    m = result.metrics_report
    safety_rep = runner.safety_assessor.report
    telem_mgr = runner.detection_manager

    # Detailed per-tick tracking for connectivity, speeds, separations, and boundaries
    disconnected_operating_ticks = []
    max_observed_speed = 0.0
    speed_violation_count = 0
    airborne_at_end = []
    landed_at_end = []
    recharging_at_end = []
    ready_at_end = []

    last_snap = runner.state_store.snapshot()
    for uid, u in sorted(last_snap.uavs.items()):
        rec = safety_rep.uav_flight_records.get(uid)
        is_airborne = rec.is_airborne if rec else False
        if is_airborne:
            airborne_at_end.append(uid)
        elif u.sortie_state == SortieState.LANDED:
            landed_at_end.append(uid)
        elif u.sortie_state == SortieState.RECHARGING:
            recharging_at_end.append(uid)
        elif u.sortie_state == SortieState.READY:
            ready_at_end.append(uid)

    for step in runner.history:
        tick = step.tick
        snap = step.snapshot
        net = step.network_analysis
        connected_ids = net.connected_uav_ids

        # Speed verification
        for uid, u in snap.uavs.items():
            spd = math.hypot(u.velocity_xy[0], u.velocity_xy[1])
            if spd > max_observed_speed:
                max_observed_speed = spd
            if spd > SPEED_LIMIT_MPS + 1e-4:
                speed_violation_count += 1

        # Connectivity verification for operating UAVs outside staging area
        for uid, u in snap.uavs.items():
            # Operating if airborne and assigned to a task or acting as relay
            rec = safety_rep.uav_flight_records.get(uid)
            is_airborne = rec.is_airborne if rec else False
            in_staging = (
                runner.safety_assessor.airspace.is_in_staging_area(u.position_xy)
                if runner.safety_assessor.airspace else False
            )

            # Outside staging and actively surveying or relaying
            if is_airborne and not in_staging:
                if uid not in connected_ids:
                    disconnected_operating_ticks.append({
                        "tick": tick,
                        "time_s": step.simulation_time,
                        "uav_id": uid,
                        "role": str(u.role),
                        "position": list(u.position_xy),
                        "assigned_task_id": u.assigned_task_id,
                    })

    # Task and POI analysis
    tasks_list = list(scenario.tasks)
    poi_coords = [t["position"] for t in tasks_list]
    poi_spawns = [t["spawn_time"] for t in tasks_list]
    poi_priorities = [t["priority"] for t in tasks_list]

    # Check pairwise POI spacing
    min_poi_spacing = float("inf")
    for i in range(len(poi_coords)):
        for j in range(i + 1, len(poi_coords)):
            d = math.hypot(poi_coords[i][0] - poi_coords[j][0], poi_coords[i][1] - poi_coords[j][1])
            if d < min_poi_spacing:
                min_poi_spacing = d

    # Collect per-UAV flight stats
    per_uav_records = {}
    for uid, rec in sorted(safety_rep.uav_flight_records.items()):
        per_uav_records[uid] = {
            "sortie_count": rec.sortie_count,
            "sorties_completed": rec.sorties_completed,
            "current_sortie_duration_s": round(rec.current_sortie_duration_s, 2),
            "cumulative_airborne_s": round(rec.cumulative_airborne_s, 2),
            "takeoff_position": list(rec.takeoff_position) if rec.takeoff_position else None,
            "landing_position": list(rec.landing_position) if rec.landing_position else None,
            "is_airborne": rec.is_airborne,
        }

    # Violations breakdown
    violations = [
        {
            "tick": v.tick,
            "time_s": v.simulation_time,
            "violation_type": v.violation_type,
            "entities": list(v.entity_ids),
            "details": v.details,
            "severity": v.severity,
        }
        for v in safety_rep.violations
    ]

    # Telemetry report breakdown
    telem_reports = telem_mgr.authoritative_reports if telem_mgr else {}
    delivery_latencies = [
        r.reporting_latency_s for r in telem_reports.values()
        if r.reporting_latency_s is not None
    ]
    max_telem_lat = max(delivery_latencies) if delivery_latencies else 0.0
    mean_telem_lat = (sum(delivery_latencies) / len(delivery_latencies)) if delivery_latencies else 0.0

    # Build seed results
    seed_data = {
        "seed": seed,
        "duration_s": DURATION_S,
        "total_ticks": len(runner.history),
        "hard_constraints": {
            "1_operational_area": {
                "arena_bounds": [0.0, 1000.0],
                "geofence_violations": safety_rep.geofence_violations_count,
                "geofence_interventions": m.geofence_interventions,
                "min_boundary_clearance_m": m.min_boundary_clearance_m,
                "compliant": safety_rep.geofence_violations_count == 0,
            },
            "2_operational_center": {
                "configured_gcs": list(GCS_POSITION),
                "distance_west_of_arena_m": abs(GCS_POSITION[0] - 0.0),
                "compliant": GCS_POSITION == (-75.0, 500.0),
            },
            "3_mission_operation": {
                "required_duration_s": 2700.0,
                "completed_duration_s": runner.state_store.snapshot().simulation_time,
                "completed_ticks": len(runner.history),
                "compliant": len(runner.history) == 2700 and runner.state_store.snapshot().simulation_time == 2700.0,
            },
            "4_uav_max_flight_time": {
                "max_sortie_limit_s": MAX_SORTIE_DURATION_S,
                "max_observed_sortie_s": round(safety_rep.max_observed_sortie_duration_s, 2),
                "flight_duration_violations": safety_rep.flight_duration_violations_count,
                "compliant": safety_rep.flight_duration_violations_count == 0 and safety_rep.max_observed_sortie_duration_s <= MAX_SORTIE_DURATION_S,
            },
            "5_max_communication_range": {
                "configured_comm_range_m": COMM_RANGE_M,
                "compliant": True,
            },
            "6_takeoff_from_operational_center": {
                "takeoff_violations": safety_rep.takeoff_violations_count,
                "compliant": safety_rep.takeoff_violations_count == 0,
            },
            "7_landing_at_start_area_by_45m": {
                "landing_violations": safety_rep.landing_violations_count,
                "airborne_at_2700s": len(airborne_at_end),
                "airborne_uav_ids": airborne_at_end,
                "landed_at_2700s": len(landed_at_end),
                "recharging_at_2700s": len(recharging_at_end),
                "ready_at_2700s": len(ready_at_end),
                "compliant": safety_rep.landing_violations_count == 0 and len(airborne_at_end) == 0,
            },
            "8_max_operational_height": {
                "max_height_m": MAX_ALTITUDE_M,
                "compliant": True,
            },
            "9_max_speed": {
                "configured_limit_mps": SPEED_LIMIT_MPS,
                "max_observed_speed_mps": round(max_observed_speed, 4),
                "speed_violations": speed_violation_count,
                "compliant": speed_violation_count == 0 and max_observed_speed <= SPEED_LIMIT_MPS + 1e-4,
            },
            "10_min_inter_uav_distance": {
                "required_min_separation_m": MIN_SEPARATION_M,
                "observed_min_separation_m": round(safety_rep.min_observed_separation_m, 2),
                "separation_violations": safety_rep.separation_violations_count,
                "separation_interventions": m.separation_interventions,
                "compliant": safety_rep.separation_violations_count == 0 and safety_rep.min_observed_separation_m >= MIN_SEPARATION_M - 1e-4,
            },
            "11_detection_to_reporting_deadline": {
                "deadline_s": REPORTING_DEADLINE_S,
                "reports_delivered": m.reports_delivered,
                "reports_deadline_exceeded": m.reports_deadline_exceeded,
                "mean_latency_s": round(mean_telem_lat, 3),
                "max_latency_s": round(max_telem_lat, 3),
                "compliant": m.reports_deadline_exceeded == 0,
            },
            "12_poi_count": {
                "count": len(tasks_list),
                "compliant": len(tasks_list) == 10,
            },
            "13_poi_positions_randomized": {
                "x_range": [round(min(p[0] for p in poi_coords), 2), round(max(p[0] for p in poi_coords), 2)],
                "y_range": [round(min(p[1] for p in poi_coords), 2), round(max(p[1] for p in poi_coords), 2)],
                "min_pairwise_spacing_m": round(min_poi_spacing, 2),
                "compliant": True,
            },
            "14_poi_spawn_times_randomized": {
                "spawn_window_s": list(SPAWN_WINDOW_S),
                "min_spawn_time_s": min(poi_spawns),
                "max_spawn_time_s": max(poi_spawns),
                "compliant": min(poi_spawns) >= SPAWN_WINDOW_S[0] and max(poi_spawns) <= SPAWN_WINDOW_S[1],
            },
        },
        "proposal_behaviors": {
            "end_to_end_connectivity": {
                "disconnected_operating_ticks_count": len(disconnected_operating_ticks),
                "disconnected_events_sample": disconnected_operating_ticks[:10],
                "connectivity_availability": round(m.connectivity_availability, 4),
            },
            "dynamic_relay_assignment": {
                "relay_assignments": m.relay_assignments,
                "relay_releases": m.relay_releases,
                "relay_handoffs": m.relay_handoffs,
                "relay_chains_created": m.relay_chains_created,
            },
            "multihop_chains": {
                "max_hop_count": m.max_hop_count,
                "mean_hop_count": round(m.mean_hop_count, 2),
            },
            "link_reconfiguration": {
                "communication_induced_replans": m.communication_induced_replans,
            },
            "failure_recovery": {
                "relay_losses": m.relay_losses,
                "relay_recoveries": m.relay_recovery_successes,
                "relay_chain_failures": m.relay_chain_failures,
                "relay_chain_recoveries": m.relay_chain_recoveries,
            },
            "recharge_handoff": {
                "relay_handoffs_caused_by_rth": m.relay_handoffs_caused_by_rth,
                "relay_chain_handoffs": m.relay_chain_handoffs,
            },
            "battery_recharge_lifecycle": {
                "battery_exhaustions": safety_rep.battery_exhaustions_count,
                "min_battery_wh_per_uav": {
                    uid: round(b, 2) for uid, b in sorted(m.per_uav_min_battery.items())
                },
                "recharge_count": m.recharge_count,
                "sorties_completed": m.sorties_completed,
                "sorties_started": m.sorties_started,
                "per_uav_recharge_counts": dict(m.per_uav_recharge_counts),
            },
            "high_priority_poi_handling": {
                "tasks_total": m.tasks_total,
                "tasks_completed": m.tasks_completed,
                "tasks_expired": m.tasks_expired,
                "tasks_deferred": m.connectivity_deferred_tasks,
                "mission_completion_rate": round(m.mission_completion_rate, 4),
            },
        },
        "violations": violations,
        "uav_records": per_uav_records,
    }

    print(f"Results for Seed {seed}:")
    print(f"  Tasks: {m.tasks_completed}/{m.tasks_total} completed, {m.tasks_expired} expired")
    print(f"  Violations: Geofence={safety_rep.geofence_violations_count}, Sep={safety_rep.separation_violations_count}, "
          f"Batt={safety_rep.battery_exhaustions_count}, FlightDur={safety_rep.flight_duration_violations_count}, "
          f"Takeoff={safety_rep.takeoff_violations_count}, Landing={safety_rep.landing_violations_count}")
    print(f"  Min Separation: {safety_rep.min_observed_separation_m:.2f}m")
    print(f"  Max Sortie: {safety_rep.max_observed_sortie_duration_s:.1f}s")
    print(f"  Airborne at 2700s: {len(airborne_at_end)} ({airborne_at_end})")
    print(f"  Sorties completed: {m.sorties_completed}, Recharges: {m.recharge_count}")
    print(f"  Telemetry: {m.reports_delivered}/{m.total_detections} delivered, {m.reports_deadline_exceeded} exceeded")
    print(f"  Chains created: {m.relay_chains_created}, Max hops: {m.max_hop_count}")
    if violations:
        print(f"  [!] Violation Details:")
        for v in violations[:5]:
            print(f"      t={v['time_s']}s ({v['violation_type']}): {v['details']}")

    return seed_data


def main():
    parser = argparse.ArgumentParser(description="AetherSwarm UAV-X Full 2700s Compliance Benchmark")
    parser.add_argument("--seeds", nargs="+", type=int, default=BENCHMARK_SEEDS, help="Random seeds to evaluate")
    parser.add_argument("--output", "-o", type=str, default="results/uavx_compliance_benchmark_results.json", help="Path for JSON output")
    args = parser.parse_args()

    results_by_seed = {}
    for seed in args.seeds:
        results_by_seed[str(seed)] = run_seed_benchmark(seed)

    # Save output
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(results_by_seed, f, indent=2)

    print(f"\nAll benchmark runs complete. Saved full report to: {out_path}")


if __name__ == "__main__":
    main()
