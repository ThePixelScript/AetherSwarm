#!/usr/bin/env python3
"""Authoritative trace exporter for AetherSwarm Webots R2025a 3D robotics visualization.

Reuses existing AetherSwarm MissionRunner infrastructure without modifying core
production logic. Serializes the authoritative simulation state into an immutable
JSON trace file consumed by the Webots Supervisor controller.

Usage:
  .venv/bin/python scripts/export_webots_trace.py --scenario scenarios/poc_round1.yaml --experiment E1 --output visualization/webots/data/e1_authoritative_trace.json
  .venv/bin/python scripts/export_webots_trace.py --scenario scenarios/demo_in_flight_recovery.yaml --output visualization/webots/data/recovery_authoritative_trace.json
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
from pathlib import Path
from typing import Any

# Ensure src is on sys.path
repo_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(repo_root / "src"))

from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.autonomy.task_allocator import A0TaskAllocator
from ares_swarm.core.enums import FailureState, Role, RTHState, TaskStatus
from ares_swarm.core.event_scheduler import ScheduledEvent, ScheduledEventType
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import ScenarioConfig, load_scenario


def calculate_yaw(vx: float, vy: float, previous_yaw: float) -> float:
    """Compute heading angle in radians from planar velocity; retain previous heading when static."""
    speed = math.hypot(vx, vy)
    if speed > 0.05:
        return math.atan2(vy, vx)
    return previous_yaw


def validate_trace_data(trace: dict[str, Any]) -> None:
    """Validate completeness and numerical integrity of trace data before serialization."""
    required_meta = [
        "scenario_name",
        "arena_bounds_x",
        "arena_bounds_y",
        "max_altitude",
        "gcs_position",
        "tick_duration",
        "total_ticks",
        "uav_ids",
        "task_ids",
    ]
    for key in required_meta:
        if key not in trace["metadata"]:
            raise ValueError(f"Trace metadata missing required key: {key}")

    ticks = trace.get("ticks", [])
    if not ticks:
        raise ValueError("Trace contains zero simulation ticks")

    for step in ticks:
        for uav_id, u in step["uavs"].items():
            pos = u["position"]
            if len(pos) != 3 or any(not math.isfinite(coord) for coord in pos):
                raise ValueError(f"Invalid position for {uav_id} at tick {step['tick']}: {pos}")
            vel = u["velocity"]
            if len(vel) != 3 or any(not math.isfinite(v) for v in vel):
                raise ValueError(f"Invalid velocity for {uav_id} at tick {step['tick']}: {vel}")
            if not math.isfinite(u["yaw"]):
                raise ValueError(f"Invalid yaw for {uav_id} at tick {step['tick']}: {u['yaw']}")


def export_trace(
    scenario_path: str | Path,
    output_path: str | Path,
    experiment: str | None = None,
    seed: int = 42,
    a1: bool = True,
    max_ticks: int | None = None,
) -> Path:
    """Run authoritative AetherSwarm simulation and export Webots trace JSON."""
    scenario_file = Path(scenario_path)
    base_scenario = load_scenario(scenario_file)

    # Configure allocator
    allocator = A1TaskAllocator() if a1 else A0TaskAllocator()
    adapter = A0AutonomyAdapter(allocator=allocator)

    # Setup experiment specific modifiers
    scheduled_events: list[ScheduledEvent] = []
    comm_range = 100.0 if experiment in ("E0", "E1", "E3", "E4") else base_scenario.communication.max_range
    tasks = base_scenario.tasks
    uavs = base_scenario.uavs

    if experiment == "E1":
        # Official E1 relay failure at tick 300
        scheduled_events.append(
            ScheduledEvent(
                tick=300,
                event_type=ScheduledEventType.UAV_FAILURE,
                uav_id="uav_2",
                reason="Relay failure E1",
            )
        )

    # Construct scenario configuration
    scenario = dataclasses.replace(
        base_scenario,
        communication=dataclasses.replace(base_scenario.communication, max_range=comm_range),
        tasks=tasks,
        uavs=uavs,
    )

    runner = MissionRunner(
        scenario=scenario,
        seed=seed,
        autonomy_adapter=adapter,
    )

    for evt in scheduled_events:
        runner.sim_engine.event_scheduler.schedule(evt)

    # For dynamic in-flight recovery scenarios (e.g. demo_in_flight_recovery.yaml):
    # Dynamically detect when a task is IN_PROGRESS without assuming initial assignment
    is_recovery_demo = "recovery" in scenario_file.name.lower() or scenario.name == "demo_in_flight_recovery"
    recovery_failure_scheduled = False

    limit = max_ticks if max_ticks is not None else scenario.max_ticks
    step_history = []
    previous_yaw: dict[str, float] = {u["id"]: 0.0 for u in scenario.uavs}

    print(f"Running authoritative simulation: {scenario.name} (seed={seed}, max_ticks={limit})...")
    for tick_idx in range(limit):
        snap_before = runner.state_store.snapshot()

        if is_recovery_demo and not recovery_failure_scheduled:
            for tid, t in snap_before.tasks.items():
                if t.status == TaskStatus.IN_PROGRESS and t.assigned_uav_id:
                    fail_tick = tick_idx + 1
                    runner.sim_engine.event_scheduler.schedule(
                        ScheduledEvent(
                            tick=fail_tick,
                            event_type=ScheduledEventType.UAV_FAILURE,
                            uav_id=t.assigned_uav_id,
                            reason="In-flight hardware failure demo",
                        )
                    )
                    recovery_failure_scheduled = True
                    print(f"  [Recovery Demo] Detected task '{tid}' IN_PROGRESS by '{t.assigned_uav_id}' at tick {tick_idx}. Scheduled failure for tick {fail_tick}.")
                    break

        step_res = runner.step()
        step_history.append(step_res)

    print(f"Simulation completed. Total ticks: {len(step_history)}.")

    # Build Webots trace dataset
    uav_ids = [u["id"] for u in scenario.uavs]
    task_ids = [t["id"] for t in scenario.tasks]
    gcs_x, gcs_y = scenario.gcs_position

    metadata = {
        "scenario_name": scenario.name,
        "experiment": experiment or "baseline",
        "autonomy": "A1" if a1 else "A0",
        "seed": seed,
        "arena_bounds_x": list(scenario.arena_bounds_x),
        "arena_bounds_y": list(scenario.arena_bounds_y),
        "max_altitude": scenario.max_height,
        "gcs_position": [gcs_x, gcs_y, 0.0],
        "tick_duration": scenario.dt,
        "total_ticks": len(step_history),
        "min_separation_m": scenario.min_separation_m,
        "uav_ids": uav_ids,
        "task_ids": task_ids,
    }

    ticks_data = []
    all_events_data = []

    for step in step_history:
        sim_tick = step.tick
        sim_time = step.simulation_time
        snap = step.snapshot

        uavs_dict = {}
        for uid in uav_ids:
            u_state = snap.uavs.get(uid)
            if not u_state:
                continue

            ux, uy = u_state.position_xy
            vx, vy = u_state.velocity_xy

            # Compute metric 3D altitude (layer * 15m when flying, 0.5m when landed at GCS)
            is_landed = (u_state.rth_state == RTHState.COMPLETE) or (
                not u_state.active and u_state.failure_state != FailureState.FAILED and math.hypot(ux - gcs_x, uy - gcs_y) <= 1.0
            )
            uz = 0.5 if is_landed else max(10.0, float(u_state.altitude_layer) * 15.0)

            # Compute yaw from planar velocity
            cur_yaw = calculate_yaw(vx, vy, previous_yaw[uid])
            previous_yaw[uid] = cur_yaw

            uavs_dict[uid] = {
                "id": uid,
                "position": [round(ux, 3), round(uy, 3), round(uz, 3)],
                "velocity": [round(vx, 3), round(vy, 3), 0.0],
                "yaw": round(cur_yaw, 4),
                "active": u_state.active,
                "failure_state": u_state.failure_state.value if hasattr(u_state.failure_state, "value") else str(u_state.failure_state),
                "battery_percent": round(u_state.battery_percent, 2),
                "battery_energy": round(u_state.battery_energy, 2),
                "assigned_task_id": u_state.assigned_task_id,
                "rth_state": u_state.rth_state.value if hasattr(u_state.rth_state, "value") else str(u_state.rth_state),
                "role": u_state.role.value if hasattr(u_state.role, "value") else str(u_state.role),
            }

        tasks_dict = {}
        for tid in task_ids:
            t_state = snap.tasks.get(tid)
            if not t_state:
                continue
            tx, ty = t_state.position_xy
            tasks_dict[tid] = {
                "id": tid,
                "position": [round(tx, 3), round(ty, 3), 0.0],
                "priority": t_state.priority,
                "status": t_state.status.value if hasattr(t_state.status, "value") else str(t_state.status),
                "assigned_uav_id": t_state.assigned_uav_id,
                "service_progress": round(t_state.service_progress, 2),
                "service_duration": round(t_state.service_duration, 2),
            }

        # Network topology derived authoritative links
        active_links = []
        for link in step.network_analysis.network.links:
            if link.active:
                active_links.append({
                    "source": link.source_id,
                    "target": link.target_id,
                    "distance": round(link.distance, 2),
                    "pdr": round(link.estimated_pdr, 4),
                })

        step_events = [
            {
                "tick": sim_tick,
                "time": sim_time,
                "type": ev.event_type.value if hasattr(ev.event_type, "value") else str(ev.event_type),
                "entity_id": ev.entity_id,
                "payload": ev.payload,
            }
            for ev in step.events
        ]
        all_events_data.extend(step_events)

        ticks_data.append({
            "tick": sim_tick,
            "time": sim_time,
            "uavs": uavs_dict,
            "tasks": tasks_dict,
            "network": {
                "active_links": active_links,
                "connected_uav_ids": list(step.network_analysis.connected_uav_ids),
                "hop_counts": dict(step.network_analysis.hop_counts),
                "routes_to_gcs": {
                    uid: list(route) if route else None
                    for uid, route in step.network_analysis.routes_to_gcs.items()
                },
            },
            "events": step_events,
        })

    trace_data = {
        "metadata": metadata,
        "events": all_events_data,
        "ticks": ticks_data,
    }

    validate_trace_data(trace_data)

    out_file = Path(output_path)
    out_file.parent.mkdir(parents=True, exist_ok=True)
    with open(out_file, "w", encoding="utf-8") as f:
        json.dump(trace_data, f, indent=2)

    file_size_mb = out_file.stat().st_size / (1024 * 1024)
    print(f"Exported trace successfully: {out_file} ({file_size_mb:.2f} MB, {len(ticks_data)} ticks)")
    return out_file


def main() -> int:
    parser = argparse.ArgumentParser(description="Export authoritative AetherSwarm simulation trace for Webots.")
    parser.add_argument("--scenario", "-s", type=str, default="scenarios/poc_round1.yaml", help="Path to scenario YAML")
    parser.add_argument("--output", "-o", type=str, default="visualization/webots/data/e1_authoritative_trace.json", help="Output trace JSON path")
    parser.add_argument("--experiment", "-e", type=str, default="E1", help="Experiment name (e.g. E0, E1, E4)")
    parser.add_argument("--seed", type=int, default=42, help="Deterministic random seed")
    parser.add_argument("--no-a1", action="store_true", help="Use A0 instead of A1")
    parser.add_argument("--max-ticks", type=int, default=None, help="Override maximum ticks")

    args = parser.parse_args()

    export_trace(
        scenario_path=args.scenario,
        output_path=args.output,
        experiment=args.experiment,
        seed=args.seed,
        a1=not args.no_a1,
        max_ticks=args.max_ticks,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
