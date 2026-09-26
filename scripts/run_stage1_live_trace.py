#!/usr/bin/env python3
"""Live 5-Second Autonomous Swarm State Diagnostic Runner for AetherSwarm UAV-X.

Runs the Stage-1 deterministic scenario, captures live authoritative state snapshots
at configurable intervals (default 5s), immediately logs all domain events in real time,
and outputs a clean diagnostic trace to console and log file.
"""
from __future__ import annotations

import argparse
import dataclasses
import math
from pathlib import Path
import sys
from typing import Any, Dict, List, Optional, Set

repo_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(repo_root / "src"))

from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.autonomy.ingress_coordinator import IngressCoordinator
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.core.enums import EventType, FailureState, Role, RTHState, SortieState, TaskStatus
from ares_swarm.core.models import StateSnapshot, UAVState
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import load_scenario


def format_event(ev: Any) -> str:
    """Format a domain event cleanly with timestamp and relevant payload."""
    t_str = f"[{ev.simulation_time:6.2f}s]"
    ev_type = ev.event_type.value if hasattr(ev.event_type, "value") else str(ev.event_type)
    payload = ev.payload or {}
    entity = ev.entity_id or ""

    if ev_type == EventType.POI_DETECTED.value:
        return f"{t_str} POI_DETECTED {entity} -> {payload.get('task_id')} (pos={payload.get('position')}, dist={payload.get('distance')}m)"
    elif ev_type == EventType.TELEMETRY_DELIVERED.value:
        return f"{t_str} TELEMETRY_DELIVERED {payload.get('task_id')} by {payload.get('detecting_uav_id')} via {payload.get('route')} latency={payload.get('latency_s', 0):.4f}s (hops={payload.get('hop_count')})"
    elif ev_type == EventType.TASK_ASSIGNED.value:
        return f"{t_str} TASK_ASSIGNED {entity} -> {payload.get('task_id')}"
    elif ev_type == EventType.TASK_COMPLETED.value:
        return f"{t_str} TASK_COMPLETED {payload.get('uav_id')} -> {entity}"
    elif ev_type == EventType.TASK_DEFERRED.value:
        return f"{t_str} TASK_DEFERRED {entity} (uav={payload.get('uav_id')}, reason={payload.get('reason')})"
    elif ev_type == EventType.RELAY_ASSIGNED.value:
        return f"{t_str} RELAY_ASSIGNED {entity} (for={payload.get('relay_for_uav_id')}, pos={payload.get('target_position')})"
    elif ev_type == EventType.RELAY_RELEASED.value:
        return f"{t_str} RELAY_RELEASED {entity} (prev={payload.get('previous_role')}, new={payload.get('new_role')}, reason={payload.get('reason')})"
    elif ev_type == EventType.RELAY_HANDOFF.value:
        return f"{t_str} RELAY_HANDOFF old={payload.get('old_relay_id')} -> new={payload.get('new_relay_id')} (for={payload.get('relay_for_uav_id')})"
    elif ev_type == EventType.RTH_TRIGGERED.value:
        return f"{t_str} RTH_STARTED {entity}"
    elif ev_type == EventType.UAV_LANDED.value:
        return f"{t_str} LANDED {entity} (final_energy={payload.get('final_energy', 0):.1f} Wh)"
    elif ev_type == EventType.UAV_RECHARGING.value:
        return f"{t_str} RECHARGE_STARTED {entity} (duration={payload.get('recharge_duration_s')}s)"
    elif ev_type == EventType.UAV_RECHARGED.value:
        return f"{t_str} RECHARGE_COMPLETE {entity}"
    elif ev_type == EventType.UAV_FAILED.value:
        return f"{t_str} UAV_FAILED {entity} (reason={payload.get('reason')})"
    elif ev_type in (EventType.DEPARTURE_STARTED.value, EventType.SORTIE_STARTED.value):
        return f"{t_str} TAKEOFF / DEPARTURE_STARTED {entity}"
    elif ev_type == EventType.DEPARTURE_CLEARED.value:
        return f"{t_str} DEPARTURE_CLEARED {entity} (staging->corridor cleared)"
    else:
        return f"{t_str} {ev_type} entity={entity} {payload}"


def main():
    parser = argparse.ArgumentParser(description="Live 5-Second Swarm State Diagnostic Runner")
    parser.add_argument("--scenario", type=str, default="scenarios/poc_round1.yaml")
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--interval", type=float, default=5.0)
    parser.add_argument("--logfile", type=str, default="logs/stage1_live_state_trace.txt")
    args = parser.parse_args()

    log_path = Path(args.logfile)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = open(log_path, "w", encoding="utf-8")

    def emit(line: str):
        print(line)
        log_file.write(line + "\n")
        log_file.flush()

    emit("=" * 100)
    emit("AETHERSWARM LIVE AUTONOMOUS SWARM STATE TRACE")
    emit(f"Scenario: {args.scenario} | Seed: {args.seed} | Snapshot Interval: {args.interval}s")
    emit(f"Log output: {log_path.resolve()}")
    emit("=" * 100)

    # 1. Load Scenario & Setup Stage-1 Compliance Profile
    base_scenario = load_scenario(Path(args.scenario))
    dp = dataclasses.replace(base_scenario.challenge_profile.detection_pipeline, enabled=True)
    airspace = dataclasses.replace(base_scenario.challenge_profile.airspace, enabled=True)
    cp = dataclasses.replace(
        base_scenario.challenge_profile,
        enabled=True,
        enforce_separation=True,
        enforce_sortie_limit=True,
        enforce_single_sortie=True,
        enforce_geofence=True,
        detection_pipeline=dp,
        airspace=airspace,
    )
    base_scenario = dataclasses.replace(base_scenario, challenge_profile=cp)

    # 2. Autonomy & Runner
    comm_analyzer = BaselineCommunicationAnalyzer(config=base_scenario.communication)
    allocator = A1TaskAllocator(comm_analyzer=comm_analyzer)
    ingress_coordinator = IngressCoordinator(
        comm_analyzer=comm_analyzer,
        gcs_position=base_scenario.gcs_position,
        d_safe=85.0,
    )
    adapter = A0AutonomyAdapter(allocator=allocator, ingress_coordinator=ingress_coordinator)

    runner = MissionRunner(
        scenario=base_scenario,
        seed=args.seed,
        autonomy_adapter=adapter,
        comm_analyzer=comm_analyzer,
    )

    limit = base_scenario.max_ticks
    last_snapshot_time = -1.0
    gcs = base_scenario.gcs_position

    seen_event_ids: Set[str] = set()

    for tick in range(limit):
        step_res = runner.step()
        sim_time = step_res.simulation_time
        snap = step_res.snapshot
        net = step_res.network_analysis

        # 1. Print any newly emitted domain events immediately
        for ev in step_res.events:
            ev_id = ev.event_id
            if ev_id not in seen_event_ids:
                seen_event_ids.add(ev_id)
                # Filter out pure physics step ticker events for high clarity
                if ev.event_type not in (EventType.SIMULATION_STEP, EventType.TASK_PROGRESS):
                    emit(format_event(ev))

        # 2. Print periodic full state snapshot
        if sim_time >= last_snapshot_time + args.interval - 1e-6 or tick == 0 or tick == limit - 1:
            last_snapshot_time = sim_time
            connected_ids = set(net.connected_uav_ids) if net else set()
            routes = net.routes_to_gcs if net else {}
            hops = net.hop_counts if net else {}

            airborne_count = 0
            landed_count = 0
            ready_count = 0

            for u in snap.uavs.values():
                rec = runner.safety_assessor.report.uav_flight_records.get(u.id)
                is_airborne = bool(rec.is_airborne) if rec else (u.active and u.rth_state != RTHState.COMPLETE)
                if is_airborne:
                    airborne_count += 1
                elif u.sortie_state in (SortieState.LANDED, SortieState.RECHARGING):
                    landed_count += 1
                elif u.sortie_state == SortieState.READY:
                    ready_count += 1

            tasks_completed = sum(1 for t in snap.tasks.values() if t.status == TaskStatus.COMPLETE)
            tasks_pending = sum(1 for t in snap.tasks.values() if t.status in (TaskStatus.PENDING, TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS))
            tasks_deferred = sum(1 for t in snap.tasks.values() if t.status in (TaskStatus.DEFERRED, TaskStatus.UNREACHABLE))
            pois_detected = len(runner.detection_manager.detected_task_ids) if runner.detection_manager else 0

            active_links = []
            if net and net.network and net.network.links:
                for l in net.network.links:
                    if l.active:
                        active_links.append(f"{l.source_id}<->{l.target_id}")

            emit("-" * 100)
            emit(
                f"[SNAPSHOT @ {sim_time:6.1f}s | Tick {tick:4d}] "
                f"GCS=({gcs[0]:.1f}, {gcs[1]:.1f}) | "
                f"Airborne={airborne_count} Landed={landed_count} Ready={ready_count} | "
                f"Connected={len(connected_ids)} Disconnected={len(snap.uavs) - len(connected_ids)} | "
                f"POIs: Comp={tasks_completed} Det={pois_detected} Pend={tasks_pending} Def={tasks_deferred}"
            )
            if active_links:
                emit(f"  Active RF Links: {', '.join(active_links)}")
            else:
                emit("  Active RF Links: NONE")

            for u_id in sorted(snap.uavs.keys()):
                u = snap.uavs[u_id]
                x, y = u.position_xy
                z = 15.0 if (u.active and u.rth_state != RTHState.COMPLETE) else 0.0
                vx, vy = u.velocity_xy
                speed = math.hypot(vx, vy)
                is_conn = "YES" if u.id in connected_ids else "NO"
                h_cnt = hops.get(u.id, 0)
                tgt_str = f"({u.target_position[0]:.1f},{u.target_position[1]:.1f})" if u.target_position else "None"
                relay_tgt = f" (relay_for={u.relay_target_id})" if u.role == Role.RELAY and u.relay_target_id else ""
                task_str = u.assigned_task_id or "None"

                emit(
                    f"  {u.id:5s} | Pos=({x:6.1f}, {y:6.1f}, {z:4.1f}) | Speed={speed:4.1f}m/s | "
                    f"Bat={u.battery_percent:5.1f}% | Role={u.role.value:8s}{relay_tgt} | "
                    f"Sortie={u.sortie_state.value:8s} | RTH={u.rth_state.value:8s} | Active={str(u.active):5s} | "
                    f"Task={task_str:6s} | Tgt={tgt_str:14s} | Conn={is_conn:3s} (Hops={h_cnt})"
                )
            emit("-" * 100)

        if base_scenario.return_by_mission_end:
            if all(not u.active and u.rth_state == RTHState.COMPLETE for u in snap.uavs.values() if u.failure_state != FailureState.FAILED):
                emit(f"[{sim_time:6.2f}s] ALL UAVS SAFELY LANDED AT GCS — MISSION COMPLETE")
                break

    log_file.close()
    print(f"\nTrace complete. Full logs written to: {log_path.resolve()}")


if __name__ == "__main__":
    main()
