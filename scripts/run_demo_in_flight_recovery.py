#!/usr/bin/env python3
"""Deterministic demo-only scenario runner proving in-flight UAV failure and dynamic A1 recovery.

Demonstrates the complete lifecycle:
  active task
  -> UAV failure
  -> TASK_DEFERRED
  -> A1 reassigns the SAME task
  -> replacement UAV completes it
  -> RTH / landing

Usage:
  .venv/bin/python scripts/run_demo_in_flight_recovery.py [--seed SEED] [--output-dir DIR]
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, is_dataclass
from pathlib import Path

# Add src to sys.path
repo_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(repo_root / "src"))

from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.core.enums import FailureState, RTHState, TaskStatus
from ares_swarm.core.event_scheduler import ScheduledEvent, ScheduledEventType
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import load_scenario
from ares_swarm.visualization.replay import ReplayRecorder


def run_in_flight_recovery_demo(seed: int = 42, output_dir: Path | str | None = None) -> dict:
    scenario_path = repo_root / "scenarios" / "demo_in_flight_recovery.yaml"
    scenario = load_scenario(scenario_path)

    out_dir = Path(output_dir) if output_dir else repo_root / "results" / "demo"
    out_dir.mkdir(parents=True, exist_ok=True)

    # Track intermediate DEFERRED state right after scheduled event application
    hook_captured_deferred: list[dict] = []

    def safety_hook(snapshot, net_analysis):
        for tid, t in snapshot.tasks.items():
            if t.status == TaskStatus.DEFERRED:
                hook_captured_deferred.append({
                    "tick": snapshot.simulation_tick,
                    "time": snapshot.simulation_time,
                    "task_id": tid,
                    "status": t.status.value,
                    "assigned_uav_id": t.assigned_uav_id,
                })

    allocator = A1TaskAllocator()
    adapter = A0AutonomyAdapter(allocator=allocator)
    runner = MissionRunner(
        scenario=scenario,
        seed=seed,
        autonomy_adapter=adapter,
        safety_hook=safety_hook,
    )

    recorder = ReplayRecorder()
    timeline_entries: list[str] = []

    selected_task: str | None = None
    original_uav: str | None = None
    task_in_progress_tick: int | None = None
    failure_tick: int | None = None
    replacement_uav: str | None = None
    reassignment_tick: int | None = None
    completion_tick: int | None = None
    deferred_verified = False

    # Execute simulation loop tick-by-tick
    for tick in range(scenario.max_ticks):
        snap_before = runner.state_store.snapshot()

        # Step 2 & 3: Runtime dynamic detection of in-progress task (never pre-assumed)
        if failure_tick is None:
            for tid, t in snap_before.tasks.items():
                if t.status == TaskStatus.IN_PROGRESS and t.assigned_uav_id:
                    selected_task = tid
                    original_uav = t.assigned_uav_id
                    task_in_progress_tick = snap_before.simulation_tick
                    failure_tick = task_in_progress_tick + 1

                    # Step 4: Schedule canonical failure event for next deterministic tick
                    runner.sim_engine.event_scheduler.schedule(
                        ScheduledEvent(
                            tick=failure_tick,
                            event_type=ScheduledEventType.UAV_FAILURE,
                            uav_id=original_uav,
                            reason="In-flight propulsion failure demo",
                        )
                    )
                    timeline_entries.append(
                        f"Tick {task_in_progress_tick} (t={snap_before.simulation_time}s): Task '{selected_task}' IN_PROGRESS by '{original_uav}' (actively servicing)"
                    )
                    break

        # Step 5: Execute simulation step
        step = runner.step()
        recorder.record(step.snapshot)

        # Step 6 & 7: Verification at failure tick
        if failure_tick is not None and step.tick == failure_tick:
            # Verify hook captured DEFERRED transition
            matching_deferred = [
                d for d in hook_captured_deferred
                if d["tick"] == failure_tick and d["task_id"] == selected_task
            ]
            if matching_deferred:
                deferred_verified = True
                timeline_entries.append(
                    f"Tick {failure_tick} (t={step.simulation_time}s) [Pre-Autonomy]: UAV '{original_uav}' FAILED -> Task '{selected_task}' transitioned to DEFERRED (assigned_uav=None)"
                )

            # Check post-autonomy reassignment
            snap_after = step.snapshot
            u_fail = snap_after.uavs[original_uav]
            t_post = snap_after.tasks[selected_task]

            assert not u_fail.active, f"UAV {original_uav} should be inactive"
            assert u_fail.failure_state == FailureState.FAILED, f"UAV {original_uav} should be in FAILED state"
            assert t_post.status == TaskStatus.ASSIGNED, f"Task {selected_task} should be re-assigned"
            assert t_post.assigned_uav_id != original_uav, f"Task {selected_task} must be reassigned to another UAV"

            replacement_uav = t_post.assigned_uav_id
            reassignment_tick = step.tick
            timeline_entries.append(
                f"Tick {reassignment_tick} (t={step.simulation_time}s) [Post-Autonomy]: A1 reassigns SAME task '{selected_task}' to replacement UAV '{replacement_uav}'"
            )

        # Track domain events for timeline
        for ev in step.events:
            ev_type = ev.event_type.value if hasattr(ev.event_type, "value") else str(ev.event_type)
            if ev_type == "TASK_ASSIGNED" and step.tick != reassignment_tick:
                timeline_entries.append(
                    f"Tick {step.tick} (t={step.simulation_time}s): TASK_ASSIGNED -> {ev.entity_id} (task: {ev.payload.get('task_id')})"
                )
            elif ev_type == "UAV_FAILED":
                timeline_entries.append(
                    f"Tick {step.tick} (t={step.simulation_time}s): UAV_FAILED -> {ev.entity_id} ({ev.payload.get('reason')})"
                )
            elif ev_type == "TASK_COMPLETED":
                if ev.entity_id == selected_task:
                    completion_tick = step.tick
                timeline_entries.append(
                    f"Tick {step.tick} (t={step.simulation_time}s): TASK_COMPLETED -> task '{ev.entity_id}' completed by '{ev.payload.get('uav_id')}'"
                )
            elif ev_type == "RTH_TRIGGERED":
                timeline_entries.append(
                    f"Tick {step.tick} (t={step.simulation_time}s): RTH_TRIGGERED -> {ev.entity_id}"
                )
            elif ev_type == "UAV_LANDED":
                timeline_entries.append(
                    f"Tick {step.tick} (t={step.simulation_time}s): UAV_LANDED -> {ev.entity_id} at GCS (energy: {round(ev.payload.get('final_energy', 0.0), 2)} Wh)"
                )

    # Step 8 & 9 verifications
    assert deferred_verified, "TASK_DEFERRED was not verified in StateStore lifecycle"
    assert completion_tick is not None, f"Replacement UAV failed to complete task {selected_task}"

    # Build Replay JSON
    replay_data = {
        "scenario": scenario.name,
        "seed": seed,
        "total_ticks": len(runner.history),
        "gcs_position": scenario.gcs_position,
        "tasks": [dict(t) for t in scenario.tasks],
        "events": [
            {
                "tick": e.simulation_tick,
                "time": e.simulation_time,
                "type": e.event_type.value if hasattr(e.event_type, "value") else str(e.event_type),
                "entity_id": e.entity_id,
                "payload": e.payload,
            }
            for e in runner.all_events
        ],
        "ticks": [
            {
                "tick": step.tick,
                "time": step.simulation_time,
                "uavs": {
                    uid: {
                        "position": list(u.position_xy),
                        "velocity": list(u.velocity_xy),
                        "active": u.active,
                        "failure_state": u.failure_state.value if hasattr(u.failure_state, "value") else str(u.failure_state),
                        "battery_percent": round(u.battery_percent, 2),
                        "battery_energy": round(u.battery_energy, 2),
                        "assigned_task": u.assigned_task_id,
                        "rth_state": u.rth_state.value if hasattr(u.rth_state, "value") else str(u.rth_state),
                    }
                    for uid, u in step.snapshot.uavs.items()
                },
                "tasks": {
                    tid: {
                        "status": t.status.value if hasattr(t.status, "value") else str(t.status),
                        "assigned_uav": t.assigned_uav_id,
                        "progress": round(t.service_progress, 2),
                    }
                    for tid, t in step.snapshot.tasks.items()
                },
                "network": {
                    "connected_uavs": list(step.network_analysis.connected_uav_ids),
                    "hop_counts": dict(step.network_analysis.hop_counts),
                    "active_links": [
                        {
                            "source": link.source_id,
                            "target": link.target_id,
                            "pdr": round(link.estimated_pdr, 4),
                            "distance": round(link.distance, 2),
                        }
                        for link in step.network_analysis.network.links
                        if link.active
                    ],
                },
            }
            for step in runner.history
        ],
    }

    replay_path = out_dir / "replay_in_flight_recovery.json"
    with open(replay_path, "w", encoding="utf-8") as f:
        json.dump(replay_data, f, indent=2)

    timeline_path = out_dir / "timeline_in_flight_recovery.txt"
    timeline_content = (
        "================================================================================\n"
        "DEMO EVENT TIMELINE: IN-FLIGHT FAILURE AND DYNAMIC A1 RECOVERY\n"
        "================================================================================\n"
        + "\n".join(timeline_entries)
        + "\n================================================================================\n"
    )
    with open(timeline_path, "w", encoding="utf-8") as f:
        f.write(timeline_content)

    safety_rep = runner.safety_assessor.report

    summary = {
        "scenario_name": scenario.name,
        "seed": seed,
        "original_uav": original_uav,
        "selected_task": selected_task,
        "task_in_progress_tick": task_in_progress_tick,
        "failure_tick": failure_tick,
        "deferred_verified": deferred_verified,
        "replacement_uav": replacement_uav,
        "reassignment_tick": reassignment_tick,
        "completion_tick": completion_tick,
        "total_ticks": len(runner.history),
        "separation_violations": safety_rep.separation_violations_count,
        "min_separation_m": round(safety_rep.min_observed_separation_m, 2),
        "replay_path": str(replay_path),
        "timeline_path": str(timeline_path),
        "timeline_entries": timeline_entries,
    }

    print(timeline_content)
    print("DEMO VERIFICATION SUMMARY:")
    print(f"  Scenario:               {summary['scenario_name']}")
    print(f"  Seed:                   {summary['seed']}")
    print(f"  Original UAV:           {summary['original_uav']}")
    print(f"  Task ID:                {summary['selected_task']}")
    print(f"  IN_PROGRESS Tick:       {summary['task_in_progress_tick']}")
    print(f"  Failure Tick:           {summary['failure_tick']}")
    print(f"  TASK_DEFERRED Tick:     {summary['failure_tick']} (verified via StateStore lifecycle)")
    print(f"  Replacement UAV:        {summary['replacement_uav']}")
    print(f"  Reassignment Tick:      {summary['reassignment_tick']}")
    print(f"  Task Completion Tick:   {summary['completion_tick']}")
    print(f"  Separation Violations:  {summary['separation_violations']}")
    print(f"  Min Separation:         {summary['min_separation_m']}m")
    print(f"  Replay Dataset:         {summary['replay_path']}")
    print(f"  Event Timeline:         {summary['timeline_path']}")

    return summary


def main():
    parser = argparse.ArgumentParser(description="Run in-flight recovery demo.")
    parser.add_argument("--seed", type=int, default=42, help="Deterministic random seed (default: 42)")
    parser.add_argument("--output-dir", type=str, default=None, help="Output directory for replay and timeline")
    args = parser.parse_args()

    run_in_flight_recovery_demo(seed=args.seed, output_dir=args.output_dir)


if __name__ == "__main__":
    main()
