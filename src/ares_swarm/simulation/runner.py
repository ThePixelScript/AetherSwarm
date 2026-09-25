"""Central MissionRunner and scenario execution layer."""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field, replace
import json
from pathlib import Path
import random
from types import MappingProxyType
from typing import Any, Callable, Optional, Sequence
import inspect

from ..autonomy.a0_adapter import A0AutonomyAdapter
from ..autonomy.task_allocator import A0TaskAllocator
from ..communication.analysis import BaselineCommunicationAnalyzer
from ..core.commands import (
    AssignTaskCommand,
    CompleteRTHCommand,
    ProgressTaskCommand,
    SetTargetPositionCommand,
    StepPhysicsCommand,
)
from ..core.enums import RTHState, TaskStatus
from ..core.events import CommandRejection, DomainEvent, EventType, StateTransitionResult
from ..core.models import StateSnapshot, TaskState, UAVState
from ..core.simulator import SimulationEngine
from ..core.state_store import StateStore
from ..evaluation.metrics import MissionMetricsReport, compute_mission_metrics
from ..interfaces.communication import NetworkAnalysis
from ..safety.airspace import ChallengeAirspace
from ..safety.geofence import GeofenceEnforcer
from ..safety.safety_assessor import SafetyAssessor, SafetyReport
from ..safety.separation import SeparationEnforcer
from ..telemetry.manager import DetectionManager
from .scenario import ScenarioConfig, create_initial_snapshot, load_scenario


@dataclass(frozen=True)
class StepResult:
    tick: int
    simulation_time: float
    network_analysis: NetworkAnalysis
    applied_commands: tuple[Any, ...]
    rejected_commands: tuple[CommandRejection, ...]
    events: tuple[DomainEvent, ...]
    snapshot: StateSnapshot


@dataclass
class MissionResult:
    scenario_name: str
    seed: int
    total_ticks: int
    simulation_time: float
    final_snapshot: StateSnapshot
    step_history: list[StepResult] = field(default_factory=list)
    all_events: list[DomainEvent] = field(default_factory=list)
    initial_snapshot: StateSnapshot | None = None
    safety_report: Any = None
    metrics_report: MissionMetricsReport | None = None
    telemetry_manager: Any = None
    separation_enforcer: Any = None
    geofence_enforcer: Any = None

    def to_dict(self) -> dict[str, Any]:
        snap = self.final_snapshot
        uav_summaries = {
            uid: {
                "position": list(u.position_xy),
                "velocity": list(u.velocity_xy),
                "battery_energy": round(u.battery_energy, 4),
                "battery_percent": round(u.battery_percent, 2),
                "role": u.role.value if hasattr(u.role, "value") else str(u.role),
                "rth_state": u.rth_state.value if hasattr(u.rth_state, "value") else str(u.rth_state),
                "active": u.active,
                "assigned_task_id": u.assigned_task_id,
            }
            for uid, u in sorted(snap.uavs.items())
        }

        task_summaries = {
            tid: {
                "status": t.status.value if hasattr(t.status, "value") else str(t.status),
                "service_progress": round(t.service_progress, 4),
                "service_duration": round(t.service_duration, 4),
                "assigned_uav_id": t.assigned_uav_id,
            }
            for tid, t in sorted(snap.tasks.items())
        }

        tasks_total = len(snap.tasks)
        sim_time = snap.simulation_time
        tasks_spawned = sum(1 for t in snap.tasks.values() if t.created_time <= sim_time)
        tasks_completed = sum(1 for t in snap.tasks.values() if t.status == TaskStatus.COMPLETE)
        tasks_assigned = sum(1 for t in snap.tasks.values() if t.status in (TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS))
        tasks_expired = sum(
            1 for t in snap.tasks.values()
            if t.status != TaskStatus.COMPLETE and (
                t.status in (TaskStatus.UNREACHABLE, TaskStatus.DEFERRED) or
                (t.created_time <= sim_time and t.deadline > 0.0 and sim_time >= t.deadline)
            )
        )
        mission_completion_rate = round(tasks_completed / tasks_total, 4) if tasks_total > 0 else 0.0

        initial_energy = sum(self.initial_snapshot.uavs[u].battery_capacity for u in snap.uavs) if self.initial_snapshot else (
            sum(self.step_history[0].snapshot.uavs[u].battery_capacity for u in snap.uavs) if self.step_history else sum(u.battery_capacity for u in snap.uavs.values())
        )
        final_energy = sum(u.battery_energy for u in snap.uavs.values())
        total_energy_consumed = round(max(0.0, initial_energy - final_energy), 4)

        last_net = self.step_history[-1].network_analysis if self.step_history else None
        connected_uavs = list(last_net.connected_uav_ids) if last_net else []

        eval_report = None
        if self.metrics_report:
            eval_report = self.metrics_report.to_dict()
        elif self.initial_snapshot:
            eval_report = compute_mission_metrics(
                step_history=self.step_history,
                initial_snapshot=self.initial_snapshot,
                final_snapshot=snap,
                dt=1.0,
                safety_report=self.safety_report,
                telemetry_manager=self.telemetry_manager,
                separation_enforcer=self.separation_enforcer,
                geofence_enforcer=self.geofence_enforcer,
            ).to_dict()

        res_dict = {
            "scenario_name": self.scenario_name,
            "seed": self.seed,
            "total_ticks": self.total_ticks,
            "final_simulation_time": round(self.simulation_time, 4),
            "metrics": {
                "tasks_total": tasks_total,
                "tasks_spawned": tasks_spawned,
                "tasks_assigned": tasks_assigned,
                "tasks_completed": tasks_completed,
                "tasks_expired": tasks_expired,
                "mission_completion_rate": mission_completion_rate,
                "total_energy_consumed": total_energy_consumed,
                "final_connected_uav_count": len(connected_uavs),
                "final_connected_uavs": connected_uavs,
                "total_events_generated": len(self.all_events),
            },
            "uavs": uav_summaries,
            "tasks": task_summaries,
        }
        if eval_report:
            res_dict["evaluation"] = eval_report
        if self.telemetry_manager is not None:
            res_dict["telemetry"] = self.telemetry_manager.get_metrics()
            res_dict["telemetry_reports"] = {
                tid: r.to_dict()
                for tid, r in sorted(self.telemetry_manager.authoritative_reports.items())
            }
        if self.separation_enforcer is not None:
            res_dict["safety_interventions"] = {
                "total_interventions": self.separation_enforcer.total_interventions,
                "per_uav_interventions": dict(self.separation_enforcer.per_uav_interventions),
            }
        if self.geofence_enforcer is not None:
            res_dict["geofence_interventions"] = {
                "total_interventions": self.geofence_enforcer.total_interventions,
                "per_uav_interventions": dict(self.geofence_enforcer.per_uav_interventions),
                "min_observed_clearance_m": (
                    round(self.geofence_enforcer.min_observed_clearance_m, 2)
                    if self.geofence_enforcer.min_observed_clearance_m != float("inf")
                    else None
                ),
            }
        return res_dict

    def save_json(self, output_path: str | Path) -> Path:
        out = Path(output_path)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2)
        return out


class MissionRunner:
    """Deterministic central multi-tick coordinator for swarm mission scenarios."""

    def __init__(
        self,
        scenario: ScenarioConfig | str | Path | dict[str, Any],
        seed: int | None = None,
        enable_task_progress: bool = True,
        safety_hook: Optional[Callable[[StateSnapshot, NetworkAnalysis], Any]] = None,
        autonomy_adapter: Optional[Any] = None,
        comm_analyzer: Optional[Any] = None,
    ):
        if isinstance(scenario, ScenarioConfig):
            self.scenario = scenario
        else:
            self.scenario = load_scenario(scenario)

        self.seed = seed if seed is not None else self.scenario.seed
        self.enable_task_progress = enable_task_progress
        self.safety_hook = safety_hook
        self._custom_autonomy_adapter = autonomy_adapter
        self._custom_comm_analyzer = comm_analyzer

        self.initial_snapshot: StateSnapshot
        self.state_store: StateStore
        self.comm_analyzer: BaselineCommunicationAnalyzer
        self.autonomy_adapter: Any
        self.sim_engine: SimulationEngine
        airspace = None
        challenge_profile = getattr(self.scenario, "challenge_profile", None)
        enforce_sortie = False
        enforce_single = False
        max_sortie_s = 1200.0
        rth_safety_margin_s = 15.0
        self.detection_manager: DetectionManager | None = None
        self.separation_enforcer: SeparationEnforcer | None = None
        self.geofence_enforcer: GeofenceEnforcer | None = None

        if challenge_profile and challenge_profile.enabled:
            enforce_sortie = challenge_profile.enforce_sortie_limit
            enforce_single = challenge_profile.enforce_single_sortie
            max_sortie_s = challenge_profile.max_sortie_duration_s
            rth_safety_margin_s = challenge_profile.rth_safety_margin_s
            if challenge_profile.airspace.enabled:
                airspace = ChallengeAirspace(
                    arena_bounds_x=challenge_profile.airspace.arena_bounds_x,
                    arena_bounds_y=challenge_profile.airspace.arena_bounds_y,
                    max_height=challenge_profile.airspace.max_height,
                    staging_pad_center=challenge_profile.airspace.staging_pad_center,
                    staging_pad_radius_m=challenge_profile.airspace.staging_pad_radius_m,
                    corridor_bounds_x=challenge_profile.airspace.corridor_bounds_x,
                    corridor_bounds_y=challenge_profile.airspace.corridor_bounds_y,
                )
            if challenge_profile.detection_pipeline.enabled:
                self.detection_manager = DetectionManager(
                    config=challenge_profile.detection_pipeline,
                    gcs_position=self.scenario.gcs_position,
                )
            if challenge_profile.enforce_separation:
                self.separation_enforcer = SeparationEnforcer(
                    min_separation_m=self.scenario.min_separation_m,
                    gcs_position=self.scenario.gcs_position,
                    staging_pad_radius_m=airspace.staging_pad_radius_m if airspace else 15.0,
                )
            if getattr(challenge_profile, "enforce_geofence", False) and airspace is not None:
                self.geofence_enforcer = GeofenceEnforcer(
                    airspace=airspace,
                )

        self.safety_assessor = SafetyAssessor(
            arena_bounds_x=self.scenario.arena_bounds_x,
            arena_bounds_y=self.scenario.arena_bounds_y,
            min_separation_m=self.scenario.min_separation_m,
            gcs_position=self.scenario.gcs_position,
            airspace=airspace,
            max_sortie_duration_s=max_sortie_s,
            rth_safety_margin_s=rth_safety_margin_s,
            enforce_sortie_limit=enforce_sortie,
            enforce_single_sortie=enforce_single,
        )
        self.history: list[StepResult] = []
        self.all_events: list[DomainEvent] = []

        self.reset(self.seed)

    def reset(self, seed: int | None = None) -> StateSnapshot:
        """Reset runner to initial deterministic state."""
        if seed is not None:
            self.seed = seed
        random.seed(self.seed)

        self.initial_snapshot = create_initial_snapshot(self.scenario)
        self.state_store = StateStore(self.initial_snapshot)
        if self._custom_comm_analyzer is not None:
            self.comm_analyzer = self._custom_comm_analyzer
        else:
            self.comm_analyzer = BaselineCommunicationAnalyzer(config=self.scenario.communication)
        if self._custom_autonomy_adapter is not None:
            self.autonomy_adapter = self._custom_autonomy_adapter
        else:
            self.autonomy_adapter = A0AutonomyAdapter(allocator=A0TaskAllocator())
        self.sim_engine = SimulationEngine(
            state_store=self.state_store,
            dt=self.scenario.dt,
            idle_rate=self.scenario.battery_idle_rate,
            movement_rate=self.scenario.battery_movement_rate,
            max_speed=self.scenario.speed_limit,
        )
        self.safety_assessor.reset()
        if self.detection_manager is not None:
            self.detection_manager.reset()
        if self.separation_enforcer is not None:
            self.separation_enforcer.reset()
        if self.geofence_enforcer is not None:
            self.geofence_enforcer.reset()
        self.history.clear()
        self.all_events.clear()
        return self.state_store.snapshot()

    def step(self) -> StepResult:
        """Execute exactly one deterministic simulation tick."""
        current_snap = self.state_store.snapshot()
        current_tick = current_snap.simulation_tick

        applied_commands = []
        rejected_commands = []
        tick_events = []
        # 0. Apply scheduled simulation events before autonomy decisions
        scheduled_res = self.sim_engine.process_scheduled_events()
        applied_commands.extend(scheduled_res.applied_commands)
        rejected_commands.extend(scheduled_res.rejected_commands)
        tick_events.extend(scheduled_res.emitted_events)

        current_snap = self.state_store.snapshot()  

        # 1. Gamma Communication Analysis (read-only)
        net_analysis = self.comm_analyzer.analyze(current_snap)

        # 2. Deterministic Safety Assessment & Preemptive RTH triggers
        self.safety_assessor.assess_snapshot(current_snap, net_analysis)
        if self.scenario.enable_auto_rth:
            challenge_profile = getattr(self.scenario, "challenge_profile", None)
            enable_sortie_rth = bool(
                challenge_profile and challenge_profile.enabled and challenge_profile.enforce_sortie_limit
            )
            rth_cmds = self.safety_assessor.evaluate_rth_triggers(
                current_snap,
                speed_limit=self.scenario.speed_limit,
                idle_rate=self.scenario.battery_idle_rate,
                movement_rate=self.scenario.battery_movement_rate,
                mission_duration=self.scenario.duration,
                enable_battery_rth=True,
                enable_time_rth=self.scenario.return_by_mission_end,
                enable_sortie_rth=enable_sortie_rth,
            )
            if rth_cmds:
                res_rth = self.state_store.apply(rth_cmds)
                applied_commands.extend(res_rth.applied_commands)
                rejected_commands.extend(res_rth.rejected_commands)
                tick_events.extend(res_rth.emitted_events)

        # 3. Optional Safety hook (external observer/policy hook)
        if self.safety_hook:
            self.safety_hook(current_snap, net_analysis)

        # 4. Beta A0 Autonomy Allocation (Filter dynamically visible tasks)
        snap_for_alloc = self.state_store.snapshot()
        visible_tasks = {
            tid: t
            for tid, t in snap_for_alloc.tasks.items()
            if t.created_time <= snap_for_alloc.simulation_time
        }
        unassigned_visible = [
            t for t in visible_tasks.values()
            if t.status in (TaskStatus.PENDING, TaskStatus.DEFERRED)
        ]
        alloc_snap = replace(snap_for_alloc, tasks=MappingProxyType(visible_tasks))
        accepts_net = getattr(
            self.autonomy_adapter,
            "accepts_network_analysis",
            getattr(getattr(self.autonomy_adapter, "allocator", None), "accepts_network_analysis", None),
        )
        if accepts_net is True:
            assign_cmds = self.autonomy_adapter.plan(alloc_snap, net_analysis)
        elif accepts_net is False:
            assign_cmds = self.autonomy_adapter.plan(alloc_snap)
        else:
            sig = inspect.signature(self.autonomy_adapter.plan)
            if "network_analysis" in sig.parameters:
                assign_cmds = self.autonomy_adapter.plan(alloc_snap, net_analysis)
            else:
                assign_cmds = self.autonomy_adapter.plan(alloc_snap)
        if assign_cmds:
            res_assign = self.state_store.apply(assign_cmds)
            applied_commands.extend(res_assign.applied_commands)
            rejected_commands.extend(res_assign.rejected_commands)
            tick_events.extend(res_assign.emitted_events)

        # 5. Task Service Progress (Simulation domain progression upon arrival)
        if self.enable_task_progress:
            snap_mid = self.state_store.snapshot()
            progress_cmds = []
            for uav in sorted(snap_mid.uavs.values(), key=lambda u: u.id):
                if not uav.assigned_task_id:
                    continue
                task = snap_mid.tasks.get(uav.assigned_task_id)
                if not task or task.status not in (TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS):
                    continue
                dx = uav.position_xy[0] - task.position_xy[0]
                dy = uav.position_xy[1] - task.position_xy[1]
                dist = (dx**2 + dy**2) ** 0.5
                if dist <= 0.05:  # At target location
                    progress_cmds.append(
                        ProgressTaskCommand(
                            source_tick=current_tick,
                            uav_id=uav.id,
                            task_id=task.id,
                            delta_progress=self.scenario.dt,
                        )
                    )
            if progress_cmds:
                res_prog = self.state_store.apply(progress_cmds)
                applied_commands.extend(res_prog.applied_commands)
                rejected_commands.extend(res_prog.rejected_commands)
                tick_events.extend(res_prog.emitted_events)

                # Clear stale target_position for any UAV whose task completed
                clear_cmds = []
                for ev in res_prog.emitted_events:
                    if ev.event_type == EventType.TASK_COMPLETED:
                        completed_uav_id = ev.payload.get("uav_id")
                        if completed_uav_id:
                            clear_cmds.append(
                                SetTargetPositionCommand(
                                    source_tick=current_tick,
                                    uav_id=completed_uav_id,
                                    target_position=None,
                                )
                            )
                if clear_cmds:
                    res_clear = self.state_store.apply(clear_cmds)
                    applied_commands.extend(res_clear.applied_commands)
                    rejected_commands.extend(res_clear.rejected_commands)
                    tick_events.extend(res_clear.emitted_events)

        # 6. Delta Physics Simulation Swarm Stepping (Batched transaction)
        flight_phases = self.safety_assessor._uav_flight_phases
        step_res = self.sim_engine.step_swarm(
            speed=self.scenario.speed_limit,
            separation_enforcer=self.separation_enforcer,
            geofence_enforcer=self.geofence_enforcer,
            airspace=self.safety_assessor.airspace,
            flight_phases=flight_phases,
        )
        applied_commands.extend(step_res.applied_commands)
        rejected_commands.extend(step_res.rejected_commands)
        tick_events.extend(step_res.emitted_events)

        # 7. (Removed custom queue)
        snap_post_step = self.state_store.snapshot()

        # 8. RTH Arrival Completion (Canonical CompleteRTHCommand on GCS arrival)
        complete_rth_cmds = []
        gcs_pos = self.scenario.gcs_position
        for uav in sorted(snap_post_step.uavs.values(), key=lambda u: u.id):
            if uav.rth_state == RTHState.ACTIVE:
                dx = uav.position_xy[0] - gcs_pos[0]
                dy = uav.position_xy[1] - gcs_pos[1]
                dist_to_gcs = (dx**2 + dy**2) ** 0.5
                pad_radius = self.safety_assessor.airspace.staging_pad_radius_m if self.safety_assessor.airspace else 0.05
                if dist_to_gcs <= pad_radius:  # Staging pad arrival radius
                    complete_rth_cmds.append(
                        CompleteRTHCommand(
                            source_tick=current_tick,
                            uav_id=uav.id,
                        )
                    )
        if complete_rth_cmds:
            res_complete = self.state_store.apply(complete_rth_cmds)
            applied_commands.extend(res_complete.applied_commands)
            rejected_commands.extend(res_complete.rejected_commands)
            tick_events.extend(res_complete.emitted_events)

        # 8. Post-physics analysis, perception detection, and telemetry routing
        post_physics_snap = self.state_store.snapshot()
        if self.detection_manager is not None:
            post_physics_net = self.comm_analyzer.analyze(post_physics_snap)
            detect_events = self.detection_manager.step_perception(
                post_physics_snap,
                self.safety_assessor.report.uav_flight_records,
            )
            telem_events = self.detection_manager.step_telemetry(
                post_physics_snap,
                post_physics_net,
            )
            tick_events.extend(detect_events)
            tick_events.extend(telem_events)
            active_net_analysis = post_physics_net
        else:
            active_net_analysis = net_analysis

        # 9. Deterministic Safety Assessment After Physics Movement & Landing
        self.safety_assessor.assess_snapshot(post_physics_snap, active_net_analysis)

        # 10. Authoritative Clock Advance
        self.sim_engine.advance_tick()

        # 11. Collect updated authoritative state
        final_snap = self.state_store.snapshot()
        self.all_events.extend(tick_events)

        result = StepResult(
            tick=current_tick,
            simulation_time=current_snap.simulation_time,
            network_analysis=active_net_analysis,
            applied_commands=tuple(applied_commands),
            rejected_commands=tuple(rejected_commands),
            events=tuple(tick_events),
            snapshot=final_snap,
        )
        self.history.append(result)
        return result

    def run(self, max_ticks: int | None = None) -> MissionResult:
        """Run simulation loop until max_ticks or completion."""
        limit = max_ticks if max_ticks is not None else self.scenario.max_ticks

        for _ in range(limit):
            step_res = self.step()
            # Early termination if all UAVs are successfully landed and mission requires return
            if self.scenario.return_by_mission_end:
                if all(not u.active and u.rth_state == RTHState.COMPLETE for u in step_res.snapshot.uavs.values() if u.failure_state.name != "FAILED"):
                    break

        final_snap = self.state_store.snapshot()
        metrics_report = compute_mission_metrics(
            step_history=self.history,
            initial_snapshot=self.initial_snapshot,
            final_snapshot=final_snap,
            dt=self.scenario.dt,
            safety_report=self.safety_assessor.report,
            telemetry_manager=self.detection_manager,
            separation_enforcer=self.separation_enforcer,
            geofence_enforcer=self.geofence_enforcer,
        )
        return MissionResult(
            scenario_name=self.scenario.name,
            seed=self.seed,
            total_ticks=len(self.history),
            simulation_time=final_snap.simulation_time,
            final_snapshot=final_snap,
            step_history=list(self.history),
            all_events=list(self.all_events),
            initial_snapshot=self.initial_snapshot,
            safety_report=self.safety_assessor.report,
            metrics_report=metrics_report,
            telemetry_manager=self.detection_manager,
            separation_enforcer=self.separation_enforcer,
            geofence_enforcer=self.geofence_enforcer,
        )


def main(argv: Sequence[str] | None = None) -> int:
    """CLI entrypoint for ares-simulate."""
    parser = argparse.ArgumentParser(description="ARES Swarm Mission Simulator (M0)")
    parser.add_argument("--scenario", "-s", type=str, default="scenarios/basic.yaml", help="Path to scenario YAML file")
    parser.add_argument("--seed", type=int, default=None, help="Deterministic random seed")
    parser.add_argument("--ticks", "-t", type=int, default=None, help="Maximum simulation ticks override")
    parser.add_argument("--output", "-o", type=str, default=None, help="Output JSON path for summary results")

    args = parser.parse_args(argv)

    scenario_path = Path(args.scenario)
    if not scenario_path.is_file():
        # Fall back to default scenario if not found
        default_scenario = ScenarioConfig(
            name="cli_default_mission",
            seed=args.seed if args.seed is not None else 42,
            dt=1.0,
            speed_limit=5.0,
            duration=10.0,
            max_ticks=10,
            uavs=(
                {"id": "u1", "position": [0.0, 0.0], "battery_capacity": 100.0},
                {"id": "u2", "position": [0.0, 0.0], "battery_capacity": 100.0},
                {"id": "u3", "position": [0.0, 0.0], "battery_capacity": 100.0},
            ),
            tasks=(
                {"id": "t1", "position": [0.0, 10.0], "priority": 2, "service_duration": 2.0},
                {"id": "t2", "position": [10.0, 0.0], "priority": 1, "service_duration": 2.0},
            ),
        )
        runner = MissionRunner(default_scenario, seed=args.seed)
    else:
        runner = MissionRunner(scenario_path, seed=args.seed)

    result = runner.run(max_ticks=args.ticks)
    summary = result.to_dict()

    out_path = args.output
    if not out_path:
        out_path = Path("results") / result.scenario_name / "summary.json"
    result_path = result.save_json(out_path)

    print(f"Simulation completed successfully.")
    print(f"Scenario: {summary['scenario_name']}")
    print(f"Total Ticks: {summary['total_ticks']}")
    print(f"Final Time: {summary['final_simulation_time']}s")
    print(f"Tasks Completed: {summary['metrics']['tasks_completed']} / {summary['metrics']['tasks_total']}")
    print(f"Total Energy Consumed: {summary['metrics']['total_energy_consumed']} Wh")
    print(f"Results written to: {result_path}")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
