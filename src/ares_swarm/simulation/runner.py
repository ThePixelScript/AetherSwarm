"""Central MissionRunner and scenario execution layer."""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
from pathlib import Path
import random
from typing import Any, Callable, Optional, Sequence

from ..autonomy.a0_adapter import A0AutonomyAdapter
from ..autonomy.task_allocator import A0TaskAllocator
from ..communication.analysis import BaselineCommunicationAnalyzer
from ..core.commands import AssignTaskCommand, ProgressTaskCommand, StepPhysicsCommand
from ..core.enums import TaskStatus
from ..core.events import CommandRejection, DomainEvent, StateTransitionResult
from ..core.models import StateSnapshot, TaskState, UAVState
from ..core.simulator import SimulationEngine
from ..core.state_store import StateStore
from ..interfaces.communication import NetworkAnalysis
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

    def to_dict(self) -> dict[str, Any]:
        snap = self.final_snapshot
        uav_summaries = {
            uid: {
                "position": list(u.position_xy),
                "velocity": list(u.velocity_xy),
                "battery_energy": round(u.battery_energy, 4),
                "battery_percent": round(u.battery_percent, 2),
                "role": u.role.value if hasattr(u.role, "value") else str(u.role),
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

        tasks_completed = sum(1 for t in snap.tasks.values() if t.status == TaskStatus.COMPLETE)
        tasks_assigned = sum(1 for t in snap.tasks.values() if t.assigned_uav_id is not None or t.status == TaskStatus.COMPLETE)

        initial_energy = sum(self.step_history[0].snapshot.uavs[u].battery_capacity for u in snap.uavs) if self.step_history else sum(u.battery_capacity for u in snap.uavs.values())
        final_energy = sum(u.battery_energy for u in snap.uavs.values())
        total_energy_consumed = round(max(0.0, initial_energy - final_energy), 4)

        last_net = self.step_history[-1].network_analysis if self.step_history else None
        connected_uavs = list(last_net.connected_uav_ids) if last_net else []

        return {
            "scenario_name": self.scenario_name,
            "seed": self.seed,
            "total_ticks": self.total_ticks,
            "final_simulation_time": round(self.simulation_time, 4),
            "metrics": {
                "tasks_total": len(snap.tasks),
                "tasks_assigned": tasks_assigned,
                "tasks_completed": tasks_completed,
                "total_energy_consumed": total_energy_consumed,
                "final_connected_uav_count": len(connected_uavs),
                "final_connected_uavs": connected_uavs,
                "total_events_generated": len(self.all_events),
            },
            "uavs": uav_summaries,
            "tasks": task_summaries,
        }

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
    ):
        if isinstance(scenario, ScenarioConfig):
            self.scenario = scenario
        else:
            self.scenario = load_scenario(scenario)

        self.seed = seed if seed is not None else self.scenario.seed
        self.enable_task_progress = enable_task_progress
        self.safety_hook = safety_hook

        self.state_store: StateStore
        self.comm_analyzer: BaselineCommunicationAnalyzer
        self.autonomy_adapter: A0AutonomyAdapter
        self.sim_engine: SimulationEngine
        self.history: list[StepResult] = []
        self.all_events: list[DomainEvent] = []

        self.reset(self.seed)

    def reset(self, seed: int | None = None) -> StateSnapshot:
        """Reset runner to initial deterministic state."""
        if seed is not None:
            self.seed = seed
        random.seed(self.seed)

        initial_snapshot = create_initial_snapshot(self.scenario)
        self.state_store = StateStore(initial_snapshot)
        self.comm_analyzer = BaselineCommunicationAnalyzer(config=self.scenario.communication)
        self.autonomy_adapter = A0AutonomyAdapter(allocator=A0TaskAllocator())
        self.sim_engine = SimulationEngine(
            state_store=self.state_store,
            dt=self.scenario.dt,
            idle_rate=self.scenario.battery_idle_rate,
            movement_rate=self.scenario.battery_movement_rate,
        )
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

        # 1. Gamma Communication Analysis (read-only)
        net_analysis = self.comm_analyzer.analyze(current_snap)

        # 2. Safety hook (optional interface for safety assessment)
        if self.safety_hook:
            self.safety_hook(current_snap, net_analysis)

        # 3. Beta A0 Autonomy Allocation
        pending_tasks = [t for t in current_snap.tasks.values() if t.status == TaskStatus.PENDING]
        if pending_tasks:
            assign_cmds = self.autonomy_adapter.plan(current_snap)
            if assign_cmds:
                res_assign = self.state_store.apply(assign_cmds)
                applied_commands.extend(res_assign.applied_commands)
                rejected_commands.extend(res_assign.rejected_commands)
                tick_events.extend(res_assign.emitted_events)

        # 4. Task Service Progress (Simulation domain progression upon arrival)
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

        # 5. Delta Physics Simulation Swarm Stepping (Batched transaction)
        step_res = self.sim_engine.step_swarm(speed=self.scenario.speed_limit)
        applied_commands.extend(step_res.applied_commands)
        rejected_commands.extend(step_res.rejected_commands)
        tick_events.extend(step_res.emitted_events)

        # 6. Authoritative Clock Advance
        self.sim_engine.advance_tick()

        # 7. Collect updated authoritative state
        final_snap = self.state_store.snapshot()
        self.all_events.extend(tick_events)

        result = StepResult(
            tick=current_tick,
            simulation_time=current_snap.simulation_time,
            network_analysis=net_analysis,
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
            self.step()

        final_snap = self.state_store.snapshot()
        return MissionResult(
            scenario_name=self.scenario.name,
            seed=self.seed,
            total_ticks=len(self.history),
            simulation_time=final_snap.simulation_time,
            final_snapshot=final_snap,
            step_history=list(self.history),
            all_events=list(self.all_events),
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
