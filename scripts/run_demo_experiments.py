import copy
from pathlib import Path

from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import load_scenario
from ares_swarm.autonomy.task_allocator import A0TaskAllocator
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.core.event_scheduler import ScheduledEvent, ScheduledEventType

def run_experiment(name: str, a1: bool, seed: int = 42):
    # Load base scenario
    base_scenario = load_scenario(Path("scenarios/poc_round1.yaml"))
    import dataclasses
    
    # E0: Baseline (no changes)
    
    # E1: Relay failure
    scheduled_events = []
    if name == "E1":
        # Fail uav_2 at 300s
        scheduled_events.append(ScheduledEvent(
            tick=300,
            event_type=ScheduledEventType.UAV_FAILURE,
            uav_id="uav_2",
            reason="Relay failure E1"
        ))
        
    tasks = base_scenario.tasks
    uavs = base_scenario.uavs
        
    if name == "E3":
        # Clustered tasks far away to test competing priorities
        new_tasks = []
        for t in base_scenario.tasks:
            t_mod = dict(t)
            t_mod["position"] = [800.0, 800.0]
            new_tasks.append(t_mod)
        tasks = tuple(new_tasks)
            
    if name == "E4":
        # Battery constrained (75% of 4200 baseline)
        new_uavs = []
        for u in base_scenario.uavs:
            u_mod = dict(u)
            u_mod["battery_capacity"] = 3150.0
            u_mod["battery_energy"] = 3150.0
            new_uavs.append(u_mod)
        uavs = tuple(new_uavs)

    base_scenario = dataclasses.replace(
        base_scenario, 
        communication=dataclasses.replace(base_scenario.communication, max_range=100.0),
        tasks=tasks,
        uavs=uavs
    )

    conds = []
    if name == "E2":
        from ares_swarm.communication.scenario import CommunicationCondition
        # Build E2 impairment scenario
        # normal topology -> degradation -> outage -> recovery
        # 0 - 150: normal
        # 150 - 250: degradation on key relays (e.g. uav_1 <-> uav_2, uav_2 <-> uav_3)
        # 250 - 350: full outage
        # 350+: recovery
        conds = [
            CommunicationCondition(source_id="uav_1", target_id="uav_2", start_time_s=150.0, end_time_s=250.0, quality_multiplier=0.3),
            CommunicationCondition(source_id="uav_2", target_id="uav_3", start_time_s=150.0, end_time_s=250.0, quality_multiplier=0.3),
            CommunicationCondition(source_id="uav_1", target_id="uav_2", start_time_s=250.0, end_time_s=350.0, outage=True),
            CommunicationCondition(source_id="uav_2", target_id="uav_3", start_time_s=250.0, end_time_s=350.0, outage=True),
        ]

    from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
    comm_analyzer = BaselineCommunicationAnalyzer(
        config=base_scenario.communication,
        scenario_conditions=tuple(conds)
    )

    allocator = A1TaskAllocator(comm_analyzer=comm_analyzer) if a1 else A0TaskAllocator()
    adapter = A0AutonomyAdapter(allocator=allocator)

    runner = MissionRunner(
        scenario=base_scenario,
        seed=seed,
        autonomy_adapter=adapter,
        comm_analyzer=comm_analyzer
    )

    # Inject scheduled events if any
    for evt in scheduled_events:
        runner.sim_engine.event_scheduler.schedule(evt)

    metrics = runner.run()
    summary = metrics.to_dict()
    eval_m = summary.get("evaluation", {})
    mission = eval_m.get("mission", {})
    comm = eval_m.get("communication", {})
    safe = eval_m.get("safety", {})
    
    print(f"[{name} - {'A1' if a1 else 'A0'}]")
    print(f"Tasks Completed: {mission.get('tasks_completed', 0)}/{mission.get('tasks_total', 0)}")
    print(f"Completion Time: {mission.get('completion_time_s', 0)}s")
    print(f"Priority Score: {mission.get('priority_weighted_score', 0):.4f}")
    print(f"Energy Consumed: {mission.get('total_energy_consumed_wh', 0):.2f} Wh")
    print(f"Connectivity: {comm.get('connectivity_availability', 0)*100:.1f}%")
    print(f"Downtime: {comm.get('downtime_s', 0)}s")
    print(f"Route PDR: {comm.get('model_estimated_route_pdr')}")
    print(f"Route Latency: {comm.get('model_estimated_route_latency_ms')} ms")
    print(f"Min Sep: {safe.get('min_inter_uav_separation_m', 0.0):.2f}m")
    print(f"Sep Violations: {safe.get('separation_violation_count', 0)}")
    print("-" * 40)
    return metrics

if __name__ == "__main__":
    for exp in ["E0", "E1", "E2", "E3", "E4"]:
        run_experiment(exp, a1=False)
        run_experiment(exp, a1=True)
