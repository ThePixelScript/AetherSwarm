import json
from ares_swarm.simulation.runner import MissionRunner, MissionConfig
from ares_swarm.simulation.poc_scenario import PoCScenarioBuilder
from ares_swarm.autonomy.task_allocator import A0TaskAllocator
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.core.models import StateSnapshot, UAVState, TaskState
from ares_swarm.core.enums import FailureState, TaskStatus
from ares_swarm.communication.config import CommunicationConfig

def run_experiment(name: str, a1: bool, seed: int = 42):
    allocator = A1TaskAllocator() if a1 else A0TaskAllocator()
    
    # We use PoCScenarioBuilder for deterministic setup
    builder = PoCScenarioBuilder(seed=seed)
    # 5 UAVs, 10 tasks
    snapshot = builder.build_initial_state(num_uavs=5, num_tasks=10)
    
    # Customize snapshot based on experiment
    uavs = dict(snapshot.uavs)
    tasks = dict(snapshot.tasks)
    
    # E0: Baseline (no changes)
    
    # E1: Relay failure
    scheduled_events = []
    if name == "E1":
        # Fail u2 at 300s
        from ares_swarm.core.event_scheduler import ScheduledEvent, ScheduledEventType
        scheduled_events.append(ScheduledEvent(
            simulation_tick=300,
            event_type=ScheduledEventType.UAV_FAILURE,
            uav_id="u2",
            reason="Relay failure E1"
        ))
        
    if name == "E3":
        # Clustered tasks far away to test competing priorities
        for tid, t in tasks.items():
            tasks[tid] = TaskState(
                id=t.id,
                position_xy=(800.0, 800.0),
                priority=t.priority,
                status=t.status
            )
            
    if name == "E4":
        # Battery constrained
        for uid, u in uavs.items():
            uavs[uid] = UAVState(
                id=u.id,
                position_xy=u.position_xy,
                active=u.active,
                battery_capacity=1000.0,
                battery_energy=1000.0,
            )

    custom_snapshot = StateSnapshot(
        simulation_tick=snapshot.simulation_tick,
        simulation_time=snapshot.simulation_time,
        state_version=snapshot.state_version,
        uavs=uavs,
        tasks=tasks,
        gcs_position=snapshot.gcs_position
    )

    runner = MissionRunner(
        config=MissionConfig(
            dt=1.0,
            max_speed=5.0,
            total_mission_time_s=2700,
        ),
        allocator=allocator,
        comm_config=CommunicationConfig(max_range=200.0, packet_loss=0.0)
    )
    
    # Inject scheduled events if any
    runner.simulation_engine.event_scheduler.add_events(scheduled_events)

    metrics = runner.run_mission(custom_snapshot)
    
    print(f"[{name} - {'A1' if a1 else 'A0'}]")
    print(f"Tasks Completed: {metrics.tasks_completed}/{metrics.tasks_total}")
    print(f"Connectivity: {metrics.connectivity_availability*100:.1f}%")
    print(f"Downtime: {metrics.downtime_s}s")
    print(f"Route PDR: {metrics.model_estimated_route_pdr}")
    print(f"Min Sep: {metrics.min_inter_uav_separation_m:.2f}m")
    print(f"Sep Violations: {metrics.collision_count}")
    print("-" * 40)
    return metrics

if __name__ == "__main__":
    for exp in ["E0", "E1", "E3", "E4"]:
        run_experiment(exp, a1=False)
        run_experiment(exp, a1=True)
