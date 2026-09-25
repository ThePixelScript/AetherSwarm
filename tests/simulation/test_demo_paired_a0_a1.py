import pytest
import dataclasses
from pathlib import Path
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import load_scenario
from ares_swarm.autonomy.task_allocator import A0TaskAllocator
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.core.event_scheduler import ScheduledEvent, ScheduledEventType
from ares_swarm.core.enums import FailureState

def get_mission_runner(name: str, a1: bool) -> MissionRunner:
    allocator = A1TaskAllocator() if a1 else A0TaskAllocator()
    from ares_swarm.autonomy.ingress_coordinator import IngressCoordinator
    from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
    from ares_swarm.communication.config import CommunicationConfig
    ingress_coordinator = IngressCoordinator(
        comm_analyzer=BaselineCommunicationAnalyzer(CommunicationConfig(max_range=100.0)),
        gcs_position=(-75.0, 500.0),
        d_safe=85.0,
    )
    adapter = A0AutonomyAdapter(allocator=allocator, ingress_coordinator=ingress_coordinator)

    # Load base scenario
    base_scenario = load_scenario(Path("scenarios/poc_round1.yaml"))
    base_scenario = dataclasses.replace(base_scenario, communication=dataclasses.replace(base_scenario.communication, max_range=100.0))

    scheduled_events = []
    if name == "E1":
        scheduled_events.append(ScheduledEvent(
            tick=300,
            event_type=ScheduledEventType.UAV_FAILURE,
            uav_id="uav_2",
            reason="Relay failure E1"
        ))

    tasks = base_scenario.tasks
    uavs = base_scenario.uavs

    if name == "E4":
        new_uavs = []
        for u in base_scenario.uavs:
            u_mod = dict(u)
            u_mod["battery_capacity"] = 3150.0
            u_mod["battery_energy"] = 3150.0
            new_uavs.append(u_mod)
        uavs = tuple(new_uavs)

    scenario = dataclasses.replace(base_scenario, tasks=tasks, uavs=uavs)

    runner = MissionRunner(scenario=scenario, seed=42, autonomy_adapter=adapter)
    for evt in scheduled_events:
        runner.sim_engine.event_scheduler.schedule(evt)

    return runner

def test_demo_scenarios():
    runner_a0_e0 = get_mission_runner("E0", a1=False)
    runner_a1_e0 = get_mission_runner("E0", a1=True)

    # Verify A0/A1 fairness on E0
    assert runner_a0_e0.scenario.seed == runner_a1_e0.scenario.seed == 42
    assert runner_a0_e0.scenario.communication.max_range == 100.0

    runner_a0_e4 = get_mission_runner("E4", a1=False)
    # E4 initial battery is lower than E0
    e0_bat = runner_a0_e0.scenario.uavs[0]["battery_energy"]
    e4_bat = runner_a0_e4.scenario.uavs[0]["battery_energy"]
    assert e4_bat < e0_bat
    assert e4_bat == 3150.0

    # E1 schedules valid uav_2 failure and failure event actually occurs
    runner_a0_e1 = get_mission_runner("E1", a1=False)
    # Verify it is scheduled
    assert any(evt.uav_id == "uav_2" and evt.event_type == ScheduledEventType.UAV_FAILURE for evt in runner_a0_e1.sim_engine.event_scheduler._events)

    # Run E1 briefly past tick 300
    metrics = runner_a0_e1.run(max_ticks=305)

    # Verify UAV_FAILED event actually occurs
    uav_failed_events = [e for e in runner_a0_e1.all_events if e.event_type.value == "UAV_FAILED"]
    assert len(uav_failed_events) == 1
    assert uav_failed_events[0].entity_id == "uav_2"

    # Verify StateStore reflects failure
    snapshot = runner_a0_e1.state_store.snapshot()
    assert snapshot.uavs["uav_2"].failure_state == FailureState.FAILED
    assert not snapshot.uavs["uav_2"].active
