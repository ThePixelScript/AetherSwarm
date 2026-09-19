import pytest
from pathlib import Path
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import load_scenario
from ares_swarm.autonomy.task_allocator import A0TaskAllocator
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter

def get_mission_runner(scenario_name: str, a1: bool) -> MissionRunner:
    allocator = A1TaskAllocator() if a1 else A0TaskAllocator()
    adapter = A0AutonomyAdapter(allocator=allocator)
    
    # Load base scenario
    scenario = load_scenario(Path("scenarios/poc_round1.yaml"))
    return MissionRunner(scenario=scenario, seed=42, autonomy_adapter=adapter)

def test_demo_scenarios():
    runner_a0 = get_mission_runner("E0", a1=False)
    runner_a1 = get_mission_runner("E0", a1=True)
    assert runner_a0 is not None
    assert runner_a1 is not None

