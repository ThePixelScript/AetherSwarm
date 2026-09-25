import pytest
from ares_swarm.simulation.scenario import load_scenario
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.autonomy.task_allocator import A0TaskAllocator
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.communication.scenario import CommunicationCondition


def test_a1_preserves_telemetry_under_controlled_outage():
    base_scenario = load_scenario("scenarios/e2_controlled.yaml")

    conds = [
        CommunicationCondition(source_id="uav_1", target_id="uav_3", start_time_s=150.0, end_time_s=250.0, quality_multiplier=0.2),
        CommunicationCondition(source_id="uav_1", target_id="uav_3", start_time_s=250.0, end_time_s=350.0, outage=True),
        CommunicationCondition(source_id="uav_2", target_id="uav_3", start_time_s=150.0, end_time_s=250.0, quality_multiplier=0.2),
        CommunicationCondition(source_id="uav_2", target_id="uav_3", start_time_s=250.0, end_time_s=350.0, outage=True),
        CommunicationCondition(source_id="uav_3", target_id="uav_4", start_time_s=0.0, end_time_s=400.0, outage=True),
    ]

    # --- Run A0 ---
    a0_analyzer = BaselineCommunicationAnalyzer(config=base_scenario.communication, scenario_conditions=tuple(conds))
    a0_runner = MissionRunner(
        scenario=base_scenario,
        seed=2026,
        autonomy_adapter=A0AutonomyAdapter(allocator=A0TaskAllocator()),
        comm_analyzer=a0_analyzer,
    )
    a0_res = a0_runner.run()
    a0_dict = a0_res.to_dict()

    # A0 assigns uav_3, telemetry fails
    assert a0_dict["telemetry"]["reports_delivered"] == 0
    assert a0_dict["telemetry"]["reports_deadline_exceeded"] == 1
    assert "uav_3" in a0_dict["telemetry"]["per_uav_detection_counts"]

    # --- Run A1 ---
    a1_analyzer = BaselineCommunicationAnalyzer(config=base_scenario.communication, scenario_conditions=tuple(conds))
    a1_runner = MissionRunner(
        scenario=base_scenario,
        seed=2026,
        autonomy_adapter=A0AutonomyAdapter(allocator=A1TaskAllocator(comm_analyzer=a1_analyzer)),
        comm_analyzer=a1_analyzer,
    )
    a1_res = a1_runner.run()
    a1_dict = a1_res.to_dict()

    # A1 assigns uav_4, telemetry succeeds
    assert a1_dict["telemetry"]["reports_delivered"] == 1
    assert a1_dict["telemetry"]["reports_deadline_exceeded"] == 0
    assert "uav_4" in a1_dict["telemetry"]["per_uav_detection_counts"]
