import pytest
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.communication.config import CommunicationConfig
from ares_swarm.communication.scenario import CommunicationCondition
from ares_swarm.core.models import StateSnapshot, UAVState


def test_a1_analyzer_injection_does_not_infer():
    # Setup conditions matching E2
    config = CommunicationConfig(max_range=100.0)
    conds = [
        CommunicationCondition(source_id="uav_1", target_id="uav_2", start_time_s=150.0, end_time_s=250.0, quality_multiplier=0.3),
        CommunicationCondition(source_id="uav_1", target_id="uav_2", start_time_s=250.0, end_time_s=350.0, outage=True),
    ]

    analyzer = BaselineCommunicationAnalyzer(config=config, scenario_conditions=tuple(conds))

    allocator = A1TaskAllocator(comm_analyzer=analyzer)

    # 1. Normal state (t=100)
    snap_normal = StateSnapshot(
        state_version=1,
        simulation_tick=100,
        simulation_time=100.0,
        gcs_position=(0.0, 0.0),
        uavs={"uav_1": UAVState(id="uav_1", position_xy=(50.0, 0.0)), "uav_2": UAVState(id="uav_2", position_xy=(90.0, 0.0))},
        tasks={},
    )
    net_normal = analyzer.analyze(snap_normal)
    extracted_analyzer = allocator._get_comm_analyzer(net_normal)
    # Validate it correctly retrieved exact configuration
    assert extracted_analyzer.config.max_range == 100.0

    # 2. Degradation (t=200)
    snap_degrade = StateSnapshot(
        state_version=2,
        simulation_tick=200,
        simulation_time=200.0,
        gcs_position=(0.0, 0.0),
        uavs={"uav_1": UAVState(id="uav_1", position_xy=(50.0, 0.0)), "uav_2": UAVState(id="uav_2", position_xy=(90.0, 0.0))},
        tasks={},
    )
    net_degrade = analyzer.analyze(snap_degrade)
    extracted_degrade = allocator._get_comm_analyzer(net_degrade)
    assert extracted_degrade.config.max_range == 100.0
    assert len(extracted_degrade.scenario_conditions) == 2

    # 3. Outage (t=300)
    snap_outage = StateSnapshot(
        state_version=3,
        simulation_tick=300,
        simulation_time=300.0,
        gcs_position=(0.0, 0.0),
        uavs={"uav_1": UAVState(id="uav_1", position_xy=(50.0, 0.0)), "uav_2": UAVState(id="uav_2", position_xy=(90.0, 0.0))},
        tasks={},
    )
    net_outage = analyzer.analyze(snap_outage)
    extracted_outage = allocator._get_comm_analyzer(net_outage)
    assert extracted_outage.config.max_range == 100.0
    assert len(extracted_outage.scenario_conditions) == 2
