import pytest
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.communication.config import CommunicationConfig
from ares_swarm.core.models import UAVState, TaskState, StateSnapshot
from ares_swarm.core.enums import Role, TaskStatus

def test_exact_topology_breaking_two_assignment_counterexample():
    uav_a = UAVState(id='a', position_xy=(80.0, 0.0), active=True, role=Role.IDLE)
    uav_b = UAVState(id='b', position_xy=(80.0, 60.0), active=True, role=Role.IDLE)
    uav_c = UAVState(id='c', position_xy=(160.0, 30.0), active=True, role=Role.IDLE)

    t1 = TaskState(id='t1', position_xy=(0.0, -80.0), status=TaskStatus.PENDING, priority=2.0)
    t2 = TaskState(id='t2', position_xy=(0.0, 80.0), status=TaskStatus.PENDING, priority=1.0)

    snap = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs={'a': uav_a, 'b': uav_b, 'c': uav_c},
        tasks={'t1': t1, 't2': t2},
        gcs_position=(0.0, 0.0)
    )

    analyzer = BaselineCommunicationAnalyzer(CommunicationConfig(max_range=100.0))
    net = analyzer.analyze(snap)

    allocator = A1TaskAllocator()
    allocator._comm_analyzer = analyzer
    res = allocator.allocate(snap, network_analysis=net, snapshot=snap)
    
    # Assert 'a' gets t1, 'c' gets t2. 
    # 'b' MUST NOT be assigned to 't2' because it would disconnect 'c'.
    assert len(res.assignments) == 2
    assignments_dict = {a.task_id: a.uav_id for a in res.assignments}
    assert assignments_dict['t1'] == 'a'
    assert assignments_dict['t2'] == 'c'
