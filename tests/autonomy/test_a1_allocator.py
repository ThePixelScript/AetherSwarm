import pytest
from dataclasses import replace
from types import MappingProxyType

from ares_swarm.autonomy.a1_allocator import A1CommunicationAwareAllocator
from ares_swarm.autonomy.task_allocator import TaskAllocatorConfig, AllocationWeights
from ares_swarm.core.models import UAVState, TaskState, StateSnapshot
from ares_swarm.core.enums import TaskStatus
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.communication.config import CommunicationConfig

@pytest.fixture
def a1_allocator():
    config = TaskAllocatorConfig(weights=AllocationWeights(wP=1.0, wT=1.0, wE=1.0, wC=1.0, wN=1.0, wS=1.0))
    alloc = A1CommunicationAwareAllocator(config=config)
    return alloc

def make_task(t_id, x, y):
    return TaskState(
        id=t_id,
        position_xy=(x, y),
        priority=100.0,
        status=TaskStatus.PENDING,
    )

def make_uav(u_id, x, y):
    return UAVState(
        id=u_id,
        active=True,
        position_xy=(x, y),
    )

def make_snapshot(uavs, tasks):
    return StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({u.id: u for u in uavs}),
        tasks=MappingProxyType({t.id: t for t in tasks}),
        gcs_position=(0.0, 0.0),
    )

def get_network(snap):
    analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=15.0, base_latency=5.0, packet_loss=0.0))
    return analyzer.analyze(snap)

def test_basic_feasible(a1_allocator):
    u1 = make_uav("u1", 5.0, 0.0)
    u2 = make_uav("u2", 10.0, 0.0)
    t = make_task("t1", 0.0, 0.0)
    
    snap = make_snapshot([u1, u2], [t])
    na = get_network(snap)
    
    result = a1_allocator.allocate(snap, network_analysis=na)
    assert result.assignments[0].uav_id == "u1"

def test_critical_node_case(a1_allocator):
    u1 = make_uav("u1", 10.0, 0.0)  
    u2 = make_uav("u2", 20.0, 0.0) 
    u3 = make_uav("u3", 0.0, 10.0)  
    
    t = make_task("t1", 10.0, 1.0) 
    
    snap = make_snapshot([u1, u2, u3], [t])
    na = get_network(snap)
    
    result_a0 = a1_allocator.allocate(snap)
    assert result_a0.assignments[0].uav_id == "u1"
    
    result_a1 = a1_allocator.allocate(snap, network_analysis=na)
    assert result_a1.assignments[0].uav_id == "u2"

def test_disconnected_candidate(a1_allocator):
    u1 = make_uav("u1", 100.0, 0.0) 
    u2 = make_uav("u2", 10.0, 0.0) 
    t = make_task("t1", 100.0, 0.0) 
    
    snap = make_snapshot([u1, u2], [t])
    na = get_network(snap)
    
    result = a1_allocator.allocate(snap, network_analysis=na)
    assert result.assignments[0].uav_id == "u2"

def test_all_disconnected(a1_allocator):
    u1 = make_uav("u1", 100.0, 0.0)
    u2 = make_uav("u2", 120.0, 0.0)
    t = make_task("t1", 100.0, 0.0)
    
    snap = make_snapshot([u1, u2], [t])
    na = get_network(snap)
    
    result = a1_allocator.allocate(snap, network_analysis=na)
    assert result.assignments[0].uav_id == "u1"

def test_equal_utility(a1_allocator):
    u2 = make_uav("u2", 10.0, 0.0)
    u1 = make_uav("u1", 10.0, 0.0)
    t = make_task("t1", 0.0, 0.0)
    
    snap = make_snapshot([u2, u1], [t])
    na = get_network(snap)
    
    result = a1_allocator.allocate(snap, network_analysis=na)
    assert result.assignments[0].uav_id == "u1"

def test_network_partition(a1_allocator):
    u1 = make_uav("u1", 10.0, 0.0)
    u2 = make_uav("u2", 100.0, 0.0)
    t = make_task("t1", 0.0, 0.0)
    
    snap = make_snapshot([u1, u2], [t])
    na = get_network(snap)
    
    result = a1_allocator.allocate(snap, network_analysis=na)
    assert result.assignments[0].uav_id == "u1"

def test_no_network_penalty_case(a1_allocator):
    u1 = make_uav("u1", 10.0, 0.0)
    u2 = make_uav("u2", 15.0, 0.0)
    t = make_task("t1", 0.0, 0.0)
    
    snap = make_snapshot([u1, u2], [t])
    na = get_network(snap)
    
    result_a0 = a1_allocator.allocate(snap)
    result_a1 = a1_allocator.allocate(snap, network_analysis=na)
    
    assert result_a0.assignments[0].uav_id == result_a1.assignments[0].uav_id == "u1"

def test_integration_scenario(a1_allocator):
    u1 = make_uav("u1", 10.0, 0.0)
    u2 = make_uav("u2", 20.0, 0.0)
    t = make_task("t1", 10.0, 0.0) 
    
    snap = make_snapshot([u1, u2], [t])
    na_critical = get_network(snap) 
    
    res_a0 = a1_allocator.allocate(snap)
    assert res_a0.assignments[0].uav_id == "u1"
    
    res_a1_crit = a1_allocator.allocate(snap, network_analysis=na_critical)
    assert res_a1_crit.assignments[0].uav_id == "u2"
    
    analyzer2 = BaselineCommunicationAnalyzer(config=CommunicationConfig(max_range=30.0, base_latency=5.0, packet_loss=0.0))
    na_noncritical = analyzer2.analyze(snap)
    
    res_a1_noncrit = a1_allocator.allocate(snap, network_analysis=na_noncritical)
    assert res_a1_noncrit.assignments[0].uav_id == "u1"
