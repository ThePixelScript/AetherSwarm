"""Accumulated endpoint connectivity evidence, not a trajectory-safety proof."""
from dataclasses import replace
from types import MappingProxyType
import pytest
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator, A1AllocatorConfig
from ares_swarm.autonomy.task_allocator import A0TaskAllocator, AllocationWeights
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.communication.config import CommunicationConfig
from ares_swarm.core.models import UAVState, TaskState, StateSnapshot
from ares_swarm.core.enums import Role
from ares_swarm.core.state_store import StateStore


def batch_case(protect_c=True):
    """Active RELAY c participates in Gamma but cannot accept a survey task."""
    nodes = [UAVState(id='a', position_xy=(80., 0.)),
             UAVState(id='b', position_xy=(80., 60.)),
             UAVState(id='c', position_xy=(160., 30.),
                      role=Role.RELAY if protect_c else Role.IDLE)]
    tasks = [TaskState(id='t1', position_xy=(0., -80.), priority=2),
             TaskState(id='t2', position_xy=(0., 80.), priority=1)]
    snapshot = StateSnapshot(simulation_tick=0, simulation_time=0., state_version=0,
                            uavs=MappingProxyType({u.id: u for u in nodes}),
                            tasks=MappingProxyType({t.id: t for t in tasks}),
                            gcs_position=(0., 0.))
    return snapshot, BaselineCommunicationAnalyzer(CommunicationConfig(max_range=100.))


def apply_hypothetical(snapshot, assignments):
    uavs = dict(snapshot.uavs)
    for assignment in assignments:
        uavs[assignment.uav_id] = replace(uavs[assignment.uav_id],
                                         position_xy=snapshot.tasks[assignment.task_id].position_xy)
    return replace(snapshot, uavs=MappingProxyType(uavs))


def test_protected_ineligible_peer_forces_unsafe_second_task_to_remain_unassigned():
    snapshot, analyzer = batch_case()
    store = StateStore(snapshot)
    before = store.snapshot().to_dict()
    net = analyzer.analyze(snapshot)
    allocator = A1TaskAllocator(comm_analyzer=analyzer)
    assert 'c' in net.connected_uav_ids
    for uid, tid in [('a', 't1'), ('b', 't2')]:
        assert allocator.compute_utility(snapshot.uavs[uid], snapshot.tasks[tid], net, snapshot).total != float('-inf')
    result = allocator.allocate(snapshot, network_analysis=net, snapshot=snapshot)
    assert [(a.uav_id, a.task_id) for a in result.assignments] == [('a', 't1')]
    assert result.unassigned_tasks == ('t2',)
    assert 'c' in result.infeasible_uavs
    final = apply_hypothetical(snapshot, result.assignments)
    assert set(net.connected_uav_ids) <= set(analyzer.analyze(final).connected_uav_ids)
    unsafe = replace(final, uavs={**final.uavs, 'b': replace(final.uavs['b'], position_xy=(0., 80.))})
    assert 'c' not in analyzer.analyze(unsafe).connected_uav_ids
    assert store.snapshot().to_dict() == before


def test_safe_alternative_candidate_is_selected_and_final_batch_connected():
    snapshot, analyzer = batch_case(protect_c=False)
    net = analyzer.analyze(snapshot)
    result = A1TaskAllocator(comm_analyzer=analyzer).allocate(snapshot, network_analysis=net)
    assert [(a.uav_id, a.task_id) for a in result.assignments] == [('a', 't1'), ('c', 't2')]
    assert set(net.connected_uav_ids) <= set(analyzer.analyze(apply_hypothetical(snapshot, result.assignments)).connected_uav_ids)


@pytest.mark.parametrize('raise_error', [False, True])
def test_allocation_restores_existing_context_even_on_exception(monkeypatch, raise_error):
    snapshot, analyzer = batch_case()
    allocator = A1TaskAllocator(comm_analyzer=analyzer)
    previous_snap, previous_net = object(), object()
    allocator._current_snapshot, allocator._current_network_analysis = previous_snap, previous_net
    if raise_error:
        def fail(*args, **kwargs):
            raise RuntimeError('injected analyzer failure')
        monkeypatch.setattr(allocator, 'compute_utility', fail)
        with pytest.raises(RuntimeError, match='injected'):
            allocator.allocate(snapshot, network_analysis=object())
    else:
        allocator.allocate(snapshot, network_analysis=analyzer.analyze(snapshot))
    assert allocator._current_snapshot is previous_snap
    assert allocator._current_network_analysis is previous_net


def test_production_destination_aware_a1_defers_where_a0_dispatches_into_blackout():
    nodes = {'a': UAVState(id='a', position_xy=(100., 0.)),
             'b': UAVState(id='b', position_xy=(30., 0.))}
    task = TaskState(id='t', position_xy=(90., 0.), priority=5)
    snapshot = StateSnapshot(simulation_tick=0, simulation_time=0., state_version=0,
                            uavs=nodes, tasks={'t': task}, gcs_position=(0., 0.))
    analyzer = BaselineCommunicationAnalyzer(CommunicationConfig(max_range=50.))
    net = analyzer.analyze(snapshot)
    assert A0TaskAllocator().allocate(snapshot).assignments[0].uav_id == 'a'
    allocator = A1TaskAllocator(comm_analyzer=analyzer)
    assert allocator.config.destination_aware
    result = allocator.allocate(snapshot, network_analysis=net)
    assert result.assignments == ()
    assert result.unassigned_tasks == ('t',)


def test_no_implicit_zero_utility_cutoff_added_to_a0_contract():
    snapshot, analyzer = batch_case()
    config = A1AllocatorConfig(weights=AllocationWeights(wT=1.))
    result = A1TaskAllocator(config, comm_analyzer=analyzer).allocate(snapshot, network_analysis=analyzer.analyze(snapshot))
    assert result.assignments
    assert result.assignments[0].score < 0.


def test_batch_output_is_independent_of_input_insertion_order():
    snapshot, analyzer = batch_case()
    reverse = replace(snapshot, uavs=dict(reversed(list(snapshot.uavs.items()))),
                      tasks=dict(reversed(list(snapshot.tasks.items()))))
    allocator = A1TaskAllocator(comm_analyzer=analyzer)
    assert allocator.allocate(snapshot, network_analysis=analyzer.analyze(snapshot)) == allocator.allocate(reverse, network_analysis=analyzer.analyze(reverse))
