"""Actual motion, planning emission and dependent-release regressions."""
from dataclasses import replace
import math
import pytest
from ares_swarm.autonomy.elastic_tether import ElasticChainTether
from ares_swarm.autonomy.connectivity_planner import ConnectivityAwarePlanner, ConnectivityAwarePlannerConfig
from ares_swarm.autonomy.relay_manager import DynamicRelayManager, RelayChain, ChainStatus
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.communication.config import CommunicationConfig
from ares_swarm.core.models import StateSnapshot, UAVState, TaskState
from ares_swarm.core.enums import Role, RTHState, TaskStatus, SortieState
from ares_swarm.core.commands import AssignTaskCommand, SetTargetPositionCommand, ReleaseRelayRoleCommand
from ares_swarm.core.state_store import StateStore
from ares_swarm.core.simulator import SimulationEngine
from ares_swarm.safety.separation import SeparationEnforcer


def fixture(positions=((25., 500.), (55., 500.), (85., 500.))):
    nodes = {uid: UAVState(id=uid, position_xy=p, target_position=(p[0]+50, p[1]),
              role=Role.RELAY if uid != 's' else Role.SURVEYOR, sortie_state=SortieState.ACTIVE)
             for uid, p in zip(('r1', 'r2', 's'), positions)}
    snap = StateSnapshot(simulation_tick=0, simulation_time=0., state_version=0,
                         uavs=nodes, tasks={}, gcs_position=(0., 500.))
    rm = DynamicRelayManager()
    rm.register_chain(RelayChain('c','t','s',['r1','r2'],[(70.,500.),(140.,500.)],0,0.,ChainStatus.FORMING))
    analyzer = BaselineCommunicationAnalyzer(CommunicationConfig(max_range=100.))
    return snap, rm, ElasticChainTether(rm, analyzer)


def move(snapshot, guard, safety=True):
    store = StateStore(snapshot)
    SimulationEngine(store).step_swarm(motion_guard=guard,
        separation_enforcer=SeparationEnforcer(gcs_position=snapshot.gcs_position) if safety else None)
    return store.snapshot()


def test_all_members_move_during_forming_without_mutating_prediction_input():
    snap, rm, guard = fixture()
    before = snap.to_dict()
    copied = guard.prepare_snapshot(snap, 5., 1.)
    assert snap.to_dict() == before
    final = move(snap, guard)
    assert all(final.uavs[u].position_xy != snap.uavs[u].position_xy for u in snap.uavs)
    assert rm.chains['c'].status == ChainStatus.FORMING
    assert guard.gamma_checks > 0


@pytest.mark.parametrize('downstream,upstream,positions', [
    ('r2','r1',((0.,500.),(80.,500.),(110.,500.))),
    ('s','r2',((0.,500.),(30.,500.),(110.,500.))),
])
def test_every_downstream_holds_then_resumes(downstream, upstream, positions):
    snap, _, guard = fixture(positions)
    nodes = dict(snap.uavs)
    nodes[upstream] = replace(nodes[upstream], target_position=None)
    snap = replace(snap, uavs=nodes)
    final = move(snap, guard)
    assert math.dist(final.uavs[downstream].position_xy, final.uavs[upstream].position_xy) <= 80.+1e-8
    assert math.dist(final.uavs[downstream].position_xy, snap.uavs[downstream].position_xy) < 1e-7
    nodes = dict(final.uavs)
    nodes[upstream] = replace(nodes[upstream], position_xy=(nodes[upstream].position_xy[0]+5,500.))
    advanced = replace(final, uavs=nodes)
    assert move(advanced, guard).uavs[downstream].position_xy[0] > nodes[downstream].position_xy[0]


def test_safety_changed_upstream_still_cannot_break_tether():
    snap, _, guard = fixture(((0.,500.),(80.,500.),(110.,500.)))
    from ares_swarm.core.commands import StepPhysicsCommand
    commands = [StepPhysicsCommand(source_tick=0,uav_id=uid,
        new_position_xy=(u.position_xy[0]+(5 if uid=='r2' else 0),500.),
        new_velocity_xy=(5. if uid=='r2' else 0.,0.), delta_energy=1.) for uid,u in snap.uavs.items()]
    filtered = guard.filter_commands(snap, commands, 1., 1., .5)
    assert next(c for c in filtered if c.uav_id=='r2').new_position_xy[0] <= 80.+1e-8


def test_rth_remains_under_safety_not_experimental_tether():
    snap, _, guard = fixture()
    nodes = dict(snap.uavs)
    nodes['s'] = replace(nodes['s'], rth_state=RTHState.ACTIVE, target_position=(0.,500.))
    snap = replace(snap,uavs=nodes)
    assert guard.prepare_snapshot(snap,5.,1.).uavs['s'] == snap.uavs['s']


def planner_case(task_x, count):
    snap = StateSnapshot(simulation_tick=0, simulation_time=0., state_version=0,
        gcs_position=(0.,500.), uavs={f'u{i}': UAVState(id=f'u{i}', position_xy=(0.,500.+25*i)) for i in range(count)},
        tasks={'t': TaskState(id='t',position_xy=(task_x,500.),priority=1,service_duration=10.)})
    planner = ConnectivityAwarePlanner(config=ConnectivityAwarePlannerConfig(elastic_tether_enabled=True))
    return snap, planner


@pytest.mark.parametrize('distance,count,relays',[(140.,2,1),(210.,3,2)])
def test_complete_team_emitted_same_tick_and_surveryor_not_held(distance,count,relays):
    snap, planner = planner_case(distance,count)
    commands = planner.plan(snap)
    assert len([c for c in commands if isinstance(c,AssignTaskCommand)]) == 1
    assert len(planner.relay_manager.chains['chain_t'].relay_ids) == relays
    assert {c.source_tick for c in commands} == {0}
    store = StateStore(snap)
    assert not store.apply(commands).rejected_commands
    current = store.snapshot()
    sid = planner.relay_manager.chains['chain_t'].surveyor_id
    monitors = planner.monitor_active_tasks(current)
    assert not any(isinstance(c,SetTargetPositionCommand) and c.uav_id==sid
                   and c.target_position != snap.tasks['t'].position_xy for c in monitors)


def test_incomplete_or_locked_team_emits_nothing():
    snap, planner = planner_case(210.,2)
    assert planner.plan(snap) == []
    snap, planner = planner_case(140.,2)
    snap = replace(snap,uavs={k:replace(v,role_lock_until=10.) for k,v in snap.uavs.items()})
    assert planner.plan(snap) == []
    assert not planner.relay_manager.chains


def test_two_disjoint_teams_same_tick_and_no_stolen_active_worker():
    snap, planner = planner_case(140.,5)
    tasks = dict(snap.tasks, t2=TaskState(id='t2',position_xy=(140.,550.),priority=1,service_duration=10.))
    snap = replace(snap,tasks=tasks)
    commands = planner.plan(snap)
    assert len([c for c in commands if isinstance(c,AssignTaskCommand)]) == 2
    teams = [set([c.surveyor_id,*c.relay_ids]) for c in planner.relay_manager.chains.values()]
    assert not teams[0] & teams[1]
    store = StateStore(snap); store.apply(commands)
    assert not planner.plan(store.snapshot())


def test_pending_report_retains_chain_then_dependency_release():
    snap, rm, _ = fixture()
    assert not any(isinstance(c,ReleaseRelayRoleCommand) for c in rm.step(snap,pending_report_task_ids={'t'}))
    released = rm.step(snap)
    assert {c.uav_id for c in released if isinstance(c,ReleaseRelayRoleCommand)} >= {'r1','r2'}


def test_repeat_motion_exact_and_separation_preserved():
    snap, _, guard = fixture()
    a = move(snap,guard)
    assert a == move(snap,guard)
    assert min(math.dist(a.uavs[x].position_xy,a.uavs[y].position_xy)
               for x,y in [('r1','r2'),('r2','s'),('r1','s')]) >= 20.
