import pytest
from ares_swarm.core.transitions import *
from ares_swarm.core.enums import *
from ares_swarm.core.models import Vector2D, LinkState
from ares_swarm.core.exceptions import TransitionError, ModelError
from ares_swarm.interfaces.autonomy import ActionProposal

def commit(owned, cls, **kwargs):
    store, key = owned
    return store.commit_transition(cls(timestamp=store.get_snapshot().state.simulation_time,
                                       reason="test", **kwargs), writer_key=key)

def test_movement(owned):
    snap = commit(owned, MoveUAV, uav_id="u1", position=Vector2D(50,60))
    assert snap.state.uavs[0].position == Vector2D(50,60)

def test_task_lifecycle(owned):
    snap = commit(owned, AssignTask, uav_id="u1", task_id="t1")
    assert snap.state.tasks[0].assigned_uav_id == "u1"
    assert snap.state.uavs[0].assigned_task_id == "t1"
    commit(owned, ChangeTaskStatus, task_id="t1", status=TaskStatus.IN_PROGRESS)
    store, key = owned
    store.commit_transition(AdvanceTime(timestamp=10, reason="clock"), writer_key=key)
    snap = commit(owned, ChangeTaskStatus, task_id="t1", status=TaskStatus.COMPLETED)
    assert snap.state.tasks[0].completed_time == 10
    assert snap.state.uavs[0].assigned_task_id is None
    with pytest.raises(TransitionError):
        commit(owned, ChangeTaskStatus, task_id="t1", status=TaskStatus.IN_PROGRESS)

def test_unassignment(owned):
    commit(owned, AssignTask, uav_id="u1", task_id="t1")
    snap = commit(owned, UnassignTask, uav_id="u1")
    assert snap.state.tasks[0].status is TaskStatus.PENDING
    assert snap.state.uavs[0].assigned_task_id is None

@pytest.mark.parametrize("uid,tid", [("missing","t1"), ("u1","missing")])
def test_unknown_assignment(owned, uid, tid):
    with pytest.raises(TransitionError):
        commit(owned, AssignTask, uav_id=uid, task_id=tid)

@pytest.mark.parametrize("pct", [-1, 101])
def test_battery_bounds_atomic(owned, pct):
    before = owned[0].get_snapshot()
    with pytest.raises(TransitionError):
        commit(owned, UpdateBattery, uav_id="u1", battery_pct=pct,
               energy_remaining=50, estimated_rth_energy=5, safety_reserve=10)
    assert owned[0].get_snapshot() == before

def test_battery_valid(owned):
    snap = commit(owned, UpdateBattery, uav_id="u1", battery_pct=50,
                  energy_remaining=50, estimated_rth_energy=5, safety_reserve=10)
    assert snap.state.uavs[0].battery_pct == 50

def test_failure_cleanup_and_movement_rejection(owned):
    commit(owned, AssignTask, uav_id="u1", task_id="t1")
    commit(owned, UpdateConnectivity, uav_id="u2", connected_to_gcs=True,
           route_to_gcs=("u2","u1","gcs"))
    snap = commit(owned, MarkUAVFailed, uav_id="u1")
    assert not snap.state.uavs[0].active
    assert not snap.state.uavs[1].connected_to_gcs
    assert snap.state.tasks[0].status is TaskStatus.PENDING
    with pytest.raises(TransitionError):
        commit(owned, MoveUAV, uav_id="u1", position=Vector2D(1,1))
    with pytest.raises(TransitionError):
        commit(owned, ChangeRole, uav_id="u1", role=UAVRole.SCOUT)

def test_time_rules(owned):
    store, key = owned
    store.commit_transition(AdvanceTime(timestamp=2, reason="clock"), writer_key=key)
    with pytest.raises(TransitionError):
        store.commit_transition(AdvanceTime(timestamp=1, reason="clock"), writer_key=key)
    with pytest.raises(TransitionError):
        store.commit_transition(MoveUAV(timestamp=3, reason="future", uav_id="u1",
                                       position=Vector2D(0,0)), writer_key=key)
    with pytest.raises(ModelError):
        AdvanceTime(timestamp=-1, reason="invalid")

def test_link_and_route(owned):
    snap = commit(owned, UpdateLink, link=LinkState("u1","gcs", distance=10))
    assert len(snap.state.network.links) == 1
    commit(owned, UpdateConnectivity, uav_id="u1", connected_to_gcs=True, route_to_gcs=("u1","gcs"))
    with pytest.raises(TransitionError):
        commit(owned, UpdateLink, link=LinkState("u1","unknown"))
    with pytest.raises(TransitionError):
        commit(owned, UpdateConnectivity, uav_id="u1", connected_to_gcs=True,
               route_to_gcs=("u1","unknown","gcs"))

def test_role_rth_handover(owned):
    commit(owned, ChangeRole, uav_id="u1", role=UAVRole.RELAY)
    commit(owned, UpdateHandoverState, uav_id="u1", handover_state=HandoverState.PENDING)
    commit(owned, SetRTHState, uav_id="u1", rth_state=RTHState.REQUESTED)
    commit(owned, SetRTHState, uav_id="u1", rth_state=RTHState.RETURNING)
    with pytest.raises(TransitionError):
        commit(owned, AssignTask, uav_id="u1", task_id="t1")
    with pytest.raises(TransitionError):
        commit(owned, ChangeRole, uav_id="u1", role=UAVRole.SCOUT)

def test_proposal_is_not_transition(owned):
    store, key = owned
    proposal = ActionProposal("p", 0, "move", "u1", "test", "planner")
    with pytest.raises(TransitionError):
        store.commit_transition(proposal, writer_key=key)

def test_duplicate_assignment(owned):
    commit(owned, AssignTask, uav_id="u1", task_id="t1")
    with pytest.raises(TransitionError):
        commit(owned, AssignTask, uav_id="u2", task_id="t1")

def test_handover_lifecycle(owned):
    with pytest.raises(TransitionError):
        commit(owned, UpdateHandoverState, uav_id="u1", handover_state=HandoverState.COMPLETED)
    for state in (HandoverState.PENDING, HandoverState.PREPARING,
                  HandoverState.VERIFYING, HandoverState.COMPLETED, HandoverState.NONE):
        commit(owned, UpdateHandoverState, uav_id="u1", handover_state=state)

def test_unknown_task_completion(owned):
    with pytest.raises(TransitionError):
        commit(owned, ChangeTaskStatus, task_id="missing", status=TaskStatus.COMPLETED)
