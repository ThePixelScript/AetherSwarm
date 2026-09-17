from dataclasses import FrozenInstanceError, replace
import pytest
from ares_swarm.core.models import SwarmState, GCSState, Vector2D
from ares_swarm.core.state_store import StateStore
from ares_swarm.core.transitions import AdvanceTime, MoveUAV
from ares_swarm.core.exceptions import AuthorizationError, TransitionError

def test_snapshot_deep_immutability(owned):
    store, key = owned
    snap = store.get_snapshot()
    with pytest.raises(FrozenInstanceError):
        snap.state.uavs[0].battery_pct = 0
    with pytest.raises(TypeError):
        snap.state.metadata["nested"]["items"][0] = 99
    with pytest.raises(TypeError):
        snap.state.metadata["new"] = 1
    with pytest.raises(FrozenInstanceError):
        snap.state.mission.ended = True
    with pytest.raises(FrozenInstanceError):
        snap.state.network.links = ()
    store.commit_transition(AdvanceTime(timestamp=1, reason="tick"), writer_key=key)
    assert snap.state.simulation_time == 0
    assert store.get_snapshot().state.simulation_time == 1

def test_input_metadata_is_detached():
    metadata = {"nested": [1]}
    state = SwarmState(GCSState("gcs", Vector2D(0,0)), metadata=metadata)
    metadata["nested"].append(9)
    assert state.metadata["nested"] == (1,)

def test_writer_capability(owned):
    store, _ = owned
    with pytest.raises(AuthorizationError):
        store.commit_transition(AdvanceTime(timestamp=1, reason="tick"), writer_key=object())
    with pytest.raises(AuthorizationError):
        store.reset(writer_key=object())
    assert store.get_snapshot().revision == 0

def test_validate_does_not_commit(owned):
    store, _ = owned
    store.validate_transition(AdvanceTime(timestamp=1, reason="tick"))
    assert store.get_snapshot().revision == 0

def test_reset(owned, initial):
    store, key = owned
    store.commit_transition(AdvanceTime(timestamp=5, reason="tick"), writer_key=key)
    snap = store.reset(writer_key=key)
    assert snap.state == initial
    assert snap.revision == 2
    other = replace(initial, simulation_time=2)
    assert store.reset(other, writer_key=key).state == other
    assert store.reset(writer_key=key).state == initial

def test_atomic_rejection(owned):
    store, key = owned
    before = store.get_snapshot()
    with pytest.raises(TransitionError):
        store.commit_transition(MoveUAV(timestamp=0, reason="bad", uav_id="missing",
                                       position=Vector2D(0,0)), writer_key=key)
    assert store.get_snapshot() == before

def test_stale_revision(owned):
    store, key = owned
    transition = AdvanceTime(timestamp=1, reason="tick", expected_revision=0)
    store.commit_transition(transition, writer_key=key)
    with pytest.raises(TransitionError, match="stale"):
        store.commit_transition(transition, writer_key=key)
