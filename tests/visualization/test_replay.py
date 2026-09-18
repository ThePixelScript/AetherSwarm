from ares_swarm.core.models import StateSnapshot
from ares_swarm.visualization.replay import ReplayRecorder


def test_replay_records_snapshots():
    recorder = ReplayRecorder()
    snapshot = StateSnapshot(
        simulation_tick=1,
        simulation_time=1.0,
        state_version=1,
    )

    recorder.record(snapshot)

    assert recorder.latest() == snapshot
    assert len(recorder.snapshots) == 1


def test_replay_clear():
    recorder = ReplayRecorder()
    snapshot = StateSnapshot(
        simulation_tick=1,
        simulation_time=1.0,
        state_version=1,
    )

    recorder.record(snapshot)
    recorder.clear()

    assert recorder.latest() is None
    assert recorder.snapshots == []
def test_replay_returns_snapshots_in_recording_order():
    recorder = ReplayRecorder()

    snapshot1 = StateSnapshot(
        simulation_tick=1,
        simulation_time=1.0,
        state_version=1,
    )
    snapshot2 = StateSnapshot(
        simulation_tick=2,
        simulation_time=2.0,
        state_version=2,
    )

    recorder.record(snapshot1)
    recorder.record(snapshot2)

    assert recorder.replay() == (snapshot1, snapshot2)