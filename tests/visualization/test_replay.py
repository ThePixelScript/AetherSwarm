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