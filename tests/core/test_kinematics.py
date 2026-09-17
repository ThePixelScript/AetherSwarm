from ares_swarm.core.commands import StepPhysicsCommand
from ares_swarm.core.kinematics import move_towards
from ares_swarm.core.state_store import StateStore

def test_move_towards_moves_in_straight_line():
    position, velocity = move_towards(
        current=(0.0, 0.0),
        target=(10.0, 0.0),
        speed=5.0,
        dt=1.0,
    )

    assert position == (5.0, 0.0)
    assert velocity == (5.0, 0.0)


def test_move_towards_stops_at_target():
    position, velocity = move_towards(
        current=(8.0, 0.0),
        target=(10.0, 0.0),
        speed=5.0,
        dt=1.0,
    )

    assert position == (10.0, 0.0)
    assert velocity == (0.0, 0.0)


def test_move_towards_when_already_at_target():
    position, velocity = move_towards(
        current=(10.0, 10.0),
        target=(10.0, 10.0),
        speed=5.0,
        dt=1.0,
    )

    assert position == (10.0, 10.0)
    assert velocity == (0.0, 0.0)


def test_move_towards_rejects_invalid_speed():
    try:
        move_towards((0.0, 0.0), (10.0, 0.0), -1.0, 1.0)
        assert False
    except ValueError:
        assert True