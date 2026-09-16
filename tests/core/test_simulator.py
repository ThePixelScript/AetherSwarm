from src.ares_swarm.core.simulator import SimulationClock
def test_clock_starts_at_zero():
    clock = SimulationClock()
    assert clock.time == 0.0
def test_clock_advances():
    clock = SimulationClock()
    assert clock.advance() == 1.0
    assert clock.advance() == 2.0
def test_clock_rejects_invalid_dt():
    try:
        SimulationClock(0)
        assert False
    except ValueError:
        assert True