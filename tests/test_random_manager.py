import numpy as np
import pytest
from ares_swarm.core.random_manager import RandomManager
from ares_swarm.core.exceptions import ModelError

@pytest.mark.parametrize("stream", ["simulation","communication","events"])
def test_repeatable(stream):
    a,b = RandomManager(42),RandomManager(42)
    assert np.array_equal(getattr(a,stream).random(20), getattr(b,stream).random(20))
    assert not np.array_equal(getattr(RandomManager(42),stream).random(20),
                              getattr(RandomManager(43),stream).random(20))

def test_stream_independence():
    a,b = RandomManager(42),RandomManager(42)
    a.simulation.random(100)
    assert np.array_equal(a.communication.random(10), b.communication.random(10))
    assert not np.array_equal(RandomManager(42).simulation.random(10),
                              RandomManager(42).events.random(10))

@pytest.mark.parametrize("seed", [-1, True, "42", 1.5])
def test_invalid_seed(seed):
    with pytest.raises(ModelError):
        RandomManager(seed)
