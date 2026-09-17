"""Independent named NumPy streams; never uses the global RNG."""
import numpy as np
from .exceptions import ModelError

class RandomManager:
    """Owned by the future engine; fixed stream IDs avoid order-dependent seeding."""
    def __init__(self, seed: int) -> None:
        if type(seed) is not int or seed < 0:
            raise ModelError("seed must be a nonnegative integer")
        self.seed = seed
        self.simulation = np.random.default_rng(np.random.SeedSequence(seed, spawn_key=(0,)))
        self.communication = np.random.default_rng(np.random.SeedSequence(seed, spawn_key=(1,)))
        self.events = np.random.default_rng(np.random.SeedSequence(seed, spawn_key=(2,)))
