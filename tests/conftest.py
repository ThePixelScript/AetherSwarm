from pathlib import Path
import pytest
from ares_swarm.core.models import SwarmState, GCSState, UAVState, TaskState, Vector2D
from ares_swarm.core.state_store import StateStore

@pytest.fixture
def initial():
    return SwarmState(gcs=GCSState("gcs", Vector2D(0, 0)),
                      uavs=(UAVState("u1", Vector2D(10, 0)), UAVState("u2", Vector2D(20, 0))),
                      tasks=(TaskState("t1", Vector2D(100, 100)),),
                      metadata={"nested": {"items": [1, 2]}})

@pytest.fixture
def owned(initial):
    key = object()
    return StateStore(initial, writer_key=key), key

@pytest.fixture
def root():
    return Path(__file__).resolve().parents[1]
