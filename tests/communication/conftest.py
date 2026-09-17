"""Tiny deterministic snapshot fixtures, using the existing core models."""
from dataclasses import replace
import pytest
from ares_swarm.core.models import GCSState, UAVState, SwarmState, Vector2D
from ares_swarm.core.snapshot import StateSnapshot
from ares_swarm.core.config import CommunicationConfig
from ares_swarm.communication.channel import ChannelModel

@pytest.fixture
def channel():
    return ChannelModel(CommunicationConfig(max_range=10.0, base_latency=5.0, packet_loss=0.0))

@pytest.fixture
def snapshot_factory():
    def make(positions=(), *, failed=(), inactive=(), reverse=False):
        from ares_swarm.core.enums import FailureStatus
        uavs = tuple(UAVState(uid, Vector2D(x, y)) for uid, x, y in positions)
        uavs = tuple(replace(u, active=False, failure_status=FailureStatus.FAILED)
                     if u.id in failed else replace(u, active=False)
                     if u.id in inactive else u for u in uavs)
        if reverse:
            uavs = tuple(reversed(uavs))
        return StateSnapshot(SwarmState(
            gcs=GCSState("gcs", Vector2D(0.0, 0.0)), uavs=uavs), revision=0)
    return make
