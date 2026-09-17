"""Tiny deterministic snapshot fixtures, using the new alpha core models."""
from dataclasses import replace, dataclass
import pytest
from ares_swarm.core.models import UAVState, StateSnapshot
from ares_swarm.core.enums import FailureState

@dataclass
class CommunicationConfig:
    max_range: float = 10.0
    base_latency: float = 5.0
    packet_loss: float = 0.0

@pytest.fixture
def channel():
    from ares_swarm.communication.channel import ChannelModel
    return ChannelModel(CommunicationConfig(max_range=10.0, base_latency=5.0, packet_loss=0.0))

@pytest.fixture
def snapshot_factory():
    def make(positions=(), *, failed=(), inactive=(), reverse=False):
        uavs_dict = {}
        for uid, x, y in positions:
            uav = UAVState(id=uid, position_xy=(float(x), float(y)))
            if uid in failed:
                uav = replace(uav, active=False, failure_state=FailureState.FAILED)
            elif uid in inactive:
                uav = replace(uav, active=False)
            uavs_dict[uid] = uav
            
        if reverse:
            uavs_dict = {k: uavs_dict[k] for k in reversed(list(uavs_dict.keys()))}
            
        return StateSnapshot(
            simulation_tick=0,
            simulation_time=0.0,
            state_version=0,
            uavs=uavs_dict,
            gcs_position=(0.0, 0.0)
        )
    return make
