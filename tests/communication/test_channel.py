from dataclasses import replace
import math
import pytest
from ares_swarm.communication.channel import ChannelModel, LinkCondition
from ares_swarm.core.config import CommunicationConfig
from ares_swarm.core.enums import FailureStatus
from ares_swarm.core.models import GCSState, UAVState, Vector2D
from ares_swarm.core.exceptions import ModelError

def pair(distance):
    return GCSState("gcs", Vector2D(0,0)), UAVState("u", Vector2D(distance,0))

@pytest.mark.parametrize("distance,quality", [(0,1), (5,2/3), (10,0.5)])
def test_quality_and_boundary(channel, distance, quality):
    result = channel.evaluate(*pair(distance), simulation_time=3)
    assert result.range_feasible and result.usable
    assert result.distance == distance
    assert result.link.link_quality == pytest.approx(quality)
    assert result.estimated_pdr == 1 - result.packet_loss_probability
    assert result.etx == pytest.approx(1/result.estimated_pdr)
    assert result.link.latency_ms == 5*(1+distance/10)
    assert result.link.last_updated == 3
    assert result.link.rssi is None and result.link.snr is None

def test_outside_range(channel):
    result = channel.evaluate(*pair(math.nextafter(10, math.inf)), simulation_time=0)
    assert not result.range_feasible and not result.usable
    assert result.etx is None and result.estimated_pdr == 0

@pytest.mark.parametrize("status", [FailureStatus.HEALTHY, FailureStatus.FAILED, FailureStatus.LOST])
def test_inactive(channel, status):
    gcs,u = pair(1)
    result = channel.evaluate(gcs, replace(u, active=False, failure_status=status), simulation_time=0)
    assert not result.usable and result.unavailable_reason == "inactive_endpoint"

def test_degraded_node_has_no_implicit_penalty(channel):
    gcs,u = pair(1)
    result = channel.evaluate(gcs, replace(u, failure_status=FailureStatus.DEGRADED), simulation_time=0)
    assert result == channel.evaluate(gcs,u,simulation_time=0)

@pytest.mark.parametrize("base_loss", [0, 0.3, math.nextafter(1,0), 1])
def test_pdr_boundaries(base_loss):
    model = ChannelModel(CommunicationConfig(10, 5, base_loss))
    result = model.evaluate(*pair(0), simulation_time=0)
    assert result.estimated_pdr == 1-base_loss
    if base_loss == 1:
        assert not result.usable and result.etx is None
        assert result.unavailable_reason == "zero_pdr"
    else:
        assert result.etx == 1/(1-base_loss)

def test_degradation_and_outage(channel):
    clean = channel.evaluate(*pair(5), simulation_time=0)
    degraded = channel.evaluate(*pair(5), simulation_time=0,
                                condition=LinkCondition(0.5, 7))
    assert degraded.link.link_quality == clean.link.link_quality/2
    assert degraded.estimated_pdr == pytest.approx(clean.estimated_pdr/2)
    assert degraded.link.latency_ms == clean.link.latency_ms + 7
    assert not channel.evaluate(*pair(5), simulation_time=0,
                                condition=LinkCondition(outage=True)).usable
    assert not channel.evaluate(*pair(5), simulation_time=0,
                                condition=LinkCondition(quality_multiplier=0)).usable

def test_monotonic_and_repeatable(channel):
    results = [channel.evaluate(*pair(d), simulation_time=0) for d in range(11)]
    qualities = [r.link.link_quality for r in results]
    assert all(a>b for a,b in zip(qualities, qualities[1:]))
    assert results == [channel.evaluate(*pair(d), simulation_time=0) for d in range(11)]
    assert channel.evaluate(*pair(5), simulation_time=0) == channel.evaluate(
        *reversed(pair(5)), simulation_time=0)

@pytest.mark.parametrize("kwargs", [{"quality_multiplier":-1}, {"quality_multiplier":1.1},
                                  {"latency_penalty_ms":-1}, {"outage":"yes"}])
def test_invalid_conditions(kwargs):
    with pytest.raises(ModelError):
        LinkCondition(**kwargs)

@pytest.mark.parametrize("time", [-1, math.nan, math.inf, True])
def test_invalid_time(channel, time):
    with pytest.raises(ModelError):
        channel.evaluate(*pair(1), simulation_time=time)

def test_self_link(channel):
    gcs,_ = pair(0)
    with pytest.raises(ModelError):
        channel.evaluate(gcs,gcs,simulation_time=0)
