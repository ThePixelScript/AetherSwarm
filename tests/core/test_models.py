import pytest
import dataclasses
from types import MappingProxyType
from ares_swarm.core.models import UAVState, TaskState, StateSnapshot
from ares_swarm.core.enums import Role, TaskStatus, FailureState, RTHState

def test_models_immutability():
    uav = UAVState(id="uav-1")
    with pytest.raises(dataclasses.FrozenInstanceError):
        uav.battery_energy = 50.0

    snapshot = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({"uav-1": uav}),
        tasks=MappingProxyType({}),
    )
    with pytest.raises(TypeError):
        snapshot.uavs["uav-1"] = uav

def test_battery_percent_property():
    uav1 = UAVState(id="uav-1", battery_capacity=100.0, battery_energy=75.0)
    assert uav1.battery_percent == 75.0

    uav_zero = UAVState(id="uav-0", battery_capacity=0.0, battery_energy=0.0)
    assert uav_zero.battery_percent == 0.0

def test_snapshot_to_dict():
    uav = UAVState(id="uav-1", battery_capacity=100.0, battery_energy=50.0)
    snapshot = StateSnapshot(
        simulation_tick=1,
        simulation_time=1.0,
        state_version=1,
        uavs=MappingProxyType({"uav-1": uav}),
        tasks=MappingProxyType({}),
    )
    data = snapshot.to_dict()
    assert isinstance(data, dict)
    assert data["uavs"]["uav-1"]["battery_percent"] == 50.0
