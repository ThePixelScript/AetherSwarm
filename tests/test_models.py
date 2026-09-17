from dataclasses import replace
import math
import pytest
from ares_swarm.core.models import *
from ares_swarm.core.enums import *
from ares_swarm.core.exceptions import ModelError

def test_enums():
    assert {x.value for x in UAVRole} == {"SCOUT", "RELAY", "BACKUP_RELAY", "EMERGENCY_SCOUT", "RETURN_TO_HOME", "IDLE"}
    assert {x.value for x in TaskStatus} == {"PENDING", "ASSIGNED", "IN_PROGRESS", "COMPLETED", "FAILED", "CANCELLED"}

@pytest.mark.parametrize("value", [-1, 101, float("nan"), float("inf"), True, "90"])
def test_bad_battery(value):
    with pytest.raises(ModelError):
        UAVState("u", Vector2D(0, 0), battery_pct=value)

@pytest.mark.parametrize("change", [
    {"uavs": (UAVState("u", Vector2D(0,0)), UAVState("u", Vector2D(1,1)))},
    {"tasks": (TaskState("t", Vector2D(0,0)), TaskState("t", Vector2D(1,1)))},
    {"simulation_time": -1},
    {"uavs": (UAVState("gcs", Vector2D(0,0)),)},
    {"uavs": (UAVState("u1", Vector2D(0,0), assigned_task_id="missing"),)},
    {"network": NetworkState((LinkState("gcs", "missing"),))},
])
def test_invalid_aggregate(initial, change):
    with pytest.raises(ModelError):
        replace(initial, **change)

def test_nonreciprocal_assignment(initial):
    with pytest.raises(ModelError):
        replace(initial, tasks=(replace(initial.tasks[0], status=TaskStatus.ASSIGNED, assigned_uav_id="u1"),))

@pytest.mark.parametrize("route", [("u1","missing","gcs"), ("u1","u1","gcs"), ("u2","gcs")])
def test_invalid_routes(initial, route):
    with pytest.raises(ModelError):
        replace(initial, uavs=(replace(initial.uavs[0], connected_to_gcs=True,
                route_to_gcs=route, parent_relay_id=route[1], hop_count=len(route)-1),))

def test_model_field_types():
    with pytest.raises(ModelError):
        UAVState("u", Vector2D(0,0), role="SCOUT")
    with pytest.raises(ModelError):
        GeoFence(10, 0, 0, 1)
    with pytest.raises(ModelError):
        Vector2D(float("nan"), 1)
    with pytest.raises(ModelError):
        TaskState("t", Vector2D(0,0), status=TaskStatus.COMPLETED)
    with pytest.raises(ModelError):
        LinkState("u", "gcs", estimated_pdr=2)

def test_initial_state(initial):
    assert len(initial.uavs) == 2
    assert initial.simulation_time == 0

def test_rth_and_mission_consistency():
    with pytest.raises(ModelError):
        UAVState("u", Vector2D(0,0), rth_state=RTHState.RETURNING)
    with pytest.raises(ModelError):
        MissionState(ended=True)
