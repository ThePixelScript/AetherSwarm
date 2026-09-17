import json
import pytest
from ares_swarm.core.serialization import dumps, loads
from ares_swarm.core.transitions import *
from ares_swarm.core.events import SimulationEvent
from ares_swarm.core.enums import *
from ares_swarm.core.models import Vector2D, LinkState
from ares_swarm.core.config import load_config, load_scenario
from ares_swarm.core.exceptions import SerializationError

def test_state_snapshot_roundtrip(initial, owned):
    for value in (initial, owned[0].get_snapshot()):
        decoded = loads(dumps(value))
        assert decoded == value
        assert dumps(decoded) == dumps(value)

@pytest.mark.parametrize("cls,kwargs", [
    (AdvanceTime, {}),
    (MoveUAV, {"uav_id":"u1", "position":Vector2D(1,2)}),
    (ChangeRole, {"uav_id":"u1", "role":UAVRole.RELAY}),
    (AssignTask, {"uav_id":"u1", "task_id":"t1"}),
    (UnassignTask, {"uav_id":"u1"}),
    (UpdateBattery, {"uav_id":"u1", "battery_pct":50, "energy_remaining":50,
                     "estimated_rth_energy":2, "safety_reserve":5}),
    (UpdateConnectivity, {"uav_id":"u1", "connected_to_gcs":True, "route_to_gcs":("u1","gcs")}),
    (UpdateLink, {"link":LinkState("u1","gcs")}),
    (ChangeTaskStatus, {"task_id":"t1","status":TaskStatus.IN_PROGRESS}),
    (MarkUAVFailed, {"uav_id":"u1"}),
    (SetRTHState, {"uav_id":"u1", "rth_state":RTHState.REQUESTED}),
    (UpdateHandoverState, {"uav_id":"u1", "handover_state":HandoverState.PENDING}),
])
def test_transition_roundtrip(cls, kwargs):
    value = cls(timestamp=0, reason="roundtrip", **kwargs)
    assert loads(dumps(value)) == value

def test_config_event_roundtrip(root):
    cfg = load_config(root/"configs/default.yaml")
    scenario = load_scenario(root/"scenarios/basic.yaml", cfg)
    event = SimulationEvent("e", 1, EventType.UAV_FAILURE, payload={"nested":[1,2]})
    for value in (cfg, scenario, event):
        assert loads(dumps(value)) == value

@pytest.mark.parametrize("text", [
    '{}', '{"schema_version":2,"type":"SwarmState","data":{}}',
    '{"schema_version":1,"type":"os.system","data":{}}',
    '{"schema_version":1,"type":"Vector2D","data":{"x":NaN,"y":0}}',
    '{"schema_version":1,"type":"Vector2D","data":{"x":0,"y":0,"z":1}}',
    '{"schema_version":1,"schema_version":1,"type":"Vector2D","data":{"x":0,"y":0}}',
])
def test_untrusted_json(text):
    with pytest.raises(SerializationError):
        loads(text)
