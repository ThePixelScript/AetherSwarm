from dataclasses import FrozenInstanceError, replace
import json
import os
import subprocess
import sys
from pathlib import Path
import pytest

from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.communication.channel import LinkCondition
from ares_swarm.core.config import CommunicationConfig, load_config, load_scenario
from ares_swarm.core.enums import UAVRole
from ares_swarm.core.models import NetworkState, LinkState, SwarmState, GCSState, Vector2D
from ares_swarm.core.state_store import StateStore
from ares_swarm.core.transitions import MarkUAVFailed, AdvanceTime
from ares_swarm.core.serialization import dumps, loads
from ares_swarm.core.exceptions import ModelError
from ares_swarm.interfaces.communication import NetworkAnalysis, CommunicationAnalyzer

def analyzer():
    return BaselineCommunicationAnalyzer(CommunicationConfig(10,5,0))

def test_chain_then_failed_relay(snapshot_factory):
    snapshot=snapshot_factory((("u1",10,0),("u2",20,0)))
    key=object()
    store=StateStore(snapshot.state,writer_key=key)
    service: CommunicationAnalyzer=analyzer()
    before=store.get_snapshot()
    encoded=dumps(before)
    result=service.analyze(before)
    assert result.reachable_uav_ids == ("u1","u2")
    assert result.routes_to_gcs["u2"] == ("u2","u1","gcs")
    assert result.hop_counts["u2"] == 2
    assert result.articulation_points == ("u1",)
    assert result.network_health["average_hop_count"] == 1.5
    assert dumps(store.get_snapshot()) == encoded
    # The test harness commits a core transition; communication receives no writer.
    store.commit_transition(MarkUAVFailed(timestamp=0,reason="fixture failure",uav_id="u1"),
                            writer_key=key)
    after=store.get_snapshot()
    encoded_after=dumps(after)
    failed_result=service.analyze(after)
    assert failed_result.snapshot_revision == after.revision
    assert failed_result.disconnected_uav_ids == ("u2",)
    assert failed_result.routes_to_gcs == {"u2":None}
    assert failed_result.hop_counts == {"u2":None}
    assert failed_result.edge_metrics == ()
    assert dumps(store.get_snapshot()) == encoded_after
    assert all(u.role is UAVRole.IDLE for u in after.state.uavs)
    assert after.state.tasks == before.state.tasks
    assert after.state.mission == before.state.mission
    assert result == service.analyze(before)

def test_serialization_order_and_immutability(snapshot_factory):
    snapshot=snapshot_factory((("u2",20,0),("u1",10,0),("u3",100,0)))
    result=analyzer().analyze(snapshot)
    assert loads(dumps(result)) == result
    reverse=replace(snapshot,state=replace(snapshot.state,uavs=tuple(reversed(snapshot.state.uavs))))
    assert dumps(result) == dumps(analyzer().analyze(reverse))
    assert result.edge_metrics is result.network.links
    with pytest.raises(TypeError):
        result.routes_to_gcs["u2"]=None
    with pytest.raises(TypeError):
        result.hop_counts["u2"]=99
    with pytest.raises(TypeError):
        result.network_health["connectivity_ratio"]=9
    with pytest.raises(FrozenInstanceError):
        result.edge_metrics[0].etx=99

def test_legacy_constructor_and_json():
    legacy=NetworkAnalysis(0,NetworkState(),("u1",))
    assert legacy.connected_uav_ids == ("u1",)
    raw='{"schema_version":1,"type":"NetworkAnalysis","data":{"snapshot_revision":0,"network":{"links":[],"recovery_state":"NORMAL"},"connected_uav_ids":["u1"]}}'
    assert loads(raw) == legacy
    assert loads(dumps(legacy)) == legacy

def test_conditions_copied_and_repeatable(snapshot_factory):
    supplied={("u1","gcs"):LinkCondition(outage=True)}
    service=BaselineCommunicationAnalyzer(CommunicationConfig(10,5,0),supplied)
    supplied.clear()
    snapshot=snapshot_factory((("u1",10,0),("u2",20,0)))
    assert service.analyze(snapshot).disconnected_uav_ids == ("u1","u2")
    assert dumps(service.analyze(snapshot)) == dumps(service.analyze(snapshot))
    with pytest.raises(TypeError):
        service.conditions[("gcs","u1")]=LinkCondition()

def test_stored_network_is_not_an_implicit_condition(snapshot_factory):
    snapshot=snapshot_factory((("u1",10,0),))
    state=replace(snapshot.state,network=NetworkState((LinkState("gcs","u1",active=False),)))
    # Old measured links are not event semantics; explicit conditions are separate inputs.
    assert analyzer().analyze(replace(snapshot,state=state)).connected_uav_ids == ("u1",)

def test_simulation_time_not_wall_clock(snapshot_factory):
    service=analyzer()
    snapshot=snapshot_factory((("u1",10,0),))
    later=replace(snapshot,state=replace(snapshot.state,simulation_time=5),revision=1)
    result=service.analyze(later)
    assert result.simulation_time == 5
    assert all(link.last_updated == 5 for link in result.edge_metrics)
    assert result.routes_to_gcs == service.analyze(snapshot).routes_to_gcs

def test_gcs_only_contract(snapshot_factory):
    result=analyzer().analyze(snapshot_factory())
    assert result.routes_to_gcs == {}
    assert result.network_health["connectivity_ratio"] is None
    assert loads(dumps(result)) == result

@pytest.mark.parametrize("change", [
    {"hop_counts":{"u1":99}},
    {"routes_to_gcs":{"u1":("gcs","u1")}},
    {"disconnected_uav_ids":("u1",)},
    {"network":NetworkState()},
    {"components":(("gcs",),("u1",))},
])
def test_contract_rejects_inconsistent_complete_results(snapshot_factory,change):
    result=analyzer().analyze(snapshot_factory((("u1",10,0),)))
    with pytest.raises(ModelError):
        replace(result,**change)

def test_existing_yaml_config_and_scenario():
    root=Path(__file__).resolve().parents[2]
    config=load_config(root/"configs/default.yaml")
    scenario=load_scenario(root/"scenarios/basic.yaml",config)
    state=SwarmState(GCSState("gcs",Vector2D(config.gcs.x,config.gcs.y)),
                     uavs=scenario.uavs,tasks=scenario.tasks)
    store=StateStore(state,writer_key=object())
    result=BaselineCommunicationAnalyzer(config.communication).analyze(store.get_snapshot())
    assert result.connected_uav_ids == ("uav-1","uav-2")

def test_process_hash_seed_independence():
    code = """
from ares_swarm.core.models import *
from ares_swarm.core.snapshot import StateSnapshot
from ares_swarm.core.config import CommunicationConfig
from ares_swarm.core.serialization import dumps
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
positions = {("a",6,4),("b",6,-4),("u",12,0),("z",100,0)}
state = SwarmState(GCSState("gcs",Vector2D(0,0)),
                  uavs=tuple(UAVState(uid,Vector2D(x,y)) for uid,x,y in positions))
print(dumps(BaselineCommunicationAnalyzer(CommunicationConfig(10,5,0)).analyze(StateSnapshot(state,0))))
"""
    outputs=[]
    for seed in ("1","7","103"):
        env=dict(os.environ,PYTHONHASHSEED=seed,PYTHONDONTWRITEBYTECODE="1")
        outputs.append(subprocess.run([sys.executable,"-c",code],env=env,
                       text=True,capture_output=True,check=True).stdout)
    assert len(set(outputs)) == 1
    assert json.loads(outputs[0])["data"]["routes_to_gcs"]["u"] == ["u","a","gcs"]
