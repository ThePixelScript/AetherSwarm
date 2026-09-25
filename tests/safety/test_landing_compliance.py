import pytest
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import load_scenario
from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
import dataclasses

def test_canonical_landing_compliance():
    base_scenario = load_scenario('scenarios/poc_round1.yaml')
    
    dp = dataclasses.replace(base_scenario.challenge_profile.detection_pipeline, enabled=True)
    airspace = dataclasses.replace(base_scenario.challenge_profile.airspace, enabled=True)
    cp = dataclasses.replace(
        base_scenario.challenge_profile,
        enabled=True,
        enforce_separation=True,
        enforce_sortie_limit=True,
        enforce_single_sortie=True,
        enforce_geofence=True,
        detection_pipeline=dp,
        airspace=airspace,
    )
    
    # We do NOT set gcs_position here, because our fix in ScenarioConfig.__post_init__ handles it!
    scenario = dataclasses.replace(base_scenario, challenge_profile=cp)
    
    comm = BaselineCommunicationAnalyzer(config=scenario.communication)
    allocator = A1TaskAllocator(comm_analyzer=comm)
    from ares_swarm.autonomy.ingress_coordinator import IngressCoordinator
    ingress_coordinator = IngressCoordinator(comm_analyzer=comm, gcs_position=scenario.gcs_position, d_safe=85.0)
    adapter = A0AutonomyAdapter(allocator=allocator, ingress_coordinator=ingress_coordinator)
    
    runner = MissionRunner(
        scenario=scenario,
        seed=2026,
        autonomy_adapter=adapter,
        comm_analyzer=comm
    )
    
    res = runner.run()
    
    assert runner.geofence_enforcer is not None
    assert runner.safety_assessor.gcs_position == (-75.0, 500.0)
    
    s_rep = res.safety_report
    assert s_rep.landing_violations_count == 0
    assert s_rep.geofence_violations_count == 0
    assert s_rep.min_observed_separation_m >= 20.0 - 1e-4
    assert s_rep.flight_duration_violations_count == 0
    
    # Check all landed
    for uid, u in res.final_snapshot.uavs.items():
        assert u.rth_state.name == 'COMPLETE'
        # Check inside authorized staging area on x=-75 pad line
        assert runner.safety_assessor.airspace.is_in_staging_area(u.position_xy)
        assert abs(u.position_xy[0] - (-75.0)) <= 1.0
