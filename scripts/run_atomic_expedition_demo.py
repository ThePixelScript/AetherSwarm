"""Deterministic bounded 8-UAV experiment using the existing discovery/generator/runner."""
from __future__ import annotations
import argparse
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.generate_scenario import generate_final_mission_scenario
from scripts.export_webots_trace import validate_trace_data
from ares_swarm.simulation.scenario import load_scenario
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.autonomy.connectivity_planner import ConnectivityAwarePlanner, ConnectivityAwarePlannerConfig
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer


def run(output, seed=2026):
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    # Bounded visibility experiment, not a full-arena coverage claim. Generation
    # and altitude-aware discovery are Divesh's unchanged implementations.
    generated = generate_final_mission_scenario(seed=seed, num_uavs=8,
        output_scenario=output/'scenario.yaml', run_simulation=False,
        custom_x_range=(30.,200.), custom_y_range=(440.,560.),
        spawn_start=0., spawn_end=120., emergence_start=20., emergence_end=180.)
    scenario = load_scenario(generated['scenario_path'])
    scenario = replace(scenario, duration=1200., max_ticks=1200,
        challenge_profile=replace(scenario.challenge_profile, enforce_sortie_limit=True,
                                  enforce_single_sortie=True))
    analyzer = BaselineCommunicationAnalyzer(scenario.communication)
    planner = ConnectivityAwarePlanner(config=ConnectivityAwarePlannerConfig(
        elastic_tether_enabled=True, comm_range_m=scenario.communication.max_range,
        speed_limit=scenario.speed_limit), allocator=A1TaskAllocator(comm_analyzer=analyzer))
    runner = MissionRunner(scenario, seed=seed, comm_analyzer=analyzer, connectivity_planner=planner)
    result = runner.run()
    report = result.to_dict()
    moving, concurrent = [], []
    previous = runner.initial_snapshot
    ticks = []
    for step in result.step_history:
        snap = step.snapshot
        moving.append(sum(math.dist(previous.uavs[k].position_xy,u.position_xy)>1e-8 for k,u in snap.uavs.items()))
        concurrent.append(sum(t.status.value in ('ASSIGNED','IN_PROGRESS') for t in snap.tasks.values()))
        ticks.append({'tick':step.tick,'time':step.simulation_time,
          'uavs': {k:{'id':k,'position':[*u.position_xy,u.altitude_m if u.active else .5],
             'velocity':[*u.velocity_xy,0.], 'yaw':math.atan2(u.velocity_xy[1],u.velocity_xy[0]),
             'role':u.role.value,'active':u.active,'failure_state':u.failure_state.value,
             'rth_state':u.rth_state.value,'battery_percent':u.battery_percent,
             'assigned_task_id':u.assigned_task_id} for k,u in snap.uavs.items()},
          'tasks':{k:{'id':k,'position':[*t.position_xy,0.], 'status':t.status.value,
             'priority':t.priority,'assigned_uav_id':t.assigned_uav_id,
             'service_progress':t.service_progress,'service_duration':t.service_duration} for k,t in snap.tasks.items()},
          'network':{'active_links':[{'source':e.source_id,'target':e.target_id,'pdr':e.estimated_pdr} for e in step.network_analysis.edge_metrics],
             'routes_to_gcs':dict(step.network_analysis.routes_to_gcs),
             'route_pdr_to_gcs':dict(step.network_analysis.route_pdr_to_gcs),
             'connected_uav_ids':step.network_analysis.connected_uav_ids,
             'hop_counts':dict(step.network_analysis.hop_counts)},
          'events':[{'type':e.event_type.value,'entity_id':e.entity_id,'payload':dict(e.payload)} for e in step.events]})
        previous = snap
    discovery = runner.discovery_manager.get_summary()
    records = discovery['discoveries']
    safety = result.safety_report
    metrics = {'seed':seed,'uav_count':len(scenario.uavs),'known_pois':len(scenario.tasks),
        'hidden_pois':len(scenario.hidden_pois),'hidden_discovered':len(records),
        'discovery_records':records,
        'mean_discovery_distance':sum(r['distance'] for r in records)/len(records) if records else None,
        'emergence_to_discovery_s':{r['poi_id']:r['time']-r['emergence_time'] for r in records},
        'max_simultaneous_moving_uavs':max(moving,default=0),'max_concurrent_tasks':max(concurrent,default=0),
        'teams':planner.expedition_plans,'tether_holds':len(runner.elastic_tether.holds),
        'gamma_checks':runner.elastic_tether.gamma_checks,'post_safety_scales':runner.elastic_tether.post_safety_scales,
        'communication':report['evaluation']['communication'],'tasks':report['metrics'],
        'reports':report.get('telemetry'), 'minimum_separation':safety.min_observed_separation_m,
        'violations':{'separation':safety.separation_violations_count,'geofence':safety.geofence_violations_count,
          'flight':safety.flight_duration_violations_count,'landing':safety.landing_violations_count,
          'battery':safety.battery_exhaustions_count},
        'landed':sum(not u.active and u.rth_state.value=='COMPLETE' for u in result.final_snapshot.uavs.values()),
        'rth_events':sum(e.event_type.value=='RTH_TRIGGERED' for e in result.all_events),
        'max_airborne_s':safety.max_observed_sortie_duration_s,
        'rejected_commands':sum(len(s.rejected_commands) for s in result.step_history),
        'rejection_details':[{'tick':s.tick,'command':type(r.command).__name__,
            'reason':r.reason} for s in result.step_history for r in s.rejected_commands]}
    trace = {'metadata':{'scenario_name':scenario.name,'seed':seed,'arena_bounds_x':scenario.arena_bounds_x,
        'arena_bounds_y':scenario.arena_bounds_y,'max_altitude':scenario.max_height,
        'gcs_position':[*scenario.gcs_position,0.],'tick_duration':scenario.dt,'total_ticks':len(ticks),
        'uav_ids':sorted(result.final_snapshot.uavs),'task_ids':sorted(result.final_snapshot.tasks)},
        'ticks':ticks,'events':[e for t in ticks for e in t['events']]}
    validate_trace_data(trace)
    metrics['history_sha256'] = hashlib.sha256(json.dumps(trace,sort_keys=True).encode()).hexdigest()
    for filename, data in [('metrics.json',metrics),('trace.json',trace),('report.json',report),
                           ('effective_scenario.json',asdict(scenario))]:
        (output/filename).write_text(json.dumps(data,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    return metrics


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--outdir',default='results/atomic-expedition')
    parser.add_argument('--runs',type=int,default=3)
    args=parser.parse_args()
    values=[run(Path(args.outdir)/f'run-{i}') for i in range(args.runs)]
    if not values or any(v != values[0] for v in values[1:]):
        raise RuntimeError('Demo repetitions differ or no runs requested')
    verify_demo(values[0])
    summary={'runs':len(values),'deterministic':True,'metrics':values[0]}
    (Path(args.outdir)/'summary.json').write_text(json.dumps(summary,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in values[0].items() if k not in ('teams','discovery_records')},indent=2))


def verify_demo(metrics):
    """Fail closed on safety, exercised mechanisms, and useful mission progress."""
    if (any(metrics['violations'].values()) or metrics['minimum_separation'] < 20.
            or metrics['max_airborne_s'] > 1200. or metrics['landed'] != metrics['uav_count']
            or metrics['gamma_checks'] == 0 or metrics['tether_holds'] == 0
            or metrics['max_concurrent_tasks'] < 2 or metrics['max_simultaneous_moving_uavs'] < 2
            or metrics['hidden_discovered'] == 0 or metrics['mean_discovery_distance'] <= 0
            or metrics['tasks']['tasks_completed'] == 0 or metrics['reports']['reports_delivered'] == 0):
        raise RuntimeError('Experimental demo acceptance gate failed; inspect metrics.json')


if __name__=='__main__':
    main()
