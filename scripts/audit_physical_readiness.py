"""Audit actual post-step Gamma connectivity without trusting planned stations."""
import argparse
import json
import math
from pathlib import Path

from ares_swarm.simulation.scenario import load_scenario
from ares_swarm.simulation.runner import MissionRunner


def audit(path, ticks=720):
    runner = MissionRunner(load_scenario(path), seed=2026)
    failures, rows = [], []
    for _ in range(ticks):
        step = runner.step()
        snap = step.snapshot
        net = runner.comm_analyzer.analyze(snap)
        eligible = {k for k,u in snap.uavs.items() if u.active and
                    u.sortie_state.value not in ('LANDED','RECHARGING')}
        missing = sorted(eligible-set(net.connected_uav_ids))
        row = {'tick':snap.simulation_tick,'missing':missing,
               'uavs':{k:{'position':u.position_xy,'target':u.target_position,
                          'role':u.role.value,'rth':u.rth_state.value,'task':u.assigned_task_id}
                       for k,u in snap.uavs.items()},
               'chains':{k:{'status':c.status,'relays':c.relay_ids,'surveyor':c.surveyor_id,
                            'stations':c.station_positions} for k,c in runner.relay_manager.chains.items()}}
        if missing:
            row['cut_distances'] = {k:{'gcs':math.dist(snap.uavs[k].position_xy,snap.gcs_position),
                **{j:math.dist(snap.uavs[k].position_xy,snap.uavs[j].position_xy)
                   for j in net.connected_uav_ids}} for k in missing}
            failures.append(row)
        rows.append(row)
    ranges=[]
    for row in failures:
        t=row['tick']
        if ranges and ranges[-1][1]+1==t: ranges[-1][1]=t
        else: ranges.append([t,t])
    summary={'scenario':str(path),'ticks':ticks,'pass':ticks-len(failures),'fail':len(failures),
             'ranges':ranges,'affected':sorted({u for r in failures for u in r['missing']}),
             'tasks_completed':sum(t.status.value=='COMPLETE' for t in snap.tasks.values()),
             'tasks_total':len(snap.tasks),'landed':sum(not u.active for u in snap.uavs.values()),
             'minimum_separation':runner.safety_assessor.report.min_observed_separation_m,
             'geofence_violations':runner.safety_assessor.report.geofence_violations_count,
             'separation_violations':runner.safety_assessor.report.separation_violations_count}
    return {'summary':summary,'failures':failures,'ticks':rows}


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--scenario',default='scenarios/random_seed_2026.yaml')
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    result=audit(args.scenario)
    path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result['summary'],indent=2))
