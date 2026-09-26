"""Executed batch and bounded generalization evidence; no new autonomy policy."""
import argparse
import dataclasses
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from release_evidence import write_json


def batch_evidence():
    ns = runpy.run_path(str(ROOT / 'tests/autonomy/test_a1_release_regressions.py'))
    ns['test_protected_ineligible_peer_forces_unsafe_second_task_to_remain_unassigned']()
    ns['test_safe_alternative_candidate_is_selected_and_final_batch_connected']()
    from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
    results = []
    for protected in (True, False):
        snapshot, analyzer = ns['batch_case'](protected)
        net = analyzer.analyze(snapshot)
        result = A1TaskAllocator(comm_analyzer=analyzer).allocate(snapshot, network_analysis=net)
        final = analyzer.analyze(ns['apply_hypothetical'](snapshot, result.assignments))
        results.append({'protected_c_ineligible': protected,
                        'assignments': [dataclasses.asdict(a) for a in result.assignments],
                        'unassigned_tasks': result.unassigned_tasks,
                        'protected_connected_peers': net.connected_uav_ids,
                        'final_connected_peers': final.connected_uav_ids,
                        'connectivity_preserved': set(net.connected_uav_ids) <= set(final.connected_uav_ids)})
    return results


def random_evidence():
    from scripts.generate_random_scenario import generate_random_scenario
    from ares_swarm.simulation.scenario import load_scenario
    from ares_swarm.simulation.runner import MissionRunner
    from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
    from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
    from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
    results = []
    for seed in (2026, 42, 137):
        scenario = load_scenario(generate_random_scenario(seed))
        cp = dataclasses.replace(scenario.challenge_profile, enforce_separation=True, enforce_geofence=True,
             detection_pipeline=dataclasses.replace(scenario.challenge_profile.detection_pipeline, enabled=True))
        scenario = dataclasses.replace(scenario, challenge_profile=cp)
        analyzer = BaselineCommunicationAnalyzer(scenario.communication)
        runner = MissionRunner(scenario=scenario, seed=seed, comm_analyzer=analyzer,
                 autonomy_adapter=A0AutonomyAdapter(A1TaskAllocator(comm_analyzer=analyzer)))
        result = runner.run()
        report = result.to_dict()
        results.append({'seed': seed, 'profile': 'uniform full-arena, no fixed canonical ingress',
                        'completed': report['metrics']['tasks_completed'],
                        'uncompleted': report['metrics']['tasks_total'] - report['metrics']['tasks_completed'],
                        'task_statuses': {k: v['status'] for k, v in report['tasks'].items()},
                        'communication': report['evaluation']['communication'],
                        'safety': report['evaluation']['safety'], 'telemetry': report['telemetry']})
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['batch', 'random'])
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    write_json(args.output, batch_evidence() if args.mode == 'batch' else random_evidence())


if __name__ == '__main__':
    main()
