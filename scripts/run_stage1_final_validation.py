"""Execute release gates; failures and missing evidence produce a nonzero exit."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import sys
import tempfile

from validation_support import (Commands, read_json, pytest_run, require_canonical,
                                e2_projection, e2_causal, identical_hashes, write_json)

ROOT = Path(__file__).resolve().parents[1]


def validate(output):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    run_dir = Path(tempfile.mkdtemp(prefix='validation-', dir=output / 'runs'))
    commands = Commands(ROOT, run_dir / 'logs')
    summary = {'status': 'INCOMPLETE', 'verified_at_utc': datetime.now(timezone.utc).isoformat(),
               'artifacts_directory': str(run_dir.relative_to(ROOT)) if run_dir.is_relative_to(ROOT) else str(run_dir)}
    try:
        summary['repository'] = {
            'commit': commands.run('git-head', ['git', 'rev-parse', 'HEAD']),
            'branch': commands.run('git-branch', ['git', 'branch', '--show-current']),
            'origin': commands.run('git-origin', ['git', 'remote', 'get-url', 'origin']),
            'starting_status': commands.run('git-status', ['git', 'status', '--short']),
        }
        summary['environment'] = {
            'python': commands.run('python', [sys.executable, '--version']),
            'pip': commands.run('pip-version', [sys.executable, '-m', 'pip', '--version']),
            'pip_check': commands.run('pip-check', [sys.executable, '-m', 'pip', 'check']),
            'freeze': commands.run('pip-freeze', [sys.executable, '-m', 'pip', 'freeze']).splitlines(),
            'prefix': sys.prefix, 'base_prefix': sys.base_prefix,
        }
        summary['fresh_clone_reproduction'] = {
            'independent_git_directory': (ROOT / '.git').is_dir(),
            'venv_active': sys.prefix != sys.base_prefix,
            'venv_inside_clone': Path(sys.prefix).resolve().is_relative_to(ROOT),
            'note': 'Creation/install command provenance is recorded separately in EVIDENCE_MANIFEST.md; these checks alone do not prove freshness.',
        }
        imports = ['import ares_swarm',
                   'from ares_swarm.autonomy.a1_allocator import A1TaskAllocator',
                   'from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer',
                   'from ares_swarm.telemetry.manager import DetectionManager',
                   'from ares_swarm.simulation.runner import MissionRunner']
        for index, statement in enumerate(imports):
            commands.run(f'import-{index}', [sys.executable, '-c', statement])
        summary['direct_imports'] = {'statements': imports, 'passed': len(imports)}
        summary['tests'] = pytest_run(commands, 'full-tests', ['tests/'], run_dir)
        groups = {
            'autonomy': ['tests/autonomy/'], 'communication': ['tests/communication/'],
            'telemetry': ['tests/telemetry/'], 'safety': ['tests/safety/'],
            'simulation': ['tests/simulation/'], 'visualization': ['tests/visualization/'],
            'batch': ['tests/autonomy/test_a1_batch_allocation.py', 'tests/autonomy/test_a1_release_regressions.py'],
        }
        summary['focused_tests'] = {name: pytest_run(commands, name, paths, run_dir) for name, paths in groups.items()}
        canonical = []
        for index in range(10):
            target = run_dir / f'canonical-{index}'
            commands.run(f'canonical-{index}', [sys.executable, 'scripts/run_stage1_submission.py', '--outdir', target])
            report = read_json(target / 'stage1_submission_report.json')
            require_canonical(report)
            canonical.append(report)
        summary['canonical'] = require_canonical(canonical[0])
        summary['determinism'] = {'canonical': identical_hashes(canonical)}
        if not summary['determinism']['canonical']['identical']:
            raise ValueError('Canonical results differ')
        e2 = []
        for index in range(5):
            target = run_dir / f'e2-{index}'
            commands.run(f'e2-{index}', [sys.executable, 'scripts/run_e2_controlled.py', '--outdir', target])
            a0, a1 = (read_json(target / f'{name}_e2.json') for name in ('a0', 'a1'))
            if not e2_causal(e2_projection(a0), e2_projection(a1)):
                raise ValueError('E2 causal distinction lost')
            e2.append({'a0': a0, 'a1': a1})
        summary['E2'] = {name: e2_projection(e2[0][name]) for name in ('a0', 'a1')}
        summary['E2']['causal_difference_preserved'] = e2_causal(summary['E2']['a0'], summary['E2']['a1'])
        summary['determinism']['E2'] = identical_hashes(e2)
        if not summary['determinism']['E2']['identical']:
            raise ValueError('E2 results differ')
        failures = []
        for index in range(3):
            target = run_dir / f'failure-{index}'
            commands.run(f'failure-{index}', [sys.executable, 'scripts/run_demo_in_flight_recovery.py', '--output-dir', target])
            data = read_json(target / 'failure_summary.json')
            # Artifact locations differ by repetition, not mission observations.
            observation = {k: v for k, v in data.items() if k not in ('replay_path', 'timeline_path')}
            replay = read_json(target / 'replay_in_flight_recovery.json')
            observation['events'] = replay['events']
            if not observation['deferred_verified'] or observation['replacement_uav'] == observation['original_uav'] or observation['completion_tick'] is None:
                raise ValueError('Worker failure/reassignment chain failed')
            if not observation['task_in_progress_tick'] < observation['failure_tick'] <= observation['reassignment_tick'] < observation['completion_tick']:
                raise ValueError('Failure lifecycle timing inconsistent')
            failures.append(observation)
        summary['failure_recovery'] = failures[0]
        summary['determinism']['failure_recovery'] = identical_hashes(failures)
        if not summary['determinism']['failure_recovery']['identical']:
            raise ValueError('Failure results differ')
        commands.run('batch-evidence', [sys.executable, 'scripts/run_release_probes.py', 'batch', '--output', run_dir / 'batch.json'])
        summary['batch'] = read_json(run_dir / 'batch.json')
        if not all(case['connectivity_preserved'] for case in summary['batch']):
            raise ValueError('Batch topology protection failed')
        commands.run('random-evidence', [sys.executable, 'scripts/run_release_probes.py', 'random', '--output', run_dir / 'random.json'])
        summary['generalization'] = read_json(run_dir / 'random.json')
        commands.run('diff-check', ['git', 'diff', '--check'])
        summary['webots'] = {'native_executable': shutil.which('webots') or 'NOT_AVAILABLE',
                              'native_rendering_executed': False,
                              'standalone_control_tests': summary['focused_tests']['visualization']}
        # Copy actual first-run evidence, never hand-edit observed metrics.
        shutil.copyfile(run_dir / 'canonical-0/stage1_submission_report.json', output / 'stage1_submission_report.json')
        for name in ('a0', 'a1'):
            shutil.copyfile(run_dir / f'e2-0/{name}_e2.json', output / f'{name}_e2.json')
        write_json(output / 'failure_evidence.json', summary['failure_recovery'])
        shutil.copyfile(run_dir / 'full-tests.xml', output / 'test_results.xml')
        shutil.copyfile(run_dir / 'logs/full-tests.json', output / 'test_execution.json')
        summary['status'] = 'AUTOMATED_GATES_PASSED'
    except Exception as exc:
        summary['status'] = 'FAILED'
        summary['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        summary['commands'] = [{'name': r['name'], 'argv': r['argv'], 'returncode': r['returncode'],
                                'log': str((run_dir / 'logs' / (r['name'] + '.json')).relative_to(ROOT))}
                               for r in commands.records]
        write_json(output / 'final_validation_summary.json', summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--outdir', default='stage1_output')
    args = parser.parse_args()
    output = Path(args.outdir).resolve()
    (output / 'runs').mkdir(parents=True, exist_ok=True)
    result = validate(output)
    print(result['status'])
    print(output / 'final_validation_summary.json')


if __name__ == '__main__':
    main()
