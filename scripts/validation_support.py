"""Fail-closed release orchestration helpers, independently unit tested."""
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

try:
    from .release_evidence import digest, write_json
except ImportError:
    from release_evidence import digest, write_json


class Commands:
    def __init__(self, root, output):
        self.root, self.output = Path(root), Path(output)
        self.records = []

    def run(self, name, argv):
        result = subprocess.run([str(x) for x in argv], cwd=self.root,
                                capture_output=True, text=True, encoding='utf-8', errors='replace')
        record = {'name': name, 'argv': [str(x) for x in argv],
                  'returncode': result.returncode, 'stdout': result.stdout, 'stderr': result.stderr}
        self.records.append(record)
        write_json(self.output / f'{name}.json', record)
        result.check_returncode()
        return result.stdout.strip()


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def junit_counts(path):
    root = ET.parse(path).getroot()
    suites = [root] if root.tag == 'testsuite' else list(root.findall('testsuite'))
    if not suites:
        raise ValueError('No test suites in JUnit report')
    counts = {key: sum(int(s.attrib[key]) for s in suites)
              for key in ('tests', 'failures', 'errors', 'skipped')}
    if counts['tests'] <= 0:
        raise ValueError('No tests executed')
    passed = counts['tests'] - counts['failures'] - counts['errors'] - counts['skipped']
    if passed < 0:
        raise ValueError('Invalid JUnit counts')
    return {'collected': counts['tests'], 'passed': passed, 'failed': counts['failures'],
            'errors': counts['errors'], 'skipped': counts['skipped']}


def pytest_run(commands, name, paths, directory):
    xml = Path(directory) / f'{name}.xml'
    commands.run(name, [sys.executable, '-m', 'pytest', *paths, '-q',
                       f'--basetemp={Path(directory) / (name + "_tmp")}', f'--junitxml={xml}'])
    result = junit_counts(xml)
    if result['failed'] or result['errors']:
        raise ValueError(f'{name} failed: {result}')
    return result


def canonical_projection(report):
    """Read real schema, keeping absence explicit rather than fabricating metrics."""
    evaluation = report.get('evaluation', {})
    evidence = report.get('execution_evidence', {})
    return {
        'mission': report.get('metrics', 'NOT_REPORTED'),
        'task_completion_time_s': evaluation.get('mission', {}).get('completion_time_s', 'NOT_REPORTED'),
        'mission_termination_time_s': report.get('final_simulation_time', 'NOT_REPORTED'),
        'communication': evaluation.get('communication', 'NOT_REPORTED'),
        'telemetry': report.get('telemetry', 'NOT_REPORTED'),
        'safety': evidence.get('safety', 'NOT_REPORTED'),
        'flight_records': evidence.get('flight_records', 'NOT_REPORTED'),
    }


def require_canonical(report):
    p = canonical_projection(report)
    m, t, s = p['mission'], p['telemetry'], p['safety']
    # These are acceptance thresholds, never reported execution results.
    if m['tasks_total'] != 10 or m['tasks_completed'] != m['tasks_total']:
        raise ValueError('Canonical service gate failed')
    if t['total_detections'] != m['tasks_total'] or t['reports_delivered'] != m['tasks_total'] or t['reports_deadline_exceeded'] != 0:
        raise ValueError('Canonical reporting count gate failed')
    reports = report['telemetry_reports'].values()
    if any(r['status'] != 'DELIVERED' or r['reporting_latency_s'] > 10 for r in reports):
        raise ValueError('Reporting deadline gate failed')
    if s['minimum_separation_m'] < 20 or s['max_airborne_s'] > 1200:
        raise ValueError('Canonical separation/sortie gate failed')
    for key in ('separation_violations', 'geofence_violations', 'landing_violations',
                'flight_duration_violations', 'battery_exhaustions'):
        if s[key] != 0:
            raise ValueError(f'Canonical safety gate failed: {key}={s[key]}')
    if s['landed_count'] != s['expected_uav_count'] or p['mission_termination_time_s'] > 2700:
        raise ValueError('Canonical landing/mission-duration gate failed')
    return p


def e2_projection(report):
    events = report['execution_evidence']['events']
    return {
        'assignments': [{'uav': e['entity_id'], 'task': e['payload']['task_id'],
                         'time': e['simulation_time']} for e in events if e['event_type'] == 'TASK_ASSIGNED'],
        'tasks': report['tasks'], 'telemetry': report['telemetry'],
        'reports': report['telemetry_reports'],
    }


def e2_causal(a0, a1):
    return (a0['assignments'] != a1['assignments']
            and a1['telemetry']['reports_delivered'] > a0['telemetry']['reports_delivered']
            and a0['telemetry']['reports_deadline_exceeded'] > a1['telemetry']['reports_deadline_exceeded'])


def identical_hashes(values):
    hashes = [digest(value) for value in values]
    return {'runs': len(hashes), 'sha256': hashes, 'identical': len(set(hashes)) == 1}
