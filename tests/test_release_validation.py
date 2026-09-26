"""The validator must reject missing evidence and failed child processes."""
import json
import subprocess
import sys
import pytest
from scripts.validation_support import Commands, junit_counts, identical_hashes, e2_causal, require_canonical


def test_command_failure_is_logged_and_raised(tmp_path):
    commands = Commands(tmp_path, tmp_path / 'logs')
    with pytest.raises(subprocess.CalledProcessError):
        commands.run('failure', [sys.executable, '-c', 'import sys; print("failed stage"); sys.exit(7)'])
    log = json.loads((tmp_path / 'logs/failure.json').read_text())
    assert log['returncode'] == 7
    assert 'failed stage' in log['stdout']


def test_junit_counts_come_from_report(tmp_path):
    path = tmp_path / 'tests.xml'
    path.write_text('<testsuites><testsuite tests="9" failures="1" errors="2" skipped="1" /></testsuites>')
    assert junit_counts(path) == {'collected': 9, 'passed': 5, 'failed': 1, 'errors': 2, 'skipped': 1}


@pytest.mark.parametrize('text', ['not xml', '<testsuites/>', '<testsuites><testsuite tests="0" failures="0" errors="0" skipped="0"/></testsuites>'])
def test_unparseable_or_empty_test_report_never_falls_back(tmp_path, text):
    path = tmp_path / 'tests.xml'
    path.write_text(text)
    with pytest.raises(Exception):
        junit_counts(path)


def test_missing_canonical_evidence_cannot_pass():
    with pytest.raises((TypeError, KeyError, ValueError)):
        require_canonical({})


def test_changed_mission_value_breaks_determinism():
    assert not identical_hashes([{'latency': 1.}, {'latency': 1.00001}])['identical']


def test_same_task_completion_does_not_establish_e2_causality():
    result = {'assignments': [], 'telemetry': {'reports_delivered': 1, 'reports_deadline_exceeded': 0}}
    assert not e2_causal(result, result)
