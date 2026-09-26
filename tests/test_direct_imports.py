"""Fresh-interpreter checks catch cycles hidden by suite import order."""
import subprocess
import sys
import pytest


@pytest.mark.parametrize('statement', [
    'import ares_swarm',
    'from ares_swarm.autonomy.a1_allocator import A1TaskAllocator',
    'from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer',
    'from ares_swarm.telemetry.manager import DetectionManager',
    'from ares_swarm.simulation.runner import MissionRunner',
])
def test_standalone_import(statement):
    subprocess.run([sys.executable, '-c', statement], check=True, capture_output=True, text=True)
