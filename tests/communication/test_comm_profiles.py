import pytest
from ares_swarm.communication.config import COMM_PROFILES, CommunicationConfig
from ares_swarm.simulation.scenario import ScenarioConfig, load_scenario


def test_comm_profile_defaults():
    """Verify default organizer_reference profile is 100.0m."""
    cfg = CommunicationConfig()
    assert cfg.max_range == 100.0
    assert cfg.profile_name == "organizer_reference"


def test_comm_profile_dict_mapping():
    """Verify research profiles map correctly."""
    assert COMM_PROFILES["organizer_reference"] == 100.0
    assert COMM_PROFILES["research_150m"] == 150.0
    assert COMM_PROFILES["research_152m"] == 152.0


def test_scenario_comm_profile_parsing():
    """Verify profile parsing from scenario raw dictionary."""
    sc_dict = {
        "name": "test_prof",
        "communication": {"profile": "research_152m"},
    }
    sc = load_scenario(sc_dict)
    assert sc.communication.profile_name == "research_152m"
    assert sc.communication.max_range == 152.0
