from dataclasses import FrozenInstanceError
import pytest
from ares_swarm.interfaces.autonomy import ActionProposal
from ares_swarm.interfaces.safety import ValidationResult, ProposalRejection
from ares_swarm.core.exceptions import ModelError
from ares_swarm.core.serialization import dumps, loads

def test_revision_and_outcome_consistency():
    proposal = ActionProposal("p", 0, "move", "u1", "test", "planner")
    with pytest.raises(ModelError):
        ValidationResult(1, accepted=(proposal,))
    with pytest.raises(ModelError):
        ValidationResult(0, accepted=(proposal,), rejected=(ProposalRejection("p", "unsafe"),))
    result = ValidationResult(0, accepted=(proposal,))
    assert loads(dumps(result)) == result

def test_intent_payload_is_immutable():
    payload = {"nested": [1]}
    proposal = ActionProposal("p", 0, "move", "u1", "test", "planner", payload)
    payload["nested"].append(2)
    assert proposal.parameters["nested"] == (1,)
    with pytest.raises(TypeError):
        proposal.parameters["nested"] = ()
