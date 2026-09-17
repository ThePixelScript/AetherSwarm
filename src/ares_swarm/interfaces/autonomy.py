"""Divesh's planner contract. Proposals cannot be committed to StateStore."""
from dataclasses import dataclass, field
from typing import Any, Mapping, Protocol
from ..core.validation import Validated, freeze, require, nonnegative
from ..core.snapshot import StateSnapshot
from .communication import NetworkAnalysis

@dataclass(frozen=True, slots=True)
class ActionProposal(Validated):
    id: str
    snapshot_revision: int
    intent: str
    target: str
    reason: str
    source: str
    parameters: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        super(ActionProposal, self).__post_init__()
        nonnegative(self.snapshot_revision)
        require(all(v.strip() for v in (self.id, self.intent, self.target, self.reason, self.source)),
                "proposal identifiers and reason required")
        object.__setattr__(self, "parameters", freeze(self.parameters))

class AutonomyPlanner(Protocol):
    def plan(self, snapshot: StateSnapshot, network_analysis: NetworkAnalysis) -> list[ActionProposal]:
        """Propose intent using analyses with a matching snapshot revision."""
        ...
