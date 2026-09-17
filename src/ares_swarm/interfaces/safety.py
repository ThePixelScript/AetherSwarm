"""Sujal's safety/energy gate contract; validation algorithms are deferred."""
from dataclasses import dataclass
from typing import Protocol
from ..core.snapshot import StateSnapshot
from ..core.validation import Validated, require, nonnegative
from .autonomy import ActionProposal

@dataclass(frozen=True, slots=True)
class ProposalRejection(Validated):
    proposal_id: str
    reason: str

@dataclass(frozen=True, slots=True)
class ValidationResult(Validated):
    """Revision-bound accepted intents and non-overlapping rejection records."""
    snapshot_revision: int
    accepted: tuple[ActionProposal, ...] = ()
    rejected: tuple[ProposalRejection, ...] = ()

    def __post_init__(self) -> None:
        super(ValidationResult, self).__post_init__()
        nonnegative(self.snapshot_revision)
        require(all(p.snapshot_revision == self.snapshot_revision for p in self.accepted),
                "accepted proposal revision mismatch")
        ids = [p.id for p in self.accepted] + [p.proposal_id for p in self.rejected]
        require(len(ids) == len(set(ids)), "duplicate or conflicting proposal outcomes")

class SafetyValidator(Protocol):
    def validate(self, snapshot: StateSnapshot, proposals: list[ActionProposal]) -> ValidationResult:
        """Return accepted intent and explained rejections, never state transitions."""
        ...
