"""Immutable, versioned view. Nested state is deeply immutable."""
from dataclasses import dataclass
from .models import SwarmState
from .validation import Validated, nonnegative

@dataclass(frozen=True, slots=True)
class StateSnapshot(Validated):
    state: SwarmState
    revision: int

    def __post_init__(self) -> None:
        super(StateSnapshot, self).__post_init__()
        nonnegative(self.revision)
