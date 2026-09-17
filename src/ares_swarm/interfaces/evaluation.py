"""Sujal's observation contract."""
from typing import Protocol
from ..core.snapshot import StateSnapshot
from ..core.transitions import StateTransition

class MetricsCollector(Protocol):
    def observe(self, previous_snapshot: StateSnapshot, transition: StateTransition,
                new_snapshot: StateSnapshot) -> None:
        """Observe an already committed change without modifying domain state."""
        ...
