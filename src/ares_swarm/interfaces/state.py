"""Sarath's read-only state-access contract for future composition."""
from typing import Protocol
from ..core.snapshot import StateSnapshot

class StateReader(Protocol):
    def get_snapshot(self) -> StateSnapshot:
        """Return immutable authoritative state without exposing commit authority."""
        ...
