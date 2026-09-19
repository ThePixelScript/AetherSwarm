"""Deterministic simulation trajectory recording."""

from __future__ import annotations

from dataclasses import dataclass, field

from ares_swarm.core.models import StateSnapshot


@dataclass
class ReplayRecorder:
    """Record immutable simulation snapshots for deterministic replay."""

    snapshots: list[StateSnapshot] = field(default_factory=list)

    def record(self, snapshot: StateSnapshot) -> None:
        """Record one simulation snapshot."""
        self.snapshots.append(snapshot)

    def latest(self) -> StateSnapshot | None:
        """Return the most recently recorded snapshot."""
        if not self.snapshots:
            return None
        return self.snapshots[-1]
    def replay(self) -> tuple[StateSnapshot, ...]:
        """Return recorded snapshots in deterministic recording order."""
        return tuple(self.snapshots)

    def clear(self) -> None:
        """Clear all recorded snapshots."""
        self.snapshots.clear()