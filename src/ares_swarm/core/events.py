"""Event and result definitions for AetherSwarm simulation."""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Mapping, Tuple
from .commands import Command
from .enums import EventType, RejectionCode


@dataclass(frozen=True)
class CommandRejection:
    """Represents a rejected command with reason."""
    command: Command
    code: RejectionCode
    reason: str = field(default="")

    def __post_init__(self) -> None:
        if not self.reason:
            object.__setattr__(self, "reason", self.code.value)


@dataclass(frozen=True)
class DomainEvent:
    """Represents an immutable domain event during simulation."""
    event_id: str
    simulation_tick: int
    simulation_time: float
    event_type: EventType
    entity_id: str
    payload: Mapping[str, Any] = field(default_factory=dict)

    @classmethod
    def create(
        cls,
        simulation_tick: int,
        simulation_time: float,
        event_type: EventType,
        entity_id: str,
        payload: Mapping[str, Any],
        sequence: int = 0,
    ) -> DomainEvent:
        return cls(
            event_id=f"evt_{simulation_tick}_{sequence}",
            simulation_tick=simulation_tick,
            simulation_time=simulation_time,
            event_type=event_type,
            entity_id=entity_id,
            payload=payload,
        )


@dataclass(frozen=True)
class StateTransitionResult:
    """Result of a state transition after applying commands."""
    previous_version: int
    new_version: int
    simulation_tick: int
    applied_commands: Tuple[Command, ...] = field(default_factory=tuple)
    rejected_commands: Tuple[CommandRejection, ...] = field(default_factory=tuple)
    emitted_events: Tuple[DomainEvent, ...] = field(default_factory=tuple)