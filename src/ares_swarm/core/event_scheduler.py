"""Deterministic simulation event scheduler."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ScheduledEventType(str, Enum):
    """Types of events that can be scheduled by the simulator."""

    UAV_FAILURE = "UAV_FAILURE"
    UAV_RECOVERY = "UAV_RECOVERY"


@dataclass(frozen=True)
class ScheduledEvent:
    """Immutable event scheduled for a simulation tick."""

    tick: int
    event_type: ScheduledEventType
    uav_id: str
    reason: str = ""


class EventScheduler:
    """Deterministic scheduler for simulation-time events."""

    def __init__(self) -> None:
        self._events: list[ScheduledEvent] = []

    def schedule(self, event: ScheduledEvent) -> None:
        """Add an event to the scheduler."""
        if event.tick < 0:
            raise ValueError("tick must be non-negative")

        self._events.append(event)
        self._events.sort(
            key=lambda item: (item.tick, item.uav_id, item.event_type.value)
        )

    def due_events(self, tick: int) -> tuple[ScheduledEvent, ...]:
        """Return and remove all events scheduled for this tick."""
        due = tuple(event for event in self._events if event.tick == tick)
        self._events = [
            event for event in self._events if event.tick != tick
        ]
        return due

    def pending_events(self) -> tuple[ScheduledEvent, ...]:
        """Return pending events in deterministic order."""
        return tuple(self._events)