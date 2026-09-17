"""Serializable event descriptions; execution belongs to Phase 2."""
from dataclasses import dataclass, field
from typing import Any, Mapping
from .enums import EventType
from .validation import Validated, freeze, nonnegative, require

@dataclass(frozen=True, slots=True)
class SimulationEvent(Validated):
    id: str
    timestamp: float
    event_type: EventType
    priority: int = 0
    target: str | None = None
    payload: Mapping[str, Any] = field(default_factory=dict)
    processed: bool = False

    def __post_init__(self) -> None:
        super(SimulationEvent, self).__post_init__()
        require(bool(self.id.strip()), "empty event id")
        nonnegative(self.timestamp)
        object.__setattr__(self, "payload", freeze(self.payload))
