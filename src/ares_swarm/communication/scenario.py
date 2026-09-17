"""Deterministic scenario-driven communication conditions."""
from dataclasses import dataclass
from typing import Sequence, Mapping, Tuple

from .channel import LinkCondition
from .validation import Validated, require, nonnegative


@dataclass(frozen=True)
class CommunicationCondition(Validated):
    """A deterministic communication condition scheduled for a specific time interval."""
    source_id: str
    target_id: str
    start_time_s: float
    end_time_s: float
    outage: bool = False
    quality_multiplier: float = 1.0
    packet_loss_override: float | None = None
    latency_penalty_ms: float = 0.0

    def __post_init__(self) -> None:
        require(self.source_id != self.target_id, "source and target must be distinct")
        nonnegative(self.start_time_s)
        require(self.end_time_s >= self.start_time_s, "end time must be >= start time")
        require(0 <= self.quality_multiplier <= 1.0, "quality multiplier must be in [0,1]")
        if self.packet_loss_override is not None:
            require(0 <= self.packet_loss_override <= 1.0, "packet_loss_override must be in [0,1]")
        nonnegative(self.latency_penalty_ms)


def resolve_conditions(
    simulation_time: float,
    conditions: Sequence[CommunicationCondition]
) -> Mapping[Tuple[str, str], LinkCondition]:
    """Resolve a list of scenario conditions for the current simulation time.
    
    Returns a mapping of unordered pairs to their active LinkCondition.
    Overlapping conditions for the same pair are rejected to ensure determinism.
    """
    resolved = {}
    
    for cond in conditions:
        if cond.start_time_s <= simulation_time < cond.end_time_s:
            pair = tuple(sorted((cond.source_id, cond.target_id)))
            require(pair not in resolved, f"overlapping active conditions for pair {pair}")
            
            resolved[pair] = LinkCondition(
                quality_multiplier=cond.quality_multiplier,
                latency_penalty_ms=cond.latency_penalty_ms,
                outage=cond.outage,
                packet_loss_override=cond.packet_loss_override
            )
            
    return resolved
