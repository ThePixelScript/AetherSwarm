from dataclasses import dataclass
from .validation import Validated, require, nonnegative

@dataclass(frozen=True, slots=True)
class LinkState(Validated):
    source_id: str
    target_id: str
    distance: float = 0.0
    rssi: float | None = None
    snr: float | None = None
    packet_loss_probability: float = 0.0
    estimated_pdr: float = 1.0
    latency_ms: float = 0.0
    etx: float = 1.0
    link_quality: float = 1.0
    active: bool = True
    last_updated: float = 0.0

    def __post_init__(self) -> None:
        
        require(bool(self.source_id) and bool(self.target_id) and self.source_id != self.target_id,
                "invalid link endpoints")
        nonnegative(self.distance, self.latency_ms, self.last_updated)
        require(self.etx >= 1, "ETX must be >= 1")
        require(all(0 <= v <= 1 for v in (
            self.packet_loss_probability, self.estimated_pdr, self.link_quality)), "invalid probability")

@dataclass(frozen=True, slots=True)
class NetworkState(Validated):
    links: tuple[LinkState, ...] = ()
