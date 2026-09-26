from dataclasses import dataclass

COMM_PROFILES = {
    "organizer_reference": 100.0,
    "research_150m": 150.0,
    "research_152m": 152.0,
}

@dataclass(frozen=True)
class CommunicationConfig:
    max_range: float = 100.0
    base_latency: float = 5.0
    packet_loss: float = 0.0
    degradation_multiplier: float = 1.0
    profile_name: str = "organizer_reference"
