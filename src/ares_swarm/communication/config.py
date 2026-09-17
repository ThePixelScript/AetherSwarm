from dataclasses import dataclass

@dataclass(frozen=True)
class CommunicationConfig:
    max_range: float = 1000.0
    base_latency: float = 5.0
    packet_loss: float = 0.0
    degradation_multiplier: float = 1.0
