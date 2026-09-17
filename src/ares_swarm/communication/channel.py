"""Deterministic Stage-1 link abstraction; no physical-radio accuracy claim."""
from dataclasses import dataclass
from math import hypot, isfinite

from ..core.config import CommunicationConfig
from ..core.enums import FailureStatus
from ..core.models import GCSState, UAVState, LinkState
from ..core.validation import Validated, require, nonnegative


@dataclass(frozen=True, slots=True)
class LinkCondition(Validated):
    """Explicit current pair impairment; scheduling belongs to the caller."""
    quality_multiplier: float = 1.0
    latency_penalty_ms: float = 0.0
    outage: bool = False

    def __post_init__(self) -> None:
        super(LinkCondition, self).__post_init__()
        require(0 <= self.quality_multiplier <= 1, "quality multiplier must be in [0,1]")
        nonnegative(self.latency_penalty_ms)


@dataclass(frozen=True, slots=True)
class ChannelEvaluation(Validated):
    """Local adapter: unusable pairs have no LinkState and no finite ETX claim."""
    distance: float
    range_feasible: bool
    link: LinkState | None
    unavailable_reason: str | None = None

    @property
    def usable(self) -> bool:
        return self.link is not None

    @property
    def estimated_pdr(self) -> float:
        return self.link.estimated_pdr if self.link is not None else 0.0

    @property
    def packet_loss_probability(self) -> float:
        return 1.0 - self.estimated_pdr

    @property
    def etx(self) -> float | None:
        return self.link.etx if self.link is not None else None


def node_available(node: GCSState | UAVState) -> bool:
    """GCS is available; UAVs must be active and neither FAILED nor LOST."""
    require(isinstance(node, (GCSState, UAVState)), "expected existing GCS/UAV model")
    return isinstance(node, GCSState) or (
        node.active and node.failure_status not in (FailureStatus.FAILED, FailureStatus.LOST)
    )


@dataclass(frozen=True, slots=True)
class ChannelModel(Validated):
    """Use existing config: R=max_range, L=base_latency, p=packet_loss."""
    config: CommunicationConfig

    def evaluate(
        self,
        source: GCSState | UAVState,
        target: GCSState | UAVState,
        *,
        simulation_time: float,
        condition: LinkCondition | None = None,
    ) -> ChannelEvaluation:
        """Evaluate an unordered pair without touching either endpoint.

        q = multiplier / (1 + d/R)
        loss = 1 - (1-p)*q; PDR = 1-loss; latency = L*(1+d/R)+penalty.
        d=R is range-feasible. No positive-PDR threshold is imposed.
        """
        source_available = node_available(source)
        target_available = node_available(target)
        available = source_available and target_available
        require(source.id != target.id, "self-links are not supported")
        require(type(simulation_time) in (int, float) and isfinite(simulation_time)
                and simulation_time >= 0, "invalid simulation time")
        condition = LinkCondition() if condition is None else condition
        require(isinstance(condition, LinkCondition), "invalid link condition")
        distance = hypot(source.position.x - target.position.x,
                         source.position.y - target.position.y)
        require(isfinite(distance), "distance overflow")
        in_range = distance <= self.config.max_range
        if not available:
            return ChannelEvaluation(distance, in_range, None, "inactive_endpoint")
        if not in_range:
            return ChannelEvaluation(distance, False, None, "outside_range")
        if condition.outage:
            return ChannelEvaluation(distance, True, None, "outage")
        ratio = distance / self.config.max_range
        quality = condition.quality_multiplier / (1.0 + ratio)
        loss = 1.0 - (1.0 - self.config.packet_loss) * quality
        pdr = 1.0 - loss
        if pdr == 0.0:
            return ChannelEvaluation(distance, True, None, "zero_pdr")
        a, b = sorted((source.id, target.id))
        link = LinkState(
            source_id=a, target_id=b, distance=distance,
            packet_loss_probability=loss, estimated_pdr=pdr,
            latency_ms=self.config.base_latency * (1.0 + ratio) + condition.latency_penalty_ms,
            etx=1.0 / pdr, link_quality=quality,
            active=True, last_updated=simulation_time,
        )
        return ChannelEvaluation(distance, True, link)
