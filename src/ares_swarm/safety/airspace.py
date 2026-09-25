"""Formal Challenge Airspace model for AetherSwarm UAV-X compliance layer.

Defines the composite geometry (Operational Arena, Staging Pad, Transit Corridor)
and state-dependent flight phase rules under Challenge Assumptions V1.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Tuple

from ..core.constants import EPSILON
from ..core.enums import Role, RTHState
from ..core.models import UAVState


class FlightPhase(str, Enum):
    """Flight phase of a UAV within the challenge airspace."""
    STAGING = "STAGING"
    INGRESS = "INGRESS"
    MISSION = "MISSION"
    EGRESS = "EGRESS"
    LANDED = "LANDED"


@dataclass(frozen=True)
class ChallengeAirspace:
    """Formal composite airspace model for UAV-X Round 1 challenge missions.

    Zones:
    1. Operational Arena: [arena_bounds_x] x [arena_bounds_y], ceiling max_height.
    2. Staging Pad: Circular pad centered at staging_pad_center with staging_pad_radius_m.
    3. Transit Corridor: Rectangular channel [corridor_bounds_x] x [corridor_bounds_y].
    """
    arena_bounds_x: Tuple[float, float] = (0.0, 1000.0)
    arena_bounds_y: Tuple[float, float] = (0.0, 1000.0)
    max_height: float = 100.0
    staging_pad_center: Tuple[float, float] = (-75.0, 500.0)
    staging_pad_radius_m: float = 15.0
    corridor_bounds_x: Tuple[float, float] = (-75.0, 0.0)
    corridor_bounds_y: Tuple[float, float] = (400.0, 600.0)

    def is_in_staging_pad(self, position_xy: Tuple[float, float]) -> bool:
        """Check if 2D position is within the authorized staging pad radius."""
        dx = position_xy[0] - self.staging_pad_center[0]
        dy = position_xy[1] - self.staging_pad_center[1]
        return math.hypot(dx, dy) <= self.staging_pad_radius_m + EPSILON

    def is_in_staging_area(self, position_xy: Tuple[float, float]) -> bool:
        """Check if 2D position is within authorized staging pad or linear staging corridor.

        Accommodates up to 8 UAVs staged with >= 20m separation centered at
        staging_pad_center, strictly outside the operational arena (x <= 0).
        """
        if self.is_in_staging_pad(position_xy):
            return True
        x, y = position_xy
        x_c, y_c = self.staging_pad_center
        # Staging line strictly along west operational center x_c (e.g. -75.0)
        if abs(x - x_c) <= 5.0 and abs(y - y_c) <= 150.0:
            return True
        y_min_c, y_max_c = self.corridor_bounds_y
        return (x_c - 5.0 <= x <= EPSILON) and (y_min_c - EPSILON <= y <= y_max_c + EPSILON)

    def is_in_corridor(self, position_xy: Tuple[float, float]) -> bool:
        """Check if 2D position is within the transit corridor."""
        x, y = position_xy
        x_min, x_max = self.corridor_bounds_x
        y_min, y_max = self.corridor_bounds_y
        return (x_min - EPSILON <= x <= x_max + EPSILON) and (y_min - EPSILON <= y <= y_max + EPSILON)

    def is_in_arena(self, position_xy: Tuple[float, float]) -> bool:
        """Check if 2D position is within the primary operational arena."""
        x, y = position_xy
        x_min, x_max = self.arena_bounds_x
        y_min, y_max = self.arena_bounds_y
        return (x_min - EPSILON <= x <= x_max + EPSILON) and (y_min - EPSILON <= y <= y_max + EPSILON)

    def is_in_authorized_union(self, position_xy: Tuple[float, float]) -> bool:
        """Check if position is in any authorized airspace zone."""
        return (
            self.is_in_staging_area(position_xy)
            or self.is_in_corridor(position_xy)
            or self.is_in_arena(position_xy)
        )

    def determine_flight_phase(
        self,
        uav: UAVState,
        prev_phase: Optional[FlightPhase] = None,
    ) -> FlightPhase:
        """Determine flight phase for a UAV given its state and history."""
        # If UAV is inactive or RTH complete, it is considered landed
        if not uav.active or uav.rth_state == RTHState.COMPLETE:
            return FlightPhase.LANDED

        # If UAV is returning to home, it is in egress/RTH
        if uav.rth_state in (RTHState.REQUIRED, RTHState.ACTIVE):
            return FlightPhase.EGRESS

        # If UAV is in the corridor or staging area outside operational arena
        if not self.is_in_arena(uav.position_xy):
            if self.is_in_staging_area(uav.position_xy) and uav.role == Role.IDLE and uav.assigned_task_id is None and prev_phase in (None, FlightPhase.STAGING, FlightPhase.LANDED):
                return FlightPhase.STAGING
            return FlightPhase.INGRESS

        return FlightPhase.MISSION

    def validate_position(
        self,
        position_xy: Tuple[float, float],
        flight_phase: FlightPhase,
    ) -> Tuple[bool, Optional[str]]:
        """Validate whether position is authorized for the given flight phase.

        Returns (is_valid, failure_reason).
        """
        # First check absolute containment within the union of authorized zones
        if not self.is_in_authorized_union(position_xy):
            return False, f"Position ({position_xy[0]:.2f}, {position_xy[1]:.2f}) outside authorized airspace union"

        # Phase-specific boundary rules
        if flight_phase == FlightPhase.STAGING:
            if not self.is_in_staging_area(position_xy):
                return False, f"UAV in STAGING phase outside staging area ({self.staging_pad_radius_m}m / staging line)"

        elif flight_phase == FlightPhase.INGRESS:
            # Ingress may use corridor, staging area, or enter the arena boundary
            if not (self.is_in_corridor(position_xy) or self.is_in_staging_area(position_xy) or self.is_in_arena(position_xy)):
                return False, "UAV in INGRESS phase outside corridor or staging pad"

        elif flight_phase == FlightPhase.MISSION:
            # Active mission movement must remain strictly inside the operational arena
            if not self.is_in_arena(position_xy):
                return False, f"UAV in MISSION phase departed operational arena: ({position_xy[0]:.2f}, {position_xy[1]:.2f})"

        elif flight_phase == FlightPhase.EGRESS:
            # Egress may transit from arena through corridor to staging area
            if not (self.is_in_arena(position_xy) or self.is_in_corridor(position_xy) or self.is_in_staging_area(position_xy)):
                return False, "UAV in EGRESS phase outside authorized transit path"

        elif flight_phase == FlightPhase.LANDED:
            # Final landing must occur in authorized staging area
            if not self.is_in_staging_area(position_xy):
                return False, f"UAV landed at ({position_xy[0]:.2f}, {position_xy[1]:.2f}) outside staging pad / staging area"

        return True, None
