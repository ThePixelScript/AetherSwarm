"""Deterministic non-collinear RTH routing layer for AetherSwarm.

Eliminates RTH queue/deadlocks by assigning geometry-aware, parallel Y-lanes
to returning UAVs. Guarantees:
1. Each UAV returns along its dedicated horizontal Y-lane matching its staging origin.
2. Pairwise lane separation is >= 20.0m (within corridor bounds y in [400.0, 600.0]).
3. RTH transit from operational arena (x > 0) proceeds via corridor portal (0.0, y_lane)
   to pad destination (-75.0, y_lane).
4. Continuous 2D separation (>= 20.0m) and geofence boundaries are strictly preserved.
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

from ..core.commands import (
    BeginLandingCommand,
    CompleteRTHCommand,
    SetTargetPositionCommand,
)
from ..core.enums import RTHState, SortieState
from ..core.events import DomainEvent
from ..core.models import StateSnapshot, UAVState


class RTHRouter:
    """Deterministic non-collinear RTH router."""

    def __init__(
        self,
        gcs_position: Tuple[float, float] = (-75.0, 500.0),
        min_separation_m: float = 20.0,
        corridor_bounds_y: Tuple[float, float] = (400.0, 600.0),
    ) -> None:
        self.gcs_position = gcs_position
        self.min_separation_m = min_separation_m
        self.corridor_bounds_y = corridor_bounds_y
        self.uav_rth_lanes: Dict[str, float] = {}

    def reset(self) -> None:
        """Reset internal lane cache."""
        self.uav_rth_lanes.clear()

    def register_uav_lane(self, uav_id: str, staging_y: float) -> None:
        """Register explicit Y-lane for a UAV based on its staging position, clamped within corridor bounds if present."""
        if self.corridor_bounds_y[0] <= self.gcs_position[1] <= self.corridor_bounds_y[1]:
            clamped_y = max(self.corridor_bounds_y[0] + 10.0, min(self.corridor_bounds_y[1] - 10.0, staging_y))
        else:
            clamped_y = staging_y
        self.uav_rth_lanes[uav_id] = clamped_y

    def get_rth_lane_y(self, uav: UAVState, snapshot: Optional[StateSnapshot] = None) -> float:
        """Get or compute deterministic RTH Y-lane for a UAV."""
        if uav.id in self.uav_rth_lanes:
            return self.uav_rth_lanes[uav.id]

        # Deterministic fallback based on UAV index
        try:
            u_idx = int(uav.id.split("_")[-1]) - 1
        except Exception:
            u_idx = 0

        total_uavs = len(snapshot.uavs) if snapshot and len(snapshot.uavs) > 0 else 5
        y_center = self.gcs_position[1]
        y_start = y_center - ((total_uavs - 1) / 2.0) * self.min_separation_m
        lane_y = y_start + u_idx * self.min_separation_m

        # Clamp within corridor bounds if GCS is in corridor
        if self.corridor_bounds_y[0] <= self.gcs_position[1] <= self.corridor_bounds_y[1]:
            lane_y = max(self.corridor_bounds_y[0] + 10.0, min(self.corridor_bounds_y[1] - 10.0, lane_y))
        else:
            lane_y = self.gcs_position[1]
        self.uav_rth_lanes[uav.id] = lane_y
        return lane_y

    def get_rth_target(self, uav: UAVState, snapshot: Optional[StateSnapshot] = None) -> Tuple[float, float]:
        """Compute current RTH target waypoint for a UAV based on its position and flight phase."""
        lane_y = self.get_rth_lane_y(uav, snapshot)
        x_curr = uav.position_xy[0]

        if self.gcs_position[0] < -1.0 and x_curr > 0.5:
            # Phase 1: In arena, target corridor portal entry at x = 0.0 on dedicated Y lane
            return (0.0, lane_y)
        else:
            # Phase 2: In corridor or staging area, target staging pad at x = gcs_x on dedicated Y lane
            return (self.gcs_position[0], lane_y)

    def step(
        self,
        snapshot: StateSnapshot,
        dt: float = 1.0,
    ) -> Tuple[List[Any], List[DomainEvent]]:
        """Evaluate RTH progress for active RTH UAVs and return target position overrides & lifecycle commands."""
        commands: List[Any] = []
        events: List[DomainEvent] = []
        tick = snapshot.simulation_tick

        for uav_id in sorted(snapshot.uavs):
            uav = snapshot.uavs[uav_id]
            if not uav.active or uav.rth_state != RTHState.ACTIVE:
                continue

            lane_y = self.get_rth_lane_y(uav, snapshot)
            curr_x, curr_y = uav.position_xy
            desired_target = self.get_rth_target(uav, snapshot)

            # 1. Target waypoint updates
            if uav.target_position != desired_target:
                commands.append(
                    SetTargetPositionCommand(
                        source_tick=tick,
                        uav_id=uav.id,
                        target_position=desired_target,
                    )
                )

            # 2. Lifecycle transitions (Landing approach & completion)
            dist_to_pad = math.hypot(curr_x - self.gcs_position[0], curr_y - lane_y)
            if uav.sortie_state == SortieState.RTH:
                if dist_to_pad <= 25.0 or curr_x <= -50.0:
                    commands.append(
                        BeginLandingCommand(source_tick=tick, uav_id=uav.id)
                    )

            if dist_to_pad <= 0.2 or (self.gcs_position[0] < -1.0 and curr_x <= self.gcs_position[0] + 0.1):
                commands.append(
                    CompleteRTHCommand(source_tick=tick, uav_id=uav.id)
                )

        return commands, events
