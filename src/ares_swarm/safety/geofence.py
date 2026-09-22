"""Deterministic continuous-timestep 2D geofence enforcement layer."""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

from ..core.commands import StepPhysicsCommand
from ..core.constants import EPSILON
from ..core.enums import EventType
from ..core.events import DomainEvent
from ..core.kinematics import move_towards
from ..core.models import StateSnapshot, UAVState
from ..energy.battery import calculate_energy_cost
from .airspace import ChallengeAirspace, FlightPhase


class GeofenceEnforcer:
    """Enforces continuous 2D horizontal geofence containment for ChallengeAirspace."""

    def __init__(
        self,
        airspace: ChallengeAirspace,
        margin_m: float = 1.0,
    ) -> None:
        self.airspace = airspace
        self.margin_m = margin_m
        self.total_interventions: int = 0
        self.per_uav_interventions: Dict[str, int] = {}
        self.min_observed_clearance_m: float = float("inf")
        self._event_counter: int = 0

    def reset(self) -> None:
        """Reset internal counters."""
        self.total_interventions = 0
        self.per_uav_interventions.clear()
        self.min_observed_clearance_m = float("inf")
        self._event_counter = 0

    def compute_boundary_clearance(self, position_xy: Tuple[float, float]) -> float:
        """Compute Euclidean distance to the nearest exterior boundary of the authorized airspace union."""
        x, y = position_xy
        x_min_c, x_max_c = self.airspace.corridor_bounds_x
        y_min_c, y_max_c = self.airspace.corridor_bounds_y
        x_min_a, x_max_a = self.airspace.arena_bounds_x
        y_min_a, y_max_a = self.airspace.arena_bounds_y

        if x >= x_min_a:
            # In or near arena
            d_east = x_max_a - x
            d_south = y - y_min_a
            d_north = y_max_a - y
            # For west boundary: if y is within corridor mouth, opening is unobstructed
            if y_min_c <= y <= y_max_c:
                d_west = float("inf")
            else:
                d_west = x - x_min_a
            return max(0.0, min(d_east, d_south, d_north, d_west))
        else:
            # In corridor or staging pad
            d_north = y_max_c - y
            d_south = y - y_min_c
            # West boundary check: staging pad circular expansion
            dx_pad = x - self.airspace.staging_pad_center[0]
            dy_pad = y - self.airspace.staging_pad_center[1]
            dist_pad = math.hypot(dx_pad, dy_pad)
            d_pad = self.airspace.staging_pad_radius_m - dist_pad

            d_west = (x - x_min_c) if dist_pad > self.airspace.staging_pad_radius_m else max(x - x_min_c, d_pad)
            return max(0.0, min(d_north, d_south, d_west))

    def compute_effective_movement(
        self,
        uav: UAVState,
        configured_speed: float,
        dt: float,
        tick: int,
        sim_time: float,
    ) -> Tuple[Tuple[float, float], Tuple[float, float], Optional[DomainEvent]]:
        """Compute geofence-safe movement candidate (p_cand, v_cand) and optional intervention event.

        Does NOT mutate uav.target_position, task assignment, or autonomy state.
        """
        if uav.target_position is None:
            return (uav.position_xy, (0.0, 0.0), None)

        p0 = uav.position_xy
        p_target = uav.target_position
        curr_x, curr_y = p0
        tgt_x, tgt_y = p_target

        x_interface = self.airspace.corridor_bounds_x[1]  # Typically 0.0
        y_mouth_min = self.airspace.corridor_bounds_y[0] + self.margin_m
        y_mouth_max = self.airspace.corridor_bounds_y[1] - self.margin_m

        eff_target = p_target
        interv_type: Optional[str] = None
        relevant_boundary: Optional[str] = None

        # 1. Ingress: Corridor (x < x_interface) -> Arena (target_x >= x_interface)
        if curr_x < x_interface and tgt_x >= x_interface:
            denom = tgt_x - curr_x
            if abs(denom) > 1e-9:
                y_cross = curr_y + ((x_interface - curr_x) / denom) * (tgt_y - curr_y)
                if y_cross > y_mouth_max:
                    eff_target = (x_interface, y_mouth_max)
                    interv_type = "INGRESS_PORTAL_NORTH"
                    relevant_boundary = f"corridor_y_max_{self.airspace.corridor_bounds_y[1]}"
                elif y_cross < y_mouth_min:
                    eff_target = (x_interface, y_mouth_min)
                    interv_type = "INGRESS_PORTAL_SOUTH"
                    relevant_boundary = f"corridor_y_min_{self.airspace.corridor_bounds_y[0]}"

        # 2. Egress: Arena (curr_x >= x_interface) -> Corridor (target_x < x_interface)
        elif curr_x >= x_interface and tgt_x < x_interface:
            denom = tgt_x - curr_x
            if abs(denom) > 1e-9:
                y_cross = curr_y + ((x_interface - curr_x) / denom) * (tgt_y - curr_y)
                if y_cross > y_mouth_max:
                    eff_target = (x_interface, y_mouth_max)
                    interv_type = "EGRESS_PORTAL_NORTH"
                    relevant_boundary = f"corridor_y_max_{self.airspace.corridor_bounds_y[1]}"
                elif y_cross < y_mouth_min:
                    eff_target = (x_interface, y_mouth_min)
                    interv_type = "EGRESS_PORTAL_SOUTH"
                    relevant_boundary = f"corridor_y_min_{self.airspace.corridor_bounds_y[0]}"

        # Compute candidate step toward eff_target
        p1, v1 = move_towards(
            current=p0,
            target=eff_target,
            speed=configured_speed,
            dt=dt,
        )

        # 3. Intra-zone continuous boundary containment verification
        t_max = 1.0

        if curr_x >= x_interface and eff_target[0] >= x_interface:
            # Intra-arena movement: check arena bounding box
            x_min_a, x_max_a = self.airspace.arena_bounds_x
            y_min_a, y_max_a = self.airspace.arena_bounds_y
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]

            if p1[0] > x_max_a and abs(dx) > 1e-9:
                t_max = min(t_max, max(0.0, (x_max_a - p0[0]) / dx))
                interv_type = "BOUNDARY_TRUNCATE"
                relevant_boundary = "arena_x_max"
            elif p1[0] < x_min_a and abs(dx) > 1e-9:
                t_max = min(t_max, max(0.0, (x_min_a - p0[0]) / dx))
                interv_type = "BOUNDARY_TRUNCATE"
                relevant_boundary = "arena_x_min"

            if p1[1] > y_max_a and abs(dy) > 1e-9:
                t_max = min(t_max, max(0.0, (y_max_a - p0[1]) / dy))
                interv_type = "BOUNDARY_TRUNCATE"
                relevant_boundary = "arena_y_max"
            elif p1[1] < y_min_a and abs(dy) > 1e-9:
                t_max = min(t_max, max(0.0, (y_min_a - p0[1]) / dy))
                interv_type = "BOUNDARY_TRUNCATE"
                relevant_boundary = "arena_y_min"

        elif curr_x < x_interface and eff_target[0] <= x_interface:
            # Intra-corridor / staging pad movement: check corridor bounds
            x_min_c, x_max_c = self.airspace.corridor_bounds_x
            y_min_c, y_max_c = self.airspace.corridor_bounds_y
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]

            if p1[1] > y_max_c and abs(dy) > 1e-9:
                t_max = min(t_max, max(0.0, (y_max_c - p0[1]) / dy))
                interv_type = "BOUNDARY_TRUNCATE"
                relevant_boundary = "corridor_y_max"
            elif p1[1] < y_min_c and abs(dy) > 1e-9:
                t_max = min(t_max, max(0.0, (y_min_c - p0[1]) / dy))
                interv_type = "BOUNDARY_TRUNCATE"
                relevant_boundary = "corridor_y_min"

            # Check western boundary and staging pad
            if p1[0] < x_min_c and abs(dx) > 1e-9:
                dist_pad_target = math.hypot(
                    p1[0] - self.airspace.staging_pad_center[0],
                    p1[1] - self.airspace.staging_pad_center[1],
                )
                if dist_pad_target > self.airspace.staging_pad_radius_m:
                    t_max = min(t_max, max(0.0, (x_min_c - p0[0]) / dx))
                    interv_type = "BOUNDARY_TRUNCATE"
                    relevant_boundary = "corridor_x_min"

        # Apply truncation if required
        if t_max < 1.0:
            if t_max < 1e-4:
                p1 = p0
                v1 = (0.0, 0.0)
                interv_type = "BOUNDARY_HOLD"
            else:
                p1 = (p0[0] + t_max * (p1[0] - p0[0]), p0[1] + t_max * (p1[1] - p0[1]))
                v1 = (t_max * v1[0], t_max * v1[1])

        # Track clearance
        clearance = self.compute_boundary_clearance(p1)
        self.min_observed_clearance_m = min(self.min_observed_clearance_m, clearance)

        event = None
        if interv_type is not None:
            self.total_interventions += 1
            self.per_uav_interventions[uav.id] = self.per_uav_interventions.get(uav.id, 0) + 1
            self._event_counter += 1

            event = DomainEvent.create(
                simulation_tick=tick,
                simulation_time=sim_time,
                event_type=EventType.GEOFENCE_INTERVENTION,
                entity_id=uav.id,
                payload={
                    "uav_id": uav.id,
                    "original_position": [round(coord, 3) for coord in p0],
                    "nominal_target": [round(coord, 3) for coord in p_target],
                    "adjusted_position": [round(coord, 3) for coord in p1],
                    "intervention_type": interv_type,
                    "relevant_boundary": relevant_boundary or "airspace_portal",
                    "margin": self.margin_m,
                },
                sequence=self._event_counter,
            )

        return (p1, v1, event)

    def enforce_step(
        self,
        snapshot: StateSnapshot,
        configured_speed: float,
        dt: float,
        idle_rate: float,
        movement_rate: float,
    ) -> Tuple[List[StepPhysicsCommand], List[DomainEvent]]:
        """Compute standalone geofence-enforced StepPhysicsCommands when separation enforcer is disabled."""
        commands: List[StepPhysicsCommand] = []
        events: List[DomainEvent] = []

        for uav_id in sorted(snapshot.uavs):
            uav = snapshot.uavs[uav_id]
            if not uav.active or uav.failure_state.value == "FAILED":
                continue

            cand_pos, cand_vel, event = self.compute_effective_movement(
                uav=uav,
                configured_speed=configured_speed,
                dt=dt,
                tick=snapshot.simulation_tick,
                sim_time=snapshot.simulation_time,
            )
            if event is not None:
                events.append(event)

            dist_moved = math.hypot(
                cand_pos[0] - uav.position_xy[0],
                cand_pos[1] - uav.position_xy[1],
            )
            energy_cost = calculate_energy_cost(
                dt=dt,
                distance=dist_moved,
                idle_rate=idle_rate,
                movement_rate=movement_rate,
            )
            commands.append(
                StepPhysicsCommand(
                    source_tick=snapshot.simulation_tick,
                    uav_id=uav_id,
                    new_position_xy=cand_pos,
                    new_velocity_xy=cand_vel,
                    delta_energy=energy_cost,
                )
            )

        return commands, events
