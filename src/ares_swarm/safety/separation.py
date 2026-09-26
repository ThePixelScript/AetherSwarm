"""Deterministic continuous-timestep 2D separation enforcement layer."""
from __future__ import annotations

import math
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from ..core.commands import StepPhysicsCommand
from ..core.constants import EPSILON
from ..core.enums import EventType, FailureState, RTHState, SortieState
from ..core.events import DomainEvent
from ..core.kinematics import move_towards
from ..core.models import StateSnapshot, TaskState, UAVState
from ..energy.battery import calculate_energy_cost
from .airspace import ChallengeAirspace, FlightPhase


def min_continuous_separation(
    p_i: Tuple[float, float],
    v_i: Tuple[float, float],
    p_j: Tuple[float, float],
    v_j: Tuple[float, float],
    dt: float,
) -> float:
    """Compute exact minimum Euclidean 2D distance between two linear trajectories over t in [0, dt]."""
    r0_x = p_i[0] - p_j[0]
    r0_y = p_i[1] - p_j[1]
    v_rel_x = v_i[0] - v_j[0]
    v_rel_y = v_i[1] - v_j[1]

    a = v_rel_x**2 + v_rel_y**2
    b = 2.0 * (r0_x * v_rel_x + r0_y * v_rel_y)
    c = r0_x**2 + r0_y**2

    if a < 1e-12:
        return math.sqrt(max(0.0, c))

    t_star = -b / (2.0 * a)

    # Evaluate candidate extrema: endpoints and interior vertex (if within (0, dt))
    d0_sq = c
    d_dt_sq = a * dt**2 + b * dt + c
    min_sq = min(d0_sq, d_dt_sq)

    if 0.0 < t_star < dt:
        vertex_sq = c - (b**2) / (4.0 * a)
        min_sq = min(min_sq, vertex_sq)

    return math.sqrt(max(0.0, min_sq))


class SeparationEnforcer:
    """Enforces continuous 2D horizontal separation (>= 20.0m) over the entire simulation timestep."""

    def __init__(
        self,
        min_separation_m: float = 20.0,
        gcs_position: Tuple[float, float] = (-75.0, 500.0),
        staging_pad_radius_m: float = 15.0,
        numerical_safety_buffer_m: float = 0.005,
    ) -> None:
        self.min_separation_m = min_separation_m
        self.gcs_position = gcs_position
        self.staging_pad_radius_m = staging_pad_radius_m
        self.numerical_safety_buffer_m = numerical_safety_buffer_m
        self.total_interventions: int = 0
        self.per_uav_interventions: Dict[str, int] = {}
        self._event_counter: int = 0

    def reset(self) -> None:
        """Reset internal intervention counts and counters."""
        self.total_interventions = 0
        self.per_uav_interventions.clear()
        self._event_counter = 0

    def get_priority_key(
        self,
        uav: UAVState,
        tasks: Mapping[str, TaskState],
    ) -> Tuple[int, float, str]:
        """Compute deterministic priority tuple: (tier, remaining_distance_to_target, uav_id).

        Tiers (lower number = higher priority):
        0: Active task service (co-located at task position <= 0.05m)
        1: Transit to assigned task
        2: Return-to-Home (RTH)
        3: Idle / Standby
        """
        if uav.assigned_task_id and uav.assigned_task_id in tasks:
            task = tasks[uav.assigned_task_id]
            dist_task = math.hypot(
                uav.position_xy[0] - task.position_xy[0],
                uav.position_xy[1] - task.position_xy[1],
            )
            if dist_task <= 0.05:
                return (0, 0.0, uav.id)
            return (1, round(dist_task, 4), uav.id)

        if uav.rth_state == RTHState.ACTIVE:
            dist_gcs = math.hypot(
                uav.position_xy[0] - self.gcs_position[0],
                uav.position_xy[1] - self.gcs_position[1],
            )
            return (2, round(dist_gcs, 4), uav.id)

        rem_dist = 0.0
        if uav.target_position:
            rem_dist = math.hypot(
                uav.position_xy[0] - uav.target_position[0],
                uav.position_xy[1] - uav.target_position[1],
            )
        return (3, round(rem_dist, 4), uav.id)

    def is_landed_at_gcs(self, uav: UAVState, airspace: Optional[ChallengeAirspace] = None) -> bool:
        """Determine if UAV is safely landed and parked on the staging pad/GCS."""
        in_staging = (
            airspace.is_in_staging_area(uav.position_xy)
            if airspace is not None
            else (abs(uav.position_xy[0] - self.gcs_position[0]) <= 5.0 and abs(uav.position_xy[1] - self.gcs_position[1]) <= 150.0)
        )
        if not in_staging:
            return False

        if uav.failure_state == FailureState.FAILED:
            return True

        if uav.rth_state == RTHState.COMPLETE or uav.sortie_state in (SortieState.LANDED, SortieState.RECHARGING):
            return True

        if uav.target_position is None or not uav.active:
            return True

        return False

    def enforce_step(
        self,
        snapshot: StateSnapshot,
        configured_speed: float,
        dt: float,
        idle_rate: float,
        movement_rate: float,
        airspace: Optional[ChallengeAirspace] = None,
        flight_phases: Optional[Mapping[str, FlightPhase]] = None,
        geofence_enforcer: Optional[Any] = None,
    ) -> Tuple[List[StepPhysicsCommand], List[DomainEvent]]:
        """Compute safe, continuous-timestep StepPhysicsCommands guaranteeing separation >= min_separation_m."""
        commands: List[StepPhysicsCommand] = []
        events: List[DomainEvent] = []

        all_uavs = snapshot.uavs
        tasks = snapshot.tasks

        # 1. Identify active UAVs moving this tick vs stationary/failed/landed obstacles
        moving_uavs: List[UAVState] = []
        static_obstacles: List[Tuple[str, Tuple[float, float]]] = []

        for uav_id in sorted(all_uavs):
            uav = all_uavs[uav_id]
            if not uav.active or uav.failure_state == FailureState.FAILED:
                # Landed UAVs at GCS are exempt from airborne separation
                if not self.is_landed_at_gcs(uav, airspace):
                    # Failed in-flight UAV is a static obstacle
                    static_obstacles.append((uav.id, uav.position_xy))
                continue

            if self.is_landed_at_gcs(uav, airspace):
                continue

            if uav.target_position is None:
                # Active but hovering with no target
                static_obstacles.append((uav.id, uav.position_xy))
                energy_cost = calculate_energy_cost(
                    dt=dt,
                    distance=0.0,
                    idle_rate=idle_rate,
                    movement_rate=movement_rate,
                )
                commands.append(
                    StepPhysicsCommand(
                        source_tick=snapshot.simulation_tick,
                        uav_id=uav.id,
                        new_position_xy=uav.position_xy,
                        new_velocity_xy=(0.0, 0.0),
                        delta_energy=energy_cost,
                    )
                )
            else:
                moving_uavs.append(uav)

        # 2. Sort moving UAVs by deterministic priority
        moving_uavs.sort(key=lambda u: self.get_priority_key(u, tasks))

        # 3. Sequentially evaluate and approve continuous trajectories
        approved_trajectories: List[Tuple[str, Tuple[float, float], Tuple[float, float]]] = []
        committed_steps: Dict[str, StepPhysicsCommand] = {}
        safe_dist_threshold = self.min_separation_m + self.numerical_safety_buffer_m

        for idx, uav in enumerate(moving_uavs):
            if geofence_enforcer is not None:
                nominal_pos, nominal_vel, geo_event = geofence_enforcer.compute_effective_movement(
                    uav=uav,
                    configured_speed=configured_speed,
                    dt=dt,
                    tick=snapshot.simulation_tick,
                    sim_time=snapshot.simulation_time,
                )
                if geo_event is not None:
                    events.append(geo_event)
            else:
                nominal_pos, nominal_vel = move_towards(
                    current=uav.position_xy,
                    target=uav.target_position,
                    speed=configured_speed,
                    dt=dt,
                )
            nom_v = (
                (nominal_pos[0] - uav.position_xy[0]) / dt,
                (nominal_pos[1] - uav.position_xy[1]) / dt,
            )
            nom_dist = math.hypot(
                nominal_pos[0] - uav.position_xy[0],
                nominal_pos[1] - uav.position_xy[1],
            )

            max_alpha = 1.0

            # Obstacles to test against:
            # A. Already approved trajectories over [0, dt]
            # B. Static obstacles over [0, dt] (vel = 0)
            # C. Unprocessed moving UAVs (conservative non-encroachment at current position)
            obstacles_to_check: List[Tuple[str, Tuple[float, float], Tuple[float, float]]] = []
            for app_id, app_p, app_v in approved_trajectories:
                obstacles_to_check.append((app_id, app_p, app_v))
            for obs_id, obs_p in static_obstacles:
                obstacles_to_check.append((obs_id, obs_p, (0.0, 0.0)))
            for other_uav in moving_uavs[idx + 1:]:
                obstacles_to_check.append((other_uav.id, other_uav.position_xy, (0.0, 0.0)))

            def eval_vel_safe(test_v: Tuple[float, float], alpha: float) -> Tuple[bool, Optional[str], float]:
                cand_v = (alpha * test_v[0], alpha * test_v[1])
                worst_min_sep = float("inf")
                worst_partner = None

                for obs_id, obs_p, obs_v in obstacles_to_check:
                    sep = min_continuous_separation(
                        uav.position_xy, cand_v,
                        obs_p, obs_v,
                        dt,
                    )
                    d_init = math.hypot(uav.position_xy[0] - obs_p[0], uav.position_xy[1] - obs_p[1])
                    pair_thresh = safe_dist_threshold if d_init >= safe_dist_threshold else (self.min_separation_m if d_init >= self.min_separation_m else d_init)
                    if sep < pair_thresh - EPSILON:
                        return False, obs_id, sep
                    if sep < worst_min_sep:
                        worst_min_sep = sep
                        worst_partner = obs_id

                return True, worst_partner, worst_min_sep

            def find_best_alpha(test_v: Tuple[float, float]) -> Tuple[float, Optional[str]]:
                safe, partner, _ = eval_vel_safe(test_v, 1.0)
                if safe:
                    return 1.0, partner
                low = 0.0
                high = 1.0
                last_p = partner
                for _ in range(25):
                    mid = (low + high) / 2.0
                    safe_mid, p, _ = eval_vel_safe(test_v, mid)
                    if safe_mid:
                        low = mid
                    else:
                        high = mid
                        last_p = p
                return low, last_p

            chosen_alpha, partner_id = find_best_alpha(nom_v)
            best_v = nom_v
            intervention_type = None

            if chosen_alpha < 0.5 and (nom_v[0] != 0.0 or nom_v[1] != 0.0):
                # Direct path blocked: evaluate detour steering angles
                speed = math.hypot(nom_v[0], nom_v[1])
                base_angle = math.atan2(nom_v[1], nom_v[0])
                best_progress = chosen_alpha

                for delta_deg in [25.0, -25.0, 50.0, -50.0, 75.0, -75.0]:
                    rad = math.radians(delta_deg)
                    cand_angle = base_angle + rad
                    cand_v = (speed * math.cos(cand_angle), speed * math.sin(cand_angle))
                    cand_alpha, cand_partner = find_best_alpha(cand_v)
                    cand_progress = cand_alpha * math.cos(rad)
                    if cand_alpha >= 0.4 and cand_progress > best_progress:
                        best_progress = cand_progress
                        best_v = cand_v
                        partner_id = cand_partner
                        chosen_alpha = cand_alpha
                        intervention_type = "DETOUR"

            if intervention_type is None:
                if chosen_alpha < 1e-4:
                    chosen_alpha = 0.0
                    intervention_type = "HOLD"
                elif chosen_alpha < 1.0:
                    intervention_type = "TRUNCATE"

            actual_pos = (
                uav.position_xy[0] + chosen_alpha * best_v[0] * dt,
                uav.position_xy[1] + chosen_alpha * best_v[1] * dt,
            )
            actual_vel = (
                chosen_alpha * best_v[0],
                chosen_alpha * best_v[1],
            )

            # Airspace validation if explicitly provided
            if airspace is not None and flight_phases is not None and uav.id in flight_phases:
                phase = flight_phases[uav.id]
                is_valid, _ = airspace.validate_position(actual_pos, phase)
                if not is_valid:
                    chosen_alpha = 0.0
                    actual_pos = uav.position_xy
                    actual_vel = (0.0, 0.0)
                    intervention_type = "HOLD"

            actual_dist = math.hypot(
                actual_pos[0] - uav.position_xy[0],
                actual_pos[1] - uav.position_xy[1],
            )

            approved_trajectories.append((uav.id, uav.position_xy, actual_vel))

            energy_cost = calculate_energy_cost(
                dt=dt,
                distance=actual_dist,
                idle_rate=idle_rate,
                movement_rate=movement_rate,
            )

            committed_steps[uav.id] = StepPhysicsCommand(
                source_tick=snapshot.simulation_tick,
                uav_id=uav.id,
                new_position_xy=actual_pos,
                new_velocity_xy=actual_vel,
                delta_energy=energy_cost,
            )

            if intervention_type is not None:
                self.total_interventions += 1
                self.per_uav_interventions[uav.id] = self.per_uav_interventions.get(uav.id, 0) + 1
                self._event_counter += 1

                _, _, resulting_sep = eval_vel_safe(best_v, chosen_alpha)
                events.append(
                    DomainEvent.create(
                        simulation_tick=snapshot.simulation_tick,
                        simulation_time=snapshot.simulation_time,
                        event_type=EventType.SEPARATION_INTERVENTION,
                        entity_id=uav.id,
                        payload={
                            "uav_id": uav.id,
                            "partner_id": partner_id,
                            "nominal_movement_m": round(nom_dist, 3),
                            "actual_movement_m": round(actual_dist, 3),
                            "resulting_min_separation_m": round(resulting_sep, 3),
                            "intervention_type": intervention_type,
                            "alpha": round(chosen_alpha, 4),
                        },
                        sequence=self._event_counter,
                    )
                )

        # Assemble commands in standard sorted uav_id order
        for uav_id in sorted(all_uavs):
            if uav_id in committed_steps:
                commands.append(committed_steps[uav_id])

        return commands, events
