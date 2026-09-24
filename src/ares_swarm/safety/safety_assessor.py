"""Deterministic safety assessment, constraint verification, and RTH prevention."""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

from ..core.commands import StartRTHCommand
from ..core.constants import EPSILON
from ..core.enums import Role, RTHState
from ..core.models import StateSnapshot, UAVState
from ..interfaces.communication import NetworkAnalysis
from .airspace import ChallengeAirspace, FlightPhase


@dataclass(frozen=True)
class SafetyViolation:
    tick: int
    simulation_time: float
    violation_type: str  # "GEOFENCE", "SEPARATION", "BATTERY_EXHAUSTION", "FLIGHT_DURATION", "LANDING_LOCATION", "RELAUNCH_PROHIBITED"
    entity_ids: tuple[str, ...]
    details: str
    severity: str = "CRITICAL"  # "WARNING", "CRITICAL"


@dataclass
class UAVFlightRecord:
    """Tracks flight duration and lifecycle for a single UAV."""
    uav_id: str
    takeoff_time: Optional[float] = None
    landing_time: Optional[float] = None
    current_sortie_duration_s: float = 0.0
    cumulative_airborne_s: float = 0.0
    is_airborne: bool = False
    landing_position: Optional[Tuple[float, float]] = None


@dataclass
class SafetyReport:
    violations: list[SafetyViolation] = field(default_factory=list)
    _recorded_keys: set[tuple[int, str, tuple[str, ...]]] = field(default_factory=set)
    min_observed_separation_m: float = float("inf")
    geofence_violations_count: int = 0
    separation_violations_count: int = 0
    battery_exhaustions_count: int = 0
    flight_duration_violations_count: int = 0
    landing_violations_count: int = 0
    relaunch_violations_count: int = 0
    max_observed_sortie_duration_s: float = 0.0
    uav_flight_records: dict[str, UAVFlightRecord] = field(default_factory=dict)

    def record_violation(self, violation: SafetyViolation) -> None:
        key = (violation.tick, violation.violation_type, tuple(sorted(violation.entity_ids)))
        if key in self._recorded_keys:
            return
        self._recorded_keys.add(key)
        self.violations.append(violation)
        if violation.violation_type == "GEOFENCE":
            self.geofence_violations_count += 1
        elif violation.violation_type == "SEPARATION":
            self.separation_violations_count += 1
        elif violation.violation_type == "BATTERY_EXHAUSTION":
            self.battery_exhaustions_count += 1
        elif violation.violation_type == "FLIGHT_DURATION":
            self.flight_duration_violations_count += 1
        elif violation.violation_type == "LANDING_LOCATION":
            self.landing_violations_count += 1
        elif violation.violation_type == "RELAUNCH_PROHIBITED":
            self.relaunch_violations_count += 1


class SafetyAssessor:
    """Deterministic assessor for geofence, separation, battery, sortie duration, and single-sortie constraints."""

    def __init__(
        self,
        arena_bounds_x: tuple[float, float] = (-500.0, 500.0),
        arena_bounds_y: tuple[float, float] = (-500.0, 500.0),
        min_separation_m: float = 20.0,
        gcs_position: tuple[float, float] = (0.0, 0.0),
        rth_energy_buffer: float = 1.2,
        airspace: Optional[ChallengeAirspace] = None,
        max_sortie_duration_s: float = 1200.0,
        rth_safety_margin_s: float = 15.0,
        enforce_sortie_limit: bool = False,
        enforce_single_sortie: bool = False,
    ):
        self.arena_bounds_x = arena_bounds_x
        self.arena_bounds_y = arena_bounds_y
        self.min_separation_m = min_separation_m
        self.gcs_position = gcs_position
        self.rth_energy_buffer = rth_energy_buffer
        self.airspace = airspace
        self.max_sortie_duration_s = max_sortie_duration_s
        self.rth_safety_margin_s = rth_safety_margin_s
        self.enforce_sortie_limit = enforce_sortie_limit
        self.enforce_single_sortie = enforce_single_sortie
        self.report = SafetyReport()
        self._uav_flight_phases: dict[str, FlightPhase] = {}

    def reset(self) -> None:
        """Reset internal report and state caches."""
        self.report = SafetyReport()
        self._uav_flight_phases.clear()

    def assess_snapshot(
        self,
        snapshot: StateSnapshot,
        network_analysis: Optional[NetworkAnalysis] = None,
    ) -> list[SafetyViolation]:
        """Check all safety constraints for the current snapshot."""
        tick = snapshot.simulation_tick
        sim_time = snapshot.simulation_time
        active_uavs = [u for u in sorted(snapshot.uavs.values(), key=lambda x: x.id) if u.active]
        violations: list[SafetyViolation] = []

        # 1. Geofence verification
        if self.airspace is not None:
            # Challenge Airspace: state-dependent evaluation
            for u in sorted(snapshot.uavs.values(), key=lambda x: x.id):
                prev_phase = self._uav_flight_phases.get(u.id)
                phase = self.airspace.determine_flight_phase(u, prev_phase)
                self._uav_flight_phases[u.id] = phase

                is_valid, reason = self.airspace.validate_position(u.position_xy, phase)
                if not is_valid:
                    v = SafetyViolation(
                        tick=tick,
                        simulation_time=sim_time,
                        violation_type="GEOFENCE",
                        entity_ids=(u.id,),
                        details=f"UAV {u.id} in phase {phase.value}: {reason}",
                        severity="CRITICAL",
                    )
                    violations.append(v)
                    self.report.record_violation(v)
        else:
            # Legacy geofence verification (for UAVs not returning to GCS outside arena)
            min_x, max_x = self.arena_bounds_x
            min_y, max_y = self.arena_bounds_y

            for u in active_uavs:
                dist_gcs = math.hypot(u.position_xy[0] - self.gcs_position[0], u.position_xy[1] - self.gcs_position[1])
                if u.rth_state == RTHState.ACTIVE or dist_gcs <= 1.0:
                    continue

                x, y = u.position_xy
                if x < min_x - EPSILON or x > max_x + EPSILON or y < min_y - EPSILON or y > max_y + EPSILON:
                    v = SafetyViolation(
                        tick=tick,
                        simulation_time=sim_time,
                        violation_type="GEOFENCE",
                        entity_ids=(u.id,),
                        details=f"UAV {u.id} at ({x:.2f}, {y:.2f}) outside arena bounds [{min_x}, {max_x}] x [{min_y}, {max_y}]",
                        severity="CRITICAL",
                    )
                    violations.append(v)
                    self.report.record_violation(v)

        # 2. Inter-UAV separation verification
        for i in range(len(active_uavs)):
            for j in range(i + 1, len(active_uavs)):
                u1 = active_uavs[i]
                u2 = active_uavs[j]

                # Landed UAVs parked at GCS / Staging Pad are exempt from in-flight airborne separation checks
                d1_gcs = math.hypot(u1.position_xy[0] - self.gcs_position[0], u1.position_xy[1] - self.gcs_position[1])
                d2_gcs = math.hypot(u2.position_xy[0] - self.gcs_position[0], u2.position_xy[1] - self.gcs_position[1])

                pad_radius = self.airspace.staging_pad_radius_m if self.airspace else 1.0
                if (d1_gcs <= pad_radius and u1.rth_state == RTHState.COMPLETE) or (d2_gcs <= pad_radius and u2.rth_state == RTHState.COMPLETE):
                    continue
                if self.airspace is None and (d1_gcs <= 1.0 or d2_gcs <= 1.0):
                    continue

                dx = u1.position_xy[0] - u2.position_xy[0]
                dy = u1.position_xy[1] - u2.position_xy[1]
                dist = math.hypot(dx, dy)

                if dist < self.report.min_observed_separation_m:
                    self.report.min_observed_separation_m = dist

                if dist < self.min_separation_m - EPSILON:
                    v = SafetyViolation(
                        tick=tick,
                        simulation_time=sim_time,
                        violation_type="SEPARATION",
                        entity_ids=(u1.id, u2.id),
                        details=f"Inter-UAV distance between {u1.id} and {u2.id} is {dist:.2f}m < {self.min_separation_m}m",
                        severity="CRITICAL",
                    )
                    violations.append(v)
                    self.report.record_violation(v)

        # 3. Battery exhaustion verification
        for u in active_uavs:
            dist_gcs = math.hypot(u.position_xy[0] - self.gcs_position[0], u.position_xy[1] - self.gcs_position[1])
            if u.battery_energy <= EPSILON and dist_gcs > 1.0:
                v = SafetyViolation(
                    tick=tick,
                    simulation_time=sim_time,
                    violation_type="BATTERY_EXHAUSTION",
                    entity_ids=(u.id,),
                    details=f"UAV {u.id} battery exhausted ({u.battery_energy:.2f} Wh) at distance {dist_gcs:.2f}m from GCS",
                    severity="CRITICAL",
                )
                violations.append(v)
                self.report.record_violation(v)

        # 4. Flight time, Sortie Duration, and Single-Sortie tracking
        for u in sorted(snapshot.uavs.values(), key=lambda x: x.id):
            rec = self.report.uav_flight_records.get(u.id)
            if rec is None:
                rec = UAVFlightRecord(uav_id=u.id)
                self.report.uav_flight_records[u.id] = rec

            dist_gcs = math.hypot(u.position_xy[0] - self.gcs_position[0], u.position_xy[1] - self.gcs_position[1])
            speed = math.hypot(u.velocity_xy[0], u.velocity_xy[1])
            pad_radius = self.airspace.staging_pad_radius_m if self.airspace else 1.0

            # Single-sortie policy enforcement: prohibit relaunch after landing
            if self.enforce_single_sortie and rec.landing_time is not None and not rec.is_airborne:
                if u.active and (speed > EPSILON or dist_gcs > pad_radius):
                    v = SafetyViolation(
                        tick=tick,
                        simulation_time=sim_time,
                        violation_type="RELAUNCH_PROHIBITED",
                        entity_ids=(u.id,),
                        details=f"UAV {u.id} attempted secondary sortie after landing at {rec.landing_time:.1f}s (single-sortie policy)",
                        severity="CRITICAL",
                    )
                    violations.append(v)
                    self.report.record_violation(v)

            # Detect takeoff (requires actual physical movement or departure from pad)
            # A staged UAV that is assigned a task but stationary on pad is NOT airborne
            if u.active and not rec.is_airborne and rec.landing_time is None:
                if dist_gcs > pad_radius or speed > EPSILON:
                    rec.takeoff_time = sim_time
                    rec.is_airborne = True
                elif self.airspace is None and dist_gcs > 1.0:
                    rec.takeoff_time = sim_time
                    rec.is_airborne = True

            # Update airborne duration
            if rec.is_airborne:
                takeoff = rec.takeoff_time if rec.takeoff_time is not None else sim_time
                rec.current_sortie_duration_s = max(0.0, sim_time - takeoff)
                if rec.current_sortie_duration_s > self.report.max_observed_sortie_duration_s:
                    self.report.max_observed_sortie_duration_s = rec.current_sortie_duration_s

                # Detect landing
                if not u.active or u.rth_state == RTHState.COMPLETE:
                    rec.landing_time = sim_time
                    rec.is_airborne = False
                    rec.landing_position = u.position_xy
                    rec.cumulative_airborne_s += rec.current_sortie_duration_s

                    # Validate landing position if airspace configured
                    if self.airspace is not None:
                        if not self.airspace.is_in_staging_pad(u.position_xy):
                            v = SafetyViolation(
                                tick=tick,
                                simulation_time=sim_time,
                                violation_type="LANDING_LOCATION",
                                entity_ids=(u.id,),
                                details=f"UAV {u.id} landed at ({u.position_xy[0]:.2f}, {u.position_xy[1]:.2f}) outside authorized staging pad",
                                severity="CRITICAL",
                            )
                            violations.append(v)
                            self.report.record_violation(v)

                # Over-duration check (if enforce_sortie_limit is active)
                if self.enforce_sortie_limit and rec.is_airborne:
                    if rec.current_sortie_duration_s > self.max_sortie_duration_s + EPSILON:
                        v = SafetyViolation(
                            tick=tick,
                            simulation_time=sim_time,
                            violation_type="FLIGHT_DURATION",
                            entity_ids=(u.id,),
                            details=f"UAV {u.id} airborne duration {rec.current_sortie_duration_s:.1f}s exceeded max sortie duration {self.max_sortie_duration_s:.1f}s",
                            severity="CRITICAL",
                        )
                        violations.append(v)
                        self.report.record_violation(v)

        return violations

    def evaluate_rth_triggers(
        self,
        snapshot: StateSnapshot,
        speed_limit: float = 5.0,
        idle_rate: float = 1.0,
        movement_rate: float = 0.5,
        mission_duration: float = 2700.0,
        enable_battery_rth: bool = True,
        enable_time_rth: bool = False,
        enable_sortie_rth: bool = False,
    ) -> list[StartRTHCommand]:
        """Determine if any active UAV must trigger RTH to prevent battery exhaustion, mission overtime, or sortie expiration."""
        commands: list[StartRTHCommand] = []
        tick = snapshot.simulation_tick
        sim_time = snapshot.simulation_time
        sorted_uav_ids = sorted(snapshot.uavs.keys())

        for u in sorted(snapshot.uavs.values(), key=lambda x: x.id):
            if not u.active or u.rth_state != RTHState.NONE:
                continue

            dx = u.position_xy[0] - self.gcs_position[0]
            dy = u.position_xy[1] - self.gcs_position[1]
            dist_to_gcs = math.hypot(dx, dy)

            pad_radius = self.airspace.staging_pad_radius_m if self.airspace else 1.0
            if dist_to_gcs <= pad_radius:
                # Already at GCS / Staging Pad
                continue

            return_time_s = dist_to_gcs / max(1.0, speed_limit)
            return_energy = (idle_rate * return_time_s + movement_rate * dist_to_gcs) * self.rth_energy_buffer

            # 1. Battery Trigger
            uav_idx = sorted_uav_ids.index(u.id)
            stagger_time = uav_idx * 8.0
            battery_trigger = enable_battery_rth and (u.battery_energy <= return_energy + (idle_rate * stagger_time * self.rth_energy_buffer) + EPSILON)

            # 2. Mission Overtime Trigger (stagger return by UAV index)
            time_left = max(0.0, mission_duration - sim_time)
            time_trigger = enable_time_rth and (time_left <= (return_time_s * self.rth_energy_buffer + stagger_time) + EPSILON)

            # 3. Sortie Duration Trigger (V1 Assumption: 1200s limit - required transit time - safety margin)
            sortie_trigger = False
            if enable_sortie_rth:
                rec = self.report.uav_flight_records.get(u.id)
                current_sortie = rec.current_sortie_duration_s if rec and rec.is_airborne else 0.0
                remaining_sortie = max(0.0, self.max_sortie_duration_s - current_sortie)
                # Transit time to GCS plus safety margin AND stagger time for landing queue
                required_rth_time = return_time_s + self.rth_safety_margin_s + stagger_time
                if remaining_sortie <= required_rth_time + EPSILON:
                    sortie_trigger = True

            if battery_trigger or time_trigger or sortie_trigger:
                commands.append(StartRTHCommand(source_tick=tick, uav_id=u.id))

        return commands
