"""Deterministic safety assessment, constraint verification, and RTH prevention."""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, List, Optional, Sequence, Tuple

from ..core.commands import StartRTHCommand
from ..core.constants import EPSILON
from ..core.enums import RTHState
from ..core.models import StateSnapshot, UAVState
from ..interfaces.communication import NetworkAnalysis


@dataclass(frozen=True)
class SafetyViolation:
    tick: int
    simulation_time: float
    violation_type: str  # "GEOFENCE", "SEPARATION", "BATTERY_EXHAUSTION"
    entity_ids: tuple[str, ...]
    details: str
    severity: str = "CRITICAL"  # "WARNING", "CRITICAL"


@dataclass
class SafetyReport:
    violations: list[SafetyViolation] = field(default_factory=list)
    _recorded_keys: set[tuple[int, str, tuple[str, ...]]] = field(default_factory=set)
    min_observed_separation_m: float = float("inf")
    geofence_violations_count: int = 0
    separation_violations_count: int = 0
    battery_exhaustions_count: int = 0

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


class SafetyAssessor:
    """Deterministic assessor for geofence, separation, and battery safety constraints."""

    def __init__(
        self,
        arena_bounds_x: tuple[float, float] = (-500.0, 500.0),
        arena_bounds_y: tuple[float, float] = (-500.0, 500.0),
        min_separation_m: float = 20.0,
        gcs_position: tuple[float, float] = (0.0, 0.0),
        rth_energy_buffer: float = 1.2,
    ):
        self.arena_bounds_x = arena_bounds_x
        self.arena_bounds_y = arena_bounds_y
        self.min_separation_m = min_separation_m
        self.gcs_position = gcs_position
        self.rth_energy_buffer = rth_energy_buffer
        self.report = SafetyReport()

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

        # 1. Geofence verification (for UAVs not returning to GCS outside arena)
        min_x, max_x = self.arena_bounds_x
        min_y, max_y = self.arena_bounds_y

        for u in active_uavs:
            # If UAV is in RTH or landed at GCS which is outside arena, exempt from arena bounds
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

                # Landed UAVs parked at GCS are exempt from in-flight airborne separation checks
                d1_gcs = math.hypot(u1.position_xy[0] - self.gcs_position[0], u1.position_xy[1] - self.gcs_position[1])
                d2_gcs = math.hypot(u2.position_xy[0] - self.gcs_position[0], u2.position_xy[1] - self.gcs_position[1])
                if d1_gcs <= 1.0 or d2_gcs <= 1.0:
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
            # If battery is 0 and UAV is not at GCS, it's exhausted
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
    ) -> list[StartRTHCommand]:
        """Determine if any active UAV must trigger RTH to prevent battery exhaustion or mission overtime."""
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

            if dist_to_gcs <= 1.0:
                # Already at GCS
                continue

            return_time_s = dist_to_gcs / max(1.0, speed_limit)
            return_energy = (idle_rate * return_time_s + movement_rate * dist_to_gcs) * self.rth_energy_buffer

            # Trigger RTH if remaining battery cannot safely return to GCS
            battery_trigger = enable_battery_rth and (u.battery_energy <= return_energy + EPSILON)

            # Or trigger RTH if time left in mission is required to return to GCS
            # Stagger return by UAV index (8s = 40m in-trail separation at 5m/s) to prevent approach congestion
            uav_idx = sorted_uav_ids.index(u.id)
            stagger_time = uav_idx * 8.0
            time_left = max(0.0, mission_duration - sim_time)
            time_trigger = enable_time_rth and (time_left <= (return_time_s * self.rth_energy_buffer + stagger_time) + EPSILON)

            if battery_trigger or time_trigger:
                commands.append(StartRTHCommand(source_tick=tick, uav_id=u.id))

        return commands
