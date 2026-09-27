"""Hidden and emerging point of interest (POI) discovery simulation module.

TEAM SIMULATION ASSUMPTION:
The detection model implemented herein uses an altitude-dependent sensor detection
footprint for discovering initially hidden and dynamically emerging points of
interest in the simulation arena. This is an explicit team simulation assumption
designed for algorithmic benchmark evaluation and does NOT represent real-world
physical sensor specifications or manufacturer hardware limits.
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

from ..core.commands import DiscoverTaskCommand
from ..core.enums import FailureState, SortieState, TaskStatus
from ..core.models import StateSnapshot, TaskState, UAVState


# TEAM SIMULATION ASSUMPTION:
# Detection radius (m) as an altitude-dependent footprint table.
# Clamped below 20m to 80m and above 100m to 230m.
# Linearly interpolated between adjacent table coordinates.
ALTITUDE_DETECTION_TABLE: tuple[tuple[float, float], ...] = (
    (20.0, 80.0),
    (40.0, 130.0),
    (60.0, 170.0),
    (80.0, 190.0),
    (100.0, 230.0),
)


def compute_detection_radius(altitude_m: float) -> float:
    """Compute altitude-dependent detection radius using linear interpolation and clamping.

    TEAM SIMULATION ASSUMPTION:
    This model computes simulated sensor footprint radius as a function of UAV altitude.
    It is a simplified team simulation assumption for POI discovery benchmarking, not a
    real-world hardware sensor specification.

    Altitude Table:
        20.0 m  -> 80.0 m
        40.0 m  -> 130.0 m
        60.0 m  -> 170.0 m
        80.0 m  -> 190.0 m
        100.0 m -> 230.0 m

    Bounds:
        <= 20.0 m: clamped to 80.0 m
        >= 100.0 m: clamped to 230.0 m
        Intermediate altitudes: linearly interpolated.
    """
    if altitude_m <= 20.0:
        return 80.0
    if altitude_m >= 100.0:
        return 230.0

    for (a0, r0), (a1, r1) in zip(ALTITUDE_DETECTION_TABLE, ALTITUDE_DETECTION_TABLE[1:]):
        if a0 <= altitude_m <= a1:
            ratio = (altitude_m - a0) / (a1 - a0)
            return r0 + ratio * (r1 - r0)

    return 80.0


def get_uav_altitude(uav: Any) -> float:
    """Determine current UAV altitude in meters from available state attributes."""
    if hasattr(uav, "altitude_m") and uav.altitude_m is not None:
        return float(uav.altitude_m)
    if hasattr(uav, "altitude") and uav.altitude is not None:
        return float(uav.altitude)
    if hasattr(uav, "position_z") and uav.position_z is not None:
        return float(uav.position_z)
    if hasattr(uav, "altitude_layer") and uav.altitude_layer is not None:
        return float(uav.altitude_layer * 20.0)
    return 20.0


@dataclass
class HiddenPOI:
    """Ground-truth definition of a hidden or emerging point of interest.

    Ground-truth coordinates and emergence properties are maintained exclusively
    within the simulation/discovery engine and are NOT exposed to task allocators
    or planners until discovered by an eligible UAV's detection footprint.
    """
    id: str
    position_xy: tuple[float, float]
    priority: int = 1
    emergence_time: float = 0.0
    service_duration: float = 2.0
    deadline: float = 0.0
    deadline_offset: float = 0.0
    emergency_flag: bool = False
    metadata: dict[str, Any] = field(default_factory=dict)

    # Discovery tracking state
    discovered: bool = False
    discovered_tick: Optional[int] = None
    discovered_time: Optional[float] = None
    discovered_by: Optional[str] = None

    def reset(self) -> None:
        """Reset discovery state for reproducible simulation replays."""
        self.discovered = False
        self.discovered_tick = None
        self.discovered_time = None
        self.discovered_by = None


class DiscoveryManager:
    """Manages ground-truth hidden POIs, altitude footprints, and discovery events."""

    def __init__(
        self,
        hidden_pois: Sequence[HiddenPOI | dict[str, Any]] | None = None,
    ) -> None:
        self.hidden_pois: list[HiddenPOI] = []
        if hidden_pois:
            for item in hidden_pois:
                self.add_hidden_poi(item)

        # Discovery metrics
        self.total_discoveries: int = 0
        self.discovery_records: list[dict[str, Any]] = []

    @property
    def total_hidden_pois(self) -> int:
        """Total number of registered hidden/emerging POIs."""
        return len(self.hidden_pois)

    def is_discovered(self, poi_id: str) -> bool:
        """Check if a hidden POI has been discovered."""
        for p in self.hidden_pois:
            if p.id == poi_id:
                return p.discovered
        return False

    def add_hidden_poi(self, item: HiddenPOI | dict[str, Any]) -> None:
        """Add a hidden/emerging POI to the ground-truth simulation registry."""
        if isinstance(item, HiddenPOI):
            self.hidden_pois.append(item)
            return

        raw_pos = item.get("position", item.get("position_xy", (0.0, 0.0)))
        pos_xy = (float(raw_pos[0]), float(raw_pos[1]))
        emergence_time = float(item.get("emergence_time", item.get("spawn_time", 0.0)))
        deadline_raw = float(item.get("deadline", item.get("deadline_offset", 0.0)))

        h_poi = HiddenPOI(
            id=str(item["id"]),
            position_xy=pos_xy,
            priority=int(item.get("priority", 1)),
            emergence_time=emergence_time,
            service_duration=float(item.get("service_duration", item.get("required_progress", 2.0))),
            deadline=deadline_raw if deadline_raw >= emergence_time else (emergence_time + deadline_raw if deadline_raw > 0 else 0.0),
            deadline_offset=deadline_raw,
            emergency_flag=bool(item.get("emergency_flag", False)),
            metadata=dict(item.get("metadata", {})),
        )
        self.hidden_pois.append(h_poi)

    def reset(self) -> None:
        """Reset internal state and all POI discovery markers."""
        for poi in self.hidden_pois:
            poi.reset()
        self.total_discoveries = 0
        self.discovery_records.clear()

    def step(self, snapshot: StateSnapshot) -> list[DiscoverTaskCommand]:
        """Evaluate UAV detection footprints against emerged, undiscovered POIs."""
        commands: list[DiscoverTaskCommand] = []
        sim_time = snapshot.simulation_time
        sim_tick = snapshot.simulation_tick

        # 1. Identify emerged, undiscovered POIs
        active_hidden = [
            poi for poi in self.hidden_pois
            if not poi.discovered and sim_time >= poi.emergence_time
        ]
        if not active_hidden:
            return []

        # 2. Identify eligible airborne UAVs
        eligible_uavs = [
            u for u in sorted(snapshot.uavs.values(), key=lambda x: x.id)
            if u.active
            and u.failure_state != FailureState.FAILED
            and u.sortie_state not in (SortieState.LANDED, SortieState.RECHARGING)
        ]
        if not eligible_uavs:
            return []

        # 3. Check detection footprints
        for poi in active_hidden:
            if poi.discovered:
                continue

            for uav in eligible_uavs:
                altitude = get_uav_altitude(uav)
                radius = compute_detection_radius(altitude)
                dist = math.hypot(uav.position_xy[0] - poi.position_xy[0], uav.position_xy[1] - poi.position_xy[1])

                if dist <= radius:
                    # Mark discovered immediately to prevent double-discovery
                    poi.discovered = True
                    poi.discovered_tick = sim_tick
                    poi.discovered_time = sim_time
                    poi.discovered_by = uav.id

                    self.total_discoveries += 1
                    rec = {
                        "poi_id": poi.id,
                        "discovered_by": uav.id,
                        "tick": sim_tick,
                        "time": sim_time,
                        "uav_position": uav.position_xy,
                        "uav_altitude": altitude,
                        "detection_radius": radius,
                        "distance": round(dist, 2),
                        "poi_position": poi.position_xy,
                        "priority": poi.priority,
                        "emergence_time": poi.emergence_time,
                    }
                    self.discovery_records.append(rec)

                    # Determine deadline for discovered task
                    if poi.deadline > 0.0:
                        task_deadline = poi.deadline
                    elif poi.deadline_offset > 0.0:
                        task_deadline = sim_time + poi.deadline_offset
                    else:
                        task_deadline = 0.0

                    # Create standard TaskState representation
                    revealed_task = TaskState(
                        id=poi.id,
                        position_xy=poi.position_xy,
                        priority=poi.priority,
                        created_time=sim_time,
                        deadline=task_deadline,
                        service_duration=poi.service_duration,
                        status=TaskStatus.PENDING,
                        emergency_flag=poi.emergency_flag,
                    )

                    commands.append(
                        DiscoverTaskCommand(
                            source_tick=sim_tick,
                            uav_id=uav.id,
                            task=revealed_task,
                            emergence_time=poi.emergence_time,
                            metadata=rec,
                        )
                    )
                    break  # Stop checking other UAVs for this POI on this tick

        return commands

    def get_summary(self) -> dict[str, Any]:
        """Return discovery telemetry summary dictionary."""
        total = len(self.hidden_pois)
        discovered = sum(1 for p in self.hidden_pois if p.discovered)
        return {
            "total_hidden_pois": total,
            "discovered_pois": discovered,
            "discovered_count": discovered,
            "undiscovered_pois": total - discovered,
            "pending_hidden_count": total - discovered,
            "discoveries": list(self.discovery_records),
        }
