"""Challenge and simulation configuration model."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ChallengeSimulationConfig:
    """Challenge and physical simulation parameters under Round-1 specifications.

    Parameters:
        arena_bounds_x: Authorized arena X bounds [min_x, max_x] (m).
        arena_bounds_y: Authorized arena Y bounds [min_y, max_y] (m).
        max_altitude_m: Maximum operational altitude ceiling (m).
        speed_limit_mps: Maximum physical horizontal speed limit (m/s).
        min_separation_m: Minimum pairwise horizontal separation distance (m).
        comm_range_m: Maximum RF communication link range (m).
        comm_base_latency_ms: Base nominal communication latency (ms).
        staging_pad_center: Staging pad center coordinate (x, y) (m).
        staging_pad_radius_m: Staging pad circular radius (m).
        corridor_bounds_x: Transit corridor X bounds [min_x, max_x] (m).
        corridor_bounds_y: Transit corridor Y bounds [min_y, max_y] (m).
        mission_duration_s: Total mission scenario duration (s).
        max_sortie_duration_s: Maximum allowable airborne flight duration per sortie (s).
        rth_safety_margin_s: Safety buffer reserve before sortie duration expiry (s).
        reporting_deadline_s: Maximum allowable latency between POI detection and GCS report (s).
        detection_fov_radius_m: Sensor perception circular ground FOV radius (m).
        processing_delay_s: Sensor perception processing delay before buffering (s).
    """

    arena_bounds_x: tuple[float, float] = (0.0, 1000.0)
    arena_bounds_y: tuple[float, float] = (0.0, 1000.0)
    max_altitude_m: float = 100.0
    speed_limit_mps: float = 5.0
    min_separation_m: float = 20.0
    comm_range_m: float = 100.0
    comm_base_latency_ms: float = 5.0
    staging_pad_center: tuple[float, float] = (-75.0, 500.0)
    staging_pad_radius_m: float = 15.0
    corridor_bounds_x: tuple[float, float] = (-75.0, 0.0)
    corridor_bounds_y: tuple[float, float] = (450.0, 550.0)
    mission_duration_s: float = 2700.0
    max_sortie_duration_s: float = 1200.0
    rth_safety_margin_s: float = 15.0
    reporting_deadline_s: float = 10.0
    detection_fov_radius_m: float = 40.0
    processing_delay_s: float = 0.0
    recharge_duration_s: float = 300.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "arena_bounds_x": list(self.arena_bounds_x),
            "arena_bounds_y": list(self.arena_bounds_y),
            "max_altitude_m": self.max_altitude_m,
            "speed_limit_mps": self.speed_limit_mps,
            "min_separation_m": self.min_separation_m,
            "comm_range_m": self.comm_range_m,
            "comm_base_latency_ms": self.comm_base_latency_ms,
            "staging_pad_center": list(self.staging_pad_center),
            "staging_pad_radius_m": self.staging_pad_radius_m,
            "corridor_bounds_x": list(self.corridor_bounds_x),
            "corridor_bounds_y": list(self.corridor_bounds_y),
            "mission_duration_s": self.mission_duration_s,
            "max_sortie_duration_s": self.max_sortie_duration_s,
            "rth_safety_margin_s": self.rth_safety_margin_s,
            "reporting_deadline_s": self.reporting_deadline_s,
            "detection_fov_radius_m": self.detection_fov_radius_m,
            "processing_delay_s": self.processing_delay_s,
            "recharge_duration_s": self.recharge_duration_s,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ChallengeSimulationConfig:
        return cls(
            arena_bounds_x=tuple(float(v) for v in data.get("arena_bounds_x", (0.0, 1000.0))),
            arena_bounds_y=tuple(float(v) for v in data.get("arena_bounds_y", (0.0, 1000.0))),
            max_altitude_m=float(data.get("max_altitude_m", 100.0)),
            speed_limit_mps=float(data.get("speed_limit_mps", 5.0)),
            min_separation_m=float(data.get("min_separation_m", 20.0)),
            comm_range_m=float(data.get("comm_range_m", 100.0)),
            comm_base_latency_ms=float(data.get("comm_base_latency_ms", 5.0)),
            staging_pad_center=tuple(float(v) for v in data.get("staging_pad_center", (-75.0, 500.0))),
            staging_pad_radius_m=float(data.get("staging_pad_radius_m", 15.0)),
            corridor_bounds_x=tuple(float(v) for v in data.get("corridor_bounds_x", (-75.0, 0.0))),
            corridor_bounds_y=tuple(float(v) for v in data.get("corridor_bounds_y", (450.0, 550.0))),
            mission_duration_s=float(data.get("mission_duration_s", 2700.0)),
            max_sortie_duration_s=float(data.get("max_sortie_duration_s", 1200.0)),
            rth_safety_margin_s=float(data.get("rth_safety_margin_s", 15.0)),
            reporting_deadline_s=float(data.get("reporting_deadline_s", 10.0)),
            detection_fov_radius_m=float(data.get("detection_fov_radius_m", 40.0)),
            processing_delay_s=float(data.get("processing_delay_s", 0.0)),
            recharge_duration_s=float(data.get("recharge_duration_s", 300.0)),
        )
