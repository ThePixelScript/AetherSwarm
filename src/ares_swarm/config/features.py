"""Feature configuration model."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class FeatureConfig:
    """Feature enablement and compliance enforcement flags.

    Parameters:
        enforce_separation: Whether continuous inter-UAV horizontal separation is actively enforced.
        enforce_geofence: Whether operational airspace geofence boundaries are actively enforced.
        enforce_sortie_limit: Whether maximum airborne sortie duration limits are enforced.
        enforce_single_sortie: Whether single sortie restriction is enforced (relaunch prohibited).
        enable_detection_pipeline: Whether sensor FOV perception and GCS telemetry buffering are enabled.
        enable_auto_rth: Whether automatic RTH is triggered on low energy or sortie deadline.
        return_by_mission_end: Whether all UAVs must complete RTH landing before mission completion.
    """

    enforce_separation: bool = False
    enforce_geofence: bool = False
    enforce_sortie_limit: bool = True
    enforce_single_sortie: bool = True
    enable_detection_pipeline: bool = False
    enable_auto_rth: bool = True
    return_by_mission_end: bool = True

    def to_dict(self) -> dict[str, Any]:
        return {
            "enforce_separation": self.enforce_separation,
            "enforce_geofence": self.enforce_geofence,
            "enforce_sortie_limit": self.enforce_sortie_limit,
            "enforce_single_sortie": self.enforce_single_sortie,
            "enable_detection_pipeline": self.enable_detection_pipeline,
            "enable_auto_rth": self.enable_auto_rth,
            "return_by_mission_end": self.return_by_mission_end,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> FeatureConfig:
        return cls(
            enforce_separation=bool(data.get("enforce_separation", False)),
            enforce_geofence=bool(data.get("enforce_geofence", False)),
            enforce_sortie_limit=bool(data.get("enforce_sortie_limit", True)),
            enforce_single_sortie=bool(data.get("enforce_single_sortie", True)),
            enable_detection_pipeline=bool(data.get("enable_detection_pipeline", False)),
            enable_auto_rth=bool(data.get("enable_auto_rth", True)),
            return_by_mission_end=bool(data.get("return_by_mission_end", True)),
        )
