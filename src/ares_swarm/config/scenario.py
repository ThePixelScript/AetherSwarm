"""Scenario generation configuration model."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ScenarioGenConfig:
    """Configuration for scenario generation and POI sampling.

    Attributes:
        seed: Deterministic random seed.
        scenario_type: Scenario classification ('canonical', 'random_demo', 'recovery').
        num_pois: Total number of POIs to generate (default: 10).
        x_range: Independent uniform sampling bounds for X coordinate (m). Default: (5.0, 995.0).
        y_range: Independent uniform sampling bounds for Y coordinate (m). Default: (5.0, 995.0).
        min_spacing_m: Minimum pairwise POI spacing constraint in meters.
            Default is 0.0, which performs direct independent uniform sampling with no spatial rejection.
            When > 0.0, deterministic rejection sampling enforces this spacing constraint (producing
            constrained random placement, not independent uniform samples).
        margin_m: Boundary margin applied only when explicitly requested (default: 0.0).
        spawn_window_s: POI creation / spawn time interval [min_s, max_s] (s).
        service_duration_s: Service time required to service each POI (s).
        deadline_offset_s: Relative deadline duration from spawn time (s).
    """

    seed: int = 42
    scenario_type: str = "random"
    num_pois: int = 10
    x_range: tuple[float, float] = (5.0, 995.0)
    y_range: tuple[float, float] = (5.0, 995.0)
    min_spacing_m: float = 0.0
    margin_m: float = 0.0
    spawn_window_s: tuple[float, float] = (0.0, 300.0)
    service_duration_s: float = 2.0
    deadline_offset_s: float = 10.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "seed": self.seed,
            "scenario_type": self.scenario_type,
            "num_pois": self.num_pois,
            "x_range": list(self.x_range),
            "y_range": list(self.y_range),
            "min_spacing_m": self.min_spacing_m,
            "margin_m": self.margin_m,
            "spawn_window_s": list(self.spawn_window_s),
            "service_duration_s": self.service_duration_s,
            "deadline_offset_s": self.deadline_offset_s,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ScenarioGenConfig:
        return cls(
            seed=int(data.get("seed", 42)),
            scenario_type=str(data.get("scenario_type", "random")),
            num_pois=int(data.get("num_pois", 10)),
            x_range=tuple(float(v) for v in data.get("x_range", (5.0, 995.0))),
            y_range=tuple(float(v) for v in data.get("y_range", (5.0, 995.0))),
            min_spacing_m=float(data.get("min_spacing_m", 0.0)),
            margin_m=float(data.get("margin_m", 0.0)),
            spawn_window_s=tuple(float(v) for v in data.get("spawn_window_s", (0.0, 300.0))),
            service_duration_s=float(data.get("service_duration_s", 2.0)),
            deadline_offset_s=float(data.get("deadline_offset_s", 10.0)),
        )
