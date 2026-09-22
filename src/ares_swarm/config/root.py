"""Root configuration container for AetherSwarm."""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Mapping
import yaml

from .challenge import ChallengeSimulationConfig
from .features import FeatureConfig
from .presentation import WebotsPresentationConfig
from .scenario import ScenarioGenConfig


@dataclass(frozen=True)
class AetherSwarmConfig:
    """Authoritative root configuration model for AetherSwarm.

    Decomposes configuration cleanly into four distinct layers:
    1. challenge: Physical arena, corridor, flight boundaries, and challenge rules.
    2. features: Active feature flags and safety constraint enforcements.
    3. scenario: Scenario generation parameters (sampling bounds, spacing, POIs).
    4. presentation: Webots 3D presentation, visual interpolation, camera, and HUD layers.
    """

    challenge: ChallengeSimulationConfig = field(default_factory=ChallengeSimulationConfig)
    features: FeatureConfig = field(default_factory=FeatureConfig)
    scenario: ScenarioGenConfig = field(default_factory=ScenarioGenConfig)
    presentation: WebotsPresentationConfig = field(default_factory=WebotsPresentationConfig)

    def to_dict(self) -> dict[str, Any]:
        """Serialize configuration to a structured dictionary."""
        return {
            "challenge": self.challenge.to_dict(),
            "features": self.features.to_dict(),
            "scenario": self.scenario.to_dict(),
            "presentation": self.presentation.to_dict(),
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> AetherSwarmConfig:
        """Create AetherSwarmConfig from structured dictionary with hierarchical overrides."""
        c_raw = data.get("challenge", {})
        f_raw = data.get("features", {})
        s_raw = data.get("scenario", {})
        p_raw = data.get("presentation", {})

        return cls(
            challenge=ChallengeSimulationConfig.from_dict(c_raw),
            features=FeatureConfig.from_dict(f_raw),
            scenario=ScenarioGenConfig.from_dict(s_raw),
            presentation=WebotsPresentationConfig.from_dict(p_raw),
        )

    @classmethod
    def load(cls, source: str | Path | Mapping[str, Any]) -> AetherSwarmConfig:
        """Load configuration from a YAML file, file path, or dictionary."""
        if isinstance(source, (str, Path)):
            p = Path(source)
            if not p.is_file():
                raise FileNotFoundError(f"Configuration file not found: {p}")
            with open(p, "r", encoding="utf-8") as f:
                raw = yaml.safe_load(f) or {}
        elif isinstance(source, Mapping):
            raw = dict(source)
        else:
            raise TypeError(f"Unsupported configuration source type: {type(source)}")

        # If file has a top-level 'config' key, extract it
        if "config" in raw and isinstance(raw["config"], dict):
            raw = raw["config"]

        return cls.from_dict(raw)

    def with_overrides(
        self,
        challenge_overrides: Mapping[str, Any] | None = None,
        feature_overrides: Mapping[str, Any] | None = None,
        scenario_overrides: Mapping[str, Any] | None = None,
        presentation_overrides: Mapping[str, Any] | None = None,
    ) -> AetherSwarmConfig:
        """Return a new AetherSwarmConfig with explicit layer overrides applied."""
        new_c = self.challenge
        if challenge_overrides:
            c_dict = self.challenge.to_dict()
            c_dict.update(challenge_overrides)
            new_c = ChallengeSimulationConfig.from_dict(c_dict)

        new_f = self.features
        if feature_overrides:
            f_dict = self.features.to_dict()
            f_dict.update(feature_overrides)
            new_f = FeatureConfig.from_dict(f_dict)

        new_s = self.scenario
        if scenario_overrides:
            s_dict = self.scenario.to_dict()
            s_dict.update(scenario_overrides)
            new_s = ScenarioGenConfig.from_dict(s_dict)

        new_p = self.presentation
        if presentation_overrides:
            p_dict = self.presentation.to_dict()
            p_dict.update(presentation_overrides)
            new_p = WebotsPresentationConfig.from_dict(p_dict)

        return AetherSwarmConfig(
            challenge=new_c,
            features=new_f,
            scenario=new_s,
            presentation=new_p,
        )
