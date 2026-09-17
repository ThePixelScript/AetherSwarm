"""Strict YAML configuration and validation CLI; no simulation execution."""
import argparse
import json
from dataclasses import dataclass
from pathlib import Path
import yaml
from .models import GeoFence, UAVState, TaskState
from .events import SimulationEvent
from .validation import Validated, require, nonnegative
from .exceptions import ConfigError, ModelError

@dataclass(frozen=True, slots=True)
class SimulationConfig(Validated):
    timestep: float
    duration: float
    seed: int

    def __post_init__(self) -> None:
        super(SimulationConfig, self).__post_init__()
        require(0 < self.timestep <= self.duration, "invalid timestep/duration")
        nonnegative(self.seed)

@dataclass(frozen=True, slots=True)
class GCSConfig(Validated):
    x: float
    y: float

@dataclass(frozen=True, slots=True)
class UAVConfig(Validated):
    count: int
    speed: float
    battery_capacity: float
    communication_range: float

    def __post_init__(self) -> None:
        super(UAVConfig, self).__post_init__()
        require(self.count > 0 and min(self.speed, self.battery_capacity, self.communication_range) > 0,
                "invalid UAV configuration")

@dataclass(frozen=True, slots=True)
class SafetyConfig(Validated):
    minimum_separation: float
    battery_reserve: float
    geofence: GeoFence

    def __post_init__(self) -> None:
        super(SafetyConfig, self).__post_init__()
        require(self.minimum_separation > 0, "separation must be positive")
        nonnegative(self.battery_reserve)

@dataclass(frozen=True, slots=True)
class CommunicationConfig(Validated):
    max_range: float
    base_latency: float
    packet_loss: float

    def __post_init__(self) -> None:
        super(CommunicationConfig, self).__post_init__()
        require(self.max_range > 0 and 0 <= self.packet_loss <= 1, "invalid communication configuration")
        nonnegative(self.base_latency)

@dataclass(frozen=True, slots=True)
class AutonomyConfig(Validated):
    role_cooldown: float
    assignment_cooldown: float
    minimum_improvement_threshold: float

    def __post_init__(self) -> None:
        super(AutonomyConfig, self).__post_init__()
        nonnegative(self.role_cooldown, self.assignment_cooldown, self.minimum_improvement_threshold)

@dataclass(frozen=True, slots=True)
class AppConfig(Validated):
    simulation: SimulationConfig
    gcs: GCSConfig
    uavs: UAVConfig
    safety: SafetyConfig
    communication: CommunicationConfig
    autonomy: AutonomyConfig

    def __post_init__(self) -> None:
        super(AppConfig, self).__post_init__()
        require(self.uavs.communication_range == self.communication.max_range,
                "communication range aliases must agree")
        require(self.safety.battery_reserve < self.uavs.battery_capacity,
                "reserve must be below battery capacity")

@dataclass(frozen=True, slots=True)
class ScenarioConfig(Validated):
    name: str
    uavs: tuple[UAVState, ...]
    tasks: tuple[TaskState, ...] = ()
    events: tuple[SimulationEvent, ...] = ()

    def __post_init__(self) -> None:
        super(ScenarioConfig, self).__post_init__()
        require(bool(self.name.strip()), "scenario name required")
        for collection in (self.uavs, self.tasks, self.events):
            require(len({x.id for x in collection}) == len(collection), "duplicate scenario IDs")
        require(not any(e.processed for e in self.events), "initial events cannot be processed")

class StrictLoader(yaml.SafeLoader):
    """Safe YAML loader rejecting duplicate keys."""

def _mapping(loader: StrictLoader, node: yaml.MappingNode, deep: bool = False) -> dict:
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str) or key in result:
            raise ConfigError("YAML keys must be unique strings")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result

StrictLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)

def _load(path: str | Path, cls: type):
    from .serialization import from_plain
    try:
        with Path(path).open(encoding="utf-8") as stream:
            raw = yaml.load(stream, Loader=StrictLoader)
        return from_plain(cls, raw)
    except (OSError, yaml.YAMLError, ModelError, ValueError, TypeError, RecursionError) as exc:
        raise ConfigError(f"{path}: {exc}") from exc

def load_config(path: str | Path) -> AppConfig:
    """Load strict defaults; reject unknown keys and malformed numeric values."""
    return _load(path, AppConfig)

def load_scenario(path: str | Path, config: AppConfig) -> ScenarioConfig:
    """Validate initial states and event references without executing events."""
    from .models import SwarmState, GCSState, Vector2D
    from .enums import EventType
    scenario = _load(path, ScenarioConfig)
    try:
        require(len(scenario.uavs) == config.uavs.count, "scenario UAV count mismatch")
        SwarmState(gcs=GCSState("gcs", Vector2D(config.gcs.x, config.gcs.y)),
                   uavs=scenario.uavs, tasks=scenario.tasks)
        ids = {u.id for u in scenario.uavs}
        for u in scenario.uavs:
            require(u.energy_remaining <= config.uavs.battery_capacity, "energy exceeds capacity")
            require(abs(u.battery_pct - 100*u.energy_remaining/config.uavs.battery_capacity) < 1e-8,
                    "initial battery percentage/energy mismatch")
        for e in scenario.events:
            require(e.timestamp <= config.simulation.duration, "event after mission duration")
            if e.event_type in (EventType.UAV_FAILURE, EventType.BATTERY_WARNING):
                require(e.target in ids, "unknown event UAV target")
    except ModelError as exc:
        raise ConfigError(str(exc)) from exc
    return scenario

def main() -> int:
    """Validate YAML only; emit a machine-readable result."""
    parser = argparse.ArgumentParser(description="Validate ARES-Swarm Phase 1 configuration")
    parser.add_argument("--config", required=True)
    parser.add_argument("--scenario")
    args = parser.parse_args()
    try:
        cfg = load_config(args.config)
        scenario = load_scenario(args.scenario, cfg) if args.scenario else None
    except ConfigError as exc:
        parser.exit(2, f"Configuration error: {exc}\n")
    print(json.dumps({"valid": True, "seed": cfg.simulation.seed,
                      "scenario": scenario.name if scenario else None, "phase": 1}))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
