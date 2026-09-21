"""Deterministic scenario configuration and state snapshot factory."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, Sequence, Tuple
import yaml

from ..communication.config import CommunicationConfig
from ..core.enums import FailureState, Role, RTHState, TaskStatus
from ..core.models import StateSnapshot, TaskState, UAVState


@dataclass(frozen=True)
class ChallengeAirspaceConfig:
    """Configuration for formal Challenge Airspace under Challenge Assumptions V1."""
    enabled: bool = False
    staging_pad_center: tuple[float, float] = (-75.0, 500.0)
    staging_pad_radius_m: float = 15.0
    corridor_bounds_x: tuple[float, float] = (-75.0, 0.0)
    corridor_bounds_y: tuple[float, float] = (450.0, 550.0)
    arena_bounds_x: tuple[float, float] = (0.0, 1000.0)
    arena_bounds_y: tuple[float, float] = (0.0, 1000.0)
    max_height: float = 100.0


@dataclass(frozen=True)
class ChallengeProfileConfig:
    """Configuration for Challenge Compliance Layer V1 (explicitly opt-in)."""
    enabled: bool = False
    max_sortie_duration_s: float = 1200.0
    rth_safety_margin_s: float = 15.0
    enforce_sortie_limit: bool = True
    enforce_single_sortie: bool = True
    airspace: ChallengeAirspaceConfig = field(default_factory=ChallengeAirspaceConfig)


@dataclass(frozen=True)
class ScenarioConfig:
    name: str = "default_mission"
    seed: int = 42
    dt: float = 1.0
    speed_limit: float = 5.0
    duration: float = 10.0
    max_ticks: int = 10
    gcs_position: tuple[float, float] = (0.0, 0.0)
    arena_bounds_x: tuple[float, float] = (-100.0, 100.0)
    arena_bounds_y: tuple[float, float] = (-100.0, 100.0)
    max_height: float = 100.0
    min_separation_m: float = 20.0
    communication: CommunicationConfig = field(default_factory=CommunicationConfig)
    battery_idle_rate: float = 1.0
    battery_movement_rate: float = 0.5
    enable_auto_rth: bool = True
    return_by_mission_end: bool = False
    uavs: tuple[dict[str, Any], ...] = ()
    tasks: tuple[dict[str, Any], ...] = ()
    challenge_profile: ChallengeProfileConfig = field(default_factory=ChallengeProfileConfig)


def load_scenario(source: str | Path | dict[str, Any]) -> ScenarioConfig:
    """Load scenario configuration from YAML file, path, or dictionary."""
    if isinstance(source, (str, Path)):
        path = Path(source)
        if not path.is_file():
            raise FileNotFoundError(f"Scenario file not found: {path}")
        with open(path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f) or {}
    elif isinstance(source, dict):
        raw = dict(source)
    else:
        raise TypeError(f"Unsupported scenario source type: {type(source)}")

    name = str(raw.get("name", "unnamed_scenario"))
    seed = int(raw.get("seed", 42))
    dt = float(raw.get("dt", 1.0))
    if dt <= 0:
        raise ValueError("dt must be greater than 0")

    speed_limit = float(raw.get("speed_limit", 5.0))
    if speed_limit <= 0:
        raise ValueError("speed_limit must be greater than 0")

    duration = float(raw.get("duration", 10.0))
    max_ticks = int(raw.get("max_ticks", int(duration / dt) if dt > 0 else 10))

    gcs_raw = raw.get("gcs_position", (0.0, 0.0))
    gcs_position = (float(gcs_raw[0]), float(gcs_raw[1]))

    arena_raw = raw.get("arena", {})
    arena_bounds_x = (
        float(arena_raw.get("bounds_x", (-100.0, 100.0))[0]),
        float(arena_raw.get("bounds_x", (-100.0, 100.0))[1]),
    )
    arena_bounds_y = (
        float(arena_raw.get("bounds_y", (-100.0, 100.0))[0]),
        float(arena_raw.get("bounds_y", (-100.0, 100.0))[1]),
    )
    max_height = float(arena_raw.get("max_height", raw.get("max_height", 100.0)))
    min_separation_m = float(raw.get("min_separation_m", 20.0))

    comm_raw = raw.get("communication", {})
    communication = CommunicationConfig(
        max_range=float(comm_raw.get("max_range", 1000.0)),
        base_latency=float(comm_raw.get("base_latency", 5.0)),
        packet_loss=float(comm_raw.get("packet_loss", 0.0)),
        degradation_multiplier=float(comm_raw.get("degradation_multiplier", 1.0)),
    )

    battery_raw = raw.get("battery", {})
    battery_idle_rate = float(battery_raw.get("idle_rate", 1.0))
    battery_movement_rate = float(battery_raw.get("movement_rate", 0.5))
    enable_auto_rth = bool(raw.get("enable_auto_rth", True))
    return_by_mission_end = bool(raw.get("return_by_mission_end", False))

    uavs_raw = tuple(raw.get("uavs", []))
    tasks_raw = tuple(raw.get("tasks", []))

    challenge_raw = raw.get("challenge_profile", {})
    challenge_enabled = bool(challenge_raw.get("enabled", False))
    airspace_raw = challenge_raw.get("airspace", {})
    airspace_config = ChallengeAirspaceConfig(
        enabled=bool(airspace_raw.get("enabled", challenge_enabled)),
        staging_pad_center=tuple(float(x) for x in airspace_raw.get("staging_pad_center", (-75.0, 500.0))),
        staging_pad_radius_m=float(airspace_raw.get("staging_pad_radius_m", 15.0)),
        corridor_bounds_x=tuple(float(x) for x in airspace_raw.get("corridor_bounds_x", (-75.0, 0.0))),
        corridor_bounds_y=tuple(float(y) for y in airspace_raw.get("corridor_bounds_y", (450.0, 550.0))),
        arena_bounds_x=tuple(float(x) for x in airspace_raw.get("arena_bounds_x", arena_bounds_x)),
        arena_bounds_y=tuple(float(y) for y in airspace_raw.get("arena_bounds_y", arena_bounds_y)),
        max_height=float(airspace_raw.get("max_height", max_height)),
    )
    challenge_profile = ChallengeProfileConfig(
        enabled=challenge_enabled,
        max_sortie_duration_s=float(challenge_raw.get("max_sortie_duration_s", 1200.0)),
        rth_safety_margin_s=float(challenge_raw.get("rth_safety_margin_s", 15.0)),
        enforce_sortie_limit=bool(challenge_raw.get("enforce_sortie_limit", True)),
        enforce_single_sortie=bool(challenge_raw.get("enforce_single_sortie", True)),
        airspace=airspace_config,
    )

    return ScenarioConfig(
        name=name,
        seed=seed,
        dt=dt,
        speed_limit=speed_limit,
        duration=duration,
        max_ticks=max_ticks,
        gcs_position=gcs_position,
        arena_bounds_x=arena_bounds_x,
        arena_bounds_y=arena_bounds_y,
        max_height=max_height,
        min_separation_m=min_separation_m,
        communication=communication,
        battery_idle_rate=battery_idle_rate,
        battery_movement_rate=battery_movement_rate,
        enable_auto_rth=enable_auto_rth,
        return_by_mission_end=return_by_mission_end,
        uavs=uavs_raw,
        tasks=tasks_raw,
        challenge_profile=challenge_profile,
    )


def create_initial_snapshot(scenario: ScenarioConfig) -> StateSnapshot:
    """Build deterministic StateSnapshot from ScenarioConfig."""
    uavs: dict[str, UAVState] = {}
    for item in scenario.uavs:
        u_id = str(item["id"])
        pos_raw = item.get("position", item.get("position_xy", (0.0, 0.0)))
        pos_xy = (float(pos_raw[0]), float(pos_raw[1]))
        capacity = float(item.get("battery_capacity", 100.0))
        energy = float(item.get("battery_energy", capacity))
        role_val = item.get("role", "IDLE")
        role = Role(role_val) if isinstance(role_val, str) else role_val

        uavs[u_id] = UAVState(
            id=u_id,
            position_xy=pos_xy,
            battery_capacity=capacity,
            battery_energy=energy,
            role=role,
            active=bool(item.get("active", True)),
            failure_state=FailureState(item.get("failure_state", "NORMAL")),
            rth_state=RTHState(item.get("rth_state", "NONE")),
        )

    tasks: dict[str, TaskState] = {}
    for item in scenario.tasks:
        t_id = str(item["id"])
        pos_raw = item.get("position", item.get("position_xy", (0.0, 0.0)))
        pos_xy = (float(pos_raw[0]), float(pos_raw[1]))
        priority = int(item.get("priority", 1))
        service_duration = float(item.get("service_duration", item.get("required_progress", 1.0)))

        created_time = float(item.get("created_time", item.get("spawn_time", 0.0)))
        deadline_raw = float(item.get("deadline", item.get("deadline_offset", 0.0)))
        if deadline_raw > 0.0 and deadline_raw < created_time:
            deadline = created_time + deadline_raw
        elif deadline_raw > 0.0:
            deadline = deadline_raw
        else:
            deadline = 0.0

        tasks[t_id] = TaskState(
            id=t_id,
            position_xy=pos_xy,
            priority=priority,
            created_time=created_time,
            deadline=deadline,
            service_duration=service_duration,
            status=TaskStatus(item.get("status", "PENDING")),
            emergency_flag=bool(item.get("emergency_flag", False)),
        )

    return StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType(uavs),
        tasks=MappingProxyType(tasks),
        gcs_position=scenario.gcs_position,
    )
