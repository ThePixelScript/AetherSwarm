"""Deterministic scenario configuration and state snapshot factory."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, Sequence
import yaml

from ..communication.config import CommunicationConfig
from ..core.enums import FailureState, Role, RTHState, TaskStatus
from ..core.models import StateSnapshot, TaskState, UAVState


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
    communication: CommunicationConfig = field(default_factory=CommunicationConfig)
    battery_idle_rate: float = 1.0
    battery_movement_rate: float = 0.5
    uavs: tuple[dict[str, Any], ...] = ()
    tasks: tuple[dict[str, Any], ...] = ()


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

    uavs_raw = tuple(raw.get("uavs", []))
    tasks_raw = tuple(raw.get("tasks", []))

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
        communication=communication,
        battery_idle_rate=battery_idle_rate,
        battery_movement_rate=battery_movement_rate,
        uavs=uavs_raw,
        tasks=tasks_raw,
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

        tasks[t_id] = TaskState(
            id=t_id,
            position_xy=pos_xy,
            priority=priority,
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
