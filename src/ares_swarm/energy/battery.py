from __future__ import annotations


def calculate_energy_cost(
    dt: float,
    distance: float,
    idle_rate: float,
    movement_rate: float,
) -> float:
    """Calculate energy consumed during one simulation step."""

    if dt <= 0:
        raise ValueError("dt must be greater than 0")

    if distance < 0:
        raise ValueError("distance must be non-negative")

    if idle_rate < 0 or movement_rate < 0:
        raise ValueError("energy rates must be non-negative")

    idle_energy = idle_rate * dt
    movement_energy = movement_rate * distance

    return idle_energy + movement_energy