from __future__ import annotations

import math
from typing import Tuple


def move_towards(
    current: Tuple[float, float],
    target: Tuple[float, float],
    speed: float,
    dt: float,
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """Move deterministically toward a target for one simulation step."""

    if speed < 0:
        raise ValueError("speed must be non-negative")

    if dt <= 0:
        raise ValueError("dt must be greater than 0")

    dx = target[0] - current[0]
    dy = target[1] - current[1]
    distance = math.hypot(dx, dy)

    if distance == 0:
        return current, (0.0, 0.0)

    step = speed * dt

    if distance <= step:
        return target, (0.0, 0.0)

    vx = speed * dx / distance
    vy = speed * dy / distance

    new_position = (
        current[0] + vx * dt,
        current[1] + vy * dt,
    )

    return new_position, (vx, vy)