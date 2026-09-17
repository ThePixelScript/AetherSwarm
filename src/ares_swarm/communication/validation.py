from dataclasses import dataclass
from typing import Any

def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)

def nonnegative(*args: float) -> None:
    for arg in args:
        if arg is not None and arg < 0:
            raise ValueError("Value must be non-negative")

def freeze(obj: Any) -> Any:
    # simple freeze fallback
    import types
    if isinstance(obj, dict):
        return types.MappingProxyType(obj)
    if isinstance(obj, list):
        return tuple(obj)
    return obj

class Validated:
    def __post_init__(self) -> None:
        pass
