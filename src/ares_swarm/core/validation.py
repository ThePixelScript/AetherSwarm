"""Strict runtime typing and deeply immutable JSON metadata."""
import math
from dataclasses import fields
from types import MappingProxyType, UnionType
from typing import Any, Mapping, get_args, get_origin, get_type_hints
from collections.abc import Mapping as ABCMapping
from .exceptions import ModelError

def require(condition: bool, message: str) -> None:
    if not condition:
        raise ModelError(message)

def matches(value: Any, annotation: Any) -> bool:
    if annotation is Any:
        return True
    origin, args = get_origin(annotation), get_args(annotation)
    if origin is UnionType:
        return any(matches(value, a) for a in args)
    if origin is tuple:
        return isinstance(value, tuple) and all(matches(v, args[0]) for v in value)
    if origin in (Mapping, ABCMapping):
        return isinstance(value, ABCMapping) and all(
            matches(k, args[0]) and matches(v, args[1]) for k, v in value.items())
    if annotation is float:
        return type(value) in (int, float) and math.isfinite(value)
    if annotation is int:
        return type(value) is int
    return isinstance(value, annotation)

def freeze(value: Any) -> Any:
    if isinstance(value, ABCMapping):
        require(all(type(k) is str for k in value), "metadata keys must be strings")
        return MappingProxyType({k: freeze(v) for k, v in value.items()})
    if isinstance(value, (tuple, list)):
        return tuple(freeze(v) for v in value)
    require(value is None or type(value) in (str, bool, int, float), "metadata must be JSON data")
    if type(value) is float:
        require(math.isfinite(value), "non-finite metadata")
    return value

class Validated:
    """Base for frozen dataclasses with strict runtime field validation."""
    def __post_init__(self) -> None:
        hints = get_type_hints(type(self))
        for field in fields(self):
            require(matches(getattr(self, field.name), hints[field.name]),
                    f"{type(self).__name__}.{field.name}: invalid type or non-finite value")

def nonnegative(*values: float) -> None:
    require(all(v >= 0 for v in values), "values must be nonnegative")
