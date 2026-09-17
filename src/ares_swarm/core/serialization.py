"""Versioned, allowlisted JSON round trips; no pickle or dynamic class loading."""
import json
from dataclasses import fields, is_dataclass
from enum import Enum
from typing import Any, get_type_hints, get_args, get_origin
from types import UnionType
from collections.abc import Mapping
from .exceptions import SerializationError
from .validation import matches

def to_plain(value: Any) -> Any:
    """Convert immutable models to ordinary JSON data without live references."""
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value):
        return {f.name: to_plain(getattr(value, f.name)) for f in fields(value)}
    if isinstance(value, Mapping):
        return {k: to_plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [to_plain(v) for v in value]
    if value is None or type(value) in (str, int, float, bool):
        return value
    raise SerializationError(f"unsupported value: {type(value).__name__}")

def from_plain(cls: Any, data: Any) -> Any:
    """Strictly reconstruct known typed data; never coerce strings to numbers."""
    origin, args = get_origin(cls), get_args(cls)
    if cls is Any:
        return data
    if origin is UnionType:
        for option in args:
            try:
                return from_plain(option, data)
            except (ValueError, TypeError):
                pass
        raise SerializationError("no matching union type")
    if origin is tuple:
        if not isinstance(data, list):
            raise SerializationError("expected JSON array")
        return tuple(from_plain(args[0], x) for x in data)
    if origin is Mapping:
        if not isinstance(data, dict):
            raise SerializationError("expected mapping")
        return {from_plain(args[0], k): from_plain(args[1], v) for k, v in data.items()}
    if isinstance(cls, type) and issubclass(cls, Enum):
        return cls(data)
    if is_dataclass(cls):
        if not isinstance(data, dict):
            raise SerializationError("expected object")
        hints = get_type_hints(cls)
        if set(data) - set(hints):
            raise SerializationError(f"unknown fields: {set(data)-set(hints)}")
        return cls(**{k: from_plain(hints[k], v) for k, v in data.items()})
    if not matches(data, cls):
        raise SerializationError(f"invalid value for {cls}")
    return data

def _registry() -> dict[str, type]:
    from . import models, snapshot, transitions, events, config
    from ..interfaces import autonomy, communication, safety
    result = {}
    for module in (models, snapshot, transitions, events, config, autonomy, communication, safety):
        for value in vars(module).values():
            if isinstance(value, type) and is_dataclass(value) and value.__module__ == module.__name__:
                result[value.__name__] = value
    return result

def dumps(value: Any) -> str:
    """Serialize an allowlisted top-level model with schema version and type tag."""
    if type(value).__name__ not in _registry() or _registry()[type(value).__name__] is not type(value):
        raise SerializationError("unknown model type")
    return json.dumps({"schema_version": 1, "type": type(value).__name__, "data": to_plain(value)},
                      sort_keys=True, allow_nan=False, separators=(",", ":"))

def loads(text: str) -> Any:
    """Read a version-one model; reject unrecognized types and schema versions."""
    def unique(pairs):
        result = {}
        for k, v in pairs:
            if k in result:
                raise SerializationError("duplicate JSON key")
            result[k] = v
        return result
    try:
        raw = json.loads(text, object_pairs_hook=unique)
        if not isinstance(raw, dict) or set(raw) != {"schema_version", "type", "data"}:
            raise SerializationError("invalid envelope")
        if type(raw["schema_version"]) is not int or raw["schema_version"] != 1:
            raise SerializationError("unsupported schema")
        if type(raw["type"]) is not str:
            raise SerializationError("type tag must be a string")
        cls = _registry().get(raw["type"])
        if cls is None:
            raise SerializationError("unknown model type")
        return from_plain(cls, raw["data"])
    except (ValueError, TypeError, KeyError, RecursionError) as exc:
        raise SerializationError(str(exc)) from exc
