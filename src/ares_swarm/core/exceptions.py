"""Public infrastructure exceptions."""
class AresError(Exception):
    """Base application error."""

class ModelError(AresError, ValueError):
    """Invalid domain value or cross-reference."""

class TransitionError(AresError, ValueError):
    """Rejected transition; state remains unchanged."""

class AuthorizationError(AresError, PermissionError):
    """Missing or incorrect writer capability."""

class ConfigError(AresError, ValueError):
    """Invalid configuration."""

class SerializationError(AresError, ValueError):
    """Invalid serialized data."""
