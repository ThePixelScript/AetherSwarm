"""Structured JSON-lines formatting; no global logging configuration."""
import json
import logging
from datetime import datetime, timezone
from .serialization import to_plain

class StructuredFormatter(logging.Formatter):
    """Format standard LogRecords with simulation context carried in extra."""
    def format(self, record: logging.LogRecord) -> str:
        return json.dumps({
            "timestamp": datetime.fromtimestamp(record.created, timezone.utc).isoformat(),
            "simulation_time": getattr(record, "simulation_time", None),
            "category": getattr(record, "category", record.name),
            "event": getattr(record, "event", None),
            "entity": getattr(record, "entity", None),
            "message": record.getMessage(),
            "seed": getattr(record, "seed", None),
            "scenario": getattr(record, "scenario", None),
            "metadata": to_plain(getattr(record, "metadata", {})),
        }, allow_nan=False, sort_keys=True)
