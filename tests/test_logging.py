import json
import logging
from ares_swarm.core.logging import StructuredFormatter

def test_structured_record():
    record = logging.LogRecord("ares", logging.INFO, "", 0, "accepted %s", ("move",), None)
    record.simulation_time = 1.0
    record.seed = 42
    record.metadata = {"transition_id":"t"}
    result = json.loads(StructuredFormatter().format(record))
    assert result["message"] == "accepted move"
    assert result["seed"] == 42
    assert result["simulation_time"] == 1
    assert result["metadata"]["transition_id"] == "t"
