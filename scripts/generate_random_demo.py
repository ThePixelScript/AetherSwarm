#!/usr/bin/env python3
"""Backward-compatibility wrapper delegating to generate_scenario.py."""
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from generate_scenario import (  # noqa: F401
    sample_random_pois,
    generate_scenario_dict,
    generate_demo_scenario_dict,
    compute_poi_metrics,
    generate_and_export_scenario,
    generate_and_export_demo,
    main,
)

if __name__ == "__main__":
    sys.exit(main())
