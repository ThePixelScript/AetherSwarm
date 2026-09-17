# AetherSwarm (ARES-Swarm)

Adaptive Resilient Emergency Swarm for the PUSHPAK UAV-X challenge.

**Status: Phase 1 foundation plus M0 communication baseline.** The Stage-1 target claim is “Autonomous swarm
decision-making validated in a deterministic simulation environment.”
This release provides infrastructure; it does not yet run autonomous missions.

Implemented: typed immutable state, explicit validated transitions, capability-protected
StateStore, deterministic RNG streams, strict YAML configuration, versioned JSON,
structured logging formatter, module contracts and tests. M0 adds a deterministic
channel abstraction, NetworkX graph, connectivity analysis and shortest-hop routing,
exposed through the existing immutable NetworkAnalysis contract.

Not implemented: simulation stepping/event execution, physical-radio propagation,
allocation, weighted routing, adaptive relays, fault recovery, handover behavior, collision
avoidance, energy consumption, GNN, visualization, ROS 2/PX4 or hardware validation.

## Setup (Python 3.12.x)

From the uav-x directory, with Python 3.12 on PATH:

~~~powershell
python --version
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe -m pytest -q
.venv\Scripts\python.exe scripts\validate_config.py --config configs\default.yaml --scenario scenarios\basic.yaml
~~~

On Linux/macOS use .venv/bin/python. No activation is required.
requirements.txt uses constraints.txt to reproduce the tested dependency versions.
Python metadata deliberately requires 3.12.x. NumPy, PyYAML and NetworkX are runtime
dependencies; pytest is a test extra. SciPy, Pandas and Matplotlib belong
to later phases when their capabilities are needed.

Create a fresh .venv at the repository root after relocation; do not copy or reuse
a moved virtual environment. Editable installation must point to this checkout.

The installed command is also available:

~~~text
ares-validate-config --config configs/default.yaml --scenario scenarios/basic.yaml
~~~

Expected CLI output for the supplied example:
~~~json
{"valid": true, "seed": 42, "scenario": "basic", "phase": 1}
~~~

## Repository

- src/ares_swarm/core: enums, models, snapshots, transitions, StateStore, events,
  configuration, RNG, logging, validation and serialization.
- src/ares_swarm/interfaces: typed contracts for future team modules.
- src/ares_swarm/communication: M0 analysis-only channel, graph, connectivity, routing and adapter.
- docs/algorithms: communication formulas, API semantics, complexity and tests.
- configs/default.yaml: shared parameter configuration.
- scenarios/basic.yaml: explicit UAV/task/event initial data.
- scripts/validate_config.py: validation only.
- tests: invariant, transition, immutability, configuration, RNG, serialization and logging tests.
- docs/architecture.md: ownership, APIs, consistency rules and Phase-2 entry point.

## Team

| Owner | Responsibility |
|---|---|
| Sarath | Architecture, StateStore/contracts, integration, repository and merge coordination |
| Divesh | Autonomy, emergency prioritization, anti-oscillation; optional later GNN |
| Shakeel | Communication, topology, routing, relay scoring and resilience support |
| Sujal | Simulation, safety, energy, handover execution, evaluation and replay |

No algorithm can obtain a writable state reference through the supported API.
See architecture.md for the writer capability and the limits of Python immutability.
