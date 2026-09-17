> Historical verification before repository consolidation. Commands below use the current portable environment layout.

# M0 communication verification

These historical results were obtained before source consolidation into the
current clone. Reproduction commands now use a project-local .venv.
See integration-audit.md for the current repository and cross-branch status.

## Executed results

Environment: CPython 3.12.14, NetworkX 3.6.1, Windows.
Before implementation: all 87 Phase-1 tests passed.

| Check | Actual result |
|---|---|
| Channel tests | 23 passed |
| Graph tests | 12 passed |
| Connectivity tests | 12 passed |
| Routing tests | 10 passed |
| Snapshot integration/contract tests | 14 passed |
| All focused communication tests | 71 passed in 1.37 s |
| Existing Phase-1 tests only | 87 passed in 0.31 s |
| Full suite | 158 passed in 1.67 s |
| Compile source using Python compile() | 26 files passed |
| Import channel/graph/connectivity/routing/analysis | Passed |
| Existing config/basic scenario validation | Passed |
| pip check | No broken requirements |
| Updated editable package installation | Passed |

Times are the actual local test-run durations, not performance claims about missions.
No simulation mission, packet experiment, benchmark or hardware test was executed.

## Test commands

From the project root, using a project-local virtual environment:

~~~powershell
$env:PYTHONDONTWRITEBYTECODE='1'
.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider tests\communication
.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider tests --ignore=tests\communication
.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider
.venv\Scripts\python.exe scripts\validate_config.py --config configs\default.yaml --scenario scenarios\basic.yaml
.venv\Scripts\python.exe -m pip check
~~~

Individual test modules were executed after their implementation in channel,
graph, connectivity, routing and integration order. Compilation used compile()
on every source file so validation did not write bytecode outside the configured
workspace. Hash-seed tests launched separate Python processes.

## Files added

All paths below are relative to the project root.

~~~text
src/ares_swarm/communication/__init__.py
src/ares_swarm/communication/channel.py
src/ares_swarm/communication/graph.py
src/ares_swarm/communication/connectivity.py
src/ares_swarm/communication/routing.py
src/ares_swarm/communication/analysis.py
tests/communication/conftest.py
tests/communication/test_channel.py
tests/communication/test_graph.py
tests/communication/test_connectivity.py
tests/communication/test_routing.py
tests/communication/test_integration.py
docs/algorithms/communication-model.md
docs/algorithms/connectivity-analysis.md
docs/algorithms/routing.md
docs/m0-verification.md
~~~

## Files modified

~~~text
src/ares_swarm/interfaces/communication.py
pyproject.toml
constraints.txt
README.md
docs/architecture.md
~~~

No existing core model, state store, transition, event, clock, config schema, RNG or
Phase-1 test file was changed. The new analysis.py only composes the communication
pipeline behind the existing protocol.

## Resolved contract differences

1. Core LinkState disallows infinite ETX. ChannelEvaluation.link=None and etx=None
   represent unusable/zero-PDR pairs; usable edges reuse canonical core LinkState.
2. The original NetworkAnalysis is minimal. Additive fields preserve its existing
   constructor and old JSON; gcs_id=None explicitly identifies legacy partial views.
3. The workstream plan illustrates GCS-first routes. M0 keeps the existing core's
   UAV-first routes and tests the lexicographic tie-break in that direction.
4. Existing config has only range/base latency/base loss. Formulas reuse those fields.
   Optional explicit LinkCondition inputs add current impairment hooks without
   changing event/config semantics or adding an event scheduler.
5. Authoritative stored link observations are not an impairment schedule. The graph
   is rebuilt from current endpoint states/config/explicit conditions. No derived
   result is written back into core.
6. Derived graph links are undirected canonical pairs. Core storage permits directed
   link records; M0 does not change that core convention or commit graph records.

## Completion and next step

Local M0 implementation and required validation are complete. No implementation
blocker remains. The repository was already untracked on inspection; this work
does not claim commits, pushes, PR creation or coordination with absent team members.

M1 entry point: Sarath reviews the additive NetworkAnalysis contract and Divesh can
consume its routes/connectivity/topology evidence for communication-aware decisions.
Sujal can invoke BaselineCommunicationAnalyzer after each future snapshot.
M1 work has not started. Reliability-weighted routing/A2, relay support, handover,
recovery planning and GNN remain deferred.
