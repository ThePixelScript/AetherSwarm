> Historical verification before repository consolidation. Commands below use the current portable environment layout.

# Phase 1 verification

Executed on Windows using CPython 3.12.14. No mission simulation was executed.

## Actual results

- Editable package installation: passed.
- Installation using requirements.txt with --no-build-isolation: passed.
- Pytest, final editable code: 87 passed in 0.32 seconds.
- Non-editable wheel build and installation: passed.
- Pytest against installed site-packages wheel: 87 passed in 0.38 seconds.
- Installed CLI with default config/basic scenario: valid=true, seed=42, scenario=basic.
- Python compileall and interface import smoke checks: passed.
- pip check: no broken requirements.
- Editable installation restored after wheel verification for development.

The first dependency download required network permission. The initial wheel command
could not write pip's external cache; the build succeeded with --no-cache-dir.
No unresolved test or installation failure remains. Linux/macOS execution and hostile
Python reflection resistance were not tested or claimed.

## Commands executed

From the uav-x directory (use a project-local environment):

~~~powershell
.venv\Scripts\python.exe -m pip install --no-build-isolation -r requirements.txt
.venv\Scripts\python.exe -m pytest -q
.venv\Scripts\python.exe scripts\validate_config.py --config configs\default.yaml --scenario scenarios\basic.yaml
.venv\Scripts\ares-validate-config.exe --config configs\default.yaml --scenario scenarios\basic.yaml
.venv\Scripts\python.exe -m compileall -q src
.venv\Scripts\python.exe -m pip check
.venv\Scripts\python.exe -m pip wheel --no-cache-dir --no-deps --no-build-isolation --wheel-dir dist .
.venv\Scripts\python.exe -m pip install --no-deps --force-reinstall dist\ares_swarm-0.1.0-py3-none-any.whl
.venv\Scripts\python.exe -m pytest -q
.venv\Scripts\python.exe -m pip install --no-build-isolation --no-deps -e .
~~~

## New file inventory

All project files are new; no pre-existing source files were replaced.

~~~text
uav-x/
  .gitignore
  README.md
  pyproject.toml
  requirements.txt
  constraints.txt
  configs/default.yaml
  scenarios/basic.yaml
  scripts/validate_config.py
  src/ares_swarm/__init__.py
  src/ares_swarm/core/
    __init__.py
    enums.py
    models.py
    snapshot.py
    transitions.py
    state_store.py
    events.py
    config.py
    random_manager.py
    logging.py
    exceptions.py
    validation.py
    serialization.py
  src/ares_swarm/interfaces/
    __init__.py
    state.py
    autonomy.py
    communication.py
    safety.py
    evaluation.py
  tests/
    conftest.py
    test_models.py
    test_state_store.py
    test_transitions.py
    test_config.py
    test_random_manager.py
    test_serialization.py
    test_logging.py
    test_interfaces.py
  docs/
    architecture.md
    phase1-verification.md
~~~

A workspace-root .gitignore also excludes the local .venv. Generated build,
egg-info, cache and dist artifacts are ignored. validation.py, serialization.py,
interfaces/state.py, constraints.txt and additional tests keep validation, codecs,
read access and dependency reproducibility separate from domain/state logic.

## Scope boundary

No allocation, channel, routing, relay, fault recovery, handover execution, collision,
energy dynamics, visualization or GNN algorithms exist. Lifecycle validation and
referential cleanup are structural consistency rules only. Phase 2 starts with
SimulationEngine as specified in architecture.md; it has not been implemented.
