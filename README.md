# AetherSwarm

> **ARES-Swarm**: Resilient Autonomous Multi-UAV Coordination Framework for Disaster Response Operations.

AetherSwarm is a deterministic, CPU-only Stage-1 simulation framework designed for autonomous multi-UAV coordination under strict physical constraints: radio frequency range cutoffs, UAV hardware failures, battery depletion, 20-minute sortie limits, emergency point-of-interest (POI) tasks, and continuous airspace safety boundaries.

---

## 1. Problem Statement & Scope

Disaster-response aerial operations demand reliable point-of-interest reconnaissance across large disaster areas ($1000\,\text{m} \times 1000\,\text{m}$) where cellular and satellite infrastructures are degraded or unavailable. To deliver critical reconnaissance data to a Ground Control Station (GCS) located outside the search arena:

1. **RF Horizon Limitations**: UAV wireless transceivers have bounded communication range ($R_{\text{comm}} = 100.0\,\text{m}$). Points of interest situated up to $\approx 1185\,\text{m}$ from the base station require dynamic multi-hop aerial relay chains to route packets back to the GCS.
2. **Strict Reporting Deadlines**: Critical detections must reach the GCS within $10.0\,\text{s}$ of physical sensor discovery ($t_{\text{gcs}} - t_{\text{detect}} \le 10.0\,\text{s}$). Disconnected scouts cannot fulfill this requirement without active relay paths.
3. **Flight Endurance & Sortie Limits**: Airframes operate under a hard continuous airborne limit ($1200.0\,\text{s}$ / 20 minutes) and finite battery energy, requiring automated return-to-home (RTH), task handoffs, and battery replenishment rotations.
4. **Collision & Airspace Geofencing**: All airborne vehicles must maintain continuous $\ge 20.0\,\text{m}$ inter-UAV separation and observe bounded multi-zone geofence corridors between the staging pad and the operational arena.

AetherSwarm addresses these coupled challenges through deterministic single-writer state coordination, dynamic task and relay role assignment, atomic multi-hop relay chain planning, and reproducible benchmark evaluation.

---

## 2. Current Implementation Status Matrix

The repository has progressed through structured development milestones. The table below outlines the implementation and validation status of all core subsystems on the current `main` branch:

| Subsystem / Capability | Status | Phase | Verification / Test Coverage |
| :--- | :---: | :---: | :--- |
| **Deterministic Kinematics & Simulation Loop** | `VALIDATED` | Phase 0 | Discrete 1.0s ticks, constant-velocity 2D planar motion (`test_simulator.py`) |
| **Single-Writer State Store & Event Log** | `VALIDATED` | Phase 0 | Atomic command application, read-only snapshots, deterministic event sourcing (`test_state_store.py`) |
| **20m Continuous Separation Enforcement** | `VALIDATED` | Phase 1 | Spatial grid filtering, analytical bisection, zero false freezes (`test_separation_enforcement.py`) |
| **Composite Airspace Geofencing & RTH Corridors** | `VALIDATED` | Phase 1 | Staging pad, transit corridor, operational arena, non-collinear lanes (`test_geofence_enforcement.py`, `test_rth_routing.py`) |
| **20-Minute Sortie Lifecycle & RTH** | `VALIDATED` | Phase 2 | Preemptive return calculation, staged queue arrivals, landing deadlock resolution (`test_sortie_rotation.py`) |
| **Linear Battery Discharge & Ground Recharge** | `VALIDATED` | Phase 2 | 300s ground replenishment cycle, 7560 Wh capacity model (`test_battery_recharge_lifecycle.py`) |
| **Sensor FOV Perception & Telemetry Routing** | `VALIDATED` | Phase 2 | Radial $40.0\,\text{m}$ FOV detection, packet queue, end-to-end $10.0\,\text{s}$ deadline assessment (`test_detection_reporting.py`) |
| **A0 Baseline Task Allocator** | `VALIDATED` | Phase 1 | Deterministic priority-first greedy matching with battery and distance weighting (`test_a0_core_integration.py`) |
| **A1 Dynamic Relay Role Switching** | `VALIDATED` | Phase 3 | Surveyor, dedicated relay, and backup relay dynamic roles (`test_dynamic_relay_management.py`) |
| **Single-Relay Connectivity Planning** | `VALIDATED` | Phase 4 | Effective range $R_{\text{eff}} = 95.0\,\text{m}$, mid-point relay positioning up to $190\,\text{m}$ (`test_connectivity_aware_planning.py`) |
| **Multi-Hop Relay Chain Planning** | `VALIDATED` | Phase 5B | Geometric lower bound $H_{\min} = \lceil D / 95\rceil$, atomic candidate selection, airborne relay reuse, localized link handoffs (`test_multihop_relay_planning.py`) |
| **Physical Relay Readiness Verification** | `VALIDATED` | Phase 5C | Surveyor hold during `FORMING`, Gamma topological verification, station tolerance checks (`test_physical_relay_readiness.py`) |
| **Altitude-Aware Hidden POI Discovery** | `VALIDATED` | Phase 5C | Dynamic emergence, altitude-scaled detection FOV, `DiscoverTaskCommand`, normal task allocation (`test_hidden_poi_discovery.py`) |
| **Deterministic Ground Departure Sequencing** | `VALIDATED` | Phase 5C | Staging separation, taxi clearances, zero launch collision risks (`test_departure_sequencing.py`) |
| **Webots 3D Visualization & Spatial Verifier** | `VALIDATED` | Phase 3 | Cyberbotics Webots R2025a supervisor controller, trace replay, independent spatial checks (`test_webots_control_layer.py`) |

---

## 3. Simulation Model & Architectural Principles

### 2D Planar Core Simulation
The authoritative simulation core (`src/ares_swarm/`) models vehicle kinematics in 2D horizontal coordinates ($x, y$) at discrete 1.0-second timesteps. The altitude limit ($100.0\,\text{m}$) is carried as configuration metadata and verified in the downstream 3D visualization layer.

### Single-Writer State Architecture
All state mutations are mediated exclusively through typed commands dispatched to the [`StateStore`](src/ares_swarm/core/state_store.py). Autonomy planners, safety assessors, and network analyzers receive immutable [`StateSnapshot`](src/ares_swarm/core/state_store.py) instances, guaranteeing zero state race conditions and bitwise-reproducible execution traces.

### Authoritative System Boundary
- **Headless Core (`src/ares_swarm/`)**: The single authoritative source of truth. All flight decisions, task allocation, RF link evaluation, battery depletion, failure injection, and metric calculations occur strictly within Python under Linux/WSL.
- **Webots 3D Robotics Layer (`visualization/webots/`)**: A downstream consumer and independent spatial verifier. It ingests immutable JSON simulation traces exported by the core simulation to render 3D quadrotor flight, ground grids, and HUD telemetry, while independently auditing pairwise separation and boundary compliance. Webots commands zero simulation logic and introduces no physics overrides.

---

## 4. Quickstart Guide

### Prerequisites
- Linux or Windows Subsystem for Linux (WSL 2) with Ubuntu 22.04 / 24.04.
- Python 3.12 (or virtual environment with Python 3.10+).
- Git.

### Environment Setup
Clone the repository and set up the Python virtual environment:

```bash
git clone https://github.com/ThePixelScript/AetherSwarm.git
cd AetherSwarm

# Create and activate virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install dependencies and local package in editable mode
pip install --upgrade pip
pip install -r requirements.txt
pip install -e .
```

### Running Test Suites
Verify repository integrity against the full deterministic test suite:

```bash
pytest tests/ -q
```
*Current test suite status: 463 passed in ~13-15 seconds.*

To run targeted subsystem suites:
```bash
# Physical relay readiness tests (5 tests)
pytest tests/autonomy/test_physical_relay_readiness.py

# Hidden POI emergence and altitude-aware discovery (13 tests)
pytest tests/simulation/test_hidden_poi_discovery.py

# Ground departure sequencing and staging separation (9 tests)
pytest tests/safety/test_departure_sequencing.py

# Multi-hop relay planning and deadline tests (9 tests)
pytest tests/test_multihop_relay_planning.py

# Battery lifecycle and recharge rotation tests (7 tests)
pytest tests/test_battery_recharge_lifecycle.py
```

### Generating Final-Profile Scenarios
Generate a reproducible working scenario containing exactly 5 known POIs and 5–7 emerging hidden POIs with the 800 m circular radius constraint:

```bash
# Generate scenario YAML and export 2700-tick Webots trace
python scripts/generate_scenario.py \
  --seed 2026 \
  --final-profile \
  --max-ticks 2700 \
  --output-scenario scenarios/random_seed_2026.yaml \
  --output-trace visualization/webots/data/random_scenario_trace.json
```

### Running Full-Mission Physical Readiness Audit
Execute tick-by-tick post-step Gamma connectivity verification for ticks 1–1700:

```bash
python scripts/audit_physical_readiness.py \
  --scenario scenarios/random_seed_2026.yaml \
  --output results/audit_1700.json \
  --ticks 1700
```
*Validated result on seed 2026: 1700/1700 PASS, 0 disconnected airborne ticks, 0 separation violations, 0 geofence violations.*

### Webots 3D Visualization Playback
To view the generated simulation trace in Cyberbotics Webots R2025a:

```powershell
# From Windows PowerShell (using the project launcher):
.\scripts\launch_random_webots.ps1 -Seed 2026
```
The launcher automatically resolves the world file at `visualization/webots/worlds/uavx_round1.wbt` via mapped drive `Z:` or clean Windows path and initiates replay with independent spatial auditing.

Alternatively, run headless spatial verification directly from the terminal without opening the Webots GUI:
```bash
python visualization/webots/controllers/aetherswarm_supervisor/aetherswarm_supervisor.py visualization/webots/data/random_scenario_trace.json
```

---

## 5. Repository Directory Layout

```text
AetherSwarm/
├── configs/                             # Centralized YAML configuration files
│   ├── default.yaml                     # Authoritative baseline configuration
│   └── challenge_profile.yaml           # Opt-in challenge compliance overrides
├── docs/                                # Authoritative engineering & architecture documentation
├── scenarios/                           # Scenario definitions (E1 benchmark & randomized sets)
│   ├── poc_round1.yaml                  # Frozen Benchmark E1 scenario
│   └── ...
├── scripts/                             # Utility scripts & experiment runners
│   ├── generate_scenario.py             # Reproducible randomized scenario generator
│   └── run_fleet_size_sweep.py          # Fleet size scaling experiment harness
├── src/ares_swarm/                      # Authoritative simulation core
│   ├── autonomy/                        # Task allocators & multi-hop connectivity planners
│   ├── communication/                   # RF channel models, graph topology & routing
│   ├── core/                            # State store, models, events, kinematics, simulator
│   ├── energy/                          # Battery models & discharge curves
│   ├── evaluation/                      # Mission performance metrics & reporter
│   ├── safety/                          # Separation enforcer, geofencing, safety assessor
│   ├── simulation/                      # Mission runner, scenario loader, main entry points
│   └── telemetry/                       # Sensor detection, FOV perception & packet routing
├── tests/                               # Comprehensive Pytest test suite (463 tests)
└── visualization/                       # Downstream visualization tools
    └── webots/                          # Cyberbotics Webots R2025a supervisor & proto assets
```

---

## 6. Authoritative Documentation Index & Precedence

> [!NOTE]
> **Documentation Precedence Rule**:
> When consulting project documentation, **CURRENT AUTHORITATIVE** documents reflect the active codebase baseline on branch `main` and take precedence over earlier **HISTORICAL / FEATURE RECORD** specifications. Feature record documents preserve the design rationale of specific milestones, while experiment evidence files document immutable empirical outputs.

### Current Authoritative Documentation (Active Baseline)
| Document | Classification | Focus & Scope |
| :--- | :---: | :--- |
| [`SYSTEM_ARCHITECTURE.md`](docs/SYSTEM_ARCHITECTURE.md) | `CURRENT AUTHORITATIVE` | Component decomposition, single-writer state store, exact 11-stage tick loop, and data flow. |
| [`AUTONOMY_AND_PLANNING.md`](docs/AUTONOMY_AND_PLANNING.md) | `CURRENT AUTHORITATIVE` | Task allocation hierarchy (A0, A1, Phase 4 single-relay, Phase 5B multi-hop planning). |
| [`RELAY_AND_MULTIHOP.md`](docs/RELAY_AND_MULTIHOP.md) | `CURRENT AUTHORITATIVE` | Multi-hop relay chains, geometric bounds ($H_{\min}, K_{\min}$), atomic selection, handoffs, and recovery. |
| [`COMMUNICATION_AND_TELEMETRY.md`](docs/COMMUNICATION_AND_TELEMETRY.md) | `CURRENT AUTHORITATIVE` | Simulated RF propagation, NetworkX routing, sensor FOV detection, and the $10\,\text{s}$ reporting pipeline. |
| [`SAFETY_ENDURANCE_AND_RTH.md`](docs/SAFETY_ENDURANCE_AND_RTH.md) | `CURRENT AUTHORITATIVE` | 20m separation enforcer, composite geofence corridors, 20-min sortie bounds, and recharge mechanics. |
| [`CONFIGURATION_AND_SCENARIOS.md`](docs/CONFIGURATION_AND_SCENARIOS.md) | `CURRENT AUTHORITATIVE` | Centralized 4-tier configuration architecture and reproducible scenario generation mechanics. |
| [`WEBOTS_INTEGRATION.md`](docs/WEBOTS_INTEGRATION.md) | `CURRENT AUTHORITATIVE` | Downstream 3D visualization, supervisor controller, trace playback, and independent spatial auditing. |
| [`TESTING_AND_REPRODUCIBILITY.md`](docs/TESTING_AND_REPRODUCIBILITY.md) | `CURRENT AUTHORITATIVE` | Test organization, deterministic tie-breaking conventions, and regression verification commands. |
| [`EXPERIMENTS_AND_VALIDATION.md`](docs/EXPERIMENTS_AND_VALIDATION.md) | `CURRENT AUTHORITATIVE` | Experimental records: fleet sizing sweep ($N=1\dots16$), Phase 4 evaluation, and Phase 5B benchmarks. |
| [`LIMITATIONS_AND_ROADMAP.md`](docs/LIMITATIONS_AND_ROADMAP.md) | `CURRENT AUTHORITATIVE` | Documented system limitations, edge-case risks, and technical roadmap for Phase 5C and Phase 6. |

### Historical Specifications & Milestone Feature Records
| Document | Classification | Role & Context |
| :--- | :---: | :--- |
| [`CHALLENGE_MISSION_CONTRACT.md`](docs/CHALLENGE_MISSION_CONTRACT.md) | `HISTORICAL RECORD` | Baseline competition rules and traceability matrix. |
| [`CHALLENGE_ASSUMPTIONS_V1.md`](docs/CHALLENGE_ASSUMPTIONS_V1.md) | `HISTORICAL RECORD` | Initial V1 frozen simulation conventions; annotated with Phase 2 sortie rotation updates. |
| [`CHALLENGE_COMPLIANCE_LAYER_V1.md`](docs/CHALLENGE_COMPLIANCE_LAYER_V1.md) | `HISTORICAL RECORD` | Initial compliance layer specification from Phase 1. |
| [`CONFIGURATION_ARCHITECTURE.md`](docs/CONFIGURATION_ARCHITECTURE.md) | `FEATURE RECORD` | Phase 2 configuration refactoring architecture. |
| [`GEOFENCE_ENFORCEMENT_V1.md`](docs/GEOFENCE_ENFORCEMENT_V1.md) | `FEATURE RECORD` | Phase 1 geofence enforcer design specification. |
| [`SEPARATION_ENFORCEMENT_V1.md`](docs/SEPARATION_ENFORCEMENT_V1.md) | `FEATURE RECORD` | Phase 1 spatial grid separation enforcer design specification. |
| [`SORTIE_ROTATION.md`](docs/SORTIE_ROTATION.md) | `FEATURE RECORD` | Phase 2 multi-wave sortie rotation and landing deadlock resolution design. |
| [`DYNAMIC_RELAY_ROLES.md`](docs/DYNAMIC_RELAY_ROLES.md) | `FEATURE RECORD` | Phase 3 dynamic relay role states (`SCOUT`, `RELAY`, `BACKUP_RELAY`). |
| [`CONNECTIVITY_AWARE_PLANNING.md`](docs/CONNECTIVITY_AWARE_PLANNING.md) | `FEATURE RECORD` | Phase 4 single-relay connectivity planner specification ($D \le 190\,\text{m}$). |
| [`DETECTION_REPORTING_ARCHITECTURE_V1.md`](docs/DETECTION_REPORTING_ARCHITECTURE_V1.md) | `FEATURE RECORD` | Telemetry packet dissemination and FOV perception specification. |
| [`algorithms/task-allocation.md`](docs/algorithms/task-allocation.md) | `FEATURE RECORD` | Foundational A0 greedy allocation algorithm design and math formulation. |
| [`visualization/webots/README.md`](visualization/webots/README.md) | `CURRENT AUTHORITATIVE` | Operational guide for launching, configuring, and verifying Webots R2025a playback. |

### Experiment Evidence & Benchmark Output
| Document | Classification | Contents |
| :--- | :---: | :--- |
| [`FLEET_SIZE_FEASIBILITY.md`](docs/FLEET_SIZE_FEASIBILITY.md) | `EXPERIMENT EVIDENCE` | Empirical data and analysis from the Phase 1 fleet size scaling sweep ($N=1\dots16$). |
| [`DETECTION_REPORTING_VALIDATION_V1.md`](docs/DETECTION_REPORTING_VALIDATION_V1.md) | `EXPERIMENT EVIDENCE` | Phase 2 telemetry reporting benchmark findings across initial test scenarios. |
| [`RANDOMIZED_SCENARIO.md`](docs/RANDOMIZED_SCENARIO.md) | `FEATURE RECORD` | Uniform scenario generator verification records and seed validation. |
| [`phase4_comparison_results.json`](docs/phase4_comparison_results.json) | `EXPERIMENT EVIDENCE` | Authoritative JSON benchmark results comparing Phase 4 against seeds 2026, 42, 5001. |

---

## 7. License & Attribution

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
