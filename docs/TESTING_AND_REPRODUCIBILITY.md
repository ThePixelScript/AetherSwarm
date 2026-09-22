# Testing Strategy & Reproducibility Architecture

## 1. Testing Philosophy & Quality Standard

AetherSwarm enforces a rigorous verification standard to ensure that autonomous swarm decision-making is reliable, mathematically grounded, and 100% reproducible.

### Fundamental Verification Principles
1. **Zero Flakiness via Decoupled Time**: Tests never assert against wall-clock delays (`time.sleep` is strictly prohibited). All temporal dynamics advance via discrete, deterministic simulation ticks ($\Delta t = 1.0\,\text{s}$).
2. **Absolute Seed-Based Determinism**: Given identical random seeds and scenario inputs, simulation trajectories, routing tables, and domain event logs produce identical bitwise results across different hardware architectures and operating systems.
3. **Inviolable Regression Baselines**: The initial benchmark scenario (`poc_round1.yaml`) and legacy test suites are never altered or weakened to accommodate new features. Every enhancement must strictly maintain 100% backward pass rates.
4. **Fast Local Execution**: The entire 333-test regression suite executes in under 20 seconds on a standard developer workstation, enabling immediate feedback on every commit.

---

## 2. Test Suite Organization

The test directory (`tests/`) mirrors the architecture of the production codebase:

```text
tests/
├── autonomy/                            # 43 tests: A0 greedy allocation & A1 destination-aware routing
│   ├── test_a0_core_integration.py      # 9 tests
│   ├── test_a1_allocator.py             # 18 tests
│   ├── test_a1_destination_aware.py     # 5 tests
│   └── test_task_allocator.py           # 16 tests
├── communication/                       # 86 tests: RF channel propagation, graph topologies & routing
│   ├── test_channel.py                  # 23 tests
│   ├── test_connectivity.py            # 12 tests
│   ├── test_failure_topology.py         # 4 tests
│   ├── test_graph.py                   # 12 tests
│   ├── test_integration.py             # 4 tests
│   ├── test_m2_isolation.py             # 7 tests
│   ├── test_route_pdr.py                # 7 tests
│   ├── test_routing.py                  # 10 tests
│   ├── test_scenario.py                 # 13 tests
│   └── test_weighted_routing.py         # 11 tests
├── core/                                # 38 tests: StateStore, models, kinematics & scheduler
│   ├── test_event_scheduler.py          # 3 tests
│   ├── test_kinematics.py               # 4 tests
│   ├── test_models.py                   # 3 tests
│   ├── test_simulator.py                # 28 tests
│   └── test_state_store.py              # 5 tests
├── energy/                              # 3 tests: Battery discharge & reserve thresholds
│   └── test_battery.py                  # 3 tests
├── evaluation/                          # 2 tests: Metric calculations & reporting
│   └── test_metrics.py                  # 2 tests
├── integration/                         # 2 tests: End-to-end M0 simulation loop
│   └── test_m0_loop.py                  # 2 tests
├── safety/                              # 40 tests: Separation, geofence, and challenge compliance
│   ├── test_challenge_compliance_v1.py  # 13 tests
│   ├── test_geofence_enforcement.py     # 12 tests
│   ├── test_safety_assessor.py          # 4 tests
│   └── test_separation_enforcement.py   # 11 tests
├── simulation/                          # 11 tests: Mission runner, RTH, and scenario execution
│   ├── test_demo_paired_a0_a1.py        # 1 test
│   ├── test_mission_runner.py           # 5 tests
│   ├── test_poc_scenario.py             # 4 tests
│   └── test_rth_completion.py           # 1 test
├── telemetry/                           # 15 tests: FOV perception, packet routing & 10s deadline
│   └── test_detection_reporting.py      # 15 tests
├── visualization/                       # 14 tests: Trace playback and Webots supervisor controls
│   ├── test_replay.py                   # 3 tests
│   └── test_webots_control_layer.py     # 11 tests
└── Root Subsystem Suites:               # 79 tests: High-level architectural milestones
    ├── test_aetherswarm_config.py       # 8 tests
    ├── test_connectivity_aware_planning.py # 9 tests (Phase 4 single-relay)
    ├── test_dynamic_relay_management.py # 8 tests (Phase 3 dynamic roles)
    ├── test_multihop_relay_planning.py  # 9 tests (Phase 5B multi-hop chains)
    ├── test_random_scenario_generator.py # 11 tests
    └── test_sortie_rotation.py          # 7 tests (Phase 2 sortie rotation)
```

**Total Active Test Count**: **333 tests passing** (100% pass rate).

---

## 3. Deterministic Tie-Breaking & Precision Standards

To eliminate platform-specific variations:
1. **Utility Rounding**: All assignment utility scores are rounded to 8 decimal places (`round(score, 8)`). This eliminates IEEE 754 floating-point epsilon variations across Intel/AMD/ARM architectures.
2. **Lexicographical Tie-Breaking**: When candidate utility scores evaluate identically:
   - Tasks are resolved in lexicographical ascending order of `task.id`.
   - UAV candidates are selected in lexicographical ascending order of `uav.id`.
3. **Reproducible Pseudorandomness**: All randomized scenario generation and failure injection harnesses use explicit instances of `random.Random(seed)` or `numpy.random.default_rng(seed)`. Global random seeds are never relied upon.

---

## 4. Test Execution Commands

### 4.1 Running the Full Test Suite
Execute all 333 tests using Pytest:

```bash
pytest
```

Expected output:
```text
============================= 333 passed in 17.50s =============================
```

### 4.2 Running Targeted Phase & Subsystem Suites

```bash
# 1. Phase 5B Multi-Hop Relay Planning (9 tests)
pytest tests/test_multihop_relay_planning.py

# 2. Phase 4 Connectivity-Aware Planning (9 tests)
pytest tests/test_connectivity_aware_planning.py

# 3. Phase 3 Dynamic Relay Roles (8 tests)
pytest tests/test_dynamic_relay_management.py

# 4. Phase 2 Sortie Rotation & Landing Deadlock (7 tests)
pytest tests/test_sortie_rotation.py

# 5. Challenge Compliance Layer V1 (13 tests)
pytest tests/safety/test_challenge_compliance_v1.py

# 6. Detection & 10s Reporting Pipeline (15 tests)
pytest tests/telemetry/test_detection_reporting.py

# 7. 20m Separation Enforcer (11 tests)
pytest tests/safety/test_separation_enforcement.py

# 8. Webots Visualizer & Replay Layer (14 tests)
pytest tests/visualization/
```

### 4.3 Running with Verbose Output & Timing
```bash
# Run with detailed test-name logging and timing summary
pytest -v --durations=10
```
