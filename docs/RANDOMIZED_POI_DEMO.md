# Randomized POI Scenario Generator (Demo / Experimental Only)

> [!WARNING]
> **DEMO / EXPERIMENTAL USE ONLY**
> This tool is strictly designed for visual demonstrations in Webots R2025a to display realistic, non-grid POI distributions.
> - It does **NOT** modify or replace canonical competition benchmark scenarios (e.g. [`scenarios/poc_round1.yaml`](file:///home/dell/swarm_ws/AetherSwarm/scenarios/poc_round1.yaml)).
> - It does **NOT** alter the frozen, authoritative E1 baseline trace ([`visualization/webots/data/e1_authoritative_trace.json`](file:///home/dell/swarm_ws/AetherSwarm/visualization/webots/data/e1_authoritative_trace.json)).
> - It does **NOT** alter core autonomy, communication, or safety enforcement semantics.

---

## 1. Architectural Overview

To demonstrate organic, non-grid swarm operations without compromising simulation fidelity or creating visual-only synthetic paths, AetherSwarm implements a strict 5-stage pipeline:

```mermaid
flowchart LR
    A["1. Rejection Sampling<br/>(10 POIs, Spacing &ge; 40m, Margins)"] --> B["2. Deterministic Scenario YAML<br/>(scenarios/demo_random_seed_N.yaml)"]
    B --> C["3. Authoritative Simulation<br/>(MissionRunner + A1 Autonomy + Safety)"]
    C --> D["4. Immutable Trace Export<br/>(visualization/webots/data/random_demo_trace.json)"]
    D --> E["5. Webots 3D Visualization<br/>(aetherswarm_supervisor dynamic placement)"]
```

### Architectural Principles:
1. **Zero Webots Autonomy / No Controller Randomization**: The Webots supervisor controller is strictly a playback and spatial verification engine. POI positions and spawn times are **never** randomized inside Webots.
2. **Authoritative Consistency**: The generated POI positions in the scenario YAML are the ground-truth task targets dispatched by [`A1TaskAllocator`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/autonomy/a1_allocator.py), navigated to by [`SimulationEngine`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/core/simulator.py), and safety-checked by [`SeparationEnforcer`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/safety/separation.py) and [`GeofenceEnforcer`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/safety/geofence.py).
3. **Dynamic 3D Beacon Placement**: At trace load time, the Webots supervisor queries the authoritative task positions from tick 0 and updates the `translation` fields of the 3D POI target nodes (`POI_01` .. `POI_10`). The physical 3D ground markers precisely match the flight targets.
4. **Deterministic Reproducibility**: Given a seed $S$, the PRNG sequence generates identical POIs, identical priorities, identical spawn times, identical simulation results, and an identical trace bit-for-bit.

---

## 2. Rejection Sampling & Geometric Constraints

POIs are generated using spatial rejection sampling subject to the following rules:

1. **Count**: Exactly 10 POIs (`poi_01` to `poi_10`).
2. **Operational Arena Containment**:
   - Standard operational arena is $x \in [0.0, 1000.0]$, $y \in [0.0, 1000.0]$.
   - Boundary margin: configurable (default $\ge 30.0$ m).
   - Ingress corridor exclusion: $x \ge \text{margin} > 0.0$ guarantees no POI is ever placed in the staging/corridor area ($x \le 0.0$).
3. **Anti-Clustering & Anti-Grid**:
   - Coordinates are drawn from continuous uniform distributions rather than discretized grid steps.
   - Rejection condition: if $\text{dist}(p_\text{new}, p_i) < \text{min\_spacing}$ for any existing $p_i$, $p_\text{new}$ is rejected and resampled.
4. **Operational Swarm Reach (Default Demonstration Corridor)**:
   - By default, POIs are sampled within $x \in [60.0, 420.0]$ and $y \in [220.0, 780.0]$ to demonstrate multi-hop RF mesh connectivity and active servicing within the 1200s battery endurance limit.
   - For unrestricted arena sampling, pass `--full-arena` ($x, y \in [30.0, 970.0]$).
5. **Deterministic Ordering**:
   - When multiple POIs have identical spawn times, ties are deterministically resolved by lexicographic task ID sorting (`(t["spawn_time"], t["id"])`).

---

## 3. CLI Usage

The generator script is located at [`scripts/generate_random_demo.py`](file:///home/dell/swarm_ws/AetherSwarm/scripts/generate_random_demo.py).

### Basic Command (Generates YAML, runs simulation, exports trace):
```bash
# In WSL:
.venv/bin/python scripts/generate_random_demo.py --seed 2026

# Output:
# Scenario: scenarios/demo_random_seed_2026.yaml
# Trace:    visualization/webots/data/random_demo_trace.json
```

### CLI Arguments:
| Argument | Type | Default | Description |
|---|---|---|---|
| `--seed` | `int` | `2026` | PRNG seed for deterministic scenario and trace generation |
| `--num-pois` | `int` | `10` | Number of POIs to place (must be 10 for UAV-X challenge) |
| `--min-spacing` | `float` | `40.0` | Minimum pairwise 2D Euclidean distance between POIs (meters) |
| `--margin` | `float` | `30.0` | Minimum clearance distance from arena boundaries (meters) |
| `--spawn-start` | `float` | `0.0` | Earliest POI appearance time (seconds) |
| `--spawn-end` | `float` | `300.0` | Latest POI appearance time (seconds) |
| `--full-arena` | `flag` | `False` | Expand sampling across full $[30, 970] \times [30, 970]$ arena |
| `--output-scenario` | `path` | `scenarios/demo_random_seed_{seed}.yaml` | Destination scenario YAML file |
| `--output-trace` | `path` | `visualization/webots/data/random_demo_trace.json` | Destination JSON trace file |
| `--max-ticks` | `int` | `None` | Optional tick limit (default: full duration 2700 ticks) |
| `--no-run` | `flag` | `False` | Generate scenario YAML only without running simulation |

---

## 4. Validated Demonstration Seeds

Three distinct seeds were generated, executed through the authoritative simulation engine, and validated in the independent Webots spatial verification engine:

### Summary Table

| Metric | Seed 42 | Seed 101 | Seed 2026 |
|---|---|---|---|
| **POIs Generated** | 10 | 10 | 10 |
| **Configured Min Spacing** | 40.0 m | 40.0 m | 40.0 m |
| **Actual Min Spacing** | **78.11 m** | **45.86 m** | **49.95 m** |
| **Average Spacing** | 254.21 m | 214.74 m | 250.97 m |
| **Spatial Bounds (X)** | $[91.30, 392.03]$ | $[87.66, 390.31]$ | $[102.88, 419.88]$ |
| **Spatial Bounds (Y)** | $[234.01, 706.81]$ | $[329.06, 681.10]$ | $[233.85, 715.11]$ |
| **Spawn Window** | $28.1\text{s} - 267.7\text{s}$ | $2.0\text{s} - 277.2\text{s}$ | $72.0\text{s} - 241.5\text{s}$ |
| **Tasks Completed** | **10 / 10 (100%)** | **10 / 10 (100%)** | **10 / 10 (100%)** |
| **Min Separation Observed** | **20.00 m** | **20.00 m** | **20.00 m** |
| **Separation Violations** | **0** | **0** | **0** |
| **Geofence Violations** | **0** | **0** | **0** |
| **Altitude Violations** | **0** | **0** | **0** |
| **Trace Size** | 23.30 MB | 23.25 MB | 26.47 MB |

### Reproduction Commands:
```bash
# Seed 42
.venv/bin/python scripts/generate_random_demo.py --seed 42 --output-trace visualization/webots/data/random_demo_seed_42.json

# Seed 101
.venv/bin/python scripts/generate_random_demo.py --seed 101 --output-trace visualization/webots/data/random_demo_seed_101.json

# Seed 2026 (Default Demo Trace)
.venv/bin/python scripts/generate_random_demo.py --seed 2026 --output-trace visualization/webots/data/random_demo_trace.json
```

---

## 5. Webots Visualization Workflow

The Webots supervisor controller [`visualization/webots/controllers/aetherswarm_supervisor/aetherswarm_supervisor.py`](file:///home/dell/swarm_ws/AetherSwarm/visualization/webots/controllers/aetherswarm_supervisor/aetherswarm_supervisor.py) supports several convenient ways to replay the randomized demo:

### Method A: Direct Command-Line Execution (Standalone or Webots)
```bash
.venv/bin/python visualization/webots/controllers/aetherswarm_supervisor/aetherswarm_supervisor.py random
# or provide path directly:
.venv/bin/python visualization/webots/controllers/aetherswarm_supervisor/aetherswarm_supervisor.py visualization/webots/data/random_demo_trace.json
```

### Method B: Environment Variable Selector
Set `AETHERSWARM_SCENARIO` before launching Webots or the supervisor:
```bash
export AETHERSWARM_SCENARIO=random
```

### Method C: Webots Robot customData
In Webots R2025a, open [`visualization/webots/worlds/uavx_round1.wbt`](file:///home/dell/swarm_ws/AetherSwarm/visualization/webots/worlds/uavx_round1.wbt). In the scene tree, select the `AetherSwarmSupervisor` Robot node and set its `customData` field to:
```text
random
```
The supervisor will automatically load `visualization/webots/data/random_demo_trace.json`, update the 3D POI beacons to match the randomized scenario, and replay the flight paths with full HUD telemetry.
