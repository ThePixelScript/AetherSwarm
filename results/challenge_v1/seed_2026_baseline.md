# Challenge V1 Benchmark Result: Seed 2026 Baseline

> **Status**: EXPERIMENTAL BENCHMARK ARTIFACT (INTERNAL PROJECT BASELINE)
> **Authority Level**: PROJECT SIMULATION ASSUMPTIONS V1 (NOT AN ORGANIZER COMPLIANCE CLAIM)
> **Target Scenario**: `results/random/random_seed_2026.yaml`
> **Deterministic Repeat**: VERIFIED (Under tested execution environment and configuration; 100% numerically identical outputs across runs)
> **Global Compliance**: NOT ORGANIZER COMPLIANT (Multiple constraints unserviced, violated, or not implemented in core)

---

## 1. Reproduction Instructions

```bash
# Generate deterministic challenge scenario YAML with Challenge V1 profile
.venv/bin/python scripts/generate_random_scenario.py --seed 2026 --output results/random/random_seed_2026.yaml

# Execute authoritative simulation with Challenge V1 compliance layer
.venv/bin/python -c '
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import load_scenario
config = load_scenario("results/random/random_seed_2026.yaml")
runner = MissionRunner(config, seed=2026)
result = runner.run()
'
```

---

## 2. Configuration Summary

- **Random Seed**: `2026`
- **Mission Duration**: $2700.0\,\text{s}$ ($45.0\,\text{min}$), $\Delta t = 1.0\,\text{s}$, $2700$ ticks
- **Operational Center / Staging Pad**: $[-75.0, 500.0]$, radius $R_{\text{pad}} = 15.0\,\text{m}$
- **Transit Corridor**: $x \in [-75.0, 0.0]$, $y \in [400.0, 600.0]$
- **Operational Arena**: $x \in [0.0, 1000.0]$, $y \in [0.0, 1000.0]$
- **Altitude Ceiling**: $100.0\,\text{m}$ (Stored as configuration metadata; **NOT ENFORCED in 2D planar core**)
- **Speed Limit**: $5.0\,\text{m/s}$
- **Communication Range**: $100.0\,\text{m}$ Euclidean cutoff
- **Sortie Duration Cap**: $1200.0\,\text{s}$ per UAV
- **RTH Safety Margin**: $15.0\,\text{s}$
- **Single-Sortie Policy**: `enforce_single_sortie: true` (relaunch prohibited and enforced)

---

## 3. Mission & Task Outcome

- **Total Tasks**: $10$
- **Tasks Completed**: $10 / 10$ ($100.0\%$)
- **Tasks Expired / Unserviced**: $0$
- **Tasks In-Progress at Termination**: $0$
- **Simulation Completion Time**: $2700.0\,\text{s}$ (Full scheduled mission duration)

---

## 4. Per-UAV Sortie & Airborne Metrics

| UAV ID | Takeoff Time | Landing Time | Sortie Duration | Airborne at End? | Sortie Limit ($\le 1200\,\text{s}$) |
|---|:---:|:---:|:---:|:---:|:---:|
| **`uav_1`** | $0.0\,\text{s}$ | $1184.0\,\text{s}$ | $1184.0\,\text{s}$ | False (Landed) | **PASS** |
| **`uav_2`** | $0.0\,\text{s}$ | $1184.0\,\text{s}$ | $1184.0\,\text{s}$ | False (Landed) | **PASS** |
| **`uav_3`** | $189.0\,\text{s}$* | $1374.0\,\text{s}$ | $1185.0\,\text{s}$ | False (Landed) | **PASS** |
| **`uav_4`** | $0.0\,\text{s}$ | $1185.0\,\text{s}$ | $1185.0\,\text{s}$ | False (Landed) | **PASS** |
| **`uav_5`** | $0.0\,\text{s}$ | $1185.0\,\text{s}$ | $1185.0\,\text{s}$ | False (Landed) | **PASS** |

*\*Note: `uav_3` remained staged on the ground at the pad until $t = 189.0\,\text{s}$ before departing on its first task. Its continuous airborne sortie lasted exactly $1185.0\,\text{s} \le 1200.0\,\text{s}$.*

- **Max Observed Sortie Duration**: **$1185.0\,\text{s}$** (Complies with V1 assumption $\le 1200.0\,\text{s}$).

---

## 5. Constraint Audit & Safety Metrics

| Constraint / Metric | Observed Value | Status | Detailed Explanation |
|---|:---:|:---:|---|
| **Sortie Duration Limit ($\le 1200\,\text{s}$)** | $1185.0\,\text{s}$ max | **PASS** | All 5 UAVs triggered dynamic RTH and landed before $1200\,\text{s}$. |
| **Flight Duration Violations** | $0$ | **PASS** | Zero airframes exceeded the $1200\,\text{s}$ limit. |
| **Single-Sortie Relaunch** | $0$ violations | **PROHIBITED & ENFORCED** | Touchdown terminates flight duty; relaunch prohibited. Zero secondary sortie attempts occurred. |
| **Landing Location** | $0$ violations | **PASS** | All 5 UAVs landed within $R_{\text{pad}} \le 15.0\,\text{m}$ of GCS $[-75.0, 500.0]$. |
| **Geofence Compliance** | **$3$ violations** | **FAIL / VIOLATIONS OBSERVED** | **Genuine Airspace Violation**: `uav_5` clipped the northern corridor boundary ($y = 600.84\,\text{m}$ to $602.33\,\text{m} > 600.0\,\text{m}$) at ticks 157–159 while outside the arena ($x \in [-8.18, -3.40]$) during unconstrained straight-line ingress toward `poi_04` ($[491.45, 756.69]$). Corridor-containment waypoint routing is not yet implemented. |
| **Inter-UAV Separation ($\ge 20\,\text{m}$)** | $\min = 1.43\,\text{m}$ (64 violations) | **FAIL / MEASURED ONLY** | Measured in 2D. Uncoordinated straight-line trajectories crossed. No trajectory deconfliction planner exists yet. |
| **Altitude Ceiling ($100\,\text{m}$)** | Planar 2D core | **NOT ENFORCED IN CORE** | The authoritative Python core is planar 2D. `max_height=100.0` is metadata only. Downstream Webots verification may observe 3D position, but this does not make the core altitude-compliant. |
| **Detection $\to$ Reporting ($\le 10\,\text{s}$)** | Not modeled | **NOT IMPLEMENTED / NOT MEASURED** | Sensor FOV intersection and telemetry packet delivery timers are not yet modeled in the codebase. |
| **Multi-Wave Fleet Rotation** | Single sortie only | **NOT IMPLEMENTED** | Once fleet lands (by $t \approx 1374\,\text{s}$), the arena remains vacant for the remainder of the $2700\,\text{s}$ mission. |

---

## 6. Supported Communication Metrics

- **Connectivity Availability Ratio**: $0.3496$ ($34.96\%$ of mission time with active GCS route)
- **Total Network Downtime**: $1194.0\,\text{s}$
- **Model Estimated Route Latency**: $8.17\,\text{ms}$ (average multi-hop transmission delay when connected)
- **Model Estimated Route Packet Delivery Ratio (PDR)**: $0.6357$ ($63.57\%$)

---

## 7. Known Structural Infeasibility Under Single-Sortie V1

Under the single-sortie V1 assumption (no battery swapping or relaunching), a 5-UAV fleet provides a maximum cumulative flight capacity of $5 \times 1200\,\text{s} = 6000\,\text{drone-seconds}$.
- For a $2700\,\text{s}$ ($45\,\text{min}$) mission, average fleet concurrency is $\le 2.22$ UAVs.
- In Seed 2026, because all 10 tasks spawned before $t = 484\,\text{s}$, the initial single sortie was able to service all 10 POIs before executing forced RTH.
- However, in challenge scenarios with late-spawning POIs ($t > 1200\,\text{s}$), tasks appearing after fleet touchdown will be **permanently unserviced** unless multi-wave rotation is introduced. This is an authoritative structural finding of the single-sortie constraint.
