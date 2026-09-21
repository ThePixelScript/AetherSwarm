# Detection → GCS Reporting V1: Implementation Evidence & Validation

> **Status**: IMPLEMENTED & VALIDATED
> **Authority Level**: INTERNAL PROJECT VALIDATION EVIDENCE (PROJECT SIMULATION ASSUMPTIONS V1)
> **Parent Document**: [`docs/DETECTION_REPORTING_ARCHITECTURE_V1.md`](file:///home/dell/swarm_ws/AetherSwarm/docs/DETECTION_REPORTING_ARCHITECTURE_V1.md)
> **Committed Baseline**: `c66ac26` (`feat(telemetry): implement detection to gcs reporting v1`)
> **Protected Legacy Baseline**: `scenarios/poc_round1.yaml` (E1 Benchmark)

---

## 1. Scope & System Boundary

The **Detection → GCS Reporting V1** subsystem implements perception-based POI discovery, delay-tolerant telemetry packet buffering, and GCS delivery deadline tracking for the AetherSwarm simulation platform.

### Boundary Definitions
- **Opt-In Execution**: The detection and reporting pipeline is strictly opt-in via `challenge_profile.detection_pipeline.enabled = true`. By default (`enabled: false`), the subsystem remains uninstantiated, ensuring zero runtime overhead and preserving existing E1 behavior.
- **Gamma Communication Layer Relation**: The telemetry subsystem does not implement a secondary network model or routing engine. It performs read-only queries against the existing Gamma communication analyzer ([`BaselineCommunicationAnalyzer`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/communication/analysis.py)) and its derived routing topology ([`NetworkAnalysis.routes_to_gcs`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/interfaces/communication.py)).
- **Separation of Concerns**: Perception discovery is strictly independent of autonomy task assignment, task servicing, or task completion.

---

## 2. Implementation Status Table

| Capability / Requirement | Category | Details / Current Implementation Status |
| :--- | :--- | :--- |
| **Sensor FOV Detection** | `IMPLEMENTED` | Euclidean 2D radius threshold check ($R_{\text{fov}} = 40.0\,\text{m}$) evaluated per simulation tick. |
| **2D Detection Geometry** | `IMPLEMENTED` | Planar $\sqrt{(x_u - x_t)^2 + (y_u - y_t)^2}$ distance evaluation against spawned POI positions. |
| **Airborne-Only Detection** | `IMPLEMENTED` | Staged/grounded UAVs have inactive sensors; only airborne UAVs (`rec.is_airborne` or displacement/speed $> \epsilon$) detect. |
| **First-Detection Timestamp** | `IMPLEMENTED` | First valid detection immutable fixes $t_{\text{detect}}$ for the target POI. |
| **Duplicate Suppression** | `IMPLEMENTED` | Subsequent sightings by the same or different UAVs are logged to `sightings_log` without modifying the authoritative report. |
| **Telemetry Lifecycle** | `IMPLEMENTED` | Strict state transitions: `PENDING` $\to$ `DELIVERED` or `DEADLINE_EXCEEDED`. Terminal states are immutable. |
| **Direct Routing** | `IMPLEMENTED` | Packets delivered directly when detecting UAV has a 1-hop link to GCS (`['uav_id', 'GCS']`). |
| **Multihop Routing** | `IMPLEMENTED` | Packets routed over mesh relays via existing Gamma routes (`hop_count = len(route) - 1`). |
| **Delay-Tolerant Buffering** | `IMPLEMENTED` | Packets without an active GCS route remain queued as `PENDING` awaiting network reconnection. |
| **Deadline Handling** | `IMPLEMENTED` | Strict $10.0\,\text{s}$ deadline measured from $t_{\text{detect}}$. Packets exceeding $10.0\,\text{s}$ buffer wait transition to `DEADLINE_EXCEEDED` (`BUFFER_TIMEOUT_NO_ROUTE`). |
| **Trace Events** | `IMPLEMENTED` | Deterministic emission of `POI_DETECTED`, `TELEMETRY_DELIVERED`, and `TELEMETRY_DEADLINE_EXCEEDED` events. |
| **Reporting Metrics** | `IMPLEMENTED` | Total detections, delivered count, deadline exceeded count, compliance ratio, latencies, and per-UAV breakdowns. |
| **Deterministic Replay** | `IMPLEMENTED` | Lexicographical candidate tie-breaking `(task_id, uav_id)` and deterministic iteration orders ensure identical replays. |
| **Altitude Enforcement** | `NOT IMPLEMENTED` | Core simulation is 2D planar. `max_height` is metadata; altitude is not enforced in the Python core. |
| **20m Deconfliction** | `MEASURED` | Inter-UAV separation violations are logged and counted by `SafetyAssessor`; no collision avoidance planner exists in core. |
| **Multi-Wave Fleet Rotation** | `NOT IMPLEMENTED` | Multi-wave relaunch is not implemented. Single-sortie lifecycle is enforced. |
| **Battery Swap / Relaunch** | `NOT IMPLEMENTED` | Relaunch after landing is prohibited in V1 (`RELAUNCH_PROHIBITED`). |
| **Detection-to-Reporting Requirement** | `ASSUMPTION` | $10.0\,\text{s}$ reporting deadline and $40.0\,\text{m}$ sensor radius are project assumptions derived from challenge problem analysis. |

---

## 3. Simulation Architecture & Execution Order

### Authoritative Tick Sequence

Within [`MissionRunner.step()`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/simulation/runner.py#L258), the simulation tick is executed in the following order:

```
0. Scheduled Events Processing (failures, recoveries, dynamic task arrivals)
1. Pre-Physics Communication Analysis (BaselineCommunicationAnalyzer.analyze)
2. Deterministic Safety Assessment & Preemptive RTH Triggers (Autonomy inputs)
3. Optional Safety Hook (External observer)
4. Autonomy Task Allocation (A0TaskAllocator.plan)
5. Task Service Progress (Domain progression for co-located UAVs)
6. Swarm Physics Stepping (SimulationEngine.step_swarm)
7. RTH Arrival Completion (Canonical CompleteRTHCommand on GCS touchdown)
8. Post-Physics Communication & Telemetry:
   ├── Post-Physics State Snapshot
   ├── Fresh Post-Physics NetworkAnalysis (comm_analyzer.analyze)
   ├── Perception FOV Detection (DetectionManager.step_perception)
   └── Telemetry Routing & Retry (DetectionManager.step_telemetry)
9. Deterministic Safety Assessment After Physics (SafetyAssessor.assess_snapshot)
10. Authoritative Clock Advance (sim_engine.advance_tick)
11. State Snapshot Collection & History Recording
```

### Rationale for Dual Communication Analysis

- **Pre-Physics Network Analysis (Step 1)**: Evaluates network connectivity based on UAV coordinates at the start of the tick. This topology informs autonomy decisions (e.g., whether a UAV has active GCS reachability before accepting tasks) and pre-step safety triggers.
- **Post-Physics Network Analysis (Step 8)**: During physics execution (Step 6), UAVs displace up to $v_{\text{max}} \cdot \Delta t$ ($5.0\,\text{m/s} \cdot 1.0\,\text{s} = 5.0\,\text{m}$). Evaluating telemetry packet forwarding against the pre-physics topology would evaluate radio connectivity over stale geometric coordinates. Executing a fresh, read-only Gamma communication analysis immediately following physics ensures that packet delivery and routing decisions reflect the true physical state of the swarm at the moment of perception and transmission.

---

## 4. Test Evidence & Regression Suite

Testing was conducted using `pytest` within the project virtual environment (`.venv/bin/pytest`).

### Telemetry Unit & Integration Suite (`tests/telemetry/test_detection_reporting.py`)
- **Result**: **15 passed** in 1.66s.
- **Categories Covered**:
  - `test_a_fov_entry_detection`: UAV within $40\,\text{m}$ sensor FOV while airborne triggers `POI_DETECTED` and initializes `TelemetryReport`.
  - `test_b_outside_fov_no_detection`: UAV outside $40\,\text{m}$ sensor FOV produces no detection.
  - `test_c_independent_of_task_assignment_or_service`: Detection succeeds for unassigned POIs and POIs assigned to other UAVs.
  - `test_d_single_hop_delivery_to_gcs`: Direct link to GCS delivers packet immediately with `hop_count = 1`.
  - `test_e_multi_hop_relay_delivery_to_gcs`: Multi-hop mesh route correctly records route path and hop count.
  - `test_f_delivery_within_deadline`: Delivery occurring at latency $\le 10.0\,\text{s}$ marked `DELIVERED`.
  - `test_g_no_route_buffering`: Absence of route maintains packet in `PENDING` state inside the buffer.
  - `test_h_reconnect_before_deadline`: UAV reconnecting to GCS at $t \le t_{\text{detect}} + 10\,\text{s}$ transitions to `DELIVERED`.
  - `test_i_buffer_timeout_deadline_exceeded`: Buffered packet without route past $10.0\,\text{s}$ transitions to `DEADLINE_EXCEEDED` (`BUFFER_TIMEOUT_NO_ROUTE`).
  - `test_j_duplicate_sighting_suppression`: Subsequent sightings by same or secondary UAVs do not overwrite authoritative report or reset $t_{\text{detect}}$.
  - `test_k_deterministic_tie_breaking_simultaneous`: Simultaneous detections in identical tick resolve deterministically via lexicographical `(task_id, uav_id)` sort.
  - `test_l_repeated_run_determinism`: Independent executions of identical scenario configuration produce bitwise-identical telemetry results.
  - `test_m_e1_invariance`: Canonical E1 benchmark (`scenarios/poc_round1.yaml`) produces identical completion and safety metrics when detection is disabled.
  - `test_n_zero_detections_metrics_edge_case`: Scenarios with zero detections return valid neutral metrics without division-by-zero errors.
  - `test_o_terminal_state_protection`: Reports in terminal states `DELIVERED` and `DEADLINE_EXCEEDED` are immutable and cannot be overwritten.

### Repository Regression Suite
- **Total Tests**: **247 passed** in 6.15s.
- **Regressions**: 0 failures, 0 errors.

---

## 5. Seed 2026 Integration Evidence

Integration validation was executed against the deterministic scenario [`results/random/random_seed_2026.yaml`](file:///home/dell/swarm_ws/AetherSwarm/results/random/random_seed_2026.yaml) with `challenge_profile.detection_pipeline.enabled = true`.

> [!NOTE]
> **Context Disclaimer**:
> This run represents an **experimental baseline scenario evaluation** to measure the telemetry pipeline under current planner constraints. It is **NOT organizer-compliance evidence**.

### Observed Telemetry Metrics

| Metric | Observed Value | Rationale / Explanation |
| :--- | :--- | :--- |
| **Total POI Detections** | `10` | All 10 POIs in the scenario were successfully detected by airborne UAVs upon entering the $40.0\,\text{m}$ FOV. |
| **Reports Delivered $\le 10\,\text{s}$** | `0` | No UAV maintained or reached an active communication route to GCS within $10.0\,\text{s}$ of detection. |
| **Reports Deadline Exceeded** | `10` | All 10 packets remained buffered beyond the $10.0\,\text{s}$ deadline and transitioned to `DEADLINE_EXCEEDED`. |
| **Reporting Compliance Ratio** | `0.0000` | $0 / 10$ reports delivered within deadline. |
| **Mean Reporting Latency** | `None` | No packets successfully delivered within deadline. |
| **Max Reporting Latency** | `None` | No packets successfully delivered within deadline. |
| **Packets Initially Buffered** | `10` | 100% of generated reports entered the delay-tolerant buffer. |
| **Buffer Deadlines Expired** | `10` | All packets timed out due to absence of an active radio link to GCS. |

### Per-UAV Detection Breakdown
- `uav_5`: 3 detections (`poi_04`, `poi_05`, `poi_07`)
- `uav_2`: 3 detections (`poi_01`, `poi_03`, `poi_09`)
- `uav_4`: 3 detections (`poi_06`, `poi_08`, `poi_10`)
- `uav_3`: 1 detection (`poi_02`)
- `uav_1`: 0 detections

---

## 6. Representative Trace Examples

The following events from the seed 2026 execution illustrate the lifecycle of a detected POI under disconnected mesh conditions:

### 1. Perception Detection Event (`POI_DETECTED`)
```json
{
  "simulation_tick": 254,
  "simulation_time": 254.0,
  "event_type": "POI_DETECTED",
  "entity_id": "uav_5",
  "payload": {
    "task_id": "poi_04",
    "uav_id": "uav_5",
    "t_detect": 254.0,
    "position": [491.45, 756.69],
    "distance": 38.367
  }
}
```

### 2. Telemetry Timeout Event (`TELEMETRY_DEADLINE_EXCEEDED`)
```json
{
  "simulation_tick": 265,
  "simulation_time": 265.0,
  "event_type": "TELEMETRY_DEADLINE_EXCEEDED",
  "entity_id": "uav_5",
  "payload": {
    "task_id": "poi_04",
    "uav_id": "uav_5",
    "t_detect": 254.0,
    "elapsed_s": 11.0,
    "status": "DEADLINE_EXCEEDED",
    "reason": "BUFFER_TIMEOUT_NO_ROUTE"
  }
}
```

---

## 7. Determinism & Repeatability

Two independent, back-to-back executions of the seed 2026 scenario with detection enabled were compared:
- **Telemetry Metrics**: Bitwise identical (`telem1 == telem2`).
- **Authoritative Reports**: All 10 report records, including timestamps and statuses, matched identically.
- **Trace Event Stream**: All 20 emitted perception/telemetry domain events matched in tick, event type, entity ID, sequence number, and payload.

*Note: Determinism has been verified for the evaluated deterministic pseudorandom scenarios; this verification does not imply guarantees beyond tested seeds.*

---

## 8. Limitations & Remaining Stage 1 Gaps

The implementation of Detection → GCS Reporting V1 addresses the measurement and telemetry delivery pipeline. The following operational gaps remain for subsequent milestones:

1. **Planar 2D Core**: The Python simulation operates strictly in 2D $(x, y)$. Airspace altitudes (`max_height: 100.0m`) are metadata and not enforced in core physics or kinematics.
2. **Absence of Motion Deconfliction Planner**: There is no active collision avoidance or velocity deconfliction algorithm. Separation violations ($< 20\,\text{m}$) are detected and counted as safety violations by `SafetyAssessor`, but not actively prevented.
3. **Absence of Dynamic Relay Autonomy**: The $0.0000$ reporting compliance on seed 2026 is directly attributable to the absence of dedicated relay positioning or comms-aware path planning in the current A0 autonomy layer. UAVs disperse across the $1000\,\text{m} \times 1000\,\text{m}$ arena without maintaining a connected chain back to GCS ($R_{\text{comm}} = 100.0\,\text{m}$).
4. **Single-Sortie Constraint**: Ground battery exchange and multi-wave relaunch are not implemented.

---

## 9. Reproducibility Instructions

The test and benchmark evaluations recorded in this document can be reproduced using the following commands:

### Run Telemetry Test Suite
```bash
.venv/bin/pytest -v tests/telemetry/test_detection_reporting.py
```

### Run Full Repository Test Suite
```bash
.venv/bin/pytest -q
```

### Reproduce Seed 2026 Benchmark Evaluation
```python
import yaml
from pathlib import Path
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import load_scenario

with open("results/random/random_seed_2026.yaml", "r", encoding="utf-8") as f:
    config = yaml.safe_load(f)

config["challenge_profile"]["detection_pipeline"] = {
    "enabled": True,
    "sensor_fov_radius_m": 40.0,
    "reporting_deadline_s": 10.0,
}

runner = MissionRunner(load_scenario(config))
result = runner.run()
metrics = result.to_dict()["telemetry"]
print("Telemetry metrics:", metrics)
```
