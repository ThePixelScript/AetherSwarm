# Architecture Specification: Detection → GCS Reporting Pipeline (V1)

> **Document Type**: ARCHITECTURAL SPECIFICATION & DESIGN DECISION
> **Status**: SPECIFICATION ONLY (PRE-IMPLEMENTATION REVIEW)
> **Target Subsystem**: Perception FOV Detection & Telemetry Dissemination
> **Applicable Profiles**: `challenge_profile.detection_pipeline` (Opt-In; disabled by default)
> **Authoritative Baseline**: Commit `9604830` (`feat(challenge): add UAV-X compliance layer v1`)

---

## 1. Problem Statement & Current Gap

The UAV-X Challenge problem statement mandates:
> *"Detection to reporting to center $\le 10\,\text{s}$."*

In the current authoritative baseline (`9604830`), this operational requirement is **not modeled**:
1. **Perception Gap**: UAVs possess no sensory field-of-view (FOV) representation. Tasks are merely spatial point coordinates spawned in `StateSnapshot.tasks`.
2. **Telemetry Gap**: Electronic data packets carrying target detection information do not exist. There is no telemetry lifecycle or packet routing pipeline distinct from task assignment.
3. **Semantic Confusion Gap**: In legacy scenarios, `deadline_offset: 10.0` was an operational task inspection proxy, conflating physical service arrival with target detection and radio report transmission.

This specification formalizes the **Detection $\to$ GCS Reporting $\le 10\,\text{s}$** subsystem, decoupling target perception, radio telemetry transmission, and physical task servicing into rigorous, deterministic, single-writer state operations.

---

## 2. Decoupling: Detection vs. Task Lifecycle

To prevent semantic conflation, the simulation architecture enforces an absolute distinction across four separate domain concepts:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ 1. TASK EXISTENCE (Spawn)                                                   │
│    POI coordinates exist in the arena: t_sim >= task.created_time           │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ 2. SENSOR DETECTION (Perception Event)                                      │
│    Airborne UAV enters sensor FOV: ||p_uav - p_poi|| <= R_fov               │
│    Timestamp: t_detect                                                      │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ 3. TELEMETRY REPORT TO GCS (Electronic Packet Transmission)                 │
│    Discovered POI payload routes via multi-hop mesh graph to GCS            │
│    Timestamp: t_gcs_received                                                │
│    Primary Constraint: reporting_latency_s = (t_gcs_received - t_detect)    │
│    Compliance: reporting_latency_s <= 10.0 s                                │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ 4. TASK SERVICE (Physical Actuation & Inspection)                           │
│    Assigned UAV flies to POI and lingers for service_duration:              │
│    TASK_ASSIGNED -> TASK_STARTED -> TASK_PROGRESS -> TASK_COMPLETED         │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Inviolable Invariants
1. $\text{DETECTION} \neq \text{TASK ASSIGNMENT}$: A UAV may detect and report a POI without being assigned to service it.
2. $\text{DETECTION} \neq \text{TASK START}$: Detecting a target does not initiate physical inspection lingering.
3. $\text{DETECTION} \neq \text{TASK COMPLETION}$: Task service duration and completion time are physical actuation metrics; reporting latency is an electronic communication metric.
4. **Pre-Service Reporting**: In BVLOS swarm operations, a scout UAV routinely detects and reports a target minutes before a dedicated service or payload delivery UAV arrives to service it.

---

## 3. Sensor Model Specification

### 3.1 Parameter Representation (Project Simulation Assumption)
* The sensor FOV radius $R_{\text{fov}} = 40.0\,\text{m}$ is **NOT an organizer-confirmed requirement**.
* In accordance with `docs/CHALLENGE_ASSUMPTIONS_V1.md`, $R_{\text{fov}}$ is modeled as an **explicitly configurable project simulation assumption**:
  ```yaml
  challenge_profile:
    detection_pipeline:
      enabled: true               # Explicitly opt-in; false by default
      sensor_fov_radius_m: 40.0   # Project Simulation Assumption V1 (Configurable)
      processing_delay_s: 0.0     # Delay between detection and packet queuing
      reporting_deadline_s: 10.0  # Challenge constraint
  ```

### 3.2 Detection Eligibility & Trigger Semantics
* **Representation**: Planar Euclidean disc centered at UAV ground track coordinates:
  $$\mathcal{D}_i(t) = \left\{ \mathbf{x} \in \mathbb{R}^2 \;\middle|\; \|\mathbf{x} - \mathbf{p}_{\text{uav}, i}(t)\| \le R_{\text{fov}} + \epsilon \right\}$$
* **UAV Airborne Precondition**: A UAV is eligible to perform sensor detection if and only if it is active and airborne (`is_airborne == True`). Staged, stationary-on-pad, landed, or inactive UAVs have inactive sensors and cannot trigger detections.
* **POI Existence Precondition**: A POI is eligible for detection if and only if it has already spawned into the arena at the current simulation time ($t_{\text{sim}} \ge \text{task.created\_time}$). Future, unspawned POIs cannot be detected.
* **Instantaneous Geometric Trigger**: Detection occurs on the exact simulation tick where an airborne UAV's position brings an existing, active POI position $\mathbf{p}_{\text{poi}}$ inside $\mathcal{D}_i(t)$.
* **Multi-Target Detection**: If a UAV's FOV encompasses multiple existing POIs in the same tick, all intersecting eligible POIs are detected.
* **Deterministic Tie-Breaking**: When multiple POIs are detected within the same tick (by the same or different UAVs), detection records are processed in deterministic lexicographical order by `(task_id, uav_id)`.

---

## 4. Duplicate Suppression & First-Detection Rule

1. **Authoritative First-Detection Rule**:
   * The **first** eligible airborne UAV that brings an existing POI within its sensor FOV generates the unique, authoritative detection event and telemetry report for that POI.
   * The authoritative timestamp $t_{\text{detect}}$ is permanently fixed to this initial sighting.
2. **Exactly One Authoritative Telemetry Report**:
   * In V1, exactly one authoritative `TelemetryReport` is generated per POI.
   * Subsequent sensor crossings by the same UAV or other UAVs do **not** create duplicate telemetry records and do **not** reset the 10-second delivery timer.
   * Later crossings may optionally be logged as non-authoritative diagnostic sightings (`DUPLICATE_SIGHTING`) without affecting the primary reporting lifecycle.
3. **Simultaneous Multi-UAV First Sighting**:
   * If two or more airborne UAVs simultaneously bring a previously undetected POI into their respective FOVs at the exact same tick $t$, the UAV with the lexicographically smallest `uav_id` is designated as the primary detecting agent.

---

## 5. Telemetry Packet Model & Lifecycle

### 5.1 Immutable Telemetry Record Schema
Every detected POI is represented by an immutable telemetry data structure:

```python
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

class TelemetryStatus(str, Enum):
    PENDING = "PENDING"                       # Report queued in swarm buffer; awaiting delivery to GCS
    DELIVERED = "DELIVERED"                   # Successfully received at GCS within deadline (<= 10.0s)
    DEADLINE_EXCEEDED = "DEADLINE_EXCEEDED"   # Reached GCS after > 10.0s, or 10.0s expired while disconnected

@dataclass(frozen=True)
class TelemetryReport:
    task_id: str                              # Unique POI identifier
    detecting_uav_id: str                     # Authoritative initial discovering UAV ID
    t_detect: float                           # Simulation time of initial sensor intersection
    t_report_generated: float                 # Simulation time packet entered transmission queue (t_detect + tau)
    t_gcs_received: Optional[float] = None    # Simulation time packet arrived at GCS node
    hop_count: Optional[int] = None           # Number of RF hops traversed to GCS
    route: Optional[Tuple[str, ...]] = None   # Complete node route sequence, e.g. ("uav_2", "uav_1", "gcs")
    reporting_latency_s: Optional[float] = None  # Exact latency: t_gcs_received - t_detect
    status: TelemetryStatus = TelemetryStatus.PENDING
```

### 5.2 Telemetry State Transitions
```
                ┌──────────────────────────────────────┐
                │          [ POI Detected ]            │
                │ t_report_generated = t_detect + tau  │
                └──────────────────┬───────────────────┘
                                   │
                                   ▼
                ┌──────────────────────────────────────┐
                │          STATUS: PENDING             │
                │   (Queued in Swarm Telemetry Buffer) │
                └───────┬──────────────────────┬───────┘
                        │                      │
           Route Found  │                      │ Age > 10.0 s
      Age <= 10.0 s     │                      │ (Disconnected / Stalled)
                        ▼                      ▼
    ┌─────────────────────────┐    ┌─────────────────────────┐
    │    STATUS: DELIVERED    │    │ STATUS: DEADLINE_EXCEED │
    │ t_gcs_recv = t_sim      │    │ Failed to report <= 10s │
    │ latency = t_gcs - t_det │    │ Retained in Audit Log   │
    └─────────────────────────┘    └─────────────────────────┘
```

### 5.3 Deadline Semantics
* The binding challenge metric is:
  $$\text{reporting\_latency\_s} = t_{\text{gcs\_received}} - t_{\text{detect}}$$
* **Compliant Delivery**: $\text{reporting\_latency\_s} \le 10.0\,\text{s} \implies \text{status} = \text{DELIVERED}$.
* **Deadline Violation**: $\text{reporting\_latency\_s} > 10.0\,\text{s} \implies \text{status} = \text{DEADLINE\_EXCEEDED}$.

---

## 6. Communication Subsystem Integration

### 6.1 Reuse of Existing Gamma Infrastructure
* **No Second Network**: The telemetry pipeline **MUST NOT** construct an independent or ad-hoc communication network.
* It directly consumes the topology, graph, and routes computed by `BaselineCommunicationAnalyzer` in `NetworkAnalysis`:
  - `net_analysis.routes_to_gcs`: Authoritative mapping `uav_id -> Tuple[str, ...] | None`.
  - `net_analysis.connected_uav_ids`: Set of UAV IDs with active simple paths to `"gcs"`.
  - `net_analysis.hop_counts`: Minimal hop count from each node to `"gcs"`.

### 6.2 Delay-Tolerant Buffering & No-Route Handling
1. **Immediate Delivery Case**:
   * At tick $t_{\text{sim}}$, UAV $i$ detects POI $k$.
   * Telemetry pipeline queries `net_analysis.routes_to_gcs.get(i)`.
   * If a route exists (`route is not None`):
     - The report routes across the active path `(uav_i, ..., gcs)`.
     - `hop_count = len(route) - 1`.
     - `t_gcs_received = t_sim`.
     - `reporting_latency_s = t_gcs_received - t_detect`.
     - If `reporting_latency_s <= 10.0`: `status = TelemetryStatus.DELIVERED`.
     - Else: `status = TelemetryStatus.DEADLINE_EXCEEDED`.

2. **Disconnected / Partitioned Buffer Case**:
   * If `net_analysis.routes_to_gcs.get(i) is None`:
     - The detecting UAV has no RF path to GCS.
     - The report remains buffered in the active telemetry store with `status = TelemetryStatus.PENDING`.
   * **No Silent Deletions**: An unrouted packet is **never discarded or deleted**. It remains queued until:
     - **Reconnection within Deadline**: If a route to GCS becomes available at $t_{\text{reconnect}}$ where $t_{\text{reconnect}} - t_{\text{detect}} \le 10.0\,\text{s}$, the packet is delivered: `status = TelemetryStatus.DELIVERED`, `t_gcs_received = t_{\text{reconnect}}`.
     - **Deadline Expiration**: If $t_{\text{sim}} - t_{\text{detect}} > 10.0\,\text{s}$ while still pending, the status transitions immediately to `TelemetryStatus.DEADLINE_EXCEEDED`.
     - The failed record is permanently retained in the metrics report for 100% audit visibility.

---

## 7. Tick Ordering & Dual NetworkAnalysis Specification

### 7.1 The Kinematic-Topological Inconsistency Problem
In the baseline `MissionRunner.step()`, communication analysis occurs at the beginning of the tick:
$$\text{Snapshot}(t) \longrightarrow \text{Gamma Analysis} \longrightarrow \text{Autonomy} \longrightarrow \text{Physics} \longrightarrow \text{Snapshot}(t + \Delta t)$$

If sensor detection occurs **post-physics** (evaluating positions at $t + \Delta t$), telemetry routing **MUST NOT** reuse the pre-physics `NetworkAnalysis`.
* In one tick, UAVs move up to $v_{\max} \cdot \Delta t = 5.0\,\text{m}$.
* A UAV might move into or out of communication range ($100.0\,\text{m}$) during that step.
* Evaluating telemetry delivery using pre-physics routes would evaluate radio links across stale coordinates where the drones no longer reside.

### 7.2 The Dual-Analysis Architecture
To maintain absolute mathematical and physical consistency without compromising autonomy or safety, `MissionRunner.step()` executes two distinct read-only network analyses:

```
MissionRunner.step() Sequence:
─────────────────────────────────────────────────────────────────────────────
1.  Apply Scheduled Events (SimEngine: spawn tasks, external triggers)
2.  Pre-Physics Gamma Communication Analysis:
    - Analyzes current_snap (positions at start of tick).
    - Produces pre_physics_net_analysis.
3.  Deterministic Safety Assessment & Preemptive RTH triggers (uses pre-physics analysis).
4.  Autonomy Allocation (A0/A1 allocates tasks using pre-physics analysis).
5.  Task Service Lingering Progress (Advances progress for collocated UAVs).
6.  Physics Swarm Step (SimEngine: integrates velocities to new positions).
─────────────────────────────────────────────────────────────────────────────
7.  Post-Physics Snapshot & Post-Physics Gamma Communication Analysis:
    - Captures post_physics_snap with updated coordinates.
    - Runs comm_analyzer.analyze(post_physics_snap) -> post_physics_net_analysis.
8.  Sensor Perception & Detection Stage:
    - Evaluates eligible airborne UAVs against eligible spawned POIs.
    - Emits EventType.POI_DETECTED for newly intersecting pairs.
    - Generates immutable PENDING TelemetryReport records.
9.  Telemetry Dissemination & Routing Stage:
    - Queries post_physics_net_analysis.routes_to_gcs for all PENDING reports.
    - Routes packets for connected nodes -> STATUS: DELIVERED.
    - Evaluates deadlines for unrouted reports -> STATUS: DEADLINE_EXCEEDED if >10s.
    - Emits TELEMETRY_DELIVERED or TELEMETRY_DEADLINE_EXCEEDED events.
─────────────────────────────────────────────────────────────────────────────
10. RTH Touchdown Completion (CompleteRTHCommand for UAVs reaching GCS pad).
11. Post-Physics Safety Assessor (Airspace containment, geofence, separation).
12. Clock Advance & History Capture (Advance tick, store StepResult).
```

### 7.3 Why Two Analyses Per Tick Are Acceptable & Deterministic
1. **Mathematical Purity**: `BaselineCommunicationAnalyzer.analyze(snapshot)` is a **pure, side-effect-free function**. Given identical snapshot coordinates, it produces identical, immutable `NetworkAnalysis` outputs. Running it twice on two distinct, immutable state snapshots (`pre_physics_snap` and `post_physics_snap`) maintains 100% mathematical determinism.
2. **Physical Soundness**:
   - Autonomy and safety planning operate on the state known at decision time (pre-physics).
   - Sensor detection and radio propagation operate on the physical reality resulting from movement (post-physics).
3. **Negligible Computational Overhead**:
   - For a 5-UAV swarm + 1 GCS node ($|V| = 6$), constructing the NetworkX graph and running Dijkstra shortest path requires $< 0.1\,\text{ms}$.
   - Two passes per tick add $< 0.2\,\text{ms}$ total, which is completely imperceptible across a 2700-tick mission while guaranteeing total physical correctness.

---

## 8. Authoritative Trace & Webots Visualization Boundaries

### 8.1 Strict Authority Boundary
* **Python Core is Authoritative**: All perception detections, telemetry lifecycles, and deadline evaluations occur strictly within the Python simulation engine.
* **Webots is Downstream and Observational**: Webots **MUST NOT** independently compute detections or report delivery. It merely visualizes the authoritative events serialized into the trace JSON.

### 8.2 Trace Schema Extensions
1. **Domain Events Emitted**:
   * `EventType.POI_DETECTED`: `{task_id: str, uav_id: str, t_detect: float, position: Tuple[float, float]}`
   * `EventType.TELEMETRY_DELIVERED`: `{task_id: str, uav_id: str, t_detect: float, t_gcs: float, latency_s: float, hop_count: int, route: list[str]}`
   * `EventType.TELEMETRY_DEADLINE_EXCEEDED`: `{task_id: str, uav_id: str, t_detect: float, elapsed_s: float, last_known_hop: int}`

2. **Metadata Block**:
   ```json
   "metadata": {
     "detection_pipeline_enabled": true,
     "sensor_fov_radius_m": 40.0,
     "reporting_deadline_s": 10.0
   }
   ```

3. **Per-Tick Trace Block**:
   ```json
   "telemetry": {
     "active_pending_count": 0,
     "reports": [
       {
         "task_id": "poi_04",
         "detecting_uav_id": "uav_5",
         "t_detect": 156.0,
         "t_gcs_received": 156.0,
         "latency_s": 0.0,
         "hop_count": 2,
         "status": "DELIVERED"
       }
     ]
   }
   ```

---

## 9. Official Evaluation Metrics

The detection and reporting pipeline adds six dedicated metrics to `MissionMetricsReport`:

| Metric Field | Type | Description |
|---|:---:|---|
| `total_detections` | `int` | Total count of unique POIs detected by airborne UAVs during mission |
| `reports_delivered` | `int` | Count of detection reports received at GCS within deadline ($\le 10.0\,\text{s}$) |
| `reports_deadline_exceeded` | `int` | Count of reports that arrived late ($>10.0\,\text{s}$) or expired in buffer |
| `reporting_compliance_ratio` | `float` | $\frac{\text{reports\_delivered}}{\text{total\_detections}}$ (Strict ratio; $1.0$ if all delivered $\le 10\,\text{s}$) |
| `mean_reporting_latency_s` | `float \| None` | Mean latency across all delivered reports ($t_{\text{gcs}} - t_{\text{detect}}$) |
| `max_reporting_latency_s` | `float \| None` | Maximum observed latency among delivered reports |

### Independence from Existing Metrics:
* **Distinct from Task Completion**: A mission can achieve $100\%$ task completion but have $0\%$ reporting compliance if detection reports were partitioned.
* **Distinct from Network Availability**: Network availability measures all-node route existence over time; reporting compliance measures deadline-bounded message dissemination.

---

## 10. Inviolability of Baseline E1

* The detection and reporting pipeline is **strictly opt-in**:
  `challenge_profile.detection_pipeline.enabled` defaults to `False`.
* When disabled, the post-physics communication analysis and telemetry stages are bypassed entirely.
* **Golden Baseline Invariance**: Benchmark E1 (`scenarios/poc_round1.yaml`) executes with the pipeline disabled, guaranteeing 100% exact numerical and behavioral invariance ($10/10$ tasks, $429\,\text{s}$, $0$ violations, 4/4 passing tests).

---

## 11. Automated Test Plan (Tests A through K)

Before merging any production implementation, the following 11 automated unit and integration tests must pass:

| Test ID | Test Case Name | Target Behavior to Prove |
|---|---|---|
| **Test A** | `test_detection_on_fov_entry` | Existing POI entering sensor range $R_{\text{fov}}$ of airborne UAV immediately generates `POI_DETECTED` and pending report. |
| **Test B** | `test_detection_distinct_from_service` | POI detected at $t=10\,\text{s}$ remains in `PENDING` service status until assigned UAV arrives. |
| **Test C** | `test_connected_single_hop_report` | UAV in direct link to GCS delivers report at $t_{\text{gcs}} = t_{\text{detect}}$ with $\text{latency} = 0.0\,\text{s}$ and $\text{hops} = 1$. |
| **Test D** | `test_connected_multi_hop_report` | UAV routing via 2 relay hops delivers report with $\text{hops} = 3$ and route sequence verified. |
| **Test E** | `test_reporting_latency_within_deadline` | Report delivered within $10.0\,\text{s}$ is marked `DELIVERED`. |
| **Test F** | `test_disconnected_report_buffering` | Disconnected detecting UAV buffers report in `PENDING` status without packet loss. |
| **Test G** | `test_reconnect_before_deadline_delivers` | UAV reconnects at $t_{\text{detect}} + 6.0\,\text{s} \le 10.0\,\text{s} \to$ marked `DELIVERED`. |
| **Test H** | `test_reconnect_after_deadline_exceeds` | UAV reconnects at $t_{\text{detect}} + 12.0\,\text{s} > 10.0\,\text{s} \to$ marked `DEADLINE_EXCEEDED`. |
| **Test I** | `test_duplicate_detection_suppressed` | Subsequent entry into FOV of second UAV does not overwrite original $t_{\text{detect}}$ or duplicate packet. Exactly one report per POI. |
| **Test J** | `test_deterministic_reproducibility` | Replay of identical seed produces bit-exact identical detection timestamps, routes, and latencies. |
| **Test K** | `test_legacy_e1_invariance` | With `detection_pipeline.enabled = false`, E1 produces identical 4/4 passing tests and zero regressions. |

---

## 12. Affected Files for Subsequent Implementation Pass

| Subsystem Component | Target File | Nature of Expected Modification |
|---|---|---|
| **Configuration** | `src/ares_swarm/simulation/scenario.py` | Add `DetectionPipelineConfig` to `ChallengeProfileConfig` |
| **Domain Enums** | `src/ares_swarm/core/enums.py` | Add `TelemetryStatus`, `EventType.POI_DETECTED`, `EventType.TELEMETRY_DELIVERED`, `EventType.TELEMETRY_DEADLINE_EXCEEDED` |
| **Telemetry Models** | `src/ares_swarm/core/models.py` | Add `TelemetryReport` dataclass |
| **Perception & Telemetry Engine** | `src/ares_swarm/telemetry/manager.py` | New module: FOV intersection, buffering, routing integration, and deadline checks |
| **Simulation Loop** | `src/ares_swarm/simulation/runner.py` | Integrate post-physics network analysis, detection, and telemetry dissemination |
| **Metrics Pipeline** | `src/ares_swarm/evaluation/metrics.py` | Add detection and telemetry latency metrics to `MissionMetricsReport` |
| **Test Suite** | `tests/telemetry/test_detection_reporting.py` | New test suite implementing Tests A through K |

---

## 13. Explicit Non-Goals

* **No Sensor Noise / Probabilistic Detection**: Sensor detection is deterministic geometric intersection.
* **No Motion Deconfliction**: Trajectory deconfliction and separation planning belong to the deconfliction milestone.
* **No Multi-Wave Rotation**: Autonomous battery swap/relaunch belongs to the fleet rotation milestone.
* **No Webots Simulation Logic**: Webots remains downstream, receiving authoritative replay events via trace JSON.

---

## 14. Implementation Gate

> [!CAUTION]
> **MANDATORY IMPLEMENTATION GATE**:
> No production implementation begins until the event semantics, telemetry lifecycle, communication integration, deadline handling, and deterministic test plan are internally consistent.
