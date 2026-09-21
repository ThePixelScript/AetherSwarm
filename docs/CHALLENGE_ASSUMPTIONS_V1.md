# Challenge Simulation Assumptions: V1 Specification

> **Status**: FROZEN PROJECT ASSUMPTIONS (V1)
> **Authority Level**: INTERNAL PROJECT CONVENTION (NOT ORGANIZER-CONFIRMED)
> **Parent Contract**: [`docs/CHALLENGE_MISSION_CONTRACT.md`](file:///home/dell/swarm_ws/AetherSwarm/docs/CHALLENGE_MISSION_CONTRACT.md)
> **Protected Baseline**: `8446fed` / Benchmark E1 (`scenarios/poc_round1.yaml`)
> **Scope**: Challenge Scenario Generation, Simulation Auditing, and Stage 1 / M2 Compliance Profiling

---

## 1. Context & Purpose

In the absence of direct, formal written clarifications from the UAV-X competition organizers regarding semantic edge cases, this document establishes **Version 1 (V1) Project Simulation Assumptions**.

The purpose of this specification is to:
1. Convert the four unresolved organizer semantic ambiguities identified in [`docs/CHALLENGE_MISSION_CONTRACT.md`](file:///home/dell/swarm_ws/AetherSwarm/docs/CHALLENGE_MISSION_CONTRACT.md) into explicit, conservative, and mathematically rigorous engineering conventions.
2. Unblock simulator auditing and subsequent autonomy development without silently inventing rules or presenting internal assumptions as organizer requirements.
3. Enforce the principle that **mission infeasibility or constraint violation is an authoritative experimental finding**, not a defect to be masked by relaxing rules.

---

## 2. Four Frozen V1 Project Assumptions

### 1. 20-Minute Flight Time & Sortie Envelope
*Organizer Evidence*: *"UAV maximum flight time = 20 min = 1200 s"*. (Silent on cumulative lifetime vs. single sortie).

> [!IMPORTANT]
> **PROJECT SIMULATION ASSUMPTION (V1)**:
> 1. **Primary Sortie Limit**: `max_sortie_duration_s = 1200.0`. Each continuous airborne sortie from takeoff to touchdown is strictly capped at $1200.0\,\text{s}$.
> 2. **Dual-Clock Tracking**: The simulation engine records both:
>    - `current_sortie_duration_s = t_sim - t_takeoff` (active flight timer; reset to 0 upon touchdown).
>    - `cumulative_airborne_time_s = sum(sortie_durations)` (mission-wide airborne total per UAV ID).
> 3. **Approaching Flight Limit (Forced Safety RTH)**:
>    When remaining sortie time reaches the required transit time back to the operational center plus a safety buffer, the UAV must enter forced `RTH_FLIGHT_LIMIT`:
>    $$\text{remaining\_sortie\_s} = 1200.0 - (t_{\text{sim}} - t_{\text{takeoff}})$$
>    $$t_{\text{rth\_required}} = \frac{\|\mathbf{p}_{\text{uav}} - \mathbf{p}_{\text{gcs}}\|}{v_{\text{speed}}} + \text{margin}_{\text{safety}} \quad (\text{margin}_{\text{safety}} = 15.0\,\text{s})$$
>    If $\text{remaining\_sortie\_s} \le t_{\text{rth\_required}}$, the UAV aborts its active task and initiates return to GCS.
> 4. **Single-Sortie Policy / Multi-Wave Rotation Distinction in V1**: Multi-wave fleet rotation is **NOT IMPLEMENTED**. Single-sortie relaunch is **PROHIBITED and ENFORCED in V1**: once an airborne UAV lands at the operational center, its flight duty for the mission is complete. Any subsequent motion or secondary sortie attempt generates a `RELAUNCH_PROHIBITED` safety violation.

---

### 2. 20-Meter Separation Metric
*Organizer Evidence*: *"Minimum distance between UAVs = 20m"*. Rule 8 sets max altitude to $100\,\text{m}$. (Silent on 2D horizontal vs. 3D Euclidean distance).

> [!IMPORTANT]
> **PROJECT SIMULATION ASSUMPTION (V1)**:
> 1. **Primary Safety Metric**: **2D Planar Horizontal Separation** ($\Delta r_{\text{2D}}$) is the primary binding safety constraint:
>    $$\Delta r_{\text{2D}} = \sqrt{(x_1 - x_2)^2 + (y_1 - y_2)^2} \ge 20.0\,\text{m}$$
>    Any pair of airborne active UAVs with $\Delta r_{\text{2D}} < 20.0 - \epsilon$ incurs an immediate `SEPARATION` safety violation in the assessor report.
> 2. **Secondary Diagnostic Metric**: **3D Euclidean Distance** ($\Delta r_{\text{3D}}$) is logged concurrently for diagnostic and spatial analysis:
>    $$\Delta r_{\text{3D}} = \sqrt{(x_1 - x_2)^2 + (y_1 - y_2)^2 + (z_1 - z_2)^2}$$
> 3. **Anti-Masking Rule**: Vertical separation ($\Delta z \ge 20\,\text{m}$) shall **NEVER** be used to suppress or clear a 2D horizontal separation violation in V1 compliance reports. Both values must be retained in all generated experiment logs.

---

### 3. Detection $\to$ Reporting $\le 10\,\text{s}$ Event Pipeline
*Organizer Evidence*: *"Detection to reporting to center <= 10s"*. (Silent on boundary event definitions).

> [!IMPORTANT]
> **PROJECT SIMULATION ASSUMPTION (V1)**:
> 1. **Discrete Boundary Timestamps**:
>    - $t_{\text{detect}}$: Simulation timestamp at which an active UAV's sensor field-of-view intersects an active POI.
>    - $t_{\text{report\_generated}}$: Simulation timestamp at which the telemetry payload is assembled at the discovering UAV ($t_{\text{detect}} + \tau_{\text{proc}}$, default $\tau_{\text{proc}} = 0.0\,\text{s}$).
>    - $t_{\text{gcs\_received}}$: Simulation timestamp at which the telemetry packet successfully reaches the Ground Control Station via direct link or multi-hop routing.
> 2. **Binding Latency Metric**:
>    $$\text{reporting\_latency\_s} = t_{\text{gcs\_received}} - t_{\text{detect}}$$
>    Compliance is achieved if and only if $\text{reporting\_latency\_s} \le 10.0\,\text{s}$.
> 3. **Decoupling from Task Servicing**: Task service completion (physical dwell time / loiter inspection) is explicitly **NOT** a proxy for reporting latency.
> 4. **Partition / Disconnection Handling**:
>    If a POI is detected while the discovering UAV is disconnected from GCS, the packet is queued in local buffer memory (`PENDING_ROUTE`). If $t_{\text{sim}} - t_{\text{detect}} > 10.0\,\text{s}$ before a valid multi-hop route to GCS is established, the event is permanently marked as `REPORTING_DEADLINE_EXCEEDED` (FAIL).
> 5. **Configurable Sensor FOV**:
>    Sensor discovery footprint is modeled as a configurable radial parameter:
>    `sensor_fov_radius_m: 40.0` (Default V1 simulation assumption placeholder; fully configurable).

---

### 4. Battery Swap & Re-Launch Policy
*Organizer Evidence*: Rules specify 5 UAVs, 20-minute flight limit, and 45-minute mission duration. (Silent on pit-stop battery replacement).

> [!IMPORTANT]
> **PROJECT SIMULATION ASSUMPTION (V1)**:
> 1. **Single Bounded Sortie Default**: By default in V1, each UAV has exactly one physical battery and one bounded sortie ($\le 1200\,\text{s}$).
> 2. **Touchdown Terminus**: Landing at the operational center ends the UAV's operational duty for the mission.
> 3. **No Automatic / Instantaneous Re-Launch**: The simulation will not assume zero-second battery hot-swapping.
> 4. **Mission Infeasibility as an Authorized Finding**:
>    Under this conservative assumption, 5 UAVs provide a maximum cumulative flight capacity of $5 \times 1200\,\text{s} = 6000\,\text{drone-seconds}$. Spanning a $2700\,\text{s}$ mission allows at most an average of $2.22$ concurrent UAVs aloft. Consequently, some 45-minute challenge scenarios requiring full-arena coverage or late-spawning POIs ($t > 1500\,\text{s}$) will be **operationally partitioned or unserviceable**. This infeasibility is a legitimate experimental finding that must be documented rather than masked.

---

## 5. Assumption Priority & Traceability Matrix

| Parameter / Semantic | Organizer Evidence | V1 Project Assumption | Confidence | Can Change Later? |
|---|---|---|:---:|:---:|
| **Flight Duration Ceiling** | *"UAV max flight time = 20 min = 1200 s"* | `max_sortie_duration_s = 1200.0` (single sortie limit) | HIGH | Yes (if cumulative cap confirmed) |
| **Airframe Re-launch** | Not mentioned in rules | Single sortie only; no re-launch in V1 default | MEDIUM | Yes (if pit-stop turnaround defined) |
| **Separation Dimension** | *"Min distance between UAVs = 20m"* | 2D horizontal separation primary; 3D logged | HIGH | Yes (if 3D Euclidean approved) |
| **Detection Event** | *"Detection to reporting <= 10s"* | Physical sensor intersection ($R_{\text{fov}} \le 40\,\text{m}$) | HIGH | Yes (if search model specified) |
| **Reporting Receipt** | *"reporting to center"* | Telemetry packet receipt at GCS node | HIGH | Yes (if task service confirmed) |
| **Sensor FOV Radius** | Not specified | `sensor_fov_radius_m = 40.0` (placeholder) | LOW | Yes (easily configured) |
| **GCS Takeoff Exemption** | Center is $75\,\text{m}$ outside arena | Ingress/Egress transit corridor $[-75, 0] \times [450, 550]$ | HIGH | Yes (corridor bounds configurable) |

---

## 6. Non-Negotiable Engineering Principles

1. **Inviolability of Benchmark E1**:
   Under no circumstances shall baseline scenario `poc_round1.yaml`, the frozen commit `8446fed`, or any existing benchmark results be altered to satisfy these assumptions. E1 remains an unyielding regression baseline.
2. **Zero Fabrication & Honest Transparency**:
   Assumptions shall never be reported to stakeholders or external reviewers as "organizer requirements." They must be explicitly cited as `PROJECT SIMULATION ASSUMPTION (V1)`.
3. **Failure Visibility**:
   When a scenario cannot be completed under V1 assumptions (e.g., partitioned network, expired reporting deadline, or flight timeout), the simulation engine must faithfully record the violation. Assumptions must never be relaxed to artificially produce a "clean" run.
4. **Architectural Separation**:
   All new challenge logic must exist within an opt-in `challenge_profile` schema, leaving core default execution identical to legacy behavior when disabled.

---

## 7. Implementation Contract: Proposed Configuration Schema

When implementation begins in future tasks, the following configuration fields may be introduced into the scenario definition under a dedicated namespace. *(Note: These fields are specifications only; not yet implemented in code)*:

```yaml
# Conceptual Scenario Schema Extension (Specification Only)
challenge_profile:
  enabled: false                          # false preserves exact legacy E1 behavior
  enforce_flight_limit: true
  max_sortie_duration_s: 1200.0
  track_cumulative_flight_time: true

  separation_metric: "2D"                 # "2D" (primary) | "3D" (diagnostic)
  min_separation_m: 20.0

  detection_pipeline:
    enabled: true
    sensor_fov_radius_m: 40.0
    reporting_deadline_s: 10.0
    processing_delay_s: 0.0

  fleet_lifecycle:
    allow_relaunch: false                 # V1 default: single sortie per airframe
    allow_battery_swap: false
    ground_turnaround_s: 180.0            # applicable only if allow_relaunch: true

  airspace:
    model: "composite_corridor"
    staging_pad_center: [-75.0, 500.0]
    staging_pad_radius_m: 15.0
    corridor_x_bounds: [-75.0, 0.0]
    corridor_y_bounds: [450.0, 550.0]
    arena_x_bounds: [0.0, 1000.0]
    arena_y_bounds: [0.0, 1000.0]
```

---

## 8. Final Decision

Implementation may begin against **CHALLENGE_ASSUMPTIONS_V1**, but all four unresolved items remain explicitly project assumptions and must never be described as organizer-confirmed facts.
