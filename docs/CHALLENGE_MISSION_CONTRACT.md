# Challenge Mission Contract: UAV-X Round 1 Semantics

> **Status**: DRAFT / SPECIFICATION ONLY
> **Baseline**: `8446fed` (FROZEN)
> **Target Scenarios**: Randomized UAV-X Round 1 Challenge Scenarios
> **Protected Benchmark**: E1 PoC Baseline (`scenarios/poc_round1.yaml`)

---

## 1. Purpose

This mission contract establishes the formal specification translating the organizer-provided UAV-X Round 1 problem statement into rigorous, unambiguous simulation semantics.

Prior testing revealed that several operational constraints (such as flight endurance, real-time reporting latency, geofence corridors, and multi-UAV separation) were either not modeled, modeled using operational proxies, or ambiguously defined. To prevent premature, ad-hoc, or non-deterministic modifications to the production simulation engine (`src/ares_swarm/`), this document:
1. Distinguishes organizer-confirmed requirements from unresolved interpretations and simulation assumptions.
2. Formally maps constraints to architectural subsystems.
3. Establishes the mathematical and geometric invariants required for compliance.
4. Preserves the frozen baseline `8446fed` and benchmark `E1` via strict backward-compatibility rules.

---

## 2. Organizer-Confirmed Constraints

The following 14 constraints are directly derived from the official organizer sample scenario (`scenarios/poc_round1.yaml` header and UAV-X problem statement). No text has been silently reinterpreted:

1. **Operational Area**: $1000\,\text{m} \times 1000\,\text{m}$ planar rectangular area ($x \in [0.0, 1000.0]$, $y \in [0.0, 1000.0]$).
2. **Operational Center (GCS)**: Located $75\,\text{m}$ outside the operational area ($[-75.0, 500.0]$).
3. **Mission Duration**: Total operational window = $45\,\text{min} = 2700.0\,\text{s}$.
4. **UAV Maximum Flight Time**: $20\,\text{min} = 1200.0\,\text{s}$ per UAV.
5. **Maximum Communication Range**: $100.0\,\text{m}$ cutoff for inter-UAV and UAV-to-GCS RF links.
6. **Takeoff Location**: All UAVs must take off from the operational center.
7. **Landing Location & Deadline**: All UAVs must return and land at the starting area before or by mission end ($t \le 2700.0\,\text{s}$).
8. **Maximum Altitude**: $100.0\,\text{m}$ above ground level.
9. **Maximum Horizontal Speed**: $5.0\,\text{m/s}$.
10. **Minimum Inter-UAV Separation**: $20.0\,\text{m}$ minimum distance between any active pair during flight.
11. **Detection to Reporting to Center**: Time elapsed from target detection to telemetry report receipt at the operational center must be $\le 10.0\,\text{s}$.
12. **Target Quantity**: Exactly 10 Points of Interest (POIs).
13. **Target Spatial Distribution**: POI spatial coordinates are randomly distributed within the operational area.
14. **Target Temporal Distribution**: POI appearance/spawn timestamps are randomly distributed across the mission.

---

## 3. Semantic Ambiguities & Critical Classifications

### A. "20-Minute Maximum Flight Time"
- **Organizer Evidence**: Rule specifies *"UAV maximum flight time = 20 min = 1200 s"*.
- **Unspecified**: Does this represent:
  1. *Maximum continuous sortie duration* from takeoff to landing (allowing battery swap/re-launch)?
  2. *Cumulative total airborne time* for a physical airframe ID across the entire 45-minute competition window?
  3. *Battery energy proxy* ($4200\,\text{Wh}$ capacity at nominal discharge rate)?
- **Classification**: **UNRESOLVED**
- **Simulation Assumption (Working)**: Bounded continuous sortie duration $\le 1200\,\text{s}$ per takeoff-to-touchdown cycle. Both continuous sortie time and cumulative mission airborne time must be tracked independently.

### B. "20-Meter Separation Metric": 2D Planar vs. 3D Euclidean
- **Organizer Evidence**: Rule specifies *"Minimum distance between UAVs = 20m"*. Rule 8 allows altitudes up to $100\,\text{m}$.
- **Unspecified**: Whether distance is evaluated horizontally in the 2D plane ($\sqrt{\Delta x^2 + \Delta y^2} \ge 20\,\text{m}$) or as true 3D Euclidean separation ($\sqrt{\Delta x^2 + \Delta y^2 + \Delta z^2} \ge 20\,\text{m}$).
- **Classification**: **UNRESOLVED**
- **Simulation Assumption (Working)**: Current assessor strictly evaluates 2D horizontal separation. If 3D separation is confirmed by organizers, altitude stratification ($\Delta z \ge 20\,\text{m}$) would unconditionally solve spatial deconfliction even on crossing horizontal ground tracks.

### C. "Detection to Reporting to Center $\le 10\,\text{s}$" Event Chain
- **Organizer Evidence**: Rule specifies *"Detection to reporting to center <= 10s"*.
- **Unspecified**: The exact boundary events defining the timer start and end:
  - Timer Start: Instant POI spawns in simulation? Instant POI enters an airborne UAV's sensor cone? Instant UAV creates the digital payload?
  - Timer End: Instant packet arrives at GCS radio interface? Instant task is fully serviced on-site?
- **Classification**: **UNRESOLVED**
- **Simulation Assumption (Working)**: Timer starts at $t_{\text{detect}}$ (when an airborne UAV reaches Euclidean distance $\le R_{\text{sensor}}$ to an active POI) and stops at $t_{\text{gcs\_recv}}$ (when the telemetry packet successfully routes over the active multi-hop graph to GCS). Task physical inspection is explicitly decoupled.

### D. Battery Swap / Recharge / Relaunch Operations
- **Organizer Evidence**: Rule specifies 5 UAVs, 20-minute flight time, and 45-minute mission duration.
- **Unspecified**: Whether landed UAVs at the operational center may undergo battery swap/recharging and execute subsequent sorties, or whether each UAV is single-sortie only (which caps total fleet airborne seconds at $5 \times 1200 = 6000\,\text{s}$, permitting only $\approx 2.22$ concurrent UAVs aloft).
- **Classification**: **UNRESOLVED**
- **Simulation Assumption (Working)**: Landed UAVs require an explicit ground turnaround delay $T_{\text{turnaround}} \ge 180.0\,\text{s}$ before being eligible for a secondary sortie.
- **V1 Baseline Resolution**: Multi-wave fleet rotation is **NOT IMPLEMENTED**. Single-sortie relaunch is **PROHIBITED and ENFORCED in V1** (touchdown terminates operational duty; any relaunch attempt triggers `RELAUNCH_PROHIBITED`).

### E. Continuous Swarm Connectivity vs. Opportunistic Reporting
- **Organizer Evidence**: Rule specifies communication range $= 100\,\text{m}$.
- **Unspecified**: Whether the swarm must maintain unbroken global graph connectivity to GCS at all times, or if disconnected single-UAV sorties are permitted provided any detected POI can be reported to GCS within $10.0\,\text{s}$ of discovery.
- **Classification**: **SIMULATION ASSUMPTION**
- **Working Assumption**: AetherSwarm prioritizes connected mesh topologies (A1 policy), but unserviced POIs in disconnected regions trigger explicit unreachable/partitioned states rather than simulator crashes.

---

## 4. Current System Mapping

| Organizer Requirement | Implementation Status | Code Location | Technical Evidence & Discrepancy |
|---|:---:|---|---|
| **1. Operational Area ($1000\times 1000\,\text{m}$)** | **Implemented** | [`src/ares_swarm/safety/safety_assessor.py`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/safety/safety_assessor.py) | Checked in `_check_geofence`: flags violation if $x \notin [0, 1000]$ or $y \notin [0, 1000]$. |
| **2. Operational Center ($75\,\text{m}$ outside)** | **Partially Implemented** | [`scenarios/poc_round1.yaml`](file:///home/dell/swarm_ws/AetherSwarm/scenarios/poc_round1.yaml), [`src/ares_swarm/safety/safety_assessor.py`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/safety/safety_assessor.py) | Baseline set GCS at $[-50.0, 500.0]$ ($50\,\text{m}$ offset). Random generator uses $[-75.0, 500.0]$, but core assessor triggers geofence alarms outside $x < 0$. |
| **3. Mission Duration ($45\,\text{min} = 2700\,\text{s}$)** | **Implemented** | [`src/ares_swarm/simulation/runner.py`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/simulation/runner.py) | Hard termination at `duration: 2700.0`, tick limit 2700 at $\Delta t = 1.0\,\text{s}$. |
| **4. UAV Max Flight Time ($20\,\text{min} = 1200\,\text{s}$)** | **Modeled Differently** | [`src/ares_swarm/energy/battery.py`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/energy/battery.py), [`src/ares_swarm/safety/safety_assessor.py`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/safety/safety_assessor.py) | Modeled purely as battery energy ($4200\,\text{Wh}$ at $3.5\,\text{W/s}$ motion burn). Wall-clock airborne flight duration is **not** tracked or bounded. |
| **5. Max Comm Range ($100\,\text{m}$)** | **Implemented** | [`src/ares_swarm/communication/channel.py`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/communication/channel.py) | Strict Euclidean cutoff: pairs with distance $> 100.0\,\text{m}$ return disconnected. |
| **6. Takeoff from Operational Center** | **Not Implemented** | [`scenarios/poc_round1.yaml`](file:///home/dell/swarm_ws/AetherSwarm/scenarios/poc_round1.yaml) | Baseline UAVs pre-staged inside arena at $x = +50.0\,\text{m}$. Core assessor lacks transit corridor to support staging at $x = -75.0\,\text{m}$. |
| **7. Land at Start by $45\,\text{min}$** | **Implemented** | [`src/ares_swarm/safety/safety_assessor.py`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/safety/safety_assessor.py) | Automatic RTH evaluates return travel time and forces landing at GCS before $t = 2700.0\,\text{s}$. |
| **8. Maximum Altitude ($100\,\text{m}$)** | **Not Enforced in Core** | [`src/ares_swarm/core/models.py`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/core/models.py) | Engine models 2D planar kinematics with discrete integer `altitude_layer: int = 1`. `max_height` is stored as configuration metadata only and is NOT enforced in the core simulation engine. Webots downstream verification may observe 3D position, but this does not make the core simulation altitude-compliant. |
| **9. Maximum Speed ($5\,\text{m/s}$)** | **Implemented** | [`src/ares_swarm/core/kinematics.py`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/core/kinematics.py) | Clamped to $\min(\text{distance}, v_{\max} \cdot \Delta t)$ with $v_{\max} = 5.0\,\text{m/s}$. |
| **10. Minimum Separation ($20\,\text{m}$)** | **Partially Implemented** | [`src/ares_swarm/safety/safety_assessor.py`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/safety/safety_assessor.py) | Assessor checks pairwise 2D distance. Pass in E1 (parallel lanes); Fail in unconstrained random scenarios (no deconfliction planner). |
| **11. Detection $\to$ Report $\le 10\,\text{s}$** | **Not Implemented** | [`src/ares_swarm/core/models.py`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/core/models.py), [`src/ares_swarm/core/simulator.py`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/core/simulator.py) | Perception discovery and telemetry packet dissemination are not modeled. `deadline_offset: 10.0` was an operational service window proxy. |
| **12. Exactly 10 POIs** | **Implemented** | [`scenarios/poc_round1.yaml`](file:///home/dell/swarm_ws/AetherSwarm/scenarios/poc_round1.yaml), [`scripts/generate_random_scenario.py`](file:///home/dell/swarm_ws/AetherSwarm/scripts/generate_random_scenario.py) | Enforced in both baseline configuration and challenge generator. |
| **13. Random POI Positions** | **Implemented (Generator)** | [`scripts/generate_random_scenario.py`](file:///home/dell/swarm_ws/AetherSwarm/scripts/generate_random_scenario.py) | Fixed in E1; fully implemented via PRNG uniform spatial sampling in generator. |
| **14. Random POI Spawn Times** | **Implemented (Generator)** | [`scripts/generate_random_scenario.py`](file:///home/dell/swarm_ws/AetherSwarm/scripts/generate_random_scenario.py) | Fixed in E1; fully implemented via PRNG uniform temporal sampling in generator. |

---

## 5. Airspace Semantics Proposal

To eliminate ad-hoc geofence bypasses while honoring the true physical launch site at $[-75.0, 500.0]$, the mission airspace is formally defined as a composite geometric zone with state-dependent operational semantics.

```
       Zone 1: STAGING PAD           Zone 2: TRANSIT CORRIDOR           Zone 3: OPERATIONAL ARENA
  ┌────────────────────────────┐    ┌────────────────────────────┐    ┌───────────────────────────┐
  │ Center: [-75.0, 500.0]     │    │ X: [-75.0, 0.0]            │    │ X: [0.0, 1000.0]          │
  │ Radius: R_pad = 15.0 m     │───►│ Y: [450.0, 550.0]          │───►│ Y: [0.0, 1000.0]          │
  │ Ground Holding & Service   │    │ Ingress (East) / Egress (W)│    │ Active Task Search & Work │
  └────────────────────────────┘    └────────────────────────────┘    └───────────────────────────┘
```

### Proposed Geometric Zones (Configurable Parameters)
- **Zone 1: Staging Pad (`StagingPad`)**:
  - Center: $\mathbf{p}_{\text{gcs}} = [-75.0, 500.0]$
  - Radius: $R_{\text{pad}} = 15.0\,\text{m}$ (Proposed simulation parameter)
  - Semantics: Ground parking, pre-flight staging, battery turnaround, and touchdown.
- **Zone 2: Transit Corridor (`TransitCorridor`)**:
  - Bounding Box: $x \in [-75.0, 0.0]$, $y \in [450.0, 550.0]$ (Proposed simulation parameter)
  - Semantics: Dedicated two-way transit channel connecting Staging Pad to Operational Arena.
  - Deconfliction Lanes: Outbound Ingress (North Lane: $y \in [510.0, 540.0]$); Inbound Egress (South Lane: $y \in [460.0, 490.0]$).
- **Zone 3: Operational Arena (`OperationalArena`)**:
  - Bounding Box: $x \in [0.0, 1000.0]$, $y \in [0.0, 1000.0]$
  - Ceiling: $z \le 100.0\,\text{m}$
  - Semantics: Primary search, relay, and POI servicing volume.

### State-Dependent Geofence Semantics
A position $\mathbf{p} = (x, y)$ is evaluated against the UAV's operational flight state:
1. `GROUND_STAGED` / `LANDED`: Must satisfy $\|\mathbf{p} - \mathbf{p}_{\text{gcs}}\| \le R_{\text{pad}}$.
2. `INGRESS`: Must be within `TransitCorridor` with trajectory heading toward the arena entrance ($v_x \ge 0$).
3. `MISSION` (Search / Relay / Service): Must be strictly within `OperationalArena` ($x \in [0, 1000], y \in [0, 1000]$). Entering the transit corridor while in active mission mode is flagged as a boundary departure.
4. `EGRESS` / `RTH`: Must be within `OperationalArena` navigating toward corridor entrance, or within `TransitCorridor` with heading toward GCS ($v_x \le 0$).
5. `TOUCHDOWN`: Final descent within $R_{\text{pad}}$ of $\mathbf{p}_{\text{gcs}}$.

---

## 6. Flight-Time Semantics Proposal

To support both possible interpretations of Rule 4 without making premature assumptions, the simulation telemetry contract must record four independent temporal metrics per UAV:

```python
@dataclass
class UAVFlightTelemetry:
    uav_id: str
    takeoff_timestamp: Optional[float] = None
    touchdown_timestamp: Optional[float] = None
    current_sortie_duration_s: float = 0.0
    cumulative_airborne_time_s: float = 0.0
    sortie_count: int = 0
    in_flight: bool = False
```

### Measurement vs. Decision Boundary
- **What Can Be Measured Immediately**:
  - Exact time elapsed since last takeoff: $t_{\text{airborne}} = t_{\text{sim}} - t_{\text{takeoff}}$.
  - Total cumulative flight time over all completed sorties.
  - Margin to $1200.0\,\text{s}$ limit.
- **What Depends on the Unresolved Rule**:
  - If single-sortie rule: `cumulative_airborne_time_s <= 1200.0` is a hard mission-wide invariant per UAV ID.
  - If multi-sortie rule: `current_sortie_duration_s <= 1200.0` is enforced per flight, with `sortie_count > 1` permitted after a valid ground turnaround period ($T_{\text{ground}} \ge 180\,\text{s}$).

---

## 7. Separation Semantics

### Measurement Layer vs. Planning Layer
The separation contract must separate the **Safety Assessor** (which measures ground truth) from the **Motion Planner** (which generates collision-free paths).

```
[ Motion Planner / Autonomy ] ──► [ Kinematic Stepping ] ──► [ Safety Assessor ]
(Generates deconflicted paths)    (Updates positions)         (Independent ground truth audit)
```

### Evaluation Findings
- **Observed Baseline E1**: Minimum separation $= 23.02\,\text{m}$ (0 violations across 2700 ticks due to fixed $40\,\text{m}$ parallel lanes).
- **Observed Random Scenarios**: Minimum separation $= 0.98\,\text{m}$ to $6.80\,\text{m}$ (7 to 27 violations per seed due to crossing straight-line paths).
- **Metric Dimensionality**: Assessor currently evaluates 2D distance. If organizer requires 2D, crossing trajectories on the same plane must yield or divert laterally. If 3D is confirmed, discrete altitude flight levels resolve crossing paths unconditionally.

### Why Simple Speed Yielding Fails
Simple velocity throttling (stopping when CPA $< 22\,\text{m}$) exhibits critical failure modes:
1. **Intersection Stop**: A yielding UAV stopping on an intersecting vector may halt directly inside the passing UAV's $20\,\text{m}$ bubble.
2. **Head-On Collinear Deadlock**: Two UAVs facing each other cannot resolve conflict by stopping alone.
3. **Relay Partition**: A yielding UAV that stops may break an active multi-hop communication link to downstream nodes.

### Candidate Future Approaches (Subject to Evaluation)
- **Altitude Stratification**: Dedicated flight levels ($z \in \{30, 50, 70, 90\}\,\text{m}$) if 3D metric is confirmed.
- **Dual-Lane Transit Corridors**: Directionally segregated flight paths in high-traffic choke points.
- **Deterministic Setback Yielding**: Speed reduction combined with minimum stopping distance setback $\ge 25.0\,\text{m}$ from projected intersection points.

---

## 8. Detection $\to$ Reporting Semantics

### The Formal Event Chain
Detection and reporting represent an electronic telemetry pipeline completely distinct from physical task inspection.

```
1. [ POI Spawns ] (Ground truth active in arena; undiscovered by swarm)
         │
         ▼
2. [ UAV Sensor Intersect ] (dist(p_uav, p_poi) <= R_sensor)
         │  ──► Event: DISCOVERY (t_detect = current_time)
         ▼
3. [ Telemetry Report Formed ] (Payload assembled: poi_id, coords, t_detect)
         │  ──► Event: REPORT_GENERATED (t_report = current_time + tau_proc)
         ▼
4. [ RF Mesh Routing ] (Shortest path evaluated across active channel graph)
         │
         ├── Path Available ──► Packet advances over N hops (tau_hop = 0.01s)
         │                      ──► Event: GCS_RECEIVED (t_gcs = t_report + N * tau_hop)
         │                      ──► Latency: Δt = t_gcs - t_detect
         │                      ──► Status: (Δt <= 10.0s) ? COMPLIANT : EXCEEDED
         │
         └── Path Broken    ──► Packet buffered at UAV. Retry next tick.
                                If current_time - t_detect > 10.0s:
                                ──► Status: TIMEOUT_VIOLATION
```

### Candidate Telemetry State Schema
```python
@dataclass(frozen=True)
class POITelemetryRecord:
    poi_id: str
    detecting_uav_id: str
    t_spawn: float
    t_detect: float
    t_report_generated: float
    t_gcs_received: Optional[float]
    hop_count: int
    reporting_latency_s: Optional[float]
    status: str  # "PENDING" | "DELIVERED" | "TIMEOUT_FAILED"
```

*Crucial Distinction*: Physical task completion requires flying to the POI and hovering for `service_duration` ($2.0\,\text{s}$ to $30.0\,\text{s}$), which can take hundreds of seconds of flight time. Reporting is purely electronic packet delivery.

---

## 9. Connectivity Reality Check

### Hard Geometric Limits
- **Arena Area**: $1000\,\text{m} \times 1000\,\text{m} = 1.0\,\text{km}^2$.
- **GCS Position**: $[-75.0, 500.0]$.
- **Maximum Radio Range ($R_{\text{comm}}$)**: $100.0\,\text{m}$.
- **Fleet Size**: 5 UAVs.

```
Maximum Possible Linear Span = 5 UAVs * 100 m = 500 m
Distance from GCS to Arena Midpoint (500, 500) = 575 m
Distance from GCS to Far Arena Corner (1000, 1000) = 1185.6 m
```

### Rigorous Architectural Distinctions
1. **Continuous GCS Connectivity**: **IMPOSSIBLE across entire arena**. 5 UAVs cannot bridge distances $> 500\,\text{m}$. Any POI spawning at $x > 425\,\text{m}$ cannot maintain a simultaneous unbroken RF link to GCS even if all 5 UAVs act solely as stationary relays.
2. **Task Reachability**: **FEASIBLE**. Individual UAVs can fly to any coordinate $(x, y) \in [0, 1000] \times [0, 1000]$ at $5\,\text{m/s}$ within battery limits.
3. **$\le 10\,\text{s}$ Reporting Feasibility**: **PARTITION-CONSTRAINED**. A detected POI can only be reported within $10.0\,\text{s}$ if the discovering UAV is already connected to GCS at detection time or can establish an RF route within 10 seconds. In uncoordinated random distributions, reporting will fail for distant POIs unless mobile relay positioning is explicitly scheduled.

---

## 10. Architectural Impact & Layer Boundaries

```
┌───────────────────────────────────────────────────────────────────────────┐
│                    AETHERSWARM SUBSYSTEM ARCHITECTURE                    │
├───────────────────────────────────────────────────────────────────────────┤
│ 1. COMPLIANCE & MEASUREMENT LAYER (src/ares_swarm/safety/)                 │
│    - Ground-truth evaluation of 14 organizer constraints                  │
│    - Multi-zone geofence auditing (Staging Pad, Corridor, Arena)          │
│    - Independent flight timer (takeoff-to-touchdown duration)             │
│    - Pairwise separation tracking (min distance & violation logging)      │
│    - Telemetry delivery deadline logging (t_gcs - t_detect <= 10s)        │
├───────────────────────────────────────────────────────────────────────────┤
│ 2. AUTONOMY & MOTION LAYER (src/ares_swarm/autonomy/, /planning/)         │
│    - Fleet sortie scheduler & staged wave rotation                        │
│    - State-dependent flight regimes (Ingress, Search, Relay, Egress)       │
│    - Deterministic trajectory deconfliction & lane adherence              │
│    - Destination-aware task allocation (A1 policy)                        │
├───────────────────────────────────────────────────────────────────────────┤
│ 3. COMMUNICATION LAYER (src/ares_swarm/communication/)                    │
│    - Dynamic RF topology graph generation (100m threshold)                │
│    - Multi-hop shortest path telemetry routing                            │
│    - Telemetry packet queuing, forwarding, and hop latency modeling       │
├───────────────────────────────────────────────────────────────────────────┤
│ 4. DOWNSTREAM VISUALIZATION & VERIFICATION (visualization/webots/)        │
│    - Non-authoritative observational replay of simulation traces          │
│    - Independent 3D spatial verification log                              │
│    - High-fidelity presentation of airframes, beacons, and corridors      │
└───────────────────────────────────────────────────────────────────────────┘
```

**Architectural Rule**: The simulation engine in `src/ares_swarm/` remains the sole authoritative source of truth. Webots is strictly a downstream viewer and shall never execute autonomy or state updates.

---

## 11. E1 Backward-Compatibility Contract

1. **Frozen Baseline**: Baseline `8446fed` and the E1 benchmark scenario ([`scenarios/poc_round1.yaml`](file:///home/dell/swarm_ws/AetherSwarm/scenarios/poc_round1.yaml)) are permanently protected.
2. **Opt-In Scenario Configuration**: All new capabilities (corridor geofencing, flight duration limits, sensor detection events) must be gated by explicit scenario configuration flags:
   ```yaml
   # Default values preserve exact E1 behavior
   enforce_flight_time_limit: false    # true only for challenge scenarios
   require_sensor_discovery: false     # true only for challenge scenarios
   airspace_model: "simple_box"        # "composite_corridor" for challenge
   ```
3. **Test Suite Invariance**: The test suite (232 tests total, preserving all 219 legacy baseline tests without alteration) must continue to pass with zero regressions.
4. **Benchmark Purity**: No existing benchmark metrics ($10/10$ tasks, $429\,\text{s}$ duration, $100\%$ connectivity) shall be modified to satisfy challenge constraints.

---

## 12. Open Questions Matrix

| # | Question | Organizer Evidence | Current Working Interpretation | Needed Clarification | Blocks Which Feature? |
|---|---|---|---|---|---|
| **1** | **20-Minute Flight Limit Scope** | Rule 4: *"UAV maximum flight time = 20 min = 1200 s"* | Maximum single continuous sortie duration $\le 1200\,\text{s}$. | Is cumulative mission flight time capped at $1200\,\text{s}$ per UAV ID? | Fleet duty cycle scheduler; fleet size sizing. |
| **2** | **2D vs. 3D Separation Metric** | Rule 10: *"Minimum distance between UAVs = 20m"*; Rule 8: *"Max altitude = 100m"* | Planar horizontal separation ($\Delta r_{2D} \ge 20\,\text{m}$). | Does vertical altitude separation ($\Delta z \ge 20\,\text{m}$) satisfy Rule 10? | Trajectory deconfliction planner design. |
| **3** | **Exact Detection $\to$ Reporting Event Chain** | Rule 11: *"Detection to reporting to center <= 10s"* | Sensor discovery ($R_{\text{fov}} \le 40\,\text{m}$) to GCS RF packet arrival $\le 10\,\text{s}$. | Does reporting imply physical verification or pure telemetry packet delivery? | Perception subsystem and network telemetry queue. |
| **4** | **Battery Swap / Relaunch Permission** | Rules 3 & 4 ($45\,\text{min}$ mission vs. $20\,\text{min}$ flight time; 5 UAVs) | Landed UAVs can undergo battery swap ($T_{\text{ground}} \ge 180\,\text{s}$) and relaunch. | Are airframes limited to a single sortie, or are multi-sortie turnarounds allowed? | Mission fleet rotation feasibility over 45 minutes. |

---

## 13. Implementation Gate

> [!CAUTION]
> **MANDATORY ENGINEERING GATE**:
> No production code modifications in `src/ares_swarm/` or `tests/` shall begin until:
> 1. The four critical semantic questions above are formally clarified by organizers **OR** explicitly frozen as documented project simulation assumptions.
> 2. A formal Phase 7E Implementation Plan is reviewed and approved against this contract.
> 3. Strict backward-compatibility guarantees for benchmark E1 are verified.
