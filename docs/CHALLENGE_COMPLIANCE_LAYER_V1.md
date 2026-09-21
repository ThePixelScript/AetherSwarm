# Challenge Compliance Layer V1: Architecture, Hardening & Verification

> **Status**: IMPLEMENTED, HARDENED & VERIFIED
> **Authority Level**: INTERNAL PROJECT COMPLIANCE LAYER (PROJECT SIMULATION ASSUMPTIONS V1)
> **Parent Documents**: [`docs/CHALLENGE_MISSION_CONTRACT.md`](file:///home/dell/swarm_ws/AetherSwarm/docs/CHALLENGE_MISSION_CONTRACT.md), [`docs/CHALLENGE_ASSUMPTIONS_V1.md`](file:///home/dell/swarm_ws/AetherSwarm/docs/CHALLENGE_ASSUMPTIONS_V1.md)
> **Protected Baseline**: `8446fed` / Benchmark E1 (`scenarios/poc_round1.yaml`)

---

## 1. Architectural Purpose

The **Challenge Compliance Layer V1** introduces formal airspace boundary definitions, sortie duration compliance monitoring, and single-sortie lifecycle tracking to the AetherSwarm simulation engine without altering the frozen baseline or the legacy benchmark E1.

All challenge-specific behaviors are strictly opt-in via the `challenge_profile` configuration block. When disabled (default), the simulator executes identical legacy logic, ensuring 100% backward compatibility.

> [!CAUTION]
> **Strict Compliance Grounding**:
> 1. All parameters implemented herein are **Project Simulation Assumptions (V1)**, not organizer-confirmed requirements.
> 2. **Altitude**: The core Python simulation (`src/ares_swarm/`) is **2D planar**. The parameter `max_height: 100.0m` is configuration metadata only and is **NOT evaluated or enforced in the core simulation**. (Altitude is verified downstream in the 3D Webots supervisor only).
> 3. **Single Sortie**: Ground battery swapping and re-launch are prohibited in V1. Relaunch attempts generate `RELAUNCH_PROHIBITED` violations.

---

## 2. Formal Challenge Airspace Model

The airspace is modeled as a composite geometry with state-dependent operational flight phase rules via [`ChallengeAirspace`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/safety/airspace.py).

### Chosen V1 Default Airspace Geometry (Project Simulation Parameters)
- **Zone 1: Staging Pad**:
  - Center: $\mathbf{p}_{\text{gcs}} = [-75.0, 500.0]$
  - Radius: $R_{\text{pad}} = 15.0\,\text{m}$
  - Role: Ground parking, takeoff, and touchdown.
- **Zone 2: Transit Corridor**:
  - Longitudinal Bounds: $x \in [-75.0, 0.0]$
  - Lateral Bounds: $y \in [400.0, 600.0]$ (Width: $200.0\,\text{m}$, accommodating $40\,\text{m}$ initial vehicle spacing)
  - Role: Dedicated two-way transit channel connecting Staging Pad to the Operational Arena.
- **Zone 3: Operational Arena**:
  - Spatial Bounds: $x \in [0.0, 1000.0]$, $y \in [0.0, 1000.0]$
  - Altitude Metadata: `max_height: 100.0m` (Metadata only; not enforced in planar core).
  - Role: Active search, relay, and task execution volume.

### State-Dependent Flight Phase Rules
A UAV position $\mathbf{p} = (x, y)$ is evaluated against its current [`FlightPhase`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/safety/airspace.py):
1. `STAGING`: Position must reside within $R_{\text{pad}} \le 15.0\,\text{m}$ of $\mathbf{p}_{\text{gcs}}$.
2. `INGRESS`: Position must reside within `TransitCorridor`, `StagingPad`, or the entry edge of `OperationalArena`.
3. `MISSION`: Position must be strictly inside `OperationalArena` ($[0, 1000] \times [0, 1000]$). Departures back into the corridor or arena exterior are flagged as boundary departures.
4. `EGRESS`: Position must reside in `OperationalArena` or `TransitCorridor` navigating toward $\mathbf{p}_{\text{gcs}}$.
5. `LANDED`: Final touchdown must be located within $R_{\text{pad}} \le 15.0\,\text{m}$ of $\mathbf{p}_{\text{gcs}}$. Landings outside the pad trigger a `LANDING_LOCATION` safety violation.

---

## 3. 20-Minute Sortie Compliance Mechanics

Implemented in accordance with V1 Project Assumptions:
- `max_sortie_duration_s = 1200.0`
- `rth_safety_margin_s = 15.0`
- `enforce_single_sortie = true`

### Sortie Tracking & Timing Invariants
[`SafetyAssessor`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/safety/safety_assessor.py) tracks per-UAV flight records:
- **Takeoff Detection (Hardened)**:
  - A staged UAV assigned a task or role but with zero velocity (`speed <= EPSILON` and `dist_gcs <= pad_radius`) is **STILL ON THE GROUND** and **NOT AIRBORNE**.
  - Takeoff is recorded when actual physical motion begins (`speed > EPSILON`) or when physically leaving the staging pad (`dist_gcs > pad_radius`).
- **Landing Detection**:
  - Recorded upon actual touchdown via `CompleteRTHCommand` (`EventType.UAV_LANDED` / `u.rth_state == RTHState.COMPLETE`).
- **Single-Sortie Policy**:
  - Touchdown terminates the UAV's operational duty for the mission.
  - Any subsequent movement or secondary sortie attempt generates a `RELAUNCH_PROHIBITED` safety violation.
- **Duration Metrics**:
  - `current_sortie_duration_s = landing_time - takeoff_time`.
  - `cumulative_airborne_s`: Total mission airborne time.
  - `max_observed_sortie_duration_s`: High-water mark recorded in [`SafetyReport`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/safety/safety_assessor.py).

### Dynamic RTH Trigger Calculation
Rather than an arbitrary static timer, the RTH trigger is derived dynamically on every tick:
$$\text{remaining\_sortie\_s} = \max(0.0, \text{max\_sortie\_duration\_s} - \text{current\_sortie\_duration\_s})$$
$$\text{transit\_time\_s} = \frac{\|\mathbf{p}_{\text{uav}} - \mathbf{p}_{\text{gcs}}\|}{\max(1.0, v_{\text{speed\_limit}})}$$
$$\text{required\_rth\_time\_s} = \text{transit\_time\_s} + \text{rth\_safety\_margin\_s}$$

When `remaining_sortie_s <= required_rth_time_s`, a `StartRTHCommand` is issued to ensure touchdown before the $1200.0\,\text{s}$ limit expires. If a UAV fails to return in time, a `FLIGHT_DURATION` violation is logged.

---

## 4. Backward Compatibility Contract

- **Default Scenario Config**: `challenge_profile: { enabled: false }`.
- **E1 Invariance**: In `scenarios/poc_round1.yaml`, `challenge_profile` is omitted. `MissionRunner` defaults to legacy geofence checks and battery RTH.
- **Opt-In Verification**: Challenge capabilities activate only when `challenge_profile.enabled: true`.

---

## 5. Test Coverage Summary

Implemented in [`tests/safety/test_challenge_compliance_v1.py`](file:///home/dell/swarm_ws/AetherSwarm/tests/safety/test_challenge_compliance_v1.py):
- **Test A**: Staging position accepted ($[-75.0, 500.0]$).
- **Test B**: Ingress corridor accepted ($[-30.0, 500.0]$).
- **Test C**: Mission arena accepted ($[500.0, 500.0]$).
- **Test D**: Illegal position rejected ($[-75.0, 200.0]$ outside corridor; $[-30.0, 500.0]$ while on mission).
- **Test E**: RTH corridor accepted during egress.
- **Test F**: Landing outside staging region rejected (`LANDING_LOCATION` violation).
- **Test G**: Accurate 1200s sortie tracking across takeoff, in-flight, and landing.
- **Test H**: Over-duration detection (`FLIGHT_DURATION` violation).
- **Test I**: Preemptive RTH triggered dynamically before sortie limit.
- **Test J**: Frozen E1 benchmark maintains 100% exact behavior (10/10 tasks, 0 violations, 429s).
- **Test K**: Staged assigned-but-not-moving UAV is confirmed NOT airborne (requires physical motion).
- **Test L**: Single-sortie policy confirmed: secondary sortie after landing generates `RELAUNCH_PROHIBITED` violation.
- **Test M**: End-to-end integration: full flight loop verifies actual movement, dynamic RTH, and touchdown safely $\le 1200.0\,\text{s}$ with exact duration match.

---

## 6. Verification Status & Capability Disclosure

- **Total Test Suite**: **232 passed** (219 legacy baseline tests + 13 hardened challenge tests).
- **E1 Golden Benchmark**: Verified 100% invariant ($10/10$ tasks, $429\,\text{s}$, $0$ violations).
- **Altitude ($100\,\text{m}$)**: **NOT ENFORCED IN CORE**. The authoritative Python core is planar 2D. `max_height=100.0` is configuration metadata only. Downstream Webots verification may observe 3D position, but this does not make the core simulation altitude-compliant.
- **Separation ($\ge 20\,\text{m}$)**: **MEASURED ONLY (FAIL in Seed 2026)**. Evaluated in 2D planar space. In Seed 2026, uncoordinated straight-line trajectories crossed resulting in 64 violations (min separation $1.43\,\text{m}$). Motion deconfliction planning is not yet implemented.
- **Geofence Compliance (Seed 2026)**: **VIOLATIONS OBSERVED (FAIL in Seed 2026)**. 3 genuine violations occurred at ticks 157–159 when `uav_5` clipped the northern corridor boundary ($y = 600.84\,\text{m}$ to $602.33\,\text{m} > 600.0\,\text{m}$) while outside the arena ($x \in [-8.18, -3.40]$) during unconstrained straight-line ingress toward `poi_04`. Corridor-containment waypoint routing is not yet implemented.
- **Detection $\to$ Reporting ($\le 10\,\text{s}$)**: **NOT IMPLEMENTED / NOT MEASURED**. Sensor discovery FOV intersection and telemetry packet delivery timers are not yet modeled in the codebase.
- **Multi-Wave Fleet Rotation**: **NOT IMPLEMENTED**. Once the initial fleet lands ($t \approx 1374\,\text{s}$), the arena remains vacant for the remainder of the $2700\,\text{s}$ mission.
- **Single-Sortie Relaunch**: **PROHIBITED and ENFORCED in V1** (touchdown terminates vehicle duty; any secondary sortie attempt triggers `RELAUNCH_PROHIBITED`).
- **Reproducible Benchmark**: Generated for Seed 2026 at [`results/challenge_v1/seed_2026_baseline.md`](file:///home/dell/swarm_ws/AetherSwarm/results/challenge_v1/seed_2026_baseline.md).
