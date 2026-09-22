# Connectivity-Aware Mission Planning (Phase 4)

## Overview

Phase 4 introduces a **Connectivity-Aware Mission Planner** (`ConnectivityAwarePlanner`) that gates every task assignment with a communication and endurance feasibility check before issuing an `AssignTaskCommand`. It integrates tightly with the existing `DynamicRelayManager` and telemetry pipeline.

---

## V1 Feasibility Rule

> **A task assignment is connectivity-feasible if and only if there exists a confirmed communication path from the assigned UAV's operating position (the POI) back to the GCS under the following rules:**

### Case 1: Direct Link
- The Euclidean distance from the POI location to the GCS ≤ `comm_range_m × effective_range_factor` (default: `100m × 0.95 = 95m`)
- No relay required.

### Case 2: Existing Active Relay Covers POI
- At least one UAV with `role=RELAY`, `active=True`, `failure_state=NORMAL`, and `rth_state=NONE` currently exists in the network.
- That relay is connected to the GCS (present in `NetworkAnalysis.connected_uav_ids`).
- The relay's current position is within `comm_range_m × effective_range_factor` of the POI.
- No new relay deployment required.

### Case 3: Single Intermediate Relay Deployment
- A relay can be positioned at the **midpoint** between the GCS and POI.
- Both `d(GCS → midpoint)` ≤ effective range AND `d(midpoint → POI)` ≤ effective range.
- This means the **maximum supportable POI distance is `2 × 95m = 190m`** from GCS.
- An eligible candidate UAV must be available (not assigned, not already acting as relay, endurance-sufficient) to be deployed to the midpoint.

### Multi-Hop Limitation
- **V1 does not support multi-hop chains.** Only a single relay intermediate is evaluated.
- POIs beyond `2 × effective_range_m ≈ 190m` from GCS are infeasible under V1 and will be **deferred** (`connectivity_deferred_tasks` incremented).
- The system logs the infeasibility reason: `"POI distance Xm exceeds single-relay coverage (190m)"`.

---

## What Constitutes a Feasible Assignment

An assignment is feasible if **all three** of the following pass:

1. **Sortie Limit Check** (if `enforce_sortie_limit=True`): Current airborne duration + estimated time to complete task + RTH margin ≤ `max_sortie_duration_s` (1200s).
2. **Battery Energy Check**: Available battery energy ≥ energy required for transit + service + return, with a 20% safety margin plus `min_battery_reserve_wh` (15 Wh).
3. **Connectivity Check**: One of Cases 1, 2, or 3 above is satisfied at the POI location.

If any of these checks fails, the candidate UAV is **rejected** for that task (`connectivity_rejected_assignments` incremented per rejected candidate). If **all candidates** for a task are rejected, the task is deferred (`connectivity_deferred_tasks` incremented).

---

## Direct vs. Relayed Connectivity

| Condition | Handling |
|-----------|----------|
| POI ≤ 95m from GCS | Direct link. No relay deployed. |
| POI > 95m, existing relay covers POI | Relay already in position. No new relay dispatched. |
| POI > 95m, no relay in position, distance ≤ 190m | New relay candidate selected via `DynamicRelayManager.select_relay_candidate()`. `AssignRelayRoleCommand` + `SetTargetPositionCommand` emitted. |
| POI > 190m from GCS | Infeasible. Task deferred. |

---

## How Relay Assignment Interacts with Planning

When a relay deployment is required (Case 3):

1. The planner calls `relay_manager.select_relay_candidate()` to find an eligible UAV that can reach the midpoint within endurance limits.
2. If found, two commands are emitted before the `AssignTaskCommand`:
   - `AssignRelayRoleCommand(uav_id=relay_id, target_position=midpoint, relay_for_uav_id=surveyor_id)`
   - `SetTargetPositionCommand(uav_id=relay_id, target_position=midpoint)`
3. The relay UAV is marked as `assigned` in the planning cycle's `assigned_uav_ids` set — it will not be selected as a surveyor for any other task in the same planning tick.
4. The relay mapping is registered immediately: `surveyor_to_relay[surveyor_id] = relay_id`.
5. `relay_required_for_assignment` is incremented once per task that required a relay.

**Important:** The relay UAV must travel to the midpoint position before the link is established. There is no instantaneous link guarantee; connectivity monitoring tracks this via `connected_uav_ids`.

---

## What Happens on Relay Loss / RTH

The `monitor_active_tasks()` method runs every simulation tick (step 3.7 in the mission loop, before allocator planning):

### Relay RTH / Departure
- If the designated relay for a surveyor enters RTH, LANDING, or LANDED state **and** has not been replaced (detected by checking `relay_uav.role != Role.RELAY` after state transition), the relay is marked as `relay_lost_unrecovered = True`.

### Relay Hardware Failure
- If the relay UAV has `failure_state != FailureState.NORMAL` or is `active=False`, it is treated as lost.

### Disconnection Tolerance
- If a surveyor UAV is disconnected from the GCS for ≥ `disconnected_replan_tolerance_s` (default: 10s) **and** is physically beyond `comm_range_m` from the GCS, replanning is triggered.
- This handles cases where the relay failed silently (no explicit state transition detected).

### What Replanning Does
When `trigger_replan` is True:
1. `ReleaseTaskCommand` is issued (task status returns to `DEFERRED`).
2. `SetTargetPositionCommand` sends the surveyor towards the GCS position.
3. `communication_induced_replans` counter is incremented.
4. The UAV's disconnection timer is cleared.

The task re-enters the DEFERRED queue and may be reassigned on a subsequent planning tick if connectivity is restored.

---

## What Causes Communication-Induced Replanning

| Trigger | Condition |
|---------|-----------|
| Relay loss without replacement | Designated relay fails/RTH and no handoff occurred |
| Sustained disconnection | Surveyor disconnected ≥ 10s while beyond comm range |

**Note:** Replanning does **not** trigger if the relay is simply moving to a new position during a handoff — the handoff logic in `DynamicRelayManager` handles continuity.

---

## Metric Definitions

| Metric | Definition |
|--------|------------|
| `connectivity_feasibility_checks` | Total number of (task, UAV) pairs evaluated for feasibility. Incremented once per candidate UAV per task per planning tick. |
| `connectivity_feasible_assignments` | Number of tasks **actually assigned** after passing feasibility. Incremented **once per task** after the best candidate is selected and `AssignTaskCommand` is emitted. Does NOT count every passing candidate. |
| `connectivity_rejected_assignments` | Number of (task, candidate UAV) feasibility checks that **failed**. One per rejected candidate, not one per task. |
| `connectivity_deferred_tasks` | Number of tasks where **all candidates were infeasible** (or no candidates existed) and the task was not assigned in this tick. |
| `relay_required_for_assignment` | Number of task assignments that required deploying a new relay (Case 3). |
| `connectivity_preserved_during_task` | Cumulative seconds during which assigned surveyors were present in `connected_uav_ids` (i.e., linked to GCS). |
| `communication_induced_replans` | Number of active task aborts triggered by connectivity loss or relay failure. |

### Critical Distinction: `feasibility_checks` vs. `feasible_assignments`

- `feasibility_checks` = **probe count** — how many (candidate, task) combinations were evaluated
- `feasible_assignments` = **assignment count** — how many tasks successfully received a UAV assignment

For N candidates evaluated for T tasks, `feasibility_checks ≤ N×T` and `feasible_assignments ≤ T`.

---

## Known Limitations (V1)

1. **Single relay only**: The V1 planner evaluates exactly one relay intermediate. Multi-hop chains (e.g., GCS → relay_A → relay_B → POI) are not supported. POIs beyond 190m from GCS are permanently deferred.

2. **Relay midpoint heuristic**: The relay position is always set to the geometric midpoint between GCS and POI. This is not optimal for all arena geometries and does not account for obstacle avoidance.

3. **No relay pre-positioning**: Relays are dispatched reactively at task assignment time. If the relay needs 20+ seconds to reach its position, there is a connectivity gap before the link is established.

4. **Feasibility uses instantaneous snapshot**: The check uses current UAV positions and battery levels. If the mission state changes between planning ticks (e.g., a candidate's battery is consumed by a previous assignment), the next tick's check may differ.

5. **Single-hop PDR impact**: Each relay hop multiplies the packet delivery probability. Two-hop paths (GCS ↔ relay ↔ UAV) have lower model-estimated PDR than direct single-hop paths, as seen in experimental results.

6. **Seeds produce identical results for fixed-position POIs**: Because POI positions are deterministic (not randomized), all three experiment seeds produce identical metrics. Seed variation only matters for randomized scenario generation.

7. **No relay load balancing**: A single candidate relay is selected greedily. If multiple tasks simultaneously need relays, the same UAV may be selected multiple times (though the `assigned_uav_ids` set prevents double-assignment within a single planning tick).

---

## Experimental Results (3-Seed Comparison)

> Seeds: 2026, 42, 5001 | POIs: 10 | Fleet: 5 UAVs | Mission: 2700s | Sortie: 1200s

### Configuration Difference

| Parameter | Baseline | Connectivity-Aware |
|-----------|----------|-------------------|
| Connectivity planner | Disabled | Enabled |
| Relay manager | Disabled | Enabled |
| Comm range | 100m | 100m |
| All other params | Identical | Identical |

### Results (all seeds produce identical values due to fixed POI positions)

| Metric | Baseline | Connectivity-Aware | Δ |
|--------|----------|-------------------|---|
| Completed POIs | 10/10 | 10/10 | = |
| Completion Rate | 100% | 100% | = |
| Reporting Delivered | 2/10 | **10/10** | **+8** |
| Deadline Exceeded | 8 | **0** | **−8** |
| **Reporting Compliance** | **20%** | **100%** | **+80pp** |
| Connectivity Availability | 55.3% | **93.9%** | +38.6pp |
| PDR (model est.) | 0.9574 | 0.7748 | −0.183 (relay hop cost) |
| Latency ms (model est.) | 5.74 | 7.75 | +2.01ms (relay latency) |
| Relay Assignments | 0 | 5 | +5 |
| Relay Handoffs | 0 | 0 | = |
| Feasibility Checks | 0 | 30 | +30 |
| Feasible Assignments | 0 | 10 | +10 |
| Rejected Assignments | 0 | 0 | = |
| Deferred Tasks | 0 | 0 | = |
| Relay-Required Assignments | 0 | **5** | +5 |
| Conn. Preserved (s) | 0 | 81 | +81s |
| Comm-Induced Replans | 0 | 0 | = |
| Max Sortie Duration (s) | 1185 | 1185 | = |
| Battery Exhaustion | 0 | 0 | = |
| Safety Violations | 0 | 0 | = |
| Geofence Violations | 0 | 0 | = |

### Analysis

**Reporting compliance improved from 20% to 100%.** Without connectivity-aware planning, 8 out of 10 reports failed their 10-second deadline because UAVs operating beyond 95m from GCS had no communication path. The baseline allocator assigns tasks without verifying connectivity, so reports from out-of-range UAVs are buffered and delivered late (or not at all within the deadline).

**5 of 10 tasks required relay deployment.** POIs at x=80 (GCS distance ~152m) and x=120 (~188m) are beyond direct range. The planner correctly identified these and deployed relay UAVs to midpoint positions, enabling the 10-second deadline to be met.

**PDR decrease with connectivity-aware mode** is expected and correct: the model-estimated PDR for a 2-hop path is the product of per-hop PDRs. This is a real tradeoff — relay paths have lower PDR but far higher connectivity availability and deadline compliance.

**All 10 POIs completed in both modes** — completion rate is unchanged because the POIs are well within the fleet's endurance envelope regardless of relay deployment. The connectivity planner does not reduce task throughput in this scenario.

**No deferred tasks** — all POIs are within the 190m single-relay coverage limit, so the planner successfully finds a feasible path for every task.

**Communication-induced replans: 0** — relays remained healthy throughout the mission; no relay failures or RTH during active task service were observed.

> **Note:** The three seeds produce identical metrics because the POI positions are fixed (not seed-randomized). Seed variation would produce different results in randomly-generated scenarios via the `ScenarioGenConfig` pathway.

---

## Architecture Integration

```
MissionRunner.step()
  ├── 3.0  Physics / Movement
  ├── 3.1  Sensors / Detection
  ├── 3.2  Telemetry
  ├── 3.5  Relay Manager (DynamicRelayManager)
  ├── 3.7  Connectivity Planner: monitor_active_tasks()   ← NEW (Phase 4)
  │         └── ReleaseTaskCommand if relay lost / disconnected
  └── 4.0  Autonomy Allocation
        └── If connectivity_planner is set:
              ConnectivityAwarePlanner.plan()             ← NEW (Phase 4)
                ├── check_task_connectivity_feasibility()  per (task, uav)
                ├── AssignRelayRoleCommand + SetTargetPositionCommand  (if relay needed)
                └── AssignTaskCommand  (if feasible candidate found)
```

The planner **replaces** the standard autonomy adapter for task assignment when enabled. It wraps `A1TaskAllocator` internally for utility computation but adds the feasibility gate before any assignment.

---

## Files

| File | Purpose |
|------|---------|
| `src/ares_swarm/autonomy/connectivity_planner.py` | Core planner: feasibility checks, relay deployment, active monitoring |
| `src/ares_swarm/autonomy/__init__.py` | Exports `ConnectivityAwarePlanner` |
| `src/ares_swarm/simulation/runner.py` | Integration: step 3.7 monitor + step 4.0 planning override |
| `src/ares_swarm/simulation/scenario.py` | `enable_connectivity_aware_planning` flag |
| `src/ares_swarm/evaluation/metrics.py` | Phase 4 metric fields on `MissionMetricsReport` |
| `src/ares_swarm/core/enums.py` | `COMMUNICATION_REPLAN` event type |
| `src/ares_swarm/core/state_store.py` | Phase 4 metric propagation |
| `tests/test_connectivity_aware_planning.py` | 9 deterministic unit/integration tests |
| `scripts/phase4_comparison.py` | 3-seed baseline vs. connectivity-aware comparison |
| `docs/phase4_comparison_results.json` | Raw experiment results (JSON) |
