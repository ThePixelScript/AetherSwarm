# Experimental Validation & Empirical Benchmarks

## 1. Overview of Experimental Campaigns

The development of AetherSwarm has been guided by empirical benchmarking across five distinct experimental campaigns:

```
┌────────────────────────────────────────────────────────────────────────┐
│ Phase 1: Fleet Size Feasibility Sweep (N = 1 to 16 UAVs)               │
│ • Quantified cumulative airborne capacity over 45-minute missions     │
│ • Evaluated GCS staging corridor congestion inflection points          │
├────────────────────────────────────────────────────────────────────────┤
│ Phase 2: Sortie Rotation & Deadlock Resolution                         │
│ • Validated 20-min sortie ceiling and 300s recharge replenishment      │
│ • Eliminated multi-UAV arrival freeze at GCS staging pad               │
├────────────────────────────────────────────────────────────────────────┤
│ Phase 4: Single-Relay Connectivity-Aware Validation                    │
│ • Compared controlled fixed-coordinate vs. randomized 3-seed envelopes │
│ • Established the 190m single-relay reach ceiling                      │
├────────────────────────────────────────────────────────────────────────┤
│ Phase 5B: Deterministic Multi-Hop Relay Chain Validation               │
│ • Verified collinear stationing, atomic allocation, and link handoff   │
│ • Extended geometric reach up to 1185.6m across full arena geometry    │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Phase 1: Fleet Size Feasibility Sweep ($N = 1 \dots 16$)

Phase 1 evaluated swarm scalability under the UAV-X operational constraints: a $2700.0\,\text{s}$ (45-minute) mission duration and a $1200.0\,\text{s}$ (20-minute) maximum continuous flight ceiling per vehicle.

### 2.1 Theoretical Flight Capacity Model
For a fleet of $N$ UAVs operating over mission duration $T_{\text{mission}} = 2700\,\text{s}$:
- If single-sortie restriction is enforced (no battery recharging):
  $$\text{Capacity}_{\text{max}} = N \times 1200.0\,\text{drone-seconds}$$
  For $N = 5$:
  $$\text{Capacity}_{\text{max}} = 6,000\,\text{drone-seconds}$$
  Average concurrent airborne aircraft aloft:
  $$\bar{N}_{\text{airborne}} = \frac{6,000\,\text{s}}{2,700\,\text{s}} \approx 2.22 \text{ concurrent UAVs}$$
  *Conclusion*: A 5-UAV fleet with single-sortie limits cannot simultaneously maintain a distant multi-hop relay chain and perform arena-wide search across a 45-minute mission.

### 2.2 Empirical Sweep Results ($N = 1 \dots 16$)
Executed via [`scripts/run_fleet_size_sweep.py`](file:///home/dell/swarm_ws/AetherSwarm/scripts/run_fleet_size_sweep.py):

| Fleet Size ($N$) | Completed Tasks | Serviced POIs (%) | Total Sorties | Safety Violations | Pad Congestion Index |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **1** | 2 / 10 | 20.0% | 1 | 0 | 0.00 |
| **3** | 5 / 10 | 50.0% | 3 | 0 | 0.05 |
| **5** | 8 / 10 | 80.0% | 5 | 0 | 0.12 |
| **8** | 10 / 10 | 100.0% | 8 | 0 | 0.28 |
| **12** | 10 / 10 | 100.0% | 12 | 0 | 0.54 |
| **16** | 10 / 10 | 100.0% | 16 | 0 | 0.82 |

*Key Finding*: Fleet sizes $N \ge 8$ achieve complete task servicing for near-corridor POIs, but as fleet size scales beyond $N = 12$, arrival queuing at the single GCS pad $(-75, 500)$ increases, requiring staggered RTH offsets.

---

## 3. Phase 4: Single-Relay Connectivity Validation

Phase 4 evaluated the single-relay [`ConnectivityAwarePlanner`](file:///home/dell/swarm_ws/AetherSwarm/src/ares_swarm/autonomy/connectivity_planner.py) where effective link range was capped at $R_{\text{eff}} = 95.0\,\text{m}$ (maximum single-relay envelope $D \le 190.0\,\text{m}$).

### 3.1 Controlled vs. Randomized Methodology
In early Phase 4 testing, a methodology flaw was identified: evaluating seeds 2026, 42, and 5001 against `scenarios/poc_round1.yaml` produced identical results because POI positions were hardcoded in that file.

The experiment was corrected by generating true randomized POI distributions via [`scripts/generate_scenario.py`](file:///home/dell/swarm_ws/AetherSwarm/scripts/generate_scenario.py):

```bash
# Corrected validation generation:
python scripts/generate_scenario.py --seed 2026 --num-uavs 8 --num-pois 10 --output scenarios/phase4_seed2026.yaml
python scripts/generate_scenario.py --seed 42   --num-uavs 8 --num-pois 10 --output scenarios/phase4_seed42.yaml
python scripts/generate_scenario.py --seed 5001 --num-uavs 8 --num-pois 10 --output scenarios/phase4_seed5001.yaml
```

### 3.2 Phase 4 Randomized 3-Seed Results

| Metric | Seed 2026 | Seed 42 | Seed 5001 | Swarm Total / Avg |
| :--- | :---: | :---: | :---: | :---: |
| **Nearest POI to GCS** | $161.4\,\text{m}$ | $178.2\,\text{m}$ | $542.4\,\text{m}$ | — |
| **Farthest POI to GCS** | $1042.8\,\text{m}$ | $1112.5\,\text{m}$ | $1168.1\,\text{m}$ | — |
| **Feasible Tasks ($D \le 190\,\text{m}$)** | 2 / 10 | 1 / 10 | 0 / 10 | 3 / 30 (10.0%) |
| **Tasks Assigned & Serviced** | 2 | 1 | 0 | 3 |
| **Tasks Deferred (Out of Range)** | 8 | 9 | 10 | 27 / 30 (90.0%) |
| **Total Physical Detections** | 2 | 1 | 0 | 3 |
| **Reports Delivered to GCS** | 2 | 1 | 0 | 3 |
| **Reports Within 10s Deadline** | 2 (100.0%) | 1 (100.0%) | 0 (N/A) | 3 (100.0%) |
| **Mean Reporting Latency** | $0.0\,\text{s}$ | $0.0\,\text{s}$ | N/A | $0.0\,\text{s}$ |
| **Safety Violations (20m / Geofence)** | 0 | 0 | 0 | 0 |

### 3.3 Critical Findings of Phase 4
1. **100% Reporting Compliance on Assigned Tasks**: Every task assigned within the $190.0\,\text{m}$ envelope achieved immediate, compliant reporting to GCS ($0.0\,\text{s}$ transmission delay across the active relay bridge).
2. **Safe Deferral**: Out-of-envelope tasks were correctly and deterministically deferred (`TASK_DEFERRED`) without sending disconnected scouts into communication blackouts.
3. **The Multi-Hop Imperative**: Across 3 randomized seeds, 90.0% of POIs were situated beyond $190.0\,\text{m}$ (Seed 5001 had zero feasible tasks). This empirical finding proved that multi-hop relay chains are mandatory for full-arena disaster response.

---

## 4. Phase 5B: Deterministic Multi-Hop Implementation Validation

Phase 5B implemented multi-hop relay chains up to 13 hops ($K \le 12$ relays). The implementation was verified across 9 targeted deterministic tests in [`tests/test_multihop_relay_planning.py`](file:///home/dell/swarm_ws/AetherSwarm/tests/test_multihop_relay_planning.py):

| Test Case | Scenario Evaluated | Verification Result |
| :--- | :--- | :---: |
| `test_multihop_chain_geometry_and_stations` | Deep-arena POI at $(500, 500)$ ($D = 575\,\text{m}$) requiring 6 intermediate relays ($H=7$). | `PASS` (Collinear station spacing $\le 95.0\,\text{m}$) |
| `test_atomic_fleet_reservation` | Target requiring $K=3$ relays with only 2 available UAVs in fleet. | `PASS` (Atomic rejection; 0 deployed, task deferred) |
| `test_airborne_relay_reuse` | Consecutive POIs in adjacent sectors; retargeting airborne relays without landing. | `PASS` (Existing airborne relays repositioned) |
| `test_localized_link_handoff` | Approaching 1200s sortie limit on relay 2 in 4-hop chain; replacement dispatched. | `PASS` (Station 2 swapped; chain continuity preserved) |
| `test_localized_failure_recovery` | Hardware failure injected on intermediate relay in 3-hop chain. | `PASS` (Standby UAV assumes station; link restored) |
| `test_clean_chain_teardown` | Surveyor completes POI loiter inspection. | `PASS` (All relays released to `UAVRole.IDLE`) |
| `test_metrics_instrumentation` | Full mission run with multiple multi-hop deployments. | `PASS` (All 5 Phase 5B metrics recorded accurately) |
| `test_reporting_compliance_across_multihop` | POI detection packet routed across 5-hop mesh to GCS. | `PASS` (Reporting latency $\le 10.0\,\text{s}$) |
| `test_backward_compatibility_single_relay` | Legacy task within $190\,\text{m}$ evaluated by multi-hop planner. | `PASS` (Deploys exact 1-relay chain identical to Phase 4) |

---

## 5. Summary Benchmark Comparison

| Dimension | Baseline A0 (Phase 1) | Single-Relay (Phase 4) | Multi-Hop (Phase 5B) |
| :--- | :---: | :---: | :---: |
| **Max Reach Envelope** | Direct ($100\,\text{m}$) | Single Relay ($190\,\text{m}$) | Geometric Full Arena ($D \le 1185.6\,\text{m}$)* |
| **Max Communication Hops** | 1 hop | 2 hops | 13 hops |
| **Max Relay Count per Task** | 0 | 1 | 12 |
| **Reporting Compliance ($\le 10\,\text{s}$)** | Fails beyond 100m | 100% (within 190m) | 100% (within active chain) |
| **Atomic Fleet Reservation** | N/A (single UAV) | Partial | Fully Atomic ($1 + K_{\min}$) |
| **In-Flight Link Handoff** | Not Supported | Not Supported | Supported (Localized replacement) |
| **Airborne Relay Reuse** | Not Supported | Not Supported | Supported |
| **Total Test Suite** | 280 tests | 324 tests | **333 tests (100% passing)** |

*\* Note: Full-arena reach in Phase 5B denotes geometric planner capability ($H_{\min} \le 13, K_{\min} \le 12$) verified on deterministic unit/scenario benchmarks. Full-arena randomized operational validation across multiple seeds is the scope of Phase 5C.*
