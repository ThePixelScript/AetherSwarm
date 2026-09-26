# AetherSwarm Stage-1 verified release

## Executive status

The bounded deterministic Stage-1 PoC is reproducible. Release corrections address
untrustworthy evidence, non-self-contained tests, direct-import health and small A1
batch-context/compatibility defects. They do not add a new autonomous capability.
The source tested is `aeb4837fbe0e2d3c7a60da35b34a064c39d8964d`; subsequent publication
changes contain evidence and documentation only. See the evidence manifest for provenance.

Submission branch: `aether/stage1-submission-verified`.
Submission tag: `stage1-submission-verified`.

This is simulation-only preliminary design evidence, not complete organizer-scenario,
hardware, RF or general disaster-zone coverage validation.

## Frozen architecture

StateStore remains the authoritative owner. Gamma consumes snapshots and returns derived
network analysis. A1 uses the same configured analyzer for destination and accumulated
batch endpoint checks. Canonical commands apply decisions; existing movement, safety,
RTH and telemetry modules execute the mission. No core/Gamma/safety/physics implementation
was changed by this release correction, and no generalized relay branch was merged.

## Executed evidence

| Measure | Result |
|---|---:|
| Full suite | 336 passed; 0 failed, 0 errors, 0 skipped |
| Independent focused suites | Autonomy 62; communication 103; telemetry 15; safety 42; simulation 18; visualization 14; batch 8 |
| Canonical serviced tasks | 10/10 |
| All-task service completion | 429 s |
| Mission termination after return/landing | 1183 s |
| Detections / timely delivered reports | 10 / 10 |
| Reporting deadline exceeded | 0 |
| Mean / maximum reporting delay | 0.423 / 4.019 s |
| Model-estimated available-route PDR | 0.334 |
| Model-estimated route latency | 18.59 ms |
| Active-UAV connectivity availability | 0.9959 |
| Time with at least one disconnected active UAV | 8 s |
| Modeled energy consumed (uncalibrated Wh label) | 6504.8121 |
| Minimum modeled airborne separation | 20.00500005608731 m |
| Separation / geofence / landing / flight-duration / exhaustion violations | 0 / 0 / 0 / 0 / 0 |
| Maximum airborne duration | 1182 s |
| Landed UAVs | 5/5 |

Ten canonical executions produced identical substantive reports and history hashes.
Passing tests support the bounded model; they do not establish physical safety margins.
The separation margin is approximately 5 mm in this exact simulation, not a robust
real-airframe clearance claim. Historical 23.02 m, 100% connectivity, PDR=1 and 5 ms
claims are not the current canonical results.

## Batch and production A1 evidence

With active, connected, task-ineligible relay c, a-to-t1 is accepted and t2 remains
unassigned because b-to-t2 would disconnect c. The final Gamma analysis preserves c.
With c eligible, the safe a-to-t1/c-to-t2 alternative is selected. StateStore is unchanged
by planning. Default destination-aware A1 explicitly defers a blackout assignment that
A0 makes. Scoring context is restored on success and exceptions. The old current-position
unit comparison is named legacy, not represented as production-mode proof.

## Controlled E2 causal result

Both allocators service the same task under identical seed/configuration/impairments.
At tick 260, A0 assigns `uav_3`, which detects at 264 but has no reporting route and
times out at 275 (11 s). A1 assigns `uav_4`, detects at 264 and delivers via
`uav_4 -> uav_2 -> gcs` after 0.01859641602783313 s. Thus A0 delivers 0/1 reports;
A1 delivers 1/1. Five repetitions preserve both assignments and outcomes.
This is a controlled causal demonstration, not a population-wide claim of superiority.

## Worker failure

`uav_1` is servicing `poi_recovery`; the maintained demo observes it in progress at
tick 7, fails it at tick 8, verifies DEFERRED state, and reassigns the same task to
`uav_2` at tick 8. Completion occurs at tick 21. Three repetitions match. Minimum
separation is 31.23 m with no separation violations. This proves worker-task recovery,
not autonomous reconstruction of a failed relay network.

## Generalization and limitations

Existing full-arena random seeds 2026, 42 and 137, run without fixed canonical ingress,
each service 0/10 tasks and produce zero detections. All tasks remain PENDING; they are
not relabeled as DEFERRED to improve the story. Recorded separation is 20 m with zero
separation/geofence/exhaustion observations, but repeated safety interventions and no
progress are not evidence of useful mission robustness. Their reporting ratio of 1.0
is vacuous and must not appear as successful reporting.

Other explicit limits:

- Canonical starts are separated inside the arena, not a verified grounded center takeoff.
- Fixed three-relay/two-surveyor ingress uses scenario-specific IDs/positions, not general relay optimization.
- The ten canonical PoIs occupy a bounded fixed area; all detections come from one surveyor.
- A1 checks accumulated endpoints, not continuously connected simultaneous trajectories.
- Reporting uses route availability plus modeled delay, not packet sampling, contention or calibrated RF.
- Model route PDR is conditional on available positive-route samples; connectivity captures outage exposure separately.
- RTH/landing and flight limits are verified in this scenario, not for every possible deployment.
- Native Webots playback: **NOT VERIFIED**. Export and standalone control support are tested.

## Reproduction and freeze

The README gives clone, fresh Python 3.12 environment, pinned install, tests, canonical,
E2, failure demo and final-validator commands. The original `D:\AetherSwarm` environment
was not reused. The final clean clone installed successfully, passed `pip check` and
five separate direct-import probes, then executed the complete validator.

The validator fails closed on child-process failure, invalid test XML, missing canonical
fields, failed mission/safety gates, lost E2 causal distinction and nondeterminism. It
does not substitute a test count or invent unavailable metrics.

Freeze this bounded release after publication. General relay reassignment, arbitrary
chains, make-before-break handoff, shared relay trunks, recharge/multi-sortie, packet-level
simulation, GNN and physical RF validation remain Stage 2. Immediate non-code work is
the proposal, architecture/results figures, an honestly labeled demo video and packaging.
