# Independent release review

## Scope and provenance

The release branch starts at `6b7797f05d71a11a4059611ac236742eee33d42e`.
The generalized experimental branch was not merged. The original checkout and
its untracked developer files were left intact. Verification uses independent
GitHub clones and newly created Python 3.12 environments.

## Reconciled findings

| Previous claim | Independent result |
|---|---|
| Candidate canonical service 10/10 | Reproduced before release corrections |
| 316 passing in a clean install | Not reproduced initially: 304 passed, 3 failed, 9 errors; missing untracked scenario/trace fixtures |
| Existing validator proves metrics | Rejected: literal results, unchecked failures and fallback test count were not evidence |
| Stale tracked canonical report means candidate still unsafe | Rejected as current runtime evidence: the report was historical generated output, not a fresh candidate run |
| Accumulated batch protects topology | Correct endpoint mechanism; strengthened regression protects an ineligible active peer |
| Previous batch regression alone closes loophole | Incomplete: eligible peer c could itself take the second task |
| Dynamic relays/make-before-break complete | Unsupported; explicitly Stage 2 |
| Existing clean checkout reused a venv | Insufficient reproduction; new clone-local environments are required |

## Minimal production corrections

1. Restore A1's previous per-call scoring context in `finally`, including exceptions.
2. Sort available UAV IDs so output ordering is deterministic, not dict-insertion-dependent.
3. Preserve A0-compatible feasible negative-utility assignments: the batch override had
   introduced an implicit zero threshold absent from `TaskAllocatorConfig`. Topology-invalid
   negative-infinity candidates remain excluded. No weights or feasibility gates were tuned.
4. Remove telemetry's runtime import of an annotation-only scenario type, breaking a fresh
   direct-import cycle without changing detection/reporting behavior.

No StateStore, Gamma, safety, physics or ingress implementation was changed.

## Batch audit

Tasks are ordered by priority, emergency status and ID. Feasibility uses inherited A0
gates. Accepted destinations update a copied working snapshot and Gamma analysis before
the next task. Candidate and previously connected peer identities must remain connected.
The canonical runner injects the same configured Gamma analyzer into A1 and MissionRunner.
Planning does not call StateStore mutation methods.

For GCS `(0,0)`, a `(80,0)`, b `(80,60)`, c `(160,30)`, t1 `(0,-80)` and t2 `(0,80)`:
both individual moves initially appear feasible. With c active in RELAY role, a takes t1,
t2 remains unassigned, and c stays connected. Forcing b to t2 disconnects c. With c eligible,
the safe alternative a-to-t1/c-to-t2 remains valid. These are endpoint proofs, not proofs
of simultaneous trajectory connectivity.

## Every changed legacy A1 expectation

| Change in candidate | Classification | Release treatment |
|---|---|---|
| Task x=60 to x=40 in 50 m range | Fixture correction, but no longer an equal-travel causal comparison | Correct comment; do not use it as standalone communication-benefit evidence |
| One-hop u1 replaced by u2 | Valid destination-feasibility semantic update | u1 move to x=37.5 exceeds 30 m GCS range; assert u1 utility is -infinity and u2 utility is 4.239375 |
| Disconnected swarm gets zero assignments | Valid hard-gate semantic update | Rename/comment safe deferral; do not describe it as blackout dispatch or liveness recovery |
| Controlled unit test disables destination awareness | Scoring-only coverage, insufficient production evidence | Label explicitly legacy; add default-mode blackout regression and retain executed E2 causal evidence |

## Evidence definitions and limits

- Service completion time is when all tasks are COMPLETE; mission termination includes return/landing.
- `tasks_assigned` counts currently assigned/in-progress tasks, not cumulative assignments; use events for assignment history.
- Connectivity availability is connected active-UAV time slots divided by active-UAV slots.
- Downtime is time steps with at least one disconnected active UAV, not the complement of the connected fraction.
- Model route PDR averages positive available-route samples; disconnected samples are represented by connectivity/downtime, not included in that conditional mean.
- Route latency is a modeled sum, not measured packet latency. Reporting uses route availability and modeled delay, not sampled PDR.
- Energy fields retain historical Wh labels but are not calibrated to an actual airframe.
- Canonical tasks/initial positions are fixed and bounded. All detections may come from one surveyor; a two-surveyor role assignment is not proof of balanced workload.
- Canonical initial UAVs are separated inside the arena, not a validated center takeoff sequence.
- Native Webots GUI/video needs separate playback verification. The release tests cover standalone trace/control behavior only.

## Static hygiene

Removed the tracked `scripts/fix_tests.py` assertion-replacement utility. Git retains it
historically; no original user scratch files were deleted. Generated traces, caches,
environments and repeated validation runs are ignored. Historical reports remain historical;
the release manifest identifies the current authoritative artifacts.
