# STAGE-1 VERIFIED CLAIMS

## VERIFIED STAGE-1
- deterministic simulation architecture
- FANET graph abstraction
- multi-hop GCS routing
- communication-aware task assignment
- hard destination connectivity gating
- batch connectivity preservation
- fixed scenario-specific three-relay/two-surveyor ingress deployment
- task-worker failure reassignment
- safety / RTH
- telemetry SLA

## QUALIFIED / BOUNDED
- Webots as visualization/replay only
- Native Webots rendering is not verified by standalone controller tests.
- PDR and latency are model estimates, not sampled packet delivery or RF measurements.
- Reporting uses route availability and modeled delay; reporting compliance is not PDR.
- Batch protection covers accumulated destination topologies, not moving trajectories.
- Canonical UAVs start separated inside the arena, not grounded together at the GCS.
- Fixed PoIs/deployment prove a bounded PoC, not arbitrary full-arena coverage.
- Historical metric metadata saying failure recovery is deferred refers to unavailable aggregate recovery metrics; the controlled worker-reassignment experiment is separate evidence.

## Evidence authority
Use `stage1_output/EVIDENCE_MANIFEST.md`, its executed JSON artifacts and
`FINAL_STAGE1_SUBMISSION_RELEASE_REPORT.md`. Older development reports are not
current release evidence. No claims in this file imply hardware validation.

## Claim-consistency audit
The release search covered adaptive/dynamic relay, make-before-break, self-healing,
100% PDR, physical RF, packet-level, AODV, OLSR, multi-sortie and recharge.

| Location | Classification |
|---|---|
| README and this document: dynamic relay/handoff/RF/packet-level/recharge | Explicit experimental/Stage-2 limits, not implemented claims |
| `docs/CHALLENGE_MISSION_CONTRACT.md`: swap/relaunch permission and multi-sortie rules | Historical assumptions/questions, not current implemented capability |
| `docs/DETECTION_REPORTING_VALIDATION_V1.md`: absence of dynamic relays | Historical experiment, not current canonical metrics |
| `docs/algorithms/task-allocation.md`: future adaptive relay and handover | Historical roadmap; current A1 status is described here |
| `src/ares_swarm/evaluation/metrics.py`: deferred dynamic relay/recovery metrics | Legacy metadata; not evidence of a generalized recovery controller |
| `src/ares_swarm/telemetry/manager.py`: no packet-level queuing | Accurate limitation |

No current submission claim of self-healing, 100% PDR, AODV or OLSR is supported.
Historical development documents are retained rather than rewritten as new evidence.

## STAGE-2 ROADMAP
- bounded temporary relay reallocation
- arbitrary deep relay-chain synthesis
- make-before-break general handoff
- multi-sortie/recharge
- packet-level ns-3
- GNN
- physical RF validation
