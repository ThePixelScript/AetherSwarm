# Atomic expedition emission and elastic tether (experimental)

This isolated branch builds on integration source
`2494b3f004045a1e92318d58ed1369e4db89201e`. It is not the frozen five-UAV
Stage-1 release. Native Webots playback has NOT been verified here.

## Contracts and implementation

Enable `ConnectivityAwarePlannerConfig(elastic_tether_enabled=True)` explicitly.
Default legacy behavior is preserved. The runner, planner and tether share ONE
DynamicRelayManager and the configured Gamma analyzer. A conflicting explicit
manager is rejected. StateStore is unchanged and remains per-command transactional.

**Atomic expedition plan emission**, not an atomic StateStore transaction:
the planner checks the entire surveyor/relay reservation, rejects incomplete or
locked teams, excludes already committed relays from new station deployment,
and emits the full same-tick command set. Disjoint teams can be emitted in one
tick. Existing on-station shared infrastructure retains dependency bookkeeping.
The score uses existing A1 utility minus its travel weight times total relay
travel, with existing energy feasibility gates. IDs resolve ties. This is a
greedy candidate-team selection, not global assignment optimization.

Corridor station generation and altitude-dependent discovery remain upstream
implementations. Hidden locations become tasks only through DiscoverTaskCommand
after emergence and footprint detection. The altitude table is a simulation
assumption, not measured sensing performance.

FORMING means moving, not waiting for every relay to reach its station. ACTIVE
still requires station arrival and Gamma connectivity. Departure clearance and
safety can legitimately delay an individual vehicle; same-tick commands do not
promise every vehicle moves under every geometry.

For each ordered chain link the prospective nominal step is clipped along its
actual velocity/target segment to an 80 m sphere (85 m safe hop minus 5 m margin).
Physical Gamma range stays 100 m. Existing oversized links are not teleported:
the guard allows no further stretching of their initial distance. Safety filters
the copied targets, then the tether checks the actual accepted motion batch.
Offending endpoints are slowed locally; continuous pair separation is rechecked
after slowing because independent speed changes can otherwise create collisions.
The 20 m separation plus numerical buffer is maintained where initially available;
pre-existing grounded/coincident starts are not magically separated. Gamma verifies
currently connected non-RTH peers on prospective immutable snapshots. A bounded
projection either accepts a safe batch or fails explicitly. Authoritative targets
are not overwritten with temporary hold points. Energy is recalculated for the
actual shortened step. Returning vehicles are excluded from chain membership,
but may still wait for safety/protected-peer constraints; arbitrary layouts have
no liveness guarantee. Complexity is bounded by 65 projection passes, each with
O(UAVs squared) separation checks plus Gamma analysis when geometrically feasible.

Pending reports retain dependent infrastructure until delivery/timeout, except
existing failure/RTH handling. Existing dependency-aware teardown is reused.
No RF, packet-level, optimal path-planning or generalized handover claims are made.

## Reproduce

From this branch's repository root, with Python 3.12:

```powershell
python -m venv .venv
.venv/Scripts/python -m pip install -e ".[dev]"
.venv/Scripts/python -m pytest tests/ -q --basetemp=pytest_atomic_full
.venv/Scripts/python scripts/run_atomic_expedition_demo.py --runs 3
```

The unchanged eight-UAV generator produces five known and five hidden PoIs for
seed 2026. This demo explicitly bounds PoIs to x=30..200, y=440..560: it is NOT
full-arena coverage evidence. Known emergence is 0..120 s and hidden emergence
20..180 s. Runtime uses a 1200 s single-sortie limit. `scenario.yaml` records
generator output; `effective_scenario.json` additionally records these runtime
overrides. Reproduce through the script, not the raw generator YAML alone.

Outputs: `results/atomic-expedition/summary.json` and each `run-N/` containing
metrics, full report, effective config and replay-schema-validated `trace.json`.
Three-run comparison includes discoveries, team choices, metrics and complete
trace hashes. No Webots process is launched. Use the existing Webots replay
integration with the generated trace only after checking native playback.

## Executed evidence

Three runs produced identical substantive results and trace hash
`7bf29fc83e9cc817a68e3fa3c3876e92ccd6c51ec85893a6df3a7250ac1b4bb1`:

| Measurement | Result |
|---|---:|
| UAVs / known / hidden / hidden discovered | 8 / 5 / 5 / 5 |
| Mean discovery distance | 51.388 m |
| Maximum simultaneously moving UAVs / assigned tasks | 8 / 3 |
| Serviced tasks | 7 / 10 |
| Delivered reports / missed reporting deadlines | 9 / 0 |
| Connectivity availability / downtime (existing evaluator) | 1.0 / 0 s |
| Model route PDR / latency | 0.4885 / 12.02 ms |
| Minimum assessed separation | 20.00035039561619 m |
| Separation / geofence / flight / landing / battery violations | all 0 |
| Landed / maximum airborne | 8 / 1142 s |
| Nominal tether clips / Gamma checks / post-safety scaled ticks | 1011 / 1126 / 1048 |

There are ten rejected redundant ReleaseRelayRoleCommands against already-IDLE
vehicles in upstream teardown flow, recorded verbatim in rejection_details.
There are no rejected expedition assignment/movement commands in this run.
These are not silently presented as a rejection-free mission. Reports can exceed
serviced tasks because sensing/telemetry and physical service are distinct.
The high hold count and incomplete service expose conservative geometry/liveness
limitations; this does not establish superiority over serial deployment.

Focused tests cover complete/partial teams, multiple teams, protected workers,
all-link hold/resume, concurrent FORMING motion, snapshot immutability, dependent
release and actual runner integration. Existing discovery and atomic-chain tests
remain enabled. Missing generated random-scenario fixtures in three safety tests
are replaced with the same deterministic generator, not weakened assertions.

Final executed validation: 47 focused discovery/chain/tether tests passed;
468 full-suite tests passed, zero failures. `pip check`, compilation and
`git diff --check` passed. Native Webots is still unverified.
