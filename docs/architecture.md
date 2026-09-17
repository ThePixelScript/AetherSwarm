# Phase 1 architecture

## Scope and ownership

StateStore is the sole authoritative owner of SwarmState. SimulationEngine does not
exist yet. In Phase 2 the composition root creates an opaque object as writer_key,
constructs StateStore with it and transfers the key only to SimulationEngine.
Other modules receive StateSnapshot values or the StateReader protocol.

The key is checked by identity on commit_transition and reset. A source string is
audit data, not authorization. Python reflection/private-attribute access is not a
security boundary; this API prevents accidental mutation within trusted application
code. Do not use object.__setattr__, private fields or monkey-patching to bypass it.

All model dataclasses are frozen. Collections use tuples. Metadata/payload mappings
are recursively copied to mapping proxies, and lists become tuples. Caller-owned
metadata cannot alter stored values. Snapshot data can safely share these immutable
objects. A snapshot retains its historical values after commits and resets.

## Pipeline

~~~text
Scenario + seed
      |
SimulationEngine (Phase 2; owns writer capability)
      |
StateStore.get_snapshot()
      |
Communication analysis -> optional later GNN -> deterministic planner
      |
ActionProposal intents
      |
Safety/energy validation
      |
SimulationEngine converts accepted intent to typed StateTransition
      |
StateStore.commit_transition(transition, writer_key=engine_key)
      |
Immutable new snapshot -> observer / structured logs
~~~

ActionProposal is a different type from StateTransition and cannot be committed.
Intent parameters are immutable JSON data at this stage. Concrete intent schemas
will be introduced alongside each later algorithm, without weakening typed mutations.

## Interfaces

| Owner | Contract | Result |
|---|---|---|
| Shakeel | CommunicationAnalyzer.analyze(snapshot) | NetworkAnalysis tagged with snapshot_revision |
| Divesh | AutonomyPlanner.plan(snapshot, network_analysis) | list of ActionProposal |
| Sujal | SafetyValidator.validate(snapshot, proposals) | ValidationResult: accepted intents and explained rejections |
| Sujal | MetricsCollector.observe(previous_snapshot, transition, new_snapshot) | Observation only |
| Sarath | StateReader.get_snapshot() and StateStore APIs | Immutable snapshot / validated commit |

The future engine must reject mismatched analysis/proposal revisions and must pass
expected_revision on accepted transitions. StateStore detects stale transitions.
Protocols describe contracts. M0 implements CommunicationAnalyzer through
communication.analysis.BaselineCommunicationAnalyzer; planner/safety implementations
remain deferred. See algorithms/connectivity-analysis.md for the additive,
backward-compatible NetworkAnalysis contract and graph semantics.

## Commit semantics

- commit_transition constructs a candidate using immutable replacements.
- Whole-state constructors validate the candidate before assignment.
- Rejection leaves both state and revision unchanged.
- validate_transition checks the same path but never commits.
- AdvanceTime explicitly sets simulation_time. It does not move UAVs, drain batteries
  or execute events. Other transitions must use the current simulation timestamp.
- Time cannot decrease except through explicit reset.
- Revisions increase on every commit/reset, including same-time transitions.
- reset() restores the original initial state. reset(state) restores the supplied
  validated state without replacing the original reset baseline.
- This store is single-threaded; the future engine serializes commits. It is not a
  concurrent database and makes no thread-safety claim.

## Structural invariants versus policy

StateStore checks types, finite numbers, IDs, references, reciprocal assignments,
task lifecycle, inactive-UAV movement, route representation and timestamp ordering.
It does not decide task utility, geofence safety, separation, endurance feasibility,
critical relay protection, link feasibility or cooldown policy.

AssignTask updates UAV and task together. UnassignTask releases both, resets task
start time and returns it to PENDING; partial service progress is not modeled yet.
Completion requires IN_PROGRESS and releases both assignment references.
COMPLETED, FAILED and CANCELLED are terminal until an explicit state reset.

MarkUAVFailed clears that UAV's assignment, invalidates routes containing it, and
disables incident stored links. This is referential cleanup, not fault detection or
recovery planning. It selects no replacement and reassigns no task.

Routes are tuples (source UAV, ..., GCS). parent_relay_id is the next-hop ID and may
be the GCS for a direct route. Disconnected UAVs have empty routes, no parent, and
zero hops. Route nodes must exist and be active. Phase 1 does not require stored link
records for each hop: link observations and route decisions have separate updates.
Later communication analysis verifies edge feasibility.

## Units and configuration

- Positions, range and separation: meters in a local Cartesian frame.
- Velocity/speed: meters per second.
- Simulation times, durations, locks and event timestamps: seconds.
- Heading: degrees in [0,360); altitude_layer is a nonnegative index, not meters.
- Battery capacity, remaining energy, reserve and estimated RTH energy: watt-hours.
- battery_pct: [0,100]. Initial scenario percentage must agree with configured capacity.
  UpdateBattery carries both quantities; runtime capacity calibration is not modeled yet.
- base_latency and LinkState.latency_ms: milliseconds.
- rssi: optional dBm; snr: optional dB. No propagation calculation is implemented.
- packet loss, PDR and link quality: [0,1]; finite ETX >= 1. Disabled links remain
  representable without infinite ETX. Estimated PDR need not equal 1-loss because it
  is a separate observation estimate.
- Geofence: an axis-aligned rectangle. Membership enforcement is future safety work.
- communication.max_range is the canonical range; uavs.communication_range is a
  required matching compatibility alias.
- YAML is safely loaded with duplicate/unknown field and malformed type rejection.
  No implicit defaults overlay, arbitrary class construction or event execution occurs.

## Determinism and serialization

RandomManager(seed) exposes simulation, communication and events generators with
fixed SeedSequence spawn keys 0, 1, 2. Consuming one stream does not advance others.
No global random state is used. Exact sequence reproducibility assumes the pinned
NumPy version; dependencies are captured in constraints.txt.

Separate streams alone do not guarantee paired packet-loss realizations across
different future algorithms. Phase 3 must define packet/event identities and
indexed draws before claims of matched communication realizations.

JSON envelopes contain schema_version=1, a whitelisted model type and ordinary data.
Enums use string values; tuples use JSON arrays and recover to tuples. Unknown
types/fields/schema versions and non-finite model values are rejected. No pickle,
dynamic imports from input, or serialized writer capabilities are supported.
Snapshots serialize data and revision, never the StateStore itself.

StructuredFormatter emits timestamp, simulation_time, category, event, entity,
message, seed, scenario and metadata. Wall-clock timestamp is intentionally not a
deterministic simulation field. No global logger/handler is installed on import.

## Integration workflow

Sarath owns core contract changes and reviews cross-package dependencies.
Divesh and Shakeel consume snapshots and return analyses/proposals.
Sujal implements the engine and evaluation against those contracts.
Each change includes focused tests, and integration happens only after tests pass.
No member should edit another member's algorithm file to resolve interface drift;
agree on a versioned contract change through Sarath.

## Exact Phase 2 entry point

Create src/ares_swarm/core/simulator.py with SimulationEngine. It receives validated
AppConfig, ScenarioConfig, StateStore, its writer capability, and RandomManager.
Start with explicit time advancement and deterministic event ordering
(timestamp ascending, priority descending, stable event ID as tie breaker).
Use existing SimulationEvent descriptions; introduce event-state storage and
accepted event transitions through StateStore when execution is added. Do not keep
a second authoritative processed-event state in the engine.

Add event progression and motion incrementally with tests proving repeatability,
same-time ordering, monotonic time, and pause/reset behavior. All commits go through
StateStore. Communication/autonomy algorithms remain their later phases.
