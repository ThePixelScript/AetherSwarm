# Canonical-core integration audit (2026-09-17)

This report supersedes the earlier gamma-specific proposal-alias recommendation.
No source algorithms, teammate branches, or Git history were changed by this audit.

## Fetched state

| Ref | Commit | HEAD comparison (ahead / behind) |
|---|---|---|
| local aether/gamma | e202867c | - |
| origin/aether/gamma | e202867c | 0 / 0 |
| origin/aether/alpha | bcf2c315 | 0 / 1 |
| origin/aether/beta | 04a8dcbe | 0 / 3 |

Gamma's verified Phase-1/M0 implementation is staged, not committed. Commit ancestry
alone therefore does not describe the working implementation. History and upstream
are preserved. No merge, rebase, commit, or push was performed.

## Alpha and beta

Alpha is the candidate canonical core. Beta's core files match alpha exactly.
Beta A0AutonomyAdapter consumes alpha's StateSnapshot and converts AllocationResult
assignments into canonical AssignTaskCommand objects. It does not commit state.

Beta still defines TaskActionProposal for a legacy helper. That class fails gamma's
strict ValidationResult, but the current alpha/beta adapter bypasses the helper.
Alpha does not provide gamma's interfaces.autonomy.ActionProposal. Importing that
class into beta is therefore NOT the current canonical integration fix. Do not add
a duplicate shared proposal class or weaken validation. Sarath must specify the
single proposal/safety boundary if it is retained around alpha commands.

## Real alpha/gamma incompatibilities

| Gamma dependency | Alpha candidate |
|---|---|
| core.snapshot.StateSnapshot(state, revision) | core.models.StateSnapshot with top-level mappings, tick, state_version |
| UAVState.position and Vector2D | UAVState.position_xy tuple |
| FailureStatus, battery_pct, energy_remaining | FailureState, battery_percent, battery_energy |
| GCSState, LinkState, NetworkState, SwarmState | These gamma model types are absent |
| core.config, core.validation, gamma serialization | These gamma infrastructure modules are absent |
| capability-protected commit_transition | StateStore.apply(commands), set_simulation_clock |
| shared ActionProposal and ValidationResult | Not defined by alpha |

The channel, graph, analyzer and NetworkAnalysis import gamma-specific contracts.
Overlaying them on alpha is not yet import/API compatible. Alpha's public mutation
API also does not enforce gamma's writer-key boundary: engine-only commit access
must be agreed with Sarath, not silently replaced during communication work.

## Smallest safe integration slice

1. Keep alpha's authoritative models and StateStore as the candidate base; do not
   merge a second core implementation alongside them.
2. Adapt communication input access to alpha's snapshot, positions, state_version
   and failure enum. Do not construct a competing authoritative SwarmState.
3. Retain one NetworkAnalysis contract. Relocate/adapt derived link metrics and
   communication configuration as non-authoritative communication data, rather
   than importing gamma-only authoritative NetworkState. Agree the GCS identifier
   and absent recovery-state handling explicitly; do not fabricate snapshot fields.
4. Preserve channel formulas, connectivity and deterministic shortest-hop logic.
5. Add alpha-snapshot integration tests, then run focused and full tests on the
   actual adapted candidate before declaring team compatibility or pushing.

Expected review surface: communication/channel.py, graph.py, analysis.py,
interfaces/communication.py, related config/serialization boundaries, and
tests/communication fixtures/integration tests. Connectivity/routing algorithms
should need no policy change. Sarath owns core/command/engine authorization;
Shakeel owns communication adaptations. No beta source fix is needed for the
currently tested alpha-native adapter path.

## Executed evidence

- git fetch --all --prune succeeded; ref comparisons were refreshed.
- Isolated exact beta export: tests/core plus test_a0_core_integration.py:
  15 passed. Imports were verified to resolve to that export, not gamma.
- Gamma tests/communication plus tests/test_interfaces.py: 73 passed.
- Alpha/beta core diff: empty.
- No old D:\\Drone references found in project source/docs/config/test paths.
- Staged whitespace validation passed.

Prior gamma relocation validation: 158 full tests (71 communication, 87 Phase 1),
Python 3.12.14, editable imports, config, compile and dependency checks passed.
Those historical full results are NOT alpha/gamma integration evidence. No full
suite was rerun after a compatibility fix because no such source fix was applied.

Gamma M0 remains snapshot-in/analysis-out with one local StateStore and one local
NetworkAnalysis. There is no communication state mutation or M1 implementation.

## Continuation

Review the staged baseline before making a local preservation commit:

    git diff --cached --check
    git diff --cached --stat
    git diff --cached
    git commit -m "chore: checkpoint verified Phase 1 and M0 before alpha adaptation"

This is a local checkpoint, not a merge-ready integration claim. Do not push or
merge alpha/beta until the canonical adapter slice and combined tests are clean.
No force operations or metadata replacement are needed.

Status: gamma standalone M0 verified; alpha/gamma integration blocked by the
incompatible core/snapshot/config/result contracts described above.
