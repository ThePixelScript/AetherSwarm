# AetherSwarm

CPU-only, deterministic algorithm-level UAV swarm simulation for Stage-1 preliminary design verification.

**Claim:** Autonomous swarm decision-making validated in a deterministic simulation environment.
This is not real-UAV, calibrated-radio, or hardware-in-the-loop validation.

## Submission scope

- Authoritative StateStore and canonical commands; immutable snapshots.
- Read-only Gamma FANET graph analysis, multi-hop GCS routes, deterministic shortest-hop routing.
- Communication-aware A1 with hard destination connectivity and accumulated batch endpoint protection.
- Fixed, scenario-specific three-relay/two-surveyor deployment PoC; not general adaptive relay management.
- Worker-failure task deferral and reassignment in a maintained controlled experiment.
- Modeled 2D separation/geofence intervention, RTH and landing, sortie-duration assessment.
- Independent detection-to-GCS reporting SLA evaluation.

PDR and route latency are **model estimates**. Delivery uses route availability and modeled delay; it does not sample packets, apply PDR as a delivery probability, or simulate radio contention.
The batch gate protects accumulated destination topologies, not the entire simultaneous flight trajectories.

## Reproduce (Windows PowerShell, Python 3.12)

Run from a fresh directory. Do not reuse another checkout's virtual environment.

```powershell
git clone https://github.com/ThePixelScript/AetherSwarm.git AetherSwarm_release_verify
cd AetherSwarm_release_verify
git checkout aether/stage1-submission-verified
py -3.12 -m venv .venv-release
.\.venv-release\Scripts\python.exe -m pip install -c requirements-release.txt -e ".[dev]"
.\.venv-release\Scripts\python.exe -m pip check
.\.venv-release\Scripts\python.exe -m pytest tests/ -q --basetemp=pytest_release
.\.venv-release\Scripts\python.exe scripts/run_stage1_submission.py
.\.venv-release\Scripts\python.exe scripts/run_e2_controlled.py
.\.venv-release\Scripts\python.exe scripts/run_demo_in_flight_recovery.py --output-dir results/recovery
.\.venv-release\Scripts\python.exe scripts/run_stage1_final_validation.py
```

If the Windows Python launcher is unavailable, use the absolute path of an installed Python 3.12 interpreter for the venv command. On POSIX use `python3.12` and `.venv-release/bin/python`.
The historical import/package name remains `ares_swarm` / `ares-swarm` for compatibility; the project is AetherSwarm.

The validator runs the full suite, independent focused suites, 10 canonical runs, 5 paired E2 runs, 3 failure runs, batch probes and a bounded random sweep. It exits nonzero on failed required execution, malformed evidence or a failed canonical gate. Expect several minutes on a laptop. It does not declare that native Webots or hardware was tested.

## Evidence locations

| Artifact | Meaning |
|---|---|
| `stage1_output/stage1_submission_report.json` | Generated canonical service/reporting/safety results and lifecycle evidence |
| `stage1_output/final_validation_summary.json` | Executed counts, metrics, determinism hashes, environment and source SHA |
| `stage1_output/EVIDENCE_MANIFEST.md` | Provenance and commands, including tested source commit |
| `FINAL_STAGE1_SUBMISSION_RELEASE_REPORT.md` | Human release review and limitations |
| `e2_output/a0_e2.json`, `a1_e2.json` | Different assignments and reporting outcomes, not just task completion |
| `results/recovery/` | Worker failure/reassignment timeline, replay and summary |
| `stage1_output/runs/` | Generated per-run evidence and captured stdout/stderr/return codes |

Reproduction regenerates the tracked canonical report. Evidence generated at a tested code commit may be published in a subsequent evidence-only commit; read the manifest rather than attributing it to a fabricated SHA.

## Webots (optional visualization only)

The core and tests do not require Webots. Standalone replay/control tests generate their own traces. The legacy export command below creates a visualization trace; it is **not** the submission runner and does not prove the canonical ingress result:

```powershell
.\.venv-release\Scripts\python.exe scripts/export_webots_trace.py --scenario scenarios/poc_round1.yaml --experiment E1 --seed 42 --max-ticks 150 --output visualization/webots/data/e1_authoritative_trace.json
```

With Webots separately installed, open `visualization/webots/worlds/uavx_round1.wbt`. Native 3D playback must be checked on the recording machine before describing the video as verified. Webots displays simulated telemetry; it is neither the autonomy engine nor physical RF validation.

## Important limits

- Canonical PoIs are a fixed bounded demonstration, not full-arena randomized coverage proof.
- The canonical UAVs are initially separated inside the arena; this is **not a verified takeoff-from-center demonstration**.
- Fixed ingress uses scenario-specific UAV IDs and waypoints. Do not apply it as a general relay controller.
- Some initial/return communication downtime is permitted by the current demo; inspect measured values, not a claim of continuous 100% connectivity.
- Model energy units are not flight-hardware calibrated.
- Task service and information reporting are distinct. A 10 s reporting SLA is not a spawn-to-service deadline.
- General random runs may safely defer most tasks. Zero detections must not be presented as successful mission reporting.
- Historical design documents and old generated files are not release evidence. The manifest and release report take precedence.

## Stage 2 / separate experimental work

Dynamic relay reassignment, arbitrary chain synthesis, make-before-break handoff, shared relay trunks, recharge/multi-sortie operations, ns-3/packet-level models, GNN and physical RF validation are not verified Stage-1 features. No experimental integration branch is required by this release.

## Ownership

Sarath: core/integration; Divesh: autonomy; Shakeel: communication; Sujal: simulation/safety/energy/evaluation. Release corrections preserve these boundaries.
