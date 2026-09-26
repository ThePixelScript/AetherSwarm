# Stage-1 evidence manifest

Verification date: 27 September 2026 (Asia/Kolkata; machine JSON uses UTC).

- Release branch: `aether/stage1-submission-verified`
- Tested source commit: `aeb4837fbe0e2d3c7a60da35b34a064c39d8964d`
- Parent candidate: `6b7797f05d71a11a4059611ac236742eee33d42e`
- Release tag: `stage1-submission-verified` (annotated, attached to the subsequent evidence-only release commit).
- No generalized integration branch was merged.
- Python: 3.12.14; pip: 25.0.1; exact tested package constraints: `requirements-release.txt`.
- Canonical scenario: `scenarios/poc_round1.yaml`; effective runner seed: **2026**, supplied by the runner default (the YAML's seed 42 is not the effective command default).

## Clean-room provenance

Two independent GitHub clones were used outside `D:\AetherSwarm`:
`AetherSwarm_release_verify` for review/corrections and
`AetherSwarm_release_cleanroom` for final committed-source execution.
Both are under `C:\Users\anwar\OneDrive\Documents\ChatGPT\Drone`.
The latter has its own `.git` directory and brand-new `.venv-release`.
Neither reuses the original repository virtual environment.

Before publication, the second GitHub clone fetched the new release branch from the
first local clone and checked out the exact source SHA above. This allowed the release
to be tested **before** pushing it. The manifest does not pretend that the unpublished
branch was already available from GitHub. Installation and imports resolve to the
second clone, and its validator records an empty starting worktree status.

Executed setup (the Python launcher is absent on this machine, so the documented
absolute-interpreter alternative was used):

```powershell
git clone https://github.com/ThePixelScript/AetherSwarm.git AetherSwarm_release_cleanroom
cd AetherSwarm_release_cleanroom
git fetch ../AetherSwarm_release_verify aether/stage1-submission-verified
git switch -c aether/stage1-submission-verified FETCH_HEAD
& 'C:\Users\anwar\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -m venv .venv-release
.\.venv-release\Scripts\python.exe -m pip install -c requirements-release.txt -e ".[dev]"
.\.venv-release\Scripts\python.exe -m pip check
.\.venv-release\Scripts\python.exe scripts/run_stage1_final_validation.py
```

The validator invokes full/focused pytest, five independent direct imports,
10 canonical runs, five paired E2 runs, three worker-failure runs, batch probes and
three existing random seeds. Its JSON stores each exact command and exit code.
Stdout/stderr logs and individual traces remain in the generated `runs/` directory;
they are reproducible but not all versioned. The full-test log and JUnit are versioned.

## Authoritative artifacts

| File | What it proves |
|---|---|
| `stage1_submission_report.json` | Actual canonical task, report, communication and safety results; events, landing records and complete-history hash |
| `a0_e2.json`, `a1_e2.json` | Same controlled scenario, different assignment and reporting outcome |
| `failure_evidence.json` | Worker in service, failure/deferral, replacement and task completion, with observed event sequence |
| `test_results.xml` | Actual 336 collected/passed, zero failed/errors/skipped |
| `test_execution.json` | Full-suite command, stdout/stderr and zero exit status |
| `final_validation_summary.json` | Source SHA/environment, full/focused counts, batch results, run hashes, random sweep and commands |
| `../docs/RELEASE_REVIEW.md` | Architectural/test audit and semantic qualifications |
| `../FINAL_STAGE1_SUBMISSION_RELEASE_REPORT.md` | Human interpretation of this evidence |

Reports are copied from executed output, never hand-edited. The publishing commit adds
evidence/docs only after the tested code commit; Git can verify that `src/`, `tests/`,
`scripts/` and dependency definitions are identical between them. This avoids inventing
a self-referential report SHA. Use `git rev-parse stage1-submission-verified^{commit}`
for the final published commit.

## Reproducibility and claim boundaries

Canonical report hashes match across 10 executions, including exact event/trajectory/
network-history evidence. E2 matches across five repetitions; worker-failure observations
match across three, excluding only output-location strings. No mission metric is normalized.

Reporting compliance is not packet PDR. Route PDR/latency are deterministic model estimates.
Native Webots playback was **not verified**. The maintained export command executed, and
14 standalone visualization/control tests passed; neither establishes native 3D physics/RF.
Full-arena seeds 2026, 42 and 137 completed zero tasks with zero detections: their vacuous
report-compliance value of 1.0 must not be presented as mission success. The canonical
proof is bounded and initially prestaged, not a general center-launch coverage proof.
