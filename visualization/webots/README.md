# AetherSwarm Webots R2025a 3D Robotics Layer

## Overview
This directory contains the downstream 3D robotics simulation, visual working model, and independent spatial verification layer for **AetherSwarm** using Cyberbotics Webots R2025a.

> [!IMPORTANT]
> **Authoritative Architecture**: Headless AetherSwarm (`ares_swarm`) running in Linux/WSL remains the single authoritative source of truth for all mission state, A1 task allocation, Gamma RF communications analysis, battery/energy dynamics, hardware failure/recovery logic, and official benchmark metrics. Webots visualizes the authoritative AetherSwarm working model from an immutable simulation trace; it acts purely as an observational downstream consumer and independent spatial verifier and does not simulate autonomy, communication protocols, or physics overrides.

---

## Directory Structure
```text
visualization/webots/
├── README.md                                    # This document
├── worlds/
│   └── uavx_round1.wbt                         # 1000m x 1000m arena with 100m grid, GCS, drop-lines & viewpoints
├── controllers/
│   └── aetherswarm_supervisor/
│       └── aetherswarm_supervisor.py           # In-world Supervisor controller & dual-mode spatial verifier
├── protos/
│   ├── SwarmDrone.proto                        # Polished quadrotor drone model with navigation lighting
│   └── TaskPOI.proto                           # Dual-ring POI target marker and status pylon
└── data/
    ├── .gitkeep
    ├── e1_authoritative_trace.json             # Official E1 benchmark trace (2,700 ticks)
    ├── random_scenario_trace.json              # Randomized working scenario trace (2,700 ticks)
    ├── recovery_authoritative_trace.json       # In-flight failure & recovery scenario trace (100 ticks)
    ├── webots_spatial_verification.txt         # Independent spatial verification report output
    └── screenshots/                            # Milestone high-resolution captures
```

---

## Visual Working Model & Spatial Verification Features

1. **Metric Ground Grid & Arena Boundary**:
   - Subtle 100m grid lines across the 1000m x 1000m operational arena for clear spatial scale reference.
   - High-visibility safety perimeter boundary line with scale tick marks every 100m.
   - 100m operational ceiling boundary wireframe and vertical corner warning beacons.

2. **Ground Control Station (GCS)**:
   - Located at authoritative coordinate `[-75.0, 500.0, 0.0]`.
   - Distinctive 38m x 38m heavy operations apron with safety perimeter and runway transition corridor.
   - Dual-ring helipad with high-contrast "H" touchdown marking.
   - Mobile tactical command module with telemetry radome.
   - 22m communications lattice tower with directional microwave dish and high-intensity aviation beacon.

3. **UAV Visibility & Drop-Lines**:
   - Full quadrotor airframes with port (red) and starboard (green) aviation navigation lights.
   - Forward heading indicator nose cone.
   - High-luminance status beacon and halo ring for clear visibility at 1000m overview scale.
   - Dynamic real-time vertical ground drop-lines and ground footprints for intuitive altitude readability.

4. **Authoritative Status Indicators**:
   - **Nominal / Active**: High-tech teal/cyan airframe with emerald green status beacon.
   - **Hardware Failure (FAILED)**: Warning crimson airframe with brilliant red beacon (retains authoritative coordinate without synthetic crash animation).
   - **Return-To-Home (RTH)**: Amber/yellow warning illumination.
   - **Landed / Standby**: Subdued dark slate with dim standby indicator.

5. **POI Task Beacons**:
   - Dual concentric ground target rings with crosshair alignment axes.
   - Telemetry masts with dual-tier omnidirectional beacon sphere and halo ring.
   - Dynamic states: Golden Pending, Vibrant Cyan In-Progress, Emerald Green Complete, Red Alert Deferred.

6. **RF Communication Mesh & Active Routes**:
   - High-clarity cyan RF mesh links dynamically updated from authoritative connectivity graph.
   - Highlighted active route overlays showing multi-hop paths to GCS without synthetic role inventions.

7. **Dedicated Camera Viewpoints**:
   - `demo_presentation_cam`: Dedicated presentation perspective framed tightly around the active corridor and mission zone ($X \in [-75, 450], Y \in [200, 750]$).
   - `overview_cam`: 1000m comprehensive mission overview perspective.
   - `e1_failure_cam`: Close-up view of central cluster for tick-300 relay failure demonstration.
   - `recovery_cam`: Focused view on `poi_recovery` for tick-8 failure and dynamic A1 reassignment.
   - `gcs_landing_cam`: Helipad approach, descent, and touchdown viewpoint.
   - `overhead_cam`: Vertical nadir orthographic-style swarm overview.
   - `mission_camera`: Legacy compatible overview camera.

8. **Smooth Sub-Tick Visual Interpolation**:
   - Glides UAV 3D translations and scalar yaw rotations across intermediate Webots animation steps between authoritative 1.0s ticks.
   - Configurable via `AETHERSWARM_SUBSTEPS` (default: 4 sub-steps per tick).
   - Authoritative states, events, HUD metrics, and spatial verifications remain strictly locked to 1.0s ticks.

9. **Interactive Playback & Working-Model Control Layer**:
   - Built-in presentation replay controls with visible clickable HUD buttons:
     `[ RESET ]`, `[ -3s ]`, `[ -1s ]`, `[ PLAY / PAUSE ]`, `[ +1s ]`, `[ +3s ]`, `[ SPD: Nx ]`, `[ CAM: MODE ]`.
   - **Keyboard Replay Controls**:
     - `Home` / `R`: Reset to tick 0 and pause
     - `Left Arrow`: Step backward -1s (-1 tick, clamped at tick 0)
     - `Shift + Left Arrow`: Step backward -3s (-3 ticks, clamped at tick 0)
     - `Right Arrow`: Step forward +1s (+1 tick, clamped at final tick)
     - `Shift + Right Arrow`: Step forward +3s (+3 ticks, clamped at final tick)
     - `Space`: Toggle Play/Pause
   - **Configurable Playback Speed**:
     - Presets: `0.25x`, `0.5x`, `1.0x`, `2.0x`, `4.0x`.
     - `Up Arrow` / `]`: Increase speed to next higher preset
     - `Down Arrow` / `[`: Decrease speed to next lower preset
     - Keys `1`, `2`, `3`, `4`, `5`: Directly select 0.25x, 0.5x, 1x, 2x, 4x
     - HUD button click: Cycle speed through presets
   - **Camera Selection**:
     - Modes: `Overview`, `Follow UAV`, `GCS`, `Recovery`.
     - `O`: Overview perspective (arena-wide high vantage)
     - `F`: Follow active UAV (chase tracking)
     - `G`: GCS launch corridor & touchdown area
     - `V`: Recovery zone & failure intervention area
     - `C` / HUD button click: Cycle through cameras
   - **Presentation-Layer Visibility Toggles**:
     - `H`: Toggle HUD overlay
     - `P` / `T`: Toggle 3D POI target markers and beacons
     - `M`: Toggle active RF communication mesh lines
     - `U`: Toggle multi-hop routing paths to GCS
     - `D`: Toggle altitude drop lines and ground crosshair footprints
     - `B`: Toggle 100m metric reference ground grid
   - **Direct Mouse Click Support**: Clickable HUD controls for Reset, -3s, -1s, Play/Pause, +1s, +3s, Speed, Camera, and layer toggles.
   - **Immediate Visual Reconstruction**: Seeking immediately reconstructs and renders the selected frame, UAV coordinates, yaw headings, drone health indicators, POI statuses, and RF mesh links.
   - **Continuous Sub-Tick Interpolation**: Smooth visual interpolation resumes seamlessly after seeking.
   - **Strict Immutability**: The authoritative simulation trace is strictly read-only; replay controls only modify the observational playback cursor without mutating or rerunning simulation state.
   - **Inspection Hold**: Scenario completion holds the final frame open with a mission complete banner for presenter inspection (auto-quit via `AETHERSWARM_AUTO_QUIT=1`).

10. **In-World HUD Telemetry**:
   - Lightweight overlay showing real-time scenario name, tick progress, swarm active/failed counts, POI task completions, and independent spatial verification metrics.

---

## Requirements & Runtime Architecture
- **Webots**: Cyberbotics Webots R2025a installed on the host Windows system (`C:\Program Files\Webots\msys64\mingw64\bin\webotsw.exe` or `webots.exe`).
- **Python Runtime**: Standard Python 3.10+ (Webots built-in controller engine). The supervisor runs standalone from the exported trace and does not require active virtualenv packages or IPC sockets.
- **Path Resolution**: The project launcher resolves paths directly against the repository checkout, checking the mapped `Z:` drive or using PowerShell `.ProviderPath` to resolve the clean UNC path without provider prefix corruption.

---

## Launch & Execution Commands

### 1. Canonical Project Launcher (PowerShell)
The primary entry point for randomized and final-profile Webots missions is [`scripts/launch_random_webots.ps1`](../../scripts/launch_random_webots.ps1):

```powershell
# From the repository root in Windows PowerShell:
.\scripts\launch_random_webots.ps1 -Seed 2026

# Perform generation and trace export without opening GUI:
.\scripts\launch_random_webots.ps1 -Seed 2026 -NoLaunch
```

The script executes the authoritative simulation pipeline via WSL, exports `visualization/webots/data/random_scenario_trace.json`, verifies trace metadata integrity, and starts the Webots GUI with `visualization/webots/worlds/uavx_round1.wbt`.

### 2. Manual Launch via PowerShell
If launching manually from Windows PowerShell:

```powershell
$env:AETHERSWARM_SCENARIO = "random"

$WebotsExe = if (Test-Path "C:\Program Files\Webots\msys64\mingw64\bin\webotsw.exe") {
    "C:\Program Files\Webots\msys64\mingw64\bin\webotsw.exe"
} else {
    "C:\Program Files\Webots\webotsw.exe"
}

$WorldPath = if (Test-Path "Z:\home\dell\swarm_ws\AetherSwarm\visualization\webots\worlds\uavx_round1.wbt") {
    "Z:\home\dell\swarm_ws\AetherSwarm\visualization\webots\worlds\uavx_round1.wbt"
} else {
    (Resolve-Path ".\visualization\webots\worlds\uavx_round1.wbt").ProviderPath
}

Start-Process -FilePath $WebotsExe -ArgumentList "`"$WorldPath`"" -WorkingDirectory (Split-Path -Parent $WebotsExe)
```

### 3. Scenario Selector Variable (`AETHERSWARM_SCENARIO`)
The supervisor controller determines which trace to load based on the `AETHERSWARM_SCENARIO` environment variable or the CLI argument:
- `random` (default for randomized runs): loads `visualization/webots/data/random_scenario_trace.json`.
- `e1`: loads `visualization/webots/data/e1_authoritative_trace.json` (2,700 ticks).
- `recovery`: loads `visualization/webots/data/recovery_authoritative_trace.json` (100 ticks).

### 4. Standalone Observational Spatial Verification (Headless CLI / CI)
The supervisor controller can be executed directly from Python (in WSL or Windows) to evaluate authoritative traces without launching the Webots 3D interface:

```bash
# In WSL:
.venv/bin/python visualization/webots/controllers/aetherswarm_supervisor/aetherswarm_supervisor.py random

# Or pass a specific trace path directly:
.venv/bin/python visualization/webots/controllers/aetherswarm_supervisor/aetherswarm_supervisor.py visualization/webots/data/random_scenario_trace.json
```

---

## Independent Spatial Verification

During simulation replay, [`aetherswarm_supervisor.py`](controllers/aetherswarm_supervisor/aetherswarm_supervisor.py) independently audits spatial physics at every tick:
1. **Inter-UAV Minimum Separation**: Asserts that all active airborne UAV pairs maintain Euclidean distance $\ge 20.0\,\text{m}$.
2. **Geofence Boundary Compliance**: Asserts that all UAV coordinates remain strictly within the composite airspace geometry ($1000\,\text{m} \times 1000\,\text{m}$ arena and GCS staging corridor).
3. **Altitude Ceiling Compliance**: Asserts that all UAVs observe the $100.0\,\text{m}$ maximum operational ceiling.

Results are written automatically upon simulation completion to:
`visualization/webots/data/webots_spatial_verification.txt`

### Validated Run Results (Seed 2026, 2700 Ticks)
- **Verified Ticks**: 2,700
- **Minimum Observed Separation**: $20.00\,\text{m}$ (Constraint: $\ge 20.0\,\text{m}$)
- **Separation Violations**: 0
- **Geofence Violations**: 0
- **Altitude Violations**: 0
