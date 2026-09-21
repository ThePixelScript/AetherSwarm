# AetherSwarm Webots R2025a 3D Robotics Layer

## Overview
This directory contains the downstream 3D robotics simulation, visual research demonstration, and independent spatial verification layer for **AetherSwarm** using Cyberbotics Webots R2025a.

> [!IMPORTANT]
> **Authoritative Architecture**: Headless AetherSwarm (`ares_swarm`) running in Linux/WSL remains the single authoritative source of truth for all mission state, A1 task allocation, Gamma RF communications analysis, battery/energy dynamics, hardware failure/recovery logic, and official benchmark metrics. Webots acts purely as an observational downstream consumer and independent spatial verifier; it does not simulate autonomy, communication protocols, or physics overrides.

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
    ├── recovery_authoritative_trace.json       # In-flight failure & recovery demo trace (100 ticks)
    ├── webots_spatial_verification.txt         # Independent spatial verification report output
    └── screenshots/                            # Demo milestone high-resolution captures
```

---

## Visual Demonstration Features (Phase 7C Polish)

1. **Metric Ground Grid & Arena Boundary**:
   - Subtle 100m grid lines across the 1000m x 1000m operational arena for clear spatial scale reference.
   - High-visibility safety perimeter boundary line with scale tick marks every 100m.
   - 100m operational ceiling boundary wireframe and vertical corner warning beacons.

2. **Ground Control Station (GCS)**:
   - Located at authoritative coordinate `[-50.0, 500.0, 0.0]`.
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
   - `overview_cam`: 1000m comprehensive mission overview perspective.
   - `e1_failure_cam`: Close-up view of central cluster for tick-300 relay failure demonstration.
   - `recovery_cam`: Focused view on `poi_recovery` for tick-8 failure and dynamic A1 reassignment.
   - `gcs_landing_cam`: Helipad approach, descent, and touchdown viewpoint.
   - `overhead_cam`: Vertical nadir orthographic-style swarm overview.

8. **In-World HUD Telemetry**:
   - Lightweight overlay showing real-time scenario name, tick progress, swarm active/failed counts, POI task completions, and independent spatial verification metrics.

---

## Requirements
- **Webots**: Cyberbotics Webots R2025a installed on the host system (`C:\Program Files\Webots\msys64\mingw64\bin\webots.exe`).
- **Python Runtime**: Standard Python 3.10+ (using standard library + Webots `controller` API). Zero dependencies on `ares_swarm`, `numpy`, `networkx`, `scipy`, or virtual environments.
- **Controller Execution**: Normal in-world Webots controller (`controller "aetherswarm_supervisor"`). No external IPC, named pipes, or sockets are required.

---

## Windows-Local Deployment
Webots on Windows executes natively against local drive paths. If developing within WSL, copy or mirror the project to `C:\AetherSwarmWebots\`:
```powershell
Copy-Item -Recurse "\\wsl.localhost\Ubuntu-24.04\home\dell\swarm_ws\AetherSwarm\visualization\webots" "C:\AetherSwarmWebots"
```

---

## Launch & Execution Commands

### 1. Official E1 Benchmark Scenario (Default)
Executes all 2,700 ticks of the official E1 benchmark, reproducing the tick-300 failure, communication topology reconfiguration, task completion, and RTH landing.
```powershell
$env:AETHERSWARM_SCENARIO = "e1"
Start-Process -FilePath "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" `
  -ArgumentList @("C:\AetherSwarmWebots\worlds\uavx_round1.wbt") `
  -WorkingDirectory "C:\Program Files\Webots"
```

### 2. In-Flight Recovery Demo Scenario
Executes the focused 100-tick in-flight recovery demonstration showing `uav_1` failure at tick 8 while on active task `poi_recovery`, dynamic A1 reassignment to `uav_2`, task completion at tick 21, and staged RTH/landing.
```powershell
$env:AETHERSWARM_SCENARIO = "recovery"
Start-Process -FilePath "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" `
  -ArgumentList @("C:\AetherSwarmWebots\worlds\uavx_round1.wbt") `
  -WorkingDirectory "C:\Program Files\Webots"
```

### 3. Standalone Observational Spatial Verification (CLI / CI)
The supervisor controller can also be executed directly from the terminal (WSL or Windows) to perform observational spatial verification against the authoritative traces without opening Webots:
```bash
# In WSL:
.venv/bin/python visualization/webots/controllers/aetherswarm_supervisor/aetherswarm_supervisor.py e1
.venv/bin/python visualization/webots/controllers/aetherswarm_supervisor/aetherswarm_supervisor.py recovery
```

---

## Independent Spatial Verification
During simulation playback, the Supervisor independently monitors and records 3D physical coordinates at every tick, verifying:
1. **Inter-UAV Minimum Separation**: Asserts that all active UAV pairs maintain distance $\ge 20.0\,\text{m}$.
2. **Geofence Boundary Compliance**: Asserts that all UAVs remain strictly within the $1000\times 1000\,\text{m}$ operational arena (or designated GCS staging corridor).
3. **Altitude Ceiling Compliance**: Asserts that all UAVs observe the $100.0\,\text{m}$ maximum operational ceiling.

Results are written automatically upon simulation completion to:
`data/webots_spatial_verification.txt`
