# AetherSwarm Webots R2025a 3D Robotics Layer

## Overview
This directory contains the downstream 3D robotics simulation and independent spatial verification layer for **AetherSwarm** using Cyberbotics Webots R2025a.

> [!IMPORTANT]
> **Authoritative Architecture**: Headless AetherSwarm (`ares_swarm`) running in Linux/WSL remains the single authoritative source of truth for all mission state, A1 task allocation, Gamma RF communications analysis, battery/energy dynamics, hardware failure/recovery logic, and official benchmark metrics. Webots acts purely as an observational downstream consumer and independent spatial verifier; it does not simulate autonomy, communication protocols, or physics overrides.

---

## Directory Structure
```text
visualization/webots/
├── README.md                                    # This document
├── worlds/
│   └── uavx_round1.wbt                         # 1000m x 1000m ENU coordinate arena with GCS and POIs
├── controllers/
│   └── aetherswarm_supervisor/
│       └── aetherswarm_supervisor.py           # In-world Python Supervisor controller
├── protos/
│   ├── SwarmDrone.proto                        # 3D quadrotor drone model with status lighting
│   └── TaskPOI.proto                           # Point-of-Interest ground marker and status pylon
└── data/
    ├── .gitkeep
    ├── e1_authoritative_trace.json             # Official E1 benchmark trace (2,700 ticks)
    ├── recovery_authoritative_trace.json       # In-flight failure & recovery demo trace (100 ticks)
    └── webots_spatial_verification.txt         # Independent spatial verification report output
```

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

## Trace Generation (WSL)
Authoritative traces are produced deterministically from AetherSwarm:
```bash
# In WSL:
cd /home/dell/swarm_ws/AetherSwarm
.venv/bin/python scripts/export_webots_trace.py --scenario e1
.venv/bin/python scripts/export_webots_trace.py --scenario recovery
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

---

## Independent Spatial Verification
During simulation playback, the Supervisor independently monitors and records 3D physical coordinates at every tick, verifying:
1. **Inter-UAV Minimum Separation**: Asserts that all active UAV pairs maintain distance $\ge 20.0\,\text{m}$.
2. **Geofence Boundary Compliance**: Asserts that all UAVs remain strictly within the $1000\times 1000\,\text{m}$ operational arena (or designated GCS staging corridor).
3. **Altitude Ceiling Compliance**: Asserts that all UAVs observe the $100.0\,\text{m}$ maximum operational ceiling.

Results are written automatically upon simulation completion to:
`data/webots_spatial_verification.txt`
