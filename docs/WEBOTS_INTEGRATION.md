# Webots R2025a 3D Robotics Layer & Spatial Verification

## 1. Overview & Architectural Role

The 3D robotics simulation layer in [`visualization/webots/`](file:///home/dell/swarm_ws/AetherSwarm/visualization/webots/) provides visual playback, presentation controls, and independent spatial verification for AetherSwarm missions using Cyberbotics Webots R2025a.

```
┌────────────────────────────────────────────────────────────────────────┐
│                   AETHERSWARM CORE (HEADLESS WSL / LINUX)               │
│                                                                        │
│   • Single authoritative source of truth for all flight physics        │
│   • Generates bitwise-reproducible mission traces (.json)              │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │ Unidirectional Read-Only Trace
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│              WEBOTS R2025a 3D ROBOTICS LAYER (WINDOWS / LINUX)         │
│                                                                        │
│   • Ingests immutable simulation trace file                            │
│   • Renders 3D drone airframes, metric ground grid, and RF links       │
│   • Provides interactive playback HUD and camera controls              │
│   • Runs independent spatial verification (separation, geofence)       │
│   • Zero state mutations, physics overrides, or autonomy logic         │
└────────────────────────────────────────────────────────────────────────┘
```

> [!IMPORTANT]
> **Authoritative System Boundary**:
> The headless simulation engine (`src/ares_swarm/`) running under Linux/WSL is the **single authoritative source of truth** for all mission state, autonomy planning, RF link propagation, battery depletion, and benchmark metrics. Webots acts purely as an observational downstream consumer and independent spatial auditor. It executes zero autonomous decisions and introduces no physics overrides.

---

## 2. Directory Layout & Key Assets

```text
visualization/webots/
├── README.md                                    # Operational guide & launch documentation
├── worlds/
│   └── uavx_round1.wbt                         # 1000m x 1000m arena world with GCS apron & viewpoints
├── controllers/
│   └── aetherswarm_supervisor/
│       └── aetherswarm_supervisor.py           # In-world Supervisor controller & dual-mode spatial verifier
├── protos/
│   ├── SwarmDrone.proto                        # Quadrotor drone model with aviation navigation lighting
│   └── TaskPOI.proto                           # Dual-ring POI target marker and status pylon
└── data/
    ├── e1_authoritative_trace.json             # Frozen official E1 benchmark trace (2,700 ticks)
    ├── random_scenario_trace.json              # Randomized working scenario trace (2,700 ticks)
    ├── recovery_authoritative_trace.json       # In-flight failure & recovery scenario trace (100 ticks)
    └── webots_spatial_verification.txt         # Independent spatial verification report output
```

---

## 3. Visual Working Model Features

### 3.1 Metric Ground Grid & Airspace Boundary
- **100m Reference Grid**: Subtle grid lines across the $1000\,\text{m} \times 1000\,\text{m}$ ground plane provide direct visual scale reference.
- **Perimeter Boundary Line**: High-visibility boundary wireframe marking the $1000\,\text{m} \times 1000\,\text{m}$ operational search arena.
- **Operational Ceiling**: Visual wireframe boundary at $100.0\,\text{m}$ altitude with vertical corner warning beacons.

### 3.2 Ground Control Station (GCS)
- Located at the authoritative coordinate: $\mathbf{p}_{\text{gcs}} = [-75.0, 500.0, 0.0]$.
- Features a $38\,\text{m} \times 38\,\text{m}$ heavy operations apron, dual-ring touchdown helipad with "H" markings, tactical command shelter, and a $22\,\text{m}$ communications lattice tower with high-intensity aviation beacon.

### 3.3 Dynamic Drone Airframes (`SwarmDrone.proto`)
- Full quadrotor geometry featuring port (red) and starboard (green) navigation lights and directional nose cone.
- Real-time vertical ground drop-lines and circular ground footprints for unambiguous altitude readability.
- **Authoritative Status Lighting**:
  - `ACTIVE / NOMINAL`: Cyan/teal airframe with emerald green status beacon.
  - `FAILED`: Crimson airframe with brilliant red beacon (held stationary at coordinate without synthetic crash animations).
  - `RTH`: Amber/yellow warning beacon.
  - `LANDED / STANDBY`: Subdued dark slate airframe with dim indicator.

### 3.4 Active RF Mesh Link Overlays
- Dynamic cyan lines rendered between active UAV nodes and GCS whenever physical Euclidean distance satisfies $d \le 100.0\,\text{m}$.
- Mesh lines update synchronously every tick directly from the authoritative communication graph.

---

## 4. Interactive Replay Controls & Camera Viewpoints

The supervisor controller features built-in presentation replay controls and an on-screen HUD:

### 4.1 Keyboard Replay Shortcuts
- `Space`: Toggle Play / Pause.
- `Left Arrow` / `Right Arrow`: Step backward/forward 1 second ($\pm 1$ tick).
- `Shift + Left Arrow` / `Shift + Right Arrow`: Step backward/forward 3 seconds ($\pm 3$ ticks).
- `Home` / `R`: Reset to tick 0.
- `Up Arrow` / `Down Arrow`: Increase / decrease playback speed.
- Keys `1`, `2`, `3`, `4`, `5`: Select speed preset ($0.25\times, 0.5\times, 1.0\times, 2.0\times, 4.0\times$).

### 4.2 Camera Viewpoint Shortcuts
- `O`: Overview perspective (arena-wide high vantage point).
- `F`: Follow active UAV (chase camera tracking active surveyor).
- `G`: GCS launch apron and landing touchdown viewpoint.
- `V`: Recovery zone perspective focusing on dynamic failure intervention.
- `C`: Cycle through all defined camera modes.

### 4.3 Presentation Visibility Toggles
- `H`: Toggle in-world HUD telemetry overlay.
- `M`: Toggle active RF mesh communication lines.
- `D`: Toggle altitude ground drop-lines and shadow footprints.
- `B`: Toggle 100m metric reference ground grid.
- `P` / `T`: Toggle 3D POI target pylons and ground target rings.

---

## 5. Independent Downstream Spatial Verification

While replaying simulation traces, [`aetherswarm_supervisor.py`](file:///home/dell/swarm_ws/AetherSwarm/visualization/webots/controllers/aetherswarm_supervisor/aetherswarm_supervisor.py) independently audits spatial physics at every tick:

1. **Inter-UAV Minimum Separation**: Independently calculates 3D Euclidean distances between all active UAV pairs, verifying:
   $$\Delta r_{\text{3D}} \ge 20.0\,\text{m}$$
2. **Geofence Boundary Compliance**: Audits that all UAV coordinates remain strictly within the composite airspace geometry (Staging Pad, Transit Corridor, or Operational Arena).
3. **Altitude Ceiling Compliance**: Verifies that no vehicle exceeds the $100.0\,\text{m}$ ceiling.

### Output Verification Artifact
Upon scenario completion, results are written automatically to:
`visualization/webots/data/webots_spatial_verification.txt`

Example spatial verification summary:
```text
==================================================
AETHERSWARM INDEPENDENT SPATIAL VERIFICATION
Scenario: e1 (2700 ticks)
--------------------------------------------------
Pairwise Separation Checks Evaluated: 8,421
Separation Violations (< 20.0m):       0 (PASS)
Geofence Ingress/Egress Checks:       13,500
Geofence Boundary Violations:          0 (PASS)
Altitude Ceiling Checks (<= 100.0m):  13,500
Ceiling Violations:                    0 (PASS)
--------------------------------------------------
OVERALL SPATIAL VERIFICATION:         PASSED
==================================================
```

---

## 6. Execution Commands

### Launching Webots GUI on Windows
```powershell
$env:AETHERSWARM_SCENARIO = "random"
Start-Process -FilePath "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" `
  -ArgumentList @("C:\AetherSwarmWebots\worlds\uavx_round1.wbt") `
  -WorkingDirectory "C:\Program Files\Webots"
```

### Standalone Headless Spatial Verification (WSL / Linux CLI)
The supervisor controller can be executed directly from the terminal without opening the Webots graphical interface:
```bash
# Verify official benchmark trace
python visualization/webots/controllers/aetherswarm_supervisor/aetherswarm_supervisor.py e1

# Verify randomized working scenario trace
python visualization/webots/controllers/aetherswarm_supervisor/aetherswarm_supervisor.py random

# Verify in-flight failure recovery trace
python visualization/webots/controllers/aetherswarm_supervisor/aetherswarm_supervisor.py recovery
```
