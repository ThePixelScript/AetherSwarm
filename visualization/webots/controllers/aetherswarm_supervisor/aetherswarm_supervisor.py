#!/usr/bin/env python3
"""Authoritative trace consumer and 3D spatial verification Supervisor for Webots R2025a.

Consumes immutable AetherSwarm simulation traces, manifests the authoritative
multi-UAV spatial execution, updates 3D mesh communication links, and performs
independent runtime spatial constraint verification (minimum separation, geofence,
altitude compliance).

Constraint:
  - Uses ONLY standard library modules and the Webots 'controller' API.
  - Zero imports from ares_swarm, numpy, networkx, scipy, or WSL virtualenv.
  - Purely observational spatial verification; never overrides authoritative state.
"""
from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

# Webots Controller API
try:
    from controller import Supervisor
except ImportError:
    # If running outside Webots for syntax checking / dry-run
    Supervisor = None


LOG_FILE = Path(__file__).resolve().parent / ".." / ".." / "data" / "webots_execution.log"


def log_msg(msg: str) -> None:
    """Log to stdout and persistent file."""
    print(msg)
    try:
        LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write(msg + "\n")
    except Exception:
        pass


def find_trace_file(supervisor: Any = None) -> Path:
    """Locate authoritative trace JSON deterministically from explicit selector.

    Priority:
      1. Direct environment path: AETHERSWARM_TRACE_PATH or WEBOTS_TRACE_PATH
      2. Scenario name from env: AETHERSWARM_SCENARIO ("e1" or "recovery")
      3. Command-line argument: sys.argv[1] ("e1", "recovery", or direct path)
      4. Robot customData field: supervisor.getCustomData() ("e1", "recovery")
      5. Canonical default: e1_authoritative_trace.json (official benchmark)
    """
    base_dir = Path(__file__).resolve().parent
    data_dir = (base_dir / ".." / ".." / "data").resolve()
    if not data_dir.is_dir():
        data_dir = Path("visualization/webots/data").resolve()

    # 1. Direct path from environment
    env_path = os.environ.get("AETHERSWARM_TRACE_PATH") or os.environ.get("WEBOTS_TRACE_PATH")
    if env_path:
        p = Path(env_path)
        if p.is_file():
            log_msg(f"[Webots Supervisor] Selected trace from environment path: {p}")
            return p

    # Determine explicit scenario selector (e1 vs recovery)
    selector = ""
    env_scen = os.environ.get("AETHERSWARM_SCENARIO")
    if env_scen:
        selector = env_scen.strip().lower()

    if not selector and len(sys.argv) > 1:
        selector = sys.argv[1].strip().lower()

    if not selector and supervisor is not None:
        try:
            custom_data = supervisor.getCustomData()
            if custom_data:
                selector = custom_data.strip().lower()
        except Exception:
            pass

    # 2. Map selector to known authoritative trace files
    if "recovery" in selector:
        target = data_dir / "recovery_authoritative_trace.json"
        if target.is_file():
            log_msg(f"[Webots Supervisor] Selected recovery trace (selector='{selector}'): {target}")
            return target

    if "e1" in selector:
        target = data_dir / "e1_authoritative_trace.json"
        if target.is_file():
            log_msg(f"[Webots Supervisor] Selected E1 trace (selector='{selector}'): {target}")
            return target

    # 3. Deterministic canonical default: official E1 benchmark trace
    default_e1 = data_dir / "e1_authoritative_trace.json"
    if default_e1.is_file():
        log_msg(f"[Webots Supervisor] Defaulting deterministically to official E1 trace: {default_e1}")
        return default_e1

    # Fallback to recovery trace if E1 is absent
    recovery = data_dir / "recovery_authoritative_trace.json"
    if recovery.is_file():
        log_msg(f"[Webots Supervisor] E1 trace not found; selecting recovery trace: {recovery}")
        return recovery

    raise FileNotFoundError(
        f"Authoritative trace JSON not found in {data_dir}. "
        "Run scripts/export_webots_trace.py to generate authoritative traces."
    )


class WebotsAetherSwarmSupervisor:
    """Consumes authoritative trace and drives Webots 3D robotic simulation."""

    def __init__(self) -> None:
        if Supervisor is None:
            raise RuntimeError("Webots Supervisor API is not available.")

        self.supervisor = Supervisor()
        self.time_step = int(self.supervisor.getBasicTimeStep())
        if self.time_step <= 0:
            self.time_step = 32

        self.trace_file = find_trace_file(self.supervisor)
        log_msg(f"[Webots Supervisor] Loading authoritative trace: {self.trace_file}")
        with open(self.trace_file, "r", encoding="utf-8") as f:
            self.trace_data = json.load(f)

        self.metadata = self.trace_data.get("metadata", {})
        self.ticks = self.trace_data.get("ticks", [])
        self.total_ticks = len(self.ticks)
        self.min_separation_m = float(self.metadata.get("min_separation_m", 20.0))
        self.max_altitude_m = float(self.metadata.get("max_altitude", 100.0))
        self.gcs_pos = self.metadata.get("gcs_position", [-50.0, 500.0, 0.0])

        log_msg(f"[Webots Supervisor] Trace scenario: {self.metadata.get('scenario_name')}")
        log_msg(f"[Webots Supervisor] Total simulation ticks: {self.total_ticks}")
        log_msg(f"[Webots Supervisor] Official constraints: min_sep={self.min_separation_m}m, max_alt={self.max_altitude_m}m")

        # Lookup drone nodes and material fields
        self.drone_nodes: dict[str, Any] = {}
        self.drone_trans_fields: dict[str, Any] = {}
        self.drone_rot_fields: dict[str, Any] = {}
        self.drone_materials: dict[str, Any] = {}
        self.drone_beacons: dict[str, Any] = {}

        for uid in self.metadata.get("uav_ids", []):
            def_name = f"UAV_{uid.split('_')[-1]}"  # e.g. uav_1 -> UAV_1
            node = self.supervisor.getFromDef(def_name)
            if node:
                self.drone_nodes[uid] = node
                self.drone_trans_fields[uid] = node.getField("translation")
                self.drone_rot_fields[uid] = node.getField("rotation")

                mat_node = self.supervisor.getFromDef(f"{def_name}_MAT")
                if mat_node:
                    self.drone_materials[uid] = mat_node
                beacon_node = self.supervisor.getFromDef(f"{def_name}_BEACON")
                if beacon_node:
                    self.drone_beacons[uid] = beacon_node

        # Lookup POI nodes and materials
        self.poi_nodes: dict[str, Any] = {}
        self.poi_materials: dict[str, Any] = {}
        self.poi_beacons: dict[str, Any] = {}

        for tid in self.metadata.get("task_ids", []):
            parts = tid.split("_")
            def_name = f"POI_{parts[-1].upper()}" if len(parts) > 1 else f"POI_{tid.upper()}"
            node = self.supervisor.getFromDef(def_name)
            if node:
                self.poi_nodes[tid] = node
                mat_node = self.supervisor.getFromDef(f"{def_name}_MAT")
                if mat_node:
                    self.poi_materials[tid] = mat_node
                beacon_node = self.supervisor.getFromDef(f"{def_name}_BEACON")
                if beacon_node:
                    self.poi_beacons[tid] = beacon_node

        # Communication mesh lines nodes
        self.comm_coord_field = None
        self.comm_index_field = None
        coord_node = self.supervisor.getFromDef("COMM_COORD")
        lines_node = self.supervisor.getFromDef("COMM_LINES")
        if coord_node and lines_node:
            self.comm_coord_field = coord_node.getField("point")
            self.comm_index_field = lines_node.getField("coordIndex")

        # Independent spatial verification metrics
        self.min_observed_separation = float("inf")
        self.separation_violations = 0
        self.geofence_violations = 0
        self.altitude_violations = 0
        self.verification_log: list[str] = []
        # State caches for minimizing synchronous IPC overhead
        self._drone_state_cache: dict[str, tuple[str, str, bool]] = {}
        self._poi_state_cache: dict[str, str] = {}
        self._comm_topology_cache: list[tuple[str, str]] = []

    def update_drone_appearance(self, uav_id: str, state_dict: dict[str, Any]) -> None:
        """Update visible drone status colors strictly from authoritative state."""
        failure = state_dict.get("failure_state", "NORMAL")
        rth = state_dict.get("rth_state", "NONE")
        active = state_dict.get("active", True)
        curr_state = (failure, rth, active)

        if self._drone_state_cache.get(uav_id) == curr_state:
            return
        self._drone_state_cache[uav_id] = curr_state

        mat = self.drone_materials.get(uav_id)
        beacon = self.drone_beacons.get(uav_id)
        if not mat:
            return

        diff_field = mat.getField("diffuseColor")
        emis_field = mat.getField("emissiveColor")

        if failure == "FAILED" or not active:
            # Visually mark failed: dark crimson body, glowing warning
            diff_field.setSFColor([0.85, 0.05, 0.05])
            emis_field.setSFColor([0.5, 0.0, 0.0])
            if beacon:
                beacon.getField("diffuseColor").setSFColor([0.9, 0.0, 0.0])
                beacon.getField("emissiveColor").setSFColor([0.8, 0.0, 0.0])
        elif rth in ("ACTIVE", "COMPLETE"):
            # RTH returning state: royal blue
            diff_field.setSFColor([0.1, 0.4, 0.95])
            emis_field.setSFColor([0.1, 0.2, 0.6])
            if beacon:
                beacon.getField("diffuseColor").setSFColor([0.2, 0.6, 1.0])
                beacon.getField("emissiveColor").setSFColor([0.3, 0.7, 1.0])
        else:
            # Normal operational state
            diff_field.setSFColor([0.1, 0.7, 0.25])
            emis_field.setSFColor([0.05, 0.25, 0.1])
            if beacon:
                beacon.getField("diffuseColor").setSFColor([0.1, 0.8, 0.3])
                beacon.getField("emissiveColor").setSFColor([0.1, 0.8, 0.3])

    def update_poi_appearance(self, task_id: str, state_dict: dict[str, Any]) -> None:
        """Update visible POI status beacons strictly from authoritative state."""
        status = state_dict.get("status", "PENDING")
        if self._poi_state_cache.get(task_id) == status:
            return
        self._poi_state_cache[task_id] = status

        mat = self.poi_materials.get(task_id)
        beacon = self.poi_beacons.get(task_id)
        if not mat:
            return

        diff_field = mat.getField("diffuseColor")
        emis_field = mat.getField("emissiveColor")

        if status == "COMPLETE":
            # Completed: Emerald green
            diff_field.setSFColor([0.15, 0.80, 0.25])
            emis_field.setSFColor([0.2, 0.6, 0.2])
            if beacon:
                beacon.getField("diffuseColor").setSFColor([0.15, 0.85, 0.25])
                beacon.getField("emissiveColor").setSFColor([0.2, 0.7, 0.2])
        elif status == "IN_PROGRESS":
            # Active service: Vibrant orange
            diff_field.setSFColor([0.95, 0.55, 0.05])
            emis_field.setSFColor([0.5, 0.25, 0.0])
            if beacon:
                beacon.getField("diffuseColor").setSFColor([1.0, 0.6, 0.0])
                beacon.getField("emissiveColor").setSFColor([0.8, 0.4, 0.0])
        elif status == "DEFERRED":
            # Deferred: Warning amber/red
            diff_field.setSFColor([0.85, 0.2, 0.1])
            emis_field.setSFColor([0.6, 0.1, 0.0])
            if beacon:
                beacon.getField("diffuseColor").setSFColor([0.9, 0.2, 0.1])
                beacon.getField("emissiveColor").setSFColor([0.8, 0.1, 0.0])
        else:
            # Pending / Assigned: Gold
            diff_field.setSFColor([0.85, 0.70, 0.10])
            emis_field.setSFColor([0.2, 0.15, 0.0])
            if beacon:
                beacon.getField("diffuseColor").setSFColor([0.85, 0.75, 0.1])
                beacon.getField("emissiveColor").setSFColor([0.5, 0.4, 0.05])

    def update_comm_mesh(self, active_links: list[dict[str, Any]], uav_positions: dict[str, list[float]]) -> None:
        """Render active RF communication links via native Webots IndexedLineSet."""
        if not self.comm_coord_field or not self.comm_index_field:
            return

        points: list[list[float]] = []
        indices: list[int] = []
        topology: list[tuple[str, str]] = []

        gcs_coords = [self.gcs_pos[0], self.gcs_pos[1], 1.0]

        for link in active_links:
            src = link["source"]
            tgt = link["target"]
            topology.append((src, tgt))

            src_pos = gcs_coords if src == "gcs" else uav_positions.get(src)
            tgt_pos = gcs_coords if tgt == "gcs" else uav_positions.get(tgt)

            if src_pos and tgt_pos:
                p1_idx = len(points)
                p2_idx = p1_idx + 1
                points.append(src_pos)
                points.append(tgt_pos)
                indices.extend([p1_idx, p2_idx, -1])

        # If topology changed or initial, rebuild line set structure
        if topology != self._comm_topology_cache:
            self._comm_topology_cache = topology
            while self.comm_coord_field.getCount() > 0:
                self.comm_coord_field.removeMF(-1)
            while self.comm_index_field.getCount() > 0:
                self.comm_index_field.removeMF(-1)

            for p in points:
                self.comm_coord_field.insertMFVec3f(-1, p)
            for idx in indices:
                self.comm_index_field.insertMFInt32(-1, idx)
        else:
            # Same topology: simply update vertex coordinates in place
            for i, p in enumerate(points):
                if i < self.comm_coord_field.getCount():
                    self.comm_coord_field.setMFVec3f(i, p)

    def perform_spatial_verification(self, tick: int, sim_time: float, active_positions: dict[str, list[float]]) -> None:
        """Independent observational spatial constraint verification directly from 3D coordinates."""
        uids = sorted(active_positions.keys())

        # 1. Pairwise inter-UAV separation verification
        for i in range(len(uids)):
            for j in range(i + 1, len(uids)):
                u1 = uids[i]
                u2 = uids[j]
                p1 = active_positions[u1]
                p2 = active_positions[u2]

                dx = p1[0] - p2[0]
                dy = p1[1] - p2[1]
                dz = p1[2] - p2[2]
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)

                if dist < self.min_observed_separation:
                    self.min_observed_separation = dist

                if dist < (self.min_separation_m - 0.01):
                    self.separation_violations += 1
                    msg = f"Tick {tick} ({sim_time}s): SEPARATION VIOLATION between {u1} and {u2}: {dist:.2f}m < {self.min_separation_m}m"
                    self.verification_log.append(msg)
                    print(f"  [Webots Spatial Warning] {msg}")

        # 2. Geofence & Altitude Compliance
        for uid, pos in active_positions.items():
            x, y, z = pos
            # Altitude check
            if z > self.max_altitude_m + 0.5:
                self.altitude_violations += 1
                msg = f"Tick {tick} ({sim_time}s): ALTITUDE VIOLATION for {uid}: {z:.2f}m > {self.max_altitude_m}m"
                self.verification_log.append(msg)
                print(f"  [Webots Spatial Warning] {msg}")

            # Operational area boundary check (1000m x 1000m arena; allow designated GCS corridor [-65, 0] x [400, 600])
            in_arena = (0.0 <= x <= 1000.0) and (0.0 <= y <= 1000.0)
            in_gcs_corridor = (-65.0 <= x <= 5.0) and (400.0 <= y <= 600.0)
            if not (in_arena or in_gcs_corridor):
                self.geofence_violations += 1
                msg = f"Tick {tick} ({sim_time}s): GEOFENCE VIOLATION for {uid} at ({x:.1f}, {y:.1f})"
                self.verification_log.append(msg)
                print(f"  [Webots Spatial Warning] {msg}")

    def run(self) -> None:
        """Execute playback loop driven by authoritative trace steps."""
        log_msg("[Webots Supervisor] Commencing simulation playback and spatial verification...")

        # Switch to Fast simulation mode for high-throughput playback
        try:
            self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_FAST)
        except Exception:
            pass

        tick_idx = 0
        screenshot_dir = Path(__file__).resolve().parent / ".." / ".." / "data" / "screenshots"
        screenshot_dir.mkdir(parents=True, exist_ok=True)

        while self.supervisor.step(self.time_step) != -1:
            if tick_idx >= self.total_ticks:
                log_msg("[Webots Supervisor] Reached final trace tick.")
                break

            step = self.ticks[tick_idx]
            sim_tick = step["tick"]
            sim_time = step["time"]
            uavs = step["uavs"]
            tasks = step["tasks"]
            network = step.get("network", {})
            events = step.get("events", [])

            # Print significant domain events
            for ev in events:
                ev_type = ev.get("type")
                if ev_type in ("UAV_FAILED", "TASK_DEFERRED", "TASK_ASSIGNED", "TASK_COMPLETED", "RTH_TRIGGERED", "UAV_LANDED"):
                    log_msg(f"  [Authoritative Event @ Tick {sim_tick} ({sim_time}s)] {ev_type} -> {ev.get('entity_id')} payload={ev.get('payload')}")

            # 1. Update UAV spatial translations, yaw rotations, and appearances
            current_active_positions: dict[str, list[float]] = {}
            for uid, u_state in uavs.items():
                pos = u_state["position"]
                yaw = u_state.get("yaw", 0.0)

                # Set 3D position and orientation
                trans_field = self.drone_trans_fields.get(uid)
                if trans_field:
                    trans_field.setSFVec3f(pos)

                rot_field = self.drone_rot_fields.get(uid)
                if rot_field:
                    rot_field.setSFRotation([0.0, 0.0, 1.0, yaw])

                self.update_drone_appearance(uid, u_state)

                if u_state.get("active", True) and u_state.get("failure_state") != "FAILED":
                    current_active_positions[uid] = pos

            # 2. Update POI task appearances
            for tid, t_state in tasks.items():
                self.update_poi_appearance(tid, t_state)

            # 3. Update communication mesh links
            self.update_comm_mesh(network.get("active_links", []), current_active_positions)

            # 4. Perform independent observational spatial verification
            self.perform_spatial_verification(sim_tick, sim_time, current_active_positions)

            # Capture key demonstration screenshots
            if sim_tick in (0, 8, 21, 300, 305):
                shot_path = screenshot_dir / f"webots_tick_{sim_tick}.png"
                try:
                    self.supervisor.exportImage(str(shot_path), 95)
                except Exception:
                    pass

            if sim_tick % 50 == 0 or sim_tick == self.total_ticks - 1:
                log_msg(f"  [Webots Playback] Progress: tick {sim_tick}/{self.total_ticks} ({sim_time}s)")

            tick_idx += 1

        # Final Spatial Verification Report
        self.output_verification_report()

        # Gracefully quit simulation when finished
        self.supervisor.simulationQuit(0)

    def output_verification_report(self) -> None:
        """Output summary of independent spatial verification."""
        header = "=" * 80
        log_msg("\n" + header)
        log_msg("WEBOTS INDEPENDENT 3D SPATIAL VERIFICATION REPORT")
        log_msg(header)
        log_msg(f"Scenario:                   {self.metadata.get('scenario_name')}")
        log_msg(f"Verified Ticks:             {self.total_ticks}")
        log_msg(f"Minimum Observed Separation:{self.min_observed_separation:.2f}m (Constraint: >= {self.min_separation_m}m)")
        log_msg(f"Separation Violations:      {self.separation_violations}")
        log_msg(f"Geofence Violations:        {self.geofence_violations} (Area: 1000m x 1000m)")
        log_msg(f"Altitude Violations:        {self.altitude_violations} (Ceiling: <= {self.max_altitude_m}m)")
        log_msg(header)

        report_file = Path(__file__).resolve().parent / ".." / ".." / "data" / "webots_spatial_verification.txt"
        report_file.parent.mkdir(parents=True, exist_ok=True)
        with open(report_file, "w", encoding="utf-8") as f:
            f.write("WEBOTS INDEPENDENT 3D SPATIAL VERIFICATION REPORT\n")
            f.write(f"Scenario: {self.metadata.get('scenario_name')}\n")
            f.write(f"Verified Ticks: {self.total_ticks}\n")
            f.write(f"Minimum Observed Separation: {self.min_observed_separation:.2f}m\n")
            f.write(f"Separation Violations: {self.separation_violations}\n")
            f.write(f"Geofence Violations: {self.geofence_violations}\n")
            f.write(f"Altitude Violations: {self.altitude_violations}\n")
            if self.verification_log:
                f.write("\nViolations Log:\n")
                f.write("\n".join(self.verification_log[:50]) + "\n")
        log_msg(f"Verification report saved to: {report_file}")


def main() -> None:
    try:
        import traceback
        supervisor = WebotsAetherSwarmSupervisor()
        supervisor.run()
    except Exception as e:
        import traceback
        log_msg(f"[Webots Supervisor Exception] {e}\n{traceback.format_exc()}")


if __name__ == "__main__":
    main()
