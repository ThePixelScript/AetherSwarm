#!/usr/bin/env python3
"""Authoritative trace consumer and 3D spatial verification Supervisor for Webots R2025a.

Consumes immutable AetherSwarm simulation traces, manifests the authoritative
multi-UAV spatial execution, updates 3D mesh communication links, renders dynamic
altitude drop-lines, updates high-visibility status indicators, displays in-world HUD telemetry,
and performs independent runtime spatial constraint verification (minimum separation, geofence,
altitude compliance).

Constraint:
  - Uses ONLY standard library modules and the Webots 'controller' API.
  - Zero imports from ares_swarm, numpy, networkx, scipy, or WSL virtualenv.
  - Purely observational spatial verification; never overrides authoritative state.
  - Dual-mode: drives Webots 3D robotic simulation when inside Webots; provides
    independent deterministic spatial verification CLI when executed standalone.
"""
from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path
from typing import Any

# Webots Controller API
try:
    from controller import Supervisor
except ImportError:
    # Running outside Webots for CLI verification / syntax checking / dry-run
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

    # Direct path from argv
    if len(sys.argv) > 1:
        arg_p = Path(sys.argv[1])
        if arg_p.is_file():
            log_msg(f"[Webots Supervisor] Selected trace from argv path: {arg_p.resolve()}")
            return arg_p.resolve()

    # Determine explicit scenario selector (e1 vs recovery vs random/demo)
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
    if "random" in selector or "demo" in selector:
        target = data_dir / "random_demo_trace.json"
        if target.is_file():
            log_msg(f"[Webots Supervisor] Selected randomized demo trace (selector='{selector}'): {target}")
            return target

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
        self.is_standalone = Supervisor is None
        if not self.is_standalone:
            self.supervisor = Supervisor()
            self.time_step = int(self.supervisor.getBasicTimeStep())
            if self.time_step <= 0:
                self.time_step = 32
        else:
            self.supervisor = None
            self.time_step = 32
            log_msg("[Webots Supervisor] Running in standalone spatial verification mode (controller API not imported)")

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

        # Lookup drone nodes and material fields if running inside Webots
        self.drone_nodes: dict[str, Any] = {}
        self.drone_trans_fields: dict[str, Any] = {}
        self.drone_rot_fields: dict[str, Any] = {}
        self.drone_materials: dict[str, Any] = {}
        self.drone_beacons: dict[str, Any] = {}
        self.drone_halos: dict[str, Any] = {}

        if self.supervisor:
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
                    halo_node = self.supervisor.getFromDef(f"{def_name}_HALO")
                    if halo_node:
                        self.drone_halos[uid] = halo_node

        # Lookup POI nodes and materials
        self.poi_nodes: dict[str, Any] = {}
        self.poi_trans_fields: dict[str, Any] = {}
        self.poi_materials: dict[str, Any] = {}
        self.poi_beacons: dict[str, Any] = {}

        if self.supervisor:
            for tid in self.metadata.get("task_ids", []):
                parts = tid.split("_")
                def_name = f"POI_{parts[-1].upper()}" if len(parts) > 1 else f"POI_{tid.upper()}"
                node = self.supervisor.getFromDef(def_name)
                if node:
                    self.poi_nodes[tid] = node
                    self.poi_trans_fields[tid] = node.getField("translation")
                    mat_node = self.supervisor.getFromDef(f"{def_name}_MAT")
                    if mat_node:
                        self.poi_materials[tid] = mat_node
                    beacon_node = self.supervisor.getFromDef(f"{def_name}_BEACON")
                    if beacon_node:
                        self.poi_beacons[tid] = beacon_node

            # Set 3D POI world positions from trace to match authoritative task coordinates
            initial_tasks = self.ticks[0].get("tasks", {}) if self.ticks else {}
            for tid, trans_field in self.poi_trans_fields.items():
                if tid in initial_tasks and "position" in initial_tasks[tid] and trans_field:
                    t_pos = initial_tasks[tid]["position"]
                    trans_field.setSFVec3f([float(t_pos[0]), float(t_pos[1]), 0.0])

        # Communication mesh lines nodes
        self.comm_coord_field = None
        self.comm_index_field = None
        self.route_coord_field = None
        self.route_index_field = None
        self.dropline_coord_field = None
        self.dropline_index_field = None

        if self.supervisor:
            coord_node = self.supervisor.getFromDef("COMM_COORD")
            lines_node = self.supervisor.getFromDef("COMM_LINES")
            if coord_node and lines_node:
                self.comm_coord_field = coord_node.getField("point")
                self.comm_index_field = lines_node.getField("coordIndex")

            r_coord_node = self.supervisor.getFromDef("ROUTE_COORD")
            r_lines_node = self.supervisor.getFromDef("ROUTE_LINES")
            if r_coord_node and r_lines_node:
                self.route_coord_field = r_coord_node.getField("point")
                self.route_index_field = r_lines_node.getField("coordIndex")

            d_coord_node = self.supervisor.getFromDef("DROPLINE_COORD")
            d_lines_node = self.supervisor.getFromDef("DROPLINE_LINES")
            if d_coord_node and d_lines_node:
                self.dropline_coord_field = d_coord_node.getField("point")
                self.dropline_index_field = d_lines_node.getField("coordIndex")

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
        self._route_topology_cache: list[tuple[str, str]] = []
        self._last_event_banner: str = ""

    def update_drone_appearance(self, uav_id: str, state_dict: dict[str, Any]) -> None:
        """Update visible drone status colors strictly from authoritative state.

        States:
          - normal/active: clear nominal indicator (emerald beacon, tech cyan body)
          - FAILED: obvious red/crimson indicator (warning crimson body & radiant red beacon)
          - RTH: amber/yellow indicator (luminous amber/gold body & beacon)
          - LANDED: subdued indicator (slate grey body & dim standby beacon)
        """
        if not self.supervisor:
            return

        failure = state_dict.get("failure_state", "NORMAL")
        rth = state_dict.get("rth_state", "NONE")
        active = state_dict.get("active", True)
        curr_state = (failure, rth, active)

        if self._drone_state_cache.get(uav_id) == curr_state:
            return
        self._drone_state_cache[uav_id] = curr_state

        mat = self.drone_materials.get(uav_id)
        beacon = self.drone_beacons.get(uav_id)
        halo = self.drone_halos.get(uav_id)
        if not mat:
            return

        diff_field = mat.getField("diffuseColor")
        emis_field = mat.getField("emissiveColor")

        if failure == "FAILED" or not active:
            # Obvious Red/Crimson Indicator for Hardware Failure (Preserves Authoritative Position)
            diff_field.setSFColor([0.90, 0.08, 0.08])
            emis_field.setSFColor([0.70, 0.02, 0.02])
            if beacon:
                beacon.getField("diffuseColor").setSFColor([1.0, 0.0, 0.0])
                beacon.getField("emissiveColor").setSFColor([1.0, 0.1, 0.1])
            if halo:
                halo.getField("diffuseColor").setSFColor([1.0, 0.0, 0.0])
                halo.getField("emissiveColor").setSFColor([0.8, 0.0, 0.0])
        elif rth == "COMPLETE":
            # Landed State: Subdued Indicator
            diff_field.setSFColor([0.35, 0.38, 0.42])
            emis_field.setSFColor([0.06, 0.07, 0.09])
            if beacon:
                beacon.getField("diffuseColor").setSFColor([0.30, 0.30, 0.30])
                beacon.getField("emissiveColor").setSFColor([0.08, 0.08, 0.08])
            if halo:
                halo.getField("diffuseColor").setSFColor([0.35, 0.38, 0.42])
                halo.getField("emissiveColor").setSFColor([0.05, 0.05, 0.05])
        elif rth == "ACTIVE":
            # RTH Returning State: Amber / Yellow Indicator
            diff_field.setSFColor([0.95, 0.70, 0.05])
            emis_field.setSFColor([0.50, 0.32, 0.02])
            if beacon:
                beacon.getField("diffuseColor").setSFColor([1.0, 0.80, 0.10])
                beacon.getField("emissiveColor").setSFColor([0.95, 0.70, 0.05])
            if halo:
                halo.getField("diffuseColor").setSFColor([1.0, 0.80, 0.10])
                halo.getField("emissiveColor").setSFColor([0.80, 0.50, 0.05])
        else:
            # Normal Operational State: Clear Nominal Indicator
            diff_field.setSFColor([0.10, 0.65, 0.85])
            emis_field.setSFColor([0.05, 0.25, 0.40])
            if beacon:
                beacon.getField("diffuseColor").setSFColor([0.10, 0.85, 0.40])
                beacon.getField("emissiveColor").setSFColor([0.20, 0.90, 0.50])
            if halo:
                halo.getField("diffuseColor").setSFColor([0.10, 0.65, 0.85])
                halo.getField("emissiveColor").setSFColor([0.10, 0.65, 0.85])

    def update_poi_appearance(self, task_id: str, state_dict: dict[str, Any]) -> None:
        """Update visible POI status beacons strictly from authoritative state.

        States:
          - PENDING: Gold indicator
          - IN_PROGRESS: Active cyan/energy blue indicator
          - COMPLETE: Emerald green indicator
          - DEFERRED: Hazard red/coral indicator
        """
        if not self.supervisor:
            return

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
            diff_field.setSFColor([0.15, 0.85, 0.25])
            emis_field.setSFColor([0.25, 0.80, 0.30])
            if beacon:
                beacon.getField("diffuseColor").setSFColor([0.15, 0.90, 0.30])
                beacon.getField("emissiveColor").setSFColor([0.35, 0.95, 0.45])
        elif status == "IN_PROGRESS":
            # Active service: Vibrant energy cyan
            diff_field.setSFColor([0.05, 0.75, 1.00])
            emis_field.setSFColor([0.10, 0.65, 0.95])
            if beacon:
                beacon.getField("diffuseColor").setSFColor([0.05, 0.80, 1.00])
                beacon.getField("emissiveColor").setSFColor([0.30, 0.85, 1.00])
        elif status == "DEFERRED":
            # Deferred: Hazard warning red
            diff_field.setSFColor([0.90, 0.15, 0.15])
            emis_field.setSFColor([0.70, 0.10, 0.10])
            if beacon:
                beacon.getField("diffuseColor").setSFColor([0.95, 0.10, 0.10])
                beacon.getField("emissiveColor").setSFColor([0.85, 0.10, 0.10])
        else:
            # Pending / Assigned: Gold
            diff_field.setSFColor([0.90, 0.75, 0.10])
            emis_field.setSFColor([0.25, 0.18, 0.00])
            if beacon:
                beacon.getField("diffuseColor").setSFColor([0.90, 0.75, 0.10])
                beacon.getField("emissiveColor").setSFColor([0.75, 0.60, 0.05])

    @staticmethod
    def _set_indexed_line_set(
        coord_field: Any,
        index_field: Any,
        points: list[list[float]],
        indices: list[int],
    ) -> None:
        """Safely update Webots IndexedLineSet geometry without out-of-range or empty-node warnings.

        Guarantees:
        1. Zero links / empty state: generates valid Webots geometry (2 coincident underground coordinates,
           paired index [0, 1, -1]) with no invalid indices and zero visible lines.
        2. One coordinate: cannot form a line segment, treated as empty valid geometry.
        3. N coordinates: exactly valid paired indices [0, 1, -1, 2, 3, -1, ...] referencing strictly existing coordinates.
        4. Field update ordering:
           - When expanding points: add/set coordinates first, then set indices.
           - When shrinking points: shrink/set indices first, then remove excess coordinates.
           This strictly prevents out-of-range index evaluations at all times.
        """
        if not coord_field or not index_field:
            return

        # Webots IndexedLineSet requires at least 2 coordinates and at least 2 index items.
        # For empty or single-point states, provide a valid zero-length segment underground.
        if len(points) < 2 or len(indices) < 3:
            points = [[0.0, 0.0, -100.0], [0.0, 0.0, -100.0]]
            indices = [0, 1, -1]

        n_new = len(points)
        n_old = coord_field.getCount()
        m_new = len(indices)
        m_old = index_field.getCount()

        if n_new >= n_old:
            # 1. Update existing coordinates in-place
            for i in range(n_old):
                coord_field.setMFVec3f(i, points[i])
            # 2. Append additional coordinates
            for i in range(n_old, n_new):
                coord_field.insertMFVec3f(-1, points[i])

            # 3. Update indices (coord_field now has n_new >= max(indices)+1)
            for i in range(min(m_old, m_new)):
                index_field.setMFInt32(i, indices[i])
            if m_new > m_old:
                for i in range(m_old, m_new):
                    index_field.insertMFInt32(-1, indices[i])
            elif m_new < m_old:
                for _ in range(m_old - m_new):
                    index_field.removeMF(-1)
        else:
            # 1. Shrink indices first so no index >= n_new remains
            for i in range(min(m_old, m_new)):
                index_field.setMFInt32(i, indices[i])
            if m_new > m_old:
                for i in range(m_old, m_new):
                    index_field.insertMFInt32(-1, indices[i])
            elif m_new < m_old:
                for _ in range(m_old - m_new):
                    index_field.removeMF(-1)

            # 2. Update existing coordinates in-place
            for i in range(n_new):
                coord_field.setMFVec3f(i, points[i])
            # 3. Remove excess coordinates (safe since index_field only references < n_new)
            for _ in range(n_old - n_new):
                coord_field.removeMF(-1)

    def update_comm_mesh(
        self,
        active_links: list[dict[str, Any]],
        routes_to_gcs: dict[str, Any],
        uav_positions: dict[str, list[float]],
    ) -> None:
        """Render active RF communication links and active routes to GCS via native Webots IndexedLineSets."""
        if not self.comm_coord_field or not self.comm_index_field:
            return

        gcs_coords = [self.gcs_pos[0], self.gcs_pos[1], 1.2]

        # 1. Base Mesh Links (all authoritative RF links)
        points: list[list[float]] = []
        indices: list[int] = []
        topology: list[tuple[str, str]] = []

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

        # If topology or vertex count changed, update structure safely
        if topology != self._comm_topology_cache or self.comm_coord_field.getCount() != len(points):
            self._comm_topology_cache = topology
            self._set_indexed_line_set(self.comm_coord_field, self.comm_index_field, points, indices)
        else:
            # Same topology: update vertex coordinates in place
            for i, p in enumerate(points):
                self.comm_coord_field.setMFVec3f(i, p)

        # 2. Active Multihop Routes to GCS (Distinct route emphasis without inventing roles)
        if self.route_coord_field and self.route_index_field:
            r_points: list[list[float]] = []
            r_indices: list[int] = []
            r_topology: list[tuple[str, str]] = []
            seen_edges: set[tuple[str, str]] = set()

            if routes_to_gcs:
                for _uid, route in routes_to_gcs.items():
                    if route and len(route) >= 2:
                        for k in range(len(route) - 1):
                            hop_a = route[k]
                            hop_b = route[k + 1]
                            edge = (min(hop_a, hop_b), max(hop_a, hop_b))
                            if edge not in seen_edges:
                                seen_edges.add(edge)
                                r_topology.append(edge)
                                pa = gcs_coords if hop_a == "gcs" else uav_positions.get(hop_a)
                                pb = gcs_coords if hop_b == "gcs" else uav_positions.get(hop_b)
                                if pa and pb:
                                    # Slight Z elevation for route line to avoid Z-fighting
                                    pa_elev = [pa[0], pa[1], pa[2] + 0.15]
                                    pb_elev = [pb[0], pb[1], pb[2] + 0.15]
                                    idx1 = len(r_points)
                                    idx2 = idx1 + 1
                                    r_points.append(pa_elev)
                                    r_points.append(pb_elev)
                                    r_indices.extend([idx1, idx2, -1])

            if r_topology != self._route_topology_cache or self.route_coord_field.getCount() != len(r_points):
                self._route_topology_cache = r_topology
                self._set_indexed_line_set(self.route_coord_field, self.route_index_field, r_points, r_indices)
            else:
                for i, p in enumerate(r_points):
                    self.route_coord_field.setMFVec3f(i, p)

    def update_drop_lines(self, uav_positions: dict[str, list[float]]) -> None:
        """Render vertical ground drop-lines and ground footprints for intuitive altitude readability."""
        if not self.dropline_coord_field or not self.dropline_index_field:
            return

        d_points: list[list[float]] = []
        d_indices: list[int] = []

        for _uid, pos in sorted(uav_positions.items()):
            x, y, z = pos
            # Vertical altitude plumb-line from UAV to ground plane
            idx_top = len(d_points)
            idx_bot = idx_top + 1
            d_points.append([x, y, z])
            d_points.append([x, y, 0.06])
            d_indices.extend([idx_top, idx_bot, -1])

            # Ground crosshair footprint (4m span at ground level)
            g_idx = len(d_points)
            d_points.append([x - 1.8, y, 0.06])
            d_points.append([x + 1.8, y, 0.06])
            d_points.append([x, y - 1.8, 0.06])
            d_points.append([x, y + 1.8, 0.06])
            d_indices.extend([g_idx, g_idx + 1, -1, g_idx + 2, g_idx + 3, -1])

        # Rebuild or update drop line points safely
        if self.dropline_coord_field.getCount() != len(d_points):
            self._set_indexed_line_set(self.dropline_coord_field, self.dropline_index_field, d_points, d_indices)
        else:
            for i, p in enumerate(d_points):
                self.dropline_coord_field.setMFVec3f(i, p)

    def update_hud(
        self,
        sim_tick: int,
        sim_time: float,
        uavs: dict[str, Any],
        tasks: dict[str, Any],
        events: list[dict[str, Any]],
    ) -> None:
        """Lightweight native in-world HUD telemetry display via Supervisor.setLabel.

        Zero external dependencies or IPC; purely uses standard Webots overlay mechanism.
        """
        if not self.supervisor:
            return

        scenario = self.metadata.get("scenario_name", "AetherSwarm")
        active_count = sum(1 for u in uavs.values() if u.get("active", True) and u.get("failure_state") != "FAILED" and u.get("rth_state") != "COMPLETE")
        failed_count = sum(1 for u in uavs.values() if u.get("failure_state") == "FAILED")
        rth_count = sum(1 for u in uavs.values() if u.get("rth_state") == "ACTIVE")
        landed_count = sum(1 for u in uavs.values() if u.get("rth_state") == "COMPLETE")
        completed_tasks = sum(1 for t in tasks.values() if t.get("status") == "COMPLETE")
        in_prog_tasks = sum(1 for t in tasks.values() if t.get("status") == "IN_PROGRESS")
        total_tasks = len(tasks)

        # Check for important domain events to display on banner
        for ev in events:
            ev_type = ev.get("type", "")
            if ev_type == "UAV_FAILED":
                self._last_event_banner = f"ALERT: Hardware Failure on {ev.get('entity_id')} ({ev.get('payload', {}).get('reason', '')})"
            elif ev_type == "TASK_ASSIGNED":
                self._last_event_banner = f"ALLOC: Task {ev.get('payload', {}).get('task_id')} -> {ev.get('entity_id')}"
            elif ev_type == "TASK_COMPLETED":
                self._last_event_banner = f"TASK COMPLETE: {ev.get('entity_id')} by {ev.get('payload', {}).get('uav_id')}"
            elif ev_type == "RTH_TRIGGERED":
                self._last_event_banner = f"NAV: RTH Triggered for {ev.get('entity_id')}"

        # 1. Header Banner
        self.supervisor.setLabel(0, "AETHERSWARM UAV-X RESEARCH DEMONSTRATION", 0.015, 0.015, 0.045, 0xFFFFFF, 0.0, "Arial")

        # 2. Playback & Time
        time_text = f"Scenario: {scenario}  |  Tick: {sim_tick}/{self.total_ticks}  ({sim_time:.1f}s)"
        self.supervisor.setLabel(1, time_text, 0.015, 0.050, 0.038, 0x00D0FF, 0.0, "Arial")

        # 3. Swarm Status
        swarm_color = 0xFF4444 if failed_count > 0 else 0x44FF88
        swarm_text = f"Swarm: {active_count} Active  |  {failed_count} Failed  |  {rth_count} RTH  |  {landed_count} Landed"
        self.supervisor.setLabel(2, swarm_text, 0.015, 0.080, 0.038, swarm_color, 0.0, "Arial")

        # 4. POI Task Completion
        task_text = f"Tasks: {completed_tasks}/{total_tasks} Completed  ({in_prog_tasks} In Progress)"
        self.supervisor.setLabel(3, task_text, 0.015, 0.110, 0.038, 0xFFDD33, 0.0, "Arial")

        # 5. Independent Spatial Verifier Metric
        min_sep_text = f"Spatial Verifier: Min Separation: {self.min_observed_separation:.2f}m (Constraint: >= {self.min_separation_m}m)"
        self.supervisor.setLabel(4, min_sep_text, 0.015, 0.140, 0.038, 0x55FFBB, 0.0, "Arial")

        # 6. Top-Center Event Alert Banner
        if self._last_event_banner:
            banner_color = 0xFF3333 if "ALERT" in self._last_event_banner else 0x33DDFF
            self.supervisor.setLabel(5, self._last_event_banner, 0.28, 0.015, 0.042, banner_color, 0.0, "Arial")

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

        # Switch to Fast simulation mode for high-throughput playback if inside Webots
        if self.supervisor:
            try:
                self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_FAST)
            except Exception:
                pass

        screenshot_dir = Path(__file__).resolve().parent / ".." / ".." / "data" / "screenshots"
        if self.supervisor:
            screenshot_dir.mkdir(parents=True, exist_ok=True)

        tick_idx = 0

        # Loop function supporting both Webots supervisor step and standalone step
        def advance_step() -> bool:
            nonlocal tick_idx
            if self.supervisor:
                return self.supervisor.step(self.time_step) != -1
            else:
                return tick_idx < self.total_ticks

        while advance_step():
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
            all_drone_positions: dict[str, list[float]] = {}

            for uid, u_state in uavs.items():
                pos = u_state["position"]
                yaw = u_state.get("yaw", 0.0)
                all_drone_positions[uid] = pos

                # Set 3D position and orientation if in Webots
                if self.supervisor:
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
            if self.supervisor:
                for tid, t_state in tasks.items():
                    self.update_poi_appearance(tid, t_state)

                # 3. Update communication mesh links and active multihop routes
                self.update_comm_mesh(
                    network.get("active_links", []),
                    network.get("routes_to_gcs", {}),
                    all_drone_positions,
                )

                # 4. Update dynamic altitude drop-lines
                self.update_drop_lines(all_drone_positions)

                # 5. Update in-world HUD
                self.update_hud(sim_tick, sim_time, uavs, tasks, events)

            # 6. Perform independent observational spatial verification
            self.perform_spatial_verification(sim_tick, sim_time, current_active_positions)

            # Capture key demonstration screenshots
            if self.supervisor and sim_tick in (0, 8, 21, 300, 305):
                shot_path = screenshot_dir / f"webots_tick_{sim_tick}.png"
                try:
                    self.supervisor.exportImage(str(shot_path), 95)
                except Exception:
                    pass

            if sim_tick % 100 == 0 or sim_tick == self.total_ticks - 1:
                log_msg(f"  [Webots Playback] Progress: tick {sim_tick}/{self.total_ticks} ({sim_time:.1f}s)")

            tick_idx += 1

        # Final Spatial Verification Report
        self.output_verification_report()

        # Gracefully quit simulation when finished if running inside Webots
        if self.supervisor:
            try:
                self.supervisor.simulationQuit(0)
            except Exception:
                pass

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
