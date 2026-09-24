"""Connectivity-Aware Mission Planning Layer (V1) for AetherSwarm.

Provides deterministic communication and endurance feasibility checks before task
assignment and dynamically monitors active task connectivity, triggering relay
deployment and communication-induced replanning when necessary.
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

from ..core.commands import (
    AssignRelayRoleCommand,
    AssignTaskCommand,
    Command,
    ReleaseTaskCommand,
    SetTargetPositionCommand,
    StartRTHCommand,
)
from ..core.enums import FailureState, Role, RTHState, SortieState, TaskStatus
from ..core.models import StateSnapshot, TaskState, UAVState
from ..interfaces.communication import NetworkAnalysis
from .a1_allocator import A1TaskAllocator
from .relay_manager import ChainStatus, DynamicRelayManager, RelayChain


def compute_corridor_path(
    gcs_position: Tuple[float, float],
    target_position: Tuple[float, float],
    corridor_bounds_y: Tuple[float, float] = (450.0, 550.0),
    margin_m: float = 1.0,
) -> List[Tuple[float, float]]:
    """Return piecewise linear path waypoints [start, (Portal), target] avoiding corridor boundary clipping."""
    x_start, y_start = gcs_position
    x_end, y_end = target_position

    # 1. Crossing x = 0 (GCS <-> Arena)
    if (x_start < 0.0 and x_end >= 0.0) or (x_start >= 0.0 and x_end < 0.0):
        denom = x_end - x_start
        if abs(denom) > 1e-9:
            y_cross = y_start + ((0.0 - x_start) / denom) * (y_end - y_start)
            y_min_c = corridor_bounds_y[0] + margin_m
            y_max_c = corridor_bounds_y[1] - margin_m
            if y_cross < y_min_c or y_cross > y_max_c:
                return [gcs_position, (0.0, 500.0), target_position]

    # 2. Staged near corridor mouth in arena (x < 50, y < 450 or y > 550)
    y_min_c = corridor_bounds_y[0] + margin_m
    y_max_c = corridor_bounds_y[1] - margin_m
    if 0.0 <= x_start < 50.0 and (y_start < y_min_c or y_start > y_max_c):
        return [gcs_position, (50.0, 500.0), target_position]

    return [gcs_position, target_position]


def get_path_point_at_distance(
    pts: List[Tuple[float, float]],
    target_distance: float,
) -> Tuple[float, float]:
    """Return 2D point at cumulative path distance target_distance along pts."""
    if not pts:
        return (0.0, 0.0)
    if len(pts) == 1 or target_distance <= 0.0:
        return pts[0]

    total_dist = 0.0
    seg_lengths = []
    for i in range(len(pts) - 1):
        seg_len = math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        seg_lengths.append(seg_len)
        total_dist += seg_len

    if target_distance >= total_dist:
        return pts[-1]

    accum_d = 0.0
    for j in range(len(seg_lengths)):
        seg_len = seg_lengths[j]
        if accum_d + seg_len >= target_distance - 1e-9:
            rem_d = target_distance - accum_d
            frac = rem_d / seg_len if seg_len > 1e-9 else 0.0
            p0 = pts[j]
            p1 = pts[j + 1]
            sx = round(p0[0] + frac * (p1[0] - p0[0]), 2)
            sy = round(p0[1] + frac * (p1[1] - p0[1]), 2)
            return (sx, sy)
        accum_d += seg_len

    return pts[-1]


def compute_multihop_stations(
    gcs_position: Tuple[float, float],
    target_position: Tuple[float, float],
    effective_range: float = 95.0,
    corridor_bounds_y: Tuple[float, float] = (450.0, 550.0),
    margin_m: float = 1.0,
) -> Tuple[int, int, Tuple[Tuple[float, float], ...]]:
    """Compute minimum hops, intermediate relays, and equal-spaced relay stations (Step 1).

    Corridor-Aware Multi-Hop Geometry (Fix 2):
    - Determines whether direct GCS->POI path is fully valid under composite geofence.
    - If valid: retains direct-path geometry.
    - If invalid: uses piecewise path GCS -> Portal(0, 500) -> POI.
    - Places relay stations by DISTANCE ALONG PATH, ensuring every station lies
      inside legal airspace and every hop <= effective_range.
    """
    pts = compute_corridor_path(gcs_position, target_position, corridor_bounds_y, margin_m)

    total_dist = 0.0
    seg_lengths = []
    for i in range(len(pts) - 1):
        seg_len = math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        seg_lengths.append(seg_len)
        total_dist += seg_len

    if total_dist <= effective_range:
        return 1, 0, ()

    h_min = max(1, math.ceil(total_dist / effective_range))
    k_min = max(0, h_min - 1)

    stations: List[Tuple[float, float]] = []
    step_d = total_dist / h_min

    for i in range(1, k_min + 1):
        target_d = i * step_d
        st_pos = get_path_point_at_distance(pts, target_d)
        stations.append(st_pos)

    return h_min, k_min, tuple(stations)


@dataclass(frozen=True)
class ConnectivityFeasibilityResult:
    """Detailed result of a connectivity & endurance feasibility check."""
    feasible: bool
    reason: str
    relay_needed: bool = False
    relay_uav_id: Optional[str] = None
    relay_position: Optional[Tuple[float, float]] = None
    estimated_return_time_s: float = 0.0
    estimated_total_energy_wh: float = 0.0
    # Phase 5 multi-hop extensions
    relay_uav_ids: Tuple[str, ...] = ()
    relay_positions: Tuple[Tuple[float, float], ...] = ()
    hop_count: int = 1
    min_relay_count: int = 0


@dataclass(frozen=True)
class ConnectivityAwarePlannerConfig:
    """Configuration for connectivity-aware mission planning."""
    enabled: bool = True
    comm_range_m: float = 100.0
    effective_range_factor: float = 0.95  # Conservative 95m effective boundary
    max_sortie_duration_s: float = 1200.0
    speed_limit: float = 5.0
    idle_rate: float = 1.0
    movement_rate: float = 0.5
    rth_safety_margin_s: float = 15.0
    min_battery_reserve_wh: float = 15.0
    enforce_sortie_limit: bool = True
    disconnected_replan_tolerance_s: float = 10.0
    enable_multihop_chains: bool = True
    max_chain_relays: int = 12


class ConnectivityAwarePlanner:
    """Authoritative Connectivity-Aware Mission Planner with Multi-Hop Chain Support.

    Integrates:
    - Pre-assignment connectivity feasibility checking with multi-hop equal-spacing chains
    - Dynamic relay requirement evaluation & assignment via DynamicRelayManager
    - Round-trip endurance and 1200s sortie limit verification
    - Active task connectivity monitoring, localized link handoffs, and communication-induced replanning
    """

    def __init__(
        self,
        config: Optional[ConnectivityAwarePlannerConfig] = None,
        allocator: Optional[A1TaskAllocator] = None,
        relay_manager: Optional[DynamicRelayManager] = None,
        comm_range: float = 100.0,
        speed_limit: float = 5.0,
        idle_rate: float = 1.0,
        movement_rate: float = 0.5,
        max_sortie_duration_s: float = 1200.0,
        enforce_sortie_limit: bool = True,
    ) -> None:
        if config is not None:
            self.config = config
        else:
            self.config = ConnectivityAwarePlannerConfig(
                comm_range_m=comm_range,
                speed_limit=speed_limit,
                idle_rate=idle_rate,
                movement_rate=movement_rate,
                max_sortie_duration_s=max_sortie_duration_s,
                enforce_sortie_limit=enforce_sortie_limit,
            )

        self.allocator = allocator or A1TaskAllocator()
        self.relay_manager = relay_manager or DynamicRelayManager()

        # Authoritative Phase 4 metrics
        self.connectivity_feasibility_checks: int = 0
        self.connectivity_feasible_assignments: int = 0
        self.connectivity_rejected_assignments: int = 0
        self.connectivity_deferred_tasks: int = 0
        self.relay_required_for_assignment: int = 0
        self.connectivity_preserved_during_task: float = 0.0
        self.communication_induced_replans: int = 0

        # Phase 5 metrics
        self.tasks_deferred_insufficient_relays: int = 0
        self.max_hop_count: int = 0

        # Disconnection tracking per active surveyor
        self._disconnected_time: Dict[str, float] = {}

    def reset(self) -> None:
        """Reset all planner metrics and active tracking state."""
        self.connectivity_feasibility_checks = 0
        self.connectivity_feasible_assignments = 0
        self.connectivity_rejected_assignments = 0
        self.connectivity_deferred_tasks = 0
        self.relay_required_for_assignment = 0
        self.connectivity_preserved_during_task = 0.0
        self.communication_induced_replans = 0
        self.tasks_deferred_insufficient_relays = 0
        self.max_hop_count = 0
        self._disconnected_time.clear()

    def check_task_connectivity_feasibility(
        self,
        task: TaskState,
        uav: UAVState,
        snapshot: StateSnapshot,
        network_analysis: Optional[NetworkAnalysis] = None,
        flight_records: Optional[Dict[str, Any]] = None,
        exclude_uav_ids: Optional[Set[str]] = None,
    ) -> ConnectivityFeasibilityResult:
        """Evaluate communication and endurance feasibility for (task, candidate_uav)."""
        gcs = snapshot.gcs_position
        v_max = max(1.0, self.config.speed_limit)
        idle = self.config.idle_rate
        mov = self.config.movement_rate
        comm_limit = self.config.comm_range_m * self.config.effective_range_factor

        # 1. Endurance & Sortie Limit Feasibility
        d_transit_to = math.hypot(uav.position_xy[0] - task.position_xy[0], uav.position_xy[1] - task.position_xy[1])
        t_transit_to = d_transit_to / v_max

        pts_poi = compute_corridor_path(gcs, task.position_xy)
        d_return = sum(math.hypot(pts_poi[i + 1][0] - pts_poi[i][0], pts_poi[i + 1][1] - pts_poi[i][1]) for i in range(len(pts_poi) - 1))
        t_transit_ret = d_return / v_max

        t_service = max(1.0, task.service_duration)
        t_total_mission = t_transit_to + t_service + t_transit_ret

        # Sortie duration constraint
        if self.config.enforce_sortie_limit:
            current_airborne_s = 0.0
            if flight_records and uav.id in flight_records:
                rec = flight_records[uav.id]
                if getattr(rec, "is_airborne", False):
                    current_airborne_s = getattr(rec, "current_sortie_duration_s", 0.0)
            remaining_sortie_s = max(0.0, self.config.max_sortie_duration_s - current_airborne_s)
            required_sortie_s = t_total_mission + self.config.rth_safety_margin_s
            if remaining_sortie_s < required_sortie_s:
                return ConnectivityFeasibilityResult(
                    feasible=False,
                    reason=f"Insufficient remaining sortie budget ({remaining_sortie_s:.1f}s < {required_sortie_s:.1f}s)",
                )

        # Battery energy constraint
        e_transit_to = mov * d_transit_to + idle * t_transit_to
        e_service = idle * t_service
        e_transit_ret = mov * d_return + idle * t_transit_ret
        e_total_req = (e_transit_to + e_service + e_transit_ret) * 1.2

        if uav.battery_energy < (e_total_req + self.config.min_battery_reserve_wh):
            return ConnectivityFeasibilityResult(
                feasible=False,
                reason=f"Insufficient battery energy ({uav.battery_energy:.1f}Wh < {e_total_req + self.config.min_battery_reserve_wh:.1f}Wh)",
            )

        # 2. Connectivity Feasibility at POI Location
        d_gcs = d_return  # Distance from POI to GCS

        # Case 2A: Direct Line-of-Sight to GCS
        if d_gcs <= comm_limit:
            return ConnectivityFeasibilityResult(
                feasible=True,
                reason="Direct link to GCS feasible",
                relay_needed=False,
                estimated_return_time_s=t_transit_ret,
                estimated_total_energy_wh=e_total_req,
                hop_count=1,
                min_relay_count=0,
            )

        # Case 2B: Existing Active Relay Covers POI
        active_relays = [
            r for r in snapshot.uavs.values()
            if r.role == Role.RELAY and r.active and r.failure_state == FailureState.NORMAL and r.rth_state == RTHState.NONE
        ]
        for relay in active_relays:
            if network_analysis and relay.id in network_analysis.connected_uav_ids:
                d_relay_poi = math.hypot(relay.position_xy[0] - task.position_xy[0], relay.position_xy[1] - task.position_xy[1])
                if d_relay_poi <= comm_limit:
                    return ConnectivityFeasibilityResult(
                        feasible=True,
                        reason=f"Existing relay {relay.id} covers POI",
                        relay_needed=False,
                        relay_uav_id=relay.id,
                        relay_position=relay.position_xy,
                        relay_uav_ids=(relay.id,),
                        relay_positions=(relay.position_xy,),
                        estimated_return_time_s=t_transit_ret,
                        estimated_total_energy_wh=e_total_req,
                        hop_count=2,
                        min_relay_count=1,
                    )

        # Case 2C: Multi-Hop Intermediate Relay Chain Deployment (Phase 5B baseline: equal spacing)
        if not self.config.enable_multihop_chains:
            # Single-relay fallback: midpoint between GCS and POI
            r_pos = (
                round(gcs[0] + 0.5 * (task.position_xy[0] - gcs[0]), 2),
                round(gcs[1] + 0.5 * (task.position_xy[1] - gcs[1]), 2),
            )
            d_gcs_relay = math.hypot(r_pos[0] - gcs[0], r_pos[1] - gcs[1])
            d_relay_poi = math.hypot(task.position_xy[0] - r_pos[0], task.position_xy[1] - r_pos[1])
            if d_gcs_relay > comm_limit or d_relay_poi > comm_limit:
                return ConnectivityFeasibilityResult(
                    feasible=False,
                    reason=f"POI distance {d_gcs:.1f}m exceeds single-relay coverage ({comm_limit * 2:.1f}m)",
                )

        h_min, k_min, stations = compute_multihop_stations(gcs, task.position_xy, effective_range=comm_limit)

        if k_min > self.config.max_chain_relays:
            return ConnectivityFeasibilityResult(
                feasible=False,
                reason=f"Required relay count {k_min} exceeds maximum allowed chain length ({self.config.max_chain_relays})",
                relay_needed=True,
                relay_positions=stations,
                hop_count=h_min,
                min_relay_count=k_min,
            )

        excluded = set(exclude_uav_ids or set())
        excluded.add(uav.id)

        cand_ids = self.relay_manager.select_relay_chain_candidates(
            snapshot=snapshot,
            surveyor_id=uav.id,
            stations=stations,
            network_analysis=network_analysis,
            exclude_uav_ids=excluded,
            speed_limit=v_max,
            idle_rate=idle,
            movement_rate=mov,
            max_sortie_s=self.config.max_sortie_duration_s,
            enforce_sortie_limit=self.config.enforce_sortie_limit,
            flight_records=flight_records,
        )

        if cand_ids is not None:
            # First relay is closest to GCS (R_1), last is closest to surveyor (R_K)
            first_relay_id = cand_ids[0] if cand_ids else None
            first_station = stations[0] if stations else None
            return ConnectivityFeasibilityResult(
                feasible=True,
                reason=f"Relay chain of {k_min} UAVs can be deployed to {len(stations)} stations",
                relay_needed=(k_min > 0),
                relay_uav_id=first_relay_id,
                relay_position=first_station,
                relay_uav_ids=tuple(cand_ids),
                relay_positions=stations,
                estimated_return_time_s=t_transit_ret,
                estimated_total_energy_wh=e_total_req,
                hop_count=h_min,
                min_relay_count=k_min,
            )

        return ConnectivityFeasibilityResult(
            feasible=False,
            reason=f"Required {k_min}-relay chain exceeds available eligible candidates (insufficient relays)",
            relay_needed=(k_min > 0),
            relay_positions=stations,
            hop_count=h_min,
            min_relay_count=k_min,
        )

    def monitor_active_tasks(
        self,
        snapshot: StateSnapshot,
        network_analysis: Optional[NetworkAnalysis] = None,
        flight_records: Optional[Dict[str, Any]] = None,
        dt: float = 1.0,
    ) -> List[Command]:
        """Monitor currently servicing UAVs and trigger replanning if communication is lost."""
        commands: List[Command] = []
        tick = snapshot.simulation_tick
        connected_ids = set(network_analysis.connected_uav_ids) if network_analysis else set()

        active_surveyors = [
            u for u in snapshot.uavs.values()
            if u.active and u.assigned_task_id is not None and u.role in (Role.SURVEYOR, Role.SCOUT)
        ]

        for surv in sorted(active_surveyors, key=lambda x: x.id):
            task = snapshot.tasks.get(surv.assigned_task_id)
            if not task:
                continue

            is_connected = (surv.id in connected_ids)
            if is_connected:
                self.connectivity_preserved_during_task += dt
                self._disconnected_time.pop(surv.id, None)
            else:
                self._disconnected_time[surv.id] = self._disconnected_time.get(surv.id, 0.0) + dt

            # Inspect assigned relay/chain status
            designated_relay_id = self.relay_manager.surveyor_to_relay.get(surv.id)
            chain_id = self.relay_manager.surveyor_to_chain.get(surv.id)
            chain = self.relay_manager.chains.get(chain_id) if chain_id else None

            # Fix 1: Evaluate chain readiness for pre-detection holding & release
            chain_ready = True
            if chain:
                relays_in_position = True
                for rid, st_pos in zip(chain.relay_ids, chain.station_positions):
                    ruav = snapshot.uavs.get(rid)
                    if not ruav or not ruav.active or math.hypot(ruav.position_xy[0] - st_pos[0], ruav.position_xy[1] - st_pos[1]) > 5.0:
                        relays_in_position = False
                        break

                has_route = bool(network_analysis and network_analysis.routes_to_gcs.get(surv.id) is not None)

                if relays_in_position and has_route:
                    if chain.status == ChainStatus.FORMING:
                        chain.status = ChainStatus.ACTIVE
                    chain_ready = True
                elif chain.status in (ChainStatus.FORMING, ChainStatus.HANDOFF):
                    chain_ready = False

            relay_lost_unrecovered = False
            if chain:
                if chain.status == ChainStatus.DEGRADED:
                    relay_lost_unrecovered = True
            elif designated_relay_id:
                relay_uav = snapshot.uavs.get(designated_relay_id)
                if not relay_uav or not relay_uav.active or relay_uav.failure_state != FailureState.NORMAL:
                    relay_lost_unrecovered = True
                elif relay_uav.rth_state != RTHState.NONE or relay_uav.sortie_state in (SortieState.RTH, SortieState.LANDING, SortieState.LANDED):
                    # Relay is returning, verify if handoff succeeded
                    if relay_uav.role != Role.RELAY:
                        relay_lost_unrecovered = True

            # Trigger replanning if relay was lost without replacement or disconnection exceeds tolerance
            disconnect_duration = self._disconnected_time.get(surv.id, 0.0)
            trigger_replan = relay_lost_unrecovered or (
                disconnect_duration >= self.config.disconnected_replan_tolerance_s
                and math.hypot(surv.position_xy[0] - snapshot.gcs_position[0], surv.position_xy[1] - snapshot.gcs_position[1]) > self.config.comm_range_m
            )

            if trigger_replan:
                # Abort task, defer back to allocator, command safe return towards GCS
                commands.append(
                    ReleaseTaskCommand(
                        source_tick=tick,
                        uav_id=surv.id,
                        task_id=task.id,
                        reason="COMMUNICATION_LOSS_CHAIN_SEVERED",
                    )
                )
                pts_ret = compute_corridor_path(surv.position_xy, snapshot.gcs_position)
                desired_gcs_target = (0.0, 500.0) if (surv.position_xy[0] >= 0.0 and len(pts_ret) > 2) else snapshot.gcs_position
                commands.append(
                    SetTargetPositionCommand(
                        source_tick=tick,
                        uav_id=surv.id,
                        target_position=desired_gcs_target,
                        speed=self.config.speed_limit,
                    )
                )
                self.communication_induced_replans += 1
                self._disconnected_time.pop(surv.id, None)

                # Tear down surviving chain components to prevent orphan relays
                if chain_id:
                    self.relay_manager.teardown_chain(chain_id, commands=commands, tick=tick)
            elif chain and not chain_ready:
                # Fix 1: Pre-detection holding control while chain is forming
                pts = compute_corridor_path(snapshot.gcs_position, task.position_xy)
                total_d = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1))
                hold_d = max(0.0, total_d - 45.0)
                p_hold = get_path_point_at_distance(pts, hold_d)

                desired_target = (0.0, 500.0) if (surv.position_xy[0] < -1.0 and len(pts) > 2) else p_hold

                if surv.target_position != desired_target:
                    commands.append(
                        SetTargetPositionCommand(
                            source_tick=tick,
                            uav_id=surv.id,
                            target_position=desired_target,
                            speed=self.config.speed_limit,
                        )
                    )
            else:
                # Chain is operational or no chain needed: release surveyor toward task position
                pts = compute_corridor_path(snapshot.gcs_position, task.position_xy)
                if surv.position_xy[0] < -1.0 and len(pts) > 2:
                    if surv.target_position != (0.0, 500.0):
                        commands.append(
                            SetTargetPositionCommand(
                                source_tick=tick,
                                uav_id=surv.id,
                                target_position=(0.0, 500.0),
                                speed=self.config.speed_limit,
                            )
                        )
                else:
                    if surv.target_position != task.position_xy:
                        commands.append(
                            SetTargetPositionCommand(
                                source_tick=tick,
                                uav_id=surv.id,
                                target_position=task.position_xy,
                                speed=self.config.speed_limit,
                            )
                        )

        return commands

    def plan(
        self,
        snapshot: StateSnapshot,
        network_analysis: Optional[NetworkAnalysis] = None,
        flight_records: Optional[Dict[str, Any]] = None,
    ) -> List[Command]:
        """Execute connectivity-aware task allocation and relay deployment pass."""
        if not self.config.enabled:
            return []

        commands: List[Command] = []
        tick = snapshot.simulation_tick
        sim_time = snapshot.simulation_time

        # Track UAVs assigned during this planning tick
        assigned_uav_ids: Set[str] = set()

        # Sort tasks deterministically: priority descending, emergency first, id ascending
        sorted_tasks = sorted(
            [t for t in snapshot.tasks.values() if t.status in (TaskStatus.PENDING, TaskStatus.DEFERRED)],
            key=lambda t: (-t.priority, not getattr(t, "emergency_flag", False), t.id),
        )

        for task in sorted_tasks:
            # Candidates are idle/ready UAVs not yet assigned in this planning tick
            remaining_candidates = [
                u for u in snapshot.uavs.values()
                if u.active
                and u.rth_state == RTHState.NONE
                and u.failure_state == FailureState.NORMAL
                and u.sortie_state in (SortieState.READY, SortieState.ACTIVE)
                and u.id not in assigned_uav_ids
                and u.assigned_task_id is None
                and u.role != Role.RELAY
            ]

            if not remaining_candidates:
                self.connectivity_deferred_tasks += 1
                continue

            feasible_proposals: List[Tuple[float, UAVState, ConnectivityFeasibilityResult]] = []
            rejections_for_task: List[ConnectivityFeasibilityResult] = []

            for cand_uav in remaining_candidates:
                self.connectivity_feasibility_checks += 1
                res = self.check_task_connectivity_feasibility(
                    task=task,
                    uav=cand_uav,
                    snapshot=snapshot,
                    network_analysis=network_analysis,
                    flight_records=flight_records,
                    exclude_uav_ids=assigned_uav_ids,
                )
                if res.feasible:
                    utility = self.allocator.compute_utility(
                        cand_uav, task, network_analysis=network_analysis, snapshot=snapshot
                    )
                    feasible_proposals.append((utility.total, cand_uav, res))
                else:
                    self.connectivity_rejected_assignments += 1
                    rejections_for_task.append(res)

            if not feasible_proposals:
                self.connectivity_deferred_tasks += 1
                if any("insufficient" in str(r.reason).lower() for r in rejections_for_task):
                    self.tasks_deferred_insufficient_relays += 1
                continue

            # Deterministic tie-breaking: descending score, ascending UAV ID
            feasible_proposals.sort(key=lambda item: (-round(item[0], 8), item[1].id))
            _, best_uav, best_res = feasible_proposals[0]

            # Count one feasible assignment per task actually assigned (not per candidate evaluated)
            self.connectivity_feasible_assignments += 1

            # Deploy relay chain if required (Step 4)
            if best_res.relay_needed:
                relay_ids = list(best_res.relay_uav_ids) if best_res.relay_uav_ids else ([best_res.relay_uav_id] if best_res.relay_uav_id else [])
                station_positions = list(best_res.relay_positions) if best_res.relay_positions else ([best_res.relay_position] if best_res.relay_position else [])

                for r_id, r_pos in zip(relay_ids, station_positions):
                    commands.append(
                        AssignRelayRoleCommand(
                            source_tick=tick,
                            uav_id=r_id,
                            target_position=r_pos,
                            relay_for_uav_id=best_uav.id,
                        )
                    )
                    commands.append(
                        SetTargetPositionCommand(
                            source_tick=tick,
                            uav_id=r_id,
                            target_position=r_pos,
                            speed=self.config.speed_limit,
                        )
                    )
                    assigned_uav_ids.add(r_id)

                if relay_ids and station_positions:
                    chain = RelayChain(
                        chain_id=f"chain_{task.id}",
                        task_id=task.id,
                        surveyor_id=best_uav.id,
                        relay_ids=relay_ids,
                        station_positions=station_positions,
                        created_tick=tick,
                        created_time=sim_time,
                        status=ChainStatus.FORMING,
                    )
                    self.relay_manager.register_chain(chain)

                self.relay_required_for_assignment += 1
                if best_res.hop_count > self.max_hop_count:
                    self.max_hop_count = best_res.hop_count

            # Assign task to surveyor
            commands.append(
                AssignTaskCommand(
                    source_tick=tick,
                    uav_id=best_uav.id,
                    task_id=task.id,
                )
            )
            assigned_uav_ids.add(best_uav.id)

            # Fix 1: Initial target position setup for pre-detection holding if relay chain is required
            if best_res.relay_needed:
                pts = compute_corridor_path(snapshot.gcs_position, task.position_xy)
                total_d = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1))
                hold_d = max(0.0, total_d - 45.0)
                p_hold = get_path_point_at_distance(pts, hold_d)

                desired_target = (0.0, 500.0) if (best_uav.position_xy[0] < -1.0 and len(pts) > 2) else p_hold

                commands.append(
                    SetTargetPositionCommand(
                        source_tick=tick,
                        uav_id=best_uav.id,
                        target_position=desired_target,
                        speed=self.config.speed_limit,
                    )
                )
            else:
                pts = compute_corridor_path(snapshot.gcs_position, task.position_xy)
                if best_uav.position_xy[0] < -1.0 and len(pts) > 2:
                    commands.append(
                        SetTargetPositionCommand(
                            source_tick=tick,
                            uav_id=best_uav.id,
                            target_position=(0.0, 500.0),
                            speed=self.config.speed_limit,
                        )
                    )

        return commands
