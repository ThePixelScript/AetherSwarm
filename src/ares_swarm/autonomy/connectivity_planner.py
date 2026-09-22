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
from .relay_manager import DynamicRelayManager


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


class ConnectivityAwarePlanner:
    """Authoritative V1 Connectivity-Aware Mission Planner.

    Integrates:
    - Pre-assignment connectivity feasibility checking
    - Dynamic relay requirement evaluation & assignment via DynamicRelayManager
    - Round-trip endurance and 1200s sortie limit verification
    - Active task connectivity monitoring and communication-induced replanning
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
        """Evaluate whether assigning task to candidate UAV is feasible under communication & endurance."""
        gcs = snapshot.gcs_position
        comm_limit = self.config.comm_range_m * self.config.effective_range_factor
        v_max = max(1.0, self.config.speed_limit)
        idle = self.config.idle_rate
        mov = self.config.movement_rate

        # 1. Safe Return & Endurance Constraints
        d_to_task = math.hypot(uav.position_xy[0] - task.position_xy[0], uav.position_xy[1] - task.position_xy[1])
        d_return = math.hypot(task.position_xy[0] - gcs[0], task.position_xy[1] - gcs[1])
        rem_service_s = max(0.0, float(task.service_duration) - float(getattr(task, "service_progress", 0.0)))

        t_transit_to = d_to_task / v_max
        t_transit_ret = d_return / v_max
        t_req_total = t_transit_to + rem_service_s + t_transit_ret + self.config.rth_safety_margin_s

        # 1A. Sortie Duration Limit Check
        if self.config.enforce_sortie_limit and flight_records and uav.id in flight_records:
            rec = flight_records[uav.id]
            cur_airborne = getattr(rec, "current_sortie_duration_s", 0.0) if getattr(rec, "is_airborne", False) else 0.0
            if cur_airborne + t_req_total > self.config.max_sortie_duration_s:
                return ConnectivityFeasibilityResult(
                    feasible=False,
                    reason=f"Sortie limit exceeded: cur={cur_airborne:.1f}s + req={t_req_total:.1f}s > {self.config.max_sortie_duration_s:.1f}s",
                )

        # 1B. Battery Energy Check
        e_transit = idle * (t_transit_to + t_transit_ret) + mov * (d_to_task + d_return)
        e_service = idle * rem_service_s
        e_total_req = (e_transit + e_service) * 1.2 + self.config.min_battery_reserve_wh
        if uav.battery_energy < e_total_req:
            return ConnectivityFeasibilityResult(
                feasible=False,
                reason=f"Battery insufficient: available={uav.battery_energy:.1f}Wh < req={e_total_req:.1f}Wh",
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
                        estimated_return_time_s=t_transit_ret,
                        estimated_total_energy_wh=e_total_req,
                    )

        # Case 2C: Single Intermediate Relay Deployment
        # Relay midpoint between GCS and POI
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

        # Search for available relay candidate
        excluded = set(exclude_uav_ids or set())
        excluded.add(uav.id)

        cand_id = self.relay_manager.select_relay_candidate(
            snapshot=snapshot,
            target_uav_id=uav.id,
            relay_position=r_pos,
            network_analysis=network_analysis,
            exclude_uav_ids=excluded,
            speed_limit=v_max,
            idle_rate=idle,
            movement_rate=mov,
            max_sortie_s=self.config.max_sortie_duration_s,
            enforce_sortie_limit=self.config.enforce_sortie_limit,
            flight_records=flight_records,
        )

        if cand_id:
            return ConnectivityFeasibilityResult(
                feasible=True,
                reason=f"Relay {cand_id} can be deployed to {r_pos}",
                relay_needed=True,
                relay_uav_id=cand_id,
                relay_position=r_pos,
                estimated_return_time_s=t_transit_ret,
                estimated_total_energy_wh=e_total_req,
            )

        return ConnectivityFeasibilityResult(
            feasible=False,
            reason="No eligible candidate available for required relay station",
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

            # Inspect assigned relay status
            designated_relay_id = self.relay_manager.surveyor_to_relay.get(surv.id)
            relay_lost_unrecovered = False
            if designated_relay_id:
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
                        reason="COMMUNICATION_LOSS_RELAY_UNAVAILABLE",
                    )
                )
                commands.append(
                    SetTargetPositionCommand(
                        source_tick=tick,
                        uav_id=surv.id,
                        target_position=snapshot.gcs_position,
                        speed=self.config.speed_limit,
                    )
                )
                self.communication_induced_replans += 1
                self._disconnected_time.pop(surv.id, None)

        return commands

    def plan(
        self,
        snapshot: StateSnapshot,
        network_analysis: Optional[NetworkAnalysis] = None,
        flight_records: Optional[Dict[str, Any]] = None,
    ) -> List[Command]:
        """Execute connectivity-aware task allocation and relay deployment pass."""
        commands: List[Command] = []
        tick = snapshot.simulation_tick
        sim_time = snapshot.simulation_time

        # 1. Identify visible unassigned tasks
        visible_tasks = [
            t for t in snapshot.tasks.values()
            if t.created_time <= sim_time and t.status in (TaskStatus.PENDING, TaskStatus.DEFERRED)
        ]
        if not visible_tasks:
            return []

        # Deterministic sorting for tasks: descending priority, descending emergency, ascending ID
        visible_tasks.sort(
            key=lambda t: (
                -float(t.priority),
                0 if (getattr(t, "emergency_flag", False) or getattr(t, "is_emergency", False)) else 1,
                t.id,
            )
        )

        # 2. Identify available candidate UAVs
        available_uav_pool: Dict[str, UAVState] = {}
        for u in snapshot.uavs.values():
            ok, _ = self.allocator.is_uav_feasible(u, simulation_time=sim_time)
            if ok:
                available_uav_pool[u.id] = u

        assigned_uav_ids: Set[str] = set()

        # 3. Deterministic greedy allocation with connectivity feasibility gates
        for task in visible_tasks:
            remaining_candidates = [
                u for uid, u in sorted(available_uav_pool.items())
                if uid not in assigned_uav_ids
            ]
            if not remaining_candidates:
                self.connectivity_deferred_tasks += 1
                continue

            feasible_proposals: List[Tuple[float, UAVState, ConnectivityFeasibilityResult]] = []

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

            if not feasible_proposals:
                self.connectivity_deferred_tasks += 1
                continue

            # Deterministic tie-breaking: descending score, ascending UAV ID
            feasible_proposals.sort(key=lambda item: (-round(item[0], 8), item[1].id))
            _, best_uav, best_res = feasible_proposals[0]

            # Count one feasible assignment per task actually assigned (not per candidate evaluated)
            self.connectivity_feasible_assignments += 1

            # Deploy relay if required
            if best_res.relay_needed and best_res.relay_uav_id and best_res.relay_position:
                relay_id = best_res.relay_uav_id
                r_pos = best_res.relay_position

                commands.append(
                    AssignRelayRoleCommand(
                        source_tick=tick,
                        uav_id=relay_id,
                        target_position=r_pos,
                        relay_for_uav_id=best_uav.id,
                    )
                )
                commands.append(
                    SetTargetPositionCommand(
                        source_tick=tick,
                        uav_id=relay_id,
                        target_position=r_pos,
                        speed=self.config.speed_limit,
                    )
                )

                self.relay_manager.surveyor_to_relay[best_uav.id] = relay_id
                self.relay_manager.relay_to_surveyor[relay_id] = best_uav.id
                self.relay_manager.relay_positions[relay_id] = r_pos
                self.relay_manager.relay_assignments += 1
                self.relay_required_for_assignment += 1

                assigned_uav_ids.add(relay_id)

            # Assign task to surveyor
            commands.append(
                AssignTaskCommand(
                    source_tick=tick,
                    uav_id=best_uav.id,
                    task_id=task.id,
                )
            )
            assigned_uav_ids.add(best_uav.id)

        return commands
