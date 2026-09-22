"""Dynamic Relay Role Management for AetherSwarm autonomous missions.

Manages dynamic role assignment and transition between SURVEYOR and RELAY,
preemptive handoffs before RTH/recharge, and failure detection/recovery.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple

from ..core.commands import (
    AssignRelayRoleCommand,
    Command,
    HandoffRelayCommand,
    ReleaseRelayRoleCommand,
    SetTargetPositionCommand,
    StartRTHCommand,
)
from ..core.constants import EPSILON
from ..core.enums import FailureState, Role, RTHState, SortieState
from ..core.models import StateSnapshot, UAVState
from ..interfaces.communication import NetworkAnalysis


@dataclass(frozen=True)
class RelayManagementConfig:
    """Configuration for Dynamic Relay Role Management."""
    enabled: bool = True
    communication_range_m: float = 100.0
    min_relay_reserve_energy_wh: float = 50.0
    rth_handoff_margin_s: float = 40.0
    min_operational_time_s: float = 60.0
    relay_placement_ratio: float = 0.5  # Fraction along ray from GCS to target


class DynamicRelayManager:
    """Autonomous dynamic coordinator for swarm relay roles and link continuity."""

    def __init__(self, config: Optional[RelayManagementConfig] = None) -> None:
        self.config = config or RelayManagementConfig()
        # surveyor_id -> relay_id
        self.surveyor_to_relay: Dict[str, str] = {}
        # relay_id -> surveyor_id
        self.relay_to_surveyor: Dict[str, str] = {}
        # relay_id -> target coordinate
        self.relay_positions: Dict[str, Tuple[float, float]] = {}

        # Metrics
        self.relay_assignments: int = 0
        self.relay_releases: int = 0
        self.relay_handoffs: int = 0
        self.relay_losses: int = 0
        self.relay_recovery_successes: int = 0

        self.connected_time_before_handoff: float = 0.0
        self.connected_time_after_handoff: float = 0.0
        self.network_reconfiguration_time_s: Optional[float] = None

        # Tracking state for active handoff / recovery
        self._active_handoff: Optional[Dict[str, Any]] = None
        self._handoff_completed: bool = False

    def reset(self) -> None:
        """Reset internal relay manager state."""
        self.surveyor_to_relay.clear()
        self.relay_to_surveyor.clear()
        self.relay_positions.clear()
        self.relay_assignments = 0
        self.relay_releases = 0
        self.relay_handoffs = 0
        self.relay_losses = 0
        self.relay_recovery_successes = 0
        self.connected_time_before_handoff = 0.0
        self.connected_time_after_handoff = 0.0
        self.network_reconfiguration_time_s = None
        self._active_handoff = None
        self._handoff_completed = False

    def compute_relay_position(
        self,
        gcs_position: Tuple[float, float],
        target_position: Tuple[float, float],
    ) -> Tuple[float, float]:
        """Compute intermediate relay position along line from GCS to target.

        Ensures link distance to GCS is within communication range (e.g. <= 90m).
        """
        dx = target_position[0] - gcs_position[0]
        dy = target_position[1] - gcs_position[1]
        dist = math.hypot(dx, dy)
        if dist <= EPSILON:
            return target_position

        # Max safe hop from GCS is 85.0m (comfortably within 100m range)
        max_hop = min(85.0, self.config.communication_range_m * 0.85)
        step = min(max_hop, dist * self.config.relay_placement_ratio)
        nx = dx / dist
        ny = dy / dist
        return (round(gcs_position[0] + nx * step, 2), round(gcs_position[1] + ny * step, 2))

    def select_relay_candidate(
        self,
        snapshot: StateSnapshot,
        target_uav_id: str,
        relay_position: Tuple[float, float],
        network_analysis: Optional[NetworkAnalysis] = None,
        exclude_uav_ids: Optional[Set[str]] = None,
        speed_limit: float = 5.0,
        idle_rate: float = 1.0,
        movement_rate: float = 0.5,
        max_sortie_s: float = 1200.0,
        enforce_sortie_limit: bool = True,
        flight_records: Optional[Dict[str, Any]] = None,
    ) -> Optional[str]:
        """Dynamically score and select the best candidate UAV for relay role.

        Enforces Requirement 4 & 5:
        - Must NOT be in imminent RTH or recharge.
        - Must have sufficient energy and sortie budget for transit + operational service + return.
        - Considers GCS connectivity, link quality/PDR, route availability, position, role, availability.
        """
        excluded = set(exclude_uav_ids or ())
        excluded.add(target_uav_id)

        candidates = []
        gcs_pos = snapshot.gcs_position

        for u in sorted(snapshot.uavs.values(), key=lambda x: x.id):
            if u.id in excluded:
                continue

            # 1. Operational availability checks
            if not u.active:
                continue
            if u.failure_state != FailureState.NORMAL:
                continue
            if u.rth_state != RTHState.NONE:
                continue
            if u.sortie_state in (SortieState.RTH, SortieState.LANDING, SortieState.LANDED, SortieState.RECHARGING):
                continue

            # 2. Imminent RTH & Endurance Feasibility Gates (Requirement 5)
            d_to_relay = math.hypot(u.position_xy[0] - relay_position[0], u.position_xy[1] - relay_position[1])
            t_to_relay = d_to_relay / max(1.0, speed_limit)

            d_return = math.hypot(relay_position[0] - gcs_pos[0], relay_position[1] - gcs_pos[1])
            t_return = d_return / max(1.0, speed_limit)

            min_op_time = self.config.min_operational_time_s
            required_energy = (
                idle_rate * (t_to_relay + min_op_time + t_return) +
                movement_rate * (d_to_relay + d_return)
            ) * 1.2

            if u.battery_energy < required_energy:
                continue

            if enforce_sortie_limit:
                current_airborne = 0.0
                if flight_records and u.id in flight_records:
                    rec = flight_records[u.id]
                    if getattr(rec, "is_airborne", False):
                        current_airborne = getattr(rec, "current_sortie_duration_s", 0.0)
                remaining_sortie = max(0.0, max_sortie_s - current_airborne)
                required_time = t_to_relay + min_op_time + t_return + 15.0
                if remaining_sortie < required_time:
                    continue

            # 3. Dynamic Utility Scoring (Requirement 4)
            score = 0.0

            # Role & task assignment preference
            if u.role == Role.IDLE:
                score += 100.0
            elif u.role == Role.SURVEYOR:
                if u.assigned_task_id is None:
                    score += 80.0
                else:
                    score += 30.0
            elif u.role == Role.RELAY:
                score += 10.0

            # Distance penalty
            score -= 0.1 * d_to_relay

            # GCS Connectivity & Link Quality
            if network_analysis is not None:
                if u.id in network_analysis.connected_uav_ids:
                    score += 50.0
                route_pdr = network_analysis.route_pdr_to_gcs.get(u.id)
                if route_pdr is not None:
                    score += 25.0 * route_pdr
                hop = network_analysis.hop_counts.get(u.id)
                if hop is not None and hop > 0:
                    score -= 5.0 * min(5, hop)

            # Battery level
            score += 0.2 * u.battery_percent

            candidates.append((score, u.id))

        if not candidates:
            return None

        # Sort descending by score, deterministic tie-break by ascending ID
        candidates.sort(key=lambda item: (-item[0], item[1]))
        return candidates[0][1]

    def step(
        self,
        snapshot: StateSnapshot,
        network_analysis: Optional[NetworkAnalysis] = None,
        flight_records: Optional[Dict[str, Any]] = None,
        speed_limit: float = 5.0,
        idle_rate: float = 1.0,
        movement_rate: float = 0.5,
        max_sortie_s: float = 1200.0,
        enforce_sortie_limit: bool = True,
        dt: float = 1.0,
    ) -> List[Command]:
        """Execute one tick of dynamic relay role management."""
        if not self.config.enabled:
            return []

        commands: List[Command] = []
        tick = snapshot.simulation_tick
        sim_time = snapshot.simulation_time
        gcs_pos = snapshot.gcs_position

        # Track connectivity around handoff
        for surv_id, r_id in list(self.surveyor_to_relay.items()):
            is_connected = bool(network_analysis and surv_id in network_analysis.connected_uav_ids)
            if not self._handoff_completed:
                if is_connected:
                    self.connected_time_before_handoff += dt
            else:
                if is_connected:
                    self.connected_time_after_handoff += dt
                    if self._active_handoff is not None and self.network_reconfiguration_time_s is None:
                        reconfig_t = max(0.0, sim_time - self._active_handoff["handoff_start_time"])
                        self.network_reconfiguration_time_s = reconfig_t

        # 1. Inspect existing active relays
        current_relays = [u for u in snapshot.uavs.values() if u.role == Role.RELAY]
        for relay in sorted(current_relays, key=lambda x: x.id):
            surveyor_id = self.relay_to_surveyor.get(relay.id) or relay.relay_target_id
            relay_pos = self.relay_positions.get(relay.id) or relay.target_position or relay.position_xy

            # Condition A: Relay hardware failure
            if relay.failure_state != FailureState.NORMAL or not relay.active:
                self.relay_losses += 1
                # Find recovery replacement
                if surveyor_id and surveyor_id in snapshot.uavs:
                    rep_id = self.select_relay_candidate(
                        snapshot=snapshot,
                        target_uav_id=surveyor_id,
                        relay_position=relay_pos,
                        network_analysis=network_analysis,
                        exclude_uav_ids={relay.id},
                        speed_limit=speed_limit,
                        idle_rate=idle_rate,
                        movement_rate=movement_rate,
                        max_sortie_s=max_sortie_s,
                        enforce_sortie_limit=enforce_sortie_limit,
                        flight_records=flight_records,
                    )
                    if rep_id:
                        commands.append(AssignRelayRoleCommand(source_tick=tick, uav_id=rep_id, target_position=relay_pos, relay_for_uav_id=surveyor_id))
                        commands.append(SetTargetPositionCommand(source_tick=tick, uav_id=rep_id, target_position=relay_pos, speed=speed_limit))
                        self.surveyor_to_relay[surveyor_id] = rep_id
                        self.relay_to_surveyor[rep_id] = surveyor_id
                        self.relay_positions[rep_id] = relay_pos
                        self.relay_recovery_successes += 1
                        self.relay_assignments += 1
                self.relay_to_surveyor.pop(relay.id, None)
                self.relay_positions.pop(relay.id, None)
                continue

            # Condition B: Imminent RTH or active RTH on relay (Preemptive Relay Handoff)
            dist_gcs = math.hypot(relay.position_xy[0] - gcs_pos[0], relay.position_xy[1] - gcs_pos[1])
            return_time_s = dist_gcs / max(1.0, speed_limit)
            return_energy = (idle_rate * return_time_s + movement_rate * dist_gcs) * 1.2
            battery_near_rth = relay.battery_energy <= (return_energy + self.config.min_relay_reserve_energy_wh)

            sortie_near_rth = False
            if enforce_sortie_limit and flight_records and relay.id in flight_records:
                rec = flight_records[relay.id]
                cur_airborne = getattr(rec, "current_sortie_duration_s", 0.0) if getattr(rec, "is_airborne", False) else 0.0
                rem_sortie = max(0.0, max_sortie_s - cur_airborne)
                sortie_near_rth = rem_sortie <= (return_time_s + self.config.rth_handoff_margin_s)

            is_rth = (relay.rth_state != RTHState.NONE or battery_near_rth or sortie_near_rth)

            if is_rth and surveyor_id and surveyor_id in snapshot.uavs:
                # Check if handoff already initiated
                if self.surveyor_to_relay.get(surveyor_id) == relay.id:
                    # Select available replacement
                    rep_id = self.select_relay_candidate(
                        snapshot=snapshot,
                        target_uav_id=surveyor_id,
                        relay_position=relay_pos,
                        network_analysis=network_analysis,
                        exclude_uav_ids={relay.id},
                        speed_limit=speed_limit,
                        idle_rate=idle_rate,
                        movement_rate=movement_rate,
                        max_sortie_s=max_sortie_s,
                        enforce_sortie_limit=enforce_sortie_limit,
                        flight_records=flight_records,
                    )
                    if rep_id:
                        commands.append(HandoffRelayCommand(
                            source_tick=tick,
                            uav_id=relay.id,
                            replacement_uav_id=rep_id,
                            target_position=relay_pos,
                            relay_for_uav_id=surveyor_id,
                        ))
                        commands.append(SetTargetPositionCommand(source_tick=tick, uav_id=rep_id, target_position=relay_pos, speed=speed_limit))
                        self.surveyor_to_relay[surveyor_id] = rep_id
                        self.relay_to_surveyor[rep_id] = surveyor_id
                        self.relay_positions[rep_id] = relay_pos
                        self.relay_handoffs += 1
                        self.relay_assignments += 1

                        self._active_handoff = {
                            "surveyor_id": surveyor_id,
                            "old_relay_id": relay.id,
                            "new_relay_id": rep_id,
                            "handoff_start_time": sim_time,
                        }
                        self._handoff_completed = True

                        # Release old relay and send it home safely
                        commands.append(ReleaseRelayRoleCommand(source_tick=tick, uav_id=relay.id, next_role=Role.IDLE))
                        if relay.rth_state == RTHState.NONE:
                            commands.append(StartRTHCommand(source_tick=tick, uav_id=relay.id))
                        self.relay_to_surveyor.pop(relay.id, None)
                        self.relay_positions.pop(relay.id, None)
                        self.relay_releases += 1

        # 2. Inspect active SURVEYORs needing relay support
        active_surveyors = [
            u for u in snapshot.uavs.values()
            if u.active and u.role in (Role.SURVEYOR, Role.SCOUT) and u.rth_state == RTHState.NONE
        ]

        for surveyor in sorted(active_surveyors, key=lambda x: x.id):
            d_gcs = math.hypot(surveyor.position_xy[0] - gcs_pos[0], surveyor.position_xy[1] - gcs_pos[1])
            is_connected = bool(network_analysis and surveyor.id in network_analysis.connected_uav_ids)

            # Needs relay if outside 100m direct link and not currently assigned a relay
            if d_gcs > self.config.communication_range_m * 0.9 and not is_connected:
                current_relay_id = self.surveyor_to_relay.get(surveyor.id)
                if not current_relay_id or current_relay_id not in snapshot.uavs:
                    r_pos = self.compute_relay_position(gcs_pos, surveyor.position_xy)
                    cand_id = self.select_relay_candidate(
                        snapshot=snapshot,
                        target_uav_id=surveyor.id,
                        relay_position=r_pos,
                        network_analysis=network_analysis,
                        speed_limit=speed_limit,
                        idle_rate=idle_rate,
                        movement_rate=movement_rate,
                        max_sortie_s=max_sortie_s,
                        enforce_sortie_limit=enforce_sortie_limit,
                        flight_records=flight_records,
                    )
                    if cand_id:
                        commands.append(AssignRelayRoleCommand(source_tick=tick, uav_id=cand_id, target_position=r_pos, relay_for_uav_id=surveyor.id))
                        commands.append(SetTargetPositionCommand(source_tick=tick, uav_id=cand_id, target_position=r_pos, speed=speed_limit))
                        self.surveyor_to_relay[surveyor.id] = cand_id
                        self.relay_to_surveyor[cand_id] = surveyor.id
                        self.relay_positions[cand_id] = r_pos
                        self.relay_assignments += 1

        # 3. Release idle relays whose surveyors returned or no longer exist
        for relay_id, surv_id in list(self.relay_to_surveyor.items()):
            surv = snapshot.uavs.get(surv_id)
            if not surv or not surv.active or surv.rth_state != RTHState.NONE or surv.sortie_state in (SortieState.LANDED, SortieState.RECHARGING):
                relay_uav = snapshot.uavs.get(relay_id)
                if relay_uav and relay_uav.role == Role.RELAY:
                    commands.append(ReleaseRelayRoleCommand(source_tick=tick, uav_id=relay_id, next_role=Role.SURVEYOR))
                    self.relay_releases += 1
                self.relay_to_surveyor.pop(relay_id, None)
                self.surveyor_to_relay.pop(surv_id, None)
                self.relay_positions.pop(relay_id, None)

        return commands
