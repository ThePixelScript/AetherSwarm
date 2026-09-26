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
    airborne_reuse_tolerance_m: float = 25.0  # Phase 5: station tolerance for reusing eligible airborne relays


class ChainStatus:
    FORMING = "FORMING"
    ACTIVE = "ACTIVE"
    HANDOFF = "HANDOFF"
    DEGRADED = "DEGRADED"
    TEARDOWN = "TEARDOWN"


@dataclass
class RelayChain:
    """Authoritative representation of a multi-hop relay chain for a surveyor/task."""
    chain_id: str
    task_id: str
    surveyor_id: str
    relay_ids: List[str]  # Ordered [R_1, ..., R_K], R_1 closest to GCS, R_K closest to surveyor
    station_positions: List[Tuple[float, float]]  # Ordered [r_1, ..., r_K]
    created_tick: int
    created_time: float
    status: str = ChainStatus.ACTIVE
    pending_handoffs: Dict[int, Dict[str, Any]] = field(default_factory=dict)


class DynamicRelayManager:
    """Autonomous dynamic coordinator for swarm relay roles and link continuity."""

    def __init__(self, config: Optional[RelayManagementConfig] = None) -> None:
        self.config = config or RelayManagementConfig()
        # surveyor_id -> relay_id (scalar compatibility mapping, terminal relay R_K)
        self.surveyor_to_relay: Dict[str, str] = {}
        # relay_id -> surveyor_id
        self.relay_to_surveyor: Dict[str, str] = {}
        # relay_id -> target coordinate
        self.relay_positions: Dict[str, Tuple[float, float]] = {}

        # Multi-hop Relay Chains (Phase 5)
        self.chains: Dict[str, RelayChain] = {}
        self.surveyor_to_chain: Dict[str, str] = {}
        self.relay_to_chain: Dict[str, str] = {}
        # Shared Relay Trunk Reference Counting: relay_id -> Set[surveyor_id]
        self.relay_dependent_surveyors: Dict[str, Set[str]] = {}

        # Metrics (Phase 3 & 4)
        self.relay_assignments: int = 0
        self.relay_releases: int = 0
        self.relay_handoffs: int = 0
        self.relay_losses: int = 0
        self.relay_recovery_successes: int = 0

        # Phase 5 Multi-Hop Metrics
        self.relay_chains_created: int = 0
        self.relay_chain_handoffs: int = 0
        self.relay_chain_failures: int = 0
        self.relay_chain_recoveries: int = 0
        self.chain_maintenance_duration_s: float = 0.0

        self.connected_time_before_handoff: float = 0.0
        self.connected_time_after_handoff: float = 0.0
        self.network_reconfiguration_time_s: Optional[float] = None

        # Tracking state for active handoff / recovery
        self._active_handoff: Optional[Dict[str, Any]] = None
        self._handoff_completed: bool = False
        self._pending_single_handoffs: Dict[str, Dict[str, Any]] = {}

    def reset(self) -> None:
        """Reset internal relay manager state."""
        self.surveyor_to_relay.clear()
        self.relay_to_surveyor.clear()
        self.relay_positions.clear()
        self.chains.clear()
        self.surveyor_to_chain.clear()
        self.relay_to_chain.clear()
        self.relay_dependent_surveyors.clear()
        self.relay_assignments = 0
        self.relay_releases = 0
        self.relay_handoffs = 0
        self.relay_losses = 0
        self.relay_recovery_successes = 0
        self.relay_chains_created = 0
        self.relay_chain_handoffs = 0
        self.relay_chain_failures = 0
        self.relay_chain_recoveries = 0
        self.chain_maintenance_duration_s = 0.0
        self.connected_time_before_handoff = 0.0
        self.connected_time_after_handoff = 0.0
        self._active_handoff = None
        self._handoff_completed = False
        self._pending_single_handoffs.clear()

    def register_chain(
        self,
        chain: Optional[RelayChain] = None,
        chain_id: Optional[str] = None,
        surveyor_id: Optional[str] = None,
        relay_ids: Optional[Sequence[str]] = None,
        station_positions: Optional[Sequence[Tuple[float, float]]] = None,
        task_id: str = "",
        created_tick: int = 0,
        created_time: float = 0.0,
    ) -> RelayChain:
        """Register an authoritative multi-hop relay chain."""
        if chain is None:
            cid = chain_id or f"chain_{surveyor_id}_{created_tick}"
            chain = RelayChain(
                chain_id=cid,
                task_id=task_id,
                surveyor_id=surveyor_id or "",
                relay_ids=list(relay_ids or []),
                station_positions=list(station_positions or []),
                created_tick=created_tick,
                created_time=created_time,
                status=ChainStatus.ACTIVE,
            )
        if chain.chain_id in self.chains:
            old_c = self.chains[chain.chain_id]
            if old_c.surveyor_id and old_c.surveyor_id != chain.surveyor_id:
                self.surveyor_to_chain.pop(old_c.surveyor_id, None)
                self.surveyor_to_relay.pop(old_c.surveyor_id, None)
                for rid in old_c.relay_ids:
                    if rid in self.relay_dependent_surveyors:
                        self.relay_dependent_surveyors[rid].discard(old_c.surveyor_id)

        self.chains[chain.chain_id] = chain
        self.surveyor_to_chain[chain.surveyor_id] = chain.chain_id
        for rid, pos in zip(chain.relay_ids, chain.station_positions):
            self.relay_to_chain[rid] = chain.chain_id
            self.relay_to_surveyor[rid] = chain.surveyor_id
            self.relay_positions[rid] = pos
            if rid not in self.relay_dependent_surveyors:
                self.relay_dependent_surveyors[rid] = set()
            self.relay_dependent_surveyors[rid].add(chain.surveyor_id)
        if chain.relay_ids:
            # Backward-compatible scalar mapping: closest relay to surveyor is terminal relay R_K
            self.surveyor_to_relay[chain.surveyor_id] = chain.relay_ids[-1]
        self.relay_chains_created += 1
        self.relay_assignments += len(chain.relay_ids)
        return chain

    def teardown_chain(self, chain_id: str, commands: Optional[List[Command]] = None, tick: int = 0) -> List[Command]:
        """Tear down an active or degraded relay chain, releasing relays no longer needed by any surveyor."""
        cmds: List[Command] = []
        chain = self.chains.get(chain_id)
        if not chain:
            return cmds
        chain.status = ChainStatus.TEARDOWN

        # Find all surveyors associated with this chain
        chain_surveyors = [s for s, c in self.surveyor_to_chain.items() if c == chain_id]
        if chain.surveyor_id:
            chain_surveyors.append(chain.surveyor_id)
        all_survs = set(chain_surveyors)

        # Release any in-progress replacement UAVs in pending handoffs
        for h in list(chain.pending_handoffs.values()):
            rep_id = h.get("replacement_id")
            if rep_id:
                if rep_id in self.relay_dependent_surveyors:
                    for s in all_survs:
                        self.relay_dependent_surveyors[rep_id].discard(s)
                deps = self.relay_dependent_surveyors.get(rep_id, set())
                if not deps:
                    release_cmd = ReleaseRelayRoleCommand(source_tick=tick, uav_id=rep_id, next_role=Role.IDLE)
                    if commands is not None:
                        commands.append(release_cmd)
                    cmds.append(release_cmd)
                    self.relay_releases += 1
                    self.relay_to_chain.pop(rep_id, None)
                    self.relay_to_surveyor.pop(rep_id, None)
                    self.relay_positions.pop(rep_id, None)
                    self.relay_dependent_surveyors.pop(rep_id, None)
                    ret_cmd = SetTargetPositionCommand(source_tick=tick, uav_id=rep_id, target_position=(0.0, 500.0))
                    if commands is not None:
                        commands.append(ret_cmd)
                    cmds.append(ret_cmd)
        chain.pending_handoffs.clear()

        for rid in chain.relay_ids:
            if rid in self.relay_dependent_surveyors:
                for s in all_survs:
                    self.relay_dependent_surveyors[rid].discard(s)
            deps = self.relay_dependent_surveyors.get(rid, set())
            if not deps:
                release_cmd = ReleaseRelayRoleCommand(source_tick=tick, uav_id=rid, next_role=Role.IDLE)
                if commands is not None:
                    commands.append(release_cmd)
                cmds.append(release_cmd)
                self.relay_releases += 1
                self.relay_to_chain.pop(rid, None)
                self.relay_to_surveyor.pop(rid, None)
                self.relay_positions.pop(rid, None)
                self.relay_dependent_surveyors.pop(rid, None)
                ret_cmd = SetTargetPositionCommand(source_tick=tick, uav_id=rid, target_position=(0.0, 500.0))
                if commands is not None:
                    commands.append(ret_cmd)
                cmds.append(ret_cmd)
            else:
                # Other surveyor(s) still depend on this shared trunk relay!
                next_surv = next(iter(deps))
                self.relay_to_surveyor[rid] = next_surv
                next_chain = self.surveyor_to_chain.get(next_surv)
                if next_chain:
                    self.relay_to_chain[rid] = next_chain
        for s in all_survs:
            self.surveyor_to_chain.pop(s, None)
            self.surveyor_to_relay.pop(s, None)
        self.chains.pop(chain_id, None)
        return cmds

    def _get_all_reserved_uav_ids(self) -> Set[str]:
        """Collect all UAV IDs currently assigned or reserved across all chains and single relays."""
        reserved: Set[str] = set()
        for c in self.chains.values():
            reserved.add(c.surveyor_id)
            reserved.update(c.relay_ids)
            for h in c.pending_handoffs.values():
                rep = h.get("replacement_id")
                if rep:
                    reserved.add(rep)
                inc = h.get("incumbent_id")
                if inc:
                    reserved.add(inc)
        for h in self._pending_single_handoffs.values():
            rep = h.get("replacement_id")
            if rep:
                reserved.add(rep)
            inc = h.get("incumbent_id")
            if inc:
                reserved.add(inc)
        for sid, rid in self.surveyor_to_relay.items():
            reserved.add(sid)
            reserved.add(rid)
        return reserved

    def is_in_pending_handoff(self, uav_id: str) -> bool:
        """Check if a UAV is currently participating in a pending handoff."""
        for c in self.chains.values():
            for h in c.pending_handoffs.values():
                if h.get("replacement_id") == uav_id or h.get("incumbent_id") == uav_id:
                    return True
        for h in self._pending_single_handoffs.values():
            if h.get("replacement_id") == uav_id or h.get("incumbent_id") == uav_id:
                return True
        return False

    def _verify_replacement_connectivity(
        self,
        replacement_id: str,
        station_idx: int,
        chain: RelayChain,
        snapshot: StateSnapshot,
        network_analysis: Optional[NetworkAnalysis] = None,
    ) -> bool:
        """Verify replacement UAV has active adjacent links to predecessor and successor."""
        rep_u = snapshot.uavs.get(replacement_id)
        if not rep_u or not rep_u.active or rep_u.failure_state != FailureState.NORMAL:
            return False

        # 1. Predecessor node
        if station_idx == 0:
            pred_id = (network_analysis.gcs_id or "gcs") if network_analysis else "gcs"
            pred_pos = snapshot.gcs_position
        else:
            pred_id = chain.relay_ids[station_idx - 1]
            pred_u = snapshot.uavs.get(pred_id)
            pred_pos = pred_u.position_xy if pred_u else chain.station_positions[station_idx - 1]

        # 2. Successor node
        if station_idx == len(chain.relay_ids) - 1:
            succ_id = chain.surveyor_id
            succ_u = snapshot.uavs.get(succ_id)
            succ_pos = succ_u.position_xy if succ_u else None
        else:
            succ_id = chain.relay_ids[station_idx + 1]
            succ_u = snapshot.uavs.get(succ_id)
            succ_pos = succ_u.position_xy if succ_u else chain.station_positions[station_idx + 1]

        if succ_pos is None:
            return False

        # If network_analysis contains links, verify links exist in network topology
        if network_analysis is not None and network_analysis.network and network_analysis.network.links:
            active_links = {
                frozenset([l.source_id, l.target_id])
                for l in network_analysis.network.links
                if l.active
            }
            if station_idx == 0:
                gcs_id = network_analysis.gcs_id or "gcs"
                pred_link_ok = (frozenset([replacement_id, gcs_id]) in active_links) or (frozenset([replacement_id, "gcs"]) in active_links)
            else:
                pred_link_ok = frozenset([replacement_id, pred_id]) in active_links

            succ_link_ok = frozenset([replacement_id, succ_id]) in active_links
            return pred_link_ok and succ_link_ok

        # Otherwise verify Euclidean distance to adjacent positions <= communication range
        d_pred = math.hypot(rep_u.position_xy[0] - pred_pos[0], rep_u.position_xy[1] - pred_pos[1])
        d_succ = math.hypot(rep_u.position_xy[0] - succ_pos[0], rep_u.position_xy[1] - succ_pos[1])
        return (d_pred <= self.config.communication_range_m) and (d_succ <= self.config.communication_range_m)


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
        - Prioritizes eligible airborne relay reuse if already in position (Step 7).
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
            if u.assigned_task_id is not None:
                continue

            # 2. Imminent RTH & Endurance Feasibility Gates (Requirement 5)
            if u.id in self.relay_positions:
                curr_st = self.relay_positions[u.id]
                d_st = math.hypot(curr_st[0] - relay_position[0], curr_st[1] - relay_position[1])
                if d_st > self.config.airborne_reuse_tolerance_m:
                    continue

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

            # 3. Dynamic Utility Scoring (Requirement 4 & Step 7 Airborne Reuse)
            score = 0.0

            # Airborne Relay Reuse Preference (Step 7)
            is_airborne = False
            if flight_records and u.id in flight_records:
                is_airborne = getattr(flight_records[u.id], "is_airborne", False)
            elif u.position_xy != gcs_pos:
                is_airborne = True

            if is_airborne and d_to_relay <= self.config.airborne_reuse_tolerance_m:
                score += 250.0  # Significant preference for reusing an already airborne relay in position

            # Role & task assignment preference
            if u.role == Role.IDLE:
                score += 100.0
            elif u.role == Role.SURVEYOR:
                score += 80.0
            elif u.role == Role.RELAY:
                score += 50.0 if (is_airborne and d_to_relay <= self.config.airborne_reuse_tolerance_m) else 10.0

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

    def select_relay_chain_candidates(
        self,
        snapshot: StateSnapshot,
        surveyor_id: str,
        stations: Sequence[Tuple[float, float]],
        network_analysis: Optional[NetworkAnalysis] = None,
        exclude_uav_ids: Optional[Set[str]] = None,
        speed_limit: float = 5.0,
        idle_rate: float = 1.0,
        movement_rate: float = 0.5,
        max_sortie_s: float = 1200.0,
        enforce_sortie_limit: bool = True,
        flight_records: Optional[Dict[str, Any]] = None,
    ) -> Optional[List[str]]:
        """Select distinct candidate UAVs for ordered relay stations atomically (Step 3).

        Args:
            stations: Ordered stations [r_1, ..., r_K], r_1 closest to GCS, r_K closest to surveyor.

        Returns:
            List of distinct UAV IDs [R_1, ..., R_K] corresponding to stations,
            or None if any station cannot be staffed (atomic all-or-nothing).
        """
        if not stations:
            return []

        excluded = set(exclude_uav_ids or ())
        excluded.add(surveyor_id)

        selected_candidates: List[str] = []

        # Staff stations sequentially (R_1 to R_K)
        for st_pos in stations:
            cand_id = self.select_relay_candidate(
                snapshot=snapshot,
                target_uav_id=surveyor_id,
                relay_position=st_pos,
                network_analysis=network_analysis,
                exclude_uav_ids=excluded,
                speed_limit=speed_limit,
                idle_rate=idle_rate,
                movement_rate=movement_rate,
                max_sortie_s=max_sortie_s,
                enforce_sortie_limit=enforce_sortie_limit,
                flight_records=flight_records,
            )
            if not cand_id:
                # Atomic guarantee: if any link cannot be staffed, fail entirely
                return None
            selected_candidates.append(cand_id)
            excluded.add(cand_id)

        return selected_candidates

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

        # Track maintenance duration for all active chains
        for chain in self.chains.values():
            if chain.status in (ChainStatus.ACTIVE, ChainStatus.HANDOFF):
                self.chain_maintenance_duration_s += dt

        # 0. Multi-Hop Chain Monitoring & Localized Link Handoff / Recovery (Phase 5)
        for chain in list(self.chains.values()):
            if chain.status == ChainStatus.TEARDOWN:
                continue

            surv = snapshot.uavs.get(chain.surveyor_id)
            if not surv or not surv.active or surv.rth_state != RTHState.NONE or surv.sortie_state in (SortieState.LANDED, SortieState.RECHARGING) or surv.assigned_task_id is None:
                self.teardown_chain(chain.chain_id, commands=commands, tick=tick)
                continue

            # Monitor each intermediate relay in chain
            for idx, (rid, st_pos) in enumerate(list(zip(chain.relay_ids, chain.station_positions))):
                ru = snapshot.uavs.get(rid)

                # Check if this station already has an in-progress handoff
                if idx in chain.pending_handoffs:
                    handoff = chain.pending_handoffs[idx]
                    rep_id = handoff["replacement_id"]
                    inc_id = handoff.get("incumbent_id")
                    rep_u = snapshot.uavs.get(rep_id)
                    inc_u = snapshot.uavs.get(inc_id) if inc_id else None

                    # Check replacement health / abort condition
                    rep_aborted = (
                        not rep_u
                        or not rep_u.active
                        or rep_u.failure_state != FailureState.NORMAL
                        or rep_u.rth_state != RTHState.NONE
                        or rep_u.sortie_state in (SortieState.RTH, SortieState.LANDING, SortieState.LANDED, SortieState.RECHARGING)
                    )

                    if rep_aborted:
                        chain.pending_handoffs.pop(idx, None)
                        self.relay_positions.pop(rep_id, None)
                        if rep_u and rep_u.active and rep_u.role == Role.RELAY:
                            commands.append(ReleaseRelayRoleCommand(source_tick=tick, uav_id=rep_id, next_role=Role.IDLE))
                            self.relay_releases += 1

                        if inc_u and inc_u.active and inc_u.failure_state == FailureState.NORMAL:
                            # Incumbent is still operational: attempt selecting a new candidate
                            reserved = self._get_all_reserved_uav_ids()
                            new_rep_id = self.select_relay_candidate(
                                snapshot=snapshot,
                                target_uav_id=chain.surveyor_id,
                                relay_position=st_pos,
                                network_analysis=network_analysis,
                                exclude_uav_ids=reserved,
                                speed_limit=speed_limit,
                                idle_rate=idle_rate,
                                movement_rate=movement_rate,
                                max_sortie_s=max_sortie_s,
                                enforce_sortie_limit=enforce_sortie_limit,
                                flight_records=flight_records,
                            )
                            if new_rep_id:
                                commands.append(AssignRelayRoleCommand(source_tick=tick, uav_id=new_rep_id, target_position=st_pos, relay_for_uav_id=chain.surveyor_id))
                                commands.append(SetTargetPositionCommand(source_tick=tick, uav_id=new_rep_id, target_position=st_pos, speed=speed_limit))
                                self.relay_assignments += 1
                                self.relay_positions[new_rep_id] = st_pos
                                chain.pending_handoffs[idx] = {
                                    "station_idx": idx,
                                    "incumbent_id": inc_id,
                                    "replacement_id": new_rep_id,
                                    "station_position": st_pos,
                                    "status": "IN_PROGRESS",
                                    "start_tick": tick,
                                    "start_time": snapshot.simulation_time,
                                }
                                chain.status = ChainStatus.HANDOFF
                            else:
                                chain.status = ChainStatus.DEGRADED
                        else:
                            chain.status = ChainStatus.DEGRADED
                            self.relay_losses += 1
                            self.relay_chain_failures += 1
                        continue

                    # If incumbent failed while replacement was in transit
                    if inc_id and (not inc_u or not inc_u.active or inc_u.failure_state != FailureState.NORMAL):
                        self.relay_losses += 1
                        self.relay_chain_failures += 1
                        chain.status = ChainStatus.DEGRADED
                        self.relay_to_chain.pop(inc_id, None)
                        self.relay_to_surveyor.pop(inc_id, None)
                        self.relay_positions.pop(inc_id, None)
                        handoff["incumbent_id"] = None

                    # Check if replacement has arrived at the station (tolerance <= 1.0m)
                    dist_station = math.hypot(rep_u.position_xy[0] - st_pos[0], rep_u.position_xy[1] - st_pos[1])
                    if dist_station <= 1.0:
                        if self._verify_replacement_connectivity(rep_id, idx, chain, snapshot, network_analysis):
                            # Make-before-break handoff completion (atomic swap)
                            commands.append(HandoffRelayCommand(
                                source_tick=tick,
                                uav_id=inc_id or rep_id,
                                replacement_uav_id=rep_id,
                                target_position=st_pos,
                                relay_for_uav_id=chain.surveyor_id,
                            ))
                            commands.append(SetTargetPositionCommand(source_tick=tick, uav_id=rep_id, target_position=st_pos, speed=speed_limit))
                            chain.relay_ids[idx] = rep_id
                            self.relay_to_chain[rep_id] = chain.chain_id
                            self.relay_to_surveyor[rep_id] = chain.surveyor_id
                            self.relay_positions[rep_id] = st_pos
                            if idx == len(chain.relay_ids) - 1:
                                self.surveyor_to_relay[chain.surveyor_id] = rep_id

                            self.relay_handoffs += 1
                            self.relay_chain_handoffs += 1

                            if inc_id:
                                self.relay_to_chain.pop(inc_id, None)
                                self.relay_to_surveyor.pop(inc_id, None)
                                self.relay_positions.pop(inc_id, None)
                                self.relay_releases += 1
                                commands.append(ReleaseRelayRoleCommand(source_tick=tick, uav_id=inc_id, next_role=Role.IDLE))
                                if inc_u and inc_u.rth_state == RTHState.NONE:
                                    commands.append(StartRTHCommand(source_tick=tick, uav_id=inc_id))

                            chain.pending_handoffs.pop(idx, None)
                            if not chain.pending_handoffs and chain.status != ChainStatus.DEGRADED:
                                chain.status = ChainStatus.ACTIVE
                            continue
                        else:
                            chain.status = ChainStatus.HANDOFF
                            continue
                    else:
                        # Replacement still in transit
                        chain.status = ChainStatus.HANDOFF
                        continue

                # Condition A: Relay hardware failure
                if not ru or not ru.active or ru.failure_state != FailureState.NORMAL:
                    self.relay_losses += 1
                    self.relay_chain_failures += 1
                    reserved = self._get_all_reserved_uav_ids()
                    rep_id = self.select_relay_candidate(
                        snapshot=snapshot,
                        target_uav_id=chain.surveyor_id,
                        relay_position=st_pos,
                        network_analysis=network_analysis,
                        exclude_uav_ids=reserved,
                        speed_limit=speed_limit,
                        idle_rate=idle_rate,
                        movement_rate=movement_rate,
                        max_sortie_s=max_sortie_s,
                        enforce_sortie_limit=enforce_sortie_limit,
                        flight_records=flight_records,
                    )
                    if rep_id:
                        commands.append(AssignRelayRoleCommand(source_tick=tick, uav_id=rep_id, target_position=st_pos, relay_for_uav_id=chain.surveyor_id))
                        commands.append(SetTargetPositionCommand(source_tick=tick, uav_id=rep_id, target_position=st_pos, speed=speed_limit))
                        chain.relay_ids[idx] = rep_id
                        self.relay_to_chain[rep_id] = chain.chain_id
                        self.relay_to_surveyor[rep_id] = chain.surveyor_id
                        self.relay_positions[rep_id] = st_pos
                        if idx == len(chain.relay_ids) - 1:
                            self.surveyor_to_relay[chain.surveyor_id] = rep_id
                        self.relay_to_chain.pop(rid, None)
                        self.relay_to_surveyor.pop(rid, None)
                        self.relay_positions.pop(rid, None)
                        self.relay_recovery_successes += 1
                        self.relay_chain_recoveries += 1
                        self.relay_assignments += 1
                    else:
                        chain.status = ChainStatus.DEGRADED
                    continue

                # Condition B: Imminent RTH / sortie limit
                dist_gcs = math.hypot(ru.position_xy[0] - gcs_pos[0], ru.position_xy[1] - gcs_pos[1])
                return_time_s = dist_gcs / max(1.0, speed_limit)
                return_energy = (idle_rate * return_time_s + movement_rate * dist_gcs) * 1.2
                battery_near_rth = ru.battery_energy <= (return_energy + self.config.min_relay_reserve_energy_wh)

                sortie_near_rth = False
                if enforce_sortie_limit and flight_records and ru.id in flight_records:
                    rec = flight_records[ru.id]
                    cur_airborne = getattr(rec, "current_sortie_duration_s", 0.0) if getattr(rec, "is_airborne", False) else 0.0
                    rem_sortie = max(0.0, max_sortie_s - cur_airborne)
                    sortie_near_rth = rem_sortie <= (return_time_s + self.config.rth_handoff_margin_s)

                is_rth = (ru.rth_state != RTHState.NONE or battery_near_rth or sortie_near_rth)
                if is_rth:
                    reserved = self._get_all_reserved_uav_ids()
                    rep_id = self.select_relay_candidate(
                        snapshot=snapshot,
                        target_uav_id=chain.surveyor_id,
                        relay_position=st_pos,
                        network_analysis=network_analysis,
                        exclude_uav_ids=reserved,
                        speed_limit=speed_limit,
                        idle_rate=idle_rate,
                        movement_rate=movement_rate,
                        max_sortie_s=max_sortie_s,
                        enforce_sortie_limit=enforce_sortie_limit,
                        flight_records=flight_records,
                    )
                    if rep_id:
                        rep_u = snapshot.uavs.get(rep_id)
                        dist_rep_station = math.hypot(rep_u.position_xy[0] - st_pos[0], rep_u.position_xy[1] - st_pos[1]) if rep_u else 999.0
                        if dist_rep_station <= 1.0 and self._verify_replacement_connectivity(rep_id, idx, chain, snapshot, network_analysis):
                            # Already at station and verified connected: complete immediately
                            commands.append(HandoffRelayCommand(
                                source_tick=tick,
                                uav_id=ru.id,
                                replacement_uav_id=rep_id,
                                target_position=st_pos,
                                relay_for_uav_id=chain.surveyor_id,
                            ))
                            commands.append(SetTargetPositionCommand(source_tick=tick, uav_id=rep_id, target_position=st_pos, speed=speed_limit))
                            chain.relay_ids[idx] = rep_id
                            self.relay_to_chain[rep_id] = chain.chain_id
                            self.relay_to_surveyor[rep_id] = chain.surveyor_id
                            self.relay_positions[rep_id] = st_pos
                            if idx == len(chain.relay_ids) - 1:
                                self.surveyor_to_relay[chain.surveyor_id] = rep_id
                            self.relay_to_chain.pop(ru.id, None)
                            self.relay_to_surveyor.pop(ru.id, None)
                            self.relay_positions.pop(ru.id, None)
                            self.relay_handoffs += 1
                            self.relay_chain_handoffs += 1
                            self.relay_assignments += 1
                            self.relay_releases += 1
                            commands.append(ReleaseRelayRoleCommand(source_tick=tick, uav_id=ru.id, next_role=Role.IDLE))
                            if ru.rth_state == RTHState.NONE:
                                commands.append(StartRTHCommand(source_tick=tick, uav_id=ru.id))
                        else:
                            # MAKE-BEFORE-BREAK DISPATCH:
                            # Replacement is dispatched to station while incumbent stays active
                            commands.append(AssignRelayRoleCommand(source_tick=tick, uav_id=rep_id, target_position=st_pos, relay_for_uav_id=chain.surveyor_id))
                            commands.append(SetTargetPositionCommand(source_tick=tick, uav_id=rep_id, target_position=st_pos, speed=speed_limit))
                            self.relay_assignments += 1
                            self.relay_positions[rep_id] = st_pos
                            chain.pending_handoffs[idx] = {
                                "station_idx": idx,
                                "incumbent_id": ru.id,
                                "replacement_id": rep_id,
                                "station_position": st_pos,
                                "status": "IN_PROGRESS",
                                "start_tick": tick,
                                "start_time": snapshot.simulation_time,
                            }
                            chain.status = ChainStatus.HANDOFF
                    else:
                        chain.status = ChainStatus.DEGRADED

        # 1. Inspect existing active relays (Legacy single-relay fallback)
        # 1a. Check and progress existing pending single handoffs
        for inc_id, h in list(self._pending_single_handoffs.items()):
            rep_id = h["replacement_id"]
            surv_id = h["surveyor_id"]
            r_pos = h["relay_pos"]
            rep_u = snapshot.uavs.get(rep_id)
            inc_u = snapshot.uavs.get(inc_id)

            if not rep_u or not rep_u.active or rep_u.failure_state != FailureState.NORMAL or rep_u.rth_state != RTHState.NONE:
                # Replacement aborted
                self._pending_single_handoffs.pop(inc_id, None)
                self.relay_positions.pop(rep_id, None)
                if rep_u and rep_u.active and rep_u.role == Role.RELAY:
                    commands.append(ReleaseRelayRoleCommand(source_tick=tick, uav_id=rep_id, next_role=Role.IDLE))
                    self.relay_releases += 1
                continue

            dist_station = math.hypot(rep_u.position_xy[0] - r_pos[0], rep_u.position_xy[1] - r_pos[1])
            if dist_station <= 1.0:
                # Arrived at station! Complete make-before-break handoff
                commands.append(HandoffRelayCommand(
                    source_tick=tick,
                    uav_id=inc_id,
                    replacement_uav_id=rep_id,
                    target_position=r_pos,
                    relay_for_uav_id=surv_id,
                ))
                commands.append(SetTargetPositionCommand(source_tick=tick, uav_id=rep_id, target_position=r_pos, speed=speed_limit))
                self.surveyor_to_relay[surv_id] = rep_id
                self.relay_to_surveyor[rep_id] = surv_id
                self.relay_positions[rep_id] = r_pos
                self.relay_handoffs += 1
                self.relay_assignments += 1
                self._active_handoff = {
                    "surveyor_id": surv_id,
                    "old_relay_id": inc_id,
                    "new_relay_id": rep_id,
                    "handoff_start_time": h["start_time"],
                }
                self._handoff_completed = True

                # Release old relay
                if inc_u:
                    commands.append(ReleaseRelayRoleCommand(source_tick=tick, uav_id=inc_id, next_role=Role.IDLE))
                    if inc_u.rth_state == RTHState.NONE:
                        commands.append(StartRTHCommand(source_tick=tick, uav_id=inc_id))
                    self.relay_to_surveyor.pop(inc_id, None)
                    self.relay_positions.pop(inc_id, None)
                    self.relay_releases += 1

                self._pending_single_handoffs.pop(inc_id, None)

        current_relays = [u for u in snapshot.uavs.values() if u.role == Role.RELAY]
        for relay in sorted(current_relays, key=lambda x: x.id):
            if relay.id in self.relay_to_chain or self.is_in_pending_handoff(relay.id):
                continue  # Handled in multi-hop chain monitor or pending single handoff above
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
                if self.surveyor_to_relay.get(surveyor_id) == relay.id and relay.id not in self._pending_single_handoffs:
                    reserved = self._get_all_reserved_uav_ids()
                    rep_id = self.select_relay_candidate(
                        snapshot=snapshot,
                        target_uav_id=surveyor_id,
                        relay_position=relay_pos,
                        network_analysis=network_analysis,
                        exclude_uav_ids=reserved,
                        speed_limit=speed_limit,
                        idle_rate=idle_rate,
                        movement_rate=movement_rate,
                        max_sortie_s=max_sortie_s,
                        enforce_sortie_limit=enforce_sortie_limit,
                        flight_records=flight_records,
                    )
                    if rep_id:
                        rep_u = snapshot.uavs.get(rep_id)
                        dist_rep_station = math.hypot(rep_u.position_xy[0] - relay_pos[0], rep_u.position_xy[1] - relay_pos[1]) if rep_u else 999.0
                        if dist_rep_station <= 1.0:
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

                            commands.append(ReleaseRelayRoleCommand(source_tick=tick, uav_id=relay.id, next_role=Role.IDLE))
                            if relay.rth_state == RTHState.NONE:
                                commands.append(StartRTHCommand(source_tick=tick, uav_id=relay.id))
                            self.relay_to_surveyor.pop(relay.id, None)
                            self.relay_positions.pop(relay.id, None)
                            self.relay_releases += 1
                        else:
                            # MAKE-BEFORE-BREAK: Dispatch replacement while incumbent stays active on station
                            commands.append(AssignRelayRoleCommand(source_tick=tick, uav_id=rep_id, target_position=relay_pos, relay_for_uav_id=surveyor_id))
                            commands.append(SetTargetPositionCommand(source_tick=tick, uav_id=rep_id, target_position=relay_pos, speed=speed_limit))
                            self.relay_assignments += 1
                            self.relay_positions[rep_id] = relay_pos
                            self._pending_single_handoffs[relay.id] = {
                                "incumbent_id": relay.id,
                                "replacement_id": rep_id,
                                "relay_pos": relay_pos,
                                "surveyor_id": surveyor_id,
                                "start_time": sim_time,
                            }

        # 2. Inspect active SURVEYORs needing relay support
        active_surveyors = [
            u for u in snapshot.uavs.values()
            if u.active and u.role in (Role.SURVEYOR, Role.SCOUT) and u.rth_state == RTHState.NONE
        ]

        for surveyor in sorted(active_surveyors, key=lambda x: x.id):
            if surveyor.id in self.surveyor_to_chain:
                continue
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
                if relay_id in self.relay_dependent_surveyors:
                    self.relay_dependent_surveyors[relay_id].discard(surv_id)
                deps = self.relay_dependent_surveyors.get(relay_id, set())
                if not deps:
                    relay_uav = snapshot.uavs.get(relay_id)
                    if relay_uav and relay_uav.role == Role.RELAY:
                        commands.append(ReleaseRelayRoleCommand(source_tick=tick, uav_id=relay_id, next_role=Role.SURVEYOR))
                        self.relay_releases += 1
                    self.relay_to_surveyor.pop(relay_id, None)
                    self.surveyor_to_relay.pop(surv_id, None)
                    self.relay_positions.pop(relay_id, None)
                    self.relay_dependent_surveyors.pop(relay_id, None)
                else:
                    next_surv = next(iter(deps))
                    self.relay_to_surveyor[relay_id] = next_surv

        return commands
