"""Ingress coordinator for establishing safe, connected initial swarm deployment."""
from __future__ import annotations

import dataclasses
import math
from enum import Enum
from typing import Any, List, Optional, Tuple

from ..communication.analysis import BaselineCommunicationAnalyzer
from ..core.commands import AssignRelayRoleCommand, Command, SetTargetPositionCommand
from ..core.enums import Role, RTHState
from ..core.models import StateSnapshot


class IngressStatus(str, Enum):
    DEPLOYING = "DEPLOYING"
    HOLDING = "HOLDING"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"


class IngressCoordinator:
    """Coordinates connectivity-preserving initial swarm ingress deployment."""

    def __init__(
        self,
        comm_analyzer: BaselineCommunicationAnalyzer,
        gcs_position: Tuple[float, float] = (-75.0, 500.0),
        d_safe: float = 85.0,
    ) -> None:
        self.gcs_position = gcs_position
        self.d_safe = d_safe
        self.comm_analyzer = comm_analyzer
        self.status = IngressStatus.DEPLOYING

        # Safe geometric chain: GCS -> R1 -> R2 -> R3
        # GCS is (-75, 500)
        self.target_chain = [
            (55.0, 450.0),
            (0.0, 500.0),
            (55.0, 550.0),
        ]
        self.assigned_relays: list[str] = []
        self.surveyors: list[str] = []

    @property
    def deployment_complete(self) -> bool:
        return self.status == IngressStatus.COMPLETE

    def plan(self, snapshot: StateSnapshot, network_analysis: Any = None) -> List[Command]:
        if self.status in (IngressStatus.COMPLETE, IngressStatus.ABORTED):
            return []

        commands: list[Command] = []
        active_uavs = [
            u for u in sorted(snapshot.uavs.values(), key=lambda x: x.id)
            if u.active and u.rth_state == RTHState.NONE
        ]

        if not self.assigned_relays:
            if len(active_uavs) < 5:
                relay_count = max(0, len(active_uavs) - 1)
                self.assigned_relays = [u.id for u in active_uavs[1 : 1 + relay_count]]
                self.surveyors = [active_uavs[0].id] if active_uavs else []
            else:
                self.assigned_relays = ["uav_2", "uav_3", "uav_4"]
                self.surveyors = ["uav_1", "uav_5"]

            for rid in self.assigned_relays:
                commands.append(
                    AssignRelayRoleCommand(
                        source_tick=snapshot.simulation_tick,
                        uav_id=rid,
                    )
                )

        all_in_position = True

        curr_net = (
            self.comm_analyzer.analyze(snapshot)
            if network_analysis is None
            else network_analysis
        )
        before_connected = set(curr_net.connected_uav_ids)

        # Enforce sequential deployment along safe geometry
        for i, uav_id in enumerate(self.assigned_relays):
            uav = snapshot.uavs.get(uav_id)
            if not uav or uav.rth_state != RTHState.NONE:
                continue
            if i >= len(self.target_chain):
                break

            target = self.target_chain[i]
            dist = math.hypot(uav.position_xy[0] - target[0], uav.position_xy[1] - target[1])
            if dist > 1.0:
                all_in_position = False
                next_target = target

                if uav.target_position != next_target:
                    dest_uavs = dict(snapshot.uavs)
                    dest_uavs[uav.id] = dataclasses.replace(uav, position_xy=next_target)
                    dest_snap = dataclasses.replace(snapshot, uavs=dest_uavs)
                    dest_net = self.comm_analyzer.analyze(dest_snap)

                    if uav.id in dest_net.connected_uav_ids:
                        safe = True
                        for peer_id in self.assigned_relays + self.surveyors:
                            if peer_id == uav.id:
                                continue
                            if peer_id in before_connected and peer_id not in dest_net.connected_uav_ids:
                                safe = False
                                break
                        if safe:
                            commands.append(
                                SetTargetPositionCommand(
                                    source_tick=snapshot.simulation_tick,
                                    uav_id=uav.id,
                                    target_position=next_target,
                                )
                            )
                            before_connected = set(dest_net.connected_uav_ids)

        for s_id in self.surveyors:
            s_uav = snapshot.uavs.get(s_id)
            if s_uav and s_uav.rth_state == RTHState.NONE:
                # Surveyors hold at safe positions until deployment completes
                s_target = (100.0, 420.0) if s_id == "uav_1" else (100.0, 580.0)
                dist = math.hypot(s_uav.position_xy[0] - s_target[0], s_uav.position_xy[1] - s_target[1])
                if dist > 1.0:
                    all_in_position = False
                    next_target = s_target
                    if s_uav.target_position != next_target:
                        dest_uavs = dict(snapshot.uavs)
                        dest_uavs[s_uav.id] = dataclasses.replace(s_uav, position_xy=next_target)
                        dest_snap = dataclasses.replace(snapshot, uavs=dest_uavs)
                        dest_net = self.comm_analyzer.analyze(dest_snap)
                        if s_uav.id in dest_net.connected_uav_ids:
                            safe = True
                            for peer_id in self.assigned_relays:
                                if peer_id in before_connected and peer_id not in dest_net.connected_uav_ids:
                                    safe = False
                                    break
                            if safe:
                                commands.append(
                                    SetTargetPositionCommand(
                                        source_tick=snapshot.simulation_tick,
                                        uav_id=s_uav.id,
                                        target_position=next_target,
                                    )
                                )

        if all_in_position and not commands:
            self.status = IngressStatus.COMPLETE

        return commands
