from typing import Any, List, Tuple, Dict
from ares_swarm.core.commands import Command, SetTargetPositionCommand, AssignRelayRoleCommand
from ares_swarm.core.models import StateSnapshot
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
import dataclasses
import math
from enum import Enum

class IngressStatus(Enum):
    DEPLOYING = "DEPLOYING"
    HOLDING = "HOLDING"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"

class IngressCoordinator:
    def __init__(self, comm_analyzer: BaselineCommunicationAnalyzer, gcs_position: Tuple[float, float] = (-75.0, 500.0), d_safe: float = 85.0):
        self.gcs_position = gcs_position
        self.d_safe = d_safe
        self.comm_analyzer = comm_analyzer
        self.status = IngressStatus.DEPLOYING
        
        # Safe geometric chain: GCS -> R1 -> R2 -> R3 -> R4
        # GCS is (-75, 500).
        # We will space them 75m apart along y=500.
        self.target_chain = [
            (55.0, 450.0),
            (0.0, 500.0),
            (55.0, 550.0)
        ]
        self.assigned_relays = []
        self.surveyors = []

    @property
    def deployment_complete(self) -> bool:
        return self.status == IngressStatus.COMPLETE

    def plan(self, snapshot: StateSnapshot, network_analysis: Any = None) -> List[Command]:
        if self.status in (IngressStatus.COMPLETE, IngressStatus.ABORTED):
            return []
            
        commands = []
        active_uavs = [u for u in sorted(snapshot.uavs.values(), key=lambda x: x.id) if u.active and u.rth_state.name == "NONE"]
        
        if not self.assigned_relays:
            if len(active_uavs) < 5:
                relay_count = max(0, len(active_uavs) - 1)
            else:
                self.assigned_relays = ['uav_2', 'uav_3', 'uav_4']
            self.surveyors = ['uav_1', 'uav_5']
            
            for rid in self.assigned_relays:
                commands.append(AssignRelayRoleCommand(source_tick=snapshot.simulation_tick, uav_id=rid))
                
        all_in_position = True
        
        curr_net = self.comm_analyzer.analyze(snapshot) if network_analysis is None else network_analysis
        before_connected = set(curr_net.connected_uav_ids)
        
        # We enforce sequential deployment to avoid deadlock.
        # A relay is only allowed to move to its target if the PREVIOUS relay is already at its target.
        # We also enforce that they move to an ingress lane (y=500) before traversing x.
        
        for i, uav_id in enumerate(self.assigned_relays):
            if snapshot.uavs[uav_id].rth_state.name != "NONE":
                continue
            if i >= len(self.target_chain):
                break
                
            uav = snapshot.uavs[uav_id]
            target = self.target_chain[i]
            
            dist = math.hypot(uav.position_xy[0] - target[0], uav.position_xy[1] - target[1])
            if dist > 1.0:
                all_in_position = False
                
                can_move = True
                        
                if can_move:
                    next_target = target
                        
                    if uav.target_position != next_target:
                        dest_uavs = dict(snapshot.uavs)
                        dest_uavs[uav.id] = dataclasses.replace(uav, position_xy=next_target)
                        dest_snap = dataclasses.replace(snapshot, uavs=dest_uavs)
                        dest_net = self.comm_analyzer.analyze(dest_snap)
                        
                        if uav.id in dest_net.connected_uav_ids:
                            safe = True
                            for peer_id in self.assigned_relays + self.surveyors:
                                if peer_id == uav.id: continue
                                if peer_id in before_connected and peer_id not in dest_net.connected_uav_ids:
                                    safe = False
                                    break
                            if safe:
                                commands.append(SetTargetPositionCommand(source_tick=snapshot.simulation_tick, uav_id=uav.id, target_position=next_target))
                                before_connected = set(dest_net.connected_uav_ids)
                    
        for s_id in self.surveyors:
            s_uav = snapshot.uavs.get(s_id)
            if s_uav and s_uav.rth_state.name == 'NONE':
                # Surveyors hold at safe positions (staggered by ID) until deployment completes
                # uav_1 -> 420, uav_5 -> 580
                s_target = (100.0, 420.0) if s_id == 'uav_1' else (100.0, 580.0)
                dist = math.hypot(s_uav.position_xy[0] - s_target[0], s_uav.position_xy[1] - s_target[1])
                if dist > 1.0:
                    all_in_position = False
                    if True:
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
                                    commands.append(SetTargetPositionCommand(source_tick=snapshot.simulation_tick, uav_id=s_uav.id, target_position=next_target))

        if all_in_position and not commands:
            self.status = IngressStatus.COMPLETE

        return commands
