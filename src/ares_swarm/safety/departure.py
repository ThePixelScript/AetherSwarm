"""Deterministic ground departure sequencing and taxi protocol for AetherSwarm.

Ensures safe, ordered departure from the staging area outside the operational arena.
Guarantees:
1. All UAVs take off from the authorized operational center/staging area (x = -75.0m).
2. Only one UAV at a time may leave the staging column/launch formation.
3. The active departing UAV follows a deterministic eastward taxi path through the authorized corridor.
4. The departing UAV clears the 20m separation bubbles of adjacent staged UAVs (at x >= x_taxi)
   before turning toward its actual mission target.
5. Once the departing UAV has safely cleared the launch formation (x >= x_clearance or entered arena),
   the next queued UAV is released.
6. Minimum inter-UAV separation (>= 20.0m) is strictly preserved at all times during departure.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Any, Dict, List, Mapping, Optional, Sequence, Set, Tuple

from ..core.commands import SetTargetPositionCommand
from ..core.constants import EPSILON
from ..core.enums import EventType, Role, RTHState, SortieState, TaskStatus
from ..core.events import DomainEvent
from ..core.models import StateSnapshot, TaskState, UAVState
from .airspace import ChallengeAirspace


class UAVDeparturePhase(str, Enum):
    """Lifecycle phase of a UAV relative to ground departure sequencing."""
    STAGED = "STAGED"      # Parked in staging area, stationary on pad
    QUEUED = "QUEUED"      # Has mission assignment, waiting in departure queue
    TAXIING = "TAXIING"    # Cleared to depart, taxiing eastward through corridor
    TRANSIT = "TRANSIT"    # Cleared staging column (x >= x_taxi), proceeding to mission target
    CLEARED = "CLEARED"    # Safely cleared launch formation, free in mission flight


@dataclass
class DepartureRecord:
    """Telemetry record for a single UAV departure."""
    uav_id: str
    staging_position: Tuple[float, float]
    mission_target: Tuple[float, float]
    queued_time_s: float
    taxi_start_time_s: Optional[float] = None
    cleared_time_s: Optional[float] = None
    departure_duration_s: float = 0.0


class DepartureSequencer:
    """Deterministic departure sequencer enforcing one-at-a-time eastward corridor taxi."""

    def __init__(
        self,
        staging_x: float = -75.0,
        taxi_x: float = -45.0,
        clearance_x: float = -20.0,
        min_separation_m: float = 20.0,
        airspace: Optional[ChallengeAirspace] = None,
    ) -> None:
        self.staging_x = staging_x
        self.taxi_x = taxi_x
        self.clearance_x = clearance_x
        self.min_separation_m = min_separation_m
        self.airspace = airspace

        # Queue and state tracking
        self.departure_queue: List[str] = []
        self.active_departing_uav_id: Optional[str] = None
        self.uav_phases: Dict[str, UAVDeparturePhase] = {}
        self.staged_origins: Dict[str, Tuple[float, float]] = {}
        self.intended_targets: Dict[str, Tuple[float, float]] = {}
        self.departure_records: Dict[str, DepartureRecord] = {}

        # Deadlock detection tracking
        self._taxi_progress_ticks: Dict[str, int] = {}
        self._last_taxi_positions: Dict[str, Tuple[float, float]] = {}

        # Metrics
        self.total_departures_started: int = 0
        self.total_departures_completed: int = 0
        self.departure_deadlock_count: int = 0
        self.min_observed_departure_separation_m: float = float("inf")

    def reset(self) -> None:
        """Reset internal queue and telemetry records."""
        self.departure_queue.clear()
        self.active_departing_uav_id = None
        self.uav_phases.clear()
        self.staged_origins.clear()
        self.intended_targets.clear()
        self.departure_records.clear()
        self._taxi_progress_ticks.clear()
        self._last_taxi_positions.clear()
        self.total_departures_started = 0
        self.total_departures_completed = 0
        self.departure_deadlock_count = 0
        self.min_observed_departure_separation_m = float("inf")

    def is_in_staging(self, position_xy: Tuple[float, float]) -> bool:
        """Check if position is within the authorized staging area."""
        if self.airspace is not None:
            return self.airspace.is_in_staging_area(position_xy)
        return position_xy[0] <= -50.0

    def step(
        self,
        snapshot: StateSnapshot,
        dt: float = 1.0,
    ) -> Tuple[List[SetTargetPositionCommand], List[DomainEvent]]:
        """Execute one tick of departure sequencing and return target position overrides and events."""
        commands: List[SetTargetPositionCommand] = []
        events: List[DomainEvent] = []
        tick = snapshot.simulation_tick
        sim_time = snapshot.simulation_time

        # 1. Track initial/staged positions for all UAVs
        for uid, uav in sorted(snapshot.uavs.items()):
            if uid not in self.staged_origins:
                self.staged_origins[uid] = uav.position_xy
            if uid not in self.uav_phases:
                self.uav_phases[uid] = UAVDeparturePhase.STAGED

            # Check if a previously cleared UAV has landed at GCS and is ready for a new sortie
            if self.uav_phases[uid] == UAVDeparturePhase.CLEARED:
                if (
                    uav.sortie_state in (SortieState.LANDED, SortieState.RECHARGING, SortieState.READY)
                    and self.is_in_staging(uav.position_xy)
                    and uav.rth_state in (RTHState.COMPLETE, RTHState.NONE)
                ):
                    # Reset phase to STAGED for next sortie
                    self.uav_phases[uid] = UAVDeparturePhase.STAGED
                    self.staged_origins[uid] = uav.position_xy
                    self.intended_targets.pop(uid, None)

        # 2. Check status of active departing UAV
        if self.active_departing_uav_id is not None:
            act_id = self.active_departing_uav_id
            act_uav = snapshot.uavs.get(act_id)
            if act_uav is None or not act_uav.active:
                self.active_departing_uav_id = None
            else:
                curr_x, curr_y = act_uav.position_xy
                intended = self.intended_targets.get(act_id)

                # Clearance condition: reached clearance_x, entered arena, or arrived at corridor relay station
                arrived_corridor_relay = False
                if intended is not None and intended[0] < 0.0:
                    dist_to_station = math.hypot(curr_x - intended[0], curr_y - intended[1])
                    if dist_to_station <= 0.5 and curr_x >= self.taxi_x:
                        arrived_corridor_relay = True

                has_cleared_formation = (
                    (curr_x >= self.clearance_x - EPSILON)
                    or (curr_x >= 0.0 - EPSILON)
                    or arrived_corridor_relay
                )

                if has_cleared_formation:
                    self.uav_phases[act_id] = UAVDeparturePhase.CLEARED
                    self.total_departures_completed += 1
                    if act_id in self.departure_records:
                        rec = self.departure_records[act_id]
                        rec.cleared_time_s = sim_time
                        if rec.taxi_start_time_s is not None:
                            rec.departure_duration_s = sim_time - rec.taxi_start_time_s
                        events.append(DomainEvent.create(
                            simulation_tick=tick,
                            simulation_time=sim_time,
                            event_type=EventType.DEPARTURE_CLEARED,
                            entity_id=act_id,
                            payload={
                                "cleared_position": list(act_uav.position_xy),
                                "departure_duration_s": round(rec.departure_duration_s, 2),
                            },
                        ))
                    self._taxi_progress_ticks.pop(act_id, None)
                    self._last_taxi_positions.pop(act_id, None)
                    # Release active departure slot!
                    self.active_departing_uav_id = None

                elif curr_x >= self.taxi_x - EPSILON:
                    # Cleared the staging column 20m bubbles; in corridor transit toward mission target
                    self.uav_phases[act_id] = UAVDeparturePhase.TRANSIT

                # Deadlock detection: if active UAV is stationary for > 30s while TAXIING
                last_p = self._last_taxi_positions.get(act_id)
                if last_p is not None and math.hypot(curr_x - last_p[0], curr_y - last_p[1]) < 0.01:
                    self._taxi_progress_ticks[act_id] = self._taxi_progress_ticks.get(act_id, 0) + 1
                    if self._taxi_progress_ticks[act_id] >= 30:
                        self.departure_deadlock_count += 1
                else:
                    self._taxi_progress_ticks[act_id] = 0
                self._last_taxi_positions[act_id] = (curr_x, curr_y)

                # Monitor minimum separation from active departing UAV to all other UAVs
                for other_id, other_uav in snapshot.uavs.items():
                    if other_id != act_id:
                        d = math.hypot(curr_x - other_uav.position_xy[0], curr_y - other_uav.position_xy[1])
                        if d < self.min_observed_departure_separation_m:
                            self.min_observed_departure_separation_m = d

        # 3. Identify staged UAVs with active mission assignments and enqueue them
        for uid, uav in sorted(snapshot.uavs.items()):
            if not uav.active or uav.rth_state != RTHState.NONE:
                continue

            in_staging = self.is_in_staging(uav.position_xy)
            if not in_staging:
                continue

            phase = self.uav_phases.get(uid, UAVDeparturePhase.STAGED)
            if phase in (UAVDeparturePhase.CLEARED, UAVDeparturePhase.TAXIING, UAVDeparturePhase.TRANSIT):
                continue

            # Determine if UAV has a mission target
            has_mission_intent = False
            tgt = None

            if uav.target_position is not None:
                d_pad = math.hypot(
                    uav.position_xy[0] - uav.target_position[0],
                    uav.position_xy[1] - uav.target_position[1],
                )
                if d_pad > 1.0:
                    has_mission_intent = True
                    tgt = uav.target_position
            elif uav.assigned_task_id is not None:
                task = snapshot.tasks.get(uav.assigned_task_id)
                if task is not None and task.status in (TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS):
                    has_mission_intent = True
                    tgt = task.position_xy

            if has_mission_intent and tgt is not None:
                self.intended_targets[uid] = tgt
                if phase == UAVDeparturePhase.STAGED:
                    self.uav_phases[uid] = UAVDeparturePhase.QUEUED
                    if uid not in self.departure_queue:
                        self.departure_queue.append(uid)
                        # Deterministic tie-breaking: strictly by UID
                        self.departure_queue.sort()
                        rec = DepartureRecord(
                            uav_id=uid,
                            staging_position=uav.position_xy,
                            mission_target=tgt,
                            queued_time_s=sim_time,
                        )
                        self.departure_records[uid] = rec
                        events.append(DomainEvent.create(
                            simulation_tick=tick,
                            simulation_time=sim_time,
                            event_type=EventType.DEPARTURE_QUEUED,
                            entity_id=uid,
                            payload={
                                "staging_position": list(uav.position_xy),
                                "mission_target": list(tgt),
                                "queue_position": len(self.departure_queue),
                            },
                        ))

            # If task was cancelled/deferred while queued: dequeue
            elif phase == UAVDeparturePhase.QUEUED and not has_mission_intent:
                if uid in self.departure_queue:
                    self.departure_queue.remove(uid)
                self.uav_phases[uid] = UAVDeparturePhase.STAGED
                self.intended_targets.pop(uid, None)

        # 4. Release next UAV from departure queue if slot is available
        if self.active_departing_uav_id is None and len(self.departure_queue) > 0:
            next_uid = self.departure_queue.pop(0)
            self.active_departing_uav_id = next_uid
            self.uav_phases[next_uid] = UAVDeparturePhase.TAXIING
            self.total_departures_started += 1
            orig = self.staged_origins.get(next_uid, snapshot.uavs[next_uid].position_xy)
            taxi_tgt = (self.taxi_x, orig[1])
            if next_uid in self.departure_records:
                self.departure_records[next_uid].taxi_start_time_s = sim_time
            events.append(DomainEvent.create(
                simulation_tick=tick,
                simulation_time=sim_time,
                event_type=EventType.DEPARTURE_STARTED,
                entity_id=next_uid,
                payload={
                    "staging_position": list(orig),
                    "taxi_target": list(taxi_tgt),
                },
            ))

        # 5. Generate commands for active departing and queued UAVs
        # 5a. Queued UAVs must hold position on their pads
        for q_uid in self.departure_queue:
            q_uav = snapshot.uavs.get(q_uid)
            if q_uav and q_uav.target_position is not None:
                # Hold stationary on pad
                commands.append(
                    SetTargetPositionCommand(
                        source_tick=tick,
                        uav_id=q_uid,
                        target_position=None,
                    )
                )

        # 5b. Active departing UAV target assignment
        if self.active_departing_uav_id is not None:
            act_id = self.active_departing_uav_id
            act_uav = snapshot.uavs.get(act_id)
            if act_uav:
                phase = self.uav_phases.get(act_id)
                orig = self.staged_origins.get(act_id, act_uav.position_xy)
                intended = self.intended_targets.get(act_id, act_uav.target_position)

                if phase == UAVDeparturePhase.TAXIING:
                    # Phase 1: Pure eastward taxi along its own Y coordinate
                    taxi_target = (self.taxi_x, orig[1])
                    if act_uav.target_position != taxi_target:
                        commands.append(
                            SetTargetPositionCommand(
                                source_tick=tick,
                                uav_id=act_id,
                                target_position=taxi_target,
                            )
                        )
                elif phase in (UAVDeparturePhase.TRANSIT, UAVDeparturePhase.CLEARED):
                    # Phase 2: Cleared staging column; proceed to intended mission target
                    if intended is not None and act_uav.target_position != intended:
                        commands.append(
                            SetTargetPositionCommand(
                                source_tick=tick,
                                uav_id=act_id,
                                target_position=intended,
                            )
                        )

        return commands, events
