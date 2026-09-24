"""Authoritative Detection & Telemetry Reporting Manager for AetherSwarm UAV-X."""
from __future__ import annotations

import dataclasses
import math
from typing import Any, Mapping, Optional, Sequence, Tuple

from ..core.constants import EPSILON
from ..core.enums import EventType, RTHState, TelemetryStatus
from ..core.events import DomainEvent
from ..core.models import StateSnapshot, TelemetryReport, UAVState
from ..interfaces.communication import NetworkAnalysis
from ..communication.analysis import route_latency_ms
from ..simulation.scenario import DetectionPipelineConfig


class DetectionManager:
    """Manages sensor perception discovery, delay-tolerant telemetry buffering, and GCS reporting."""

    def __init__(
        self,
        config: DetectionPipelineConfig,
        gcs_position: Tuple[float, float] = (0.0, 0.0),
    ) -> None:
        self.config = config
        self.gcs_position = gcs_position
        self.authoritative_reports: dict[str, TelemetryReport] = {}
        self.pending_reports: list[str] = []
        self.detected_task_ids: set[str] = set()
        self.sightings_log: list[dict[str, Any]] = []
        self.event_counter: int = 0

    def reset(self) -> None:
        """Reset all internal telemetry and perception state."""
        self.authoritative_reports.clear()
        self.pending_reports.clear()
        self.detected_task_ids.clear()
        self.sightings_log.clear()
        self.event_counter = 0

    def step_perception(
        self,
        snapshot: StateSnapshot,
        flight_records: Optional[Mapping[str, Any]] = None,
    ) -> list[DomainEvent]:
        """Evaluate sensor FOV intersections for eligible airborne UAVs against eligible POIs."""
        if not self.config.enabled:
            return []

        # 1. Determine eligible active airborne UAVs
        airborne_uavs: list[UAVState] = []
        for uav_id, uav in sorted(snapshot.uavs.items()):
            if not uav.active or uav.rth_state == RTHState.COMPLETE:
                continue
            is_airborne = False
            if flight_records and uav_id in flight_records:
                is_airborne = bool(flight_records[uav_id].is_airborne)
            else:
                dx = uav.position_xy[0] - self.gcs_position[0]
                dy = uav.position_xy[1] - self.gcs_position[1]
                dist_gcs = math.hypot(dx, dy)
                speed = math.hypot(uav.velocity_xy[0], uav.velocity_xy[1])
                is_airborne = bool(dist_gcs > 1.0 or speed > EPSILON)
            if is_airborne:
                airborne_uavs.append(uav)

        # 2. Determine eligible spawned POIs (must already exist in arena)
        eligible_tasks = [
            t for t in snapshot.tasks.values()
            if t.created_time <= snapshot.simulation_time + EPSILON
        ]

        # 3. Find all candidate detections
        candidates: list[Tuple[str, str, Any, float]] = []
        for t in eligible_tasks:
            for uav in airborne_uavs:
                dx = uav.position_xy[0] - t.position_xy[0]
                dy = uav.position_xy[1] - t.position_xy[1]
                dist = math.hypot(dx, dy)
                if dist <= self.config.sensor_fov_radius_m + EPSILON:
                    candidates.append((t.id, uav.id, t, dist))

        # 4. Deterministic tie-breaking across multiple detections: (task_id, uav_id)
        candidates.sort(key=lambda x: (x[0], x[1]))

        events: list[DomainEvent] = []
        for task_id, uav_id, task_obj, dist in candidates:
            # First-detection rule: only the first sighting establishes authoritative report
            if task_id in self.detected_task_ids:
                self.sightings_log.append({
                    "task_id": task_id,
                    "uav_id": uav_id,
                    "time": snapshot.simulation_time,
                    "distance": dist,
                    "duplicate": True,
                })
                continue

            self.detected_task_ids.add(task_id)
            report = TelemetryReport(
                task_id=task_id,
                detecting_uav_id=uav_id,
                t_detect=snapshot.simulation_time,
                t_report_generated=snapshot.simulation_time + self.config.processing_delay_s,
                status=TelemetryStatus.PENDING,
            )
            self.authoritative_reports[task_id] = report
            self.pending_reports.append(task_id)

            self.event_counter += 1
            events.append(
                DomainEvent.create(
                    simulation_tick=snapshot.simulation_tick,
                    simulation_time=snapshot.simulation_time,
                    event_type=EventType.POI_DETECTED,
                    entity_id=uav_id,
                    payload={
                        "task_id": task_id,
                        "uav_id": uav_id,
                        "t_detect": snapshot.simulation_time,
                        "position": list(task_obj.position_xy),
                        "distance": round(dist, 3),
                    },
                    sequence=self.event_counter,
                )
            )

        return events

    def step_telemetry(
        self,
        snapshot: StateSnapshot,
        net_analysis: NetworkAnalysis,
    ) -> list[DomainEvent]:
        """Disseminate queued telemetry packets across the current communication topology."""
        if not self.config.enabled or not self.pending_reports:
            return []

        events: list[DomainEvent] = []
        to_process = sorted(self.pending_reports)
        for task_id in to_process:
            report = self.authoritative_reports[task_id]
            if report.status in (TelemetryStatus.DELIVERED, TelemetryStatus.DEADLINE_EXCEEDED):
                if task_id in self.pending_reports:
                    self.pending_reports.remove(task_id)
                continue
            if snapshot.simulation_time < report.t_report_generated:
                # Still within processing delay
                continue

            route = net_analysis.routes_to_gcs.get(report.detecting_uav_id)
            elapsed_s = snapshot.simulation_time - report.t_detect

            if route is not None:
                # Analytical delivery model: We immediately evaluate transmission success and 
                # predicted GCS arrival time (t_gcs) using the current topology's canonical route. 
                # We do not simulate packet-level queuing over future timesteps. Deadline 
                # compliance is strictly evaluated against this predicted t_gcs.
                canonical_latency_ms = route_latency_ms(net_analysis, route)
                if canonical_latency_ms is None:
                    # Missing or unusable link Canonical Route; treat as unavailable/unknown
                    pass
                else:
                    hop_count = max(0, len(route) - 1)
                    model_latency_s = canonical_latency_ms / 1000.0
                    
                    t_gcs = snapshot.simulation_time + model_latency_s
                    latency_s = max(0.0, t_gcs - report.t_detect)

                    if latency_s <= self.config.reporting_deadline_s + EPSILON:
                        status = TelemetryStatus.DELIVERED
                        evt_type = EventType.TELEMETRY_DELIVERED
                    else:
                        status = TelemetryStatus.DEADLINE_EXCEEDED
                        evt_type = EventType.TELEMETRY_DEADLINE_EXCEEDED

                    updated = dataclasses.replace(
                        report,
                        t_gcs_received=t_gcs,
                        hop_count=hop_count,
                        route=route,
                        reporting_latency_s=latency_s,
                        status=status,
                    )
                    self.authoritative_reports[task_id] = updated
                    self.pending_reports.remove(task_id)

                    self.event_counter += 1
                    events.append(
                        DomainEvent.create(
                            simulation_tick=snapshot.simulation_tick,
                            simulation_time=snapshot.simulation_time,
                            event_type=evt_type,
                            entity_id=report.detecting_uav_id,
                            payload={
                                "task_id": task_id,
                                "uav_id": report.detecting_uav_id,
                                "t_detect": report.t_detect,
                                "t_gcs": t_gcs,
                                "latency_s": round(latency_s, 3),
                                "hop_count": hop_count,
                                "route": list(route),
                                "status": status.value,
                            },
                            sequence=self.event_counter,
                        )
                    )
            else:
                # No route to GCS currently available
                if elapsed_s > self.config.reporting_deadline_s + EPSILON:
                    # Deadline expired while waiting for route in buffer
                    status = TelemetryStatus.DEADLINE_EXCEEDED
                    updated = dataclasses.replace(
                        report,
                        status=status,
                        reporting_latency_s=elapsed_s,
                    )
                    self.authoritative_reports[task_id] = updated
                    self.pending_reports.remove(task_id)

                    self.event_counter += 1
                    events.append(
                        DomainEvent.create(
                            simulation_tick=snapshot.simulation_tick,
                            simulation_time=snapshot.simulation_time,
                            event_type=EventType.TELEMETRY_DEADLINE_EXCEEDED,
                            entity_id=report.detecting_uav_id,
                            payload={
                                "task_id": task_id,
                                "uav_id": report.detecting_uav_id,
                                "t_detect": report.t_detect,
                                "elapsed_s": round(elapsed_s, 3),
                                "status": status.value,
                                "reason": "BUFFER_TIMEOUT_NO_ROUTE",
                            },
                            sequence=self.event_counter,
                        )
                    )
                else:
                    # Packet remains buffered as PENDING
                    pass

        return events

    def get_metrics(self) -> dict[str, Any]:
        """Compute summary telemetry metrics for the mission report."""
        total = len(self.authoritative_reports)
        delivered = sum(
            1 for r in self.authoritative_reports.values()
            if r.status == TelemetryStatus.DELIVERED
        )
        expired = sum(
            1 for r in self.authoritative_reports.values()
            if r.status == TelemetryStatus.DEADLINE_EXCEEDED
        )
        compliance_ratio = (delivered / total) if total > 0 else 1.0

        delivered_latencies = [
            r.reporting_latency_s for r in self.authoritative_reports.values()
            if r.status == TelemetryStatus.DELIVERED and r.reporting_latency_s is not None
        ]
        mean_latency = (sum(delivered_latencies) / len(delivered_latencies)) if delivered_latencies else None
        max_latency = max(delivered_latencies) if delivered_latencies else None

        per_uav_detections: dict[str, int] = {}
        per_uav_delivered: dict[str, int] = {}
        for r in self.authoritative_reports.values():
            per_uav_detections[r.detecting_uav_id] = per_uav_detections.get(r.detecting_uav_id, 0) + 1
            if r.status == TelemetryStatus.DELIVERED:
                per_uav_delivered[r.detecting_uav_id] = per_uav_delivered.get(r.detecting_uav_id, 0) + 1

        return {
            "total_detections": total,
            "reports_delivered": delivered,
            "reports_deadline_exceeded": expired,
            "reporting_compliance_ratio": round(compliance_ratio, 4),
            "mean_reporting_latency_s": round(mean_latency, 3) if mean_latency is not None else None,
            "max_reporting_latency_s": round(max_latency, 3) if max_latency is not None else None,
            "per_uav_detection_counts": per_uav_detections,
            "per_uav_delivered_counts": per_uav_delivered,
        }
