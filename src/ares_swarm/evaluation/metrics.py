"""Official evaluation metrics pipeline for ARES Swarm missions."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

from ..core.enums import TaskStatus
from ..core.models import StateSnapshot


@dataclass
class MissionMetricsReport:
    """Official multi-dimensional evaluation report for ARES Swarm."""
    # Mission metrics
    tasks_total: int = 0
    tasks_spawned: int = 0
    tasks_assigned: int = 0
    tasks_completed: int = 0
    tasks_expired: int = 0
    mission_completion_rate: float = 0.0
    completion_time_s: float = 0.0
    priority_weighted_score: float = 0.0
    total_energy_consumed_wh: float = 0.0

    # Communication metrics
    model_estimated_route_pdr: float | None = None
    model_estimated_route_latency_ms: float | None = None
    connectivity_availability: float = 1.0
    downtime_s: float = 0.0

    # Resilience metrics (A1/relay/failure-recovery deferred)
    relay_reallocations: int | None = None
    recovery_time_s: float | None = None
    network_reconfiguration_efficiency: float | None = None
    performance_after_failure: dict[str, Any] | None = None
    resilience_status: str = "NOT_APPLICABLE_M0"

    # Safety metrics
    separation_violation_count: int = 0
    min_inter_uav_separation_m: float = float("inf")
    battery_exhaustion_count: int = 0
    geofence_violation_count: int = 0

    # Detection & Telemetry metrics
    total_detections: int = 0
    reports_delivered: int = 0
    reports_deadline_exceeded: int = 0
    reporting_compliance_ratio: float = 1.0
    mean_reporting_latency_s: float | None = None
    max_reporting_latency_s: float | None = None
    per_uav_detection_counts: dict[str, int] = field(default_factory=dict)
    per_uav_delivered_counts: dict[str, int] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "mission": {
                "tasks_total": self.tasks_total,
                "tasks_spawned": self.tasks_spawned,
                "tasks_assigned": self.tasks_assigned,
                "tasks_completed": self.tasks_completed,
                "tasks_expired": self.tasks_expired,
                "mission_completion_rate": round(self.mission_completion_rate, 4),
                "completion_time_s": round(self.completion_time_s, 2),
                "priority_weighted_score": round(self.priority_weighted_score, 4),
                "total_energy_consumed_wh": round(self.total_energy_consumed_wh, 4),
            },
            "communication": {
                "model_estimated_route_pdr": round(self.model_estimated_route_pdr, 4) if self.model_estimated_route_pdr is not None else None,
                "model_estimated_route_latency_ms": round(self.model_estimated_route_latency_ms, 2) if self.model_estimated_route_latency_ms is not None else None,
                "connectivity_availability": round(self.connectivity_availability, 4),
                "downtime_s": round(self.downtime_s, 2),
            },
            "resilience": {
                "status": self.resilience_status,
                "note": "Dynamic relay allocation and failure recovery deferred to A1+/M1",
                "relay_reallocations": self.relay_reallocations,
                "recovery_time_s": self.recovery_time_s,
                "network_reconfiguration_efficiency": self.network_reconfiguration_efficiency,
                "performance_after_failure": self.performance_after_failure,
            },
            "safety": {
                "separation_violation_count": self.separation_violation_count,
                "min_inter_uav_separation_m": (
                    round(self.min_inter_uav_separation_m, 2)
                    if self.min_inter_uav_separation_m != float("inf") and self.min_inter_uav_separation_m is not None
                    else None
                ),
                "battery_exhaustion_count": self.battery_exhaustion_count,
                "geofence_violation_count": self.geofence_violation_count,
            },
            "telemetry": {
                "total_detections": self.total_detections,
                "reports_delivered": self.reports_delivered,
                "reports_deadline_exceeded": self.reports_deadline_exceeded,
                "reporting_compliance_ratio": round(self.reporting_compliance_ratio, 4),
                "mean_reporting_latency_s": round(self.mean_reporting_latency_s, 3) if self.mean_reporting_latency_s is not None else None,
                "max_reporting_latency_s": round(self.max_reporting_latency_s, 3) if self.max_reporting_latency_s is not None else None,
                "per_uav_detection_counts": dict(self.per_uav_detection_counts),
                "per_uav_delivered_counts": dict(self.per_uav_delivered_counts),
            },
        }


def compute_mission_metrics(
    step_history: Sequence[Any],
    initial_snapshot: StateSnapshot,
    final_snapshot: StateSnapshot,
    dt: float = 1.0,
    safety_report: Any = None,
    telemetry_manager: Any = None,
) -> MissionMetricsReport:
    """Compute official benchmark metrics from simulation step history and snapshots."""
    report = MissionMetricsReport()

    # 1. Mission Metrics
    tasks = final_snapshot.tasks
    sim_time = final_snapshot.simulation_time
    report.tasks_total = len(tasks)
    report.tasks_spawned = sum(1 for t in tasks.values() if t.created_time <= sim_time)
    report.tasks_completed = sum(1 for t in tasks.values() if t.status == TaskStatus.COMPLETE)
    # Tasks currently assigned and in-progress (do not count completed as merely assigned)
    report.tasks_assigned = sum(
        1 for t in tasks.values()
        if t.status in (TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS)
    )
    # Tasks expired/unreachable: spawned, not completed, and deadline expired, unreachable, or deferred
    report.tasks_expired = sum(
        1 for t in tasks.values()
        if t.status != TaskStatus.COMPLETE and (
            t.status in (TaskStatus.UNREACHABLE, TaskStatus.DEFERRED) or
            (t.created_time <= sim_time and t.deadline > 0.0 and sim_time >= t.deadline)
        )
    )

    if report.tasks_total > 0:
        report.mission_completion_rate = report.tasks_completed / report.tasks_total
        total_prio = sum(float(t.priority) for t in tasks.values())
        completed_prio = sum(float(t.priority) for t in tasks.values() if t.status == TaskStatus.COMPLETE)
        report.priority_weighted_score = (completed_prio / total_prio) if total_prio > 0 else 0.0

    # Completion time: earliest time when all tasks reached COMPLETE, else final simulation time
    completion_time = final_snapshot.simulation_time
    if report.tasks_total > 0 and report.tasks_completed == report.tasks_total:
        for step in step_history:
            if all(step.snapshot.tasks[tid].status == TaskStatus.COMPLETE for tid in tasks):
                completion_time = step.simulation_time
                break
    report.completion_time_s = completion_time

    # Energy consumed
    initial_energy = sum(u.battery_energy for u in initial_snapshot.uavs.values())
    final_energy = sum(u.battery_energy for u in final_snapshot.uavs.values())
    report.total_energy_consumed_wh = max(0.0, initial_energy - final_energy)

    # 2. Communication Metrics
    pdr_samples: list[float] = []
    latency_samples: list[float] = []
    connectivity_slots = 0
    connected_slots = 0
    downtime_ticks = 0

    for step in step_history:
        net = step.network_analysis
        active_uavs = [uid for uid, u in step.snapshot.uavs.items() if u.active]
        edge_latencies = {}
        for link in net.network.links:
            key = (min(link.source_id, link.target_id), max(link.source_id, link.target_id))
            edge_latencies[key] = link.latency_ms

        # Use route_pdr_to_gcs instead of arbitrary edges
        for uid in active_uavs:
            route_pdr = net.route_pdr_to_gcs.get(uid)
            if route_pdr is not None and route_pdr > 0.0:
                pdr_samples.append(route_pdr)

            # Derive end-to-end latency for active routes
            route = net.routes_to_gcs.get(uid)
            if route and len(route) >= 2:
                route_lat = sum(
                    edge_latencies.get((min(a, b), max(a, b)), 0.0) 
                    for a, b in zip(route, route[1:])
                )
                latency_samples.append(route_lat)

        # Reachability
        has_disconnected_node = False
        for uid in active_uavs:
            connectivity_slots += 1
            if uid in net.connected_uav_ids:
                connected_slots += 1
            else:
                has_disconnected_node = True

        if has_disconnected_node:
            downtime_ticks += 1

    report.model_estimated_route_pdr = (sum(pdr_samples) / len(pdr_samples)) if pdr_samples else None
    report.model_estimated_route_latency_ms = (sum(latency_samples) / len(latency_samples)) if latency_samples else None
    report.connectivity_availability = (connected_slots / connectivity_slots) if connectivity_slots > 0 else 1.0
    report.downtime_s = downtime_ticks * dt

    # 3. Resilience Metrics
    # Dynamic relay reallocation and failure recovery are deferred to A1+/M1 in official plan.
    # Explicitly marked as not applicable in M0 rather than fabricating values.
    report.relay_reallocations = None
    report.recovery_time_s = None
    report.network_reconfiguration_efficiency = None
    report.performance_after_failure = None
    report.resilience_status = "NOT_APPLICABLE_M0"

    # 4. Safety Metrics & Hard Constraints
    if safety_report:
        report.separation_violation_count = safety_report.separation_violations_count
        report.min_inter_uav_separation_m = safety_report.min_observed_separation_m
        report.battery_exhaustion_count = safety_report.battery_exhaustions_count
        report.geofence_violation_count = safety_report.geofence_violations_count

    # 5. Detection & Telemetry Metrics
    if telemetry_manager:
        telem_metrics = telemetry_manager.get_metrics()
        report.total_detections = telem_metrics.get("total_detections", 0)
        report.reports_delivered = telem_metrics.get("reports_delivered", 0)
        report.reports_deadline_exceeded = telem_metrics.get("reports_deadline_exceeded", 0)
        report.reporting_compliance_ratio = telem_metrics.get("reporting_compliance_ratio", 1.0)
        report.mean_reporting_latency_s = telem_metrics.get("mean_reporting_latency_s")
        report.max_reporting_latency_s = telem_metrics.get("max_reporting_latency_s")
        report.per_uav_detection_counts = telem_metrics.get("per_uav_detection_counts", {})
        report.per_uav_delivered_counts = telem_metrics.get("per_uav_delivered_counts", {})

    return report
