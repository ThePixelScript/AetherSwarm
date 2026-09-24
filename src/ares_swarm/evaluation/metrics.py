"""Official evaluation metrics pipeline for ARES Swarm missions."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

from ..core.enums import EventType, RTHState, SortieState, TaskStatus, TelemetryStatus
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

    separation_violation_count: int = 0
    min_inter_uav_separation_m: float = float("inf")
    battery_exhaustion_count: int = 0
    geofence_violation_count: int = 0
    separation_interventions: int = 0
    per_uav_intervention_counts: dict[str, int] = field(default_factory=dict)
    geofence_interventions: int = 0
    per_uav_geofence_intervention_counts: dict[str, int] = field(default_factory=dict)
    min_boundary_clearance_m: float = float("inf")

    # Detection & Telemetry metrics
    total_detections: int = 0
    reports_delivered: int = 0
    reports_deadline_exceeded: int = 0
    reporting_compliance_ratio: float = 1.0
    mean_reporting_latency_s: float | None = None
    max_reporting_latency_s: float | None = None
    per_uav_detection_counts: dict[str, int] = field(default_factory=dict)
    per_uav_delivered_counts: dict[str, int] = field(default_factory=dict)

    # Phase 2: Sortie rotation and handoff metrics
    sorties_started: int = 0
    sorties_completed: int = 0
    recharge_count: int = 0
    RTH_count: int = 0
    task_handoffs: int = 0
    successful_task_reassignments: int = 0
    max_continuous_sortie_duration_s: float = 0.0
    battery_exhaustions: int = 0
    landing_deadlocks: int = 0
    UAVs_landed: int = 0
    UAVs_ready_at_end: int = 0
    per_uav_initial_battery: dict[str, float] = field(default_factory=dict)
    per_uav_min_battery: dict[str, float] = field(default_factory=dict)
    per_uav_rth_trigger_times: dict[str, list[float]] = field(default_factory=dict)
    per_uav_landing_times: dict[str, list[float]] = field(default_factory=dict)
    per_uav_recharge_start_times: dict[str, list[float]] = field(default_factory=dict)
    per_uav_recharge_completion_times: dict[str, list[float]] = field(default_factory=dict)
    per_uav_recharge_counts: dict[str, int] = field(default_factory=dict)
    total_recharge_duration_s: float = 0.0
    relay_handoffs_caused_by_rth: int = 0

    # Phase 3: Dynamic Relay Management metrics
    relay_assignments: int = 0
    relay_releases: int = 0
    relay_handoffs: int = 0
    relay_losses: int = 0
    relay_recovery_successes: int = 0
    connected_time_before_handoff: float = 0.0
    connected_time_after_handoff: float = 0.0
    network_reconfiguration_time_s: float | None = None

    # Phase 4: Connectivity-Aware Planning metrics
    connectivity_feasibility_checks: int = 0
    connectivity_feasible_assignments: int = 0
    connectivity_rejected_assignments: int = 0
    connectivity_deferred_tasks: int = 0
    relay_required_for_assignment: int = 0
    connectivity_preserved_during_task: float = 0.0
    reporting_deadline_success: int = 0
    reporting_deadline_failure: int = 0
    communication_induced_replans: int = 0

    # Phase 5: Multi-Hop Relay Planning metrics
    max_hop_count: int = 0
    mean_hop_count: float = 0.0
    relay_chains_created: int = 0
    relay_chain_handoffs: int = 0
    relay_chain_failures: int = 0
    relay_chain_recoveries: int = 0
    tasks_deferred_insufficient_relays: int = 0
    chain_maintenance_duration_s: float = 0.0

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
                "min_boundary_clearance_m": (
                    round(self.min_boundary_clearance_m, 2)
                    if self.min_boundary_clearance_m != float("inf") and self.min_boundary_clearance_m is not None
                    else None
                ),
                "separation_interventions": self.separation_interventions,
                "per_uav_intervention_counts": dict(self.per_uav_intervention_counts),
                "geofence_interventions": self.geofence_interventions,
                "per_uav_geofence_intervention_counts": dict(self.per_uav_geofence_intervention_counts),
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
            "sortie_rotation": {
                "sorties_started": self.sorties_started,
                "sorties_completed": self.sorties_completed,
                "recharge_count": self.recharge_count,
                "RTH_count": self.RTH_count,
                "task_handoffs": self.task_handoffs,
                "successful_task_reassignments": self.successful_task_reassignments,
                "max_continuous_sortie_duration_s": round(self.max_continuous_sortie_duration_s, 2),
                "battery_exhaustions": self.battery_exhaustions,
                "landing_deadlocks": self.landing_deadlocks,
                "UAVs_landed": self.UAVs_landed,
                "UAVs_ready_at_end": self.UAVs_ready_at_end,
                "per_uav_initial_battery": dict(self.per_uav_initial_battery),
                "per_uav_min_battery": dict(self.per_uav_min_battery),
                "per_uav_rth_trigger_times": dict(self.per_uav_rth_trigger_times),
                "per_uav_landing_times": dict(self.per_uav_landing_times),
                "per_uav_recharge_start_times": dict(self.per_uav_recharge_start_times),
                "per_uav_recharge_completion_times": dict(self.per_uav_recharge_completion_times),
                "per_uav_recharge_counts": dict(self.per_uav_recharge_counts),
                "total_recharge_duration_s": round(self.total_recharge_duration_s, 2),
                "relay_handoffs_caused_by_rth": self.relay_handoffs_caused_by_rth,
            },
            "relay_management": {
                "relay_assignments": self.relay_assignments,
                "relay_releases": self.relay_releases,
                "relay_handoffs": self.relay_handoffs,
                "relay_losses": self.relay_losses,
                "relay_recovery_successes": self.relay_recovery_successes,
                "connected_time_before_handoff": round(self.connected_time_before_handoff, 2),
                "connected_time_after_handoff": round(self.connected_time_after_handoff, 2),
                "network_reconfiguration_time_s": (
                    round(self.network_reconfiguration_time_s, 2)
                    if self.network_reconfiguration_time_s is not None else None
                ),
            },
            "connectivity_planning": {
                "connectivity_feasibility_checks": self.connectivity_feasibility_checks,
                "connectivity_feasible_assignments": self.connectivity_feasible_assignments,
                "connectivity_rejected_assignments": self.connectivity_rejected_assignments,
                "connectivity_deferred_tasks": self.connectivity_deferred_tasks,
                "relay_required_for_assignment": self.relay_required_for_assignment,
                "connectivity_preserved_during_task": round(self.connectivity_preserved_during_task, 2),
                "reporting_deadline_success": self.reporting_deadline_success,
                "reporting_deadline_failure": self.reporting_deadline_failure,
                "communication_induced_replans": self.communication_induced_replans,
                "max_hop_count": self.max_hop_count,
                "mean_hop_count": round(self.mean_hop_count, 2),
                "relay_chains_created": self.relay_chains_created,
                "relay_chain_handoffs": self.relay_chain_handoffs,
                "relay_chain_failures": self.relay_chain_failures,
                "relay_chain_recoveries": self.relay_chain_recoveries,
                "tasks_deferred_insufficient_relays": self.tasks_deferred_insufficient_relays,
                "chain_maintenance_duration_s": round(self.chain_maintenance_duration_s, 2),
            },
        }


def compute_mission_metrics(
    step_history: Sequence[Any],
    initial_snapshot: StateSnapshot,
    final_snapshot: StateSnapshot,
    dt: float = 1.0,
    safety_report: Any = None,
    telemetry_manager: Any = None,
    separation_enforcer: Any = None,
    geofence_enforcer: Any = None,
    relay_manager: Any = None,
    connectivity_planner: Any = None,
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
    if separation_enforcer:
        report.separation_interventions = separation_enforcer.total_interventions
        report.per_uav_intervention_counts = dict(separation_enforcer.per_uav_interventions)
    if geofence_enforcer:
        report.geofence_interventions = geofence_enforcer.total_interventions
        report.per_uav_geofence_intervention_counts = dict(geofence_enforcer.per_uav_interventions)
        report.min_boundary_clearance_m = geofence_enforcer.min_observed_clearance_m

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

    # 6. Sortie Rotation & Handoff Metrics
    all_events = []
    for step in step_history:
        evs = getattr(step, "events", ())
        if evs:
            all_events.extend(evs)

    if safety_report and safety_report.uav_flight_records:
        report.sorties_started = sum(max(1 if rec.takeoff_time is not None else 0, rec.sortie_count) for rec in safety_report.uav_flight_records.values())
        report.sorties_completed = max(
            sum(getattr(rec, "sorties_completed", 0) for rec in safety_report.uav_flight_records.values()),
            sum(1 for e in all_events if getattr(e, "event_type", None) == EventType.SORTIE_COMPLETED),
        )
        report.max_continuous_sortie_duration_s = getattr(safety_report, "max_observed_sortie_duration_s", 0.0)
        report.battery_exhaustions = getattr(safety_report, "battery_exhaustions_count", 0)
    else:
        report.sorties_started = sum(1 for e in all_events if getattr(e, "event_type", None) == EventType.SORTIE_STARTED)
        report.sorties_completed = sum(1 for e in all_events if getattr(e, "event_type", None) in (EventType.SORTIE_COMPLETED, EventType.UAV_LANDED))
        report.max_continuous_sortie_duration_s = 0.0
        report.battery_exhaustions = report.battery_exhaustion_count

    report.recharge_count = sum(1 for e in all_events if getattr(e, "event_type", None) == EventType.UAV_RECHARGED)
    report.RTH_count = sum(1 for e in all_events if getattr(e, "event_type", None) == EventType.RTH_TRIGGERED)
    report.task_handoffs = sum(1 for e in all_events if getattr(e, "event_type", None) == EventType.TASK_HANDOFF)
    report.successful_task_reassignments = sum(1 for e in all_events if getattr(e, "event_type", None) == EventType.TASK_REASSIGNED)

    final_uavs = final_snapshot.uavs
    report.UAVs_landed = sum(
        1 for u in final_uavs.values()
        if getattr(u, "sortie_state", None) == SortieState.LANDED or getattr(u, "rth_state", None) == RTHState.COMPLETE
    )
    report.UAVs_ready_at_end = sum(
        1 for u in final_uavs.values()
        if getattr(u, "sortie_state", None) == SortieState.READY
    )
    report.landing_deadlocks = sum(
        1 for u in final_uavs.values()
        if getattr(u, "rth_state", None) == RTHState.ACTIVE and getattr(u, "active", False)
    )

    # Battery & Recharge tracking per UAV
    report.per_uav_initial_battery = {uid: round(u.battery_energy, 2) for uid, u in initial_snapshot.uavs.items()}
    min_bat: dict[str, float] = {uid: u.battery_energy for uid, u in initial_snapshot.uavs.items()}
    for step in step_history:
        for uid, u in step.snapshot.uavs.items():
            if uid in min_bat:
                min_bat[uid] = min(min_bat[uid], u.battery_energy)
            else:
                min_bat[uid] = u.battery_energy
    report.per_uav_min_battery = {uid: round(val, 2) for uid, val in min_bat.items()}

    rth_triggers: dict[str, list[float]] = {}
    landing_times: dict[str, list[float]] = {}
    recharge_starts: dict[str, list[float]] = {}
    recharge_ends: dict[str, list[float]] = {}
    recharge_counts: dict[str, int] = {uid: 0 for uid in initial_snapshot.uavs}
    relay_rth_handoffs = 0

    for e in all_events:
        etype = getattr(e, "event_type", None)
        uid = getattr(e, "entity_id", None)
        t = round(float(getattr(e, "simulation_time", 0.0)), 2)
        if etype == EventType.RTH_TRIGGERED and uid:
            rth_triggers.setdefault(uid, []).append(t)
        elif etype == EventType.UAV_LANDED and uid:
            landing_times.setdefault(uid, []).append(t)
        elif etype == EventType.UAV_RECHARGING and uid:
            recharge_starts.setdefault(uid, []).append(t)
        elif etype == EventType.UAV_RECHARGED and uid:
            recharge_ends.setdefault(uid, []).append(t)
            recharge_counts[uid] = recharge_counts.get(uid, 0) + 1
        elif etype == EventType.RELAY_HANDOFF:
            payload = getattr(e, "payload", {})
            if "RTH" in str(payload).upper() or "RTH" in str(getattr(e, "reason", "")).upper():
                relay_rth_handoffs += 1

    report.per_uav_rth_trigger_times = rth_triggers
    report.per_uav_landing_times = landing_times
    report.per_uav_recharge_start_times = recharge_starts
    report.per_uav_recharge_completion_times = recharge_ends
    report.per_uav_recharge_counts = recharge_counts
    report.relay_handoffs_caused_by_rth = relay_rth_handoffs

    total_rech_s = 0.0
    for uid in initial_snapshot.uavs:
        starts = recharge_starts.get(uid, [])
        ends = recharge_ends.get(uid, [])
        for i, s in enumerate(starts):
            if i < len(ends):
                total_rech_s += max(0.0, ends[i] - s)
            else:
                total_rech_s += max(0.0, sim_time - s)
    report.total_recharge_duration_s = total_rech_s

    # 7. Dynamic Relay Management Metrics
    if relay_manager is not None:
        report.relay_assignments = relay_manager.relay_assignments
        report.relay_releases = relay_manager.relay_releases
        report.relay_handoffs = relay_manager.relay_handoffs
        report.relay_losses = relay_manager.relay_losses
        report.relay_recovery_successes = relay_manager.relay_recovery_successes
        report.connected_time_before_handoff = relay_manager.connected_time_before_handoff
        report.connected_time_after_handoff = relay_manager.connected_time_after_handoff
        report.network_reconfiguration_time_s = relay_manager.network_reconfiguration_time_s
        report.relay_chains_created = getattr(relay_manager, "relay_chains_created", 0)
        report.relay_chain_handoffs = getattr(relay_manager, "relay_chain_handoffs", 0)
        report.relay_chain_failures = getattr(relay_manager, "relay_chain_failures", 0)
        report.relay_chain_recoveries = getattr(relay_manager, "relay_chain_recoveries", 0)
        report.chain_maintenance_duration_s = getattr(relay_manager, "chain_maintenance_duration_s", 0.0)
    else:
        report.relay_assignments = sum(1 for e in all_events if getattr(e, "event_type", None) == EventType.RELAY_ASSIGNED)
        report.relay_releases = sum(1 for e in all_events if getattr(e, "event_type", None) == EventType.RELAY_RELEASED)
        report.relay_handoffs = sum(1 for e in all_events if getattr(e, "event_type", None) == EventType.RELAY_HANDOFF)
        report.relay_losses = sum(1 for e in all_events if getattr(e, "event_type", None) == EventType.RELAY_LOST)
        report.relay_recovery_successes = sum(1 for e in all_events if getattr(e, "event_type", None) == EventType.RELAY_RECOVERY)

    if report.relay_handoffs > 0 or report.relay_assignments > 0 or report.relay_losses > 0:
        report.relay_reallocations = report.relay_handoffs + report.relay_assignments
        report.resilience_status = "ACTIVE_M1"

    # 8. Connectivity-Aware Mission Planning Metrics (Phase 4 & Phase 5)
    if connectivity_planner is not None:
        report.connectivity_feasibility_checks = connectivity_planner.connectivity_feasibility_checks
        report.connectivity_feasible_assignments = connectivity_planner.connectivity_feasible_assignments
        report.connectivity_rejected_assignments = connectivity_planner.connectivity_rejected_assignments
        report.connectivity_deferred_tasks = connectivity_planner.connectivity_deferred_tasks
        report.relay_required_for_assignment = connectivity_planner.relay_required_for_assignment
        report.connectivity_preserved_during_task = connectivity_planner.connectivity_preserved_during_task
        report.communication_induced_replans = connectivity_planner.communication_induced_replans
        report.tasks_deferred_insufficient_relays = getattr(connectivity_planner, "tasks_deferred_insufficient_relays", 0)
    else:
        report.communication_induced_replans = sum(1 for e in all_events if getattr(e, "event_type", None) == EventType.COMMUNICATION_REPLAN)
        report.connectivity_deferred_tasks = sum(1 for e in all_events if getattr(e, "event_type", None) == EventType.TASK_CONNECTIVITY_DEFERRED)

    # Multi-hop metrics
    delivered_hops: list[int] = []
    if telemetry_manager is not None:
        for rep in getattr(telemetry_manager, "authoritative_reports", {}).values():
            if getattr(rep, "hop_count", None) is not None and getattr(rep, "status", None) == TelemetryStatus.DELIVERED:
                delivered_hops.append(rep.hop_count)
    if not delivered_hops:
        for e in all_events:
            if getattr(e, "event_type", None) == EventType.TELEMETRY_DELIVERED:
                hc = getattr(e, "payload", {}).get("hop_count")
                if hc is not None:
                    delivered_hops.append(hc)

    if delivered_hops:
        report.max_hop_count = max(delivered_hops)
        report.mean_hop_count = round(sum(delivered_hops) / len(delivered_hops), 2)
    elif connectivity_planner is not None and getattr(connectivity_planner, "max_hop_count", 0) > 0:
        report.max_hop_count = connectivity_planner.max_hop_count
        report.mean_hop_count = float(connectivity_planner.max_hop_count)

    report.reporting_deadline_success = report.reports_delivered
    report.reporting_deadline_failure = report.reports_deadline_exceeded

    return report
