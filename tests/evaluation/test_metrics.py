"""Unit tests for official evaluation metrics pipeline."""
import pytest
from types import MappingProxyType
from unittest.mock import MagicMock

from ares_swarm.core.enums import TaskStatus
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState
from ares_swarm.evaluation.metrics import MissionMetricsReport, compute_mission_metrics
from ares_swarm.safety.safety_assessor import SafetyReport


def test_metrics_report_to_dict():
    report = MissionMetricsReport(
        tasks_total=10,
        tasks_assigned=8,
        tasks_completed=8,
        mission_completion_rate=0.8,
        completion_time_s=150.0,
        priority_weighted_score=0.85,
        total_energy_consumed_wh=250.0,
        average_pdr=0.98,
        average_latency_ms=12.5,
        connectivity_availability=0.95,
        downtime_s=15.0,
        relay_reallocations=3,
        recovery_time_s=5.0,
        network_reconfiguration_efficiency=0.9,
        collision_count=0,
        min_inter_uav_separation_m=25.4,
        battery_exhaustion_count=0,
        geofence_violation_count=0,
    )
    d = report.to_dict()
    assert "mission" in d
    assert "communication" in d
    assert "resilience" in d
    assert "safety" in d

    assert d["mission"]["tasks_total"] == 10
    assert d["mission"]["tasks_completed"] == 8
    assert d["mission"]["mission_completion_rate"] == 0.8
    assert d["communication"]["average_pdr"] == 0.98
    assert d["resilience"]["relay_reallocations"] == 3
    assert d["safety"]["collision_count"] == 0
    assert d["safety"]["min_inter_uav_separation_m"] == 25.4


def test_compute_mission_metrics_flow():
    # Initial snapshot
    u1_init = UAVState(id="u1", position_xy=(0.0, 0.0), battery_capacity=100.0, battery_energy=100.0)
    u2_init = UAVState(id="u2", position_xy=(30.0, 0.0), battery_capacity=100.0, battery_energy=100.0)
    t1_init = TaskState(id="t1", position_xy=(10.0, 0.0), priority=2, status=TaskStatus.PENDING)
    t2_init = TaskState(id="t2", position_xy=(20.0, 0.0), priority=1, status=TaskStatus.PENDING)

    init_snap = StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType({"u1": u1_init, "u2": u2_init}),
        tasks=MappingProxyType({"t1": t1_init, "t2": t2_init}),
        gcs_position=(0.0, 0.0),
    )

    # Final snapshot
    u1_final = UAVState(id="u1", position_xy=(10.0, 0.0), battery_capacity=100.0, battery_energy=90.0)
    u2_final = UAVState(id="u2", position_xy=(20.0, 0.0), battery_capacity=100.0, battery_energy=85.0)
    t1_final = TaskState(id="t1", position_xy=(10.0, 0.0), priority=2, status=TaskStatus.COMPLETE, service_progress=1.0)
    t2_final = TaskState(id="t2", position_xy=(20.0, 0.0), priority=1, status=TaskStatus.COMPLETE, service_progress=1.0)

    final_snap = StateSnapshot(
        simulation_tick=10,
        simulation_time=10.0,
        state_version=5,
        uavs=MappingProxyType({"u1": u1_final, "u2": u2_final}),
        tasks=MappingProxyType({"t1": t1_final, "t2": t2_final}),
        gcs_position=(0.0, 0.0),
    )

    # Mock step history
    mock_link = MagicMock()
    mock_link.estimated_pdr = 0.99
    mock_link.latency_ms = 8.0

    mock_net = MagicMock()
    mock_net.network.links = [mock_link]
    mock_net.connected_uav_ids = {"u1", "u2"}
    mock_net.routes_to_gcs = {"u1": ("u1", "GCS"), "u2": ("u2", "GCS")}

    mock_step = MagicMock()
    mock_step.simulation_time = 5.0
    mock_step.network_analysis = mock_net
    mock_step.snapshot = final_snap

    safety_report = SafetyReport(
        min_observed_separation_m=28.5,
        separation_violations_count=0,
        battery_exhaustions_count=0,
        geofence_violations_count=0,
    )

    metrics = compute_mission_metrics(
        step_history=[mock_step],
        initial_snapshot=init_snap,
        final_snapshot=final_snap,
        dt=1.0,
        safety_report=safety_report,
    )

    assert metrics.tasks_total == 2
    assert metrics.tasks_spawned == 2
    assert metrics.tasks_completed == 2
    assert metrics.tasks_assigned == 0
    assert metrics.tasks_expired == 0
    assert metrics.mission_completion_rate == 1.0
    assert metrics.priority_weighted_score == 1.0
    assert metrics.total_energy_consumed_wh == pytest.approx(25.0)  # (100+100) - (90+85)
    assert metrics.average_pdr == pytest.approx(0.99)
    assert metrics.average_latency_ms == pytest.approx(8.0)
    assert metrics.connectivity_availability == 1.0
    assert metrics.collision_count == 0
    assert metrics.min_inter_uav_separation_m == 28.5
