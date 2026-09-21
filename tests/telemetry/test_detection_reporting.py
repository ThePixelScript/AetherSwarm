"""Focused unit and integration tests for Detection -> GCS Reporting V1 pipeline.

Verifies:
A. FOV entry detection (within 40m radius while airborne)
B. Outside FOV (dist > 40m, no detection)
C. Detection independent of task assignment/service
D. Single-hop direct delivery to GCS
E. Multi-hop relay delivery to GCS (hop_count and route preserved)
F. Delivery within deadline (latency <= 10.0s -> DELIVERED)
G. No-route delay-tolerant buffering (packet remains PENDING)
H. Reconnection before deadline (<= 10s -> DELIVERED)
I. Reconnection after deadline / buffer timeout (> 10s -> DEADLINE_EXCEEDED)
J. Duplicate sighting suppression (authoritative report preserved)
K. Deterministic tie-breaking for simultaneous detections across UAVs
L. Repeated run determinism across full scenario
M. E1 legacy invariance (detection disabled by default)
"""
from types import MappingProxyType
import pytest

from ares_swarm.communication.models import NetworkState
from ares_swarm.core.enums import EventType, Role, RTHState, TaskStatus, TelemetryStatus
from ares_swarm.core.events import DomainEvent
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState
from ares_swarm.interfaces.communication import NetworkAnalysis
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import (
    ChallengeProfileConfig,
    DetectionPipelineConfig,
    ScenarioConfig,
    load_scenario,
)
from ares_swarm.telemetry.manager import DetectionManager


def _make_dummy_net_analysis(routes: dict[str, tuple[str, ...] | None]) -> NetworkAnalysis:
    """Helper to construct a mock NetworkAnalysis with specific GCS routes."""
    return NetworkAnalysis(
        snapshot_revision=0,
        network=NetworkState(links=()),
        connected_uav_ids=tuple(sorted(k for k, v in routes.items() if v is not None)),
        routes_to_gcs=routes,
    )


def test_a_fov_entry_detection():
    """Test A: UAV within 40m sensor FOV while airborne detects the POI."""
    mgr = DetectionManager(
        config=DetectionPipelineConfig(enabled=True, sensor_fov_radius_m=40.0, reporting_deadline_s=10.0),
        gcs_position=(0.0, 0.0),
    )
    snap = StateSnapshot(
        simulation_tick=5,
        simulation_time=5.0,
        state_version=1,
        uavs=MappingProxyType({
            "u1": UAVState(
                id="u1",
                position_xy=(50.0, 50.0),
                velocity_xy=(5.0, 0.0),
                battery_capacity=100.0,
                battery_energy=90.0,
                active=True,
            )
        }),
        tasks=MappingProxyType({
            "t1": TaskState(
                id="t1",
                position_xy=(70.0, 50.0),  # Distance = 20.0m <= 40m
                priority=1,
                service_duration=10.0,
                created_time=0.0,
                status=TaskStatus.PENDING,
            )
        }),
        gcs_position=(0.0, 0.0),
    )
    events = mgr.step_perception(snap)
    assert len(events) == 1
    assert events[0].event_type == EventType.POI_DETECTED
    assert events[0].payload["task_id"] == "t1"
    assert events[0].payload["uav_id"] == "u1"
    assert "t1" in mgr.authoritative_reports
    rep = mgr.authoritative_reports["t1"]
    assert rep.t_detect == 5.0
    assert rep.detecting_uav_id == "u1"
    assert rep.status == TelemetryStatus.PENDING


def test_b_outside_fov_no_detection():
    """Test B: UAV outside 40m sensor FOV does not detect the POI."""
    mgr = DetectionManager(
        config=DetectionPipelineConfig(enabled=True, sensor_fov_radius_m=40.0),
        gcs_position=(0.0, 0.0),
    )
    snap = StateSnapshot(
        simulation_tick=1,
        simulation_time=1.0,
        state_version=1,
        uavs=MappingProxyType({
            "u1": UAVState(
                id="u1",
                position_xy=(0.0, 0.0),
                velocity_xy=(5.0, 0.0),
                battery_capacity=100.0,
                battery_energy=90.0,
                active=True,
            )
        }),
        tasks=MappingProxyType({
            "t1": TaskState(
                id="t1",
                position_xy=(100.0, 100.0),  # Distance ~ 141.4m > 40m
                priority=1,
                service_duration=10.0,
                created_time=0.0,
                status=TaskStatus.PENDING,
            )
        }),
        gcs_position=(0.0, 0.0),
    )
    events = mgr.step_perception(snap)
    assert len(events) == 0
    assert "t1" not in mgr.authoritative_reports


def test_c_independent_of_task_assignment_or_service():
    """Test C: Detection triggers even if task is unassigned or assigned to someone else."""
    mgr = DetectionManager(
        config=DetectionPipelineConfig(enabled=True, sensor_fov_radius_m=40.0),
        gcs_position=(0.0, 0.0),
    )
    snap = StateSnapshot(
        simulation_tick=2,
        simulation_time=2.0,
        state_version=1,
        uavs=MappingProxyType({
            "u1": UAVState(
                id="u1",
                position_xy=(10.0, 10.0),
                velocity_xy=(2.0, 0.0),
                battery_capacity=100.0,
                battery_energy=90.0,
                active=True,
                assigned_task_id="t99",  # Assigned to another task
            )
        }),
        tasks=MappingProxyType({
            "t2": TaskState(
                id="t2",
                position_xy=(15.0, 10.0),  # 5m away, completely unassigned
                priority=1,
                service_duration=10.0,
                created_time=0.0,
                status=TaskStatus.PENDING,
                assigned_uav_id=None,
            )
        }),
        gcs_position=(0.0, 0.0),
    )
    events = mgr.step_perception(snap)
    assert len(events) == 1
    assert events[0].payload["task_id"] == "t2"
    assert mgr.authoritative_reports["t2"].detecting_uav_id == "u1"


def test_d_single_hop_delivery_to_gcs():
    """Test D: Single-hop direct link to GCS delivers telemetry immediately."""
    mgr = DetectionManager(
        config=DetectionPipelineConfig(enabled=True, sensor_fov_radius_m=40.0, reporting_deadline_s=10.0),
        gcs_position=(0.0, 0.0),
    )
    snap = StateSnapshot(
        simulation_tick=3,
        simulation_time=3.0,
        state_version=1,
        uavs=MappingProxyType({
            "u1": UAVState(
                id="u1",
                position_xy=(20.0, 20.0),
                velocity_xy=(2.0, 0.0),
                battery_capacity=100.0,
                battery_energy=90.0,
                active=True,
            )
        }),
        tasks=MappingProxyType({
            "t1": TaskState(
                id="t1",
                position_xy=(25.0, 20.0),
                priority=1,
                service_duration=10.0,
                created_time=0.0,
            )
        }),
        gcs_position=(0.0, 0.0),
    )
    mgr.step_perception(snap)
    net_analysis = _make_dummy_net_analysis({"u1": ("u1", "GCS")})
    events = mgr.step_telemetry(snap, net_analysis)

    assert len(events) == 1
    assert events[0].event_type == EventType.TELEMETRY_DELIVERED
    assert events[0].payload["hop_count"] == 1
    assert events[0].payload["route"] == ["u1", "GCS"]
    assert events[0].payload["latency_s"] == 0.0

    rep = mgr.authoritative_reports["t1"]
    assert rep.status == TelemetryStatus.DELIVERED
    assert rep.hop_count == 1
    assert rep.t_gcs_received == 3.0
    assert rep.reporting_latency_s == 0.0


def test_e_multi_hop_relay_delivery_to_gcs():
    """Test E: Multi-hop route through relay UAV is correctly routed and recorded."""
    mgr = DetectionManager(
        config=DetectionPipelineConfig(enabled=True, sensor_fov_radius_m=40.0, reporting_deadline_s=10.0),
        gcs_position=(0.0, 0.0),
    )
    snap = StateSnapshot(
        simulation_tick=4,
        simulation_time=4.0,
        state_version=1,
        uavs=MappingProxyType({
            "u1": UAVState(
                id="u1",
                position_xy=(100.0, 0.0),
                velocity_xy=(1.0, 0.0),
                battery_capacity=100.0,
                battery_energy=90.0,
                active=True,
            )
        }),
        tasks=MappingProxyType({
            "t1": TaskState(
                id="t1",
                position_xy=(110.0, 0.0),
                priority=1,
                service_duration=10.0,
                created_time=0.0,
            )
        }),
        gcs_position=(0.0, 0.0),
    )
    mgr.step_perception(snap)
    # Route: u1 -> u2 -> GCS (2 hops)
    net_analysis = _make_dummy_net_analysis({"u1": ("u1", "u2", "GCS")})
    events = mgr.step_telemetry(snap, net_analysis)

    assert len(events) == 1
    assert events[0].event_type == EventType.TELEMETRY_DELIVERED
    assert events[0].payload["hop_count"] == 2
    assert events[0].payload["route"] == ["u1", "u2", "GCS"]
    rep = mgr.authoritative_reports["t1"]
    assert rep.hop_count == 2
    assert rep.status == TelemetryStatus.DELIVERED


def test_f_delivery_within_deadline():
    """Test F: Delivery occurring within <= 10.0s deadline is marked DELIVERED."""
    mgr = DetectionManager(
        config=DetectionPipelineConfig(enabled=True, sensor_fov_radius_m=40.0, reporting_deadline_s=10.0),
        gcs_position=(0.0, 0.0),
    )
    snap_detect = StateSnapshot(
        simulation_tick=10,
        simulation_time=10.0,
        state_version=1,
        uavs=MappingProxyType({
            "u1": UAVState(id="u1", position_xy=(50.0, 50.0), velocity_xy=(2.0, 0.0), battery_capacity=100.0, battery_energy=90.0, active=True)
        }),
        tasks=MappingProxyType({
            "t1": TaskState(id="t1", position_xy=(50.0, 50.0), priority=1, service_duration=5.0, created_time=0.0)
        }),
        gcs_position=(0.0, 0.0),
    )
    mgr.step_perception(snap_detect)
    assert mgr.authoritative_reports["t1"].t_detect == 10.0

    # Deliver at t = 18.0s (latency = 8.0s <= 10.0s)
    snap_deliver = StateSnapshot(
        simulation_tick=18,
        simulation_time=18.0,
        state_version=2,
        uavs=snap_detect.uavs,
        tasks=snap_detect.tasks,
        gcs_position=(0.0, 0.0),
    )
    net_analysis = _make_dummy_net_analysis({"u1": ("u1", "GCS")})
    events = mgr.step_telemetry(snap_deliver, net_analysis)

    assert len(events) == 1
    assert events[0].event_type == EventType.TELEMETRY_DELIVERED
    assert events[0].payload["latency_s"] == 8.0
    assert mgr.authoritative_reports["t1"].status == TelemetryStatus.DELIVERED


def test_g_no_route_buffering():
    """Test G: When no route to GCS exists, packet is buffered as PENDING."""
    mgr = DetectionManager(
        config=DetectionPipelineConfig(enabled=True, sensor_fov_radius_m=40.0, reporting_deadline_s=10.0),
        gcs_position=(0.0, 0.0),
    )
    snap = StateSnapshot(
        simulation_tick=5,
        simulation_time=5.0,
        state_version=1,
        uavs=MappingProxyType({
            "u1": UAVState(id="u1", position_xy=(200.0, 200.0), velocity_xy=(3.0, 0.0), battery_capacity=100.0, battery_energy=90.0, active=True)
        }),
        tasks=MappingProxyType({
            "t1": TaskState(id="t1", position_xy=(200.0, 200.0), priority=1, service_duration=5.0, created_time=0.0)
        }),
        gcs_position=(0.0, 0.0),
    )
    mgr.step_perception(snap)
    net_analysis = _make_dummy_net_analysis({})  # Empty routes
    events = mgr.step_telemetry(snap, net_analysis)

    assert len(events) == 0  # Still waiting in buffer
    assert "t1" in mgr.pending_reports
    assert mgr.authoritative_reports["t1"].status == TelemetryStatus.PENDING


def test_h_reconnect_before_deadline():
    """Test H: UAV disconnected at t_detect reconnects at t_detect + 6s -> DELIVERED."""
    mgr = DetectionManager(
        config=DetectionPipelineConfig(enabled=True, sensor_fov_radius_m=40.0, reporting_deadline_s=10.0),
        gcs_position=(0.0, 0.0),
    )
    snap_detect = StateSnapshot(
        simulation_tick=10,
        simulation_time=10.0,
        state_version=1,
        uavs=MappingProxyType({
            "u1": UAVState(id="u1", position_xy=(300.0, 300.0), velocity_xy=(3.0, 0.0), battery_capacity=100.0, battery_energy=90.0, active=True)
        }),
        tasks=MappingProxyType({
            "t1": TaskState(id="t1", position_xy=(300.0, 300.0), priority=1, service_duration=5.0, created_time=0.0)
        }),
        gcs_position=(0.0, 0.0),
    )
    mgr.step_perception(snap_detect)
    # Ticks 10 to 15: disconnected
    for t in range(10, 16):
        snap_t = StateSnapshot(
            simulation_tick=t,
            simulation_time=float(t),
            state_version=t,
            uavs=snap_detect.uavs,
            tasks=snap_detect.tasks,
            gcs_position=(0.0, 0.0),
        )
        mgr.step_telemetry(snap_t, _make_dummy_net_analysis({}))
    assert mgr.authoritative_reports["t1"].status == TelemetryStatus.PENDING

    # Tick 16: reconnects
    snap_16 = StateSnapshot(
        simulation_tick=16,
        simulation_time=16.0,
        state_version=16,
        uavs=snap_detect.uavs,
        tasks=snap_detect.tasks,
        gcs_position=(0.0, 0.0),
    )
    events = mgr.step_telemetry(snap_16, _make_dummy_net_analysis({"u1": ("u1", "GCS")}))
    assert len(events) == 1
    assert events[0].event_type == EventType.TELEMETRY_DELIVERED
    assert events[0].payload["latency_s"] == 6.0
    assert mgr.authoritative_reports["t1"].status == TelemetryStatus.DELIVERED
    assert "t1" not in mgr.pending_reports


def test_i_buffer_timeout_deadline_exceeded():
    """Test I: UAV remains disconnected past 10.0s -> DEADLINE_EXCEEDED (not deleted)."""
    mgr = DetectionManager(
        config=DetectionPipelineConfig(enabled=True, sensor_fov_radius_m=40.0, reporting_deadline_s=10.0),
        gcs_position=(0.0, 0.0),
    )
    snap_detect = StateSnapshot(
        simulation_tick=10,
        simulation_time=10.0,
        state_version=1,
        uavs=MappingProxyType({
            "u1": UAVState(id="u1", position_xy=(500.0, 500.0), velocity_xy=(3.0, 0.0), battery_capacity=100.0, battery_energy=90.0, active=True)
        }),
        tasks=MappingProxyType({
            "t1": TaskState(id="t1", position_xy=(500.0, 500.0), priority=1, service_duration=5.0, created_time=0.0)
        }),
        gcs_position=(0.0, 0.0),
    )
    mgr.step_perception(snap_detect)

    # Step up to 21.0s (11.0s elapsed > 10.0s deadline) without route
    snap_21 = StateSnapshot(
        simulation_tick=21,
        simulation_time=21.0,
        state_version=21,
        uavs=snap_detect.uavs,
        tasks=snap_detect.tasks,
        gcs_position=(0.0, 0.0),
    )
    events = mgr.step_telemetry(snap_21, _make_dummy_net_analysis({}))
    assert len(events) == 1
    assert events[0].event_type == EventType.TELEMETRY_DEADLINE_EXCEEDED
    assert events[0].payload["reason"] == "BUFFER_TIMEOUT_NO_ROUTE"
    assert mgr.authoritative_reports["t1"].status == TelemetryStatus.DEADLINE_EXCEEDED
    assert "t1" not in mgr.pending_reports


def test_j_duplicate_sighting_suppression():
    """Test J: Duplicate sightings by the same or different UAVs are suppressed."""
    mgr = DetectionManager(
        config=DetectionPipelineConfig(enabled=True, sensor_fov_radius_m=40.0),
        gcs_position=(0.0, 0.0),
    )
    snap_1 = StateSnapshot(
        simulation_tick=5,
        simulation_time=5.0,
        state_version=1,
        uavs=MappingProxyType({
            "u1": UAVState(id="u1", position_xy=(10.0, 10.0), velocity_xy=(1.0, 0.0), battery_capacity=100.0, battery_energy=90.0, active=True)
        }),
        tasks=MappingProxyType({
            "t1": TaskState(id="t1", position_xy=(10.0, 10.0), priority=1, service_duration=5.0, created_time=0.0)
        }),
        gcs_position=(0.0, 0.0),
    )
    events_1 = mgr.step_perception(snap_1)
    assert len(events_1) == 1
    assert mgr.authoritative_reports["t1"].t_detect == 5.0
    assert mgr.authoritative_reports["t1"].detecting_uav_id == "u1"

    # Sighting by u2 at tick 12
    snap_2 = StateSnapshot(
        simulation_tick=12,
        simulation_time=12.0,
        state_version=2,
        uavs=MappingProxyType({
            "u2": UAVState(id="u2", position_xy=(12.0, 10.0), velocity_xy=(1.0, 0.0), battery_capacity=100.0, battery_energy=90.0, active=True)
        }),
        tasks=snap_1.tasks,
        gcs_position=(0.0, 0.0),
    )
    events_2 = mgr.step_perception(snap_2)
    assert len(events_2) == 0  # Suppressed!
    # Authoritative report untouched
    assert mgr.authoritative_reports["t1"].t_detect == 5.0
    assert mgr.authoritative_reports["t1"].detecting_uav_id == "u1"
    assert len(mgr.sightings_log) == 1
    assert mgr.sightings_log[0]["duplicate"] is True


def test_k_deterministic_tie_breaking_simultaneous():
    """Test K: When u1 and u2 detect the same POI in the same tick, lexicographical order breaks tie."""
    mgr = DetectionManager(
        config=DetectionPipelineConfig(enabled=True, sensor_fov_radius_m=40.0),
        gcs_position=(0.0, 0.0),
    )
    snap = StateSnapshot(
        simulation_tick=1,
        simulation_time=1.0,
        state_version=1,
        uavs=MappingProxyType({
            "u2": UAVState(id="u2", position_xy=(15.0, 0.0), velocity_xy=(1.0, 0.0), battery_capacity=100.0, battery_energy=90.0, active=True),
            "u1": UAVState(id="u1", position_xy=(10.0, 0.0), velocity_xy=(1.0, 0.0), battery_capacity=100.0, battery_energy=90.0, active=True),
        }),
        tasks=MappingProxyType({
            "t1": TaskState(id="t1", position_xy=(12.0, 0.0), priority=1, service_duration=5.0, created_time=0.0)
        }),
        gcs_position=(0.0, 0.0),
    )
    events = mgr.step_perception(snap)
    assert len(events) == 1
    # u1 comes before u2 alphabetically
    assert events[0].payload["uav_id"] == "u1"
    assert mgr.authoritative_reports["t1"].detecting_uav_id == "u1"


def test_l_repeated_run_determinism():
    """Test L: Running the same scenario twice yields bitwise identical detection results."""
    scenario_dict = {
        "name": "test_det_determinism",
        "seed": 42,
        "dt": 1.0,
        "speed_limit": 5.0,
        "duration": 50.0,
        "max_ticks": 50,
        "gcs_position": [0.0, 0.0],
        "uavs": [
            {"id": "u1", "position": [0.0, 0.0], "battery_capacity": 100.0},
            {"id": "u2", "position": [0.0, 0.0], "battery_capacity": 100.0},
        ],
        "tasks": [
            {"id": "t1", "position": [50.0, 50.0], "priority": 1, "service_duration": 5.0},
            {"id": "t2", "position": [100.0, 50.0], "priority": 2, "service_duration": 5.0},
        ],
        "challenge_profile": {
            "enabled": True,
            "detection_pipeline": {
                "enabled": True,
                "sensor_fov_radius_m": 40.0,
                "reporting_deadline_s": 10.0,
            },
        },
    }
    scen = load_scenario(scenario_dict)

    runner1 = MissionRunner(scen, seed=42)
    res1 = runner1.run()

    runner2 = MissionRunner(scen, seed=42)
    res2 = runner2.run()

    assert res1.to_dict()["telemetry"] == res2.to_dict()["telemetry"]
    assert res1.to_dict()["telemetry_reports"] == res2.to_dict()["telemetry_reports"]


def test_m_e1_invariance():
    """Test M: E1 benchmark produces identical results when detection is disabled (default)."""
    e1_path = "scenarios/poc_round1.yaml"
    runner = MissionRunner(e1_path, seed=42)
    res = runner.run()

    assert res.metrics_report is not None
    assert res.metrics_report.tasks_completed == 10
    assert res.metrics_report.tasks_total == 10
    assert res.metrics_report.geofence_violation_count == 0
    assert res.metrics_report.separation_violation_count == 0
    assert res.metrics_report.battery_exhaustion_count == 0
    # Telemetry should be default/disabled
    assert runner.detection_manager is None


def test_n_zero_detections_metrics_edge_case():
    """Test N: When there are zero detections, metrics gracefully return valid neutral values."""
    mgr = DetectionManager(
        config=DetectionPipelineConfig(enabled=True, sensor_fov_radius_m=40.0),
        gcs_position=(0.0, 0.0),
    )
    metrics = mgr.get_metrics()
    assert metrics["total_detections"] == 0
    assert metrics["reports_delivered"] == 0
    assert metrics["reports_deadline_exceeded"] == 0
    assert metrics["reporting_compliance_ratio"] == 1.0
    assert metrics["mean_reporting_latency_s"] is None
    assert metrics["max_reporting_latency_s"] is None
    assert metrics["per_uav_detection_counts"] == {}
    assert metrics["per_uav_delivered_counts"] == {}


def test_o_terminal_state_protection():
    """Test O: Terminal states DELIVERED and DEADLINE_EXCEEDED are immutable and never overwritten."""
    mgr = DetectionManager(
        config=DetectionPipelineConfig(enabled=True, sensor_fov_radius_m=40.0, reporting_deadline_s=10.0),
        gcs_position=(0.0, 0.0),
    )
    snap = StateSnapshot(
        simulation_tick=1,
        simulation_time=1.0,
        state_version=1,
        uavs=MappingProxyType({
            "u1": UAVState(id="u1", position_xy=(10.0, 10.0), velocity_xy=(1.0, 0.0), battery_capacity=100.0, battery_energy=90.0, active=True)
        }),
        tasks=MappingProxyType({
            "t1": TaskState(id="t1", position_xy=(10.0, 10.0), priority=1, service_duration=5.0, created_time=0.0)
        }),
        gcs_position=(0.0, 0.0),
    )
    mgr.step_perception(snap)
    net_direct = _make_dummy_net_analysis({"u1": ("u1", "GCS")})
    mgr.step_telemetry(snap, net_direct)
    assert mgr.authoritative_reports["t1"].status == TelemetryStatus.DELIVERED
    assert "t1" not in mgr.pending_reports

    # Force t1 back into pending_reports to test defensive barrier
    mgr.pending_reports.append("t1")
    snap_later = StateSnapshot(
        simulation_tick=50,
        simulation_time=50.0,
        state_version=50,
        uavs=snap.uavs,
        tasks=snap.tasks,
        gcs_position=(0.0, 0.0),
    )
    # Even if step_telemetry is run later with empty routes, status remains DELIVERED
    mgr.step_telemetry(snap_later, _make_dummy_net_analysis({}))
    assert mgr.authoritative_reports["t1"].status == TelemetryStatus.DELIVERED
    assert "t1" not in mgr.pending_reports
