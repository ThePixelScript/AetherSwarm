"""Deterministic test suite for Phase 5C corrections:
Surveyor Pre-Detection Holding & Corridor-Aware Multi-Hop Geometry.

Tests 1 to 10:
1. test_surveyor_holds_before_fov (chain incomplete -> surveyor held outside 40m -> no POI_DETECTED)
2. test_surveyor_releases_after_chain_ready (chain ready -> route live -> surveyor released -> FOV entry -> POI_DETECTED)
3. test_telemetry_after_readiness (telemetry delivered within 10s of detection)
4. test_direct_valid_path (central POI -> direct path retained)
5. test_off_axis_corridor_path (southern POI -> path uses portal -> all stations geofence-valid)
6. test_north_off_axis (northern POI -> path uses portal -> all stations geofence-valid)
7. test_corner_case (extreme corner POI -> all stations legal, all hops <= 95m)
8. test_path_hop_recalculation (hop count uses path length, not Euclidean direct distance)
9. test_separation (station-to-station separation >= 20m)
10. test_full_integration (multi-hop chain + surveyor + detection + telemetry -> 0 deadline failures, 0 geofence violations)
"""
from __future__ import annotations

import math
import pytest

from ares_swarm.autonomy.connectivity_planner import (
    ConnectivityAwarePlanner,
    ConnectivityAwarePlannerConfig,
    compute_corridor_path,
    compute_multihop_stations,
    get_path_point_at_distance,
)
from ares_swarm.autonomy.relay_manager import (
    ChainStatus,
    DynamicRelayManager,
    RelayChain,
)
from ares_swarm.core.enums import EventType, Role, TaskStatus, TelemetryStatus
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState
from ares_swarm.safety.airspace import ChallengeAirspace
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import (
    ChallengeAirspaceConfig,
    ChallengeProfileConfig,
    CommunicationConfig,
    DetectionPipelineConfig,
    ScenarioConfig,
)


def _make_airspace() -> ChallengeAirspace:
    return ChallengeAirspace(
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        corridor_bounds_x=(-75.0, 0.0),
        corridor_bounds_y=(400.0, 600.0),
        staging_pad_center=(-75.0, 500.0),
        staging_pad_radius_m=15.0,
        max_height=100.0,
    )


# --- Test 1: Surveyor Holds Before FOV ---
def test_surveyor_holds_before_fov():
    """Test 1: Surveyor is held outside 40m FOV while relay chain is forming."""
    gcs = (-75.0, 500.0)
    poi_pos = (250.0, 500.0)  # Requires relay at ~87.5m
    
    airspace_cfg = ChallengeAirspaceConfig(
        enabled=True,
        staging_pad_center=gcs,
        corridor_bounds_x=(-75.0, 0.0),
        corridor_bounds_y=(400.0, 600.0),
    )
    detect_cfg = DetectionPipelineConfig(enabled=True, sensor_fov_radius_m=40.0, reporting_deadline_s=10.0)
    prof_cfg = ChallengeProfileConfig(
        enabled=True,
        airspace=airspace_cfg,
        detection_pipeline=detect_cfg,
        enable_relay_manager=True,
        enable_connectivity_aware_planning=True,
        enforce_geofence=True,
        enforce_separation=True,
    )
    
    scenario = ScenarioConfig(
        name="test_hold_fov",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=100.0,
        max_ticks=100,
        gcs_position=gcs,
        communication=CommunicationConfig(max_range=100.0),
        uavs=(
            {"id": "uav_1", "position": [-75.0, 500.0], "role": "IDLE", "battery_capacity": 4200.0, "battery_energy": 4200.0},
            {"id": "uav_2", "position": [-75.0, 540.0], "role": "IDLE", "battery_capacity": 4200.0, "battery_energy": 4200.0},
            {"id": "uav_3", "position": [-75.0, 460.0], "role": "IDLE", "battery_capacity": 4200.0, "battery_energy": 4200.0},
        ),
        tasks=(
            {"id": "poi_1", "position": [250.0, 500.0], "priority": 1, "spawn_time": 0.0, "service_duration": 2.0},
        ),
        challenge_profile=prof_cfg,
    )
    
    runner = MissionRunner(scenario=scenario, seed=42)
    # Step simulation 5 ticks (relays are moving, chain forming)
    for _ in range(5):
        runner.step()
        
    snap = runner.state_store.snapshot()
    uav_1 = snap.uavs["uav_1"]
    
    # Distance from surveyor uav_1 to POI
    dist_to_poi = math.hypot(uav_1.position_xy[0] - poi_pos[0], uav_1.position_xy[1] - poi_pos[1])
    
    # Assert surveyor remains outside 40m FOV while chain is forming
    assert dist_to_poi >= 40.0, f"Surveyor entered FOV prematurely: dist = {dist_to_poi:.2f}m < 40m"
    
    # Assert no POI_DETECTED events were emitted
    poi_detected_events = [e for e in runner.all_events if e.event_type == EventType.POI_DETECTED]
    assert len(poi_detected_events) == 0, f"Expected 0 POI_DETECTED events during hold, got {len(poi_detected_events)}"


# --- Test 2: Surveyor Releases After Chain Ready ---
def test_surveyor_releases_after_chain_ready():
    """Test 2: Surveyor is released and enters FOV once relay chain becomes operational."""
    gcs = (-75.0, 500.0)
    poi_pos = (150.0, 500.0)  # D = 225m -> H = 3, K = 2 relays (requires 3 UAVs total)
    
    airspace_cfg = ChallengeAirspaceConfig(
        enabled=True,
        staging_pad_center=gcs,
        corridor_bounds_x=(-75.0, 0.0),
        corridor_bounds_y=(400.0, 600.0),
    )
    detect_cfg = DetectionPipelineConfig(enabled=True, sensor_fov_radius_m=40.0, reporting_deadline_s=10.0)
    prof_cfg = ChallengeProfileConfig(
        enabled=True,
        airspace=airspace_cfg,
        detection_pipeline=detect_cfg,
        enable_relay_manager=True,
        enable_connectivity_aware_planning=True,
        enforce_geofence=True,
        enforce_separation=True,
    )
    
    scenario = ScenarioConfig(
        name="test_release_chain",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=100.0,
        max_ticks=100,
        gcs_position=gcs,
        communication=CommunicationConfig(max_range=100.0),
        uavs=(
            {"id": "uav_1", "position": [-75.0, 500.0], "role": "IDLE", "battery_capacity": 4200.0, "battery_energy": 4200.0},
            {"id": "uav_2", "position": [-75.0, 540.0], "role": "IDLE", "battery_capacity": 4200.0, "battery_energy": 4200.0},
            {"id": "uav_3", "position": [-75.0, 460.0], "role": "IDLE", "battery_capacity": 4200.0, "battery_energy": 4200.0},
        ),
        tasks=(
            {"id": "poi_1", "position": [150.0, 500.0], "priority": 1, "spawn_time": 0.0, "service_duration": 2.0},
        ),
        challenge_profile=prof_cfg,
    )
    
    runner = MissionRunner(scenario=scenario, seed=42)
    # Step simulation 80 ticks to allow relays to arrive, chain to activate, surveyor to enter FOV
    for _ in range(80):
        runner.step()
        
    poi_detected_events = [e for e in runner.all_events if e.event_type == EventType.POI_DETECTED]
    assert len(poi_detected_events) == 1, "Expected POI_DETECTED to occur after chain ready"


# --- Test 3: Telemetry After Readiness ---
def test_telemetry_after_readiness():
    """Test 3: Telemetry detection happens only after route is live, ensuring delivery within 10s."""
    gcs = (-75.0, 500.0)
    poi_pos = (150.0, 500.0)
    
    airspace_cfg = ChallengeAirspaceConfig(
        enabled=True,
        staging_pad_center=gcs,
        corridor_bounds_x=(-75.0, 0.0),
        corridor_bounds_y=(400.0, 600.0),
    )
    detect_cfg = DetectionPipelineConfig(enabled=True, sensor_fov_radius_m=40.0, reporting_deadline_s=10.0)
    prof_cfg = ChallengeProfileConfig(
        enabled=True,
        airspace=airspace_cfg,
        detection_pipeline=detect_cfg,
        enable_relay_manager=True,
        enable_connectivity_aware_planning=True,
    )
    
    scenario = ScenarioConfig(
        name="test_telemetry_delivery",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=120.0,
        max_ticks=120,
        gcs_position=gcs,
        communication=CommunicationConfig(max_range=100.0),
        uavs=(
            {"id": "uav_1", "position": [-75.0, 500.0], "role": "IDLE", "battery_capacity": 4200.0, "battery_energy": 4200.0},
            {"id": "uav_2", "position": [-75.0, 540.0], "role": "IDLE", "battery_capacity": 4200.0, "battery_energy": 4200.0},
            {"id": "uav_3", "position": [-75.0, 460.0], "role": "IDLE", "battery_capacity": 4200.0, "battery_energy": 4200.0},
        ),
        tasks=(
            {"id": "poi_1", "position": [150.0, 500.0], "priority": 1, "spawn_time": 0.0, "service_duration": 2.0},
        ),
        challenge_profile=prof_cfg,
    )
    
    runner = MissionRunner(scenario=scenario, seed=42)
    for _ in range(100):
        runner.step()
        
    telem_mgr = runner.detection_manager
    assert telem_mgr is not None
    assert "poi_1" in telem_mgr.authoritative_reports
    report = telem_mgr.authoritative_reports["poi_1"]
    
    assert report.status == TelemetryStatus.DELIVERED, f"Expected DELIVERED status, got {report.status}"
    assert report.reporting_latency_s is not None
    assert report.reporting_latency_s <= 10.0, f"Reporting latency {report.reporting_latency_s}s exceeded 10s deadline"


# --- Test 4: Direct Valid Path ---
def test_direct_valid_path():
    """Test 4: Central/corridor-aligned POI retains direct GCS->POI path."""
    gcs = (-75.0, 500.0)
    poi_pos = (200.0, 500.0)
    
    pts = compute_corridor_path(gcs, poi_pos, corridor_bounds_y=(400.0, 600.0))
    assert len(pts) == 2, f"Expected direct path [GCS, POI], got {pts}"
    assert pts[0] == gcs
    assert pts[1] == poi_pos


# --- Test 5: Off-Axis Corridor Path (South) ---
def test_off_axis_corridor_path():
    """Test 5: Southern off-axis POI uses portal (0, 500) and all stations are geofence-valid."""
    gcs = (-75.0, 500.0)
    poi_pos = (200.0, 100.0)
    airspace = _make_airspace()
    
    pts = compute_corridor_path(gcs, poi_pos, corridor_bounds_y=(400.0, 600.0))
    assert len(pts) == 3, f"Expected 3-waypoint piecewise path via portal, got {pts}"
    assert pts[1] == (0.0, 500.0)
    
    h_min, k_min, stations = compute_multihop_stations(gcs, poi_pos, effective_range=95.0, corridor_bounds_y=(400.0, 600.0))
    
    assert k_min > 0
    for st in stations:
        assert airspace.is_in_authorized_union(st), f"Station {st} is outside legal airspace"


# --- Test 6: North Off-Axis Path ---
def test_north_off_axis():
    """Test 6: Northern off-axis POI uses portal (0, 500) and all stations are geofence-valid."""
    gcs = (-75.0, 500.0)
    poi_pos = (200.0, 900.0)
    airspace = _make_airspace()
    
    pts = compute_corridor_path(gcs, poi_pos, corridor_bounds_y=(400.0, 600.0))
    assert len(pts) == 3, f"Expected 3-waypoint piecewise path via portal, got {pts}"
    assert pts[1] == (0.0, 500.0)
    
    h_min, k_min, stations = compute_multihop_stations(gcs, poi_pos, effective_range=95.0, corridor_bounds_y=(400.0, 600.0))
    
    assert k_min > 0
    for st in stations:
        assert airspace.is_in_authorized_union(st), f"Station {st} is outside legal airspace"


# --- Test 7: Corner Case Geometry ---
def test_corner_case():
    """Test 7: Extreme corner POI (900, 100) produces legal stations and all hops <= 95m."""
    gcs = (-75.0, 500.0)
    poi_pos = (900.0, 100.0)
    airspace = _make_airspace()
    
    h_min, k_min, stations = compute_multihop_stations(gcs, poi_pos, effective_range=95.0, corridor_bounds_y=(400.0, 600.0))
    
    all_nodes = [gcs] + list(stations) + [poi_pos]
    for i in range(len(all_nodes) - 1):
        hop_d = math.hypot(all_nodes[i+1][0] - all_nodes[i][0], all_nodes[i+1][1] - all_nodes[i][1])
        assert hop_d <= 95.0 + 1e-5, f"Hop distance {hop_d:.2f}m between {all_nodes[i]} and {all_nodes[i+1]} exceeds 95m"
        
    for st in stations:
        assert airspace.is_in_authorized_union(st), f"Station {st} outside legal airspace"


# --- Test 8: Path Hop Recalculation ---
def test_path_hop_recalculation():
    """Test 8: Hop count calculation uses cumulative path distance along piecewise corridor route."""
    gcs = (-75.0, 500.0)
    poi_pos = (200.0, 100.0)
    
    pts = compute_corridor_path(gcs, poi_pos, corridor_bounds_y=(400.0, 600.0))
    l1 = math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1])
    l2 = math.hypot(pts[2][0] - pts[1][0], pts[2][1] - pts[1][1])
    expected_path_dist = l1 + l2
    expected_h = math.ceil(expected_path_dist / 95.0)
    
    h_min, k_min, stations = compute_multihop_stations(gcs, poi_pos, effective_range=95.0, corridor_bounds_y=(400.0, 600.0))
    
    assert h_min == expected_h, f"Expected {expected_h} hops for path dist {expected_path_dist:.2f}m, got {h_min}"


# --- Test 9: Station Separation ---
def test_separation():
    """Test 9: Generated intermediate stations maintain >= 20m separation."""
    gcs = (-75.0, 500.0)
    poi_pos = (800.0, 100.0)
    
    h_min, k_min, stations = compute_multihop_stations(gcs, poi_pos, effective_range=95.0, corridor_bounds_y=(400.0, 600.0))
    
    for i in range(len(stations) - 1):
        sep = math.hypot(stations[i+1][0] - stations[i][0], stations[i+1][1] - stations[i][1])
        assert sep >= 20.0 - 1e-5, f"Station separation {sep:.2f}m between {stations[i]} and {stations[i+1]} < 20m"


# --- Test 10: Full Integration ---
def test_full_integration():
    """Test 10: Full mission integration with multihop chain + surveyor holding + corridor routing.
    
    Verifies: 0 deadline exceedances and 0 geofence violations.
    """
    gcs = (-75.0, 500.0)
    airspace_cfg = ChallengeAirspaceConfig(
        enabled=True,
        staging_pad_center=gcs,
        corridor_bounds_x=(-75.0, 0.0),
        corridor_bounds_y=(400.0, 600.0),
    )
    detect_cfg = DetectionPipelineConfig(enabled=True, sensor_fov_radius_m=40.0, reporting_deadline_s=10.0)
    prof_cfg = ChallengeProfileConfig(
        enabled=True,
        airspace=airspace_cfg,
        detection_pipeline=detect_cfg,
        enable_relay_manager=True,
        enable_connectivity_aware_planning=True,
        enforce_geofence=True,
        enforce_separation=True,
    )
    
    scenario = ScenarioConfig(
        name="test_full_integration",
        seed=2026,
        dt=1.0,
        speed_limit=5.0,
        duration=300.0,
        max_ticks=300,
        gcs_position=gcs,
        communication=CommunicationConfig(max_range=100.0),
        uavs=(
            {"id": "uav_1", "position": [-75.0, 500.0], "role": "IDLE", "battery_capacity": 4200.0, "battery_energy": 4200.0},
            {"id": "uav_2", "position": [-75.0, 540.0], "role": "IDLE", "battery_capacity": 4200.0, "battery_energy": 4200.0},
            {"id": "uav_3", "position": [-75.0, 460.0], "role": "IDLE", "battery_capacity": 4200.0, "battery_energy": 4200.0},
            {"id": "uav_4", "position": [-75.0, 580.0], "role": "IDLE", "battery_capacity": 4200.0, "battery_energy": 4200.0},
        ),
        tasks=(
            {"id": "poi_offaxis_1", "position": [300.0, 100.0], "priority": 1, "spawn_time": 0.0, "service_duration": 2.0},
        ),
        challenge_profile=prof_cfg,
    )
    
    runner = MissionRunner(scenario=scenario, seed=2026)
    result = runner.run()
    
    metrics = result.metrics_report
    assert metrics is not None
    assert metrics.geofence_violation_count == 0, f"Expected 0 geofence violations, got {metrics.geofence_violation_count}"
    assert metrics.reports_deadline_exceeded == 0, f"Expected 0 deadline exceedances, got {metrics.reports_deadline_exceeded}"
