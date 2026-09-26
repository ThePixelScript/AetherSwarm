"""Integration tests for multi-sortie recharge lifecycle (LANDED -> RECHARGING -> READY -> REASSIGNED)."""
from __future__ import annotations

import pytest

from ares_swarm.core.enums import EventType, Role, RTHState, SortieState, TaskStatus
from ares_swarm.core.models import TaskState, UAVState
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import (
    ChallengeAirspaceConfig,
    ChallengeProfileConfig,
    CommunicationConfig,
    DetectionPipelineConfig,
    ScenarioConfig,
)


def test_multi_sortie_recharge_and_reassignment():
    """Verify that a UAV can complete a task, return to GCS, recharge, and be re-assigned to a second task."""
    gcs = (-75.0, 500.0)
    airspace = ChallengeAirspaceConfig(
        enabled=True,
        staging_pad_center=gcs,
        staging_pad_radius_m=15.0,
        corridor_bounds_x=(-75.0, 0.0),
        corridor_bounds_y=(400.0, 600.0),
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        max_height=100.0,
    )
    detect_pipe = DetectionPipelineConfig(
        enabled=True,
        sensor_fov_radius_m=40.0,
        reporting_deadline_s=10.0,
        processing_delay_s=0.0,
    )
    prof = ChallengeProfileConfig(
        enabled=True,
        max_sortie_duration_s=1200.0,
        rth_safety_margin_s=15.0,
        recharge_duration_s=30.0,
        enforce_sortie_limit=True,
        enforce_single_sortie=False,
        enforce_separation=False,
        enforce_geofence=False,
        airspace=airspace,
        detection_pipeline=detect_pipe,
        enable_relay_manager=True,
        enable_connectivity_aware_planning=True,
    )
    sc = ScenarioConfig(
        name="multi_sortie_test",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=600.0,
        max_ticks=600,
        gcs_position=gcs,
        communication=CommunicationConfig(max_range=100.0),
        challenge_profile=prof,
        recharge_duration_s=30.0,
        return_by_mission_end=False,
        enable_connectivity_aware_planning=True,
        enable_relay_manager=True,
        enable_auto_rth=True,
        uavs=(
            {"id": "uav_1", "position": [-75.0, 500.0], "battery_capacity": 250.0, "battery_energy": 200.0, "role": "IDLE"},
        ),
        tasks=(
            {"id": "task_1", "position": [10.0, 500.0], "priority": 1, "service_duration": 10.0},
            {"id": "task_2", "position": [15.0, 500.0], "priority": 2, "service_duration": 10.0, "created_time": 100.0},
        ),
    )

    runner = MissionRunner(scenario=sc, seed=42)
    res = runner.run()

    recharge_events = [e for e in res.all_events if e.event_type == EventType.UAV_RECHARGED]
    assert len(recharge_events) >= 1
    assert res.final_snapshot.tasks["task_1"].status == TaskStatus.COMPLETE
    assert res.final_snapshot.tasks["task_2"].status == TaskStatus.COMPLETE
