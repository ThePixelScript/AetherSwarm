"""Unit and regression tests for hidden and emerging POI discovery in AetherSwarm.

Verifies:
1. Exact detection radius table (20m->80m, 40m->130m, 60m->170m, 80m->190m, 100m->230m)
2. Linear interpolation between altitude points
3. Clamping below 20m and above 100m
4. Hidden POI not available before emergence
5. Emerged POI remains hidden until detection
6. POI becomes discoverable inside detection footprint
7. POI not detected outside footprint
8. Detection reveals/creates normal task exactly once
9. Discovered high-priority POI enters existing allocator
10. Deterministic random hidden/emerging POIs from seed
11. Existing fixed/known POI behavior unchanged
"""
from __future__ import annotations

import math
from pathlib import Path
import sys
from types import MappingProxyType

import pytest

# Ensure repo root and scripts are in sys.path
REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.core.commands import DiscoverTaskCommand
from ares_swarm.core.enums import EventType, FailureState, Role, RTHState, TaskStatus
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState
from ares_swarm.core.state_store import StateStore
from ares_swarm.simulation.discovery import (
    ALTITUDE_DETECTION_TABLE,
    DiscoveryManager,
    HiddenPOI,
    compute_detection_radius,
    get_uav_altitude,
)
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import ScenarioConfig, create_initial_snapshot, load_scenario
from scripts.generate_scenario import (
    generate_and_export_scenario,
    generate_final_mission_scenario,
    sample_random_hidden_pois,
    sample_random_pois,
)


def test_1_exact_detection_radius_table():
    """Verify exact detection radius values at table altitudes."""
    assert compute_detection_radius(20.0) == 80.0
    assert compute_detection_radius(40.0) == 130.0
    assert compute_detection_radius(60.0) == 170.0
    assert compute_detection_radius(80.0) == 190.0
    assert compute_detection_radius(100.0) == 230.0


def test_2_detection_radius_interpolation():
    """Verify linear interpolation between defined altitude points."""
    # 30m: halfway between 20m (80m) and 40m (130m) -> 105m
    assert math.isclose(compute_detection_radius(30.0), 105.0, rel_tol=1e-5)
    # 50m: halfway between 40m (130m) and 60m (170m) -> 150m
    assert math.isclose(compute_detection_radius(50.0), 150.0, rel_tol=1e-5)
    # 70m: halfway between 60m (170m) and 80m (190m) -> 180m
    assert math.isclose(compute_detection_radius(70.0), 180.0, rel_tol=1e-5)
    # 90m: halfway between 80m (190m) and 100m (230m) -> 210m
    assert math.isclose(compute_detection_radius(90.0), 210.0, rel_tol=1e-5)
    # 25m: 25% between 80 and 130 -> 80 + 0.25 * 50 = 92.5m
    assert math.isclose(compute_detection_radius(25.0), 92.5, rel_tol=1e-5)


def test_3_detection_radius_clamping():
    """Verify clamping below 20m (to 80m) and above 100m (to 230m)."""
    assert compute_detection_radius(0.0) == 80.0
    assert compute_detection_radius(-10.0) == 80.0
    assert compute_detection_radius(15.0) == 80.0
    assert compute_detection_radius(19.99) == 80.0
    assert compute_detection_radius(100.01) == 230.0
    assert compute_detection_radius(150.0) == 230.0
    assert compute_detection_radius(500.0) == 230.0


def test_4_hidden_poi_not_available_before_emergence():
    """Verify an emerging POI cannot be discovered before its emergence time."""
    mgr = DiscoveryManager(hidden_pois=[
        HiddenPOI(
            id="emerging_01",
            position_xy=(50.0, 50.0),
            priority=2,
            emergence_time=50.0,
        )
    ])
    # UAV right over (50.0, 50.0) at altitude 20m, but sim_time is 10.0s < 50.0s
    uav = UAVState(id="uav_1", position_xy=(50.0, 50.0), altitude_m=20.0, active=True)
    snap = StateSnapshot(
        simulation_tick=10,
        simulation_time=10.0,
        state_version=0,
        uavs=MappingProxyType({"uav_1": uav}),
        tasks=MappingProxyType({}),
    )
    cmds = mgr.step(snap)
    assert len(cmds) == 0
    assert not mgr.is_discovered("emerging_01")
    assert "emerging_01" not in snap.tasks


def test_5_emerged_poi_remains_hidden_until_detection():
    """Verify that an emerged POI is not exposed to planners until a UAV footprint reaches it."""
    mgr = DiscoveryManager(hidden_pois=[
        HiddenPOI(
            id="emerging_02",
            position_xy=(400.0, 400.0),
            priority=3,
            emergence_time=10.0,
        )
    ])
    # Emerged at 10.0s, now sim_time is 25.0s, but UAV is at (0.0, 0.0)
    uav = UAVState(id="uav_1", position_xy=(0.0, 0.0), altitude_m=20.0, active=True)
    snap = StateSnapshot(
        simulation_tick=25,
        simulation_time=25.0,
        state_version=0,
        uavs=MappingProxyType({"uav_1": uav}),
        tasks=MappingProxyType({}),
    )
    cmds = mgr.step(snap)
    assert len(cmds) == 0
    assert not mgr.is_discovered("emerging_02")
    assert "emerging_02" not in snap.tasks


def test_6_poi_discovered_inside_detection_footprint():
    """Verify that a POI within UAV altitude-derived detection footprint is discovered."""
    mgr = DiscoveryManager(hidden_pois=[
        HiddenPOI(
            id="hidden_03",
            position_xy=(100.0, 0.0),
            priority=3,
            emergence_time=0.0,
        )
    ])
    # UAV at (0.0, 0.0) with altitude 40m -> footprint radius 130m.
    # POI distance is 100m <= 130m.
    uav = UAVState(id="uav_1", position_xy=(0.0, 0.0), altitude_m=40.0, active=True)
    snap = StateSnapshot(
        simulation_tick=1,
        simulation_time=1.0,
        state_version=0,
        uavs=MappingProxyType({"uav_1": uav}),
        tasks=MappingProxyType({}),
    )
    cmds = mgr.step(snap)
    assert len(cmds) == 1
    cmd = cmds[0]
    assert isinstance(cmd, DiscoverTaskCommand)
    assert cmd.task.id == "hidden_03"
    assert cmd.task.position_xy == (100.0, 0.0)
    assert cmd.task.priority == 3
    assert cmd.task.status == TaskStatus.PENDING
    assert mgr.is_discovered("hidden_03")


def test_7_poi_not_detected_outside_footprint():
    """Verify that a POI outside the UAV altitude-derived footprint is NOT detected."""
    mgr = DiscoveryManager(hidden_pois=[
        HiddenPOI(
            id="hidden_04",
            position_xy=(85.0, 0.0),
            priority=1,
            emergence_time=0.0,
        )
    ])
    # UAV at (0.0, 0.0) with altitude 20m -> radius 80m. POI is at 85m > 80m.
    uav = UAVState(id="uav_1", position_xy=(0.0, 0.0), altitude_m=20.0, active=True)
    snap = StateSnapshot(
        simulation_tick=1,
        simulation_time=1.0,
        state_version=0,
        uavs=MappingProxyType({"uav_1": uav}),
        tasks=MappingProxyType({}),
    )
    cmds = mgr.step(snap)
    assert len(cmds) == 0
    assert not mgr.is_discovered("hidden_04")

    # Now if UAV climbs to 30m -> radius becomes 105m > 85m -> POI is discovered!
    uav_high = UAVState(id="uav_1", position_xy=(0.0, 0.0), altitude_m=30.0, active=True)
    snap_high = StateSnapshot(
        simulation_tick=2,
        simulation_time=2.0,
        state_version=0,
        uavs=MappingProxyType({"uav_1": uav_high}),
        tasks=MappingProxyType({}),
    )
    cmds_high = mgr.step(snap_high)
    assert len(cmds_high) == 1
    assert cmds_high[0].task.id == "hidden_04"


def test_8_detection_reveals_task_exactly_once():
    """Verify that StateStore registers the task and does not repeat discovery."""
    mgr = DiscoveryManager(hidden_pois=[
        {"id": "hidden_05", "position": [40.0, 0.0], "priority": 2, "emergence_time": 0.0}
    ])
    uav = UAVState(id="uav_1", position_xy=(0.0, 0.0), altitude_m=20.0, active=True)
    store = StateStore(StateSnapshot(
        simulation_tick=1,
        simulation_time=1.0,
        state_version=0,
        uavs=MappingProxyType({"uav_1": uav}),
        tasks=MappingProxyType({}),
    ))

    # Tick 1: discovered
    cmds1 = mgr.step(store.snapshot())
    assert len(cmds1) == 1
    res1 = store.apply(cmds1)
    assert any(e.event_type == EventType.POI_DISCOVERED for e in res1.emitted_events)
    assert "hidden_05" in store.snapshot().tasks
    assert store.snapshot().tasks["hidden_05"].status == TaskStatus.PENDING

    # Tick 2: UAV stays in range, but no duplicate discovery occurs
    cmds2 = mgr.step(store.snapshot())
    assert len(cmds2) == 0
    summary = mgr.get_summary()
    assert summary["total_hidden_pois"] == 1
    assert summary["discovered_pois"] == 1
    assert summary["undiscovered_pois"] == 0


def test_9_discovered_high_priority_poi_enters_allocator():
    """Verify that a discovered high-priority POI is processed and assigned by the existing allocator."""
    allocator = A1TaskAllocator()
    uav = UAVState(id="uav_1", position_xy=(10.0, 10.0), battery_capacity=100.0, battery_energy=100.0, active=True)
    t_low = TaskState(id="low_1", position_xy=(50.0, 50.0), priority=1, status=TaskStatus.PENDING, created_time=0.0)
    t_high = TaskState(id="discovered_urgent", position_xy=(20.0, 20.0), priority=3, status=TaskStatus.PENDING, created_time=5.0)

    snap = StateSnapshot(
        simulation_tick=5,
        simulation_time=5.0,
        state_version=2,
        uavs=MappingProxyType({"uav_1": uav}),
        tasks=MappingProxyType({"low_1": t_low, "discovered_urgent": t_high}),
    )
    res = allocator.allocate(snap)
    assert len(res.assignments) > 0
    # The higher priority task should be assigned
    assert any(a.task_id == "discovered_urgent" for a in res.assignments)


def test_10_deterministic_random_hidden_pois_from_seed():
    """Verify reproducible hidden and emerging POI generation from random seeds."""
    pois_a1 = sample_random_hidden_pois(seed=2026, num_hidden_pois=6)
    pois_a2 = sample_random_hidden_pois(seed=2026, num_hidden_pois=6)
    assert len(pois_a1) == 6
    assert pois_a1 == pois_a2

    # Different seed produces different positions
    pois_b = sample_random_hidden_pois(seed=2027, num_hidden_pois=6)
    assert len(pois_b) == 6
    assert pois_a1 != pois_b

    # Verify hidden flag and emergence_time exist on each POI
    for p in pois_a1:
        assert p["hidden"] is True
        assert "emergence_time" in p
        assert p["priority"] in (1, 2, 3)


def test_11_existing_known_poi_behavior_unchanged():
    """Verify that baseline scenarios with known POIs operate with zero regression."""
    scen_dict = {
        "name": "baseline_known_pois",
        "uavs": [
            {"id": "uav_1", "position": [-75.0, 500.0], "battery_capacity": 100.0}
        ],
        "tasks": [
            {"id": "known_1", "position": [100.0, 100.0], "priority": 1, "service_duration": 2.0},
            {"id": "known_2", "position": [200.0, 200.0], "priority": 2, "service_duration": 2.0},
        ],
    }
    cfg = load_scenario(scen_dict)
    snap = create_initial_snapshot(cfg)
    # Both known tasks must appear in initial snapshot
    assert "known_1" in snap.tasks
    assert "known_2" in snap.tasks
    assert len(snap.tasks) == 2

    # If hidden tasks are provided alongside known tasks, hidden are excluded from initial snapshot
    scen_with_hidden = {
        "name": "mixed_scenario",
        "uavs": [{"id": "uav_1", "position": [-75.0, 500.0]}],
        "tasks": [
            {"id": "known_1", "position": [100.0, 100.0], "priority": 1},
            {"id": "hidden_secret", "position": [300.0, 300.0], "priority": 3, "hidden": True},
        ],
        "hidden_pois": [
            {"id": "hidden_in_list", "position": [400.0, 400.0], "priority": 2, "emergence_time": 20.0}
        ]
    }
    cfg_mixed = load_scenario(scen_with_hidden)
    snap_mixed = create_initial_snapshot(cfg_mixed)
    # Only known_1 should be in initial snapshot tasks
    assert "known_1" in snap_mixed.tasks
    assert "hidden_secret" not in snap_mixed.tasks
    assert "hidden_in_list" not in snap_mixed.tasks
    assert len(snap_mixed.tasks) == 1


def test_12_mission_runner_end_to_end_discovery():
    """Verify MissionRunner end-to-end discovery in a full simulation run."""
    scen_dict = {
        "name": "e2e_discovery",
        "seed": 42,
        "dt": 1.0,
        "duration": 5.0,
        "max_ticks": 5,
        "speed_limit": 5.0,
        "min_separation_m": 20.0,
        "gcs_position": [0.0, 0.0],
        "uavs": [
            {"id": "uav_1", "position": [0.0, 0.0], "altitude_m": 40.0, "battery_capacity": 1000.0, "battery_energy": 1000.0}
        ],
        "tasks": [],
        "hidden_pois": [
            {
                "id": "discovered_poi",
                "position": [50.0, 0.0],
                "priority": 3,
                "emergence_time": 1.0,
                "service_duration": 1.0,
            }
        ],
    }
    runner = MissionRunner(scenario=scen_dict)
    assert runner.discovery_manager is not None
    assert runner.discovery_manager.total_hidden_pois == 1
    assert "discovered_poi" not in runner.initial_snapshot.tasks

    res = runner.run()
    # Task should have been discovered and added during the run
    assert "discovered_poi" in res.final_snapshot.tasks
    d = res.to_dict()
    assert "discovery" in d
    assert d["discovery"]["discovered_pois"] == 1
    assert d["discovery"]["undiscovered_pois"] == 0


def test_13_multi_emerging_pois_independent_discovery():
    """Verify multiple emerging POIs are independently discoverable exactly once."""
    scen_dict = {
        "name": "multi_discovery",
        "seed": 2026,
        "dt": 1.0,
        "duration": 50.0,
        "max_ticks": 50,
        "speed_limit": 5.0,
        "gcs_position": [0.0, 0.0],
        "uavs": [
            {"id": "uav_1", "position": [0.0, 0.0], "altitude_m": 40.0, "battery_capacity": 5000.0, "battery_energy": 5000.0},
            {"id": "uav_2", "position": [0.0, 0.0], "altitude_m": 100.0, "battery_capacity": 5000.0, "battery_energy": 5000.0},
        ],
        "tasks": [],
        "hidden_pois": [
            {"id": "emerging_alpha", "position": [80.0, 0.0], "priority": 3, "emergence_time": 5.0},
            {"id": "emerging_beta", "position": [180.0, 0.0], "priority": 2, "emergence_time": 10.0},
        ],
    }
    runner = MissionRunner(scenario=scen_dict)
    # Check that neither appears in initial snapshot
    assert "emerging_alpha" not in runner.initial_snapshot.tasks
    assert "emerging_beta" not in runner.initial_snapshot.tasks

    res = runner.run()
    disc_events = [e for e in res.all_events if e.event_type == EventType.POI_DISCOVERED]
    alpha_events = [e for e in disc_events if e.entity_id == "emerging_alpha"]
    beta_events = [e for e in disc_events if e.entity_id == "emerging_beta"]

    assert len(alpha_events) == 1, "emerging_alpha must be discovered exactly once"
    assert len(beta_events) == 1, "emerging_beta must be discovered exactly once"
    assert "emerging_alpha" in res.final_snapshot.tasks
    assert "emerging_beta" in res.final_snapshot.tasks


def test_14_hidden_count_always_five_to_seven():
    """Verify that randomly generated hidden POI count is strictly within 5..7 across many seeds."""
    for s in range(1, 101):
        pois = sample_random_hidden_pois(seed=s)
        assert 5 <= len(pois) <= 7, f"Seed {s}: count {len(pois)} not in 5..7"


def test_15_same_seed_produces_identical_hidden_pois():
    """Verify bit-for-bit reproducibility for identical seeds."""
    for s in (42, 101, 2026, 9999):
        pois_1 = sample_random_hidden_pois(seed=s)
        pois_2 = sample_random_hidden_pois(seed=s)
        assert pois_1 == pois_2, f"Seed {s}: mismatch between identical seed runs"


def test_16_high_priority_count_is_one_to_four_and_never_exceeds_total():
    """Verify high-priority (priority=3) count is in 1..4 and never exceeds hidden count."""
    for s in range(1, 101):
        pois = sample_random_hidden_pois(seed=s)
        high_prio_count = sum(1 for p in pois if p["priority"] == 3)
        assert 1 <= high_prio_count <= 4, f"Seed {s}: high_priority_count={high_prio_count} not in 1..4"
        assert high_prio_count <= len(pois), f"Seed {s}: high_priority_count exceeds total"
        normal_pois = [p for p in pois if p["priority"] != 3]
        for np in normal_pois:
            assert np["priority"] in (1, 2)

    # Edge cases: explicit small counts
    for count in (1, 2, 3, 4):
        pois_small = sample_random_hidden_pois(seed=42, num_hidden_pois=count)
        hp = sum(1 for p in pois_small if p["priority"] == 3)
        assert 1 <= hp <= count, f"count={count}: hp={hp} exceeded count"


def test_17_all_hidden_pois_have_valid_emergence_times():
    """Verify all hidden POIs have emergence times within the configured window."""
    window = (15.0, 180.0)
    for s in (42, 101, 2026):
        pois = sample_random_hidden_pois(seed=s, emergence_window=window)
        for p in pois:
            assert window[0] <= p["emergence_time"] <= window[1], (
                f"Seed {s}: POI {p['id']} emergence {p['emergence_time']} outside {window}"
            )


def test_18_all_hidden_pois_have_random_in_arena_locations():
    """Verify all hidden POIs lie strictly within arena bounds and vary across POIs."""
    for s in (42, 101, 2026):
        pois = sample_random_hidden_pois(seed=s)
        coords = set()
        for p in pois:
            x, y = p["position"]
            assert 5.0 <= x <= 995.0, f"Seed {s}: x={x} out of arena bounds"
            assert 5.0 <= y <= 995.0, f"Seed {s}: y={y} out of arena bounds"
            coords.add((x, y))
        assert len(coords) == len(pois), f"Seed {s}: duplicate coordinates found"


def test_19_existing_known_poi_generation_preserved_when_hidden_disabled():
    """Verify that when hidden POIs are disabled (num_hidden_pois=0), known POI generation is preserved."""
    assert sample_random_hidden_pois(seed=42, num_hidden_pois=0) == []
    res = generate_and_export_scenario(seed=2026, num_pois=10, num_hidden_pois=0, run_simulation=False)
    assert len(res["tasks"]) == 10
    assert len(res["hidden_pois"]) == 0


def test_20_final_profile_known_poi_count_is_exactly_five():
    """Requirement 1: Verify known POI count is exactly 5 in the final mission profile."""
    for s in (42, 101, 2026, 9999, 12345):
        scen = generate_final_mission_scenario(seed=s, run_simulation=False, save_scenario=False)
        assert len(scen["tasks"]) == 5, f"Seed {s}: expected 5 known POIs, got {len(scen['tasks'])}"


def test_21_final_profile_hidden_count_and_total_count():
    """Requirements 2 & 3: Hidden POI count is always 5..7 and total POI count is therefore 10..12."""
    for s in range(1, 101):
        scen = generate_final_mission_scenario(seed=s, run_simulation=False, save_scenario=False)
        known_count = len(scen["tasks"])
        hidden_count = len(scen["hidden_pois"])
        total_count = known_count + hidden_count
        assert known_count == 5, f"Seed {s}: known={known_count} != 5"
        assert 5 <= hidden_count <= 7, f"Seed {s}: hidden={hidden_count} not in 5..7"
        assert 10 <= total_count <= 12, f"Seed {s}: total={total_count} not in 10..12"


def test_22_final_profile_high_priority_hidden_count_and_isolation():
    """Requirement 4: High-priority hidden POI count is 1..4 (priority=3), others priority=1 or 2, emergency_flag separate."""
    for s in range(1, 101):
        scen = generate_final_mission_scenario(seed=s, run_simulation=False, save_scenario=False)
        hidden = scen["hidden_pois"]
        hp_count = sum(1 for p in hidden if p["priority"] == 3)
        assert 1 <= hp_count <= 4, f"Seed {s}: hp_count={hp_count} not in 1..4"
        assert hp_count <= len(hidden)
        for p in hidden:
            assert not p.get("emergency_flag", False), "emergency_flag must not be set on priority-3 POIs"
            assert p["priority"] in (1, 2, 3)


def test_23_final_profile_same_seed_gives_identical_scenario():
    """Requirement 5: Same seed produces bit-for-bit identical generated scenario."""
    for s in (42, 101, 2026, 9999):
        scen_a = generate_final_mission_scenario(seed=s, run_simulation=False, save_scenario=False)
        scen_b = generate_final_mission_scenario(seed=s, run_simulation=False, save_scenario=False)
        assert scen_a["tasks"] == scen_b["tasks"]
        assert scen_a["hidden_pois"] == scen_b["hidden_pois"]
        assert scen_a["poi_metrics"] == scen_b["poi_metrics"]


def test_24_final_profile_emergence_times_distinct_and_independent():
    """Requirement 6: Hidden emergence times are independently randomized and distinct (non-simultaneous)."""
    window = (10.0, 300.0)
    for s in range(1, 101):
        hidden = sample_random_hidden_pois(seed=s, emergence_window=window)
        times = [p["emergence_time"] for p in hidden]
        assert len(times) == len(set(times)), f"Seed {s}: duplicate emergence times found: {times}"
        for t in times:
            assert window[0] <= t <= window[1]


def test_25_final_profile_known_visible_at_start_and_hidden_isolated(tmp_path):
    """Requirements 7 & 8: Known POIs visible at mission start; hidden POIs excluded until discovery."""
    scen_file = tmp_path / "final_profile_scenario.yaml"
    res = generate_final_mission_scenario(seed=2026, output_scenario=scen_file, run_simulation=False)

    config = load_scenario(str(scen_file))
    initial_snapshot = create_initial_snapshot(config)
    # Requirement 7: Exactly 5 known POIs visible from start
    assert len(initial_snapshot.tasks) == 5
    for t in res["tasks"]:
        assert t["id"] in initial_snapshot.tasks
        assert initial_snapshot.tasks[t["id"]].status == TaskStatus.PENDING

    # Requirement 8: All hidden POIs remain invisible in initial snapshot
    for h in res["hidden_pois"]:
        assert h["id"] not in initial_snapshot.tasks


def test_26_final_profile_discovered_hidden_pois_enter_allocation(tmp_path):
    """Requirement 9: Discovered hidden POIs enter normal allocation and existing priority allocator."""
    scen_file = tmp_path / "final_profile_alloc.yaml"
    # Place hidden POI right inside initial UAV coverage
    res = generate_and_export_scenario(
        seed=2026,
        final_profile=True,
        emergence_start=0.0,
        emergence_end=1.0,
        custom_x_range=(10.0, 50.0),
        custom_y_range=(480.0, 520.0),
        output_scenario=scen_file,
        max_ticks=60,
        run_simulation=True,
    )
    assert res is not None
    # Verify trace was generated
    trace_path = Path(res["trace_path"])
    assert trace_path.exists()




