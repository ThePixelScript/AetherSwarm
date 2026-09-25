import dataclasses
import pytest

from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.communication.config import CommunicationConfig
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import ScenarioConfig


def create_base_scenario(uavs, tasks):
    return ScenarioConfig(
        name="test_feasibility",
        uavs=uavs,
        tasks=tasks,
        duration=400.0,
        gcs_position=(0.0, 0.0),
        communication=CommunicationConfig(max_range=100.0, base_latency=5.0),
    )


def test_articulation_regression():
    uavs = (
        {"id": "uav_1", "position": [80.0, 0.0], "role": "IDLE"},
        {"id": "uav_2", "position": [160.0, 0.0], "role": "IDLE"},
        {"id": "uav_3", "position": [240.0, 0.0], "role": "IDLE"},
    )
    tasks = (
        {
            "id": "poi_1",
            "position": [80.0, 80.0],
            "spawn_time": 0.0,
            "priority": 1,
        },
    )
    scen = create_base_scenario(uavs, tasks)

    analyzer = BaselineCommunicationAnalyzer(config=scen.communication)
    allocator = A1TaskAllocator(comm_analyzer=analyzer)
    runner = MissionRunner(
        scenario=scen,
        autonomy_adapter=A0AutonomyAdapter(allocator=allocator),
        comm_analyzer=analyzer,
    )
    runner.step()
    snap = runner.history[0].snapshot
    net = runner.history[0].network_analysis

    uav_1 = snap.uavs["uav_1"]
    uav_2 = snap.uavs["uav_2"]
    uav_3 = snap.uavs["uav_3"]
    task = snap.tasks["poi_1"]

    assert "uav_1" in net.connected_uav_ids
    assert "uav_2" in net.connected_uav_ids
    assert "uav_3" in net.connected_uav_ids

    feasible, hyp_net = allocator._evaluate_swarm_connectivity_preservation(uav_2, task, snap, net)
    assert not feasible, "uav_2 should be rejected because moving it disconnects uav_3"
    assert "uav_3" not in hyp_net.connected_uav_ids
    assert "uav_2" in hyp_net.connected_uav_ids


def test_already_disconnected_peer_regression():
    uavs = (
        {"id": "uav_1", "position": [80.0, 0.0], "role": "IDLE"},
        {"id": "uav_2", "position": [160.0, 0.0], "role": "IDLE"},
        {"id": "uav_4", "position": [500.0, 0.0], "role": "IDLE"},
    )
    tasks = (
        {
            "id": "poi_1",
            "position": [160.0, 40.0],
            "spawn_time": 0.0,
        },
    )
    scen = create_base_scenario(uavs, tasks)

    analyzer = BaselineCommunicationAnalyzer(config=scen.communication)
    allocator = A1TaskAllocator(comm_analyzer=analyzer)
    runner = MissionRunner(
        scenario=scen,
        autonomy_adapter=A0AutonomyAdapter(allocator=allocator),
        comm_analyzer=analyzer,
    )
    runner.step()
    snap = runner.history[0].snapshot
    net = runner.history[0].network_analysis

    assert "uav_4" not in net.connected_uav_ids
    assert "uav_1" in net.connected_uav_ids
    assert "uav_2" in net.connected_uav_ids

    uav_2 = snap.uavs["uav_2"]
    task = snap.tasks["poi_1"]

    feasible, hyp_net = allocator._evaluate_swarm_connectivity_preservation(uav_2, task, snap, net)
    assert feasible, "uav_2 should be feasible because no PREVIOUSLY connected peer is lost"


def test_dead_zone_regression():
    uavs = (
        {"id": "uav_1", "position": [80.0, 0.0], "role": "IDLE"},
    )
    tasks = (
        {
            "id": "poi_1",
            "position": [200.0, 0.0],
            "spawn_time": 0.0,
        },
    )
    scen = create_base_scenario(uavs, tasks)
    analyzer = BaselineCommunicationAnalyzer(config=scen.communication)
    allocator = A1TaskAllocator(comm_analyzer=analyzer)
    runner = MissionRunner(
        scenario=scen,
        autonomy_adapter=A0AutonomyAdapter(allocator=allocator),
        comm_analyzer=analyzer,
    )
    runner.step()
    snap = runner.history[0].snapshot
    net = runner.history[0].network_analysis

    uav_1 = snap.uavs["uav_1"]
    task = snap.tasks["poi_1"]

    feasible, hyp_net = allocator._evaluate_swarm_connectivity_preservation(uav_1, task, snap, net)
    assert not feasible, "uav_1 should be rejected because it enters a dead zone"


def test_normal_feasible_assignment_regression():
    uavs = (
        {"id": "uav_1", "position": [80.0, 0.0], "role": "IDLE"},
    )
    tasks = (
        {
            "id": "poi_1",
            "position": [50.0, 0.0],
            "spawn_time": 0.0,
        },
    )
    scen = create_base_scenario(uavs, tasks)
    analyzer = BaselineCommunicationAnalyzer(config=scen.communication)
    allocator = A1TaskAllocator(comm_analyzer=analyzer)
    runner = MissionRunner(
        scenario=scen,
        autonomy_adapter=A0AutonomyAdapter(allocator=allocator),
        comm_analyzer=analyzer,
    )
    runner.step()
    snap = runner.history[0].snapshot
    net = runner.history[0].network_analysis

    uav_1 = snap.uavs["uav_1"]
    task = snap.tasks["poi_1"]

    feasible, hyp_net = allocator._evaluate_swarm_connectivity_preservation(uav_1, task, snap, net)
    assert feasible, "uav_1 should be feasible"
