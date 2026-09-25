from dataclasses import replace
from pathlib import Path
from unittest.mock import MagicMock
import pytest

from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.autonomy.ingress_coordinator import IngressCoordinator
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.core.commands import AssignRelayRoleCommand, Command
from ares_swarm.core.enums import Role, TaskStatus
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState
from ares_swarm.core.state_store import StateStore
from ares_swarm.safety.airspace import ChallengeAirspace, FlightPhase
from ares_swarm.safety.separation import SeparationEnforcer
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import load_scenario


def test_a_submission_runner_smoke_construction():
    import dataclasses

    base_scenario = load_scenario(Path("scenarios/poc_round1.yaml"))
    dp = dataclasses.replace(base_scenario.challenge_profile.detection_pipeline, enabled=True)
    airspace = dataclasses.replace(base_scenario.challenge_profile.airspace, enabled=True)
    cp = dataclasses.replace(
        base_scenario.challenge_profile,
        enabled=True,
        enforce_separation=True,
        enforce_sortie_limit=True,
        enforce_single_sortie=True,
        enforce_geofence=True,
        detection_pipeline=dp,
        airspace=airspace,
    )
    base_scenario = dataclasses.replace(base_scenario, challenge_profile=cp)

    comm_analyzer = BaselineCommunicationAnalyzer(config=base_scenario.communication)
    allocator = A1TaskAllocator(comm_analyzer=comm_analyzer)
    ingress_coordinator = IngressCoordinator(
        comm_analyzer=comm_analyzer,
        gcs_position=base_scenario.gcs_position,
        d_safe=85.0,
    )

    try:
        adapter = A0AutonomyAdapter(allocator=allocator, ingress_coordinator=ingress_coordinator)
    except TypeError:
        pytest.fail("A0AutonomyAdapter failed to construct with ingress_coordinator")

    runner = MissionRunner(
        scenario=base_scenario,
        seed=2026,
        autonomy_adapter=adapter,
        comm_analyzer=comm_analyzer,
    )
    assert runner is not None


def test_b_relay_role_command():
    store = StateStore()
    uav = UAVState(id="uav_1", position_xy=(0, 0), role=Role.IDLE)
    store._uavs["uav_1"] = uav

    cmd = AssignRelayRoleCommand(source_tick=0, uav_id="uav_1")
    res = store.apply([cmd])
    assert len(res.applied_commands) == 1
    assert store.snapshot().uavs["uav_1"].role == Role.RELAY


class UnsupportedCommand(Command):
    pass


def test_c_unsupported_command_rejection():
    store = StateStore()
    uav = UAVState(id="uav_1", position_xy=(0, 0))
    store._uavs["uav_1"] = uav

    cmd = UnsupportedCommand(source_tick=0, uav_id="uav_1")
    res = store.apply([cmd])
    assert len(res.applied_commands) == 0
    assert len(res.rejected_commands) == 1


def test_d_ingress_no_command_hold():
    allocator = MagicMock()
    allocator.plan.return_value = ["mock_command"]
    ingress_coordinator = MagicMock()
    ingress_coordinator.deployment_complete = False
    ingress_coordinator.plan.return_value = []

    try:
        adapter = A0AutonomyAdapter(allocator=allocator, ingress_coordinator=ingress_coordinator)
    except TypeError:
        pytest.fail("A0AutonomyAdapter failed to construct with ingress_coordinator")

    cmds = adapter.plan(MagicMock(), MagicMock())
    assert len(cmds) == 0
    allocator.plan.assert_not_called()


def test_e_egress_safety_regression():
    enforcer = SeparationEnforcer(min_separation_m=20.0)
    # u1 is STOPPED, u2 is flying AT u1
    u1 = UAVState(id="uav_1", position_xy=(10.0, 0.0), target_position=(10.0, 0.0), velocity_xy=(0.0, 0.0))
    u2 = UAVState(id="uav_2", position_xy=(30.1, 0.0), target_position=(0.0, 0.0), velocity_xy=(-5.0, 0.0))
    snap = StateSnapshot(state_version=0, simulation_tick=0, simulation_time=0.0, uavs={"uav_1": u1, "uav_2": u2}, tasks={})

    flight_phases = {"uav_1": FlightPhase.EGRESS, "uav_2": FlightPhase.EGRESS}
    cmds, evts = enforcer.enforce_step(snap, configured_speed=5.0, dt=1.0, flight_phases=flight_phases, idle_rate=1.0, movement_rate=0.5)

    pos1 = (10.0, 0.0)
    pos2 = (30.1, 0.0)
    for c in cmds:
        if c.uav_id == "uav_1":
            pos1 = c.new_position_xy
        if c.uav_id == "uav_2":
            pos2 = c.new_position_xy

    dist = ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5
    assert dist >= 20.0 - 1e-5


def test_f_grounded_staging():
    enforcer = SeparationEnforcer(min_separation_m=20.0)
    u1 = UAVState(id="uav_1", position_xy=(-75.0, 500.0))
    u2 = UAVState(id="uav_2", position_xy=(-73.0, 500.0))
    snap = StateSnapshot(state_version=0, simulation_tick=0, simulation_time=0.0, uavs={"uav_1": u1, "uav_2": u2}, tasks={})

    flight_phases = {"uav_1": FlightPhase.STAGING, "uav_2": FlightPhase.STAGING}
    cmds, evts = enforcer.enforce_step(snap, configured_speed=5.0, dt=1.0, flight_phases=flight_phases, idle_rate=1.0, movement_rate=0.5)
    assert len(evts) == 0


def test_g_deadline_separation():
    t = TaskState(id="t1", position_xy=(0, 0), priority=1, created_time=10.0, deadline=float("inf"))
    assert t.deadline == float("inf")
