"""Authoritative Regression Suite for Physical Relay Chain Readiness & RTH Geometry.

Validates:
A. FORMING chain with relays in transit: surveyor does not advance toward final planned station/POI.
B. Planned outer station exists but is physically empty: gives no movement authority.
C. Relays physically establish chain + Gamma verifies: FORMING -> ACTIVE transition.
D. ACTIVE chain: surveyor is released toward task/POI.
E. Task A completes -> old chain teardown -> Task B assigned -> new chain FORMING -> surveyor remains connected -> chain ACTIVE -> surveyor proceeds -> no comm-loss abort.
F. Initial chain formation regression (verifying Episode 1 elimination).
G. RTH geometry regression (verifying T431-T432 elimination).
"""
import math
from pathlib import Path
from types import MappingProxyType

import pytest

from ares_swarm.autonomy.connectivity_planner import (
    ConnectivityAwarePlanner,
    is_chain_physically_ready,
)
from ares_swarm.autonomy.relay_manager import (
    ChainStatus,
    DynamicRelayManager,
    RelayChain,
)
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.communication.config import CommunicationConfig
from ares_swarm.core.enums import FailureState, Role, RTHState, SortieState, TaskStatus
from ares_swarm.core.models import StateSnapshot, TaskState, UAVState
from ares_swarm.interfaces.communication import NetworkAnalysis
from ares_swarm.safety.rth_router import RTHRouter
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import load_scenario


def _make_uav(
    uav_id: str,
    position: tuple[float, float],
    role: Role = Role.IDLE,
    assigned_task_id: str | None = None,
    target_position: tuple[float, float] | None = None,
    battery: float = 4200.0,
    rth_state: RTHState = RTHState.NONE,
    sortie_state: SortieState = SortieState.READY,
) -> UAVState:
    return UAVState(
        id=uav_id,
        position_xy=position,
        target_position=target_position,
        role=role,
        assigned_task_id=assigned_task_id,
        battery_energy=battery,
        battery_capacity=battery,
        rth_state=rth_state,
        sortie_state=sortie_state,
    )


# --- Test A: FORMING chain with relays in transit -> surveyor does not advance ---
def test_a_forming_chain_surveyor_holds_position():
    """Test A: While chain is FORMING and relays are in transit, surveyor holds position."""
    rm = DynamicRelayManager()
    planner = ConnectivityAwarePlanner(relay_manager=rm, comm_range=100.0)

    station_0 = (10.0, 500.0)
    station_1 = (90.0, 500.0)
    chain = RelayChain(
        chain_id="chain_t1",
        task_id="t1",
        surveyor_id="uav_1",
        relay_ids=["uav_2", "uav_3"],
        station_positions=[station_0, station_1],
        created_tick=0,
        created_time=0.0,
        status=ChainStatus.FORMING,
    )
    rm.register_chain(chain)

    task = TaskState(id="t1", position_xy=(180.0, 500.0), priority=1, service_duration=10.0, status=TaskStatus.IN_PROGRESS, assigned_uav_id="uav_1")
    # Relays in transit (uav_2 not at station_0, uav_3 not at station_1)
    uav_1 = _make_uav("uav_1", position=(-75.0, 500.0), role=Role.SURVEYOR, assigned_task_id="t1", target_position=(-75.0, 500.0))
    uav_2 = _make_uav("uav_2", position=(-60.0, 500.0), role=Role.RELAY, target_position=station_0)
    uav_3 = _make_uav("uav_3", position=(-75.0, 480.0), role=Role.RELAY, target_position=(-75.0, 480.0))

    snapshot = StateSnapshot(
        uavs={"uav_1": uav_1, "uav_2": uav_2, "uav_3": uav_3},
        tasks={"t1": task},
        gcs_position=(-75.0, 500.0),
        simulation_tick=1,
        simulation_time=1.0,
        state_version=2,
    )
    analyzer = BaselineCommunicationAnalyzer(CommunicationConfig(max_range=100.0))
    net = analyzer.analyze(snapshot)

    cmds = planner.monitor_active_tasks(snapshot, network_analysis=net)
    # Surveyor must NOT receive a deep target or target derived from planned outer station
    surveyor_target_cmds = [c for c in cmds if hasattr(c, "uav_id") and c.uav_id == "uav_1" and hasattr(c, "target_position")]
    for c in surveyor_target_cmds:
        assert c.target_position == (-75.0, 500.0) or c.target_position == (0.0, 500.0)
        assert c.target_position[0] < 100.0
    assert chain.status == ChainStatus.FORMING


# --- Test B: Planned outer station exists but physically empty -> no movement authority ---
def test_b_planned_outer_station_empty_gives_no_authority():
    """Test B: Planned outer station gives no authority to surveyor unless physically occupied."""
    chain = RelayChain(
        chain_id="chain_t1",
        task_id="t1",
        surveyor_id="uav_1",
        relay_ids=["uav_2"],
        station_positions=[(85.0, 500.0)],
        created_tick=0,
        created_time=0.0,
        status=ChainStatus.FORMING,
    )
    # uav_2 is physically at (-70.0, 500.0), not at (85.0, 500.0)
    uav_1 = _make_uav("uav_1", position=(-75.0, 500.0), role=Role.SURVEYOR, assigned_task_id="t1")
    uav_2 = _make_uav("uav_2", position=(-70.0, 500.0), role=Role.RELAY)

    snapshot = StateSnapshot(
        uavs={"uav_1": uav_1, "uav_2": uav_2},
        tasks={},
        gcs_position=(-75.0, 500.0),
        simulation_tick=1,
        simulation_time=1.0,
        state_version=1,
    )
    analyzer = BaselineCommunicationAnalyzer(CommunicationConfig(max_range=100.0))
    net = analyzer.analyze(snapshot)

    assert not is_chain_physically_ready(chain, snapshot, net)


# --- Test C: Relays physically establish chain + Gamma verifies -> FORMING -> ACTIVE ---
def test_c_relays_physically_ready_transitions_forming_to_active():
    """Test C: When all relays reach assigned stations and Gamma confirms connectivity, chain becomes ACTIVE."""
    rm = DynamicRelayManager()
    planner = ConnectivityAwarePlanner(relay_manager=rm, comm_range=100.0)

    station_0 = (0.0, 500.0)
    station_1 = (85.0, 500.0)
    chain = RelayChain(
        chain_id="chain_t1",
        task_id="t1",
        surveyor_id="uav_1",
        relay_ids=["uav_2", "uav_3"],
        station_positions=[station_0, station_1],
        created_tick=0,
        created_time=0.0,
        status=ChainStatus.FORMING,
    )
    rm.register_chain(chain)

    task = TaskState(id="t1", position_xy=(150.0, 500.0), priority=1, service_duration=10.0, status=TaskStatus.IN_PROGRESS, assigned_uav_id="uav_1")
    # All relays in physical position & connected
    uav_1 = _make_uav("uav_1", position=(-75.0, 500.0), role=Role.SURVEYOR, assigned_task_id="t1")
    uav_2 = _make_uav("uav_2", position=station_0, role=Role.RELAY, target_position=station_0)
    uav_3 = _make_uav("uav_3", position=station_1, role=Role.RELAY, target_position=station_1)

    snapshot = StateSnapshot(
        uavs={"uav_1": uav_1, "uav_2": uav_2, "uav_3": uav_3},
        tasks={"t1": task},
        gcs_position=(-75.0, 500.0),
        simulation_tick=10,
        simulation_time=10.0,
        state_version=5,
    )
    analyzer = BaselineCommunicationAnalyzer(CommunicationConfig(max_range=100.0))
    net = analyzer.analyze(snapshot)

    assert is_chain_physically_ready(chain, snapshot, net)
    cmds = planner.monitor_active_tasks(snapshot, network_analysis=net)
    assert chain.status == ChainStatus.ACTIVE


# --- Test D: ACTIVE chain -> surveyor is released ---
def test_d_active_chain_releases_surveyor_to_poi():
    """Test D: Once chain is ACTIVE, surveyor is commanded towards the POI task."""
    rm = DynamicRelayManager()
    planner = ConnectivityAwarePlanner(relay_manager=rm, comm_range=100.0)

    station_0 = (0.0, 500.0)
    station_1 = (85.0, 500.0)
    chain = RelayChain(
        chain_id="chain_t1",
        task_id="t1",
        surveyor_id="uav_1",
        relay_ids=["uav_2", "uav_3"],
        station_positions=[station_0, station_1],
        created_tick=0,
        created_time=0.0,
        status=ChainStatus.ACTIVE,
    )
    rm.register_chain(chain)

    task = TaskState(id="t1", position_xy=(150.0, 500.0), priority=1, service_duration=10.0, status=TaskStatus.IN_PROGRESS, assigned_uav_id="uav_1")
    uav_1 = _make_uav("uav_1", position=(-20.0, 500.0), role=Role.SURVEYOR, assigned_task_id="t1", target_position=(-20.0, 500.0))
    uav_2 = _make_uav("uav_2", position=station_0, role=Role.RELAY, target_position=station_0)
    uav_3 = _make_uav("uav_3", position=station_1, role=Role.RELAY, target_position=station_1)

    snapshot = StateSnapshot(
        uavs={"uav_1": uav_1, "uav_2": uav_2, "uav_3": uav_3},
        tasks={"t1": task},
        gcs_position=(-75.0, 500.0),
        simulation_tick=20,
        simulation_time=20.0,
        state_version=10,
    )
    analyzer = BaselineCommunicationAnalyzer(CommunicationConfig(max_range=100.0))
    net = analyzer.analyze(snapshot)

    cmds = planner.monitor_active_tasks(snapshot, network_analysis=net)
    surv_cmds = [c for c in cmds if hasattr(c, "uav_id") and c.uav_id == "uav_1" and hasattr(c, "target_position")]
    assert len(surv_cmds) > 0
    assert surv_cmds[0].target_position == (150.0, 500.0)


# --- Test E: Task A completes -> Task B assigned -> continuous connectivity maintained ---
def test_e_task_handoff_and_chain_reconfiguration_continuity():
    """Test E: Task handoff maintains continuous physical connectivity throughout reconfiguration."""
    scenario = load_scenario("scenarios/random_seed_2026.yaml")
    runner = MissionRunner(scenario=scenario, seed=2026)

    # Run past initial formation through task completions
    disconnected_ticks = []
    for t in range(1, 650):
        step = runner.step()
        snap = step.snapshot
        net = runner.comm_analyzer.analyze(snap)
        eligible = {k for k, u in snap.uavs.items() if u.active and u.sortie_state.value not in ("LANDED", "RECHARGING")}
        missing = eligible - set(net.connected_uav_ids)
        if missing:
            disconnected_ticks.append((t, sorted(missing)))

    assert len(disconnected_ticks) == 0, f"Expected 0 disconnected ticks during handoff/reconfig, got {disconnected_ticks}"


# --- Test F: Initial chain formation regression (Episode 1) ---
def test_f_initial_chain_formation_zero_disconnections():
    """Test F: Initial chain formation (ticks 1..250) exhibits zero disconnections."""
    scenario = load_scenario("scenarios/random_seed_2026.yaml")
    runner = MissionRunner(scenario=scenario, seed=2026)

    disconnected_ticks = []
    for t in range(1, 260):
        step = runner.step()
        snap = step.snapshot
        net = runner.comm_analyzer.analyze(snap)
        eligible = {k for k, u in snap.uavs.items() if u.active and u.sortie_state.value not in ("LANDED", "RECHARGING")}
        missing = eligible - set(net.connected_uav_ids)
        if missing:
            disconnected_ticks.append((t, sorted(missing)))

    assert len(disconnected_ticks) == 0, f"Episode 1 reproduced with disconnections: {disconnected_ticks}"


# --- Test G: RTH geometry regression (Episode 3 / T431-T432) ---
def test_g_rth_geometry_within_comm_horizon():
    """Test G: All returning UAVs stay within communication horizon during RTH."""
    router = RTHRouter(gcs_position=(-75.0, 500.0), corridor_bounds_y=(400.0, 600.0))
    router.register_uav_lane("uav_8", 570.0)

    # Test extreme lane UAV (e.g. lane_y = 570.0)
    uav_extreme = _make_uav("uav_8", position=(50.0, 570.0), rth_state=RTHState.ACTIVE, sortie_state=SortieState.RTH)
    tgt_arena = router.get_rth_target(uav_extreme)
    # Portal target in arena must be within comm-safe portal [440, 560]
    assert tgt_arena[0] == 0.0
    assert 440.0 <= tgt_arena[1] <= 560.0
    dist_portal_to_gcs = math.hypot(tgt_arena[0] - (-75.0), tgt_arena[1] - 500.0)
    assert dist_portal_to_gcs <= 100.0

    # In corridor (x <= 0.5), targets staging pad
    uav_corridor = _make_uav("uav_8", position=(-10.0, 560.0), rth_state=RTHState.ACTIVE, sortie_state=SortieState.RTH)
    tgt_corridor = router.get_rth_target(uav_corridor)
    assert tgt_corridor[0] == -75.0
    assert tgt_corridor[1] == 570.0
