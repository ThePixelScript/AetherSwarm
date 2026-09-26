"""Unit tests for DynamicRelayManager shared trunk reference counting and chain lifecycle."""
from __future__ import annotations

from ares_swarm.autonomy.relay_manager import (
    ChainStatus,
    DynamicRelayManager,
    RelayChain,
)
from ares_swarm.core.enums import FailureState, Role, RTHState, SortieState
from ares_swarm.core.models import StateSnapshot, UAVState


def make_test_uav(
    uav_id: str,
    position: tuple[float, float] = (0.0, 500.0),
    role: Role = Role.IDLE,
    battery: float = 1000.0,
    active: bool = True,
) -> UAVState:
    return UAVState(
        id=uav_id,
        position_xy=position,
        velocity_xy=(0.0, 0.0),
        battery_energy=battery,
        battery_capacity=1000.0,
        role=role,
        active=active,
        failure_state=FailureState.NORMAL,
        rth_state=RTHState.NONE,
        sortie_state=SortieState.ACTIVE,
    )


def test_independent_chains_lifecycle():
    """Verify registration and teardown of two completely independent chains."""
    rm = DynamicRelayManager()

    chain_a = rm.register_chain(
        chain_id="chain_a",
        surveyor_id="surv_a",
        relay_ids=["r1", "r2"],
        station_positions=[(100.0, 500.0), (200.0, 500.0)],
        task_id="task_a",
    )
    chain_b = rm.register_chain(
        chain_id="chain_b",
        surveyor_id="surv_b",
        relay_ids=["r3", "r4"],
        station_positions=[(100.0, 600.0), (200.0, 600.0)],
        task_id="task_b",
    )

    assert rm.relay_dependent_surveyors["r1"] == {"surv_a"}
    assert rm.relay_dependent_surveyors["r2"] == {"surv_a"}
    assert rm.relay_dependent_surveyors["r3"] == {"surv_b"}
    assert rm.relay_dependent_surveyors["r4"] == {"surv_b"}

    cmds_a = rm.teardown_chain("chain_a")
    assert "r1" not in rm.relay_dependent_surveyors
    assert "r2" not in rm.relay_dependent_surveyors
    assert rm.relay_dependent_surveyors["r3"] == {"surv_b"}

    cmds_b = rm.teardown_chain("chain_b")
    assert "r3" not in rm.relay_dependent_surveyors
    assert "r4" not in rm.relay_dependent_surveyors


def test_shared_relay_trunk_reference_counting():
    """Verify reference counting when two surveyors share trunk relays r1 and r2."""
    rm = DynamicRelayManager()

    # Chain A: Trunk (r1, r2) -> Branch (r3) -> Surveyor A
    chain_a = rm.register_chain(
        chain_id="chain_a",
        surveyor_id="surv_a",
        relay_ids=["r1", "r2", "r3"],
        station_positions=[(100.0, 500.0), (200.0, 500.0), (300.0, 500.0)],
        task_id="task_a",
    )

    # Chain B: Trunk (r1, r2) -> Branch (r4) -> Surveyor B
    chain_b = rm.register_chain(
        chain_id="chain_b",
        surveyor_id="surv_b",
        relay_ids=["r1", "r2", "r4"],
        station_positions=[(100.0, 500.0), (200.0, 500.0), (300.0, 600.0)],
        task_id="task_b",
    )

    # Both surveyors depend on trunk relays r1, r2
    assert rm.relay_dependent_surveyors["r1"] == {"surv_a", "surv_b"}
    assert rm.relay_dependent_surveyors["r2"] == {"surv_a", "surv_b"}
    assert rm.relay_dependent_surveyors["r3"] == {"surv_a"}
    assert rm.relay_dependent_surveyors["r4"] == {"surv_b"}

    # Teardown Chain A (Surveyor A finishes)
    cmds_a = rm.teardown_chain("chain_a")

    # r3 (unique to A) should be released
    released_uavs = [c.uav_id for c in cmds_a if hasattr(c, "uav_id") and c.__class__.__name__ == "ReleaseRelayRoleCommand"]
    assert "r3" in released_uavs
    assert "r1" not in released_uavs
    assert "r2" not in released_uavs

    # Trunk relays r1, r2 remain active for Surveyor B
    assert rm.relay_dependent_surveyors["r1"] == {"surv_b"}
    assert rm.relay_dependent_surveyors["r2"] == {"surv_b"}
    assert "r3" not in rm.relay_dependent_surveyors

    # Final Teardown of Chain B
    cmds_b = rm.teardown_chain("chain_b")
    released_b = [c.uav_id for c in cmds_b if hasattr(c, "uav_id") and c.__class__.__name__ == "ReleaseRelayRoleCommand"]
    assert "r1" in released_b
    assert "r2" in released_b
    assert "r4" in released_b
    assert len(rm.relay_dependent_surveyors) == 0


def test_atomic_candidate_selection_insufficient_relays():
    """Verify select_relay_chain_candidates enforces atomic all-or-nothing allocation."""
    rm = DynamicRelayManager()

    uavs = {
        "surv_a": make_test_uav("surv_a", role=Role.SURVEYOR),
        "r1": make_test_uav("r1", position=(100.0, 500.0), role=Role.IDLE),
        # Only 1 idle UAV available, but 2 stations required
    }
    snapshot = StateSnapshot(uavs=uavs, gcs_position=(0.0, 500.0), simulation_tick=0, simulation_time=0.0, state_version=1)

    stations = [(100.0, 500.0), (200.0, 500.0)]
    cands = rm.select_relay_chain_candidates(
        snapshot=snapshot,
        surveyor_id="surv_a",
        stations=stations,
    )
    # Must fail atomically because 2 stations are requested but only 1 candidate exists
    assert cands is None
