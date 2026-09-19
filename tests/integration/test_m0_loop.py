from types import MappingProxyType

from ares_swarm.core.models import StateSnapshot, UAVState, TaskState
from ares_swarm.core.enums import TaskStatus, Role, RTHState, FailureState
from ares_swarm.core.state_store import StateStore
from ares_swarm.core.simulator import SimulationEngine
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.communication.config import CommunicationConfig
from ares_swarm.autonomy.task_allocator import A0TaskAllocator
from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter


def _build_initial_snapshot() -> StateSnapshot:
    """Construct deterministic 3-UAV, 1-GCS, 2-Task initial state."""
    uavs = {
        "u1": UAVState(
            id="u1",
            position_xy=(0.0, 0.0),
            battery_capacity=100.0,
            battery_energy=100.0,
            role=Role.IDLE,
            active=True,
            failure_state=FailureState.NORMAL,
            rth_state=RTHState.NONE,
        ),
        "u2": UAVState(
            id="u2",
            position_xy=(0.0, 0.0),
            battery_capacity=100.0,
            battery_energy=100.0,
            role=Role.IDLE,
            active=True,
            failure_state=FailureState.NORMAL,
            rth_state=RTHState.NONE,
        ),
        "u3": UAVState(
            id="u3",
            position_xy=(0.0, 0.0),
            battery_capacity=100.0,
            battery_energy=100.0,
            role=Role.IDLE,
            active=True,
            failure_state=FailureState.NORMAL,
            rth_state=RTHState.NONE,
        ),
    }

    tasks = {
        "t_high": TaskState(
            id="t_high",
            position_xy=(0.0, 10.0),
            priority=2.0,
            status=TaskStatus.PENDING,
        ),
        "t_low": TaskState(
            id="t_low",
            position_xy=(10.0, 0.0),
            priority=1.0,
            status=TaskStatus.PENDING,
        ),
    }

    return StateSnapshot(
        simulation_tick=0,
        simulation_time=0.0,
        state_version=0,
        uavs=MappingProxyType(uavs),
        tasks=MappingProxyType(tasks),
        gcs_position=(0.0, 0.0),
    )


def _execute_m0_loop(num_ticks: int = 5):
    """Run deterministic M0 integration loop and collect step metrics."""
    store = StateStore(_build_initial_snapshot())
    comm_analyzer = BaselineCommunicationAnalyzer(config=CommunicationConfig())
    adapter = A0AutonomyAdapter(allocator=A0TaskAllocator())
    engine = SimulationEngine(store, dt=1.0, idle_rate=1.0, movement_rate=0.5)

    history = []

    for tick in range(num_ticks):
        current_snap = store.snapshot()
        assert current_snap.simulation_tick == tick

        # 1. Gamma Communication Network Analysis
        net_analysis = comm_analyzer.analyze(current_snap)
        assert net_analysis.gcs_id == "gcs"
        assert set(net_analysis.connected_uav_ids) == {"u1", "u2", "u3"}

        # 2. Beta A0 Autonomy Allocation
        pending_tasks = [t for t in current_snap.tasks.values() if t.status == TaskStatus.PENDING]
        if pending_tasks:
            assign_cmds = adapter.plan(current_snap)
            assert len(assign_cmds) == 2
            for cmd in assign_cmds:
                assert cmd.source_tick == tick

            # 3. StateStore applies assignments
            assign_result = store.apply(assign_cmds)
            assert len(assign_result.rejected_commands) == 0
            assert len(assign_result.applied_commands) == len(assign_cmds)

        snap_after_assign = store.snapshot()
        assert snap_after_assign.uavs["u1"].target_position == (0.0, 10.0)
        assert snap_after_assign.uavs["u2"].target_position == (10.0, 0.0)
        assert snap_after_assign.uavs["u3"].target_position is None

        # 4. Delta Simulation Swarm Step (Batched)
        step_result = engine.step_swarm(speed=5.0)
        assert len(step_result.rejected_commands) == 0
        assert len(step_result.applied_commands) == 3

        # 5. Advance simulation clock
        new_time = engine.advance_tick()
        snap_next = store.snapshot()
        assert snap_next.simulation_tick == tick + 1
        assert snap_next.simulation_time == (tick + 1) * 1.0
        assert new_time == snap_next.simulation_time

        history.append({
            "tick": tick,
            "connected_uavs": net_analysis.connected_uav_ids,
            "positions": {uid: u.position_xy for uid, u in snap_next.uavs.items()},
            "velocities": {uid: u.velocity_xy for uid, u in snap_next.uavs.items()},
            "batteries": {uid: u.battery_energy for uid, u in snap_next.uavs.items()},
        })

    return history


def test_m0_loop_deterministic_end_to_end():
    """Verify closed-loop M0 pipeline execution and strict determinism across runs."""
    run_1 = _execute_m0_loop(num_ticks=5)
    run_2 = _execute_m0_loop(num_ticks=5)

    assert len(run_1) == len(run_2) == 5

    # Verify identical history across both independent executions
    for t in range(5):
        assert run_1[t]["connected_uavs"] == run_2[t]["connected_uavs"]
        assert run_1[t]["positions"] == run_2[t]["positions"]
        assert run_1[t]["velocities"] == run_2[t]["velocities"]
        assert run_1[t]["batteries"] == run_2[t]["batteries"]

    # Invariants at Tick 0 (first step: speed 5.0m/s * 1.0s = 5.0m traveled)
    t0_pos = run_1[0]["positions"]
    t0_bat = run_1[0]["batteries"]
    assert t0_pos["u1"] == (0.0, 5.0)
    assert t0_pos["u2"] == (5.0, 0.0)
    assert t0_pos["u3"] == (0.0, 0.0)
    # Energy: idle (1.0*1.0) + movement (0.5*5.0) = 3.5 consumed -> 96.5 remaining
    assert t0_bat["u1"] == 96.5
    assert t0_bat["u2"] == 96.5
    assert t0_bat["u3"] == 99.0

    # Invariants at Tick 1 (second step: arrives at target 10.0m)
    t1_pos = run_1[1]["positions"]
    t1_bat = run_1[1]["batteries"]
    assert t1_pos["u1"] == (0.0, 10.0)
    assert t1_pos["u2"] == (10.0, 0.0)
    assert t1_pos["u3"] == (0.0, 0.0)
    assert t1_bat["u1"] == 93.0
    assert t1_bat["u2"] == 93.0
    assert t1_bat["u3"] == 98.0

    # Invariants at final tick (Tick 4): stopped at target, zero velocity
    final_pos = run_1[4]["positions"]
    final_vel = run_1[4]["velocities"]
    final_bat = run_1[4]["batteries"]
    assert final_pos["u1"] == (0.0, 10.0)
    assert final_pos["u2"] == (10.0, 0.0)
    assert final_pos["u3"] == (0.0, 0.0)
    assert final_vel["u1"] == (0.0, 0.0)
    assert final_vel["u2"] == (0.0, 0.0)
    assert final_vel["u3"] == (0.0, 0.0)
    # Zero distance consumes only idle energy (1.0 Wh/tick) on ticks 2, 3, 4
    assert final_bat["u1"] == 90.0
    assert final_bat["u2"] == 90.0
    assert final_bat["u3"] == 95.0


def test_m0_step_swarm_uses_single_batched_transaction(monkeypatch):
    """Verify that step_swarm submits exactly one batched StateStore.apply transaction per tick."""
    store = StateStore(_build_initial_snapshot())
    adapter = A0AutonomyAdapter(allocator=A0TaskAllocator())
    engine = SimulationEngine(store, dt=1.0)

    # Assign tasks
    assign_cmds = adapter.plan(store.snapshot())
    store.apply(assign_cmds)

    apply_calls = []
    original_apply = store.apply

    def tracking_apply(commands):
        apply_calls.append(list(commands))
        return original_apply(commands)

    monkeypatch.setattr(store, "apply", tracking_apply)

    # Execute step_swarm
    step_result = engine.step_swarm(speed=5.0)

    assert len(apply_calls) == 1
    assert len(apply_calls[0]) == 3
    assert len(step_result.applied_commands) == 3
    assert len(step_result.rejected_commands) == 0
