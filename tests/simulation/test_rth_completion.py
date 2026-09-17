"""Unit and integration tests for canonical RTH completion lifecycle in MissionRunner."""
import pytest
from ares_swarm.core.enums import EventType, Role, RTHState, TaskStatus
from ares_swarm.core.commands import CompleteRTHCommand, StartRTHCommand
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.simulation.scenario import ScenarioConfig


def test_rth_completion_lifecycle_in_mission_runner():
    """Verify complete RTH lifecycle in MissionRunner:
    StartRTH -> move toward GCS -> arrival -> CompleteRTHCommand -> StateStore updates -> UAV_LANDED event.
    """
    config = ScenarioConfig(
        name="test_rth_lifecycle",
        seed=42,
        dt=1.0,
        speed_limit=5.0,
        duration=10.0,
        max_ticks=10,
        gcs_position=(0.0, 0.0),
        uavs=(
            {
                "id": "u1",
                "position": [10.0, 0.0],
                "battery_capacity": 100.0,
                "battery_energy": 100.0,
                "role": "IDLE",
                "active": True,
            },
        ),
        tasks=(
            {"id": "t1", "position": [50.0, 0.0], "priority": 1, "service_duration": 1.0},
        ),
        enable_auto_rth=False,
    )

    runner = MissionRunner(config)

    # Initial state: UAV at (10, 0), active, RTHState.NONE
    snap0 = runner.state_store.snapshot()
    assert snap0.uavs["u1"].rth_state == RTHState.NONE
    assert snap0.uavs["u1"].active is True
    assert snap0.uavs["u1"].position_xy == (10.0, 0.0)

    # Trigger RTH via StateStore
    res_rth = runner.state_store.apply([StartRTHCommand(source_tick=0, uav_id="u1")])
    assert len(res_rth.applied_commands) == 1
    snap_after_rth = runner.state_store.snapshot()
    assert snap_after_rth.uavs["u1"].rth_state == RTHState.ACTIVE
    assert snap_after_rth.uavs["u1"].target_position == (0.0, 0.0)
    assert snap_after_rth.uavs["u1"].active is True

    # Step 1: UAV flies from (10, 0) towards (0, 0) at 5 m/s -> reaches (5, 0)
    step1 = runner.step()
    u1_s1 = step1.snapshot.uavs["u1"]
    assert u1_s1.position_xy == (5.0, 0.0)
    assert u1_s1.rth_state == RTHState.ACTIVE
    assert u1_s1.active is True
    assert not any(e.event_type == EventType.UAV_LANDED for e in step1.events)

    # Step 2: UAV flies from (5, 0) to (0, 0) -> arrives at GCS!
    # CompleteRTHCommand should be emitted and applied during this step
    step2 = runner.step()
    u1_s2 = step2.snapshot.uavs["u1"]
    assert u1_s2.position_xy == (0.0, 0.0)
    assert u1_s2.velocity_xy == (0.0, 0.0)
    assert u1_s2.rth_state == RTHState.COMPLETE
    assert u1_s2.active is False
    assert u1_s2.role == Role.IDLE
    assert u1_s2.target_position is None

    # Check UAV_LANDED event
    landed_events = [e for e in step2.events if e.event_type == EventType.UAV_LANDED]
    assert len(landed_events) == 1
    assert landed_events[0].entity_id == "u1"
    assert "final_energy" in landed_events[0].payload
    assert landed_events[0].payload["final_energy"] == u1_s2.battery_energy

    # Check CompleteRTHCommand was in applied_commands
    complete_cmds = [c for c in step2.applied_commands if isinstance(c, CompleteRTHCommand)]
    assert len(complete_cmds) == 1
    assert complete_cmds[0].uav_id == "u1"
    assert complete_cmds[0].source_tick == 1  # current_tick when step2 began

    # Step 3: Step simulation again -> Landed UAV must NOT move or be assigned
    battery_before_s3 = u1_s2.battery_energy
    step3 = runner.step()
    u1_s3 = step3.snapshot.uavs["u1"]
    assert u1_s3.position_xy == (0.0, 0.0)
    assert u1_s3.velocity_xy == (0.0, 0.0)
    assert u1_s3.active is False
    assert u1_s3.rth_state == RTHState.COMPLETE
    assert u1_s3.target_position is None
    assert u1_s3.assigned_task_id is None
    # No energy consumed while landed
    assert u1_s3.battery_energy == battery_before_s3
    # No additional commands applied for landed UAV
    assert not any(c.uav_id == "u1" for c in step3.applied_commands)
