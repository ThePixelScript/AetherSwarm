from __future__ import annotations

from ares_swarm.core.commands import (
    ProgressTaskCommand,
    StartRTHCommand,
    StepPhysicsCommand,
)
from ares_swarm.core.enums import FailureState, RTHState
from ares_swarm.core.kinematics import move_towards
from ares_swarm.core.state_store import StateStore
from ares_swarm.energy.battery import calculate_energy_cost


class SimulationEngine:
    """Deterministic simulation engine using StateStore as authority."""

    def __init__(
        self,
        state_store: StateStore,
        dt: float = 1.0,
        idle_rate: float = 1.0,
        movement_rate: float = 0.5,
    ):
        if dt <= 0:
            raise ValueError("dt must be greater than 0")

        self.state_store = state_store
        self.dt = dt
        self.idle_rate = idle_rate
        self.movement_rate = movement_rate
        self.max_speed = 5.0

    def advance_tick(self) -> float:
        """Advance the authoritative simulation clock by one tick."""

        snapshot = self.state_store.snapshot()

        next_tick = snapshot.simulation_tick + 1
        next_time = snapshot.simulation_time + self.dt

        self.state_store.set_simulation_clock(
            tick=next_tick,
            sim_time=next_time,
        )

        return next_time
    def start_rth(self, uav_id: str):
        """Request Return-to-Home through the authoritative StateStore."""

        snapshot = self.state_store.snapshot()

        command = StartRTHCommand(
            source_tick=snapshot.simulation_tick,
            uav_id=uav_id,
        )

        return self.state_store.apply([command])
    def check_battery_rth(self, rth_reserve: float = 0.0):
        """Trigger RTH for UAVs whose battery cannot safely cover the return trip."""

        if rth_reserve < 0:
            raise ValueError("rth_reserve must be non-negative")

        snapshot = self.state_store.snapshot()
        commands = []

        for uav_id in sorted(snapshot.uavs):
            uav = snapshot.uavs[uav_id]

            if not uav.active:
                continue

            if uav.failure_state == FailureState.FAILED:
                continue

            if uav.rth_state.value != "NONE":
                continue

            dx = snapshot.gcs_position[0] - uav.position_xy[0]
            dy = snapshot.gcs_position[1] - uav.position_xy[1]
            distance = (dx**2 + dy**2) ** 0.5

            return_energy = calculate_energy_cost(
                dt=self.dt,
                distance=distance,
                idle_rate=self.idle_rate,
                movement_rate=self.movement_rate,
            )

            required_energy = return_energy + rth_reserve

            if uav.battery_energy <= required_energy:
                commands.append(
                    StartRTHCommand(
                        source_tick=snapshot.simulation_tick,
                        uav_id=uav_id,
                    )
                )

        return self.state_store.apply(commands)
    def step_swarm(self, speed: float | None = None):
        """Step all active UAVs with targets in one deterministic batch."""

        snapshot = self.state_store.snapshot()

        configured_speed = self.max_speed if speed is None else speed

        if configured_speed < 0:
            raise ValueError("speed must be non-negative")

        if configured_speed > self.max_speed:
            raise ValueError("speed exceeds configured maximum")

        commands = []

        for uav_id in sorted(snapshot.uavs):
            uav = snapshot.uavs[uav_id]

            if not uav.active:
                continue

            if uav.failure_state == FailureState.FAILED:
                continue

            if uav.target_position is None:
                continue

            new_position, new_velocity = move_towards(
                current=uav.position_xy,
                target=uav.target_position,
                speed=configured_speed,
                dt=self.dt,
            )

            dx = new_position[0] - uav.position_xy[0]
            dy = new_position[1] - uav.position_xy[1]
            distance = (dx**2 + dy**2) ** 0.5

            energy_cost = calculate_energy_cost(
                dt=self.dt,
                distance=distance,
                idle_rate=self.idle_rate,
                movement_rate=self.movement_rate,
            )

            commands.append(
                StepPhysicsCommand(
                    source_tick=snapshot.simulation_tick,
                    uav_id=uav_id,
                    new_position_xy=new_position,
                    new_velocity_xy=new_velocity,
                    delta_energy=energy_cost,
                )
            )

        return self.state_store.apply(commands)
    def progress_arrived_tasks(self):
        """Progress tasks for UAVs that have reached their assigned PoI."""

        snapshot = self.state_store.snapshot()
        commands = []

        for uav_id in sorted(snapshot.uavs):
            uav = snapshot.uavs[uav_id]

            if not uav.active:
                continue

            if uav.failure_state == FailureState.FAILED:
                continue

            if uav.assigned_task_id is None:
                continue

            task = snapshot.tasks.get(uav.assigned_task_id)

            if task is None:
                continue

            if task.assigned_uav_id != uav_id:
                continue

            if uav.position_xy != task.position_xy:
                continue

            commands.append(
                ProgressTaskCommand(
                    source_tick=snapshot.simulation_tick,
                    uav_id=uav_id,
                    task_id=task.id,
                    delta_progress=self.dt,
                )
            )

        return self.state_store.apply(commands)

    def step_uav(
        self,
        uav_id: str,
        target: tuple[float, float],
        speed: float,
    ):
        """Calculate and apply one deterministic UAV physics step."""

        snapshot = self.state_store.snapshot()
        uav = snapshot.get_uav(uav_id)

        if uav is None:
            raise ValueError(f"UAV {uav_id} not found")

        new_position, new_velocity = move_towards(
            current=uav.position_xy,
            target=target,
            speed=speed,
            dt=self.dt,
        )

        dx = new_position[0] - uav.position_xy[0]
        dy = new_position[1] - uav.position_xy[1]
        distance = (dx**2 + dy**2) ** 0.5

        energy_cost = calculate_energy_cost(
            dt=self.dt,
            distance=distance,
            idle_rate=self.idle_rate,
            movement_rate=self.movement_rate,
        )

        command = StepPhysicsCommand(
            source_tick=snapshot.simulation_tick,
            uav_id=uav_id,
            new_position_xy=new_position,
            new_velocity_xy=new_velocity,
            delta_energy=energy_cost,
        )

        return self.state_store.apply([command])
def test_check_battery_rth_triggers_for_low_battery():
    uav = UAVState(
        id="u1",
        position_xy=(10.0, 0.0),
        battery_energy=5.0,
        target_position=(20.0, 0.0),
    )

    store = StateStore(
        StateSnapshot(
            simulation_tick=0,
            simulation_time=0.0,
            state_version=0,
            uavs={"u1": uav},
            gcs_position=(0.0, 0.0),
        )
    )

    engine = SimulationEngine(store)

    result = engine.check_battery_rth(rth_reserve=1.0)

    assert len(result.applied_commands) == 1
    assert store.snapshot().uavs["u1"].rth_state == RTHState.ACTIVE
    assert store.snapshot().uavs["u1"].target_position == (0.0, 0.0)


def test_check_battery_rth_does_not_trigger_with_sufficient_battery():
    uav = UAVState(
        id="u1",
        position_xy=(10.0, 0.0),
        battery_energy=100.0,
        target_position=(20.0, 0.0),
    )

    store = StateStore(
        StateSnapshot(
            simulation_tick=0,
            simulation_time=0.0,
            state_version=0,
            uavs={"u1": uav},
            gcs_position=(0.0, 0.0),
        )
    )

    engine = SimulationEngine(store)

    result = engine.check_battery_rth(rth_reserve=1.0)

    assert len(result.applied_commands) == 0
    assert store.snapshot().uavs["u1"].rth_state == RTHState.NONE