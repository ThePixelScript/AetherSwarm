from __future__ import annotations

from ares_swarm.core.commands import (
    FailUAVCommand,
    ProgressTaskCommand,
    RecoverUAVCommand,
    StartRTHCommand,
    StepPhysicsCommand,
)
from ares_swarm.core.event_scheduler import (
    EventScheduler,
    ScheduledEventType,
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
        self.event_scheduler = EventScheduler()

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
    def process_scheduled_events(self):
        """Apply all scheduled failure/recovery events for the current tick."""

        snapshot = self.state_store.snapshot()
        due_events = self.event_scheduler.due_events(snapshot.simulation_tick)

        commands = []

        for event in due_events:
            if event.event_type == ScheduledEventType.UAV_FAILURE:
                commands.append(
                    FailUAVCommand(
                        source_tick=snapshot.simulation_tick,
                        uav_id=event.uav_id,
                        reason=event.reason,
                    )
                )

            elif event.event_type == ScheduledEventType.UAV_RECOVERY:
                commands.append(
                    RecoverUAVCommand(
                        source_tick=snapshot.simulation_tick,
                        uav_id=event.uav_id,
                    )
                )

        return self.state_store.apply(commands)
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
    def step_swarm(
        self,
        speed: float | None = None,
        separation_enforcer: Any = None,
        airspace: Any = None,
        flight_phases: Any = None,
    ):
        """Step all active UAVs with targets in one deterministic batch."""

        snapshot = self.state_store.snapshot()

        configured_speed = self.max_speed if speed is None else speed

        if configured_speed < 0:
            raise ValueError("speed must be non-negative")

        if configured_speed > self.max_speed:
            raise ValueError("speed exceeds configured maximum")

        if separation_enforcer is not None:
            from dataclasses import replace
            commands, events = separation_enforcer.enforce_step(
                snapshot=snapshot,
                configured_speed=configured_speed,
                dt=self.dt,
                idle_rate=self.idle_rate,
                movement_rate=self.movement_rate,
                airspace=airspace,
                flight_phases=flight_phases,
            )
            res = self.state_store.apply(commands)
            if events:
                return replace(res, emitted_events=res.emitted_events + tuple(events))
            return res

        commands = []

        for uav_id in sorted(snapshot.uavs):
            uav = snapshot.uavs[uav_id]

            if not uav.active:
                continue

            if uav.failure_state == FailureState.FAILED:
                continue

            if uav.target_position is None:
                energy_cost = calculate_energy_cost(
                    dt=self.dt,
                    distance=0.0,
                    idle_rate=self.idle_rate,
                    movement_rate=self.movement_rate,
                )
                commands.append(
                    StepPhysicsCommand(
                        source_tick=snapshot.simulation_tick,
                        uav_id=uav_id,
                        new_position_xy=uav.position_xy,
                        new_velocity_xy=(0.0, 0.0),
                        delta_energy=energy_cost,
                    )
                )
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
