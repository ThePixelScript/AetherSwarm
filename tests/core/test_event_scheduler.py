from ares_swarm.core.event_scheduler import (
    EventScheduler,
    ScheduledEvent,
    ScheduledEventType,
)


def test_scheduler_returns_events_in_deterministic_order():
    scheduler = EventScheduler()

    scheduler.schedule(
        ScheduledEvent(
            tick=5,
            event_type=ScheduledEventType.UAV_FAILURE,
            uav_id="u2",
        )
    )
    scheduler.schedule(
        ScheduledEvent(
            tick=5,
            event_type=ScheduledEventType.UAV_FAILURE,
            uav_id="u1",
        )
    )

    events = scheduler.due_events(5)

    assert [event.uav_id for event in events] == ["u1", "u2"]


def test_scheduler_only_returns_events_due_at_requested_tick():
    scheduler = EventScheduler()

    scheduler.schedule(
        ScheduledEvent(
            tick=3,
            event_type=ScheduledEventType.UAV_FAILURE,
            uav_id="u1",
        )
    )
    scheduler.schedule(
        ScheduledEvent(
            tick=5,
            event_type=ScheduledEventType.UAV_RECOVERY,
            uav_id="u1",
        )
    )

    events = scheduler.due_events(3)

    assert len(events) == 1
    assert events[0].uav_id == "u1"
    assert events[0].event_type == ScheduledEventType.UAV_FAILURE
    assert len(scheduler.pending_events()) == 1


def test_scheduler_rejects_negative_tick():
    scheduler = EventScheduler()

    try:
        scheduler.schedule(
            ScheduledEvent(
                tick=-1,
                event_type=ScheduledEventType.UAV_FAILURE,
                uav_id="u1",
            )
        )
        assert False
    except ValueError:
        assert True