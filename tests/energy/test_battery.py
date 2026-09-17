from ares_swarm.energy.battery import calculate_energy_cost


def test_energy_cost_from_idle_and_movement():
    cost = calculate_energy_cost(
        dt=2.0,
        distance=10.0,
        idle_rate=1.0,
        movement_rate=0.5,
    )

    assert cost == 7.0


def test_zero_distance_uses_only_idle_energy():
    cost = calculate_energy_cost(
        dt=3.0,
        distance=0.0,
        idle_rate=2.0,
        movement_rate=0.5,
    )

    assert cost == 6.0


def test_rejects_invalid_values():
    try:
        calculate_energy_cost(
            dt=0,
            distance=10,
            idle_rate=1,
            movement_rate=1,
        )
        assert False
    except ValueError:
        assert True