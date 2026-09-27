"""Exercise actual runner wiring, discovery, projection, reporting and landing."""
from scripts.run_atomic_expedition_demo import run, verify_demo


def test_eight_uav_expedition_demo(tmp_path):
    metrics = run(tmp_path)
    verify_demo(metrics)
    assert metrics['uav_count'] == 8
    assert metrics['known_pois'] == 5
    assert 5 <= metrics['hidden_pois'] <= 7
    assert metrics['gamma_checks'] > 0  # Guards against competing relay managers.
    assert any(len(team['relays']) >= 2 for team in metrics['teams'])
