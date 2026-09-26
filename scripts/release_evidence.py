"""Read-only extraction of executed mission evidence; no planner behavior."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True, allow_nan=False) + '\n', encoding='utf-8')


def digest(data):
    return hashlib.sha256(json.dumps(data, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def mission_evidence(result):
    """Include exact lifecycle evidence and hash every snapshot and Gamma result."""
    history_hash = hashlib.sha256()
    for step in result.step_history:
        net = step.network_analysis
        record = {
            'snapshot': step.snapshot.to_dict(),
            'routes': dict(net.routes_to_gcs),
            'route_pdr': dict(net.route_pdr_to_gcs),
            'connected': list(net.connected_uav_ids),
            'links': [asdict(link) for link in net.edge_metrics],
        }
        history_hash.update(json.dumps(record, sort_keys=True, separators=(',', ':'), allow_nan=False).encode())
        history_hash.update(b'\n')
    safety = result.safety_report
    return {
        'events': [asdict(event) for event in result.all_events],
        'history_sha256': history_hash.hexdigest(),
        'history_ticks': len(result.step_history),
        'flight_records': {uid: asdict(rec) for uid, rec in sorted(safety.uav_flight_records.items())},
        'safety': {
            'minimum_separation_m': safety.min_observed_separation_m,
            'separation_violations': safety.separation_violations_count,
            'geofence_violations': safety.geofence_violations_count,
            'landing_violations': safety.landing_violations_count,
            'flight_duration_violations': safety.flight_duration_violations_count,
            'battery_exhaustions': safety.battery_exhaustions_count,
            'max_airborne_s': safety.max_observed_sortie_duration_s,
            'landed_count': sum(u.rth_state.value == 'COMPLETE' and not u.active for u in result.final_snapshot.uavs.values()),
            'expected_uav_count': len(result.final_snapshot.uavs),
        },
    }
