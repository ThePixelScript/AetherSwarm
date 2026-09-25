import argparse
import json
from pathlib import Path
import dataclasses

from ares_swarm.simulation.scenario import load_scenario
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.autonomy.ingress_coordinator import IngressCoordinator
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer


def main():
    parser = argparse.ArgumentParser(description="AetherSwarm IIT Bombay Pushpak Stage-1 Submission Runner")
    parser.add_argument("--scenario", type=str, default="scenarios/poc_round1.yaml")
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--outdir", type=str, default="stage1_output")
    args = parser.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("AETHERSWARM STAGE-1 SUBMISSION RUNNER")
    print("=" * 60)
    print(f"Scenario: {args.scenario}")
    print(f"Seed: {args.seed}")

    base_scenario = load_scenario(Path(args.scenario))

    dp = dataclasses.replace(base_scenario.challenge_profile.detection_pipeline, enabled=True)
    airspace = dataclasses.replace(base_scenario.challenge_profile.airspace, enabled=True)
    cp = dataclasses.replace(
        base_scenario.challenge_profile,
        enabled=True,
        enforce_separation=True,
        enforce_sortie_limit=True,
        enforce_single_sortie=True,
        enforce_geofence=True,
        detection_pipeline=dp,
        airspace=airspace,
    )

    base_scenario = dataclasses.replace(base_scenario, challenge_profile=cp)

    # 2. Setup A1 Autonomy
    comm_analyzer = BaselineCommunicationAnalyzer(config=base_scenario.communication)
    allocator = A1TaskAllocator(comm_analyzer=comm_analyzer)
    ingress_coordinator = IngressCoordinator(
        comm_analyzer=comm_analyzer,
        gcs_position=base_scenario.gcs_position,
        d_safe=85.0,
    )
    adapter = A0AutonomyAdapter(allocator=allocator, ingress_coordinator=ingress_coordinator)

    # 3. Setup Runner
    runner = MissionRunner(
        scenario=base_scenario,
        seed=args.seed,
        autonomy_adapter=adapter,
        comm_analyzer=comm_analyzer,
    )

    print("\nExecuting Mission...")
    res = runner.run()

    print("\n" + "=" * 60)
    print("STAGE-1 COMPLIANCE METRICS")
    print("=" * 60)

    summary = res.to_dict()
    metrics = summary["metrics"]

    print(f"Mission Completion Time: {res.simulation_time} s (Max 2700s)")
    print(f"Tasks Completed: {metrics['tasks_completed']}/{metrics['tasks_total']}")

    s_rep = res.safety_report
    print("\nSafety & Constraints:")
    print(f"Min Separation: {s_rep.min_observed_separation_m:.2f} m (Limit >= 20.0m)")
    print(f"Geofence Violations: {s_rep.geofence_violations_count}")
    print(f"Flight Duration Violations: {s_rep.flight_duration_violations_count}")
    print(f"Landing Violations: {s_rep.landing_violations_count}")

    print("\nTelemetry & Reporting:")
    if res.telemetry_manager:
        tel = res.telemetry_manager.get_metrics()
        print(f"Detections: {tel['total_detections']}")
        print(f"Reports Delivered: {tel['reports_delivered']} (On Time)")
        print(f"Reports Deadline Exceeded: {tel['reports_deadline_exceeded']}")
        print(f"Compliance Ratio: {tel['reporting_compliance_ratio']:.2f}")

    print("\nUAV Flight Logs:")
    for u_id, rec in s_rep.uav_flight_records.items():
        print(f"  {u_id}: Airborne: {rec.cumulative_airborne_s:.1f}s | Landed at: {rec.landing_time}s")

    print("\nExporting Evidence...")
    res.save_json(outdir / "stage1_submission_report.json")
    print(f"Evidence saved to {outdir.absolute()}")
    print("=" * 60)


if __name__ == "__main__":
    main()
