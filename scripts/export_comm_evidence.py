import csv
from pathlib import Path
from run_demo_experiments import run_experiment


def export_evidence():
    print("Running E2 A0...")
    res_a0 = run_experiment("E2", a1=False)
    print("Running E2 A1...")
    res_a1 = run_experiment("E2", a1=True)

    def write_csv(filename, res):
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["tick", "time_s", "connected_count", "route_pdr_avg", "disconnected_count"])
            for step in res.step_history:
                net = step.network_analysis
                connected = len(net.connected_uav_ids)
                disconnected = len(net.disconnected_uav_ids)
                pdrs = [p for p in net.route_pdr_to_gcs.values() if p is not None]
                avg_pdr = sum(pdrs) / len(pdrs) if pdrs else 0.0
                writer.writerow([step.tick, net.simulation_time, connected, f"{avg_pdr:.4f}", disconnected])
        print(f"Exported {filename}")

    write_csv("results/e2_a0_comm_evidence.csv", res_a0)
    write_csv("results/e2_a1_comm_evidence.csv", res_a1)

    print("\n[TOPOLOGY EVIDENCE]")

    def print_topology_at(time_s, label):
        step = next(s for s in res_a0.step_history if s.network_analysis.simulation_time >= time_s)
        net = step.network_analysis
        print(f"\n--- {label} (t={time_s}s) ---")
        print(f"Connected UAVs: {net.connected_uav_ids}")
        print(f"Disconnected UAVs: {net.disconnected_uav_ids}")
        print("Routes to GCS:")
        for u, r in net.routes_to_gcs.items():
            print(f"  {u}: {r} (PDR: {net.route_pdr_to_gcs.get(u)})")

    print_topology_at(10.0, "D. BEFORE IMPAIRMENT (Normal)")
    print_topology_at(200.0, "E. DURING IMPAIRMENT (Degradation on uav_1<->uav_2, uav_2<->uav_3)")
    print_topology_at(300.0, "E. DURING IMPAIRMENT (Full Outage)")
    print_topology_at(400.0, "F. AFTER RECOVERY (Normal)")


if __name__ == "__main__":
    Path("results").mkdir(exist_ok=True)
    export_evidence()
