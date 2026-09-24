import argparse
import json
import dataclasses
from pathlib import Path
import math

from ares_swarm.simulation.scenario import load_scenario, ScenarioConfig, ChallengeProfileConfig, ChallengeAirspaceConfig, DetectionPipelineConfig
from ares_swarm.communication.config import CommunicationConfig
from ares_swarm.simulation.runner import MissionRunner
from ares_swarm.autonomy.task_allocator import A0TaskAllocator
from ares_swarm.autonomy.a1_allocator import A1TaskAllocator
from ares_swarm.autonomy.a0_adapter import A0AutonomyAdapter
from ares_swarm.communication.analysis import BaselineCommunicationAnalyzer
from ares_swarm.communication.scenario import CommunicationCondition
from ares_swarm.core.enums import FailureState, Role, RTHState, TaskStatus

def generate_e2_yaml():
    tasks = [
        {
            'id': 'poi_01',
            'position': [160.0, 500.0],
            'priority': 1,
            'spawn_time': 260.0,
            'deadline_offset': 10.0,
            'service_duration': 60.0 
        }
    ]
    
    uavs = [
        {'id': 'uav_1', 'position': [80.0, 560.0], 'battery_capacity': 50000.0, 'battery_energy': 50000.0, 'role': 'IDLE'},
        {'id': 'uav_2', 'position': [80.0, 440.0], 'battery_capacity': 50000.0, 'battery_energy': 50000.0, 'role': 'IDLE'},
        {'id': 'uav_3', 'position': [140.0, 560.0], 'battery_capacity': 50000.0, 'battery_energy': 50000.0, 'role': 'IDLE'},
        {'id': 'uav_4', 'position': [140.0, 440.0], 'battery_capacity': 50000.0, 'battery_energy': 50000.0, 'role': 'IDLE'},
    ]
    
    scen = ScenarioConfig(
        name="e2_controlled",
        seed=2026,
        dt=1.0,
        speed_limit=5.0,
        duration=400.0,
        max_ticks=400,
        gcs_position=(0.0, 500.0),
        arena_bounds_x=(0.0, 1000.0),
        arena_bounds_y=(0.0, 1000.0),
        min_separation_m=20.0,
        enable_auto_rth=False,
        return_by_mission_end=False,
        communication=CommunicationConfig(
            max_range=100.0,
            base_latency=5.0,
        ),
        challenge_profile=ChallengeProfileConfig(
            enabled=True,
            enforce_geofence=True,
            enforce_separation=True,
            airspace=ChallengeAirspaceConfig(
                enabled=True,
                staging_pad_center=(0.0, 500.0),
                corridor_bounds_x=(0.0, 10.0),
                corridor_bounds_y=(450.0, 550.0)
            ),
            detection_pipeline=DetectionPipelineConfig(
                enabled=True,
                sensor_fov_radius_m=40.0,
                reporting_deadline_s=10.0
            )
        ),
        uavs=tuple(uavs),
        tasks=tuple(tasks)
    )
    import yaml
    out_path = Path('scenarios/e2_controlled.yaml')
    
    # We will use json.loads(json.dumps()) to strip dataclass types and turn tuples to lists
    scen_dict = dataclasses.asdict(scen)
    scen_dict = json.loads(json.dumps(scen_dict))
    
    with open(out_path, 'w') as f:
        yaml.dump(scen_dict, f, default_flow_style=False, sort_keys=False)
    print(f"Generated {out_path}")
    return out_path

def calculate_yaw(vx, vy, previous_yaw):
    speed = math.hypot(vx, vy)
    if speed > 0.05:
        return math.atan2(vy, vx)
    return previous_yaw

def generate_trace_dict(runner, scenario_name, experiment):
    trace = {
        "metadata": {
            "scenario_name": scenario_name,
            "experiment": experiment,
            "seed": runner.seed,
            "arena_bounds_x": runner.scenario.arena_bounds_x,
            "arena_bounds_y": runner.scenario.arena_bounds_y,
            "max_altitude": getattr(runner.scenario, "max_height", 100.0),
            "gcs_position": runner.scenario.gcs_position,
            "tick_duration": runner.scenario.dt,
            "total_ticks": len(runner.history)
        },
        "frames": []
    }
    
    previous_yaw = {u_id: 0.0 for u_id in runner.scenario.uavs} if isinstance(runner.scenario.uavs, dict) else {u['id']: 0.0 for u in runner.scenario.uavs}
    
    for step in runner.history:
        tick_time = step.simulation_time
        snap = step.snapshot
        net = step.network_analysis
        
        frame = {
            "tick": step.tick,
            "time_s": tick_time,
            "uavs": {},
            "tasks": {},
            "network": {
                "edges": [],
                "active_relays": [],
                "connected_to_gcs": []
            }
        }
        
        for u_id, u_state in snap.uavs.items():
            vx, vy = getattr(u_state, "velocity_xy", (0.0, 0.0))
            yaw = calculate_yaw(vx, vy, previous_yaw.get(u_id, 0.0))
            previous_yaw[u_id] = yaw
            
            frame["uavs"][u_id] = {
                "position": [u_state.position_xy[0], u_state.position_xy[1], getattr(u_state, "altitude", 30.0)],
                "yaw": yaw,
                "role": u_state.role.name if hasattr(u_state.role, "name") else str(u_state.role),
                "failure_state": u_state.failure_state.name if hasattr(u_state.failure_state, "name") else str(u_state.failure_state)
            }
        
        for t_id, t_state in snap.tasks.items():
            if t_state.status != TaskStatus.PENDING:
                frame["tasks"][t_id] = {
                    "position": [t_state.position_xy[0], t_state.position_xy[1], 0.0],
                    "status": t_state.status.name if hasattr(t_state.status, "name") else str(t_state.status),
                    "assigned_to": getattr(t_state, "assigned_uav_id", None)
                }
        
        for edge in net.edge_metrics:
            frame["network"]["edges"].append({
                "source": edge.source_id,
                "target": edge.target_id,
                "quality": edge.link_quality,
                "outage": getattr(edge, "outage", False)
            })
            
        frame["network"]["connected_to_gcs"] = list(net.connected_uav_ids)
        if hasattr(net, "active_relay_uavs"):
            frame["network"]["active_relays"] = list(net.active_relay_uavs)
        
        trace["frames"].append(frame)
        
    return trace

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--outdir", type=str, default="e2_output")
    args = parser.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    yaml_path = generate_e2_yaml()
    base_scenario = load_scenario(yaml_path)

    conds = [
        CommunicationCondition(source_id='uav_1', target_id='uav_3', start_time_s=150.0, end_time_s=250.0, quality_multiplier=0.2),
        CommunicationCondition(source_id='uav_1', target_id='uav_3', start_time_s=250.0, end_time_s=350.0, outage=True),
        CommunicationCondition(source_id='uav_2', target_id='uav_3', start_time_s=150.0, end_time_s=250.0, quality_multiplier=0.2),
        CommunicationCondition(source_id='uav_2', target_id='uav_3', start_time_s=250.0, end_time_s=350.0, outage=True),
        CommunicationCondition(source_id='uav_3', target_id='uav_4', start_time_s=0.0, end_time_s=400.0, outage=True),
    ]

    for allocator_type in ['a0', 'a1']:
        print(f"\nRunning {allocator_type.upper()}...")
        comm_analyzer = BaselineCommunicationAnalyzer(
            config=base_scenario.communication,
            scenario_conditions=conds
        )
        if allocator_type == 'a1':
            allocator = A1TaskAllocator(comm_analyzer=comm_analyzer)
        else:
            allocator = A0TaskAllocator()

        adapter = A0AutonomyAdapter(allocator=allocator)

        runner = MissionRunner(
            scenario=base_scenario,
            seed=2026,
            autonomy_adapter=adapter,
            comm_analyzer=comm_analyzer
        )

        res = runner.run()
        summary = res.to_dict()
        metrics = summary["metrics"]
        
        print(f"Tasks Completed: {metrics['tasks_completed']}/{metrics['tasks_total']}")

        out_res = outdir / f"{allocator_type}_e2.json"
        with open(out_res, "w") as f:
            json.dump(summary, f, indent=2)

        trace = generate_trace_dict(runner, "e2_controlled", "E2_CONTROLLED")
        out_trace = outdir / f"{allocator_type}_e2_trace.json"
        with open(out_trace, "w") as f:
            json.dump(trace, f, indent=2)
            
        print(f"Saved {out_res} and {out_trace}")
        
    print("\nE2_CONTROLLED Complete. Both traces ready for visualization.")

if __name__ == '__main__':
    main()
