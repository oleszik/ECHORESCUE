"""Predeclared development stress scenarios; not an additional holdout."""
import argparse
import json
from pathlib import Path
from echorescue.uncertainty_benchmark import run_one, PROTOCOL
from echorescue.multi_floor import MultiFloorConfig, MultiFloorSimulation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",required=True)
    args = parser.parse_args()
    protocol=json.loads(PROTOCOL.read_text())
    rows=[]
    for seed in protocol["targeted_seeds"]:
        for variant in protocol["planning_variants"]:
            options=protocol["primary"]
            row=run_one(seed,"medium_noise",variant,options,closure=True)
            rows.append(dict(row,scenario="closure_10_reopening_30"))
            for latency in (1,8):
                local=dict(options,knowledge_mode="local",network_profile="constrained",
                    network_latency_steps=latency,network_packet_loss_rate=0.,
                    final_sync_max_steps=16,max_steps=80)
                row=run_one(seed,"medium_noise",variant,local)
                rows.append(dict(row,scenario=f"stale_transport_latency_{latency}"))
            sim=MultiFloorSimulation(MultiFloorConfig(seed=seed,uncertainty_profile="medium_noise",
                planning_variant=variant,max_steps=200))
            result=sim.run()
            true=len(sim.confirmed_survivors & sim.environment.survivors)
            false=len(sim.confirmed_survivors - sim.environment.survivors)
            rows.append({"seed":seed,"variant":variant,"scenario":"three_floors",
                "success":result.success,"recall":true/len(sim.environment.survivors),
                "precision":true/(true+false) if true+false else None,
                "false_confirmations":false,"duration":result.steps,
                "returned":result.returned_agents,"collisions":result.wall_collisions+result.drone_collisions,
                "safety_interventions":sim.safety_interventions,"path_length":result.total_path_length})
            print(f"completed targeted {seed} {variant}",flush=True)
    Path(args.output).write_text(json.dumps({"rows":rows},indent=2,sort_keys=True)+"\n")

if __name__ == "__main__": main()
