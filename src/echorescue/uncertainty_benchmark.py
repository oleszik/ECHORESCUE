"""Frozen paired v0.12 experiment; compact seed-level results, no trajectories."""
import argparse
import hashlib
import json
from dataclasses import replace
from pathlib import Path
from random import Random
from statistics import mean, stdev
from math import sqrt

from echorescue.config import SimulationConfig
from echorescue.knowledge import ProbabilisticKnowledgeMap
from echorescue.models import Position, CellState
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.probabilistic import UNCERTAINTY_PROFILES, ProbabilityConfig
from dataclasses import asdict

PROTOCOL = Path(__file__).resolve().parents[2] / "benchmarks/uncertainty_protocol.json"


def summary(values, binary=False):
    if not values:
        return {"n": 0, "mean": None, "sd": None, "ci95": None}
    n = len(values)
    center = mean(values)
    rng = Random(912)
    samples = sorted(mean(rng.choices(values, k=n)) for _ in range(2000))
    interval = [samples[49], samples[1949]]
    if binary:
        denominator = 1+1.96**2/n
        middle = (center+1.96**2/(2*n))/denominator
        radius = 1.96*sqrt(center*(1-center)/n+1.96**2/(4*n*n))/denominator
        interval = [middle-radius, middle+radius]
    return {"n": n, "mean": center, "sd": stdev(values) if n>1 else 0,
            "ci95": interval}


def run_one(seed, profile, variant, options, closure=False):
    sim = MultiDroneSimulation(SimulationConfig(seed=seed, uncertainty_profile=profile,
        planning_variant=variant, **options))
    closure_position = next((Position(x,y) for y in range(2,sim.config.height-2)
        for x in range(2,sim.config.width-2) if sim.world.is_free(Position(x,y))
        and Position(x,y) not in sim.world.survivors), None)
    previous = {}
    route_replans = 0
    from echorescue.replay import _remaining_path
    def capture(current):
        nonlocal route_replans
        if closure and closure_position is not None:
            if current.steps == 10 and closure_position not in {r.drone.position for r in current.runtimes.values()}:
                current.world.block_cell(closure_position)
            if current.steps == 30:
                current.world.unblock_cell(closure_position)
        for identifier, runtime in current.runtimes.items():
            route = _remaining_path(runtime)
            old = previous.get(identifier, ())
            if old and route and old[-1] == route[-1] and route != old and route != old[1:]:
                route_replans += 1
            previous[identifier] = route
    result = sim.run(on_frame=capture)
    knowledge = sim.base_knowledge_map if sim.knowledge_mode == "local" else sim.occupancy_map
    assert isinstance(knowledge, ProbabilisticKnowledgeMap)
    positions = [Position(x,y) for y in range(sim.config.height) for x in range(sim.config.width)]
    errors = {p: (knowledge.probability_at(p).probability-
                  int(sim.world.cell_at(p) is CellState.OCCUPIED))**2 for p in positions}
    observed = [errors[p] for p in positions if knowledge.probability_at(p).evidence_count]
    confirmed = sim._base_confirmed_survivors if sim.knowledge_mode == "local" else sim._confirmed_survivors
    true = len(confirmed & sim.world.survivors)
    false = len(confirmed - sim.world.survivors)
    recall = true/len(sim.world.survivors) if sim.world.survivors else 1.
    returned = result.drones_returned/sim.config.drone_count
    collisions = result.collisions+result.drone_drone_collisions
    return {"seed":seed,"profile":profile,"variant":variant,
        "success":int(recall==1 and returned==1 and collisions==0 and false==0),
        "safe_return":int(returned==1 and collisions==0),"recall":recall,
        "precision":true/(true+false) if true+false else None,
        "false_confirmations":false,"duration":result.steps,
        "timeout":int(result.termination_reason=="max_steps"),
        "collisions":collisions,"unsafe_attempts":sim._stale_path_safety_interventions,
        "safety_interventions":sim.safety_shield_interventions,
        "path_length":sum(result.path_length_by_drone.values()),
        "replans":route_replans,
        "brier_all":mean(errors.values()),"brier_observed":mean(observed) if observed else None,
        "coverage":len(observed)/len(positions),"termination":result.termination_reason}


def aggregate(rows, profiles):
    metrics = [key for key in rows[0] if key not in {"seed","profile","variant","termination"}]
    groups, paired = {}, {}
    for profile in profiles:
        groups[profile] = {}
        for variant in ("naive","uncertainty-aware"):
            group = [r for r in rows if r["profile"]==profile and r["variant"]==variant]
            groups[profile][variant] = {metric: summary([r[metric] for r in group if r[metric] is not None],
                metric in {"success","safe_return","timeout"}) for metric in metrics}
            groups[profile][variant]["successful_duration"] = summary([r["duration"] for r in group if r["success"]])
            groups[profile][variant]["failure_rate"] = summary([1-r["success"] for r in group],True)
            groups[profile][variant]["non_timeout_failure"] = summary([int(not r["success"] and not r["timeout"]) for r in group],True)
        baseline = {r["seed"]:r for r in rows if r["profile"]==profile and r["variant"]=="naive"}
        candidate = {r["seed"]:r for r in rows if r["profile"]==profile and r["variant"]=="uncertainty-aware"}
        paired[profile] = {metric: summary([candidate[seed][metric]-baseline[seed][metric]
            for seed in baseline if candidate[seed][metric] is not None and baseline[seed][metric] is not None]) for metric in metrics}
        paired[profile]["both_successful_duration"] = summary([candidate[seed]["duration"]-baseline[seed]["duration"]
            for seed in baseline if candidate[seed]["success"] and baseline[seed]["success"]])
    return {"groups":groups,"paired_candidate_minus_naive":paired}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--split", choices=["development","holdout"], default="development")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    raw = PROTOCOL.read_bytes()
    protocol = json.loads(raw)
    assert protocol["profiles"] == {k:asdict(v) for k,v in UNCERTAINTY_PROFILES.items()}
    assert protocol["probability_config"] == asdict(ProbabilityConfig())
    seeds = protocol[args.split+"_seeds"]
    rows = []
    for seed in seeds:
        for profile in protocol["profiles"]:
            for variant in protocol["planning_variants"]:
                rows.append(run_one(seed,profile,variant,protocol["primary"]))
        print(f"completed {args.split} seed {seed}", flush=True)
    result = {"experiment":"v0.12", "split":args.split,
        "protocol_sha256":hashlib.sha256(raw).hexdigest(),"rows":rows,
        **aggregate(rows,protocol["profiles"])}
    Path(args.output).write_text(json.dumps(result,indent=2,sort_keys=True)+"\n",encoding="utf-8")

if __name__ == "__main__":
    main()
