import argparse
import json
from dataclasses import asdict
from pathlib import Path
from statistics import mean

from echorescue.config import SimulationConfig
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.simulation import Simulation


BENCHMARK_SCHEMA_VERSION = "1.0"


def run_benchmark(seed_count: int = 50) -> dict[str, object]:
    if seed_count < 1:
        raise ValueError("seed_count must be positive")
    multi_results = []
    single_results = []
    deterministic = True
    for seed in range(seed_count):
        config = SimulationConfig(seed=seed, drone_count=2)
        first = MultiDroneSimulation(config).run()
        second = MultiDroneSimulation(config).run()
        deterministic = deterministic and first == second
        multi_results.append(first)
        single_results.append(Simulation(SimulationConfig(seed=seed)).run())

    multi_duration = mean(result.steps for result in multi_results)
    single_duration = mean(result.steps for result in single_results)
    combined_path = mean(
        sum(result.path_length_by_drone.values()) for result in multi_results
    )
    single_path = mean(result.path_length for result in single_results)
    duration_reduction = (
        100.0 * (single_duration - multi_duration) / single_duration
    )
    path_increase = 100.0 * (combined_path - single_path) / single_path
    base_config = SimulationConfig(seed=0, drone_count=2)
    configuration = asdict(base_config)
    configuration["seed"] = f"0..{seed_count - 1}"
    safety_regression = (
        any(result.collisions for result in multi_results)
        or any(result.drone_drone_collisions for result in multi_results)
        or mean(result.survivor_recall for result in multi_results)
        < mean(result.survivor_recall for result in single_results)
    )
    return {
        "schema_version": BENCHMARK_SCHEMA_VERSION,
        "suite": {
            "seed_count": seed_count,
            "seeds": list(range(seed_count)),
            "configuration": configuration,
            "multi_drone_runs_per_seed": 2,
            "single_drone_runs_per_seed": 1,
            "determinism_check": "passed" if deterministic else "failed",
        },
        "single_drone": {
            "successful_missions": sum(
                result.mission_success for result in single_results
            ),
            "average_mission_steps": round(single_duration, 3),
            "mission_steps_range": [
                min(result.steps for result in single_results),
                max(result.steps for result in single_results),
            ],
            "average_survivor_recall": round(
                mean(result.survivor_recall for result in single_results), 6
            ),
            "wall_collisions": sum(result.collisions for result in single_results),
            "average_path_length": round(single_path, 3),
        },
        "two_drone": {
            "successful_missions": sum(
                result.mission_success for result in multi_results
            ),
            "both_drones_returned": sum(
                result.drones_returned == 2 for result in multi_results
            ),
            "average_mission_steps": round(multi_duration, 3),
            "mission_steps_range": [
                min(result.steps for result in multi_results),
                max(result.steps for result in multi_results),
            ],
            "average_survivor_recall": round(
                mean(result.survivor_recall for result in multi_results), 6
            ),
            "wall_collisions": sum(result.collisions for result in multi_results),
            "drone_drone_collisions": sum(
                result.drone_drone_collisions for result in multi_results
            ),
            "timeouts": sum(
                result.termination_reason == "max_steps"
                for result in multi_results
            ),
            "failed_missions": sum(
                not result.mission_success for result in multi_results
            ),
            "movement_conflicts": sum(
                result.movement_conflicts for result in multi_results
            ),
            "average_duplicate_exploration_ratio": round(
                mean(
                    result.duplicate_exploration_ratio
                    for result in multi_results
                ),
                6,
            ),
            "average_combined_path_length": round(combined_path, 3),
        },
        "comparison": {
            "mission_duration_reduction_percent": round(duration_reduction, 3),
            "combined_path_length_increase_percent": round(path_increase, 3),
            "safety_regression": safety_regression,
        },
    }


def benchmark_json_bytes(benchmark: dict[str, object]) -> bytes:
    return (
        json.dumps(
            benchmark,
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
        )
        + "\n"
    ).encode("utf-8")


def write_benchmark(benchmark: dict[str, object], output: str | Path) -> Path:
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(benchmark_json_bytes(benchmark))
    return path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run deterministic EchoRescue single/two-drone benchmarks."
    )
    parser.add_argument("--seeds", type=int, default=50)
    parser.add_argument(
        "--output", default="benchmarks/two_drone_50_seeds.json"
    )
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    benchmark = run_benchmark(args.seeds)
    output = write_benchmark(benchmark, args.output)
    print(output)


if __name__ == "__main__":
    main()
