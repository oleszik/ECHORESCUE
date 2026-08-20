import argparse
import json
from dataclasses import asdict
from pathlib import Path
from statistics import mean

from echorescue.config import SimulationConfig
from echorescue.multi_simulation import MultiDroneSimulation


COMMUNICATION_BENCHMARK_SCHEMA_VERSION = "1.0"
DRONE_IDS = ("drone-1", "drone-2")


def run_communication_benchmark(seed_count: int = 50) -> dict[str, object]:
    if seed_count < 1:
        raise ValueError("seed_count must be positive")
    results = []
    deterministic = True
    for seed in range(seed_count):
        config = SimulationConfig(seed=seed, drone_count=2)
        first = MultiDroneSimulation(config).run()
        second = MultiDroneSimulation(config).run()
        deterministic = deterministic and first == second
        results.append(first)

    base_config = SimulationConfig(seed=0, drone_count=2)
    configuration = asdict(base_config)
    configuration["seed"] = f"0..{seed_count - 1}"

    def average_metric(name: str, drone_id: str) -> float:
        return mean(getattr(result, name)[drone_id] for result in results)

    return {
        "schema_version": COMMUNICATION_BENCHMARK_SCHEMA_VERSION,
        "suite": {
            "seed_count": seed_count,
            "seeds": list(range(seed_count)),
            "configuration": configuration,
            "runs_per_seed": 2,
            "determinism_check": "passed" if deterministic else "failed",
        },
        "communication": {
            "average_uptime_by_drone": {
                drone_id: round(
                    average_metric("communication_uptime_by_drone", drone_id),
                    6,
                )
                for drone_id in DRONE_IDS
            },
            "average_direct_base_uptime_by_drone": {
                drone_id: round(
                    average_metric("direct_base_uptime_by_drone", drone_id),
                    6,
                )
                for drone_id in DRONE_IDS
            },
            "average_relay_uptime_by_drone": {
                drone_id: round(
                    average_metric("relay_uptime_by_drone", drone_id), 6
                )
                for drone_id in DRONE_IDS
            },
            "total_outages_by_drone": {
                drone_id: sum(
                    result.communication_outages_by_drone[drone_id]
                    for result in results
                )
                for drone_id in DRONE_IDS
            },
            "average_outages_per_mission_by_drone": {
                drone_id: round(
                    average_metric("communication_outages_by_drone", drone_id),
                    3,
                )
                for drone_id in DRONE_IDS
            },
            "longest_outage_by_drone": {
                drone_id: max(
                    result.longest_outage_by_drone[drone_id]
                    for result in results
                )
                for drone_id in DRONE_IDS
            },
            "missions_with_relay_by_drone": {
                drone_id: sum(
                    result.relay_uptime_by_drone[drone_id] > 0
                    for result in results
                )
                for drone_id in DRONE_IDS
            },
            "missions_with_any_relay": sum(
                any(result.relay_uptime_by_drone.values())
                for result in results
            ),
        },
        "mission_behavior": {
            "successful_missions": sum(
                result.mission_success for result in results
            ),
            "average_mission_steps": round(
                mean(result.steps for result in results), 3
            ),
            "average_survivor_recall": round(
                mean(result.survivor_recall for result in results), 6
            ),
            "wall_collisions": sum(result.collisions for result in results),
            "drone_drone_collisions": sum(
                result.drone_drone_collisions for result in results
            ),
            "timeouts": sum(
                result.termination_reason == "max_steps" for result in results
            ),
            "average_duplicate_exploration_ratio": round(
                mean(result.duplicate_exploration_ratio for result in results), 6
            ),
        },
    }


def benchmark_json_bytes(benchmark: dict[str, object]) -> bytes:
    return (
        json.dumps(benchmark, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
    ).encode("utf-8")


def write_benchmark(benchmark: dict[str, object], output: str | Path) -> Path:
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(benchmark_json_bytes(benchmark))
    return path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the deterministic EchoRescue communication benchmark."
    )
    parser.add_argument("--seeds", type=int, default=50)
    parser.add_argument(
        "--output", default="benchmarks/communication_50_seeds.json"
    )
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    output = write_benchmark(
        run_communication_benchmark(args.seeds), args.output
    )
    print(output)


if __name__ == "__main__":
    main()
