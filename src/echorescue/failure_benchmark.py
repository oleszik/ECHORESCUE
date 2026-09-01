import argparse
import json
from pathlib import Path

from echorescue.config import SimulationConfig
from echorescue.multi_simulation import MultiDroneSimulation, MultiSimulationResult


BENCHMARK_SCHEMA_VERSION = "1.0"
BENCHMARK_BATTERY_CAPACITY = 220.0


def _mean(values: list[float | int]) -> float:
    return round(sum(values) / len(values), 6) if values else 0.0


def _profile(results: list[MultiSimulationResult]) -> dict[str, object]:
    return {
        "missions": len(results),
        "mission_success_rate": _mean(
            [int(result.mission_success) for result in results]
        ),
        "average_survivor_recall": _mean(
            [result.survivor_recall for result in results]
        ),
        "average_mission_steps": _mean(
            [result.steps for result in results]
        ),
        "average_drones_returned": _mean(
            [result.drones_returned for result in results]
        ),
        "average_drones_failed": _mean(
            [result.drones_failed for result in results]
        ),
        "wall_collisions": sum(result.collisions for result in results),
        "drone_collisions": sum(
            result.drone_drone_collisions for result in results
        ),
        "average_path_length": _mean(
            [
                sum(result.path_length_by_drone.values())
                for result in results
            ]
        ),
        "average_energy_consumed": _mean(
            [
                sum(
                    BENCHMARK_BATTERY_CAPACITY - remaining
                    for remaining in result.energy_remaining_by_drone.values()
                )
                for result in results
            ]
        ),
    }


def run_benchmark(
    *,
    seeds: int = 50,
    failure_drone_id: str = "drone-2",
    failure_step: int = 4,
) -> dict[str, object]:
    if seeds < 1:
        raise ValueError("seeds must be positive")
    baseline_results: list[MultiSimulationResult] = []
    recovery_results: list[MultiSimulationResult] = []
    deterministic = True
    for seed in range(seeds):
        baseline_results.append(
            MultiDroneSimulation(
                SimulationConfig(seed=seed, drone_count=2)
            ).run()
        )
        config = SimulationConfig(
            seed=seed,
            drone_count=2,
            failure_schedule=((failure_drone_id, failure_step),),
        )
        first = MultiDroneSimulation(config).run()
        second = MultiDroneSimulation(config).run()
        deterministic = deterministic and first == second
        recovery_results.append(first)

    recovery_profile = _profile(recovery_results)
    failure_metrics = [
        result.failure_recovery_metrics for result in recovery_results
    ]
    assert all(metrics is not None for metrics in failure_metrics)
    typed_metrics = [metrics for metrics in failure_metrics if metrics is not None]
    acceptance = {
        "deterministic_repeats": deterministic,
        "all_failures_triggered": all(
            metrics["failures_triggered"] == 1 for metrics in typed_metrics
        ),
        "all_released_tasks_reassigned": all(
            metrics["tasks_pending"] == 0 for metrics in typed_metrics
        ),
        "all_reachable_survivors_confirmed": all(
            result.survivor_recall == 1.0 for result in recovery_results
        ),
        "all_operational_drones_returned": all(
            metrics["operational_drones_returned"]
            == metrics["operational_drones"]
            for metrics in typed_metrics
        ),
        "collision_free": recovery_profile["wall_collisions"] == 0
        and recovery_profile["drone_collisions"] == 0,
        "all_recovery_missions_successful": all(
            result.mission_success for result in recovery_results
        ),
    }
    return {
        "schema_version": BENCHMARK_SCHEMA_VERSION,
        "benchmark_type": "failure_reassignment",
        "configuration": {
            "seeds": list(range(seeds)),
            "failure_drone_id": failure_drone_id,
            "failure_step": failure_step,
            "knowledge_mode": "shared",
            "battery_capacity": BENCHMARK_BATTERY_CAPACITY,
        },
        "baseline": _profile(baseline_results),
        "failure_recovery": {
            **recovery_profile,
            "failures_triggered": sum(
                int(metrics["failures_triggered"])
                for metrics in typed_metrics
            ),
            "tasks_released": sum(
                int(metrics["tasks_released"])
                for metrics in typed_metrics
            ),
            "tasks_reassigned": sum(
                int(metrics["tasks_reassigned"])
                for metrics in typed_metrics
            ),
            "tasks_pending": sum(
                int(metrics["tasks_pending"])
                for metrics in typed_metrics
            ),
            "failed_drone_collision_avoidances": sum(
                int(metrics["failed_drone_collision_avoidances"])
                for metrics in typed_metrics
            ),
        },
        "acceptance": acceptance,
    }


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Benchmark deterministic task reassignment after a drone failure."
    )
    parser.add_argument("--seeds", type=int, default=50)
    parser.add_argument(
        "--failure-drone", choices=("drone-1", "drone-2"), default="drone-2"
    )
    parser.add_argument("--failure-step", type=int, default=4)
    parser.add_argument(
        "--output", default="benchmarks/failure_reassignment_50_seeds.json"
    )
    args = parser.parse_args(argv)
    payload = run_benchmark(
        seeds=args.seeds,
        failure_drone_id=args.failure_drone,
        failure_step=args.failure_step,
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(output)


if __name__ == "__main__":
    main()
