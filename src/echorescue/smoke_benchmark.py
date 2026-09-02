import argparse
import json
from pathlib import Path

from echorescue.config import SimulationConfig
from echorescue.multi_simulation import MultiDroneSimulation, MultiSimulationResult


BENCHMARK_SCHEMA_VERSION = "1.0"


def _mean(values: list[float | int]) -> float | None:
    return round(sum(values) / len(values), 6) if values else None


def _profile(
    results: list[MultiSimulationResult],
    smoke_metrics: list[dict[str, object]],
) -> dict[str, object]:
    detection_times = [
        result.time_to_first_detection
        for result in results
        if result.time_to_first_detection is not None
    ]
    return {
        "missions": len(results),
        "mission_success_rate": _mean(
            [int(result.mission_success) for result in results]
        ),
        "average_survivor_recall": _mean(
            [result.survivor_recall for result in results]
        ),
        "average_time_to_first_detection": _mean(detection_times),
        "missions_with_detection": len(detection_times),
        "average_mission_steps": _mean(
            [result.steps for result in results]
        ),
        "average_explored_percent": _mean(
            [result.explored_percent for result in results]
        ),
        "average_drones_returned": _mean(
            [result.drones_returned for result in results]
        ),
        "wall_collisions": sum(result.collisions for result in results),
        "drone_collisions": sum(
            result.drone_drone_collisions for result in results
        ),
        "survivor_confirmation_attempts": sum(
            int(metrics["successful_observations"])
            for metrics in smoke_metrics
        ),
        "survivor_detection_attempts": sum(
            int(metrics["detection_attempts"])
            for metrics in smoke_metrics
        ),
        "degraded_detection_attempts": sum(
            int(metrics["degraded_detection_attempts"])
            for metrics in smoke_metrics
        ),
        "degraded_detection_events": sum(
            int(metrics["degraded_detection_events"])
            for metrics in smoke_metrics
        ),
    }


def run_benchmark(*, seeds: int = 50) -> dict[str, object]:
    if seeds < 1:
        raise ValueError("seeds must be positive")
    off_results: list[MultiSimulationResult] = []
    moderate_results: list[MultiSimulationResult] = []
    off_metrics: list[dict[str, object]] = []
    moderate_metrics: list[dict[str, object]] = []
    deterministic = True
    for seed in range(seeds):
        off_config = SimulationConfig(
            seed=seed, drone_count=2, smoke_profile="off"
        )
        moderate_config = SimulationConfig(
            seed=seed, drone_count=2, smoke_profile="moderate"
        )
        off_simulation = MultiDroneSimulation(off_config)
        off_results.append(off_simulation.run())
        off_metrics.append(off_simulation.smoke_detection_metrics())
        first_simulation = MultiDroneSimulation(moderate_config)
        first = first_simulation.run()
        second = MultiDroneSimulation(moderate_config).run()
        deterministic = deterministic and first == second
        moderate_results.append(first)
        moderate_metrics.append(first_simulation.smoke_detection_metrics())

    off_profile = _profile(off_results, off_metrics)
    moderate_profile = _profile(moderate_results, moderate_metrics)
    return {
        "schema_version": BENCHMARK_SCHEMA_VERSION,
        "benchmark_type": "smoke_perception",
        "research_question": (
            "How strongly does degraded visibility reduce survivor detection "
            "and mission performance for the existing strategy?"
        ),
        "configuration": {
            "seeds": list(range(seeds)),
            "drone_count": 2,
            "knowledge_mode": "shared",
            "off_profile": "off",
            "candidate_profile": "moderate",
            "all_other_parameters": "SimulationConfig defaults",
        },
        "smoke_off": off_profile,
        "smoke_moderate": moderate_profile,
        "difference_moderate_minus_off": {
            "survivor_recall": round(
                float(moderate_profile["average_survivor_recall"])
                - float(off_profile["average_survivor_recall"]),
                6,
            ),
            "time_to_first_detection": (
                round(
                    float(moderate_profile["average_time_to_first_detection"])
                    - float(off_profile["average_time_to_first_detection"]),
                    6,
                )
                if moderate_profile["average_time_to_first_detection"]
                is not None
                and off_profile["average_time_to_first_detection"] is not None
                else None
            ),
            "mission_steps": round(
                float(moderate_profile["average_mission_steps"])
                - float(off_profile["average_mission_steps"]),
                6,
            ),
            "explored_percent": round(
                float(moderate_profile["average_explored_percent"])
                - float(off_profile["average_explored_percent"]),
                6,
            ),
            "average_drones_returned": round(
                float(moderate_profile["average_drones_returned"])
                - float(off_profile["average_drones_returned"]),
                6,
            ),
        },
        "acceptance": {
            "deterministic_repeats": deterministic,
            "off_collision_free": (
                off_profile["wall_collisions"] == 0
                and off_profile["drone_collisions"] == 0
            ),
            "moderate_collision_free": (
                moderate_profile["wall_collisions"] == 0
                and moderate_profile["drone_collisions"] == 0
            ),
            "moderate_has_degraded_detections": (
                moderate_profile["degraded_detection_attempts"] > 0
            ),
            "off_has_no_degraded_detections": (
                off_profile["degraded_detection_attempts"] == 0
            ),
        },
    }


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Benchmark survivor perception with deterministic smoke."
    )
    parser.add_argument("--seeds", type=int, default=50)
    parser.add_argument(
        "--output", default="benchmarks/smoke_perception_50_seeds.json"
    )
    args = parser.parse_args(argv)
    payload = run_benchmark(seeds=args.seeds)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(output)


if __name__ == "__main__":
    main()
