import argparse
import json
from pathlib import Path

from echorescue.config import SimulationConfig
from echorescue.events import EventType
from echorescue.multi_simulation import MultiDroneSimulation, MultiSimulationResult


BENCHMARK_SCHEMA_VERSION = "1.0"
PROFILE_MATRIX = (
    ("visual_smoke_off", "visual", "off"),
    ("visual_smoke_moderate", "visual", "moderate"),
    ("thermal_smoke_off", "thermal", "off"),
    ("thermal_smoke_moderate", "thermal", "moderate"),
)


def _mean(values: list[float | int]) -> float | None:
    return round(sum(values) / len(values), 6) if values else None


def _profile(
    results: list[MultiSimulationResult],
    perception_metrics: list[dict[str, object]],
) -> dict[str, object]:
    detection_times = [
        result.time_to_first_detection
        for result in results
        if result.time_to_first_detection is not None
    ]
    confirmation_count = sum(
        event.event_type is EventType.SURVIVOR_CONFIRMED
        for result in results
        for event in result.mission_events
    )
    attempts = sum(
        int(metrics["detection_attempts"])
        for metrics in perception_metrics
    )
    successful = sum(
        int(metrics["successful_observations"])
        for metrics in perception_metrics
    )
    thermal = perception_metrics[0].get("sensor_channel") == "thermal"
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
        "average_drones_returned": _mean(
            [result.drones_returned for result in results]
        ),
        "average_explored_percent": _mean(
            [result.explored_percent for result in results]
        ),
        "successful_survivor_observations": successful,
        "failed_detection_attempts": attempts - successful,
        "smoke_degraded_detection_attempts": sum(
            int(metrics["degraded_detection_attempts"])
            for metrics in perception_metrics
        ),
        "smoke_degraded_detection_events": sum(
            int(metrics["degraded_detection_events"])
            for metrics in perception_metrics
        ),
        "confirmation_count": confirmation_count,
        "wall_collisions": sum(result.collisions for result in results),
        "drone_collisions": sum(
            result.drone_drone_collisions for result in results
        ),
        "thermal_attempts": attempts if thermal else 0,
        "thermal_successful_observations": successful if thermal else 0,
        "thermal_failed_observations": attempts - successful if thermal else 0,
    }


def _difference(
    candidate: dict[str, object], baseline: dict[str, object]
) -> dict[str, float | None]:
    def delta(key: str) -> float | None:
        first = candidate[key]
        second = baseline[key]
        if first is None or second is None:
            return None
        return round(float(first) - float(second), 6)

    return {
        "survivor_recall": delta("average_survivor_recall"),
        "mission_success_rate": delta("mission_success_rate"),
        "time_to_first_detection": delta("average_time_to_first_detection"),
        "mission_steps": delta("average_mission_steps"),
        "drones_returned": delta("average_drones_returned"),
        "explored_percent": delta("average_explored_percent"),
    }


def run_benchmark(*, seeds: int = 50) -> dict[str, object]:
    if seeds < 1:
        raise ValueError("seeds must be positive")
    results: dict[str, list[MultiSimulationResult]] = {
        name: [] for name, _, _ in PROFILE_MATRIX
    }
    metrics: dict[str, list[dict[str, object]]] = {
        name: [] for name, _, _ in PROFILE_MATRIX
    }
    deterministic = True
    for seed in range(seeds):
        for name, sensor, smoke in PROFILE_MATRIX:
            config = SimulationConfig(
                seed=seed,
                drone_count=2,
                survivor_sensor=sensor,
                smoke_profile=smoke,
            )
            first_simulation = MultiDroneSimulation(config)
            first = first_simulation.run()
            second = MultiDroneSimulation(config).run()
            deterministic = deterministic and first == second
            results[name].append(first)
            metrics[name].append(
                first_simulation.smoke_detection_metrics()
            )

    profiles = {
        name: _profile(results[name], metrics[name])
        for name, _, _ in PROFILE_MATRIX
    }
    visual_failures = [
        seed
        for seed, result in enumerate(results["visual_smoke_moderate"])
        if result.survivor_recall < 1.0
    ]
    rescued = [
        seed
        for seed in visual_failures
        if results["thermal_smoke_moderate"][seed].survivor_recall == 1.0
    ]
    still_incomplete = [
        seed
        for seed in visual_failures
        if results["thermal_smoke_moderate"][seed].survivor_recall < 1.0
    ]
    thermal_failures = [
        seed
        for seed, result in enumerate(results["thermal_smoke_moderate"])
        if result.survivor_recall < 1.0
    ]
    return {
        "schema_version": BENCHMARK_SCHEMA_VERSION,
        "benchmark_type": "thermal_perception",
        "research_question": (
            "How robust is an abstracted Thermal Survivor channel against "
            "smoke visibility degradation compared with the Visual channel?"
        ),
        "configuration": {
            "seeds": list(range(seeds)),
            "drone_count": 2,
            "knowledge_mode": "shared",
            "confirmation_observations": 2,
            "visual_range": 3,
            "visual_base_detection_probability": 1.0,
            "visual_smoke_attenuation": 1.0,
            "thermal_range": 3,
            "thermal_base_detection_probability": 0.6,
            "thermal_smoke_attenuation": 0.15,
            "all_other_parameters": "SimulationConfig defaults",
        },
        "profiles": profiles,
        "smoke_penalty_moderate_minus_off": {
            "visual": _difference(
                profiles["visual_smoke_moderate"],
                profiles["visual_smoke_off"],
            ),
            "thermal": _difference(
                profiles["thermal_smoke_moderate"],
                profiles["thermal_smoke_off"],
            ),
        },
        "thermal_moderate_minus_visual_moderate": _difference(
            profiles["thermal_smoke_moderate"],
            profiles["visual_smoke_moderate"],
        ),
        "visual_smoke_failure_seed_analysis": {
            "visual_failure_count": len(visual_failures),
            "visual_failure_seeds": visual_failures,
            "rescued_by_thermal_count": len(rescued),
            "rescued_by_thermal_seeds": rescued,
            "still_incomplete_with_thermal_count": len(still_incomplete),
            "still_incomplete_with_thermal_seeds": still_incomplete,
            "all_thermal_moderate_failure_count": len(thermal_failures),
            "all_thermal_moderate_failure_seeds": thermal_failures,
        },
        "acceptance": {
            "deterministic_repeats": deterministic,
            "all_profiles_collision_free": all(
                profile["wall_collisions"] == 0
                and profile["drone_collisions"] == 0
                for profile in profiles.values()
            ),
            "navigation_metrics_identical": len(
                {
                    (
                        profile["average_mission_steps"],
                        profile["average_drones_returned"],
                        profile["average_explored_percent"],
                    )
                    for profile in profiles.values()
                }
            )
            == 1,
            "thermal_smoke_penalty_below_visual": (
                abs(
                    float(
                        profiles["thermal_smoke_moderate"][
                            "average_survivor_recall"
                        ]
                    )
                    - float(
                        profiles["thermal_smoke_off"][
                            "average_survivor_recall"
                        ]
                    )
                )
                < abs(
                    float(
                        profiles["visual_smoke_moderate"][
                            "average_survivor_recall"
                        ]
                    )
                    - float(
                        profiles["visual_smoke_off"][
                            "average_survivor_recall"
                        ]
                    )
                )
            ),
            "thermal_is_not_trivially_perfect": bool(thermal_failures),
        },
    }


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Benchmark isolated Visual and Thermal Survivor channels."
    )
    parser.add_argument("--seeds", type=int, default=50)
    parser.add_argument(
        "--output", default="benchmarks/thermal_perception_50_seeds.json"
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
