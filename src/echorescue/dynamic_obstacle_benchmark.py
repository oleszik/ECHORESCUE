import argparse
import json
from pathlib import Path
from statistics import mean, median, pstdev
from typing import Callable

from echorescue.config import SimulationConfig
from echorescue.multi_simulation import MultiDroneSimulation, MultiSimulationResult


BENCHMARK_SCHEMA_VERSION = "1.0"
DEVELOPMENT_SEEDS = tuple(range(10))
HOLDOUT_SEEDS = tuple(range(50, 100))
FOUR_AGENT_SEEDS = tuple(range(10))


def distribution(values: list[float | int]) -> dict[str, float | int | None]:
    return {
        "count": len(values),
        "mean": round(mean(values), 6) if values else None,
        "population_standard_deviation": (
            round(pstdev(values), 6) if values else None
        ),
        "median": round(median(values), 6) if values else None,
        "min": min(values) if values else None,
        "max": max(values) if values else None,
    }


def _path_length(result: MultiSimulationResult) -> int:
    return sum(result.path_length_by_drone.values())


def _dynamic_metrics(result: MultiSimulationResult) -> dict[str, object]:
    metrics = result.dynamic_obstacle_metrics
    if metrics is None:
        raise AssertionError("dynamic benchmark result has no dynamic metrics")
    return metrics


def _safety_check(result: MultiSimulationResult, label: str) -> None:
    if result.collisions or result.drone_drone_collisions:
        raise RuntimeError(
            f"safety failure in {label}: wall={result.collisions}, "
            f"drone={result.drone_drone_collisions}"
        )


def _profile(results: list[MultiSimulationResult]) -> dict[str, object]:
    dynamic_metrics = [
        result.dynamic_obstacle_metrics
        for result in results
        if result.dynamic_obstacle_metrics is not None
    ]
    profile: dict[str, object] = {
        "missions": len(results),
        "mission_success": {
            "count": sum(result.mission_success for result in results),
            "percentage": round(
                100 * sum(result.mission_success for result in results)
                / len(results),
                2,
            ),
        },
        "survivor_recall": distribution(
            [result.survivor_recall for result in results]
        ),
        "mission_duration": distribution([result.steps for result in results]),
        "total_path_length": distribution(
            [_path_length(result) for result in results]
        ),
        "explored_percent": distribution(
            [result.explored_percent for result in results]
        ),
        "wall_collisions": sum(result.collisions for result in results),
        "drone_collisions": sum(
            result.drone_drone_collisions for result in results
        ),
        "returned_agents": sum(result.drones_returned for result in results),
        "configured_agents": sum(result.drones_total for result in results),
    }
    if dynamic_metrics:
        for key in (
            "dynamic_obstacles_injected",
            "dynamic_obstacles_observed",
            "path_invalidations",
            "replans_total",
            "successful_replans",
            "failed_replans",
            "target_invalidations",
            "target_reassignments",
            "rtb_replans",
            "stale_path_safety_interventions",
            "additional_path_length_due_to_replanning",
        ):
            profile[key] = distribution(
                [int(metrics[key]) for metrics in dynamic_metrics]
            )
    return profile


def _paired_row(
    baseline: MultiSimulationResult, dynamic: MultiSimulationResult
) -> dict[str, object]:
    metrics = _dynamic_metrics(dynamic)
    baseline_path = _path_length(baseline)
    dynamic_path = _path_length(dynamic)
    duration_delta = dynamic.steps - baseline.steps
    path_delta = dynamic_path - baseline_path
    return {
        "seed": baseline.seed,
        "baseline_duration": baseline.steps,
        "dynamic_duration": dynamic.steps,
        "duration_delta": duration_delta,
        "additional_mission_duration_due_to_obstacles": duration_delta,
        "duration_overhead_percent": (
            round(100 * duration_delta / baseline.steps, 6)
            if baseline.steps
            else None
        ),
        "baseline_path_length": baseline_path,
        "dynamic_path_length": dynamic_path,
        "path_length_delta": path_delta,
        "additional_path_length_due_to_obstacles": path_delta,
        "path_overhead_percent": (
            round(100 * path_delta / baseline_path, 6)
            if baseline_path
            else None
        ),
        "replans": int(metrics["replans_total"]),
        "successful_replans": int(metrics["successful_replans"]),
        "failed_replans": int(metrics["failed_replans"]),
        "path_invalidations": int(metrics["path_invalidations"]),
        "dynamic_obstacles_injected": int(
            metrics["dynamic_obstacles_injected"]
        ),
        "dynamic_obstacles_observed": int(
            metrics["dynamic_obstacles_observed"]
        ),
        "baseline_survivor_recall": baseline.survivor_recall,
        "dynamic_survivor_recall": dynamic.survivor_recall,
        "survivor_recall_delta": round(
            dynamic.survivor_recall - baseline.survivor_recall, 6
        ),
        "baseline_mission_success": baseline.mission_success,
        "dynamic_mission_success": dynamic.mission_success,
        "mission_success_changed": (
            baseline.mission_success != dynamic.mission_success
        ),
    }


def _seed_list(
    rows: list[dict[str, object]],
    predicate: Callable[[dict[str, object]], bool],
) -> list[int]:
    return [
        int(row["seed"])
        for row in rows
        if predicate(row)
    ]


def _failure_analysis(rows: list[dict[str, object]]) -> dict[str, object]:
    by_duration = sorted(
        rows, key=lambda row: (-int(row["duration_delta"]), int(row["seed"]))
    )
    by_path = sorted(
        rows,
        key=lambda row: (-int(row["path_length_delta"]), int(row["seed"])),
    )
    return {
        "largest_duration_overhead": [
            {"seed": row["seed"], "delta": row["duration_delta"]}
            for row in by_duration[:5]
        ],
        "largest_path_length_overhead": [
            {"seed": row["seed"], "delta": row["path_length_delta"]}
            for row in by_path[:5]
        ],
        "multiple_replan_seeds": _seed_list(
            rows, lambda row: int(row["replans"]) > 1
        ),
        "replan_failure_seeds": _seed_list(
            rows, lambda row: int(row["failed_replans"]) > 0
        ),
        "survivor_recall_changed_seeds": _seed_list(
            rows, lambda row: float(row["survivor_recall_delta"]) != 0.0
        ),
        "mission_success_changed_seeds": _seed_list(
            rows, lambda row: bool(row["mission_success_changed"])
        ),
        "dynamic_faster_seeds": _seed_list(
            rows, lambda row: int(row["duration_delta"]) < 0
        ),
        "obstacle_never_observed_seeds": _seed_list(
            rows, lambda row: int(row["dynamic_obstacles_observed"]) == 0
        ),
        "obstacle_not_path_relevant_seeds": _seed_list(
            rows, lambda row: int(row["path_invalidations"]) == 0
        ),
    }


def run_benchmark(
    *,
    seeds: tuple[int, ...] = HOLDOUT_SEEDS,
    repeat: bool = True,
    four_agent_seeds: tuple[int, ...] = FOUR_AGENT_SEEDS,
) -> dict[str, object]:
    if not seeds:
        raise ValueError("seeds must not be empty")
    baseline_results: list[MultiSimulationResult] = []
    dynamic_results: list[MultiSimulationResult] = []
    deterministic = True
    for seed in seeds:
        baseline = MultiDroneSimulation(
            SimulationConfig(seed=seed, drone_count=2)
        ).run()
        dynamic_config = SimulationConfig(
            seed=seed, drone_count=2, dynamic_obstacles="moderate"
        )
        dynamic = MultiDroneSimulation(dynamic_config).run()
        _safety_check(baseline, f"baseline seed {seed}")
        _safety_check(dynamic, f"dynamic seed {seed}")
        if repeat:
            repeated = MultiDroneSimulation(dynamic_config).run()
            _safety_check(repeated, f"dynamic repeat seed {seed}")
            deterministic = deterministic and dynamic == repeated
        baseline_results.append(baseline)
        dynamic_results.append(dynamic)

    four_agent_results = []
    for seed in four_agent_seeds:
        result = MultiDroneSimulation(
            SimulationConfig(
                seed=seed, drone_count=4, dynamic_obstacles="moderate"
            )
        ).run()
        _safety_check(result, f"four-agent seed {seed}")
        four_agent_results.append(result)

    rows = [
        _paired_row(baseline, dynamic)
        for baseline, dynamic in zip(baseline_results, dynamic_results)
    ]
    duration_deltas = [int(row["duration_delta"]) for row in rows]
    path_deltas = [int(row["path_length_delta"]) for row in rows]
    demo_candidates = [
        int(row["seed"])
        for row in rows
        if bool(row["dynamic_mission_success"])
        and int(row["dynamic_obstacles_observed"]) > 0
        and int(row["path_invalidations"]) > 0
        and int(row["successful_replans"]) > 0
    ]
    all_results = baseline_results + dynamic_results + four_agent_results
    return {
        "schema_version": BENCHMARK_SCHEMA_VERSION,
        "benchmark_type": "dynamic_obstacles_paired_holdout",
        "configuration": {
            "development_seeds": list(DEVELOPMENT_SEEDS),
            "holdout_seeds": list(seeds),
            "four_agent_regression_seeds": list(four_agent_seeds),
            "primary_drone_count": 2,
            "sensor": "visual",
            "perception_noise": "off",
            "smoke_profile": "off",
            "relay_strategy": "off",
            "network_profile": "ideal",
            "knowledge_mode": "shared",
            "baseline_dynamic_obstacles": "off",
            "comparison_dynamic_obstacles": "moderate",
        },
        "frozen_profile": {
            "events_per_mission": 2,
            "event_steps": [12, 30],
            "placement": (
                "seeded static-map centrality ranking; excludes base, survivors, "
                "starts, nearby pairs, and connectivity-breaking cells"
            ),
            "adaptive_path_access": False,
            "persistence": "FREE to OCCUPIED for mission remainder",
        },
        "baseline": _profile(baseline_results),
        "dynamic": _profile(dynamic_results),
        "paired_overhead": {
            "mission_duration": distribution(duration_deltas),
            "total_path_length": distribution(path_deltas),
            "relative_duration_percent": distribution(
                [
                    float(row["duration_overhead_percent"])
                    for row in rows
                    if row["duration_overhead_percent"] is not None
                ]
            ),
            "relative_path_length_percent": distribution(
                [
                    float(row["path_overhead_percent"])
                    for row in rows
                    if row["path_overhead_percent"] is not None
                ]
            ),
        },
        "per_seed": rows,
        "failure_analysis": _failure_analysis(rows),
        "four_agent_regression": _profile(four_agent_results),
        "representative_demo_seed": demo_candidates[0] if demo_candidates else None,
        "acceptance": {
            "deterministic_dynamic_repeats": deterministic,
            "collision_free": all(
                result.collisions == 0 and result.drone_drone_collisions == 0
                for result in all_results
            ),
            "all_agents_returned": all(
                result.drones_returned == result.drones_total
                for result in all_results
            ),
            "all_obstacles_injected": all(
                int(_dynamic_metrics(result)["dynamic_obstacles_injected"]) == 2
                for result in dynamic_results + four_agent_results
            ),
        },
    }


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Run the paired v0.8 dynamic-obstacle holdout benchmark."
    )
    parser.add_argument(
        "--output",
        default="benchmarks/dynamic_obstacles_50_holdout_seeds.json",
    )
    parser.add_argument("--no-repeat", action="store_true")
    args = parser.parse_args(argv)
    payload = run_benchmark(repeat=not args.no_repeat)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(output)


if __name__ == "__main__":
    main()
