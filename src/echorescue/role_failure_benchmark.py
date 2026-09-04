import argparse
import json
from pathlib import Path
from statistics import mean, median, pstdev

from echorescue.config import SimulationConfig
from echorescue.multi_simulation import MultiDroneSimulation, MultiSimulationResult


BENCHMARK_SCHEMA_VERSION = "1.0"
DEVELOPMENT_SEEDS = tuple(range(10))
HOLDOUT_SEEDS = tuple(range(50, 100))
FAILURE_AGENT_ID = "drone-2"
FAILURE_STEP = 12
SCENARIOS = {
    "A_generalist_no_failure": ("generalist", False),
    "B_generalist_failure": ("generalist", True),
    "C_generalized_no_failure": ("generalized", False),
    "D_generalized_failure": ("generalized", True),
}


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


def _metrics(result: MultiSimulationResult) -> dict[str, object]:
    metrics = result.role_failure_metrics
    if metrics is None:
        raise AssertionError("role benchmark result has no role metrics")
    return metrics


def _safety_check(result: MultiSimulationResult, label: str) -> None:
    if result.collisions or result.drone_drone_collisions:
        raise RuntimeError(
            f"safety failure in {label}: wall={result.collisions}, "
            f"drone={result.drone_drone_collisions}"
        )


def _optional_metric(
    results: list[MultiSimulationResult], key: str
) -> list[float | int]:
    values = [_metrics(result)[key] for result in results]
    return [
        value
        for value in values
        if isinstance(value, (float, int)) and not isinstance(value, bool)
    ]


def aggregate(results: list[MultiSimulationResult]) -> dict[str, object]:
    configured = sum(result.drones_total for result in results)
    operational = sum(int(_metrics(result)["operational_agents"]) for result in results)
    returned = sum(
        int(_metrics(result)["operational_agents_returned"])
        for result in results
    )
    orphaned = sum(int(_metrics(result)["tasks_orphaned"]) for result in results)
    reassigned = sum(
        int(_metrics(result)["successful_reassignments"])
        for result in results
    )
    return {
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
        "reassignment_latency": distribution(
            _optional_metric(results, "mean_reassignment_latency")
        ),
        "recovery_latency": distribution(
            _optional_metric(results, "mean_recovery_latency")
        ),
        "role_changes": distribution(
            _optional_metric(results, "role_changes")
        ),
        "task_completions_after_reassignment": distribution(
            _optional_metric(results, "tasks_completed_by_reassigned_agent")
        ),
        "tasks_orphaned": orphaned,
        "successful_reassignments": reassigned,
        "failed_reassignments": sum(
            int(_metrics(result)["failed_reassignments"])
            for result in results
        ),
        "reassignment_success": {
            "count": reassigned,
            "opportunities": orphaned,
            "percentage": (
                round(100 * reassigned / orphaned, 2) if orphaned else None
            ),
        },
        "operational_return_success": {
            "count": returned,
            "operational_agents": operational,
            "percentage": round(100 * returned / operational, 2),
        },
        "configured_agents": configured,
        "wall_collisions": sum(result.collisions for result in results),
        "drone_collisions": sum(
            result.drone_drone_collisions for result in results
        ),
        "role_thrashing_missions": sum(
            bool(_metrics(result)["role_thrashing_detected"])
            for result in results
        ),
        "emergency_role_takeovers": sum(
            int(_metrics(result)["emergency_role_takeovers"])
            for result in results
        ),
    }


def _paired_row(
    seed: int, results: dict[str, MultiSimulationResult]
) -> dict[str, object]:
    a = results["A_generalist_no_failure"]
    b = results["B_generalist_failure"]
    c = results["C_generalized_no_failure"]
    d = results["D_generalized_failure"]
    bm = _metrics(b)
    dm = _metrics(d)
    return {
        "seed": seed,
        "A_generalist_no_failure": {
            "duration": a.steps,
            "path_length": _path_length(a),
            "mission_success": a.mission_success,
            "survivor_recall": a.survivor_recall,
        },
        "B_generalist_failure": {
            "duration": b.steps,
            "path_length": _path_length(b),
            "mission_success": b.mission_success,
            "survivor_recall": b.survivor_recall,
            "reassignment_latency": bm["mean_reassignment_latency"],
            "recovery_latency": bm["mean_recovery_latency"],
            "role_changes": bm["role_changes"],
            "task_completions": bm["tasks_completed_by_reassigned_agent"],
            "successful_reassignments": bm["successful_reassignments"],
            "operational_agents_returned": bm["operational_agents_returned"],
        },
        "C_generalized_no_failure": {
            "duration": c.steps,
            "path_length": _path_length(c),
            "mission_success": c.mission_success,
            "survivor_recall": c.survivor_recall,
        },
        "D_generalized_failure": {
            "duration": d.steps,
            "path_length": _path_length(d),
            "mission_success": d.mission_success,
            "survivor_recall": d.survivor_recall,
            "reassignment_latency": dm["mean_reassignment_latency"],
            "recovery_latency": dm["mean_recovery_latency"],
            "role_changes": dm["role_changes"],
            "task_completions": dm["tasks_completed_by_reassigned_agent"],
            "successful_reassignments": dm["successful_reassignments"],
            "operational_agents_returned": dm["operational_agents_returned"],
        },
        "generalist_failure_overhead": {
            "duration": b.steps - a.steps,
            "path_length": _path_length(b) - _path_length(a),
            "survivor_recall": round(b.survivor_recall - a.survivor_recall, 6),
            "mission_success_changed": b.mission_success != a.mission_success,
        },
        "generalized_failure_overhead": {
            "duration": d.steps - c.steps,
            "path_length": _path_length(d) - _path_length(c),
            "survivor_recall": round(d.survivor_recall - c.survivor_recall, 6),
            "mission_success_changed": d.mission_success != c.mission_success,
        },
        "roles_vs_generalist_under_failure": {
            "duration": d.steps - b.steps,
            "path_length": _path_length(d) - _path_length(b),
            "survivor_recall": round(d.survivor_recall - b.survivor_recall, 6),
            "recovery_latency": (
                float(dm["mean_recovery_latency"])
                - float(bm["mean_recovery_latency"])
                if dm["mean_recovery_latency"] is not None
                and bm["mean_recovery_latency"] is not None
                else None
            ),
        },
    }


def _delta_distribution(
    rows: list[dict[str, object]], section: str, field: str
) -> dict[str, float | int | None]:
    return distribution(
        [
            int(row[section][field])  # type: ignore[index]
            for row in rows
        ]
    )


def failure_analysis(rows: list[dict[str, object]]) -> dict[str, object]:
    def section(row: dict[str, object], name: str) -> dict[str, object]:
        value = row[name]
        assert isinstance(value, dict)
        return value

    generalized_by_duration = sorted(
        rows,
        key=lambda row: (
            -int(section(row, "generalized_failure_overhead")["duration"]),
            int(row["seed"]),
        ),
    )
    generalized_by_path = sorted(
        rows,
        key=lambda row: (
            -int(section(row, "generalized_failure_overhead")["path_length"]),
            int(row["seed"]),
        ),
    )
    return {
        "largest_failure_duration_overhead": [
            {
                "seed": row["seed"],
                "delta": section(row, "generalized_failure_overhead")["duration"],
            }
            for row in generalized_by_duration[:5]
        ],
        "largest_failure_path_overhead": [
            {
                "seed": row["seed"],
                "delta": section(row, "generalized_failure_overhead")["path_length"],
            }
            for row in generalized_by_path[:5]
        ],
        "slow_recovery_seeds": [
            int(row["seed"])
            for row in rows
            if float(section(row, "D_generalized_failure")["recovery_latency"] or 0) > 1
        ],
        "reassignment_failure_seeds": [
            int(row["seed"])
            for row in rows
            if int(section(row, "D_generalized_failure")["successful_reassignments"]) < 1
        ],
        "role_thrashing_seeds": [
            int(row["seed"])
            for row in rows
            if int(section(row, "D_generalized_failure")["role_changes"]) > 2
        ],
        "recall_loss_seeds": [
            int(row["seed"])
            for row in rows
            if float(section(row, "generalized_failure_overhead")["survivor_recall"]) < 0
        ],
        "mission_failure_seeds": [
            int(row["seed"])
            for row in rows
            if not bool(section(row, "D_generalized_failure")["mission_success"])
        ],
        "low_failure_effect_seeds": [
            int(row["seed"])
            for row in rows
            if abs(int(section(row, "generalized_failure_overhead")["duration"])) <= 1
        ],
        "roles_faster_seeds": [
            int(row["seed"])
            for row in rows
            if int(section(row, "roles_vs_generalist_under_failure")["duration"]) < 0
        ],
        "roles_slower_seeds": [
            int(row["seed"])
            for row in rows
            if int(section(row, "roles_vs_generalist_under_failure")["duration"]) > 0
        ],
    }


def run_benchmark(
    *, seeds: tuple[int, ...] = HOLDOUT_SEEDS, repeat: bool = True
) -> dict[str, object]:
    if not seeds:
        raise ValueError("seeds must not be empty")
    by_scenario: dict[str, list[MultiSimulationResult]] = {
        name: [] for name in SCENARIOS
    }
    rows = []
    deterministic = True
    for seed in seeds:
        seed_results: dict[str, MultiSimulationResult] = {}
        for name, (policy, failure) in SCENARIOS.items():
            config = SimulationConfig(
                seed=seed,
                drone_count=4,
                role_policy=policy,
                failure_schedule=(
                    ((FAILURE_AGENT_ID, FAILURE_STEP),) if failure else ()
                ),
            )
            result = MultiDroneSimulation(config).run()
            _safety_check(result, f"{name} seed {seed}")
            if repeat and failure:
                repeated = MultiDroneSimulation(config).run()
                _safety_check(repeated, f"{name} repeat seed {seed}")
                deterministic = deterministic and result == repeated
            seed_results[name] = result
            by_scenario[name].append(result)
        rows.append(_paired_row(seed, seed_results))

    generalized_duration = _delta_distribution(
        rows, "generalized_failure_overhead", "duration"
    )
    generalized_path = _delta_distribution(
        rows, "generalized_failure_overhead", "path_length"
    )
    roles_duration = _delta_distribution(
        rows, "roles_vs_generalist_under_failure", "duration"
    )
    roles_path = _delta_distribution(
        rows, "roles_vs_generalist_under_failure", "path_length"
    )
    demo_candidates = [
        result.seed
        for result in by_scenario["D_generalized_failure"]
        if result.mission_success
        and int(_metrics(result)["successful_reassignments"]) == 1
        and int(_metrics(result)["operational_agents_returned"]) == 3
        and int(_metrics(result)["role_changes"]) > 0
    ]
    all_results = [result for values in by_scenario.values() for result in values]
    failure_results = (
        by_scenario["B_generalist_failure"]
        + by_scenario["D_generalized_failure"]
    )
    return {
        "schema_version": BENCHMARK_SCHEMA_VERSION,
        "benchmark_type": "generalized_roles_failure_resilience",
        "configuration": {
            "development_seeds": list(DEVELOPMENT_SEEDS),
            "holdout_seeds": list(seeds),
            "drone_count": 4,
            "failure_agent_id": FAILURE_AGENT_ID,
            "failure_step": FAILURE_STEP,
            "knowledge_mode": "shared",
            "network_profile": "ideal",
            "relay_strategy": "off",
            "survivor_sensor": "visual",
            "perception_noise": "off",
            "smoke_profile": "off",
            "dynamic_obstacles": "off",
            "scenarios": list(SCENARIOS),
        },
        "frozen_scoring": {
            "path_cost": "path edges",
            "retarget_penalty": 5,
            "task_load_penalty": 4,
            "role_penalty": {
                "preferred": 0,
                "generalist": 3,
                "other_specialist": 6,
            },
            "energy_penalty": "floor(consumed_energy / 20)",
            "energy_gate": "task path + known return path + safety reserve",
            "tie_break": "score, path cost, remaining energy descending, agent ID, target",
        },
        "scenarios": {
            name: aggregate(results) for name, results in by_scenario.items()
        },
        "paired_failure_overhead": {
            "generalist_duration": _delta_distribution(
                rows, "generalist_failure_overhead", "duration"
            ),
            "generalist_path_length": _delta_distribution(
                rows, "generalist_failure_overhead", "path_length"
            ),
            "generalized_duration": generalized_duration,
            "generalized_path_length": generalized_path,
        },
        "roles_vs_generalist_under_failure": {
            "duration": roles_duration,
            "path_length": roles_path,
        },
        "per_seed": rows,
        "failure_analysis": failure_analysis(rows),
        "representative_demo_seed": demo_candidates[0] if demo_candidates else None,
        "acceptance": {
            "deterministic_failure_repeats": deterministic,
            "collision_free": all(
                result.collisions == 0 and result.drone_drone_collisions == 0
                for result in all_results
            ),
            "all_failure_tasks_reassigned": all(
                int(_metrics(result)["successful_reassignments"]) == 1
                and int(_metrics(result)["failed_reassignments"]) == 0
                for result in failure_results
            ),
            "all_operational_agents_returned": all(
                int(_metrics(result)["operational_agents_returned"])
                == int(_metrics(result)["operational_agents"])
                for result in failure_results
            ),
            "no_role_thrashing": all(
                not bool(_metrics(result)["role_thrashing_detected"])
                for result in all_results
            ),
        },
    }


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Run the paired v0.9 role/failure holdout benchmark."
    )
    parser.add_argument(
        "--output",
        default="benchmarks/generalized_roles_failure_50_holdout_seeds.json",
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
