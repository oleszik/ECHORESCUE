import argparse
import json
from dataclasses import asdict
from pathlib import Path
from statistics import mean, median, pstdev

from echorescue.config import SimulationConfig
from echorescue.multi_simulation import MultiDroneSimulation, MultiSimulationResult


BENCHMARK_SCHEMA_VERSION = "1.0"
FLEET_SIZES = (1, 2, 4, 8)
STATISTIC_FIELDS = (
    "mission_duration",
    "explored_percent",
    "total_path_length",
    "replans_total",
    "duplicate_exploration_ratio",
)


def _result_row(result: MultiSimulationResult) -> dict[str, object]:
    total_path = sum(result.path_length_by_drone.values())
    replans = (
        sum(result.local_replanning_by_drone.values())
        + result.local_replans_due_to_drones
        + result.network_routes_replanned
    )
    deconfliction = result.movement_conflicts + result.local_motion_conflicts
    return {
        "seed": result.seed,
        "fleet_size": result.drones_total,
        "survivor_recall": result.survivor_recall,
        "mission_success": result.mission_success,
        "mission_duration": result.steps,
        "time_to_first_survivor": result.time_to_first_detection,
        "explored_percent": result.explored_percent,
        "collisions": result.collisions + result.drone_drone_collisions,
        "returned_agents": result.drones_returned,
        "failed_agents": result.drones_failed,
        "total_path_length": total_path,
        "mean_path_length_per_agent": total_path / result.drones_total,
        "replans_total": replans,
        "replans_per_agent": replans / result.drones_total,
        "duplicate_exploration_ratio": result.duplicate_exploration_ratio,
        "frontier_assignments": sum(result.frontier_assignments_by_drone.values()),
        "assignment_conflicts": result.redundant_frontier_assignments,
        "deconfliction_interventions": deconfliction,
        "messages_sent": result.network_fragments_sent,
        "messages_delivered": result.network_fragments_delivered,
        "maximum_queue_size": result.network_max_queue_size,
    }


def _statistics(values: list[float]) -> dict[str, float]:
    return {
        "mean": round(mean(values), 6),
        "standard_deviation": round(pstdev(values), 6),
        "median": round(median(values), 6),
        "min": round(min(values), 6),
        "max": round(max(values), 6),
    }


def _aggregate(rows: list[dict[str, object]]) -> dict[str, object]:
    fleet_size = int(rows[0]["fleet_size"])
    return {
        "fleet_size": fleet_size,
        "missions": len(rows),
        "successful_missions": sum(bool(row["mission_success"]) for row in rows),
        "mission_success_rate": round(
            mean(bool(row["mission_success"]) for row in rows), 6
        ),
        "mean_survivor_recall": round(
            mean(float(row["survivor_recall"]) for row in rows), 6
        ),
        "mean_time_to_first_survivor": round(
            mean(
                float(row["time_to_first_survivor"])
                for row in rows
                if row["time_to_first_survivor"] is not None
            ),
            6,
        ),
        "returned_agents": sum(int(row["returned_agents"]) for row in rows),
        "failed_agents": sum(int(row["failed_agents"]) for row in rows),
        "collisions": sum(int(row["collisions"]) for row in rows),
        "mean_path_length_per_agent": round(
            mean(float(row["mean_path_length_per_agent"]) for row in rows), 6
        ),
        "mean_replans_per_agent": round(
            mean(float(row["replans_per_agent"]) for row in rows), 6
        ),
        "mean_frontier_assignments": round(
            mean(int(row["frontier_assignments"]) for row in rows), 6
        ),
        "assignment_conflicts": sum(
            int(row["assignment_conflicts"]) for row in rows
        ),
        "deconfliction_interventions": sum(
            int(row["deconfliction_interventions"]) for row in rows
        ),
        "messages_sent": sum(int(row["messages_sent"]) for row in rows),
        "messages_delivered": sum(
            int(row["messages_delivered"]) for row in rows
        ),
        "maximum_queue_size": max(int(row["maximum_queue_size"]) for row in rows),
        "statistics": {
            field: _statistics([float(row[field]) for row in rows])
            for field in STATISTIC_FIELDS
        },
    }


def run_benchmark(seed_count: int = 50) -> dict[str, object]:
    if seed_count < 1:
        raise ValueError("seed_count must be positive")
    rows_by_fleet: dict[int, list[dict[str, object]]] = {}
    deterministic = True
    for fleet_size in FLEET_SIZES:
        rows = []
        for seed in range(seed_count):
            config = SimulationConfig(seed=seed, drone_count=fleet_size)
            first = MultiDroneSimulation(config).run()
            second = MultiDroneSimulation(config).run()
            deterministic = deterministic and first == second
            rows.append(_result_row(first))
        rows_by_fleet[fleet_size] = rows

    aggregates = {
        str(fleet_size): _aggregate(rows_by_fleet[fleet_size])
        for fleet_size in FLEET_SIZES
    }
    single_duration = float(
        aggregates["1"]["statistics"]["mission_duration"]["mean"]
    )
    scaling = {}
    previous_size = None
    for fleet_size in FLEET_SIZES:
        duration = float(
            aggregates[str(fleet_size)]["statistics"]["mission_duration"]["mean"]
        )
        speedup = single_duration / duration
        item = {
            "speedup": round(speedup, 6),
            "parallel_efficiency": round(speedup / fleet_size, 6),
        }
        if previous_size is not None:
            previous_duration = float(
                aggregates[str(previous_size)]["statistics"]["mission_duration"]["mean"]
            )
            item["marginal_gain_steps"] = round(previous_duration - duration, 6)
            item["marginal_gain_percent"] = round(
                100.0 * (previous_duration - duration) / previous_duration, 6
            )
        scaling[str(fleet_size)] = item
        previous_size = fleet_size

    regressions = {}
    for smaller, larger in zip(FLEET_SIZES, FLEET_SIZES[1:]):
        examples = []
        for smaller_row, larger_row in zip(
            rows_by_fleet[smaller], rows_by_fleet[larger]
        ):
            if int(larger_row["mission_duration"]) > int(
                smaller_row["mission_duration"]
            ):
                examples.append(
                    {
                        "seed": smaller_row["seed"],
                        "smaller_duration": smaller_row["mission_duration"],
                        "larger_duration": larger_row["mission_duration"],
                        "larger_overlap": larger_row["duplicate_exploration_ratio"],
                        "larger_replans": larger_row["replans_total"],
                        "larger_deconfliction_interventions": larger_row[
                            "deconfliction_interventions"
                        ],
                    }
                )
        regressions[f"{smaller}_to_{larger}"] = {
            "count": len(examples),
            "examples": examples,
        }

    base_config = asdict(SimulationConfig(seed=0, drone_count=1))
    base_config.pop("seed")
    base_config.pop("drone_count")
    return {
        "schema_version": BENCHMARK_SCHEMA_VERSION,
        "benchmark_type": "n_agent_scaling",
        "configuration": {
            "seeds": list(range(seed_count)),
            "fleet_sizes": list(FLEET_SIZES),
            "runs_per_seed_and_fleet": 2,
            "baseline": base_config,
        },
        "determinism_check": "passed" if deterministic else "failed",
        "aggregates": aggregates,
        "scaling": scaling,
        "per_seed_regressions": regressions,
        "per_seed_results": {
            str(fleet_size): rows_by_fleet[fleet_size]
            for fleet_size in FLEET_SIZES
        },
        "acceptance": {
            "all_deterministic": deterministic,
            "all_missions_successful": all(
                bool(row["mission_success"])
                for rows in rows_by_fleet.values()
                for row in rows
            ),
            "all_agents_returned": all(
                int(row["returned_agents"]) == int(row["fleet_size"])
                for rows in rows_by_fleet.values()
                for row in rows
            ),
            "collision_free": all(
                int(row["collisions"]) == 0
                for rows in rows_by_fleet.values()
                for row in rows
            ),
        },
    }


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Benchmark 1/2/4/8-agent scaling.")
    parser.add_argument("--seeds", type=int, default=50)
    parser.add_argument(
        "--output", default="benchmarks/n_agent_scaling_50_seeds.json"
    )
    args = parser.parse_args(argv)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(run_benchmark(args.seeds), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(output)


if __name__ == "__main__":
    main()
