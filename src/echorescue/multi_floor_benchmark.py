"""Reproducible v0.11 multi-floor holdout and small fleet scaling check."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from echorescue.config import SimulationConfig
from echorescue.multi_floor import (
    MultiFloorConfig,
    MultiFloorResult,
    MultiFloorSimulation,
    metric_summary,
)
from echorescue.multi_simulation import MultiDroneSimulation


def _multi_row(result: MultiFloorResult) -> dict[str, object]:
    metrics = result.metrics
    distribution = metrics["floor_work_distribution"]
    assert isinstance(distribution, dict)
    shares = [float(value) for value in distribution.values()]
    return {
        "seed": result.seed,
        "mission_success": result.success,
        "survivor_recall": result.survivor_recall,
        "return_success": result.returned_agents == 4,
        "mission_duration": result.steps,
        "total_path_length": result.total_path_length,
        "transition_count": int(metrics["floor_transitions_total"]),
        "transition_conflicts": int(metrics["transition_conflicts"]),
        "time_to_first_new_floor": metrics["time_to_first_new_floor"],
        "vertical_transition_overhead": int(
            metrics["vertical_transition_overhead"]
        ),
        "floor_work_balance": round(min(shares) / max(shares), 6),
        "explored_percent_per_floor": metrics["explored_percent_per_floor"],
        "floor_work_distribution": distribution,
        "path_length_per_floor": metrics["path_length_per_floor"],
        "floor_transitions_per_agent": metrics["floor_transitions_per_agent"],
        "floor_ping_pong_agents": metrics["floor_ping_pong_agents"],
        "wall_collisions": result.wall_collisions,
        "drone_collisions": result.drone_collisions,
    }


def _flattened_row(seed: int) -> dict[str, object]:
    # 37x9 (333 cells) is close to three 13x9 floors (351 cells), with the
    # same survivor and agent counts. Boundary geometry differs, so this is a
    # size-matched control rather than a perfect causal counterfactual.
    config = SimulationConfig(
        width=37,
        height=9,
        seed=seed,
        drone_count=4,
        survivor_count=3,
        battery_capacity=500.0,
        max_steps=1_000,
    )
    result = MultiDroneSimulation(config).run()
    return {
        "seed": seed,
        "mission_success": result.mission_success,
        "survivor_recall": result.survivor_recall,
        "return_success": result.drones_returned == result.drones_total,
        "mission_duration": result.steps,
        "total_path_length": sum(result.path_length_by_drone.values()),
        "wall_collisions": result.collisions,
        "drone_collisions": result.drone_drone_collisions,
    }


def _summarize(rows: list[dict[str, object]], metrics: tuple[str, ...]) -> dict[str, object]:
    return {
        "missions": len(rows),
        "mission_success_rate": sum(bool(row["mission_success"]) for row in rows)
        / len(rows),
        "mean_survivor_recall": sum(float(row["survivor_recall"]) for row in rows)
        / len(rows),
        "return_success_rate": sum(bool(row["return_success"]) for row in rows)
        / len(rows),
        "total_wall_collisions": sum(int(row["wall_collisions"]) for row in rows),
        "total_drone_collisions": sum(int(row["drone_collisions"]) for row in rows),
        "metrics": {
            metric: metric_summary([float(row[metric]) for row in rows])
            for metric in metrics
        },
    }


def _paired(
    flattened: list[dict[str, object]], multi: list[dict[str, object]]
) -> dict[str, object]:
    metrics = ("mission_duration", "total_path_length")
    rows = [
        {
            "seed": int(first["seed"]),
            **{
                metric: float(second[metric]) - float(first[metric])
                for metric in metrics
            },
            "transition_count": int(second["transition_count"]),
            "transition_conflicts": int(second["transition_conflicts"]),
        }
        for first, second in zip(flattened, multi)
    ]
    return {
        "difference_semantics": "multi_floor_minus_flattened",
        "per_seed": rows,
        "statistics": {
            metric: metric_summary([float(row[metric]) for row in rows])
            for metric in (
                "mission_duration",
                "total_path_length",
                "transition_count",
                "transition_conflicts",
            )
        },
    }


def _per_floor_statistics(rows: list[dict[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for metric in (
        "explored_percent_per_floor",
        "floor_work_distribution",
        "path_length_per_floor",
    ):
        floor_values: dict[str, list[float]] = {}
        for row in rows:
            values = row[metric]
            assert isinstance(values, dict)
            for floor, value in values.items():
                floor_values.setdefault(str(floor), []).append(float(value))
        result[metric] = {
            floor: metric_summary(values)
            for floor, values in sorted(floor_values.items())
        }
    return result


def _scaling(seeds: range) -> dict[str, object]:
    result: dict[str, object] = {}
    for drone_count in (2, 4, 8):
        rows = []
        for seed in seeds:
            config = MultiFloorConfig(seed=seed, drone_count=drone_count)
            mission = MultiFloorSimulation(config).run()
            row = _multi_row(mission)
            row["return_success"] = mission.returned_agents == drone_count
            rows.append(row)
        result[str(drone_count)] = _summarize(
            rows,
            ("mission_duration", "total_path_length", "transition_count"),
        )
    return result


def run_benchmark(seed_start: int = 50, seed_end: int = 99) -> dict[str, object]:
    seeds = list(range(seed_start, seed_end + 1))
    flattened = [_flattened_row(seed) for seed in seeds]
    multi = [
        _multi_row(MultiFloorSimulation(MultiFloorConfig(seed=seed)).run())
        for seed in seeds
    ]
    profiles = {
        "A_flattened_control": {
            "rows": flattened,
            **_summarize(
                flattened, ("mission_duration", "total_path_length")
            ),
        },
        "B_multi_floor": {
            "rows": multi,
            **_summarize(
                multi,
                (
                    "mission_duration",
                    "total_path_length",
                    "transition_count",
                    "transition_conflicts",
                    "time_to_first_new_floor",
                    "vertical_transition_overhead",
                    "floor_work_balance",
                ),
            ),
            "per_floor_metrics": _per_floor_statistics(multi),
        },
    }
    scaling = _scaling(range(0, 5))
    all_rows = flattened + multi
    return {
        "schema_version": "1.0",
        "benchmark": "v0.11-multi-floor-2.5d",
        "seeds": seeds,
        "configuration": {
            "agents": 4,
            "floors": 3,
            "floor_dimensions": [13, 9],
            "flattened_dimensions": [37, 9],
            "survivors": 3,
            "communication": "ideal/shared",
            "perception": "visual_noise_off_smoke_off",
            "dynamic_obstacles": "off",
            "failures": "off",
            "relays": "off",
            "development_seeds": [0, 9],
            "holdout_seeds": [seed_start, seed_end],
            "frozen_parameters": {
                "transition_cost": 3,
                "transition_energy_cost": 3.0,
                "floor_congestion_penalty": 4,
                "stairwells_per_floor_pair": 2,
            },
            "control_caveat": (
                "Size and survivor matched; boundary geometry differs, so the "
                "paired delta is not a perfectly isolated causal estimate."
            ),
        },
        "profiles": profiles,
        "paired_vertical_overhead": _paired(flattened, multi),
        "agent_scaling_development": scaling,
        "acceptance": {
            "all_profiles_complete": all(
                bool(row["mission_success"]) for row in all_rows
            ),
            "full_recall": all(float(row["survivor_recall"]) == 1.0 for row in all_rows),
            "full_return": all(bool(row["return_success"]) for row in all_rows),
            "collision_free": all(
                int(row["wall_collisions"]) == 0
                and int(row["drone_collisions"]) == 0
                for row in all_rows
            ),
            "no_floor_ping_pong": all(
                not row.get("floor_ping_pong_agents", []) for row in multi
            ),
            "scaling_complete": all(
                float(profile["mission_success_rate"]) == 1.0
                for profile in scaling.values()
                if isinstance(profile, dict)
            ),
        },
    }


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed-start", type=int, default=50)
    parser.add_argument("--seed-end", type=int, default=99)
    parser.add_argument(
        "--output", default="benchmarks/multi_floor_50_holdout_seeds.json"
    )
    args = parser.parse_args(argv)
    if args.seed_end < args.seed_start:
        parser.error("--seed-end must not be below --seed-start")
    payload: dict[str, Any] = run_benchmark(args.seed_start, args.seed_end)
    Path(args.output).write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
