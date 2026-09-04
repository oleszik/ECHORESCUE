"""Paired v0.10 benchmark for reactive and predictive N-agent Relay."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import mean, median, pstdev

from echorescue.config import SimulationConfig
from echorescue.multi_simulation import MultiDroneSimulation


BENCHMARK_SCHEMA_VERSION = "1.0"
DEFAULT_SEEDS = tuple(range(50, 100))
PROFILES = {
    "A_relay_off": ("multi-relay", 1, False),
    "B_reactive_max_1": ("multi-relay", 1, True),
    "C_reactive_max_2": ("multi-relay", 2, True),
    "E_predictive_max_2": ("predictive", 2, True),
}
METRIC_FIELDS = (
    "communication_uptime",
    "disconnected_agent_steps",
    "mission_duration",
    "queue_size_mean",
    "queue_size_max",
    "ttl_expirations",
    "map_sync_mean_latency",
    "final_sync_steps",
    "active_relay_steps",
    "relay_travel_distance",
    "relay_energy_consumed",
    "relay_efficiency",
    "scout_opportunity_cost_steps",
)


def benchmark_config(seed: int, strategy: str, maximum_relays: int) -> SimulationConfig:
    """Return the frozen isolated-communication benchmark configuration."""

    return SimulationConfig(
        seed=seed,
        drone_count=4,
        communication_range=8,
        knowledge_mode="local",
        network_profile="constrained",
        relay_strategy=strategy,
        role_policy="generalized",
        multi_relay_max_active=maximum_relays,
        multi_relay_activation_outage_steps=3,
        multi_relay_min_unsynced_cells=12,
        multi_relay_min_hold_steps=6,
        multi_relay_max_role_steps=24,
        multi_relay_deactivation_hysteresis_steps=3,
        multi_relay_max_deployments=8,
        multi_relay_candidate_limit=48,
        relay_prediction_horizon=4,
    )


def _row(simulation: MultiDroneSimulation) -> dict[str, object]:
    result = simulation.result()
    relay = result.multi_relay_metrics or {}
    samples = simulation._communication_samples
    disconnected = sum(
        samples - simulation._communication_connected_samples.get(drone_id, 0)
        for drone_id in simulation.runtimes
    )
    mean_uptime = mean(result.communication_uptime_by_drone.values())
    return {
        "seed": result.seed,
        "mission_success": result.mission_success,
        "survivor_recall": result.survivor_recall,
        "return_success": result.drones_returned == result.drones_total,
        "mission_duration": result.steps,
        "communication_uptime": mean_uptime,
        "base_connectivity_uptime": mean_uptime,
        "disconnected_agent_steps": disconnected,
        "queue_size_mean": result.network_average_queue_size or 0.0,
        "queue_size_max": result.network_max_queue_size,
        "ttl_expirations": result.network_messages_expired,
        "map_sync_mean_latency": result.map_sync_mean_latency or 0.0,
        "map_sync_max_latency": result.map_sync_max_latency or 0,
        "final_sync_steps": result.final_sync_duration,
        "final_sync_timeout": result.final_sync_timeout,
        "active_relay_steps": int(relay.get("active_relay_steps", 0)),
        "max_active_relays": int(relay.get("max_active_relays", 0)),
        "relay_activations": int(relay.get("relay_activations", 0)),
        "relay_deactivations": int(relay.get("relay_deactivations", 0)),
        "relay_travel_distance": int(relay.get("relay_distance_travelled", 0)),
        "relay_energy_consumed": float(relay.get("relay_energy_consumed", 0.0)),
        "relay_efficiency": float(relay.get("relay_efficiency", 0.0)),
        "scout_opportunity_cost_steps": int(
            relay.get("exploration_opportunity_cost_steps", 0)
        ),
        "scout_hold_steps": int(relay.get("scout_hold_steps", 0)),
        "prevented_disconnection_steps": int(
            relay.get("prevented_disconnection_steps", 0)
        ),
        "unnecessary_predictive_activations": int(
            relay.get("unnecessary_predictive_activations", 0)
        ),
        "relay_thrashing_detected": bool(
            relay.get("relay_thrashing_detected", False)
        ),
        "wall_collisions": result.collisions,
        "drone_collisions": result.drone_drone_collisions,
        "safety_shield_interventions": result.safety_shield_interventions,
        "base_known_coverage": result.base_known_coverage,
    }


def _run(
    seed: int,
    strategy: str,
    maximum_relays: int,
    relay_roles_enabled: bool,
) -> dict[str, object]:
    simulation = MultiDroneSimulation(
        benchmark_config(seed, strategy, maximum_relays)
    )
    simulation._multi_relay_roles_enabled = relay_roles_enabled
    simulation.run()
    return _row(simulation)


def _statistics(values: list[float]) -> dict[str, float]:
    return {
        "mean": round(mean(values), 6),
        "population_std": round(pstdev(values), 6),
        "median": round(median(values), 6),
        "min": round(min(values), 6),
        "max": round(max(values), 6),
    }


def _aggregate(rows: list[dict[str, object]]) -> dict[str, object]:
    return {
        "missions": len(rows),
        "mission_success_rate": round(
            mean(bool(row["mission_success"]) for row in rows), 6
        ),
        "mean_survivor_recall": round(
            mean(float(row["survivor_recall"]) for row in rows), 6
        ),
        "return_success_rate": round(
            mean(bool(row["return_success"]) for row in rows), 6
        ),
        "metrics": {
            key: _statistics([float(row[key]) for row in rows])
            for key in METRIC_FIELDS
        },
        "relay_activations": sum(int(row["relay_activations"]) for row in rows),
        "relay_deactivations": sum(
            int(row["relay_deactivations"]) for row in rows
        ),
        "prevented_disconnection_steps": sum(
            int(row["prevented_disconnection_steps"]) for row in rows
        ),
        "unnecessary_predictive_activations": sum(
            int(row["unnecessary_predictive_activations"]) for row in rows
        ),
        "relay_thrashing_seeds": [
            int(row["seed"])
            for row in rows
            if bool(row["relay_thrashing_detected"])
        ],
        "final_sync_timeout_seeds": [
            int(row["seed"])
            for row in rows
            if bool(row["final_sync_timeout"])
        ],
        "total_wall_collisions": sum(int(row["wall_collisions"]) for row in rows),
        "total_drone_collisions": sum(
            int(row["drone_collisions"]) for row in rows
        ),
    }


def _paired(
    rows: dict[str, list[dict[str, object]]],
    first: str,
    second: str,
) -> dict[str, object]:
    by_seed = {
        int(row["seed"]): row for row in rows[first]
    }
    differences = []
    for candidate in rows[second]:
        seed = int(candidate["seed"])
        baseline = by_seed[seed]
        differences.append(
            {
                "seed": seed,
                **{
                    key: round(
                        float(candidate[key]) - float(baseline[key]), 6
                    )
                    for key in METRIC_FIELDS
                },
                "mission_regression": bool(baseline["mission_success"])
                and not bool(candidate["mission_success"]),
            }
        )
    return {
        "first": first,
        "second": second,
        "difference_semantics": "second_minus_first",
        "statistics": {
            key: _statistics([float(row[key]) for row in differences])
            for key in METRIC_FIELDS
        },
        "per_seed": differences,
    }


def _ranked(
    differences: list[dict[str, object]],
    key: str,
    *,
    reverse: bool,
    count: int = 5,
) -> list[dict[str, object]]:
    return [
        {"seed": int(row["seed"]), key: row[key]}
        for row in sorted(
            differences,
            key=lambda row: (float(row[key]), int(row["seed"])),
            reverse=reverse,
        )[:count]
    ]


def run_benchmark(
    seeds: tuple[int, ...] = DEFAULT_SEEDS,
    *,
    repeat: bool = False,
) -> dict[str, object]:
    rows = {profile: [] for profile in PROFILES}
    deterministic = True
    for seed in seeds:
        for profile, (
            strategy,
            maximum_relays,
            relay_roles_enabled,
        ) in PROFILES.items():
            first = _run(
                seed, strategy, maximum_relays, relay_roles_enabled
            )
            rows[profile].append(first)
            if repeat:
                deterministic = deterministic and first == _run(
                    seed,
                    strategy,
                    maximum_relays,
                    relay_roles_enabled,
                )
    comparisons = {
        "one_relay_vs_off": _paired(
            rows, "A_relay_off", "B_reactive_max_1"
        ),
        "two_relays_vs_one": _paired(
            rows, "B_reactive_max_1", "C_reactive_max_2"
        ),
        "predictive_vs_reactive": _paired(
            rows, "C_reactive_max_2", "E_predictive_max_2"
        ),
    }
    two_vs_one = comparisons["two_relays_vs_one"]["per_seed"]
    predictive = comparisons["predictive_vs_reactive"]["per_seed"]
    assert isinstance(two_vs_one, list) and isinstance(predictive, list)
    all_rows = [row for profile_rows in rows.values() for row in profile_rows]
    analysis = {
        "largest_connectivity_gain_two_vs_one": _ranked(
            two_vs_one, "communication_uptime", reverse=True
        ),
        "largest_relay_cost_two_vs_one": _ranked(
            two_vs_one, "active_relay_steps", reverse=True
        ),
        "two_relays_worse_than_one": [
            int(row["seed"])
            for row in two_vs_one
            if float(row["communication_uptime"]) < 0
            or bool(row["mission_regression"])
        ],
        "relay_thrashing": sorted(
            {
                int(row["seed"])
                for row in all_rows
                if bool(row["relay_thrashing_detected"])
            }
        ),
        "ttl_explosion": _ranked(
            two_vs_one, "ttl_expirations", reverse=True
        ),
        "long_final_sync": _ranked(
            predictive, "final_sync_steps", reverse=True
        ),
        "prediction_helps": [
            int(row["seed"])
            for row in predictive
            if float(row["disconnected_agent_steps"]) < 0
        ],
        "prediction_unnecessary": [
            int(row["seed"])
            for row in rows["E_predictive_max_2"]
            if int(row["unnecessary_predictive_activations"]) > 0
        ],
        "mission_regressions": sorted(
            {
                int(row["seed"])
                for comparison in comparisons.values()
                for row in comparison["per_seed"]
                if bool(row["mission_regression"])
            }
        ),
        "high_scout_opportunity_cost": _ranked(
            predictive, "scout_opportunity_cost_steps", reverse=True
        ),
    }
    aggregates = {
        profile: _aggregate(profile_rows)
        for profile, profile_rows in rows.items()
    }
    relay_rows = [
        row
        for profile, profile_rows in rows.items()
        if profile != "A_relay_off"
        for row in profile_rows
    ]
    return {
        "schema_version": BENCHMARK_SCHEMA_VERSION,
        "benchmark": "v0.10-predictive-communication-multi-relay",
        "seeds": list(seeds),
        "configuration": {
            "fleet_size": 4,
            "communication_range": 8,
            "knowledge_mode": "local",
            "network_profile": "constrained",
            "perception": "visual_noise_off_smoke_off",
            "dynamic_obstacles": "off",
            "failures": "off",
            "relay_off_control": (
                "same multi-relay transport with Relay roles disabled"
            ),
            "development_seeds": list(range(10)),
            "holdout_seeds": [50, 99],
            "frozen_parameters": {
                "activation_outage_steps": 3,
                "minimum_unsynced_cells": 12,
                "minimum_hold_steps": 6,
                "maximum_role_steps": 24,
                "deactivation_hysteresis_steps": 3,
                "prediction_horizon": 4,
                "candidate_limit": 48,
            },
        },
        "comparison_matrix": {
            "A": "A_relay_off",
            "B": "B_reactive_max_1",
            "C": "C_reactive_max_2",
            "D": "C_reactive_max_2 (reactive baseline reused)",
            "E": "E_predictive_max_2",
        },
        "profiles": aggregates,
        "per_seed": rows,
        "paired_comparisons": comparisons,
        "per_seed_analysis": analysis,
        "acceptance": {
            "deterministic_repeats": deterministic if repeat else None,
            "collision_free": all(
                int(row["wall_collisions"]) == 0
                and int(row["drone_collisions"]) == 0
                for row in all_rows
            ),
            "no_scout_holding": all(
                int(row["scout_hold_steps"]) == 0 for row in all_rows
            ),
            "no_relay_thrashing": not analysis["relay_thrashing"],
            "all_profiles_complete": all(
                float(summary["mission_success_rate"]) == 1.0
                for summary in aggregates.values()
            ),
            "relay_profiles_collision_free": all(
                int(row["wall_collisions"]) == 0
                and int(row["drone_collisions"]) == 0
                for row in relay_rows
            ),
            "relay_profiles_complete": all(
                bool(row["mission_success"]) and bool(row["return_success"])
                for row in relay_rows
            ),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--seed-start", type=int, default=50)
    parser.add_argument("--seed-end", type=int, default=99)
    parser.add_argument("--repeat", action="store_true")
    args = parser.parse_args()
    payload = run_benchmark(
        tuple(range(args.seed_start, args.seed_end + 1)),
        repeat=args.repeat,
    )
    encoded = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if args.output is None:
        print(encoded, end="")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")


if __name__ == "__main__":
    main()
