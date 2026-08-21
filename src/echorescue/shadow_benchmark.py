import argparse
import hashlib
import json
from dataclasses import asdict
from pathlib import Path
from statistics import mean

from echorescue.config import SimulationConfig
from echorescue.multi_simulation import MultiDroneSimulation


SHADOW_BENCHMARK_SCHEMA_VERSION = "1.0"
EXPECTED_BEHAVIOR_FINGERPRINT_50_SEEDS = (
    "fe372a97617c605fbafa676a4abc88a2810b1892213ed7753e1ac892c2ef381d"
)
DRONE_IDS = ("drone-1", "drone-2")
BEHAVIOR_FIELDS = (
    "termination_reason",
    "steps",
    "collisions",
    "survivor_recall",
    "drones_returned",
    "drone_drone_collisions",
    "movement_conflicts",
    "wait_steps_by_drone",
    "path_length_by_drone",
    "frontier_assignments_by_drone",
    "drone_status_by_drone",
    "return_started_step_by_drone",
    "return_path_length_by_drone",
    "position_trace_by_drone",
    "duplicate_exploration_ratio",
    "mission_success",
)


def _behavior_fingerprint(results: list[object]) -> str:
    rows = []
    for result in results:
        payload = result.to_dict()
        rows.append(
            {
                "seed": result.seed,
                **{field: payload[field] for field in BEHAVIOR_FIELDS},
            }
        )
    encoded = json.dumps(
        rows, sort_keys=True, separators=(",", ":")
    ).encode()
    return hashlib.sha256(encoded).hexdigest()


def run_shadow_benchmark(seed_count: int = 50) -> dict[str, object]:
    if seed_count < 1:
        raise ValueError("seed_count must be positive")
    results = []
    deterministic = True
    for seed in range(seed_count):
        config = SimulationConfig(
            seed=seed,
            drone_count=2,
            communication_range=8,
            local_map_shadow_mode=True,
        )
        first = MultiDroneSimulation(config).run()
        second = MultiDroneSimulation(config).run()
        deterministic = deterministic and first == second
        results.append(first)

    fingerprint = _behavior_fingerprint(results)
    base_config = SimulationConfig(
        seed=0,
        drone_count=2,
        communication_range=8,
        local_map_shadow_mode=True,
    )
    configuration = asdict(base_config)
    configuration["seed"] = f"0..{seed_count - 1}"
    convergence_steps = [
        result.time_to_map_convergence
        for result in results
        if result.time_to_map_convergence is not None
    ]

    def average_by_drone(metric: str, drone_id: str) -> float:
        return mean(getattr(result, metric)[drone_id] for result in results)

    return {
        "schema_version": SHADOW_BENCHMARK_SCHEMA_VERSION,
        "suite": {
            "seed_count": seed_count,
            "seeds": list(range(seed_count)),
            "configuration": configuration,
            "runs_per_seed": 2,
            "determinism_check": "passed" if deterministic else "failed",
            "behavior_fingerprint_sha256": fingerprint,
            "matches_pre_shadow_behavior": (
                fingerprint == EXPECTED_BEHAVIOR_FINGERPRINT_50_SEEDS
                if seed_count == 50
                else None
            ),
        },
        "shadow_knowledge": {
            "average_final_local_coverage_by_drone": {
                drone_id: round(
                    average_by_drone(
                        "local_known_coverage_by_drone", drone_id
                    ),
                    6,
                )
                for drone_id in DRONE_IDS
            },
            "average_final_base_coverage": round(
                mean(result.base_known_coverage for result in results), 6
            ),
            "minimum_final_base_coverage": round(
                min(result.base_known_coverage for result in results), 6
            ),
            "average_final_shared_shadow_coverage": round(
                mean(result.shared_shadow_coverage for result in results), 6
            ),
            "average_final_divergence": round(
                mean(
                    result.map_divergence_between_drones
                    for result in results
                ),
                6,
            ),
            "maximum_final_divergence": round(
                max(
                    result.map_divergence_between_drones
                    for result in results
                ),
                6,
            ),
            "average_peak_divergence": round(
                mean(
                    result.peak_map_divergence_between_drones
                    for result in results
                ),
                6,
            ),
            "maximum_peak_divergence": round(
                max(
                    result.peak_map_divergence_between_drones
                    for result in results
                ),
                6,
            ),
            "average_final_stale_cells_by_drone": {
                drone_id: round(
                    average_by_drone("stale_cells_by_drone", drone_id), 3
                )
                for drone_id in DRONE_IDS
            },
            "average_cells_uploaded_by_drone": {
                drone_id: round(
                    average_by_drone("cells_uploaded_by_drone", drone_id), 3
                )
                for drone_id in DRONE_IDS
            },
            "average_cells_received_by_drone": {
                drone_id: round(
                    average_by_drone("cells_received_by_drone", drone_id), 3
                )
                for drone_id in DRONE_IDS
            },
            "average_map_sync_rounds": round(
                mean(result.map_sync_events for result in results), 3
            ),
            "missions_reaching_map_convergence": len(convergence_steps),
            "average_time_to_map_convergence": (
                round(mean(convergence_steps), 3)
                if convergence_steps
                else None
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
                mean(result.duplicate_exploration_ratio for result in results),
                6,
            ),
        },
    }


def benchmark_json_bytes(benchmark: dict[str, object]) -> bytes:
    return (
        json.dumps(benchmark, indent=2, sort_keys=True, ensure_ascii=False)
        + "\n"
    ).encode("utf-8")


def write_benchmark(benchmark: dict[str, object], output: str | Path) -> Path:
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(benchmark_json_bytes(benchmark))
    return path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the deterministic EchoRescue local-map shadow benchmark."
    )
    parser.add_argument("--seeds", type=int, default=50)
    parser.add_argument(
        "--output", default="benchmarks/shadow_mode_50_seeds.json"
    )
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    output = write_benchmark(run_shadow_benchmark(args.seeds), args.output)
    print(output)


if __name__ == "__main__":
    main()
