import argparse
import json
from collections import Counter
from dataclasses import asdict
from pathlib import Path
from statistics import mean

from echorescue.config import SimulationConfig
from echorescue.multi_simulation import MultiDroneSimulation, MultiSimulationResult
from echorescue.shadow_benchmark import (
    EXPECTED_BEHAVIOR_FINGERPRINT_50_SEEDS,
    _behavior_fingerprint,
)


KNOWLEDGE_BENCHMARK_SCHEMA_VERSION = "1.0"
MODES = ("shared", "shadow", "local")


def _failure_reasons(result: MultiSimulationResult) -> tuple[str, ...]:
    reasons = []
    if result.termination_reason == "max_steps":
        reasons.append("timeout")
    if result.survivor_recall_at_base < 1.0:
        reasons.append("incomplete_base_survivor_recall")
    if result.drones_returned < result.drones_total:
        reasons.append("unsafe_or_incomplete_return")
    if result.collisions:
        reasons.append("wall_collision")
    if result.drone_drone_collisions:
        reasons.append("drone_collision")
    if result.drones_failed:
        reasons.append("terminal_drone_failure")
    return tuple(reasons)


def _summary(results: list[MultiSimulationResult]) -> dict[str, object]:
    failures = Counter(
        reason for result in results for reason in _failure_reasons(result)
    )
    return {
        "successful_missions": sum(result.mission_success for result in results),
        "success_rate": round(
            mean(result.mission_success for result in results), 6
        ),
        "average_survivor_recall_at_base": round(
            mean(result.survivor_recall_at_base for result in results), 6
        ),
        "wall_collisions": sum(result.collisions for result in results),
        "drone_drone_collisions": sum(
            result.drone_drone_collisions for result in results
        ),
        "both_drones_returned": sum(
            result.drones_returned == result.drones_total for result in results
        ),
        "timeouts": sum(
            result.termination_reason == "max_steps" for result in results
        ),
        "average_mission_steps": round(
            mean(result.steps for result in results), 3
        ),
        "mission_steps_range": [
            min(result.steps for result in results),
            max(result.steps for result in results),
        ],
        "safety_shield_interventions": sum(
            result.safety_shield_interventions for result in results
        ),
        "average_communication_uptime": round(
            mean(
                uptime
                for result in results
                for uptime in result.communication_uptime_by_drone.values()
            ),
            6,
        ),
        "average_peak_map_divergence": round(
            mean(result.peak_map_divergence_between_drones for result in results),
            6,
        ),
        "average_final_map_divergence": round(
            mean(result.map_divergence_between_drones for result in results),
            6,
        ),
        "redundant_frontier_assignments": sum(
            result.redundant_frontier_assignments for result in results
        ),
        "targets_discarded_after_reconnect": sum(
            result.targets_discarded_after_reconnect for result in results
        ),
        "local_replanning_count": sum(
            sum(result.local_replanning_by_drone.values()) for result in results
        ),
        "unique_cells_transferred_average": round(
            mean(result.unique_cells_transferred for result in results), 3
        ),
        "semantic_cell_changes_transferred": sum(
            result.semantic_cell_changes_transferred for result in results
        ),
        "failure_reasons": dict(sorted(failures.items())),
    }


def run_knowledge_benchmark(seed_count: int = 50) -> dict[str, object]:
    if seed_count < 1:
        raise ValueError("seed_count must be positive")
    results_by_mode: dict[str, list[MultiSimulationResult]] = {
        mode: [] for mode in MODES
    }
    deterministic = True
    rows = []
    for seed in range(seed_count):
        row: dict[str, object] = {"seed": seed}
        for mode in MODES:
            config = SimulationConfig(
                seed=seed,
                drone_count=2,
                communication_range=8,
                knowledge_mode=mode,
            )
            first = MultiDroneSimulation(config).run()
            second = MultiDroneSimulation(config).run()
            deterministic = deterministic and first == second
            results_by_mode[mode].append(first)
            row[mode] = {
                "mission_success": first.mission_success,
                "steps": first.steps,
                "survivor_recall_at_base": round(
                    first.survivor_recall_at_base, 6
                ),
                "drones_returned": first.drones_returned,
                "collisions": first.collisions,
                "drone_drone_collisions": first.drone_drone_collisions,
                "safety_shield_interventions": (
                    first.safety_shield_interventions
                ),
                "failure_reasons": list(_failure_reasons(first)),
            }
        rows.append(row)

    base_config = asdict(
        SimulationConfig(
            seed=0,
            drone_count=2,
            communication_range=8,
            knowledge_mode="local",
        )
    )
    base_config["seed"] = f"0..{seed_count - 1}"
    shared_fingerprint = _behavior_fingerprint(results_by_mode["shared"])
    shadow_fingerprint = _behavior_fingerprint(results_by_mode["shadow"])
    return {
        "schema_version": KNOWLEDGE_BENCHMARK_SCHEMA_VERSION,
        "suite": {
            "seed_count": seed_count,
            "seeds": list(range(seed_count)),
            "configuration": base_config,
            "runs_per_seed_per_mode": 2,
            "determinism_check": "passed" if deterministic else "failed",
            "shared_behavior_fingerprint_sha256": shared_fingerprint,
            "shadow_behavior_fingerprint_sha256": shadow_fingerprint,
            "shared_shadow_behavior_identical": (
                shared_fingerprint == shadow_fingerprint
            ),
            "matches_verified_shared_shadow_baseline": (
                shared_fingerprint == EXPECTED_BEHAVIOR_FINGERPRINT_50_SEEDS
                and shadow_fingerprint
                == EXPECTED_BEHAVIOR_FINGERPRINT_50_SEEDS
                if seed_count == 50
                else None
            ),
        },
        "modes": {
            mode: _summary(results_by_mode[mode]) for mode in MODES
        },
        "per_seed": rows,
    }


def benchmark_json_bytes(payload: dict[str, object]) -> bytes:
    return (
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
    ).encode("utf-8")


def write_benchmark(payload: dict[str, object], output: str | Path) -> Path:
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(benchmark_json_bytes(payload))
    return path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare shared, shadow, and active local knowledge modes."
    )
    parser.add_argument("--seeds", type=int, default=50)
    parser.add_argument(
        "--output", default="benchmarks/knowledge_modes_50_seeds.json"
    )
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    output = write_benchmark(
        run_knowledge_benchmark(args.seeds), args.output
    )
    print(output)


if __name__ == "__main__":
    main()
