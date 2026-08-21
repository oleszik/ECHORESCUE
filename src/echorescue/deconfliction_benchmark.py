import argparse
import json
from dataclasses import asdict
from pathlib import Path
from statistics import mean

from echorescue.config import SimulationConfig
from echorescue.multi_simulation import MultiDroneSimulation, MultiSimulationResult


DECONFLICTION_BENCHMARK_SCHEMA_VERSION = "1.0"


def _summary(results: list[MultiSimulationResult]) -> dict[str, object]:
    return {
        "successful_missions": sum(result.mission_success for result in results),
        "success_rate": round(mean(result.mission_success for result in results), 6),
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
        "average_mission_steps": round(mean(result.steps for result in results), 3),
        "safety_shield_interventions": sum(
            result.safety_shield_interventions for result in results
        ),
        "local_motion_conflicts": sum(
            result.local_motion_conflicts for result in results
        ),
        "communication_detected_conflicts": sum(
            result.communication_detected_conflicts for result in results
        ),
        "proximity_detected_conflicts": sum(
            result.proximity_detected_conflicts for result in results
        ),
        "yield_steps_by_drone": {
            drone_id: sum(
                result.yield_steps_by_drone[drone_id] for result in results
            )
            for drone_id in ("drone-1", "drone-2")
        },
        "corridor_deadlocks": sum(result.corridor_deadlocks for result in results),
        "deadlocks_resolved": sum(result.deadlocks_resolved for result in results),
        "local_replans_due_to_drones": sum(
            result.local_replans_due_to_drones for result in results
        ),
        "deconfliction_delay_steps": sum(
            result.deconfliction_delay_steps for result in results
        ),
    }


def run_deconfliction_benchmark(seed_count: int = 50) -> dict[str, object]:
    if seed_count < 1:
        raise ValueError("seed_count must be positive")
    enabled_results = []
    disabled_results = []
    deterministic = True
    rows = []
    for seed in range(seed_count):
        enabled_config = SimulationConfig(
            seed=seed,
            drone_count=2,
            communication_range=8,
            knowledge_mode="local",
            distributed_deconfliction_enabled=True,
        )
        disabled_config = SimulationConfig(
            seed=seed,
            drone_count=2,
            communication_range=8,
            knowledge_mode="local",
            distributed_deconfliction_enabled=False,
        )
        enabled = MultiDroneSimulation(enabled_config).run()
        repeated = MultiDroneSimulation(enabled_config).run()
        disabled = MultiDroneSimulation(disabled_config).run()
        deterministic = deterministic and enabled == repeated
        enabled_results.append(enabled)
        disabled_results.append(disabled)
        rows.append(
            {
                "seed": seed,
                "mission_success": enabled.mission_success,
                "steps": enabled.steps,
                "legacy_steps": disabled.steps,
                "step_delta": enabled.steps - disabled.steps,
                "survivor_recall_at_base": round(
                    enabled.survivor_recall_at_base, 6
                ),
                "drones_returned": enabled.drones_returned,
                "collisions": enabled.collisions,
                "drone_drone_collisions": enabled.drone_drone_collisions,
                "local_motion_conflicts": enabled.local_motion_conflicts,
                "communication_detected_conflicts": (
                    enabled.communication_detected_conflicts
                ),
                "proximity_detected_conflicts": (
                    enabled.proximity_detected_conflicts
                ),
                "safety_shield_interventions": (
                    enabled.safety_shield_interventions
                ),
                "legacy_safety_shield_interventions": (
                    disabled.safety_shield_interventions
                ),
            }
        )

    enabled_summary = _summary(enabled_results)
    disabled_summary = _summary(disabled_results)
    step_delta = (
        enabled_summary["average_mission_steps"]
        - disabled_summary["average_mission_steps"]
    )
    base_config = asdict(
        SimulationConfig(
            seed=0,
            drone_count=2,
            communication_range=8,
            knowledge_mode="local",
        )
    )
    base_config["seed"] = f"0..{seed_count - 1}"
    acceptance = {
        "survivor_recall_100_percent": (
            enabled_summary["average_survivor_recall_at_base"] == 1.0
        ),
        "zero_wall_and_drone_collisions": (
            enabled_summary["wall_collisions"] == 0
            and enabled_summary["drone_drone_collisions"] == 0
        ),
        "both_drones_returned_every_mission": (
            enabled_summary["both_drones_returned"] == seed_count
        ),
        "zero_safety_shield_interventions": (
            enabled_summary["safety_shield_interventions"] == 0
        ),
        "zero_deadlocks_and_timeouts": (
            enabled_summary["corridor_deadlocks"] == 0
            and enabled_summary["timeouts"] == 0
        ),
    }
    return {
        "schema_version": DECONFLICTION_BENCHMARK_SCHEMA_VERSION,
        "suite": {
            "seed_count": seed_count,
            "seeds": list(range(seed_count)),
            "configuration": base_config,
            "enabled_runs_per_seed": 2,
            "legacy_runs_per_seed": 1,
            "determinism_check": "passed" if deterministic else "failed",
            "acceptance": acceptance,
        },
        "distributed_deconfliction": enabled_summary,
        "legacy_safety_shield_baseline": disabled_summary,
        "mission_duration_delta": {
            "average_steps": round(step_delta, 3),
            "percent": round(
                100
                * step_delta
                / disabled_summary["average_mission_steps"],
                3,
            ),
        },
        "remaining_shield_cases": [
            row for row in rows if row["safety_shield_interventions"]
        ],
        "per_seed": rows,
    }


def write_benchmark(payload: dict[str, object], output: str | Path) -> Path:
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(
        (
            json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False)
            + "\n"
        ).encode("utf-8")
    )
    return path


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Benchmark distributed Active Local deconfliction."
    )
    parser.add_argument("--seeds", type=int, default=50)
    parser.add_argument(
        "--output",
        default="benchmarks/distributed_deconfliction_50_seeds.json",
    )
    args = parser.parse_args(argv)
    print(write_benchmark(run_deconfliction_benchmark(args.seeds), args.output))


if __name__ == "__main__":
    main()
