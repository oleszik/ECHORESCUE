import argparse
import json
from dataclasses import asdict
from pathlib import Path
from statistics import mean

from echorescue.config import SimulationConfig
from echorescue.multi_simulation import MultiDroneSimulation, MultiSimulationResult
from echorescue.shadow_benchmark import _behavior_fingerprint


RELAY_BENCHMARK_SCHEMA_VERSION = "1.0"
EXPECTED_ACTIVE_LOCAL_OFF_FINGERPRINT_50_SEEDS = (
    "db80668469f645f5133b2c5bc53bfbeeefe91108d9e0103dc8a6b8369761b5bb"
)


def _average_optional(values: list[int | None]) -> float | None:
    present = [value for value in values if value is not None]
    return round(mean(present), 3) if present else None


def _coverage_curve(results: list[MultiSimulationResult]) -> list[float]:
    max_samples = max(len(result.base_known_coverage_over_time) for result in results)
    curve = []
    for index in range(max_samples):
        values = [
            history[index] if index < len(history) else history[-1]
            for result in results
            if (history := result.base_known_coverage_over_time)
        ]
        curve.append(round(mean(values), 6))
    return curve


def _summary(results: list[MultiSimulationResult]) -> dict[str, object]:
    energy_consumed = [
        2 * 220.0 - sum(result.energy_remaining_by_drone.values())
        for result in results
    ]
    first_survivor = [
        result.time_to_first_base_survivor_confirmation for result in results
    ]
    all_survivors = [
        result.time_to_all_base_survivor_confirmations for result in results
    ]
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
        "average_total_path_length": round(
            mean(sum(result.path_length_by_drone.values()) for result in results),
            3,
        ),
        "average_fleet_energy_consumed": round(mean(energy_consumed), 3),
        "average_communication_uptime": round(
            mean(
                mean(result.communication_uptime_by_drone.values())
                for result in results
            ),
            6,
        ),
        "average_final_base_known_coverage": round(
            mean(result.base_known_coverage for result in results), 6
        ),
        "average_base_coverage_by_step": _coverage_curve(results),
        "average_time_to_first_base_survivor_confirmation": _average_optional(
            first_survivor
        ),
        "first_survivor_samples": sum(value is not None for value in first_survivor),
        "average_time_to_all_base_survivor_confirmations": _average_optional(
            all_survivors
        ),
        "all_survivor_samples": sum(value is not None for value in all_survivors),
        "safety_shield_interventions": sum(
            result.safety_shield_interventions for result in results
        ),
        "relay_deployments": sum(result.relay_deployments for result in results),
        "successful_relay_deployments": sum(
            result.successful_relay_deployments for result in results
        ),
        "failed_relay_deployments": sum(
            result.failed_relay_deployments for result in results
        ),
        "relay_role_steps": sum(
            sum(result.relay_steps_by_drone.values()) for result in results
        ),
        "relay_path_length": sum(
            sum(result.relay_path_length_by_drone.values()) for result in results
        ),
        "relay_unique_cells_forwarded": sum(
            result.relay_unique_cells_forwarded for result in results
        ),
        "relay_survivor_confirmations_forwarded": sum(
            result.relay_survivor_confirmations_forwarded for result in results
        ),
        "relay_outages_shortened": sum(
            result.relay_outages_shortened for result in results
        ),
        "relay_controlled_delay_steps": sum(
            result.relay_mission_delay_steps for result in results
        ),
        "relay_energy_consumed": round(
            sum(result.relay_energy_consumed for result in results), 3
        ),
    }


def _delta(adaptive: float, off: float) -> dict[str, float]:
    difference = adaptive - off
    return {
        "absolute": round(difference, 6),
        "percent": round(100 * difference / off, 3) if off else 0.0,
    }


def run_relay_benchmark(seed_count: int = 50) -> dict[str, object]:
    if seed_count < 1:
        raise ValueError("seed_count must be positive")
    off_results = []
    adaptive_results = []
    deterministic = True
    per_seed = []
    for seed in range(seed_count):
        common = {
            "seed": seed,
            "drone_count": 2,
            "communication_range": 8,
            "knowledge_mode": "local",
        }
        off = MultiDroneSimulation(
            SimulationConfig(**common, relay_strategy="off")
        ).run()
        adaptive_config = SimulationConfig(
            **common, relay_strategy="adaptive"
        )
        adaptive = MultiDroneSimulation(adaptive_config).run()
        repeated = MultiDroneSimulation(adaptive_config).run()
        deterministic = deterministic and adaptive == repeated
        off_results.append(off)
        adaptive_results.append(adaptive)
        per_seed.append(
            {
                "seed": seed,
                "off_steps": off.steps,
                "adaptive_steps": adaptive.steps,
                "step_delta": adaptive.steps - off.steps,
                "off_path_length": sum(off.path_length_by_drone.values()),
                "adaptive_path_length": sum(
                    adaptive.path_length_by_drone.values()
                ),
                "off_communication_uptime": round(
                    mean(off.communication_uptime_by_drone.values()), 6
                ),
                "adaptive_communication_uptime": round(
                    mean(adaptive.communication_uptime_by_drone.values()), 6
                ),
                "relay_deployments": adaptive.relay_deployments,
                "successful_relay_deployments": (
                    adaptive.successful_relay_deployments
                ),
                "relay_unique_cells_forwarded": (
                    adaptive.relay_unique_cells_forwarded
                ),
                "relay_survivor_confirmations_forwarded": (
                    adaptive.relay_survivor_confirmations_forwarded
                ),
                "mission_success": adaptive.mission_success,
                "survivor_recall_at_base": round(
                    adaptive.survivor_recall_at_base, 6
                ),
                "safety_shield_interventions": (
                    adaptive.safety_shield_interventions
                ),
            }
        )

    off_summary = _summary(off_results)
    adaptive_summary = _summary(adaptive_results)
    off_fingerprint = _behavior_fingerprint(off_results)
    safety_preserved = (
        adaptive_summary["success_rate"] == 1.0
        and adaptive_summary["average_survivor_recall_at_base"] == 1.0
        and adaptive_summary["wall_collisions"] == 0
        and adaptive_summary["drone_drone_collisions"] == 0
        and adaptive_summary["both_drones_returned"] == seed_count
        and adaptive_summary["timeouts"] == 0
        and adaptive_summary["safety_shield_interventions"] == 0
    )
    communication_improved = (
        adaptive_summary["average_communication_uptime"]
        > off_summary["average_communication_uptime"]
    )
    configuration = asdict(
        SimulationConfig(
            seed=0,
            drone_count=2,
            communication_range=8,
            knowledge_mode="local",
            relay_strategy="adaptive",
        )
    )
    configuration["seed"] = f"0..{seed_count - 1}"
    return {
        "schema_version": RELAY_BENCHMARK_SCHEMA_VERSION,
        "suite": {
            "seed_count": seed_count,
            "seeds": list(range(seed_count)),
            "configuration": configuration,
            "adaptive_runs_per_seed": 2,
            "off_runs_per_seed": 1,
            "determinism_check": "passed" if deterministic else "failed",
            "off_behavior_fingerprint_sha256": off_fingerprint,
            "off_matches_verified_fingerprint": (
                off_fingerprint
                == EXPECTED_ACTIVE_LOCAL_OFF_FINGERPRINT_50_SEEDS
                if seed_count == 50
                else None
            ),
            "acceptance": {
                "communication_uptime_improved": communication_improved,
                "safety_preserved": safety_preserved,
                "accepted": communication_improved and safety_preserved,
            },
        },
        "active_local_relay_off": off_summary,
        "active_local_adaptive_relay": adaptive_summary,
        "trade_off": {
            "communication_uptime_percentage_points": round(
                100
                * (
                    adaptive_summary["average_communication_uptime"]
                    - off_summary["average_communication_uptime"]
                ),
                3,
            ),
            "mission_steps": _delta(
                adaptive_summary["average_mission_steps"],
                off_summary["average_mission_steps"],
            ),
            "total_path_length": _delta(
                adaptive_summary["average_total_path_length"],
                off_summary["average_total_path_length"],
            ),
            "fleet_energy_consumed": _delta(
                adaptive_summary["average_fleet_energy_consumed"],
                off_summary["average_fleet_energy_consumed"],
            ),
            "time_to_first_base_survivor_confirmation": _delta(
                adaptive_summary[
                    "average_time_to_first_base_survivor_confirmation"
                ],
                off_summary["average_time_to_first_base_survivor_confirmation"],
            ),
        },
        "per_seed": per_seed,
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
        description="Compare Active Local missions with adaptive relay off/on."
    )
    parser.add_argument("--seeds", type=int, default=50)
    parser.add_argument(
        "--output", default="benchmarks/adaptive_relay_50_seeds.json"
    )
    args = parser.parse_args(argv)
    print(write_benchmark(run_relay_benchmark(args.seeds), args.output))


if __name__ == "__main__":
    main()
