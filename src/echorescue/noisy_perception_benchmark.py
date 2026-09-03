import argparse
import json
from pathlib import Path
from statistics import mean, median, pstdev

from echorescue.config import SimulationConfig
from echorescue.multi_simulation import MultiDroneSimulation, MultiSimulationResult


BENCHMARK_SCHEMA_VERSION = "1.0"
DEVELOPMENT_SEEDS = tuple(range(10))
HOLDOUT_SEEDS = tuple(range(50, 100))
PROFILES = {
    "A_visual_noise_off": {"survivor_sensor": "visual"},
    "B_visual_moderate_noise": {
        "survivor_sensor": "visual",
        "perception_noise": "moderate",
    },
    "C_visual_smoke_and_noise": {
        "survivor_sensor": "visual",
        "smoke_profile": "moderate",
        "perception_noise": "moderate",
    },
    "D_thermal_moderate_noise": {
        "survivor_sensor": "thermal",
        "perception_noise": "moderate",
    },
    "E_thermal_smoke_and_noise": {
        "survivor_sensor": "thermal",
        "smoke_profile": "moderate",
        "perception_noise": "moderate",
    },
}


def distribution(values: list[float | int]) -> dict[str, float | int | None]:
    return {
        "count": len(values),
        "mean": round(mean(values), 6) if values else None,
        "standard_deviation": round(pstdev(values), 6) if values else None,
        "median": round(median(values), 6) if values else None,
        "min": min(values) if values else None,
        "max": max(values) if values else None,
    }


def perception(result: MultiSimulationResult) -> dict[str, object]:
    return result.perception_metrics or {
        "false_positive_observations": 0,
        "false_negative_observations": 0,
        "false_positive_rate": 0.0,
        "false_negative_rate": 0.0,
        "hypotheses_created": result.survivors_detected,
        "false_survivor_confirmations": 0,
        "mean_confirmation_confidence": None,
        "mean_observations_per_confirmation": 2.0,
        "time_to_first_true_survivor": result.time_to_first_detection,
        "calibration": [],
    }


def aggregate(results: list[MultiSimulationResult]) -> dict[str, object]:
    metrics = [perception(result) for result in results]
    calibration = []
    for bucket_index in range(5):
        buckets = [
            item["calibration"][bucket_index]
            for item in metrics
            if item["calibration"]
        ]
        count = sum(int(bucket["count"]) for bucket in buckets)
        weighted_confidence = sum(
            float(bucket["mean_confidence"]) * int(bucket["count"])
            for bucket in buckets
            if bucket["mean_confidence"] is not None
        )
        true_count = sum(
            float(bucket["empirical_true_positive_rate"])
            * int(bucket["count"])
            for bucket in buckets
            if bucket["empirical_true_positive_rate"] is not None
        )
        calibration.append(
            {
                "range": [bucket_index / 5, (bucket_index + 1) / 5],
                "count": count,
                "mean_confidence": (
                    round(weighted_confidence / count, 6) if count else None
                ),
                "empirical_true_positive_rate": (
                    round(true_count / count, 6) if count else None
                ),
            }
        )
    total_fp = sum(int(item["false_positive_observations"]) for item in metrics)
    total_fn = sum(int(item["false_negative_observations"]) for item in metrics)
    total_tp = sum(
        int(item.get("true_positive_observations", 0)) for item in metrics
    )
    total_tn = sum(
        int(item.get("true_negative_observations", 0)) for item in metrics
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
        "time_to_first_true_survivor": distribution(
            [
                int(item["time_to_first_true_survivor"])
                for item in metrics
                if item["time_to_first_true_survivor"] is not None
            ]
        ),
        "false_positive_observations": distribution(
            [int(item["false_positive_observations"]) for item in metrics]
        ),
        "false_negative_observations": distribution(
            [int(item["false_negative_observations"]) for item in metrics]
        ),
        "confirmation_confidence": distribution(
            [
                float(item["mean_confirmation_confidence"])
                for item in metrics
                if item["mean_confirmation_confidence"] is not None
            ]
        ),
        "observations_per_confirmation": distribution(
            [
                float(item["mean_observations_per_confirmation"])
                for item in metrics
                if item["mean_observations_per_confirmation"] is not None
            ]
        ),
        "false_positive_rate": (
            round(total_fp / (total_fp + total_tn), 6)
            if total_fp + total_tn
            else 0.0
        ),
        "false_negative_rate": (
            round(total_fn / (total_fn + total_tp), 6)
            if total_fn + total_tp
            else 0.0
        ),
        "calibration": calibration,
        "wall_collisions": sum(result.collisions for result in results),
        "drone_collisions": sum(
            result.drone_drone_collisions for result in results
        ),
        "returned_agents": sum(result.drones_returned for result in results),
        "configured_agents": sum(result.drones_total for result in results),
    }


def failure_analysis(results: list[MultiSimulationResult]) -> dict[str, object]:
    rows = [(result.seed, perception(result), result) for result in results]
    return {
        "survivor_miss_seeds": [
            seed for seed, _, result in rows if result.survivor_recall < 1.0
        ],
        "false_positive_seeds": [
            seed
            for seed, metric, _ in rows
            if int(metric["false_positive_observations"]) > 0
        ],
        "false_confirmation_seeds": [
            seed
            for seed, metric, _ in rows
            if int(metric["false_survivor_confirmations"]) > 0
        ],
        "safe_but_incomplete_seeds": [
            seed
            for seed, _, result in rows
            if result.collisions == 0
            and result.drone_drone_collisions == 0
            and result.drones_returned == result.drones_total
            and result.survivor_recall < 1.0
        ],
        "many_hypotheses_seeds": [
            seed
            for seed, metric, _ in rows
            if int(metric["hypotheses_created"]) >= 20
        ],
        "long_confirmation_delay_seeds": [
            seed
            for seed, metric, _ in rows
            if metric["time_to_first_true_survivor"] is not None
            and int(metric["time_to_first_true_survivor"]) >= 30
        ],
    }


def run_benchmark(
    *, seeds: tuple[int, ...] = HOLDOUT_SEEDS, repeat: bool = True
) -> dict[str, object]:
    results_by_profile: dict[str, list[MultiSimulationResult]] = {}
    deterministic = True
    for profile_name, overrides in PROFILES.items():
        results = []
        for seed in seeds:
            config = SimulationConfig(seed=seed, drone_count=2, **overrides)
            first = MultiDroneSimulation(config).run()
            if repeat:
                deterministic = (
                    deterministic
                    and first == MultiDroneSimulation(config).run()
                )
            results.append(first)
        results_by_profile[profile_name] = results
    analyses = {
        name: failure_analysis(results)
        for name, results in results_by_profile.items()
    }
    demo_candidates = [
        result.seed
        for result in results_by_profile["B_visual_moderate_noise"]
        if result.mission_success
        and int(perception(result)["false_positive_observations"]) > 0
    ]
    return {
        "schema_version": BENCHMARK_SCHEMA_VERSION,
        "benchmark_type": "noisy_perception_holdout",
        "configuration": {
            "development_seeds": list(DEVELOPMENT_SEEDS),
            "holdout_seeds": list(seeds),
            "drone_count": 2,
            "knowledge_mode": "shared",
            "profiles": PROFILES,
        },
        "frozen_parameters": {
            "visual_false_negative_rate": 0.18,
            "thermal_false_negative_rate": 0.12,
            "visual_false_positive_rate": 0.08,
            "thermal_false_positive_rate": 0.05,
            "false_positive_confidence_range": [0.32, 0.52],
            "confirmation_threshold": 0.65,
            "minimum_positive_observations": 2,
            "rejection_threshold": 0.12,
            "negative_evidence_weight": 0.55,
            "positive_evidence_weight": 0.5,
            "negative_observation_confidence_factor": 0.25,
            "true_report_confidence": (
                "smoke_factor * distance_factor * (1 - false_negative_rate)"
            ),
        },
        "profiles": {
            name: aggregate(results)
            for name, results in results_by_profile.items()
        },
        "failure_analysis": analyses,
        "representative_demo_seed": demo_candidates[0] if demo_candidates else None,
        "acceptance": {
            "deterministic_repeats": deterministic,
            "collision_free": all(
                result.collisions == 0 and result.drone_drone_collisions == 0
                for results in results_by_profile.values()
                for result in results
            ),
            "all_agents_returned": all(
                result.drones_returned == result.drones_total
                for results in results_by_profile.values()
                for result in results
            ),
        },
    }


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Benchmark deterministic noisy survivor perception."
    )
    parser.add_argument(
        "--output",
        default="benchmarks/noisy_perception_50_holdout_seeds.json",
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
