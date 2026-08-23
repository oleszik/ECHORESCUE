"""Reproducible ideal/constrained transport benchmark."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import mean

from echorescue.config import SimulationConfig
from echorescue.events import EventType
from echorescue.multi_simulation import MultiDroneSimulation, MultiSimulationResult
from echorescue.relay_benchmark import EXPECTED_ACTIVE_LOCAL_OFF_FINGERPRINT_50_SEEDS
from echorescue.shadow_benchmark import _behavior_fingerprint


NETWORK_BENCHMARK_SCHEMA_VERSION = "1.1"


def _mean_optional(values: list[float | int | None]) -> float | None:
    available = [float(value) for value in values if value is not None]
    return round(mean(available), 6) if available else None


def _summary(results: list[MultiSimulationResult]) -> dict[str, object]:
    count = len(results)
    attempts = sum(result.network_transmission_attempts for result in results)
    successful_attempts = sum(
        result.network_successful_transmission_attempts for result in results
    )
    created_fragments = sum(
        result.network_fragments_created for result in results
    )
    delivered_fragments = sum(
        result.network_fragments_delivered for result in results
    )
    queued_messages = sum(result.network_messages_queued for result in results)
    delivered_messages = sum(
        result.network_messages_delivered for result in results
    )
    final_sync_durations = [
        result.final_sync_duration for result in results if result.final_sync_started
    ]

    def aggregate_counts(
        field: str, expected: tuple[str, ...] = ()
    ) -> dict[str, int]:
        keys = set(expected) | {
            key
            for result in results
            for key in getattr(result, field)
        }
        return {
            key: sum(getattr(result, field).get(key, 0) for result in results)
            for key in sorted(keys)
        }

    return {
        "missions": count,
        "mission_success_rate": sum(result.mission_success for result in results) / count,
        "average_survivor_recall_at_base": mean(
            result.survivor_recall_at_base for result in results
        ),
        "both_drones_returned_rate": sum(result.drones_returned == 2 for result in results) / count,
        "wall_collisions": sum(result.collisions for result in results),
        "drone_collisions": sum(result.drone_drone_collisions for result in results),
        "timeouts": sum(result.termination_reason == "max_steps" for result in results),
        "average_mission_steps": mean(result.steps for result in results),
        "average_total_path_length": mean(
            sum(result.path_length_by_drone.values()) for result in results
        ),
        "average_energy_consumed": mean(
            440.0 - sum(result.energy_remaining_by_drone.values())
            for result in results
        ),
        "average_delivery_ratio": _mean_optional(
            [result.network_delivery_ratio for result in results]
        ),
        "fragment_attempt_delivery_ratio": (
            successful_attempts / attempts if attempts else None
        ),
        "unique_fragment_eventual_delivery_ratio": (
            delivered_fragments / created_fragments
            if created_fragments
            else None
        ),
        "logical_message_completion_ratio": (
            delivered_messages / queued_messages if queued_messages else None
        ),
        "average_network_latency": _mean_optional(
            [result.network_mean_latency for result in results]
        ),
        "maximum_network_latency": max(
            (result.network_max_latency or 0) for result in results
        ),
        "average_queue_size": _mean_optional(
            [result.network_average_queue_size for result in results]
        ),
        "maximum_queue_size": max(result.network_max_queue_size for result in results),
        "maximum_backlog_duration": max(
            result.network_max_backlog_duration for result in results
        ),
        "average_map_sync_latency": _mean_optional(
            [result.map_sync_mean_latency for result in results]
        ),
        "average_survivor_knowledge_latency": _mean_optional(
            [result.survivor_knowledge_mean_latency for result in results]
        ),
        "network_fragments_sent": sum(result.network_fragments_sent for result in results),
        "network_transmission_attempts": attempts,
        "network_successful_transmission_attempts": successful_attempts,
        "network_retransmission_attempts": sum(
            result.network_retransmission_attempts for result in results
        ),
        "network_fragments_created": created_fragments,
        "network_fragments_delivered": sum(
            result.network_fragments_delivered for result in results
        ),
        "network_fragments_lost": sum(result.network_fragments_lost for result in results),
        "network_fragments_expired": sum(
            result.network_messages_expired for result in results
        ),
        "network_fragments_dropped_at_mission_end": sum(
            result.network_messages_dropped for result in results
        ),
        "network_routes_replanned": sum(
            result.network_routes_replanned for result in results
        ),
        "missions_with_final_sync": len(final_sync_durations),
        "average_final_sync_duration": (
            mean(final_sync_durations) if final_sync_durations else 0.0
        ),
        "maximum_final_sync_duration": max(final_sync_durations, default=0),
        "final_sync_timeouts": sum(result.final_sync_timeout for result in results),
        "final_sync_retransmissions": sum(
            result.final_sync_retransmissions for result in results
        ),
        "final_sync_survivor_confirmations_transferred": sum(
            result.final_sync_survivor_confirmations_transferred
            for result in results
        ),
        "relay_successful_deployments": sum(
            result.successful_relay_deployments for result in results
        ),
        "relay_failed_deployments": sum(
            result.failed_relay_deployments for result in results
        ),
        "relay_network_fragments": sum(result.relay_network_fragments for result in results),
        "proximity_avoidances_without_fresh_intent": sum(
            result.proximity_avoidances_without_fresh_intent for result in results
        ),
        "safety_shield_interventions": sum(
            result.safety_shield_interventions for result in results
        ),
        "safety_shield_cause_classification": aggregate_counts(
            "network_shield_cause_classification",
            (
                "lost_intent",
                "delayed_intent",
                "expired_intent",
                "queue_displacement",
                "contact_without_radio",
            ),
        ),
        "safety_shield_geometry_classification": aggregate_counts(
            "network_shield_geometry_classification",
            ("vertex_conflict", "edge_swap", "corridor_encounter"),
        ),
    }


def _run(
    config: SimulationConfig, *, legacy_proximity_precedence: bool = False
) -> MultiSimulationResult:
    simulation = MultiDroneSimulation(config)
    simulation._legacy_network_proximity_precedence = (
        legacy_proximity_precedence
    )
    if legacy_proximity_precedence and simulation.network_transport is not None:
        simulation.network_transport.rerouting_enabled = False
    return simulation.run()


def run_network_benchmark(seed_count: int = 50) -> dict[str, object]:
    if seed_count < 1:
        raise ValueError("seed_count must be positive")
    profiles = {
        "ideal_relay_off": {
            "network_profile": "ideal",
            "relay_strategy": "off",
        },
        "constrained_relay_off": {
            "network_profile": "constrained",
            "relay_strategy": "off",
        },
        "constrained_adaptive_relay": {
            "network_profile": "constrained",
            "relay_strategy": "adaptive",
        },
    }
    results: dict[str, list[MultiSimulationResult]] = {
        name: [] for name in profiles
    }
    result_lookup: dict[tuple[str, int], MultiSimulationResult] = {}
    legacy_shield_results: dict[str, list[MultiSimulationResult]] = {
        "constrained_relay_off": [],
        "constrained_adaptive_relay": [],
    }
    determinism = True
    per_seed = []
    for seed in range(seed_count):
        seed_row: dict[str, object] = {"seed": seed}
        for name, options in profiles.items():
            config = SimulationConfig(
                seed=seed,
                drone_count=2,
                communication_range=8,
                knowledge_mode="local",
                **options,
            )
            first = _run(config)
            second = _run(config)
            determinism = determinism and first.to_dict() == second.to_dict()
            results[name].append(first)
            result_lookup[(name, seed)] = first
            seed_row[name] = {
                "mission_success": first.mission_success,
                "survivor_recall_at_base": round(first.survivor_recall_at_base, 6),
                "drones_returned": first.drones_returned,
                "termination_reason": first.termination_reason,
                "steps": first.steps,
                "collisions": first.collisions,
                "drone_collisions": first.drone_drone_collisions,
                "safety_shield_interventions": first.safety_shield_interventions,
                "delivery_ratio": (
                    round(first.network_delivery_ratio, 6)
                    if first.network_delivery_ratio is not None
                    else None
                ),
                "final_sync_started": first.final_sync_started,
                "final_sync_duration": first.final_sync_duration,
                "final_sync_timeout": first.final_sync_timeout,
            }
            if name in legacy_shield_results:
                legacy_shield_results[name].append(
                    _run(config, legacy_proximity_precedence=True)
                )
        per_seed.append(seed_row)
    summaries = {name: _summary(values) for name, values in results.items()}
    legacy_shield_classification = {
        name: {
            "safety_shield_interventions": summary[
                "safety_shield_interventions"
            ],
            "causes": summary["safety_shield_cause_classification"],
            "geometry": summary["safety_shield_geometry_classification"],
        }
        for name, values in legacy_shield_results.items()
        for summary in (_summary(values),)
    }
    affected = []
    for row in per_seed:
        for name in profiles:
            outcome = row[name]
            if (
                not outcome["mission_success"]
                or outcome["safety_shield_interventions"]
                or outcome["termination_reason"] == "max_steps"
            ):
                affected.append({"profile": name, **outcome, "seed": row["seed"]})
    for case in affected:
        result = result_lookup[(str(case["profile"]), int(case["seed"]))]
        classifications = []
        if result.survivor_recall_at_base < 1.0:
            classifications.append("base_knowledge_incomplete_at_mission_close")
        if result.termination_reason == "max_steps":
            classifications.append("mission_timeout")
        shield_events = [
            {
                "step": event.step,
                "drone_id": event.drone_id,
                "position": [event.position.x, event.position.y],
            }
            for event in result.mission_events
            if event.event_type is EventType.SAFETY_SHIELD_INTERVENTION
        ]
        if shield_events:
            classifications.append(
                "stale_or_missing_peer_intent_reached_central_fail_safe"
            )
        case["classifications"] = classifications
        case["shield_events"] = shield_events
    ideal_fingerprint = _behavior_fingerprint(results["ideal_relay_off"])
    return {
        "schema_version": NETWORK_BENCHMARK_SCHEMA_VERSION,
        "suite": {
            "seeds": list(range(seed_count)),
            "runs_per_seed": 2,
            "communication_range": 8,
            "knowledge_mode": "local",
            "determinism_check": "passed" if determinism else "failed",
            "ideal_behavior_fingerprint_sha256": ideal_fingerprint,
            "ideal_matches_verified_fingerprint": (
                seed_count != 50
                or ideal_fingerprint
                == EXPECTED_ACTIVE_LOCAL_OFF_FINGERPRINT_50_SEEDS
            ),
            "profile_parameters": {
                "latency_steps": 1,
                "packet_loss_rate": 0.05,
                "link_capacity_units_per_step": 36,
                "maximum_fragment_units": 12,
                "fairness_age_steps": 8,
            },
            "delivery_ratio_definition": (
                "Deprecated alias of unique_fragment_eventual_delivery_ratio."
            ),
            "delivery_ratio_definitions": {
                "fragment_attempt_delivery_ratio": (
                    "successful link-hop attempts / all link-hop attempts; every retry is another denominator attempt"
                ),
                "unique_fragment_eventual_delivery_ratio": (
                    "unique end fragments delivered / unique fragments created; retries are not duplicated"
                ),
                "logical_message_completion_ratio": (
                    "fully reassembled logical messages / logical messages queued"
                ),
                "packet_loss": "failed link-hop transmission attempts",
                "ttl_drop": "unique fragments expired before end delivery",
                "mission_end_drop": (
                    "unique unfinished fragments discarded only after success or timeout closes transport"
                ),
            },
            "final_sync_power_model": (
                "landed drones use abstracted base-station power; flight battery remains unchanged"
            ),
            "routing_scope": (
                "invalid queued next hops are reconsidered from the current observed graph; predictive and full dynamic routing remain out of scope"
            ),
        },
        "profiles": summaries,
        "shield_classification_before_hardening": legacy_shield_classification,
        "affected_seeds": affected,
        "per_seed": per_seed,
    }


def write_benchmark(payload: dict[str, object], output: str | Path) -> Path:
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return path


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Run constrained network benchmarks.")
    parser.add_argument("--seeds", type=int, default=50)
    parser.add_argument(
        "--output", default="benchmarks/constrained_network_50_seeds.json"
    )
    args = parser.parse_args(argv)
    print(write_benchmark(run_network_benchmark(args.seeds), args.output))


if __name__ == "__main__":
    main()
