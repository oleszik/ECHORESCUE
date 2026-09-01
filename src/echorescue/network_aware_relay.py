"""Deterministic utility model for constrained-network Relay decisions.

The model consumes only current protocol telemetry and locally known planning
facts.  It deliberately has no access to world geometry, Survivor ground truth,
or future communication states.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from math import ceil


@dataclass(frozen=True, slots=True)
class RelayUtilityWeights:
    survivor_confirmation: float = 200.0
    critical_status_unit: float = 20.0
    recent_map_cell: float = 2.0
    general_map_cell: float = 0.1
    travel_step_cost: float = 8.0
    transfer_step_cost: float = 3.0
    backlog_step_cost: float = 8.0
    energy_unit_cost: float = 2.0
    imminent_reconnect_penalty: float = 100.0

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class RelayUtilityInput:
    confirmed_survivors: int
    critical_status_units: int
    recent_map_cells: int
    general_map_cells: int
    queue_units: int
    route_hops: int
    link_capacity_units: int
    observed_delivery_ratio: float
    ttl_remaining_steps: int
    relay_travel_steps: int
    energy_headroom: float
    expected_direct_reconnect_steps: int
    maximum_hops: int
    utility_threshold: float
    maximum_low_priority_backlog_units: int


@dataclass(frozen=True, slots=True)
class RelayUtilityDecision:
    accepted: bool
    utility: float
    reason: str
    expected_transfer_steps: int
    ttl_reserve_steps: int
    deliverable_payload_units: int
    critical_payload_units: int
    map_payload_units: int
    backpressure_required: bool

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def evaluate_relay_utility(
    inputs: RelayUtilityInput,
    weights: RelayUtilityWeights = RelayUtilityWeights(),
) -> RelayUtilityDecision:
    """Evaluate a Relay deployment with stable, inspectable arithmetic."""

    critical_units = inputs.confirmed_survivors + inputs.critical_status_units
    map_units = inputs.recent_map_cells + inputs.general_map_cells
    payload_units = critical_units + map_units
    probability = min(1.0, max(0.0, inputs.observed_delivery_ratio))
    effective_capacity = max(
        1.0,
        inputs.link_capacity_units * max(0.05, probability),
    )
    transfer_steps = ceil(
        payload_units * max(1, inputs.route_hops) / effective_capacity
    ) if payload_units else 0
    ttl_reserve = (
        inputs.ttl_remaining_steps
        - inputs.relay_travel_steps
        - transfer_steps
    )
    backpressure = (
        inputs.queue_units + map_units
        > inputs.maximum_low_priority_backlog_units
    )
    deliverable_units = max(
        0,
        min(
            payload_units,
            int(
                max(0, inputs.ttl_remaining_steps - inputs.relay_travel_steps)
                * effective_capacity
                / max(1, inputs.route_hops)
            ),
        ),
    )

    expected_value = probability ** max(1, inputs.route_hops) * (
        inputs.confirmed_survivors * weights.survivor_confirmation
        + inputs.critical_status_units * weights.critical_status_unit
        + min(inputs.recent_map_cells, 24) * weights.recent_map_cell
        + min(inputs.general_map_cells, 24) * weights.general_map_cell
    )
    cost = (
        inputs.relay_travel_steps * weights.travel_step_cost
        + transfer_steps * weights.transfer_step_cost
        + (inputs.queue_units / max(1, inputs.link_capacity_units))
        * weights.backlog_step_cost
        + max(0.0, -inputs.energy_headroom) * weights.energy_unit_cost
    )
    if (
        inputs.expected_direct_reconnect_steps
        <= inputs.relay_travel_steps + transfer_steps
    ):
        cost += weights.imminent_reconnect_penalty
    utility = round(expected_value - cost, 6)

    if payload_units == 0:
        reason = "no_relevant_backlog"
    elif critical_units == 0 and inputs.recent_map_cells == 0:
        reason = "general_map_backlog_only"
    elif inputs.route_hops > inputs.maximum_hops:
        reason = "route_too_long"
    elif ttl_reserve < 0 or deliverable_units < critical_units:
        reason = "ttl_or_capacity_insufficient"
    elif inputs.energy_headroom < 0:
        reason = "energy_reserve_insufficient"
    elif backpressure and critical_units == 0:
        reason = "low_priority_backpressure"
    elif utility + 1e-9 < inputs.utility_threshold:
        reason = "utility_below_threshold"
    elif inputs.confirmed_survivors:
        reason = "critical_survivor_backlog"
    elif inputs.critical_status_units:
        reason = "critical_status_backlog"
    else:
        reason = "valuable_recent_map_delta"

    return RelayUtilityDecision(
        accepted=reason in {
            "critical_survivor_backlog",
            "critical_status_backlog",
            "valuable_recent_map_delta",
        },
        utility=utility,
        reason=reason,
        expected_transfer_steps=transfer_steps,
        ttl_reserve_steps=ttl_reserve,
        deliverable_payload_units=deliverable_units,
        critical_payload_units=critical_units,
        map_payload_units=map_units,
        backpressure_required=backpressure,
    )
