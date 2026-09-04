"""Known-information-only planning primitives for N-agent Relay placement."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from math import hypot

from echorescue.communication import BASE_NODE_ID
from echorescue.knowledge import KnowledgeMap
from echorescue.models import CellState, Position
from echorescue.relay import known_radio_link


MULTI_RELAY_STRATEGIES = {"multi-relay", "predictive"}


@dataclass(frozen=True, slots=True)
class ConnectivityForecast:
    current_connected: bool
    current_hop_count: int | None
    projected_connected: tuple[bool, ...]
    projected_hop_counts: tuple[int | None, ...]
    expected_disconnection_step: int | None
    minimum_link_margin: float | None

    def to_dict(self) -> dict[str, object]:
        return {
            "current_connected": self.current_connected,
            "current_hop_count": self.current_hop_count,
            "projected_connected": list(self.projected_connected),
            "projected_hop_counts": list(self.projected_hop_counts),
            "expected_disconnection_step": self.expected_disconnection_step,
            "minimum_link_margin": (
                round(self.minimum_link_margin, 6)
                if self.minimum_link_margin is not None
                else None
            ),
        }


@dataclass(frozen=True, slots=True)
class RelayTargetTopology:
    targets: tuple[Position, ...]
    served_agent_ids: tuple[str, ...]
    relay_hops: int


@dataclass(frozen=True, slots=True)
class MultiRelayPlan:
    relay_ids: tuple[str, ...]
    targets: tuple[Position, ...]
    paths: tuple[tuple[Position, ...], ...]
    return_paths: tuple[tuple[Position, ...], ...]
    served_agent_ids: tuple[str, ...]
    score: int
    reason: str
    predictive: bool
    forecast_disconnect_step: int | None


@dataclass(slots=True)
class RelayDeployment:
    relay_id: str
    target: Position
    served_agent_ids: tuple[str, ...]
    upstream_relay_id: str | None
    downstream_relay_id: str | None
    activated_step: int
    minimum_release_step: int
    reason: str
    predictive: bool
    forecast_disconnect_step: int | None
    score: int
    payload_observed_steps: dict[Position, int] = field(default_factory=dict)
    task_id: str | None = None
    stable_release_steps: int = 0
    connected_service_steps: int = 0
    relayed_service_steps: int = 0
    payload_delivered: bool = False

    def to_dict(self) -> dict[str, object]:
        return {
            "relay_id": self.relay_id,
            "target": [self.target.x, self.target.y],
            "served_agent_ids": list(self.served_agent_ids),
            "upstream_relay_id": self.upstream_relay_id,
            "downstream_relay_id": self.downstream_relay_id,
            "activated_step": self.activated_step,
            "minimum_release_step": self.minimum_release_step,
            "reason": self.reason,
            "predictive": self.predictive,
            "forecast_disconnect_step": self.forecast_disconnect_step,
            "score": self.score,
            "task_id": self.task_id,
            "payload_cell_count": len(self.payload_observed_steps),
            "payload_delivered": self.payload_delivered,
            "connected_service_steps": self.connected_service_steps,
            "relayed_service_steps": self.relayed_service_steps,
        }


def _known_route(
    knowledge_map: KnowledgeMap,
    base: Position,
    positions: dict[str, Position],
    destination_id: str,
    max_range: int,
) -> tuple[str, ...]:
    nodes = {BASE_NODE_ID: base, **dict(sorted(positions.items()))}
    adjacency: dict[str, list[str]] = {node_id: [] for node_id in nodes}
    node_ids = sorted(nodes)
    for index, first_id in enumerate(node_ids):
        for second_id in node_ids[index + 1 :]:
            if known_radio_link(
                knowledge_map,
                nodes[first_id],
                nodes[second_id],
                max_range,
            ):
                adjacency[first_id].append(second_id)
                adjacency[second_id].append(first_id)
    queue: deque[tuple[str, tuple[str, ...]]] = deque(
        [(BASE_NODE_ID, (BASE_NODE_ID,))]
    )
    visited = {BASE_NODE_ID}
    while queue:
        node, route = queue.popleft()
        if node == destination_id:
            return route
        for neighbor in sorted(adjacency[node]):
            if neighbor in visited:
                continue
            visited.add(neighbor)
            queue.append((neighbor, route + (neighbor,)))
    return ()


def _route_margin(
    route: tuple[str, ...],
    base: Position,
    positions: dict[str, Position],
    max_range: int,
) -> float | None:
    if len(route) < 2:
        return None
    nodes = {BASE_NODE_ID: base, **positions}
    return min(
        max_range
        - hypot(
            nodes[first].x - nodes[second].x,
            nodes[first].y - nodes[second].y,
        )
        for first, second in zip(route, route[1:])
    )


def forecast_connectivity(
    knowledge_map: KnowledgeMap,
    *,
    base: Position,
    agent_positions: dict[str, Position],
    agent_id: str,
    planned_path: tuple[Position, ...],
    current_step: int,
    horizon: int,
    max_range: int,
) -> ConnectivityForecast:
    """Forecast one moving agent while holding other known nodes stationary.

    Every LOS decision is made from ``knowledge_map``. Unknown cells therefore
    make a forecast conservative and hidden or future world state is never read.
    """

    if horizon < 1:
        raise ValueError("prediction horizon must be positive")
    current = agent_positions[agent_id]
    if current in planned_path:
        start = planned_path.index(current)
        future = planned_path[start + 1 : start + 1 + horizon]
    else:
        future = planned_path[:horizon]
    projected = list(future)
    while len(projected) < horizon:
        projected.append(projected[-1] if projected else current)

    current_route = _known_route(
        knowledge_map, base, agent_positions, agent_id, max_range
    )
    connected: list[bool] = []
    hops: list[int | None] = []
    margins: list[float] = []
    expected_disconnect: int | None = None
    for offset, position in enumerate(projected, start=1):
        positions = dict(agent_positions)
        positions[agent_id] = position
        route = _known_route(
            knowledge_map, base, positions, agent_id, max_range
        )
        is_connected = bool(route)
        connected.append(is_connected)
        hops.append(len(route) - 1 if route else None)
        margin = _route_margin(route, base, positions, max_range)
        if margin is not None:
            margins.append(margin)
        if expected_disconnect is None and not is_connected:
            expected_disconnect = current_step + offset
    current_margin = _route_margin(
        current_route, base, agent_positions, max_range
    )
    if current_margin is not None:
        margins.append(current_margin)
    return ConnectivityForecast(
        current_connected=bool(current_route),
        current_hop_count=(len(current_route) - 1 if current_route else None),
        projected_connected=tuple(connected),
        projected_hop_counts=tuple(hops),
        expected_disconnection_step=expected_disconnect,
        minimum_link_margin=min(margins) if margins else None,
    )


def _candidate_positions(
    knowledge_map: KnowledgeMap,
    base: Position,
    primary: Position,
    *,
    max_relays: int,
    limit: int,
) -> tuple[Position, ...]:
    fractions = (
        (0.5,)
        if max_relays == 1
        else (1.0 / 3.0, 0.5, 2.0 / 3.0)
    )

    def ideal_distance(candidate: Position) -> float:
        return min(
            hypot(
                candidate.x - (base.x + (primary.x - base.x) * fraction),
                candidate.y - (base.y + (primary.y - base.y) * fraction),
            )
            for fraction in fractions
        )

    candidates = [
        position
        for position, record in knowledge_map.records
        if record.state is CellState.FREE
        and position not in {base, primary}
    ]
    return tuple(
        sorted(candidates, key=lambda item: (ideal_distance(item), item))[
            :limit
        ]
    )


def relay_target_topologies(
    knowledge_map: KnowledgeMap,
    *,
    base: Position,
    service_positions: dict[str, Position],
    primary_agent_id: str,
    max_relays: int,
    max_range: int,
    candidate_limit: int,
    topology_limit: int = 64,
) -> tuple[RelayTargetTopology, ...]:
    """Enumerate stable one- and two-Relay known-safe radio topologies."""

    if max_relays not in {1, 2}:
        raise ValueError("this bounded planner supports one or two Relays")
    primary = service_positions[primary_agent_id]
    candidates = _candidate_positions(
        knowledge_map,
        base,
        primary,
        max_relays=max_relays,
        limit=candidate_limit,
    )

    def served_by(targets: tuple[Position, ...]) -> tuple[str, ...]:
        served = []
        for agent_id, position in sorted(service_positions.items()):
            if any(
                known_radio_link(
                    knowledge_map, target, position, max_range
                )
                for target in targets
            ):
                served.append(agent_id)
        return tuple(served)

    rows: list[tuple[int, int, float, tuple[Position, ...], RelayTargetTopology]] = []
    for target in candidates:
        if not known_radio_link(knowledge_map, base, target, max_range):
            continue
        served = served_by((target,))
        if primary_agent_id not in served:
            continue
        geometry = hypot(target.x - primary.x, target.y - primary.y)
        topology = RelayTargetTopology((target,), served, 2)
        rows.append((1, -len(served), geometry, topology.targets, topology))

    if max_relays == 2:
        upstream = tuple(
            target
            for target in candidates
            if known_radio_link(knowledge_map, base, target, max_range)
        )
        downstream = tuple(
            target
            for target in candidates
            if known_radio_link(
                knowledge_map, target, primary, max_range
            )
        )
        for first in upstream:
            for second in downstream:
                if first == second or not known_radio_link(
                    knowledge_map, first, second, max_range
                ):
                    continue
                targets = (first, second)
                served = served_by(targets)
                if primary_agent_id not in served:
                    continue
                geometry = (
                    hypot(first.x - base.x, first.y - base.y)
                    + hypot(second.x - first.x, second.y - first.y)
                    + hypot(primary.x - second.x, primary.y - second.y)
                )
                topology = RelayTargetTopology(targets, served, 3)
                rows.append(
                    (2, -len(served), geometry, topology.targets, topology)
                )
    return tuple(row[-1] for row in sorted(rows)[:topology_limit])


def relay_candidate_score(
    *,
    travel_steps: int,
    task_interrupted: bool,
    role: str,
    consumed_energy: float,
    agents_helped: int,
    queue_units: int,
    predicted_steps_prevented: int,
) -> int:
    """Return the frozen, lower-is-better multi-Relay assignment score."""

    role_penalty = 0 if role == "RELAY" else (4 if role == "GENERALIST" else 8)
    return (
        travel_steps * 8
        + int(task_interrupted) * 12
        + role_penalty
        + int(consumed_energy // 20)
        - agents_helped * 50
        - min(queue_units, 144) // 4
        - predicted_steps_prevented * 15
    )
