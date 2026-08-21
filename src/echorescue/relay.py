from dataclasses import dataclass

from echorescue.communication import _line_cells
from echorescue.knowledge import KnowledgeMap
from echorescue.models import CellState, Position
from echorescue.planning import astar


@dataclass(frozen=True, slots=True)
class RelayPlan:
    position: Position
    path: tuple[Position, ...]
    return_path: tuple[Position, ...]
    movement_cost: int
    energy_required: float


def known_radio_link(
    knowledge_map: KnowledgeMap,
    first: Position,
    second: Position,
    max_range: int,
) -> bool:
    distance_squared = (first.x - second.x) ** 2 + (first.y - second.y) ** 2
    if distance_squared > max_range**2:
        return False
    return all(
        knowledge_map.cell_at(position) is CellState.FREE
        for position in _line_cells(first, second)[1:-1]
    )


def select_relay_plan(
    knowledge_map: KnowledgeMap,
    origin: Position,
    base: Position,
    scout_position: Position,
    *,
    max_range: int,
    energy_remaining: float,
    movement_cycle_cost: float,
    safety_reserve: float,
    energy_margin: float,
    blocked: frozenset[Position] = frozenset(),
) -> RelayPlan | None:
    def passable(position: Position) -> bool:
        return knowledge_map.is_known_free(position) and (
            position == origin or position not in blocked
        )

    plans = []
    for candidate, record in knowledge_map.records:
        if record.state is not CellState.FREE or candidate in blocked:
            continue
        if not known_radio_link(
            knowledge_map, candidate, base, max_range
        ) or not known_radio_link(
            knowledge_map, candidate, scout_position, max_range
        ):
            continue
        path = astar(origin, candidate, passable)
        return_path = astar(candidate, base, knowledge_map.is_known_free)
        if path is None or return_path is None:
            continue
        movement_cost = max(0, len(path) - 1)
        return_moves = max(0, len(return_path) - 1)
        energy_required = (
            (movement_cost + return_moves) * movement_cycle_cost
            + safety_reserve
            + energy_margin
        )
        if energy_remaining + 1e-9 < energy_required:
            continue
        plans.append(
            (
                movement_cost,
                return_moves,
                candidate.y,
                candidate.x,
                RelayPlan(
                    candidate,
                    path,
                    return_path,
                    movement_cost,
                    energy_required,
                ),
            )
        )
    return min(plans)[-1] if plans else None
