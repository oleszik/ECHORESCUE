from collections import deque
from dataclasses import dataclass
from hashlib import sha256
from typing import Protocol

from echorescue.models import CellState, Position
from echorescue.planning import astar


DYNAMIC_OBSTACLE_PROFILES = {"off", "moderate"}


class StaticWorldView(Protocol):
    @property
    def width(self) -> int: ...

    @property
    def height(self) -> int: ...

    @property
    def base(self) -> Position: ...

    @property
    def walls(self) -> frozenset[Position]: ...

    @property
    def survivors(self) -> frozenset[Position]: ...

    def cell_at(self, position: Position) -> CellState: ...


@dataclass(frozen=True, slots=True)
class DynamicObstacleEvent:
    position: Position
    step: int
    cause: str


def _stable_unit(*parts: object) -> float:
    payload = "|".join(str(part) for part in parts)
    digest = sha256(payload.encode("utf-8")).digest()
    return int.from_bytes(digest[:8], "big") / 2**64


def _remains_connected(
    world: StaticWorldView,
    candidate: Position,
    already_selected: set[Position],
) -> bool:
    blocked = set(world.walls) | already_selected | {candidate}
    free = {
        Position(x, y)
        for y in range(1, world.height - 1)
        for x in range(1, world.width - 1)
        if Position(x, y) not in blocked
    }
    if world.base not in free:
        return False
    seen = {world.base}
    queue = deque([world.base])
    while queue:
        for neighbor in queue.popleft().neighbors():
            if neighbor in free and neighbor not in seen:
                seen.add(neighbor)
                queue.append(neighbor)
    return seen == free


def moderate_schedule(
    world: StaticWorldView,
    *,
    seed: int,
    excluded_positions: frozenset[Position] = frozenset(),
) -> tuple[DynamicObstacleEvent, ...]:
    """Choose two map-derived, non-adversarial closure events."""

    free_cells = tuple(
        Position(x, y)
        for y in range(1, world.height - 1)
        for x in range(1, world.width - 1)
        if world.cell_at(Position(x, y)) is CellState.FREE
    )
    centrality = {position: 0 for position in free_cells}
    for destination in free_cells:
        path = astar(
            world.base,
            destination,
            lambda position: world.cell_at(position) is CellState.FREE,
        )
        if path is not None:
            for position in path[1:-1]:
                centrality[position] += 1
    ranked = sorted(
        free_cells,
        key=lambda position: (
            -centrality[position],
            _stable_unit("dynamic-obstacle", seed, position.x, position.y),
            position,
        ),
    )
    selected: list[Position] = []
    selected_set: set[Position] = set()
    for candidate in ranked:
        if (
            candidate == world.base
            or candidate in world.survivors
            or candidate in excluded_positions
            or abs(candidate.x - world.base.x)
            + abs(candidate.y - world.base.y)
            < 4
            or any(
                abs(candidate.x - other.x) + abs(candidate.y - other.y) < 4
                for other in selected
            )
            or not _remains_connected(world, candidate, selected_set)
        ):
            continue
        selected.append(candidate)
        selected_set.add(candidate)
        if len(selected) == 2:
            break
    return tuple(
        DynamicObstacleEvent(position, step, "structural_debris")
        for position, step in zip(selected, (12, 30))
    )
