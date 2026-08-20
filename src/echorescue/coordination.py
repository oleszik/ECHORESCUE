from dataclasses import dataclass

from echorescue.mapping import OccupancyMap
from echorescue.models import Position
from echorescue.planning import astar


@dataclass(frozen=True, slots=True)
class FrontierAssignment:
    target: Position
    path: tuple[Position, ...]


@dataclass(frozen=True, slots=True)
class MovementConflict:
    reason: str
    drone_ids: tuple[str, ...]
    position: Position


def _path_avoiding_other_drones(
    drone_id: str,
    start: Position,
    target: Position,
    positions: dict[str, Position],
    occupancy_map: OccupancyMap,
) -> tuple[Position, ...] | None:
    blocked = {
        position
        for other_id, position in positions.items()
        if other_id != drone_id and position != start
    }

    def passable(position: Position) -> bool:
        return occupancy_map.is_known_free(position) and (
            position == start or position not in blocked
        )

    return astar(start, target, passable)


def assign_frontiers(
    positions: dict[str, Position],
    frontiers: tuple[Position, ...],
    occupancy_map: OccupancyMap,
    current_targets: dict[str, Position | None],
) -> dict[str, FrontierAssignment]:
    """Allocate distinct reachable frontiers with deterministic global ordering."""

    assignments: dict[str, FrontierAssignment] = {}
    claimed: set[Position] = set()
    frontier_set = set(frontiers)

    # Preserve still-valid, non-conflicting assignments first.
    for drone_id in sorted(positions):
        target = current_targets.get(drone_id)
        if target is None or target not in frontier_set or target in claimed:
            continue
        path = _path_avoiding_other_drones(
            drone_id, positions[drone_id], target, positions, occupancy_map
        )
        if path is not None:
            assignments[drone_id] = FrontierAssignment(target, path)
            claimed.add(target)

    candidates: list[tuple[int, int, int, str, Position, tuple[Position, ...]]] = []
    for drone_id in sorted(positions):
        if drone_id in assignments:
            continue
        for target in sorted(frontier_set - claimed):
            path = _path_avoiding_other_drones(
                drone_id, positions[drone_id], target, positions, occupancy_map
            )
            if path is not None:
                candidates.append(
                    (len(path), target.y, target.x, drone_id, target, path)
                )

    # Greedy global minimum: distance first, then target coordinates and ID.
    for _, _, _, drone_id, target, path in sorted(candidates):
        if drone_id in assignments or target in claimed:
            continue
        assignments[drone_id] = FrontierAssignment(target, path)
        claimed.add(target)
    return assignments


def resolve_movements(
    current: dict[str, Position],
    intended: dict[str, Position],
    base: Position,
) -> tuple[dict[str, Position], tuple[MovementConflict, ...]]:
    """Resolve simultaneous intents without vertex or edge-swap collisions."""

    resolved = dict(intended)
    conflicts: list[MovementConflict] = []
    drone_ids = sorted(intended)

    # An edge swap cannot safely grant either participant priority because the
    # lower-priority drone remains in the winner's destination cell.
    edge_blocked: set[str] = set()
    for index, first_id in enumerate(drone_ids):
        for second_id in drone_ids[index + 1 :]:
            if (
                intended[first_id] == current[second_id]
                and intended[second_id] == current[first_id]
                and current[first_id] != current[second_id]
            ):
                edge_blocked.update((first_id, second_id))
                conflicts.append(
                    MovementConflict(
                        "edge_swap",
                        (first_id, second_id),
                        intended[first_id],
                    )
                )
    for drone_id in edge_blocked:
        resolved[drone_id] = current[drone_id]

    # Routes are not allowed through another drone's current cell, even when
    # that drone also intends to move. The shared docking base is exempt.
    for drone_id in drone_ids:
        if drone_id in edge_blocked or intended[drone_id] == current[drone_id]:
            continue
        occupants = tuple(
            other_id
            for other_id in drone_ids
            if other_id != drone_id and intended[drone_id] == current[other_id]
        )
        if occupants and intended[drone_id] != base:
            resolved[drone_id] = current[drone_id]
            conflicts.append(
                MovementConflict(
                    "occupied_current_cell",
                    (drone_id,) + occupants,
                    intended[drone_id],
                )
            )

    destinations: dict[Position, list[str]] = {}
    for drone_id in drone_ids:
        if resolved[drone_id] != current[drone_id]:
            destinations.setdefault(resolved[drone_id], []).append(drone_id)
    for destination, contenders in sorted(destinations.items()):
        if destination == base or len(contenders) < 2:
            continue
        contenders.sort()
        winner = contenders[0]
        losers = contenders[1:]
        for loser in losers:
            resolved[loser] = current[loser]
        conflicts.append(
            MovementConflict(
                "vertex",
                tuple([winner] + losers),
                destination,
            )
        )
    return resolved, tuple(conflicts)
