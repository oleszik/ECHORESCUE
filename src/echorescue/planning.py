from collections.abc import Callable
from heapq import heappop, heappush

from echorescue.models import Position


def _manhattan(first: Position, second: Position) -> int:
    return abs(first.x - second.x) + abs(first.y - second.y)


def astar(
    start: Position,
    goal: Position,
    passable: Callable[[Position], bool],
) -> tuple[Position, ...] | None:
    """Return a deterministic shortest path, including start and goal."""

    if not passable(start) or not passable(goal):
        return None
    frontier: list[tuple[int, int, Position]] = []
    heappush(frontier, (_manhattan(start, goal), 0, start))
    came_from: dict[Position, Position] = {}
    best_cost = {start: 0}

    while frontier:
        _, cost, current = heappop(frontier)
        if cost != best_cost.get(current):
            continue
        if current == goal:
            path = [current]
            while current != start:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return tuple(path)

        for neighbor in current.neighbors():
            if not passable(neighbor):
                continue
            next_cost = cost + 1
            if next_cost < best_cost.get(neighbor, 10**9):
                best_cost[neighbor] = next_cost
                came_from[neighbor] = current
                priority = next_cost + _manhattan(neighbor, goal)
                heappush(frontier, (priority, next_cost, neighbor))
    return None


def path_to_nearest_frontier(
    start: Position,
    frontiers: tuple[Position, ...],
    passable: Callable[[Position], bool],
) -> tuple[Position, ...] | None:
    candidates = []
    for frontier in frontiers:
        path = astar(start, frontier, passable)
        if path is not None:
            candidates.append((len(path), frontier.y, frontier.x, path))
    if not candidates:
        return None
    return min(candidates)[-1]

