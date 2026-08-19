from collections import deque
from dataclasses import dataclass
from random import Random

from echorescue.config import SimulationConfig
from echorescue.models import CellState, Position


@dataclass(frozen=True, slots=True)
class GridWorld:
    """Ground-truth grid. It must not be read by exploration logic."""

    width: int
    height: int
    base: Position
    walls: frozenset[Position]

    @classmethod
    def generate(cls, config: SimulationConfig) -> "GridWorld":
        base = Position(1, 1)
        boundary = {
            Position(x, y)
            for y in range(config.height)
            for x in range(config.width)
            if x in (0, config.width - 1) or y in (0, config.height - 1)
        }
        candidates = [
            Position(x, y)
            for y in range(1, config.height - 1)
            for x in range(1, config.width - 1)
            if Position(x, y) != base and abs(x - base.x) + abs(y - base.y) > 2
        ]
        rng = Random(config.seed)
        rng.shuffle(candidates)
        target = int(len(candidates) * config.obstacle_density)
        walls = set(boundary)

        # Add seeded obstacles only when all remaining interior free cells stay
        # connected. This guarantees that exploration is not judged on an
        # unreachable pocket created by the generator itself.
        for candidate in candidates:
            if len(walls) - len(boundary) >= target:
                break
            walls.add(candidate)
            if not cls._interior_is_connected(
                config.width, config.height, base, walls
            ):
                walls.remove(candidate)

        return cls(config.width, config.height, base, frozenset(walls))

    @staticmethod
    def _interior_is_connected(
        width: int, height: int, base: Position, walls: set[Position]
    ) -> bool:
        free_count = (width - 2) * (height - 2) - sum(
            1
            for wall in walls
            if 0 < wall.x < width - 1 and 0 < wall.y < height - 1
        )
        seen = {base}
        queue = deque([base])
        while queue:
            current = queue.popleft()
            for neighbor in current.neighbors():
                if (
                    0 < neighbor.x < width - 1
                    and 0 < neighbor.y < height - 1
                    and neighbor not in walls
                    and neighbor not in seen
                ):
                    seen.add(neighbor)
                    queue.append(neighbor)
        return len(seen) == free_count

    def contains(self, position: Position) -> bool:
        return 0 <= position.x < self.width and 0 <= position.y < self.height

    def cell_at(self, position: Position) -> CellState:
        if not self.contains(position) or position in self.walls:
            return CellState.OCCUPIED
        return CellState.FREE

    def is_free(self, position: Position) -> bool:
        return self.cell_at(position) is CellState.FREE

