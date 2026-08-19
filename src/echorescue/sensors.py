from dataclasses import dataclass

from echorescue.environment import GridWorld
from echorescue.models import CellState, Position


@dataclass(frozen=True, slots=True)
class DistanceSensor:
    """Noise-free Phase 1 range sensor with four cardinal rays."""

    max_range: int

    def observe(
        self, world: GridWorld, origin: Position
    ) -> dict[Position, CellState]:
        if not world.is_free(origin):
            raise ValueError("sensor origin must be a free cell")

        observations = {origin: CellState.FREE}
        directions = ((1, 0), (0, 1), (-1, 0), (0, -1))
        for dx, dy in directions:
            for distance in range(1, self.max_range + 1):
                position = Position(origin.x + dx * distance, origin.y + dy * distance)
                state = world.cell_at(position)
                observations[position] = state
                if state is CellState.OCCUPIED:
                    break
        return observations

