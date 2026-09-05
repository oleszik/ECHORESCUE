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



def uncertain_observations(observations: dict[Position, CellState], *,
                           seed: int, agent_id: str, step: int, profile: str,
                           floor: int = 0) -> dict[Position, CellState]:
    """Corrupt range returns only; ground truth is confined to sensor generation."""
    from echorescue.perception import deterministic_unit
    from echorescue.probabilistic import UNCERTAINTY_PROFILES
    parameters = UNCERTAINTY_PROFILES[profile]
    result = {}
    for position, state in observations.items():
        draw = deterministic_unit("occupancy", seed, agent_id, step, floor, position.x, position.y)
        occupied = draw < (parameters.occupancy_detection if state is CellState.OCCUPIED
                           else parameters.occupancy_false_positive)
        result[position] = CellState.OCCUPIED if occupied else CellState.FREE
    return result
