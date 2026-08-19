from echorescue.models import CellState, Position


class OccupancyMap:
    """The drone's discovered map, initialized entirely as unknown."""

    def __init__(self, width: int, height: int) -> None:
        self.width = width
        self.height = height
        self._cells = [CellState.UNKNOWN] * (width * height)

    def contains(self, position: Position) -> bool:
        return 0 <= position.x < self.width and 0 <= position.y < self.height

    def _index(self, position: Position) -> int:
        return position.y * self.width + position.x

    def cell_at(self, position: Position) -> CellState:
        if not self.contains(position):
            return CellState.OCCUPIED
        return self._cells[self._index(position)]

    def update(self, observations: dict[Position, CellState]) -> None:
        for position, state in observations.items():
            if self.contains(position):
                self._cells[self._index(position)] = state

    def is_known_free(self, position: Position) -> bool:
        return self.cell_at(position) is CellState.FREE

    def frontiers(self) -> tuple[Position, ...]:
        """Known-free cells bordering at least one in-bounds unknown cell."""

        result = []
        for y in range(self.height):
            for x in range(self.width):
                position = Position(x, y)
                if self.is_known_free(position) and any(
                    self.contains(neighbor)
                    and self.cell_at(neighbor) is CellState.UNKNOWN
                    for neighbor in position.neighbors()
                ):
                    result.append(position)
        return tuple(result)

    @property
    def known_cell_count(self) -> int:
        return sum(state is not CellState.UNKNOWN for state in self._cells)

    @property
    def explored_percent(self) -> float:
        return 100.0 * self.known_cell_count / len(self._cells)

