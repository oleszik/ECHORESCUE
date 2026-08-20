from dataclasses import dataclass

from echorescue.environment import GridWorld
from echorescue.models import Position


def _line_cells(start: Position, end: Position) -> tuple[Position, ...]:
    """Return a conservative supercover line between two grid cells."""

    x, y = start.x, start.y
    dx = end.x - start.x
    dy = end.y - start.y
    nx = abs(dx)
    ny = abs(dy)
    sign_x = 0 if dx == 0 else (1 if dx > 0 else -1)
    sign_y = 0 if dy == 0 else (1 if dy > 0 else -1)
    ix = 0
    iy = 0
    cells = [start]

    while ix < nx or iy < ny:
        decision = (1 + 2 * ix) * ny - (1 + 2 * iy) * nx
        if decision == 0:
            # At a corner crossing, both adjacent cells are part of the
            # conservative visibility corridor. Either may block the ray.
            cells.append(Position(x + sign_x, y))
            cells.append(Position(x, y + sign_y))
            x += sign_x
            y += sign_y
            ix += 1
            iy += 1
        elif decision < 0:
            x += sign_x
            ix += 1
        else:
            y += sign_y
            iy += 1
        cells.append(Position(x, y))
    return tuple(dict.fromkeys(cells))


@dataclass(frozen=True, slots=True)
class SurvivorSensor:
    """Ground-truth adapter exposing only visible survivor positions."""

    max_range: int

    def observe(self, world: GridWorld, origin: Position) -> tuple[Position, ...]:
        if not world.is_free(origin):
            raise ValueError("survivor sensor origin must be a free cell")

        visible = []
        for survivor in sorted(world.survivors):
            distance_squared = (
                (survivor.x - origin.x) ** 2 + (survivor.y - origin.y) ** 2
            )
            if distance_squared > self.max_range**2:
                continue
            intervening_cells = _line_cells(origin, survivor)[1:-1]
            if any(not world.is_free(cell) for cell in intervening_cells):
                continue
            visible.append(survivor)
        return tuple(visible)

