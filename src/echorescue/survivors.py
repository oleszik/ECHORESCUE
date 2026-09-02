from dataclasses import dataclass
from hashlib import sha256

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
        return self.observe_report(
            world,
            origin,
            smoke_profile="off",
            seed=0,
            step=0,
            observer_id="sensor",
        ).visible_survivors

    def observe_report(
        self,
        world: GridWorld,
        origin: Position,
        *,
        smoke_profile: str,
        seed: int,
        step: int,
        observer_id: str,
    ) -> "SurvivorObservationReport":
        if not world.is_free(origin):
            raise ValueError("survivor sensor origin must be a free cell")

        visible = []
        attempts = 0
        degraded_attempts = 0
        successful_smoke_attempts = 0
        maximum_exposure = 0.0
        for survivor in sorted(world.survivors):
            distance_squared = (
                (survivor.x - origin.x) ** 2 + (survivor.y - origin.y) ** 2
            )
            if distance_squared > self.max_range**2:
                continue
            intervening_cells = _line_cells(origin, survivor)[1:-1]
            if any(not world.is_free(cell) for cell in intervening_cells):
                continue
            attempts += 1
            line = _line_cells(origin, survivor)
            exposure = (
                sum(world.smoke.density_at(cell) for cell in line) / len(line)
                if smoke_profile != "off"
                else 0.0
            )
            maximum_exposure = max(maximum_exposure, exposure)
            visibility = max(0.05, 1.0 - exposure)
            effective_range = int(self.max_range * visibility)
            if distance_squared > effective_range**2:
                degraded_attempts += 1
                continue
            if exposure > 0.0:
                identity = "|".join(
                    (
                        str(seed),
                        smoke_profile,
                        observer_id,
                        str(step),
                        f"{origin.x},{origin.y}",
                        f"{survivor.x},{survivor.y}",
                    )
                )
                roll = int.from_bytes(
                    sha256(identity.encode("utf-8")).digest()[:8], "big"
                ) / 2**64
                if roll >= visibility:
                    degraded_attempts += 1
                    continue
                successful_smoke_attempts += 1
            visible.append(survivor)
        return SurvivorObservationReport(
            visible_survivors=tuple(visible),
            attempts=attempts,
            successful_observations=len(visible),
            degraded_attempts=degraded_attempts,
            successful_smoke_observations=successful_smoke_attempts,
            maximum_smoke_exposure=maximum_exposure,
        )


@dataclass(frozen=True, slots=True)
class SurvivorObservationReport:
    visible_survivors: tuple[Position, ...]
    attempts: int
    successful_observations: int
    degraded_attempts: int
    successful_smoke_observations: int
    maximum_smoke_exposure: float
