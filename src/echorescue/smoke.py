from dataclasses import dataclass
from random import Random

from echorescue.models import Position


@dataclass(frozen=True, slots=True)
class SmokeProfileDefinition:
    density: float
    zone_count: int
    radius: int


SMOKE_PROFILES: dict[str, SmokeProfileDefinition] = {
    "off": SmokeProfileDefinition(density=0.0, zone_count=0, radius=0),
    "moderate": SmokeProfileDefinition(density=0.65, zone_count=3, radius=3),
}


@dataclass(frozen=True, order=True, slots=True)
class SmokeCell:
    position: Position
    density: float


@dataclass(frozen=True, slots=True)
class SmokeField:
    """Deterministic ground-truth smoke density over free grid cells."""

    profile: str = "off"
    cells: tuple[SmokeCell, ...] = ()

    @classmethod
    def generate(
        cls,
        *,
        profile: str,
        seed: int,
        width: int,
        height: int,
        walls: frozenset[Position] | set[Position],
        base: Position,
    ) -> "SmokeField":
        definition = SMOKE_PROFILES[profile]
        if profile == "off":
            return cls()
        candidates = [
            Position(x, y)
            for y in range(1, height - 1)
            for x in range(1, width - 1)
            if Position(x, y) not in walls
            and abs(x - base.x) + abs(y - base.y) > definition.radius
        ]
        rng = Random(seed + 20_000_033)
        rng.shuffle(candidates)
        centers = candidates[: definition.zone_count]
        densities: dict[Position, float] = {}
        for center in centers:
            for y in range(
                max(1, center.y - definition.radius),
                min(height - 1, center.y + definition.radius + 1),
            ):
                for x in range(
                    max(1, center.x - definition.radius),
                    min(width - 1, center.x + definition.radius + 1),
                ):
                    position = Position(x, y)
                    if position in walls:
                        continue
                    distance = abs(position.x - center.x) + abs(
                        position.y - center.y
                    )
                    if distance > definition.radius:
                        continue
                    densities[position] = max(
                        densities.get(position, 0.0), definition.density
                    )
        return cls(
            profile=profile,
            cells=tuple(
                SmokeCell(position, density)
                for position, density in sorted(densities.items())
            ),
        )

    def density_at(self, position: Position) -> float:
        for cell in self.cells:
            if cell.position == position:
                return cell.density
        return 0.0

    @property
    def maximum_density(self) -> float:
        return max((cell.density for cell in self.cells), default=0.0)

    def to_debug_rows(self, width: int, height: int) -> list[list[float]]:
        densities = {cell.position: cell.density for cell in self.cells}
        return [
            [round(densities.get(Position(x, y), 0.0), 3) for x in range(width)]
            for y in range(height)
        ]
