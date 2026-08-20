from dataclasses import dataclass, field


@dataclass(slots=True)
class Battery:
    """Deterministic energy store measured in abstract energy units."""

    capacity: float
    movement_cost: float
    sensor_cost: float = 0.0
    remaining: float = field(init=False)

    def __post_init__(self) -> None:
        if self.capacity <= 0:
            raise ValueError("battery capacity must be positive")
        if self.movement_cost <= 0:
            raise ValueError("movement cost must be positive")
        if self.sensor_cost < 0:
            raise ValueError("sensor cost must not be negative")
        self.remaining = self.capacity

    @property
    def consumed(self) -> float:
        return self.capacity - self.remaining

    @property
    def remaining_percent(self) -> float:
        return 100.0 * self.remaining / self.capacity

    @property
    def movement_cycle_cost(self) -> float:
        """One movement followed by one sensor observation."""

        return self.movement_cost + self.sensor_cost

    def consume(self, amount: float) -> bool:
        if amount < 0:
            raise ValueError("energy consumption must not be negative")
        if self.remaining + 1e-9 < amount:
            return False
        self.remaining = max(0.0, self.remaining - amount)
        return True

    def estimate_path(self, position_count: int) -> float:
        moves = max(0, position_count - 1)
        return moves * self.movement_cycle_cost
