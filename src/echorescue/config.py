from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class SimulationConfig:
    """Central configuration for a deterministic Phase 1 mission."""

    width: int = 21
    height: int = 13
    seed: int = 7
    obstacle_density: float = 0.08
    sensor_range: int = 4
    survivor_count: int = 3
    survivor_sensor_range: int = 3
    survivor_confirmation_observations: int = 2
    battery_capacity: float = 220.0
    movement_energy_cost: float = 1.0
    sensor_energy_cost: float = 0.05
    energy_safety_reserve: float = 20.0
    max_steps: int = 1_000

    def __post_init__(self) -> None:
        if self.width < 7 or self.height < 7:
            raise ValueError("width and height must both be at least 7")
        if not 0.0 <= self.obstacle_density <= 0.35:
            raise ValueError("obstacle_density must be between 0.0 and 0.35")
        if self.sensor_range < 1:
            raise ValueError("sensor_range must be positive")
        interior_cells = (self.width - 2) * (self.height - 2)
        if not 0 <= self.survivor_count < interior_cells:
            raise ValueError("survivor_count must fit within the interior grid")
        if self.survivor_sensor_range < 1:
            raise ValueError("survivor_sensor_range must be positive")
        if self.survivor_confirmation_observations < 2:
            raise ValueError("survivor confirmation requires at least two observations")
        if self.battery_capacity <= 0:
            raise ValueError("battery_capacity must be positive")
        if self.movement_energy_cost <= 0:
            raise ValueError("movement_energy_cost must be positive")
        if self.sensor_energy_cost < 0:
            raise ValueError("sensor_energy_cost must not be negative")
        if not 0 <= self.energy_safety_reserve < self.battery_capacity:
            raise ValueError("energy_safety_reserve must be below battery_capacity")
        if self.max_steps < 1:
            raise ValueError("max_steps must be positive")
