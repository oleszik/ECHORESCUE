from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class SimulationConfig:
    """Central configuration for a deterministic Phase 1 mission."""

    width: int = 21
    height: int = 13
    seed: int = 7
    obstacle_density: float = 0.08
    sensor_range: int = 4
    max_steps: int = 1_000

    def __post_init__(self) -> None:
        if self.width < 7 or self.height < 7:
            raise ValueError("width and height must both be at least 7")
        if not 0.0 <= self.obstacle_density <= 0.35:
            raise ValueError("obstacle_density must be between 0.0 and 0.35")
        if self.sensor_range < 1:
            raise ValueError("sensor_range must be positive")
        if self.max_steps < 1:
            raise ValueError("max_steps must be positive")

