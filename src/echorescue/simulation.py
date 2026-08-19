from collections.abc import Callable
from dataclasses import dataclass

from echorescue.config import SimulationConfig
from echorescue.environment import GridWorld
from echorescue.mapping import OccupancyMap
from echorescue.models import Drone, Position
from echorescue.planning import path_to_nearest_frontier
from echorescue.sensors import DistanceSensor


@dataclass(frozen=True, slots=True)
class SimulationResult:
    seed: int
    completed: bool
    termination_reason: str
    steps: int
    path_length: int
    known_cells: int
    explored_percent: float
    collisions: int
    position_trace: tuple[Position, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "seed": self.seed,
            "completed": self.completed,
            "termination_reason": self.termination_reason,
            "steps": self.steps,
            "path_length": self.path_length,
            "known_cells": self.known_cells,
            "explored_percent": round(self.explored_percent, 3),
            "collisions": self.collisions,
            "position_trace": [[position.x, position.y] for position in self.position_trace],
        }


FrameCallback = Callable[["Simulation"], None]


class Simulation:
    """Deterministic single-drone Sense-Plan-Act loop."""

    def __init__(self, config: SimulationConfig) -> None:
        self.config = config
        self.world = GridWorld.generate(config)
        self.occupancy_map = OccupancyMap(config.width, config.height)
        self.drone = Drone(self.world.base)
        self.sensor = DistanceSensor(config.sensor_range)
        self.steps = 0
        self.collisions = 0
        self.completed = False
        self.termination_reason = "running"
        self.position_trace = [self.drone.position]
        self._sense()

    def _sense(self) -> None:
        observations = self.sensor.observe(self.world, self.drone.position)
        self.occupancy_map.update(observations)

    def step(self) -> bool:
        """Advance one safe movement. Return False when the mission is done."""

        if self.completed:
            return False
        if self.steps >= self.config.max_steps:
            self.completed = True
            self.termination_reason = "max_steps"
            return False

        path = path_to_nearest_frontier(
            self.drone.position,
            self.occupancy_map.frontiers(),
            self.occupancy_map.is_known_free,
        )
        if path is None:
            self.completed = True
            self.termination_reason = "exploration_complete"
            return False
        if len(path) < 2:
            # A sensor observation always resolves the current cell's immediate
            # cardinal neighborhood. This guard makes an invariant violation
            # explicit instead of silently looping forever.
            self.completed = True
            self.termination_reason = "stalled_frontier"
            return False

        next_position = path[1]
        if not self.world.is_free(next_position):
            self.collisions += 1
            self.completed = True
            self.termination_reason = "safety_violation"
            return False

        self.drone.position = next_position
        self.drone.path_length += 1
        self.steps += 1
        self.position_trace.append(next_position)
        self._sense()
        return True

    def run(self, on_frame: FrameCallback | None = None) -> SimulationResult:
        if on_frame is not None:
            on_frame(self)
        while self.step():
            if on_frame is not None:
                on_frame(self)
        if on_frame is not None:
            on_frame(self)
        return self.result()

    def result(self) -> SimulationResult:
        return SimulationResult(
            seed=self.config.seed,
            completed=self.completed,
            termination_reason=self.termination_reason,
            steps=self.steps,
            path_length=self.drone.path_length,
            known_cells=self.occupancy_map.known_cell_count,
            explored_percent=self.occupancy_map.explored_percent,
            collisions=self.collisions,
            position_trace=tuple(self.position_trace),
        )

