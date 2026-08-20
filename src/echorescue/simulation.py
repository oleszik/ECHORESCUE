from collections.abc import Callable
from dataclasses import dataclass

from echorescue.config import SimulationConfig
from echorescue.environment import GridWorld
from echorescue.events import EventType, MissionEvent, MissionLog
from echorescue.mapping import OccupancyMap
from echorescue.models import Drone, Position
from echorescue.planning import path_to_nearest_frontier
from echorescue.sensors import DistanceSensor
from echorescue.survivors import SurvivorSensor


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
    survivors_total: int
    survivors_detected: int
    survivors_confirmed: int
    survivor_recall: float
    time_to_first_detection: int | None
    mission_events: tuple[MissionEvent, ...]
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
            "survivors_total": self.survivors_total,
            "survivors_detected": self.survivors_detected,
            "survivors_confirmed": self.survivors_confirmed,
            "survivor_recall": round(self.survivor_recall, 3),
            "time_to_first_detection": self.time_to_first_detection,
            "mission_events": [event.to_dict() for event in self.mission_events],
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
        self.survivor_sensor = SurvivorSensor(config.survivor_sensor_range)
        self.mission_log = MissionLog()
        self._detected_survivors: set[Position] = set()
        self._confirmed_survivors: set[Position] = set()
        self._survivor_observations: dict[Position, int] = {}
        self._last_survivor_observation_step: dict[Position, int] = {}
        self.steps = 0
        self.collisions = 0
        self.completed = False
        self.termination_reason = "running"
        self.position_trace = [self.drone.position]
        self._sense()

    def _sense(self) -> None:
        observations = self.sensor.observe(self.world, self.drone.position)
        self.occupancy_map.update(observations)
        self._observe_survivors()

    @property
    def detected_survivors(self) -> frozenset[Position]:
        return frozenset(self._detected_survivors)

    @property
    def confirmed_survivors(self) -> frozenset[Position]:
        return frozenset(self._confirmed_survivors)

    def _record_survivor_event(
        self, position: Position, event_type: EventType
    ) -> None:
        self.mission_log.record(
            MissionEvent(
                position=position,
                step=self.steps,
                drone_id=self.drone.identifier,
                event_type=event_type,
            )
        )

    def _observe_survivors(self) -> None:
        visible_survivors = self.survivor_sensor.observe(
            self.world, self.drone.position
        )
        for position in visible_survivors:
            if self._last_survivor_observation_step.get(position) == self.steps:
                continue
            self._last_survivor_observation_step[position] = self.steps
            observations = self._survivor_observations.get(position, 0) + 1
            self._survivor_observations[position] = observations

            if position not in self._detected_survivors:
                self._detected_survivors.add(position)
                self._record_survivor_event(position, EventType.SURVIVOR_DETECTED)
            if (
                observations >= self.config.survivor_confirmation_observations
                and position not in self._confirmed_survivors
            ):
                self._confirmed_survivors.add(position)
                self._record_survivor_event(position, EventType.SURVIVOR_CONFIRMED)

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
        detection_steps = [
            event.step
            for event in self.mission_log.events
            if event.event_type is EventType.SURVIVOR_DETECTED
        ]
        survivors_total = len(self.world.survivors)
        return SimulationResult(
            seed=self.config.seed,
            completed=self.completed,
            termination_reason=self.termination_reason,
            steps=self.steps,
            path_length=self.drone.path_length,
            known_cells=self.occupancy_map.known_cell_count,
            explored_percent=self.occupancy_map.explored_percent,
            collisions=self.collisions,
            survivors_total=survivors_total,
            survivors_detected=len(self._detected_survivors),
            survivors_confirmed=len(self._confirmed_survivors),
            survivor_recall=(
                len(self._confirmed_survivors) / survivors_total
                if survivors_total
                else 1.0
            ),
            time_to_first_detection=min(detection_steps) if detection_steps else None,
            mission_events=self.mission_log.events,
            position_trace=tuple(self.position_trace),
        )
