from collections.abc import Callable
from dataclasses import dataclass

from echorescue.config import SimulationConfig
from echorescue.energy import Battery
from echorescue.environment import GridWorld
from echorescue.events import EventType, MissionEvent, MissionLog
from echorescue.mapping import OccupancyMap
from echorescue.models import Drone, DroneStatus, Position
from echorescue.planning import astar, path_to_nearest_frontier
from echorescue.sensors import DistanceSensor
from echorescue.survivors import SurvivorSensor


@dataclass(frozen=True, slots=True)
class SimulationResult:
    seed: int
    completed: bool
    termination_reason: str
    drone_status: DroneStatus
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
    battery_capacity: float
    energy_consumed: float
    energy_remaining: float
    energy_remaining_percent: float
    return_started_step: int | None
    returned_to_base: bool
    return_path_length: int
    energy_emergency: bool
    mission_success: bool
    mission_events: tuple[MissionEvent, ...]
    position_trace: tuple[Position, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "seed": self.seed,
            "completed": self.completed,
            "termination_reason": self.termination_reason,
            "drone_status": self.drone_status.value,
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
            "battery_capacity": round(self.battery_capacity, 6),
            "energy_consumed": round(self.energy_consumed, 6),
            "energy_remaining": round(self.energy_remaining, 6),
            "energy_remaining_percent": round(self.energy_remaining_percent, 3),
            "return_started_step": self.return_started_step,
            "returned_to_base": self.returned_to_base,
            "return_path_length": self.return_path_length,
            "energy_emergency": self.energy_emergency,
            "mission_success": self.mission_success,
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
        self.battery = Battery(
            capacity=config.battery_capacity,
            movement_cost=config.movement_energy_cost,
            sensor_cost=config.sensor_energy_cost,
        )
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
        self.active_frontier_target: Position | None = None
        self.current_return_path: tuple[Position, ...] = ()
        self.estimated_return_energy: float | None = None
        self.return_started_step: int | None = None
        self.return_path_length = 0
        self.energy_emergency = False
        self._exploration_complete = False
        self._sense()
        if not self.completed:
            self._refresh_return_estimate()

    @property
    def detected_survivors(self) -> frozenset[Position]:
        return frozenset(self._detected_survivors)

    @property
    def confirmed_survivors(self) -> frozenset[Position]:
        return frozenset(self._confirmed_survivors)

    @property
    def returned_to_base(self) -> bool:
        return (
            self.drone.status is DroneStatus.LANDED
            and self.drone.position == self.world.base
        )

    def _objectives_complete(self) -> bool:
        return len(self._confirmed_survivors) == len(self.world.survivors)

    def _known_return_path(
        self, origin: Position | None = None
    ) -> tuple[Position, ...] | None:
        return astar(
            origin or self.drone.position,
            self.world.base,
            self.occupancy_map.is_known_free,
        )

    def _refresh_return_estimate(self) -> tuple[Position, ...] | None:
        path = self._known_return_path()
        self.estimated_return_energy = (
            self.battery.estimate_path(len(path)) if path is not None else None
        )
        return path

    def _sense(self) -> None:
        if not self.battery.consume(self.config.sensor_energy_cost):
            self._fail_energy_emergency()
            return
        observations = self.sensor.observe(self.world, self.drone.position)
        self.occupancy_map.update(observations)
        self._observe_survivors()

    def _record_event(self, event_type: EventType) -> None:
        self.mission_log.record(
            MissionEvent(
                position=self.drone.position,
                step=self.steps,
                drone_id=self.drone.identifier,
                event_type=event_type,
                energy_remaining=self.battery.remaining,
            )
        )

    def _record_survivor_event(
        self, position: Position, event_type: EventType
    ) -> None:
        self.mission_log.record(
            MissionEvent(
                position=position,
                step=self.steps,
                drone_id=self.drone.identifier,
                event_type=event_type,
                energy_remaining=self.battery.remaining,
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

    def _fail_energy_emergency(self) -> None:
        self.drone.status = DroneStatus.ENERGY_EMERGENCY
        self.active_frontier_target = None
        self.current_return_path = ()
        self.energy_emergency = True
        self.completed = True
        self.termination_reason = "energy_emergency"
        self._record_event(EventType.ENERGY_EMERGENCY)

    def _fail_return_path_unavailable(self) -> None:
        self.drone.status = DroneStatus.RETURN_PATH_UNAVAILABLE
        self.active_frontier_target = None
        self.current_return_path = ()
        self.completed = True
        self.termination_reason = "return_path_unavailable"
        self._record_event(EventType.RETURN_PATH_UNAVAILABLE)

    def _land(self) -> None:
        self.drone.status = DroneStatus.LANDED
        self.active_frontier_target = None
        self.current_return_path = ()
        self.estimated_return_energy = 0.0
        self.completed = True
        self.termination_reason = (
            "exploration_complete" if self._exploration_complete else "returned_to_base"
        )
        self._record_event(EventType.BASE_REACHED)

    def _start_return(self, path: tuple[Position, ...]) -> bool:
        self.drone.status = DroneStatus.RETURN_HOME
        self.active_frontier_target = None
        self.current_return_path = path
        self.estimated_return_energy = self.battery.estimate_path(len(path))
        self.return_started_step = self.steps
        self._record_event(EventType.RETURN_STARTED)
        if self.drone.position == self.world.base:
            self._land()
            return False
        return True

    def _move(self, next_position: Position, returning: bool) -> bool:
        if not self.occupancy_map.is_known_free(next_position):
            self._fail_return_path_unavailable()
            return False
        if not self.world.is_free(next_position):
            self.collisions += 1
            self.completed = True
            self.termination_reason = "safety_violation"
            return False
        if not self.battery.consume(self.config.movement_energy_cost):
            self._fail_energy_emergency()
            return False

        self.drone.position = next_position
        self.drone.path_length += 1
        if returning:
            self.return_path_length += 1
        self.steps += 1
        self.position_trace.append(next_position)
        self._sense()
        return not self.completed

    def _step_explore(self) -> bool:
        return_path = self._refresh_return_estimate()
        if return_path is None:
            self._fail_return_path_unavailable()
            return False
        assert self.estimated_return_energy is not None
        required_now = self.estimated_return_energy + self.config.energy_safety_reserve
        if self.battery.remaining + 1e-9 < self.estimated_return_energy:
            self._fail_energy_emergency()
            return False
        if self.battery.remaining + 1e-9 < required_now:
            if self.drone.position == self.world.base and not self._objectives_complete():
                self._fail_energy_emergency()
                return False
            return self._start_return(return_path)

        frontier_path = path_to_nearest_frontier(
            self.drone.position,
            self.occupancy_map.frontiers(),
            self.occupancy_map.is_known_free,
        )
        if frontier_path is None:
            self._exploration_complete = True
            return self._start_return(return_path)
        if len(frontier_path) < 2:
            self.completed = True
            self.termination_reason = "stalled_frontier"
            return False

        self.active_frontier_target = frontier_path[-1]
        next_position = frontier_path[1]
        projected_return = self._known_return_path(next_position)
        if projected_return is None:
            return self._start_return(return_path)
        projected_remaining = self.battery.remaining - self.battery.movement_cycle_cost
        projected_required = (
            self.battery.estimate_path(len(projected_return))
            + self.config.energy_safety_reserve
        )
        if projected_remaining + 1e-9 < projected_required:
            if self.drone.position == self.world.base and not self._objectives_complete():
                self._fail_energy_emergency()
                return False
            return self._start_return(return_path)

        moved = self._move(next_position, returning=False)
        if moved:
            self._refresh_return_estimate()
        return moved

    def _step_return(self) -> bool:
        path = self._refresh_return_estimate()
        if path is None:
            self._fail_return_path_unavailable()
            return False
        if self.drone.position == self.world.base:
            self._land()
            return False
        assert self.estimated_return_energy is not None
        if self.battery.remaining + 1e-9 < self.estimated_return_energy:
            self._fail_energy_emergency()
            return False
        if self.current_return_path and path != self.current_return_path:
            self._record_event(EventType.RETURN_REPLANNED)
        self.current_return_path = path

        moved = self._move(path[1], returning=True)
        if not moved:
            return False
        self.current_return_path = path[1:]
        if self.drone.position == self.world.base:
            self._land()
            return False
        return True

    def step(self) -> bool:
        """Advance one deterministic state transition or safe movement."""

        if self.completed:
            return False
        if self.steps >= self.config.max_steps:
            self.completed = True
            self.active_frontier_target = None
            self.current_return_path = ()
            self.termination_reason = "max_steps"
            return False
        if self.drone.status is DroneStatus.RETURN_HOME:
            return self._step_return()
        if self.drone.status is DroneStatus.EXPLORE:
            return self._step_explore()
        return False

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
        survivor_recall = (
            len(self._confirmed_survivors) / survivors_total
            if survivors_total
            else 1.0
        )
        mission_success = (
            len(self._confirmed_survivors) == survivors_total
            and self.collisions == 0
            and self.returned_to_base
        )
        return SimulationResult(
            seed=self.config.seed,
            completed=self.completed,
            termination_reason=self.termination_reason,
            drone_status=self.drone.status,
            steps=self.steps,
            path_length=self.drone.path_length,
            known_cells=self.occupancy_map.known_cell_count,
            explored_percent=self.occupancy_map.explored_percent,
            collisions=self.collisions,
            survivors_total=survivors_total,
            survivors_detected=len(self._detected_survivors),
            survivors_confirmed=len(self._confirmed_survivors),
            survivor_recall=survivor_recall,
            time_to_first_detection=min(detection_steps) if detection_steps else None,
            battery_capacity=self.battery.capacity,
            energy_consumed=self.battery.consumed,
            energy_remaining=self.battery.remaining,
            energy_remaining_percent=self.battery.remaining_percent,
            return_started_step=self.return_started_step,
            returned_to_base=self.returned_to_base,
            return_path_length=self.return_path_length,
            energy_emergency=self.energy_emergency,
            mission_success=mission_success,
            mission_events=self.mission_log.events,
            position_trace=tuple(self.position_trace),
        )
