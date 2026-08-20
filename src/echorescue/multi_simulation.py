from collections.abc import Callable
from dataclasses import dataclass, field

from echorescue.config import SimulationConfig
from echorescue.coordination import (
    FrontierAssignment,
    assign_frontiers,
    resolve_movements,
)
from echorescue.energy import Battery
from echorescue.environment import GridWorld
from echorescue.events import EventType, MissionEvent, MissionLog
from echorescue.mapping import OccupancyMap
from echorescue.models import Drone, DroneStatus, Position
from echorescue.planning import astar
from echorescue.sensors import DistanceSensor
from echorescue.survivors import SurvivorSensor


TERMINAL_STATUSES = {
    DroneStatus.LANDED,
    DroneStatus.ENERGY_EMERGENCY,
    DroneStatus.RETURN_PATH_UNAVAILABLE,
    DroneStatus.FAILED,
}


@dataclass(slots=True)
class DroneRuntime:
    drone: Drone
    battery: Battery
    active_frontier_target: Position | None = None
    planned_path: tuple[Position, ...] = ()
    current_return_path: tuple[Position, ...] = ()
    estimated_return_energy: float | None = None
    return_started_step: int | None = None
    return_path_length: int = 0
    energy_emergency: bool = False
    wait_steps: int = 0
    frontier_assignments: int = 0
    position_trace: list[Position] = field(default_factory=list)
    survivor_observations: dict[Position, int] = field(default_factory=dict)
    last_survivor_observation_step: dict[Position, int] = field(
        default_factory=dict
    )
    return_replan_required: bool = False

    @property
    def terminal(self) -> bool:
        return self.drone.status in TERMINAL_STATUSES


@dataclass(frozen=True, slots=True)
class MultiSimulationResult:
    seed: int
    completed: bool
    termination_reason: str
    steps: int
    known_cells: int
    explored_percent: float
    collisions: int
    survivors_total: int
    survivors_detected: int
    survivors_confirmed: int
    survivor_recall: float
    time_to_first_detection: int | None
    drones_total: int
    drones_returned: int
    drones_failed: int
    drone_drone_collisions: int
    movement_conflicts: int
    wait_steps_by_drone: dict[str, int]
    path_length_by_drone: dict[str, int]
    energy_remaining_by_drone: dict[str, float]
    frontier_assignments_by_drone: dict[str, int]
    drone_status_by_drone: dict[str, DroneStatus]
    return_started_step_by_drone: dict[str, int | None]
    return_path_length_by_drone: dict[str, int]
    position_trace_by_drone: dict[str, tuple[Position, ...]]
    duplicate_exploration_ratio: float
    mission_success: bool
    mission_events: tuple[MissionEvent, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "seed": self.seed,
            "completed": self.completed,
            "termination_reason": self.termination_reason,
            "steps": self.steps,
            "known_cells": self.known_cells,
            "explored_percent": round(self.explored_percent, 3),
            "collisions": self.collisions,
            "survivors_total": self.survivors_total,
            "survivors_detected": self.survivors_detected,
            "survivors_confirmed": self.survivors_confirmed,
            "survivor_recall": round(self.survivor_recall, 3),
            "time_to_first_detection": self.time_to_first_detection,
            "drones_total": self.drones_total,
            "drones_returned": self.drones_returned,
            "drones_failed": self.drones_failed,
            "drone_drone_collisions": self.drone_drone_collisions,
            "movement_conflicts": self.movement_conflicts,
            "wait_steps_by_drone": dict(self.wait_steps_by_drone),
            "path_length_by_drone": dict(self.path_length_by_drone),
            "energy_remaining_by_drone": {
                drone_id: round(energy, 6)
                for drone_id, energy in self.energy_remaining_by_drone.items()
            },
            "frontier_assignments_by_drone": dict(
                self.frontier_assignments_by_drone
            ),
            "drone_status_by_drone": {
                drone_id: status.value
                for drone_id, status in self.drone_status_by_drone.items()
            },
            "return_started_step_by_drone": dict(
                self.return_started_step_by_drone
            ),
            "return_path_length_by_drone": dict(self.return_path_length_by_drone),
            "position_trace_by_drone": {
                drone_id: [[position.x, position.y] for position in trace]
                for drone_id, trace in self.position_trace_by_drone.items()
            },
            "duplicate_exploration_ratio": round(
                self.duplicate_exploration_ratio, 6
            ),
            "mission_success": self.mission_success,
            "mission_events": [event.to_dict() for event in self.mission_events],
        }


FrameCallback = Callable[["MultiDroneSimulation"], None]


class MultiDroneSimulation:
    """Synchronous deterministic two-drone simulation over shared knowledge."""

    def __init__(self, config: SimulationConfig) -> None:
        if config.drone_count != 2:
            raise ValueError("MultiDroneSimulation requires drone_count=2")
        self.config = config
        self.world = GridWorld.generate(config)
        self.occupancy_map = OccupancyMap(config.width, config.height)
        self.sensor = DistanceSensor(config.sensor_range)
        self.survivor_sensor = SurvivorSensor(config.survivor_sensor_range)
        self.mission_log = MissionLog()
        self.steps = 0
        self.collisions = 0
        self.drone_drone_collisions = 0
        self.movement_conflicts = 0
        self.completed = False
        self.termination_reason = "running"
        self._exploration_complete = False
        self._detected_survivors: set[Position] = set()
        self._confirmed_survivors: set[Position] = set()
        self._visited_by_cell: dict[Position, set[str]] = {}

        starts = self._resolve_start_positions()
        self.runtimes: dict[str, DroneRuntime] = {}
        for index, position in enumerate(starts, start=1):
            drone_id = f"drone-{index}"
            runtime = DroneRuntime(
                drone=Drone(position=position, identifier=drone_id),
                battery=Battery(
                    capacity=config.battery_capacity,
                    movement_cost=config.movement_energy_cost,
                    sensor_cost=config.sensor_energy_cost,
                ),
                position_trace=[position],
            )
            self.runtimes[drone_id] = runtime
            if position != self.world.base:
                self._visited_by_cell.setdefault(position, set()).add(drone_id)

        for runtime in self._ordered_runtimes():
            self._sense(runtime)
            if not runtime.terminal:
                self._refresh_return_estimate(runtime)
        self._update_completion()

    @property
    def drones(self) -> tuple[Drone, ...]:
        return tuple(runtime.drone for runtime in self._ordered_runtimes())

    @property
    def confirmed_survivors(self) -> frozenset[Position]:
        return frozenset(self._confirmed_survivors)

    @property
    def detected_survivors(self) -> frozenset[Position]:
        return frozenset(self._detected_survivors)

    def _ordered_runtimes(self) -> tuple[DroneRuntime, ...]:
        return tuple(self.runtimes[key] for key in sorted(self.runtimes))

    def _resolve_start_positions(self) -> tuple[Position, Position]:
        if self.config.drone_start_positions is None:
            starts = (self.world.base, Position(self.world.base.x + 1, self.world.base.y))
        else:
            starts = tuple(Position(x, y) for x, y in self.config.drone_start_positions)
        if len(starts) != 2 or any(not self.world.is_free(position) for position in starts):
            raise ValueError("both drone start positions must be free cells")
        if starts[0] == starts[1] and starts[0] != self.world.base:
            raise ValueError("only the shared docking base may have matching starts")
        return starts[0], starts[1]

    def _other_blockers(self, runtime: DroneRuntime) -> set[Position]:
        return {
            other.drone.position
            for other in self._ordered_runtimes()
            if (
                other.drone.identifier != runtime.drone.identifier
                and other.drone.status is not DroneStatus.LANDED
                and other.drone.position != self.world.base
            )
        }

    def _known_return_path(
        self,
        runtime: DroneRuntime,
        origin: Position | None = None,
        avoid_other_drones: bool = True,
    ) -> tuple[Position, ...] | None:
        start = origin or runtime.drone.position
        blocked = self._other_blockers(runtime) if avoid_other_drones else set()

        def passable(position: Position) -> bool:
            if position == self.world.base:
                return self.occupancy_map.is_known_free(position)
            return self.occupancy_map.is_known_free(position) and (
                position == start or position not in blocked
            )

        return astar(start, self.world.base, passable)

    def _refresh_return_estimate(
        self, runtime: DroneRuntime
    ) -> tuple[Position, ...] | None:
        path = self._known_return_path(runtime)
        runtime.estimated_return_energy = (
            runtime.battery.estimate_path(len(path)) if path is not None else None
        )
        return path

    def _return_path_is_usable(
        self, runtime: DroneRuntime, path: tuple[Position, ...]
    ) -> bool:
        blocked = self._other_blockers(runtime)
        return (
            bool(path)
            and path[0] == runtime.drone.position
            and path[-1] == self.world.base
            and all(
                self.occupancy_map.is_known_free(position) for position in path
            )
            and (len(path) == 1 or path[1] not in blocked)
            and all(
                abs(first.x - second.x) + abs(first.y - second.y) == 1
                for first, second in zip(path, path[1:])
            )
        )

    def _record_event(
        self,
        runtime: DroneRuntime,
        event_type: EventType,
        position: Position | None = None,
    ) -> None:
        self.mission_log.record(
            MissionEvent(
                position=position or runtime.drone.position,
                step=self.steps,
                drone_id=runtime.drone.identifier,
                event_type=event_type,
                energy_remaining=runtime.battery.remaining,
            )
        )

    def _sense(self, runtime: DroneRuntime) -> None:
        if runtime.terminal:
            return
        if not runtime.battery.consume(self.config.sensor_energy_cost):
            self._fail_energy(runtime)
            return
        observations = self.sensor.observe(self.world, runtime.drone.position)
        self.occupancy_map.update(observations)
        self._observe_survivors(runtime)

    def _observe_survivors(self, runtime: DroneRuntime) -> None:
        visible = self.survivor_sensor.observe(self.world, runtime.drone.position)
        for position in visible:
            if runtime.last_survivor_observation_step.get(position) == self.steps:
                continue
            runtime.last_survivor_observation_step[position] = self.steps
            count = runtime.survivor_observations.get(position, 0) + 1
            runtime.survivor_observations[position] = count
            if position not in self._detected_survivors:
                self._detected_survivors.add(position)
                self._record_event(runtime, EventType.SURVIVOR_DETECTED, position)
            if (
                count >= self.config.survivor_confirmation_observations
                and position not in self._confirmed_survivors
            ):
                self._confirmed_survivors.add(position)
                self._record_event(runtime, EventType.SURVIVOR_CONFIRMED, position)

    def _objectives_complete(self) -> bool:
        return len(self._confirmed_survivors) == len(self.world.survivors)

    def _fail_energy(self, runtime: DroneRuntime) -> None:
        runtime.drone.status = DroneStatus.ENERGY_EMERGENCY
        runtime.energy_emergency = True
        runtime.active_frontier_target = None
        runtime.planned_path = ()
        runtime.current_return_path = ()
        self._record_event(runtime, EventType.ENERGY_EMERGENCY)

    def _fail_return_path(self, runtime: DroneRuntime) -> None:
        runtime.drone.status = DroneStatus.RETURN_PATH_UNAVAILABLE
        runtime.active_frontier_target = None
        runtime.planned_path = ()
        runtime.current_return_path = ()
        self._record_event(runtime, EventType.RETURN_PATH_UNAVAILABLE)

    def _land(self, runtime: DroneRuntime) -> None:
        runtime.drone.status = DroneStatus.LANDED
        runtime.active_frontier_target = None
        runtime.planned_path = ()
        runtime.current_return_path = ()
        runtime.estimated_return_energy = 0.0
        self._record_event(runtime, EventType.BASE_REACHED)

    def _start_return(
        self, runtime: DroneRuntime, path: tuple[Position, ...]
    ) -> None:
        if runtime.drone.status is DroneStatus.RETURN_HOME:
            return
        runtime.drone.status = DroneStatus.RETURN_HOME
        runtime.active_frontier_target = None
        runtime.planned_path = ()
        runtime.current_return_path = path
        runtime.estimated_return_energy = runtime.battery.estimate_path(len(path))
        runtime.return_started_step = self.steps
        self._record_event(runtime, EventType.RETURN_STARTED)
        if runtime.drone.position == self.world.base:
            self._land(runtime)

    def _prepare_energy_states(self) -> None:
        for runtime in self._ordered_runtimes():
            if runtime.drone.status is not DroneStatus.EXPLORE:
                continue
            return_path = self._refresh_return_estimate(runtime)
            if return_path is None:
                static_path = self._known_return_path(
                    runtime, avoid_other_drones=False
                )
                if static_path is None:
                    self._fail_return_path(runtime)
                continue
            assert runtime.estimated_return_energy is not None
            if runtime.battery.remaining + 1e-9 < runtime.estimated_return_energy:
                self._fail_energy(runtime)
                continue
            required = (
                runtime.estimated_return_energy
                + self.config.energy_safety_reserve
            )
            if runtime.battery.remaining + 1e-9 < required:
                if runtime.drone.position == self.world.base:
                    if self._objectives_complete():
                        self._start_return(runtime, return_path)
                    else:
                        self._fail_energy(runtime)
                else:
                    self._start_return(runtime, return_path)

    def _allocate_frontiers(self) -> dict[str, FrontierAssignment]:
        explorers = {
            runtime.drone.identifier: runtime.drone.position
            for runtime in self._ordered_runtimes()
            if runtime.drone.status is DroneStatus.EXPLORE
        }
        if not explorers:
            return {}
        frontiers = self.occupancy_map.frontiers()
        if not frontiers:
            self._exploration_complete = True
            for runtime in self._ordered_runtimes():
                if runtime.drone.status is DroneStatus.EXPLORE:
                    path = self._known_return_path(runtime)
                    if path is None:
                        path = self._known_return_path(
                            runtime, avoid_other_drones=False
                        )
                    if path is None:
                        self._fail_return_path(runtime)
                    else:
                        self._start_return(runtime, path)
            return {}

        old_targets = {
            runtime.drone.identifier: runtime.active_frontier_target
            for runtime in self._ordered_runtimes()
            if runtime.drone.identifier in explorers
        }
        assignments = assign_frontiers(
            explorers, frontiers, self.occupancy_map, old_targets
        )
        for runtime in self._ordered_runtimes():
            drone_id = runtime.drone.identifier
            if drone_id not in explorers:
                continue
            old_target = runtime.active_frontier_target
            assignment = assignments.get(drone_id)
            runtime.active_frontier_target = (
                assignment.target if assignment is not None else None
            )
            runtime.planned_path = assignment.path if assignment is not None else ()
            if assignment is not None and assignment.target != old_target:
                event_type = (
                    EventType.FRONTIER_ASSIGNED
                    if old_target is None
                    else EventType.FRONTIER_REASSIGNED
                )
                runtime.frontier_assignments += 1
                self._record_event(runtime, event_type, assignment.target)
        return assignments

    def _plan_return_intention(self, runtime: DroneRuntime) -> Position:
        previous_path = runtime.current_return_path
        previous_path_invalid = bool(previous_path) and not (
            self._return_path_is_usable(runtime, previous_path)
        )
        replan_required = previous_path_invalid or runtime.return_replan_required
        static_path = self._known_return_path(runtime, avoid_other_drones=False)
        if static_path is None:
            self._fail_return_path(runtime)
            return runtime.drone.position
        path = self._known_return_path(runtime)
        if path is None:
            runtime.current_return_path = ()
            runtime.estimated_return_energy = runtime.battery.estimate_path(
                len(static_path)
            )
            return runtime.drone.position
        required = runtime.battery.estimate_path(len(path))
        runtime.estimated_return_energy = required
        if runtime.battery.remaining + 1e-9 < required:
            self._fail_energy(runtime)
            return runtime.drone.position
        if previous_path and path != previous_path and replan_required:
            self._record_event(runtime, EventType.RETURN_REPLANNED)
        runtime.return_replan_required = False
        runtime.current_return_path = path
        if len(path) == 1:
            self._land(runtime)
            return runtime.drone.position
        return path[1]

    def _plan_explore_intention(
        self,
        runtime: DroneRuntime,
        assignment: FrontierAssignment | None,
    ) -> Position:
        if assignment is None or len(assignment.path) < 2:
            return runtime.drone.position
        next_position = assignment.path[1]
        projected_return = self._known_return_path(runtime, origin=next_position)
        if projected_return is None:
            return runtime.drone.position
        projected_remaining = (
            runtime.battery.remaining - runtime.battery.movement_cycle_cost
        )
        projected_required = (
            runtime.battery.estimate_path(len(projected_return))
            + self.config.energy_safety_reserve
        )
        if projected_remaining + 1e-9 < projected_required:
            current_return = self._known_return_path(runtime)
            if current_return is None:
                current_return = self._known_return_path(
                    runtime, avoid_other_drones=False
                )
            if current_return is None:
                self._fail_return_path(runtime)
                return runtime.drone.position
            if runtime.drone.position == self.world.base and not self._objectives_complete():
                self._fail_energy(runtime)
                return runtime.drone.position
            self._start_return(runtime, current_return)
            if runtime.drone.status is DroneStatus.RETURN_HOME:
                return self._plan_return_intention(runtime)
            return runtime.drone.position
        return next_position

    def _plan_intentions(
        self, assignments: dict[str, FrontierAssignment]
    ) -> dict[str, Position]:
        intentions: dict[str, Position] = {}
        for runtime in self._ordered_runtimes():
            if runtime.terminal:
                continue
            if runtime.drone.status is DroneStatus.RETURN_HOME:
                destination = self._plan_return_intention(runtime)
            else:
                destination = self._plan_explore_intention(
                    runtime, assignments.get(runtime.drone.identifier)
                )
            if not runtime.terminal:
                intentions[runtime.drone.identifier] = destination
        return intentions

    def _execute_intentions(self, intentions: dict[str, Position]) -> None:
        if not intentions:
            return
        current = {
            drone_id: self.runtimes[drone_id].drone.position
            for drone_id in intentions
        }
        resolved, conflicts = resolve_movements(current, intentions, self.world.base)
        self.movement_conflicts += len(conflicts)
        conflict_waiters = {
            drone_id
            for drone_id in intentions
            if resolved[drone_id] == current[drone_id]
            and intentions[drone_id] != current[drone_id]
        }
        for drone_id in sorted(conflict_waiters):
            runtime = self.runtimes[drone_id]
            self._record_event(runtime, EventType.MOVEMENT_CONFLICT)
            if runtime.drone.status is DroneStatus.RETURN_HOME:
                runtime.return_replan_required = True

        movers: dict[str, Position] = {}
        waiting: set[str] = set()
        for drone_id in sorted(intentions):
            runtime = self.runtimes[drone_id]
            destination = resolved[drone_id]
            if destination == runtime.drone.position:
                waiting.add(drone_id)
                continue
            if not self.occupancy_map.is_known_free(destination):
                self._fail_return_path(runtime)
                continue
            if not self.world.is_free(destination):
                self.collisions += 1
                runtime.drone.status = DroneStatus.FAILED
                continue
            if not runtime.battery.consume(self.config.movement_energy_cost):
                self._fail_energy(runtime)
                continue
            movers[drone_id] = destination

        for drone_id in sorted(waiting):
            runtime = self.runtimes[drone_id]
            if runtime.terminal:
                continue
            if not runtime.battery.consume(self.config.wait_energy_cost):
                self._fail_energy(runtime)
                continue
            runtime.wait_steps += 1
            self._record_event(runtime, EventType.DRONE_WAITED)

        previous = {
            drone_id: runtime.drone.position
            for drone_id, runtime in self.runtimes.items()
        }
        for drone_id, destination in movers.items():
            runtime = self.runtimes[drone_id]
            runtime.drone.position = destination
            runtime.drone.path_length += 1
            if runtime.drone.status is DroneStatus.RETURN_HOME:
                runtime.return_path_length += 1
                if runtime.current_return_path:
                    runtime.current_return_path = runtime.current_return_path[1:]
            if destination != self.world.base:
                self._visited_by_cell.setdefault(destination, set()).add(drone_id)

        self.steps += 1
        for runtime in self._ordered_runtimes():
            runtime.position_trace.append(runtime.drone.position)

        positions: dict[Position, list[str]] = {}
        for runtime in self._ordered_runtimes():
            if runtime.drone.position != self.world.base:
                positions.setdefault(runtime.drone.position, []).append(
                    runtime.drone.identifier
                )
        self.drone_drone_collisions += sum(
            len(drone_ids) - 1 for drone_ids in positions.values() if len(drone_ids) > 1
        )
        drone_ids = sorted(movers)
        for index, first_id in enumerate(drone_ids):
            for second_id in drone_ids[index + 1 :]:
                if (
                    movers[first_id] == previous[second_id]
                    and movers[second_id] == previous[first_id]
                ):
                    self.drone_drone_collisions += 1

        for runtime in self._ordered_runtimes():
            if not runtime.terminal:
                self._sense(runtime)
            if (
                runtime.drone.status is DroneStatus.RETURN_HOME
                and runtime.drone.position == self.world.base
            ):
                self._land(runtime)
            if not runtime.terminal:
                self._refresh_return_estimate(runtime)

    def _update_completion(self) -> None:
        if not all(runtime.terminal for runtime in self._ordered_runtimes()):
            return
        self.completed = True
        if all(
            runtime.drone.status is DroneStatus.LANDED
            for runtime in self._ordered_runtimes()
        ):
            self.termination_reason = (
                "exploration_complete"
                if self._exploration_complete
                else "returned_to_base"
            )
        else:
            self.termination_reason = "mission_failed"

    def step(self) -> bool:
        if self.completed:
            return False
        if self.steps >= self.config.max_steps:
            for runtime in self._ordered_runtimes():
                if not runtime.terminal:
                    runtime.drone.status = DroneStatus.FAILED
                    runtime.active_frontier_target = None
                    runtime.planned_path = ()
                    runtime.current_return_path = ()
            self.completed = True
            self.termination_reason = "max_steps"
            return False

        self._prepare_energy_states()
        assignments = self._allocate_frontiers()
        intentions = self._plan_intentions(assignments)
        self._execute_intentions(intentions)
        self._update_completion()
        return not self.completed

    def run(self, on_frame: FrameCallback | None = None) -> MultiSimulationResult:
        if on_frame is not None:
            on_frame(self)
        while self.step():
            if on_frame is not None:
                on_frame(self)
        if on_frame is not None:
            on_frame(self)
        return self.result()

    def result(self) -> MultiSimulationResult:
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
        drones_returned = sum(
            runtime.drone.status is DroneStatus.LANDED
            for runtime in self._ordered_runtimes()
        )
        drones_failed = sum(
            runtime.drone.status in {
                DroneStatus.ENERGY_EMERGENCY,
                DroneStatus.RETURN_PATH_UNAVAILABLE,
                DroneStatus.FAILED,
            }
            for runtime in self._ordered_runtimes()
        )
        unique_visited = len(self._visited_by_cell)
        duplicate_ratio = (
            sum(len(owners) > 1 for owners in self._visited_by_cell.values())
            / unique_visited
            if unique_visited
            else 0.0
        )
        mission_success = (
            len(self._confirmed_survivors) == survivors_total
            and self.collisions == 0
            and self.drone_drone_collisions == 0
            and drones_returned == len(self.runtimes)
        )
        return MultiSimulationResult(
            seed=self.config.seed,
            completed=self.completed,
            termination_reason=self.termination_reason,
            steps=self.steps,
            known_cells=self.occupancy_map.known_cell_count,
            explored_percent=self.occupancy_map.explored_percent,
            collisions=self.collisions,
            survivors_total=survivors_total,
            survivors_detected=len(self._detected_survivors),
            survivors_confirmed=len(self._confirmed_survivors),
            survivor_recall=survivor_recall,
            time_to_first_detection=min(detection_steps) if detection_steps else None,
            drones_total=len(self.runtimes),
            drones_returned=drones_returned,
            drones_failed=drones_failed,
            drone_drone_collisions=self.drone_drone_collisions,
            movement_conflicts=self.movement_conflicts,
            wait_steps_by_drone={
                runtime.drone.identifier: runtime.wait_steps
                for runtime in self._ordered_runtimes()
            },
            path_length_by_drone={
                runtime.drone.identifier: runtime.drone.path_length
                for runtime in self._ordered_runtimes()
            },
            energy_remaining_by_drone={
                runtime.drone.identifier: runtime.battery.remaining
                for runtime in self._ordered_runtimes()
            },
            frontier_assignments_by_drone={
                runtime.drone.identifier: runtime.frontier_assignments
                for runtime in self._ordered_runtimes()
            },
            drone_status_by_drone={
                runtime.drone.identifier: runtime.drone.status
                for runtime in self._ordered_runtimes()
            },
            return_started_step_by_drone={
                runtime.drone.identifier: runtime.return_started_step
                for runtime in self._ordered_runtimes()
            },
            return_path_length_by_drone={
                runtime.drone.identifier: runtime.return_path_length
                for runtime in self._ordered_runtimes()
            },
            position_trace_by_drone={
                runtime.drone.identifier: tuple(runtime.position_trace)
                for runtime in self._ordered_runtimes()
            },
            duplicate_exploration_ratio=duplicate_ratio,
            mission_success=mission_success,
            mission_events=self.mission_log.events,
        )
