"""Deterministic discrete 2.5D exploration over connected 2D floors.

The legacy simulator intentionally keeps using :class:`Position`.  This module
is the vertical v0.11 slice: floor identity is explicit at its boundary while
each floor remains an ordinary ``GridWorld``.
"""

from __future__ import annotations

import json
from collections import Counter
from dataclasses import dataclass, field
from heapq import heappop, heappush
from pathlib import Path
from random import Random
from statistics import pstdev
from typing import Callable

from echorescue.environment import GridWorld
from echorescue.models import CellState, Position
from echorescue.knowledge import ProbabilisticKnowledgeMap
from echorescue.probabilistic import ProbabilityConfig, UNCERTAINTY_PROFILES
from echorescue.sensors import DistanceSensor, uncertain_observations
from echorescue.survivors import SurvivorSensor
from echorescue.perception import SurvivorHypothesisTracker


MULTI_FLOOR_REPLAY_SCHEMA_VERSION = "2.5"


@dataclass(frozen=True, order=True, slots=True)
class GridPosition:
    floor: int
    row: int
    col: int

    @property
    def local(self) -> Position:
        return Position(self.col, self.row)

    def horizontal_neighbors(self) -> tuple[GridPosition, ...]:
        return (
            GridPosition(self.floor, self.row, self.col + 1),
            GridPosition(self.floor, self.row + 1, self.col),
            GridPosition(self.floor, self.row, self.col - 1),
            GridPosition(self.floor, self.row - 1, self.col),
        )

    def to_list(self) -> list[int]:
        return [self.floor, self.row, self.col]


@dataclass(frozen=True, order=True, slots=True)
class FloorTransition:
    source: GridPosition
    destination: GridPosition
    traversal_cost: int = 3
    bidirectional: bool = True
    enabled: bool = True
    kind: str = "STAIRS"

    def __post_init__(self) -> None:
        if self.source.floor == self.destination.floor:
            raise ValueError("a floor transition must connect different floors")
        if self.traversal_cost < 1:
            raise ValueError("transition traversal cost must be positive")

    def destination_from(self, position: GridPosition) -> GridPosition | None:
        if not self.enabled:
            return None
        if position == self.source:
            return self.destination
        if self.bidirectional and position == self.destination:
            return self.source
        return None

    def to_dict(self) -> dict[str, object]:
        return {
            "source": self.source.to_list(),
            "destination": self.destination.to_list(),
            "traversal_cost": self.traversal_cost,
            "bidirectional": self.bidirectional,
            "enabled": self.enabled,
            "kind": self.kind,
        }


@dataclass(slots=True)
class MultiFloorKnowledge:
    widths: dict[int, int]
    heights: dict[int, int]
    cells: dict[GridPosition, CellState] = field(default_factory=dict)
    probabilistic: dict[int, ProbabilisticKnowledgeMap] = field(default_factory=dict)

    def contains(self, position: GridPosition) -> bool:
        return (
            position.floor in self.widths
            and 0 <= position.col < self.widths[position.floor]
            and 0 <= position.row < self.heights[position.floor]
        )

    def cell_at(self, position: GridPosition) -> CellState:
        if not self.contains(position):
            return CellState.OCCUPIED
        if self.probabilistic:
            return self.probabilistic[position.floor].cell_at(position.local)
        return self.cells.get(position, CellState.UNKNOWN)

    def observe(self, position: GridPosition, state: CellState) -> None:
        if self.contains(position):
            self.cells[position] = state
            if self.probabilistic:
                self.probabilistic[position.floor].observe({position.local: state},
                    step=0, source_id="mission-topology")

    def is_known_free(self, position: GridPosition) -> bool:
        return self.cell_at(position) is CellState.FREE

    def frontiers(self, floor: int | None = None) -> tuple[GridPosition, ...]:
        result = []
        positions = (tuple(GridPosition(f, y, x) for f in sorted(self.widths)
                           for y in range(self.heights[f]) for x in range(self.widths[f]))
                     if self.probabilistic else tuple(sorted(self.cells)))
        for position in positions:
            if self.cell_at(position) is not CellState.FREE:
                continue
            if floor is not None and position.floor != floor:
                continue
            if any(
                self.contains(neighbor)
                and self.cell_at(neighbor) is CellState.UNKNOWN
                for neighbor in position.horizontal_neighbors()
            ):
                result.append(position)
        return tuple(result)

    def floor_rows(self, floor: int) -> list[str]:
        symbols = {
            CellState.UNKNOWN: "?",
            CellState.FREE: ".",
            CellState.OCCUPIED: "#",
        }
        return [
            "".join(
                symbols[self.cell_at(GridPosition(floor, row, col))]
                for col in range(self.widths[floor])
            )
            for row in range(self.heights[floor])
        ]


@dataclass(slots=True)
class MultiFloorEnvironment:
    floors: dict[int, GridWorld]
    transitions: tuple[FloorTransition, ...]
    base: GridPosition

    def __post_init__(self) -> None:
        if self.base.floor not in self.floors:
            raise ValueError("base floor is not configured")
        if not self.is_free(self.base):
            raise ValueError("base must be free")
        for transition in self.transitions:
            if transition.source.floor not in self.floors:
                raise ValueError("transition source floor is not configured")
            if transition.destination.floor not in self.floors:
                raise ValueError("transition destination floor is not configured")
            if not self.is_free(transition.source) or not self.is_free(
                transition.destination
            ):
                raise ValueError("transition endpoints must be free")

    @classmethod
    def generate(cls, config: MultiFloorConfig) -> MultiFloorEnvironment:
        endpoint_cells = (
            Position(2, 2),
            Position(config.width - 3, config.height - 3),
        )
        floors: dict[int, GridWorld] = {}
        for floor in range(config.floor_count):
            protected = set(endpoint_cells)
            base = Position(1, 1)
            boundary = {
                Position(col, row)
                for row in range(config.height)
                for col in range(config.width)
                if col in (0, config.width - 1)
                or row in (0, config.height - 1)
            }
            candidates = [
                Position(col, row)
                for row in range(1, config.height - 1)
                for col in range(1, config.width - 1)
                if Position(col, row) not in protected | {base}
            ]
            rng = Random(config.seed + floor * 1_009)
            rng.shuffle(candidates)
            walls = set(boundary)
            target = int(len(candidates) * config.obstacle_density)
            for candidate in candidates:
                if len(walls) - len(boundary) >= target:
                    break
                walls.add(candidate)
                if not GridWorld._interior_is_connected(
                    config.width, config.height, base, walls
                ):
                    walls.remove(candidate)
            survivor_candidates = [
                position
                for position in candidates
                if position not in walls and position not in protected
            ]
            survivor_rng = Random(config.seed + 10_000_019 + floor * 1_009)
            survivor_rng.shuffle(survivor_candidates)
            survivors = frozenset(
                survivor_candidates[: config.survivors_per_floor]
            )
            floors[floor] = GridWorld(
                config.width,
                config.height,
                base,
                frozenset(walls),
                survivors,
            )
        transitions: list[FloorTransition] = []
        for floor in range(config.floor_count - 1):
            for index, cell in enumerate(endpoint_cells[: config.stairwells_per_pair]):
                destination_cell = endpoint_cells[
                    (index + floor) % len(endpoint_cells)
                ]
                transitions.append(
                    FloorTransition(
                        GridPosition(floor, cell.y, cell.x),
                        GridPosition(floor + 1, destination_cell.y, destination_cell.x),
                        config.transition_cost,
                    )
                )
        return cls(
            floors,
            tuple(sorted(transitions)),
            GridPosition(0, base.y, base.x),
        )

    def contains(self, position: GridPosition) -> bool:
        world = self.floors.get(position.floor)
        return world is not None and world.contains(position.local)

    def cell_at(self, position: GridPosition) -> CellState:
        world = self.floors.get(position.floor)
        if world is None:
            return CellState.OCCUPIED
        return world.cell_at(position.local)

    def is_free(self, position: GridPosition) -> bool:
        return self.cell_at(position) is CellState.FREE

    @property
    def survivors(self) -> frozenset[GridPosition]:
        return frozenset(
            GridPosition(floor, survivor.y, survivor.x)
            for floor, world in self.floors.items()
            for survivor in world.survivors
        )

    def transition_from(
        self, source: GridPosition, destination: GridPosition
    ) -> FloorTransition | None:
        for transition in self.transitions:
            if transition.destination_from(source) == destination:
                return transition
        return None

    def neighbors(self, position: GridPosition) -> tuple[tuple[GridPosition, int], ...]:
        neighbors = [
            (neighbor, 1)
            for neighbor in position.horizontal_neighbors()
            if self.is_free(neighbor)
        ]
        for transition in self.transitions:
            destination = transition.destination_from(position)
            if destination is not None and self.is_free(destination):
                neighbors.append((destination, transition.traversal_cost))
        return tuple(sorted(neighbors))

    def block_cell(self, position: GridPosition) -> None:
        if any(
            position in {transition.source, transition.destination}
            for transition in self.transitions
        ):
            raise ValueError("dynamic obstacle cannot block a transition endpoint")
        self.floors[position.floor].block_cell(position.local)


def multi_floor_astar(
    environment: MultiFloorEnvironment,
    start: GridPosition,
    goal: GridPosition,
    passable: Callable[[GridPosition], bool] | None = None,
) -> tuple[GridPosition, ...] | None:
    """Weighted deterministic A* with an admissible zero cross-floor heuristic."""

    allowed = passable or environment.is_free
    if not allowed(start) or not allowed(goal):
        return None

    def heuristic(position: GridPosition) -> int:
        if position.floor != goal.floor:
            return 0
        return abs(position.row - goal.row) + abs(position.col - goal.col)

    frontier: list[tuple[int, int, GridPosition]] = [(heuristic(start), 0, start)]
    best = {start: 0}
    came_from: dict[GridPosition, GridPosition] = {}
    while frontier:
        _, cost, current = heappop(frontier)
        if cost != best.get(current):
            continue
        if current == goal:
            path = [current]
            while current != start:
                current = came_from[current]
                path.append(current)
            return tuple(reversed(path))
        for neighbor, edge_cost in environment.neighbors(current):
            if not allowed(neighbor):
                continue
            next_cost = cost + edge_cost
            if next_cost < best.get(neighbor, 10**9):
                best[neighbor] = next_cost
                came_from[neighbor] = current
                heappush(
                    frontier,
                    (next_cost + heuristic(neighbor), next_cost, neighbor),
                )
    return None


def path_cost(
    environment: MultiFloorEnvironment, path: tuple[GridPosition, ...]
) -> int:
    total = 0
    for source, destination in zip(path, path[1:]):
        transition = environment.transition_from(source, destination)
        total += transition.traversal_cost if transition is not None else 1
    return total


@dataclass(frozen=True, slots=True)
class MultiFloorConfig:
    uncertainty_profile: str = "off"
    planning_variant: str = "naive"
    probability_config: ProbabilityConfig = ProbabilityConfig()
    floor_count: int = 3
    width: int = 13
    height: int = 9
    seed: int = 7
    drone_count: int = 4
    obstacle_density: float = 0.08
    sensor_range: int = 3
    survivor_range: int = 3
    survivors_per_floor: int = 1
    transition_cost: int = 3
    stairwells_per_pair: int = 2
    movement_energy_cost: float = 1.0
    sensor_energy_cost: float = 0.05
    transition_energy_cost: float = 3.0
    battery_capacity: float = 500.0
    energy_reserve: float = 20.0
    floor_congestion_penalty: int = 4
    communication_profile: str = "ideal"
    communication_range: int = 8
    max_steps: int = 1_000
    failure_schedule: tuple[tuple[str, int], ...] = ()
    dynamic_obstacle_schedule: tuple[tuple[int, int, int, int], ...] = ()

    def __post_init__(self) -> None:
        if self.uncertainty_profile not in {"off", *UNCERTAINTY_PROFILES}:
            raise ValueError("unknown uncertainty profile")
        if self.planning_variant not in {"naive", "uncertainty-aware"}:
            raise ValueError("unknown planning variant")
        if self.uncertainty_profile == "off" and self.planning_variant != "naive":
            raise ValueError("probabilistic perception required")
        if not 1 <= self.floor_count <= 8:
            raise ValueError("floor_count must be between 1 and 8")
        if self.width < 7 or self.height < 7:
            raise ValueError("floor dimensions must be at least 7")
        if not 1 <= self.drone_count <= 8:
            raise ValueError("drone_count must be between 1 and 8")
        if self.transition_cost < 1 or self.transition_energy_cost <= 0:
            raise ValueError("transition costs must be positive")
        if self.stairwells_per_pair not in {1, 2}:
            raise ValueError("stairwells_per_pair must be 1 or 2")
        if self.communication_profile not in {"ideal", "constrained"}:
            raise ValueError("communication_profile must be ideal or constrained")


@dataclass(slots=True)
class FloorAgent:
    identifier: str
    position: GridPosition
    energy: float
    status: str = "EXPLORE"
    target: GridPosition | None = None
    path: tuple[GridPosition, ...] = ()
    pending_transition: GridPosition | None = None
    transition_wait: int = 0
    path_length: int = 0
    floor_steps: Counter[int] = field(default_factory=Counter)
    floor_path_length: Counter[int] = field(default_factory=Counter)
    transitions: int = 0
    wait_steps: int = 0
    yielding: bool = False


@dataclass(frozen=True, slots=True)
class MultiFloorResult:
    seed: int
    success: bool
    steps: int
    survivor_recall: float
    returned_agents: int
    wall_collisions: int
    drone_collisions: int
    total_path_length: int
    metrics: dict[str, object]

    def to_dict(self) -> dict[str, object]:
        return {
            "seed": self.seed,
            "mission_success": self.success,
            "mission_duration": self.steps,
            "survivor_recall": self.survivor_recall,
            "returned_agents": self.returned_agents,
            "wall_collisions": self.wall_collisions,
            "drone_collisions": self.drone_collisions,
            "total_path_length": self.total_path_length,
            "multi_floor_metrics": self.metrics,
        }


class MultiFloorSimulation:
    """Small centralized multi-agent 2.5D research slice."""

    def __init__(
        self,
        config: MultiFloorConfig,
        environment: MultiFloorEnvironment | None = None,
    ) -> None:
        self.config = config
        self.environment = environment or MultiFloorEnvironment.generate(config)
        self.knowledge = MultiFloorKnowledge(
            {floor: world.width for floor, world in self.environment.floors.items()},
            {floor: world.height for floor, world in self.environment.floors.items()},
        )
        self.floor_hypotheses = {floor: SurvivorHypothesisTracker(
            minimum_positive_observations=2, confirmation_threshold=.65,
            rejection_threshold=.12, negative_evidence_weight=.55)
            for floor in self.environment.floors}
        if config.uncertainty_profile != "off":
            self.knowledge.probabilistic = {floor: ProbabilisticKnowledgeMap(
                world.width, world.height, floor=floor,
                probability_config=config.probability_config,
                reliability=UNCERTAINTY_PROFILES[config.uncertainty_profile].occupancy_reliability,
                planning_variant=config.planning_variant)
                for floor, world in self.environment.floors.items()}
        for transition in self.environment.transitions:
            self.knowledge.observe(transition.source, CellState.FREE)
            self.knowledge.observe(transition.destination, CellState.FREE)
        self.knowledge.observe(self.environment.base, CellState.FREE)
        self.agents = {
            f"drone-{index}": FloorAgent(
                f"drone-{index}", self.environment.base, config.battery_capacity
            )
            for index in range(1, config.drone_count + 1)
        }
        self.steps = 0
        self.confirmed_survivors: set[GridPosition] = set()
        self.events: list[dict[str, object]] = []
        self.frames: list[dict[str, object]] = []
        self.transition_conflicts = 0
        self.wall_collisions = 0
        self.safety_interventions = 0
        self.drone_collisions = 0
        self.time_to_first_new_floor: int | None = None
        self.time_to_first_survivor_per_floor: dict[int, int] = {}
        self._completed_floors: set[int] = set()
        self._orphaned_targets: set[GridPosition] = set()
        self._capture_frame()

    def _event(
        self,
        event_type: str,
        agent: FloorAgent,
        *,
        source: GridPosition | None = None,
        destination: GridPosition | None = None,
    ) -> None:
        self.events.append(
            {
                "event_type": event_type,
                "step": self.steps,
                "agent_id": agent.identifier,
                "position": agent.position.to_list(),
                "source_floor": source.floor if source is not None else None,
                "destination_floor": (
                    destination.floor if destination is not None else None
                ),
                "transition_location": (
                    source.to_list() if source is not None else None
                ),
            }
        )

    def _observe(self, agent: FloorAgent) -> None:
        world = self.environment.floors[agent.position.floor]
        if self.config.uncertainty_profile != "off":
            knowledge = self.knowledge.probabilistic[agent.position.floor]
            observed = uncertain_observations(
                DistanceSensor(self.config.sensor_range).observe(world, agent.position.local),
                seed=self.config.seed, agent_id=agent.identifier, step=self.steps,
                profile=self.config.uncertainty_profile, floor=agent.position.floor)
            knowledge.observe(observed, step=self.steps, source_id=agent.identifier)
            report = SurvivorSensor(self.config.survivor_range).observe_report(
                world, agent.position.local, smoke_profile="off", seed=self.config.seed, step=self.steps,
                observer_id=f"{agent.identifier}:floor:{agent.position.floor}",
                perception_noise=self.config.uncertainty_profile)
            tracker = self.floor_hypotheses[agent.position.floor]
            positives = set()
            for observation in report.observations:
                if observation.success:
                    positives.add(observation.position)
                    tracker.positive(observation.position, confidence=observation.confidence,
                        channel="visual", agent_id=agent.identifier, step=self.steps)
            for position in tuple(tracker.hypotheses):
                if position not in positives and SurvivorSensor(self.config.survivor_range).can_observe(
                        world, agent.position.local, position):
                    tracker.negative(position, confidence=UNCERTAINTY_PROFILES[
                        self.config.uncertainty_profile].survivor_reliability,
                        channel="visual", agent_id=agent.identifier, step=self.steps)
            for position in tracker.confirmed_locations:
                location = GridPosition(agent.position.floor, position.y, position.x)
                if location not in self.confirmed_survivors:
                    self.confirmed_survivors.add(location)
                    self.time_to_first_survivor_per_floor.setdefault(location.floor, self.steps)
                    self._event("survivor_confirmed", agent)
            return
        for row in range(world.height):
            for col in range(world.width):
                grid_position = GridPosition(agent.position.floor, row, col)
                distance = abs(row - agent.position.row) + abs(col - agent.position.col)
                if distance <= self.config.sensor_range:
                    self.knowledge.observe(grid_position, self.environment.cell_at(grid_position))
        for survivor in self.environment.survivors:
            if (
                survivor.floor == agent.position.floor
                and abs(survivor.row - agent.position.row)
                + abs(survivor.col - agent.position.col)
                <= self.config.survivor_range
                and survivor not in self.confirmed_survivors
            ):
                self.confirmed_survivors.add(survivor)
                self.time_to_first_survivor_per_floor.setdefault(
                    survivor.floor, self.steps
                )
                self._event("survivor_confirmed", agent)

    def _known_path(
        self,
        start: GridPosition,
        goal: GridPosition,
        blocked: frozenset[GridPosition] = frozenset(),
    ) -> tuple[GridPosition, ...] | None:
        failed_cells = frozenset(
            agent.position
            for agent in self.agents.values()
            if agent.status == "FAILED"
        )
        unavailable = blocked | failed_cells
        return multi_floor_astar(
            self.environment,
            start,
            goal,
            lambda position: self.knowledge.is_known_free(position)
            and (position not in unavailable or position in {start, goal}),
        )

    def _return_path(self, agent: FloorAgent) -> tuple[GridPosition, ...] | None:
        return self._known_path(agent.position, self.environment.base)

    def _allocate(self) -> None:
        reserved = {
            agent.target
            for agent in self.agents.values()
            if agent.status == "EXPLORE" and agent.target is not None
        }
        floor_load = Counter(
            (agent.target or agent.position).floor
            for agent in self.agents.values()
            if agent.status == "EXPLORE"
        )
        frontiers = self.knowledge.frontiers()
        for agent in sorted(self.agents.values(), key=lambda item: item.identifier):
            if agent.status != "EXPLORE" or agent.pending_transition is not None:
                continue
            if agent.yielding:
                continue
            if agent.path and agent.target in frontiers:
                continue
            local_frontiers = {
                target for target in frontiers if target.floor == agent.position.floor
            }
            candidates = []
            for target in frontiers:
                # Once an agent has entered another floor, finish available
                # local work before switching again. This is semantic
                # stickiness, not a seed-tuned numeric penalty.
                if agent.transitions > 0 and local_frontiers and target not in local_frontiers:
                    continue
                if target in reserved:
                    continue
                path = self._known_path(agent.position, target)
                if path is None:
                    continue
                cost = path_cost(self.environment, path)
                score = cost + self.config.floor_congestion_penalty * floor_load[target.floor]
                bonus = (self.knowledge.probabilistic[target.floor].information_bonus(target.local)
                         if self.knowledge.probabilistic else 0.0)
                candidates.append((score-bonus, cost, target, path))
            if not candidates:
                # Do not let an unassigned agent become a permanent corridor
                # obstacle. It deterministically vacates toward Base; other
                # agents continue the remaining owned frontier work.
                return_path = self._return_path(agent)
                if return_path is not None:
                    agent.status = "RETURN_HOME"
                    agent.target = self.environment.base
                    agent.path = return_path
                else:
                    agent.path = ()
                    agent.target = None
                continue
            _, _, target, path = min(candidates)
            agent.target = target
            agent.path = path
            reserved.add(target)
            floor_load[target.floor] += 1
            self._event("floor_target_assigned", agent, destination=target)
            if self._orphaned_targets:
                self._orphaned_targets.remove(min(self._orphaned_targets))
                self._event("floor_task_reassigned", agent, destination=target)

    def _apply_schedules(self) -> None:
        for drone_id, step in self.config.failure_schedule:
            agent = self.agents.get(drone_id)
            if step == self.steps and agent is not None and agent.status != "FAILED":
                if agent.target is not None and agent.target != self.environment.base:
                    self._orphaned_targets.add(agent.target)
                agent.status = "FAILED"
                agent.path = ()
                agent.target = None
                agent.pending_transition = None
                agent.transition_wait = 0
                self._event("agent_failed", agent)
        for floor, row, col, step in self.config.dynamic_obstacle_schedule:
            if step != self.steps:
                continue
            position = GridPosition(floor, row, col)
            self.environment.block_cell(position)
            self.events.append(
                {
                    "event_type": "dynamic_obstacle_activated",
                    "step": self.steps,
                    "position": position.to_list(),
                }
            )
            for agent in self.agents.values():
                if position in agent.path:
                    agent.path = ()
                    agent.target = None

    def _start_returns_if_needed(self) -> None:
        exploration_complete = not self.knowledge.frontiers()
        all_survivors = self.confirmed_survivors == set(self.environment.survivors)
        for agent in self.agents.values():
            if (
                agent.status == "RETURN_HOME"
                and agent.pending_transition is None
                and agent.position != self.environment.base
                and len(agent.path) < 2
            ):
                recovered_path = self._return_path(agent)
                if recovered_path is not None:
                    agent.target = self.environment.base
                    agent.path = recovered_path
            if agent.status != "EXPLORE":
                continue
            return_path = self._return_path(agent)
            if return_path is None:
                continue
            return_cost = path_cost(self.environment, return_path)
            low_energy = (
                agent.energy
                <= return_cost * self.config.movement_energy_cost
                + self.config.energy_reserve
                + self.config.transition_energy_cost
            )
            if (exploration_complete and all_survivors) or low_energy:
                agent.status = "RETURN_HOME"
                agent.target = self.environment.base
                agent.path = return_path

    def _advance_transitions(self) -> None:
        for agent in sorted(self.agents.values(), key=lambda item: item.identifier):
            if agent.status == "FAILED":
                continue
            if agent.pending_transition is None:
                continue
            agent.transition_wait -= 1
            if agent.transition_wait > 0:
                continue
            occupied = {
                other.position
                for other in self.agents.values()
                if other is not agent
                and other.status != "LANDED"
            }
            if (
                agent.pending_transition in occupied
                and agent.pending_transition != self.environment.base
            ):
                blocker = next(
                    other
                    for other in self.agents.values()
                    if other is not agent
                    and other.status != "LANDED"
                    and other.position == agent.pending_transition
                )
                occupied_cells = {
                    other.position
                    for other in self.agents.values()
                    if other is not blocker
                    and other.status not in {"FAILED", "LANDED"}
                }
                escape = next(
                    (
                        neighbor
                        for neighbor in blocker.position.horizontal_neighbors()
                        if self.knowledge.is_known_free(neighbor)
                        and neighbor not in occupied_cells
                    ),
                    None,
                )
                if escape is not None and blocker.pending_transition is None:
                    blocker.target = None
                    blocker.path = (blocker.position, escape)
                    blocker.yielding = True
                agent.transition_wait = 1
                agent.wait_steps += 1
                self.transition_conflicts += 1
                self._event("transition_conflict", agent)
                if (
                    agent.wait_steps >= 3
                    and agent.target is not None
                    and (escape is None or blocker.pending_transition is not None)
                ):
                    # Abort before entering the connector and route around the
                    # occupied exit. With two stairwells this resolves true
                    # opposite-direction gridlock without moving either agent
                    # through another.
                    agent.pending_transition = None
                    agent.transition_wait = 0
                    alternate = self._known_path(
                        agent.position,
                        agent.target,
                        frozenset({blocker.position}),
                    )
                    if alternate is not None:
                        agent.path = alternate
                    agent.wait_steps = 0
                continue
            source = agent.position
            agent.position = agent.pending_transition
            agent.pending_transition = None
            agent.transitions += 1
            agent.path_length += self.config.transition_cost
            agent.floor_path_length[agent.position.floor] += self.config.transition_cost
            agent.energy -= self.config.transition_energy_cost
            agent.wait_steps = 0
            if self.time_to_first_new_floor is None and agent.position.floor != 0:
                self.time_to_first_new_floor = self.steps
            self._event(
                "floor_transition_completed",
                agent,
                source=source,
                destination=agent.position,
            )
            self._event("floor_entered", agent, source=source, destination=agent.position)

    def _move(self) -> None:
        proposals: dict[str, GridPosition] = {}
        transition_requests: dict[
            tuple[GridPosition, GridPosition], list[str]
        ] = {}
        reserved_transition_exits = {
            agent.pending_transition
            for agent in self.agents.values()
            if agent.pending_transition is not None
        }
        for agent in sorted(self.agents.values(), key=lambda item: item.identifier):
            if agent.status not in {"EXPLORE", "RETURN_HOME"}:
                continue
            if agent.pending_transition is not None or len(agent.path) < 2:
                continue
            if agent.path[0] != agent.position:
                agent.path = ()
                agent.target = None
                continue
            destination = agent.path[1]
            if (
                destination in reserved_transition_exits
                and destination != self.environment.base
            ):
                agent.wait_steps += 1
                continue
            if destination.floor != agent.position.floor:
                key = (
                    (agent.position, destination)
                    if agent.position < destination
                    else (destination, agent.position)
                )
                transition_requests.setdefault(key, []).append(agent.identifier)
            proposals[agent.identifier] = destination
        blocked: set[str] = set()
        for _, drone_ids in sorted(transition_requests.items()):
            if len(drone_ids) > 1:
                winner = min(
                    drone_ids,
                    key=lambda drone_id: (
                        self.agents[drone_id].status != "RETURN_HOME",
                        drone_id,
                    ),
                )
                for drone_id in drone_ids:
                    if drone_id != winner:
                        blocked.add(drone_id)
                        self.transition_conflicts += 1
                        self._event("transition_conflict", self.agents[drone_id])
                        loser = self.agents[drone_id]
                        winner_destination = proposals[winner]
                        if loser.position == winner_destination:
                            occupied_cells = {
                                other.position
                                for other in self.agents.values()
                                if other is not loser
                                and other.status not in {"FAILED", "LANDED"}
                            }
                            escape = next(
                                (
                                    neighbor
                                    for neighbor in loser.position.horizontal_neighbors()
                                    if self.knowledge.is_known_free(neighbor)
                                    and neighbor not in occupied_cells
                                ),
                                None,
                            )
                            if escape is not None:
                                loser.target = None
                                loser.path = (loser.position, escape)
                                loser.yielding = True
                                proposals[drone_id] = escape
                                blocked.discard(drone_id)
                                blocked.add(winner)
        proposal_ids = sorted(proposals)
        for index, first_id in enumerate(proposal_ids):
            for second_id in proposal_ids[index + 1 :]:
                first = self.agents[first_id]
                second = self.agents[second_id]
                if not (
                    proposals[first_id] == second.position
                    and proposals[second_id] == first.position
                ):
                    continue
                winner_id = min(
                    (first_id, second_id),
                    key=lambda drone_id: (
                        self.agents[drone_id].status != "RETURN_HOME",
                        drone_id,
                    ),
                )
                loser_id = second_id if winner_id == first_id else first_id
                loser = self.agents[loser_id]
                occupied_cells = {
                    agent.position
                    for agent in self.agents.values()
                    if agent is not loser
                    and agent.status not in {"FAILED", "LANDED"}
                }
                escape = next(
                    (
                        neighbor
                        for neighbor in loser.position.horizontal_neighbors()
                        if self.knowledge.is_known_free(neighbor)
                        and neighbor not in occupied_cells
                        and neighbor != proposals[winner_id]
                    ),
                    None,
                )
                blocked.add(winner_id)
                if escape is None:
                    blocked.add(loser_id)
                else:
                    proposals[loser_id] = escape
                    loser.target = None
                    loser.path = (loser.position, escape)
                    loser.yielding = True
                    blocked.discard(loser_id)
        by_destination: dict[GridPosition, list[str]] = {}
        for drone_id, destination in proposals.items():
            if drone_id not in blocked:
                by_destination.setdefault(destination, []).append(drone_id)
        for destination, drone_ids in sorted(by_destination.items()):
            if destination != self.environment.base and len(drone_ids) > 1:
                for drone_id in sorted(drone_ids)[1:]:
                    blocked.add(drone_id)
                    self.transition_conflicts += int(
                        destination.floor != self.agents[drone_id].position.floor
                    )
                    self._event("transition_conflict", self.agents[drone_id])
        occupied = {
            agent.position: agent.identifier
            for agent in self.agents.values()
            if agent.status != "LANDED" and agent.position != self.environment.base
        }
        for drone_id, destination in sorted(proposals.items()):
            agent = self.agents[drone_id]
            if drone_id in blocked:
                agent.wait_steps += 1
                continue
            occupant = occupied.get(destination)
            if occupant is not None:
                occupant_destination = proposals.get(occupant)
                if (
                    occupant in blocked
                    or occupant_destination is None
                    or occupant_destination == agent.position
                ):
                    agent.wait_steps += 1
                    continue
            if not self.environment.is_free(destination):
                if self.config.uncertainty_profile != "off":
                    self.safety_interventions += 1
                    agent.path = ()
                    agent.target = None
                    continue
                self.wall_collisions += 1
                agent.path = ()
                agent.target = None
                continue
            agent.path = agent.path[1:]
            if destination.floor != agent.position.floor:
                transition = self.environment.transition_from(
                    agent.position, destination
                )
                if transition is None:
                    self.wall_collisions += 1
                    continue
                agent.pending_transition = destination
                agent.transition_wait = transition.traversal_cost
                self._event(
                    "floor_transition_started",
                    agent,
                    source=agent.position,
                    destination=destination,
                )
            else:
                agent.position = destination
                agent.path_length += 1
                agent.floor_path_length[destination.floor] += 1
                agent.energy -= self.config.movement_energy_cost
            agent.yielding = False
            agent.wait_steps = 0
        for agent in self.agents.values():
            if agent.wait_steps < 3 or agent.target is None:
                continue
            blocked_positions = frozenset(
                other.position
                for other in self.agents.values()
                if other is not agent
                and other.status not in {"FAILED", "LANDED"}
                and other.position != self.environment.base
            )
            replacement = self._known_path(
                agent.position, agent.target, blocked_positions
            )
            if replacement is not None:
                agent.path = replacement
            agent.wait_steps = 0

    def _finish_landed(self) -> None:
        for agent in self.agents.values():
            if (
                agent.status == "RETURN_HOME"
                and agent.position == self.environment.base
                and agent.pending_transition is None
            ):
                agent.status = "LANDED"
                agent.path = ()

    def _floor_coverage(self, floor: int) -> float:
        world = self.environment.floors[floor]
        known = sum(
            self.knowledge.cell_at(GridPosition(floor, row, col))
            is not CellState.UNKNOWN
            for row in range(world.height)
            for col in range(world.width)
        )
        return known / (world.width * world.height)

    def _capture_frame(self) -> None:
        links = self.communication_links()
        self.frames.append(
            {
                "step": self.steps,
                "active_floor": 0,
                **({"probabilistic_maps": {str(f): m.telemetry()
                     for f, m in self.knowledge.probabilistic.items()},
                    "perception_profile": self.config.uncertainty_profile,
                    "planning_variant": self.config.planning_variant,
                    "survivor_hypotheses": [dict(h.to_dict(), floor=f)
                        for f, tracker in self.floor_hypotheses.items()
                        for h in tracker.hypotheses.values()]}
                   if self.knowledge.probabilistic else {}),
                "floor_maps": {
                    str(floor): self.knowledge.floor_rows(floor)
                    for floor in sorted(self.environment.floors)
                },
                "drones": {
                    agent.identifier: {
                        "position": agent.position.to_list(),
                        "floor": agent.position.floor,
                        "state": agent.status,
                        "energy_remaining": round(agent.energy, 6),
                        "target": (
                            agent.target.to_list()
                            if agent.target is not None
                            else None
                        ),
                        "path": [position.to_list() for position in agent.path],
                    }
                    for agent in sorted(
                        self.agents.values(), key=lambda item: item.identifier
                    )
                },
                "confirmed_survivors": [
                    position.to_list()
                    for position in sorted(self.confirmed_survivors)
                ],
                "communication_links": [list(link) for link in links],
                "events": [
                    event for event in self.events if event["step"] == self.steps
                ],
            }
        )

    def communication_links(self) -> tuple[tuple[str, str], ...]:
        active = [
            agent
            for agent in sorted(self.agents.values(), key=lambda item: item.identifier)
            if agent.status != "FAILED"
        ]
        if self.config.communication_profile == "ideal":
            return tuple(("base", agent.identifier) for agent in active)
        nodes = [("base", self.environment.base)] + [
            (agent.identifier, agent.position) for agent in active
        ]
        links = []
        endpoints = {
            transition.source for transition in self.environment.transitions
        } | {transition.destination for transition in self.environment.transitions}
        for index, (first_id, first) in enumerate(nodes):
            for second_id, second in nodes[index + 1 :]:
                if first.floor == second.floor:
                    distance = abs(first.row - second.row) + abs(first.col - second.col)
                    if distance <= self.config.communication_range:
                        links.append((first_id, second_id))
                elif first in endpoints and second in endpoints:
                    if self.environment.transition_from(first, second) is not None:
                        links.append((first_id, second_id))
        return tuple(links)

    def step(self) -> None:
        self.steps += 1
        for knowledge in self.knowledge.probabilistic.values():
            knowledge.advance(self.steps)
        self._apply_schedules()
        self._advance_transitions()
        for agent in self.agents.values():
            if agent.status != "FAILED":
                agent.floor_steps[agent.position.floor] += 1
                agent.energy -= self.config.sensor_energy_cost
                self._observe(agent)
        for floor in self.environment.floors:
            if floor not in self._completed_floors and not self.knowledge.frontiers(floor):
                if self._floor_coverage(floor) > 0.5:
                    self._completed_floors.add(floor)
                    observer = next(iter(self.agents.values()))
                    self._event("floor_exploration_completed", observer)
        self._start_returns_if_needed()
        self._allocate()
        self._move()
        self._finish_landed()
        self._capture_frame()

    def run(self) -> MultiFloorResult:
        while self.steps < self.config.max_steps:
            operational = [
                agent for agent in self.agents.values() if agent.status != "FAILED"
            ]
            if operational and all(agent.status == "LANDED" for agent in operational):
                break
            self.step()
        return self.result()

    def result(self) -> MultiFloorResult:
        operational = [
            agent for agent in self.agents.values() if agent.status != "FAILED"
        ]
        recalled = len(self.confirmed_survivors)
        total_survivors = len(self.environment.survivors)
        recall = recalled / total_survivors if total_survivors else 1.0
        returned = sum(agent.status == "LANDED" for agent in operational)
        floors_visited = sorted(
            {
                floor
                for agent in self.agents.values()
                for floor, steps in agent.floor_steps.items()
                if steps > 0
            }
        )
        total_floor_steps = sum(
            sum(agent.floor_steps.values()) for agent in self.agents.values()
        )
        work = {
            str(floor): round(
                sum(agent.floor_steps[floor] for agent in self.agents.values())
                / max(1, total_floor_steps),
                6,
            )
            for floor in sorted(self.environment.floors)
        }
        per_floor_recall = {}
        for floor in sorted(self.environment.floors):
            expected = {item for item in self.environment.survivors if item.floor == floor}
            per_floor_recall[str(floor)] = (
                len(expected & self.confirmed_survivors) / len(expected)
                if expected
                else 1.0
            )
        floor_path = {
            str(floor): sum(agent.floor_path_length[floor] for agent in self.agents.values())
            for floor in sorted(self.environment.floors)
        }
        metrics: dict[str, object] = {
            "floors_configured": len(self.environment.floors),
            "floors_visited": floors_visited,
            "floor_transitions_total": sum(
                agent.transitions for agent in self.agents.values()
            ),
            "floor_transitions_per_agent": {
                agent.identifier: agent.transitions
                for agent in sorted(
                    self.agents.values(), key=lambda item: item.identifier
                )
            },
            "transition_conflicts": self.transition_conflicts,
            "time_to_first_new_floor": self.time_to_first_new_floor,
            "time_to_first_survivor_per_floor": {
                str(floor): step
                for floor, step in sorted(
                    self.time_to_first_survivor_per_floor.items()
                )
            },
            "explored_percent_per_floor": {
                str(floor): round(100 * self._floor_coverage(floor), 6)
                for floor in sorted(self.environment.floors)
            },
            "survivor_recall_per_floor": per_floor_recall,
            "path_length_per_floor": floor_path,
            "agent_steps_per_floor": {
                agent.identifier: {
                    str(floor): steps
                    for floor, steps in sorted(agent.floor_steps.items())
                }
                for agent in sorted(
                    self.agents.values(), key=lambda item: item.identifier
                )
            },
            "floor_work_distribution": work,
            "vertical_transition_overhead": sum(
                agent.transitions * (self.config.transition_cost - 1)
                for agent in self.agents.values()
            ),
            "floor_ping_pong_agents": [
                agent.identifier
                for agent in self.agents.values()
                if agent.transitions > 2 * max(1, self.config.floor_count - 1) + 2
            ],
        }
        success = (
            recall == 1.0
            and not (self.confirmed_survivors - self.environment.survivors)
            and returned == len(operational)
            and self.wall_collisions == 0
            and self.drone_collisions == 0
            and len(floors_visited) == len(self.environment.floors)
        )
        return MultiFloorResult(
            self.config.seed,
            success,
            self.steps,
            recall,
            returned,
            self.wall_collisions,
            self.drone_collisions,
            sum(agent.path_length for agent in self.agents.values()),
            metrics,
        )

    def replay(self, result: MultiFloorResult | None = None) -> dict[str, object]:
        final = result or self.result()
        return {
            "schema_version": MULTI_FLOOR_REPLAY_SCHEMA_VERSION,
            "mission": {
                "seed": self.config.seed,
                "floor_count": self.config.floor_count,
                "drone_count": self.config.drone_count,
                "knowledge_mode": "shared",
                "network_profile": self.config.communication_profile,
                "configuration": {
                    "transition_cost": self.config.transition_cost,
                    "transition_energy_cost": self.config.transition_energy_cost,
                    "floor_congestion_penalty": self.config.floor_congestion_penalty,
                },
            },
            "map": {
                "base": self.environment.base.to_list(),
                "floors": {
                    str(floor): {
                        "width": world.width,
                        "height": world.height,
                    }
                    for floor, world in sorted(self.environment.floors.items())
                },
                "transitions": [
                    transition.to_dict()
                    for transition in self.environment.transitions
                ],
            },
            "frames": self.frames,
            "result": final.to_dict(),
        }


def run_multi_floor_cli(
    *,
    floors: int,
    uncertainty_profile: str = "off",
    planning_variant: str = "naive",
    width: int,
    height: int,
    seed: int,
    drones: int,
    obstacle_density: float,
    survivors_per_floor: int,
    transition_cost: int,
    stairwells: int,
    max_steps: int,
    replay_out: str | None,
    failure_schedule: tuple[tuple[str, int], ...] = (),
    dynamic_obstacle_schedule: tuple[tuple[int, int, int, int], ...] = (),
) -> None:
    config = MultiFloorConfig(
        floor_count=floors,
        uncertainty_profile=uncertainty_profile,
        planning_variant=planning_variant,
        width=width,
        height=height,
        seed=seed,
        drone_count=drones,
        obstacle_density=obstacle_density,
        survivors_per_floor=survivors_per_floor,
        transition_cost=transition_cost,
        transition_energy_cost=float(transition_cost),
        stairwells_per_pair=stairwells,
        max_steps=max_steps,
        failure_schedule=failure_schedule,
        dynamic_obstacle_schedule=dynamic_obstacle_schedule,
    )
    simulation = MultiFloorSimulation(config)
    result = simulation.run()
    if replay_out is not None:
        Path(replay_out).write_text(
            json.dumps(simulation.replay(result), indent=2) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(result.to_dict(), indent=2))


def metric_summary(values: list[float]) -> dict[str, float]:
    ordered = sorted(values)
    middle = len(ordered) // 2
    median = (
        ordered[middle]
        if len(ordered) % 2
        else (ordered[middle - 1] + ordered[middle]) / 2
    )
    return {
        "mean": round(sum(values) / len(values), 6),
        "population_std": round(pstdev(values), 6),
        "median": round(median, 6),
        "min": round(min(values), 6),
        "max": round(max(values), 6),
    }
