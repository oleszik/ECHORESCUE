from collections.abc import Callable
from dataclasses import dataclass, field

from echorescue.config import SimulationConfig
from echorescue.communication import (
    CommunicationModel,
    CommunicationSnapshot,
    DroneConnection,
)
from echorescue.coordination import (
    FrontierAssignment,
    assign_frontiers,
    resolve_movements,
)
from echorescue.deconfliction import (
    IntentConflict,
    MotionIntent,
    ProximitySensor,
    detect_intent_conflict,
    priority_key,
    proximity_risk,
)
from echorescue.energy import Battery
from echorescue.environment import GridWorld
from echorescue.events import EventType, MissionEvent, MissionLog
from echorescue.knowledge import KnowledgeMap
from echorescue.map_sync import ShadowMapSynchronizer
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
    local_map: KnowledgeMap
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
    detected_survivors: set[Position] = field(default_factory=set)
    confirmed_survivors: set[Position] = field(default_factory=set)
    last_survivor_observation_step: dict[Position, int] = field(
        default_factory=dict
    )
    return_replan_required: bool = False
    yielding: bool = False
    yield_steps: int = 0
    consecutive_yield_steps: int = 0

    @property
    def terminal(self) -> bool:
        return self.drone.status in TERMINAL_STATUSES


@dataclass(frozen=True, slots=True)
class MultiSimulationResult:
    seed: int
    knowledge_mode: str
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
    communication_uptime_by_drone: dict[str, float]
    direct_base_uptime_by_drone: dict[str, float]
    relay_uptime_by_drone: dict[str, float]
    communication_outages_by_drone: dict[str, int]
    longest_outage_by_drone: dict[str, int]
    local_known_coverage_by_drone: dict[str, float]
    base_known_coverage: float
    shared_shadow_coverage: float
    map_divergence_between_drones: float
    peak_map_divergence_between_drones: float
    stale_cells_by_drone: dict[str, int]
    cells_uploaded_by_drone: dict[str, int]
    cells_received_by_drone: dict[str, int]
    map_sync_events: int
    time_to_map_convergence: int | None
    survivor_recall_at_base: float
    local_survivors_detected_by_drone: dict[str, int]
    local_survivors_confirmed_by_drone: dict[str, int]
    base_survivors_detected: int
    base_survivors_confirmed: int
    safety_shield_interventions: int
    redundant_frontier_assignments: int
    targets_discarded_after_reconnect: int
    local_replanning_by_drone: dict[str, int]
    unique_cells_transferred: int
    semantic_cell_changes_transferred: int
    local_motion_conflicts: int
    communication_detected_conflicts: int
    proximity_detected_conflicts: int
    yield_steps_by_drone: dict[str, int]
    corridor_deadlocks: int
    deadlocks_resolved: int
    local_replans_due_to_drones: int
    deconfliction_delay_steps: int
    mission_success: bool
    mission_events: tuple[MissionEvent, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "seed": self.seed,
            "knowledge_mode": self.knowledge_mode,
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
            "communication_uptime_by_drone": {
                drone_id: round(uptime, 6)
                for drone_id, uptime in (
                    self.communication_uptime_by_drone.items()
                )
            },
            "direct_base_uptime_by_drone": {
                drone_id: round(uptime, 6)
                for drone_id, uptime in self.direct_base_uptime_by_drone.items()
            },
            "relay_uptime_by_drone": {
                drone_id: round(uptime, 6)
                for drone_id, uptime in self.relay_uptime_by_drone.items()
            },
            "communication_outages_by_drone": dict(
                self.communication_outages_by_drone
            ),
            "longest_outage_by_drone": dict(self.longest_outage_by_drone),
            "local_known_coverage_by_drone": {
                drone_id: round(coverage, 6)
                for drone_id, coverage in (
                    self.local_known_coverage_by_drone.items()
                )
            },
            "base_known_coverage": round(self.base_known_coverage, 6),
            "shared_shadow_coverage": round(self.shared_shadow_coverage, 6),
            "map_divergence_between_drones": round(
                self.map_divergence_between_drones, 6
            ),
            "peak_map_divergence_between_drones": round(
                self.peak_map_divergence_between_drones, 6
            ),
            "stale_cells_by_drone": dict(self.stale_cells_by_drone),
            "cells_uploaded_by_drone": dict(self.cells_uploaded_by_drone),
            "cells_received_by_drone": dict(self.cells_received_by_drone),
            "map_sync_events": self.map_sync_events,
            "time_to_map_convergence": self.time_to_map_convergence,
            "survivor_recall_at_base": round(
                self.survivor_recall_at_base, 6
            ),
            "local_survivors_detected_by_drone": dict(
                self.local_survivors_detected_by_drone
            ),
            "local_survivors_confirmed_by_drone": dict(
                self.local_survivors_confirmed_by_drone
            ),
            "base_survivors_detected": self.base_survivors_detected,
            "base_survivors_confirmed": self.base_survivors_confirmed,
            "safety_shield_interventions": self.safety_shield_interventions,
            "redundant_frontier_assignments": (
                self.redundant_frontier_assignments
            ),
            "targets_discarded_after_reconnect": (
                self.targets_discarded_after_reconnect
            ),
            "local_replanning_by_drone": dict(
                self.local_replanning_by_drone
            ),
            "unique_cells_transferred": self.unique_cells_transferred,
            "semantic_cell_changes_transferred": (
                self.semantic_cell_changes_transferred
            ),
            "local_motion_conflicts": self.local_motion_conflicts,
            "communication_detected_conflicts": (
                self.communication_detected_conflicts
            ),
            "proximity_detected_conflicts": (
                self.proximity_detected_conflicts
            ),
            "yield_steps_by_drone": dict(self.yield_steps_by_drone),
            "corridor_deadlocks": self.corridor_deadlocks,
            "deadlocks_resolved": self.deadlocks_resolved,
            "local_replans_due_to_drones": (
                self.local_replans_due_to_drones
            ),
            "deconfliction_delay_steps": self.deconfliction_delay_steps,
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
        self.knowledge_mode = config.effective_knowledge_mode
        self.world = GridWorld.generate(config)
        self.occupancy_map = OccupancyMap(config.width, config.height)
        self.sensor = DistanceSensor(config.sensor_range)
        self.survivor_sensor = SurvivorSensor(config.survivor_sensor_range)
        self.communication_model = CommunicationModel(
            config.communication_range
        )
        self.proximity_sensor = ProximitySensor(
            config.proximity_sensor_range
        )
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
        self._base_detected_survivors: set[Position] = set()
        self._base_confirmed_survivors: set[Position] = set()
        self._visited_by_cell: dict[Position, set[str]] = {}
        self.communication_snapshot: CommunicationSnapshot
        self._communication_samples = 0
        self._communication_connected_samples: dict[str, int] = {}
        self._communication_direct_samples: dict[str, int] = {}
        self._communication_relay_samples: dict[str, int] = {}
        self._communication_outages: dict[str, int] = {}
        self._current_outage_steps: dict[str, int] = {}
        self._longest_outage_steps: dict[str, int] = {}
        self._cells_uploaded_by_drone: dict[str, int] = {}
        self._cells_received_by_drone: dict[str, int] = {}
        self._map_sync_events = 0
        self._time_to_map_convergence: int | None = None
        self._peak_map_divergence = 0.0
        self._shadow_maps_converged = True
        self._map_sync_session_active = False
        self._unique_cells_transferred: set[Position] = set()
        self._semantic_cell_changes_transferred = 0
        self.safety_shield_interventions = 0
        self.redundant_frontier_assignments = 0
        self.targets_discarded_after_reconnect = 0
        self._local_replanning_by_drone: dict[str, int] = {}
        self._peer_connected_at_last_allocation = True
        self._pending_reconnect_targets: set[str] = set()
        self.motion_intents: dict[str, MotionIntent] = {}
        self.local_motion_conflicts = 0
        self.communication_detected_conflicts = 0
        self.proximity_detected_conflicts = 0
        self.corridor_deadlocks = 0
        self.deadlocks_resolved = 0
        self.local_replans_due_to_drones = 0
        self.deconfliction_delay_steps = 0
        self._active_deconfliction_signature: tuple[object, ...] | None = None
        self._deconfliction_repeat_count = 0
        self._deadlock_reported_for_signature = False
        self._intent_sharing_active: dict[str, bool] = {}
        self._last_shared_intent_status: dict[str, DroneStatus] = {}
        self._last_communicated_intents: dict[str, MotionIntent] = {}

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
                local_map=KnowledgeMap(config.width, config.height),
                position_trace=[position],
            )
            self.runtimes[drone_id] = runtime
            if position != self.world.base:
                self._visited_by_cell.setdefault(position, set()).add(drone_id)

        self.base_knowledge_map = (
            KnowledgeMap(config.width, config.height)
            if config.base_knowledge_store_enabled
            else None
        )
        self.shadow_synchronizer = ShadowMapSynchronizer(
            {
                drone_id: runtime.local_map
                for drone_id, runtime in self.runtimes.items()
            },
            self.base_knowledge_map,
        )

        for runtime in self._ordered_runtimes():
            self._sense(runtime)
            if not runtime.terminal:
                self._refresh_return_estimate(runtime)
        self._sample_communication(record_events=False)
        self._sync_shadow_maps()
        self._sync_survivor_knowledge()
        if self.knowledge_mode == "local":
            self._record_base_event(EventType.KNOWLEDGE_MODE_ACTIVATED)
        self._update_completion()

    @property
    def drones(self) -> tuple[Drone, ...]:
        return tuple(runtime.drone for runtime in self._ordered_runtimes())

    @property
    def confirmed_survivors(self) -> frozenset[Position]:
        if self.knowledge_mode == "local":
            return frozenset(self._base_confirmed_survivors)
        return frozenset(self._confirmed_survivors)

    @property
    def detected_survivors(self) -> frozenset[Position]:
        if self.knowledge_mode == "local":
            return frozenset(self._base_detected_survivors)
        return frozenset(self._detected_survivors)

    @property
    def knowledge_sync_enabled(self) -> bool:
        return self.knowledge_mode in {"shadow", "local"}

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
        visible_ids = None
        if self.knowledge_mode == "local":
            visible_ids = set(self._communication_component(runtime.drone.identifier))
        return {
            other.drone.position
            for other in self._ordered_runtimes()
            if (
                other.drone.identifier != runtime.drone.identifier
                and (
                    visible_ids is None
                    or other.drone.identifier in visible_ids
                )
                and other.drone.status is not DroneStatus.LANDED
                and other.drone.position != self.world.base
            )
        }

    def _decision_map(self, runtime: DroneRuntime) -> object:
        if self.knowledge_mode == "local":
            return runtime.local_map
        return self.occupancy_map

    def _communication_component(self, node_id: str) -> tuple[str, ...]:
        if not hasattr(self, "communication_snapshot"):
            return (node_id,)
        for component in self.shadow_synchronizer.connected_components(
            self.communication_snapshot
        ):
            if node_id in component:
                return component
        return (node_id,)

    def _known_return_path(
        self,
        runtime: DroneRuntime,
        origin: Position | None = None,
        avoid_other_drones: bool = True,
    ) -> tuple[Position, ...] | None:
        start = origin or runtime.drone.position
        blocked = self._other_blockers(runtime) if avoid_other_drones else set()
        decision_map = self._decision_map(runtime)

        def passable(position: Position) -> bool:
            if position == self.world.base:
                return decision_map.is_known_free(position)
            return decision_map.is_known_free(position) and (
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
        decision_map = self._decision_map(runtime)
        return (
            bool(path)
            and path[0] == runtime.drone.position
            and path[-1] == self.world.base
            and all(
                decision_map.is_known_free(position) for position in path
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
        cell_count: int | None = None,
    ) -> None:
        self.mission_log.record(
            MissionEvent(
                position=position or runtime.drone.position,
                step=self.steps,
                drone_id=runtime.drone.identifier,
                event_type=event_type,
                energy_remaining=runtime.battery.remaining,
                cell_count=cell_count,
            )
        )

    def _record_base_event(
        self,
        event_type: EventType,
        cell_count: int | None = None,
    ) -> None:
        self.mission_log.record(
            MissionEvent(
                position=self.world.base,
                step=self.steps,
                drone_id="base",
                event_type=event_type,
                cell_count=cell_count,
            )
        )

    def _sample_communication(self, *, record_events: bool = True) -> None:
        """Observe radio state without feeding it back into mission behavior."""

        previous = getattr(self, "communication_snapshot", None)
        snapshot = self.communication_model.compute(
            self.world,
            self.world.base,
            {
                runtime.drone.identifier: runtime.drone.position
                for runtime in self._ordered_runtimes()
            },
        )
        self.communication_snapshot = snapshot
        self._communication_samples += 1

        if self.knowledge_mode == "local" and previous is not None:
            def peers_share_component(candidate: CommunicationSnapshot) -> bool:
                return any(
                    {"drone-1", "drone-2"}.issubset(component)
                    for component in self.shadow_synchronizer.connected_components(
                        candidate
                    )
                )

            if (
                not peers_share_component(previous)
                and peers_share_component(snapshot)
            ):
                for runtime in self._ordered_runtimes():
                    target = runtime.active_frontier_target
                    if (
                        target is not None
                        and target in runtime.local_map.frontiers()
                    ):
                        self._pending_reconnect_targets.add(
                            runtime.drone.identifier
                        )

        for runtime in self._ordered_runtimes():
            drone_id = runtime.drone.identifier
            connection = snapshot.connections[drone_id]
            self._communication_connected_samples[drone_id] = (
                self._communication_connected_samples.get(drone_id, 0)
                + int(connection.connected_to_base)
            )
            self._communication_direct_samples[drone_id] = (
                self._communication_direct_samples.get(drone_id, 0)
                + int(connection.direct_to_base)
            )
            self._communication_relay_samples[drone_id] = (
                self._communication_relay_samples.get(drone_id, 0)
                + int(connection.via_relay)
            )

            if connection.connected_to_base:
                self._current_outage_steps[drone_id] = 0
            else:
                if self._current_outage_steps.get(drone_id, 0) == 0:
                    self._communication_outages[drone_id] = (
                        self._communication_outages.get(drone_id, 0) + 1
                    )
                current = self._current_outage_steps.get(drone_id, 0) + 1
                self._current_outage_steps[drone_id] = current
                self._longest_outage_steps[drone_id] = max(
                    self._longest_outage_steps.get(drone_id, 0), current
                )

            if not record_events or previous is None:
                continue
            old_connection: DroneConnection = previous.connections[drone_id]
            if (
                old_connection.connected_to_base
                and not connection.connected_to_base
            ):
                self._record_event(runtime, EventType.COMMUNICATION_LOST)
            elif (
                not old_connection.connected_to_base
                and connection.connected_to_base
            ):
                self._record_event(runtime, EventType.COMMUNICATION_RESTORED)
            if not old_connection.via_relay and connection.via_relay:
                self._record_event(runtime, EventType.RELAY_LINK_ESTABLISHED)
            elif old_connection.via_relay and not connection.via_relay:
                self._record_event(runtime, EventType.RELAY_LINK_LOST)

    def _sync_shadow_maps(self) -> None:
        if not self.knowledge_sync_enabled:
            return

        previous_converged = self._shadow_maps_converged
        self._peak_map_divergence = max(
            self._peak_map_divergence,
            self.shadow_synchronizer.divergence_ratio(),
        )
        report = self.shadow_synchronizer.sync(self.communication_snapshot)
        current_converged = self.shadow_synchronizer.maps_converged()
        self._peak_map_divergence = max(
            self._peak_map_divergence,
            self.shadow_synchronizer.divergence_ratio(),
        )

        if not report.sync_available:
            self._map_sync_session_active = False
        if report.transfer_occurred:
            self._map_sync_events += 1
            self._unique_cells_transferred.update(
                report.transferred_positions
            )
            self._semantic_cell_changes_transferred += (
                report.semantic_cell_changes
            )
            if not self._map_sync_session_active:
                self._record_base_event(EventType.MAP_SYNC_STARTED)
            self._map_sync_session_active = True
            for runtime in self._ordered_runtimes():
                drone_id = runtime.drone.identifier
                uploaded = report.uploaded_by_drone[drone_id]
                received = report.received_by_drone[drone_id]
                self._cells_uploaded_by_drone[drone_id] = (
                    self._cells_uploaded_by_drone.get(drone_id, 0) + uploaded
                )
                self._cells_received_by_drone[drone_id] = (
                    self._cells_received_by_drone.get(drone_id, 0) + received
                )
                if uploaded:
                    self._record_event(
                        runtime,
                        EventType.MAP_CELLS_UPLOADED,
                        cell_count=uploaded,
                    )
                if received:
                    self._record_event(
                        runtime,
                        EventType.MAP_CELLS_RECEIVED,
                        cell_count=received,
                    )
            if not previous_converged and current_converged:
                self._record_base_event(EventType.MAP_CONVERGED)
                if self._time_to_map_convergence is None:
                    self._time_to_map_convergence = self.steps

        self._shadow_maps_converged = current_converged

    def _sync_survivor_knowledge(self) -> None:
        if self.knowledge_mode != "local":
            return
        for component in self.shadow_synchronizer.connected_components(
            self.communication_snapshot
        ):
            drone_ids = [
                node_id for node_id in component if node_id in self.runtimes
            ]
            include_base = "base" in component
            if len(drone_ids) + int(include_base) < 2:
                continue
            detected = set()
            confirmed = set()
            for drone_id in drone_ids:
                runtime = self.runtimes[drone_id]
                detected.update(runtime.detected_survivors)
                confirmed.update(runtime.confirmed_survivors)
            if include_base:
                detected.update(self._base_detected_survivors)
                confirmed.update(self._base_confirmed_survivors)

            for drone_id in drone_ids:
                runtime = self.runtimes[drone_id]
                new_confirmed = confirmed - runtime.confirmed_survivors
                runtime.detected_survivors.update(detected)
                runtime.confirmed_survivors.update(confirmed)
                for position in sorted(new_confirmed):
                    self._record_event(
                        runtime,
                        EventType.SURVIVOR_KNOWLEDGE_SYNCHRONIZED,
                        position,
                    )
            if include_base:
                new_base_confirmed = confirmed - self._base_confirmed_survivors
                self._base_detected_survivors.update(detected)
                self._base_confirmed_survivors.update(confirmed)
                for position in sorted(new_base_confirmed):
                    self.mission_log.record(
                        MissionEvent(
                            position=position,
                            step=self.steps,
                            drone_id="base",
                            event_type=(
                                EventType.SURVIVOR_KNOWLEDGE_SYNCHRONIZED
                            ),
                        )
                    )

    def _sense(self, runtime: DroneRuntime) -> None:
        if runtime.terminal:
            return
        if not runtime.battery.consume(self.config.sensor_energy_cost):
            self._fail_energy(runtime)
            return
        observations = self.sensor.observe(self.world, runtime.drone.position)
        if self.knowledge_sync_enabled:
            runtime.local_map.observe(
                observations,
                step=self.steps,
                source_id=runtime.drone.identifier,
            )
        if self.knowledge_mode != "local":
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
            if self.knowledge_mode == "local":
                if position not in runtime.detected_survivors:
                    runtime.detected_survivors.add(position)
                    self._detected_survivors.add(position)
                    self._record_event(
                        runtime, EventType.SURVIVOR_DETECTED, position
                    )
                if (
                    count >= self.config.survivor_confirmation_observations
                    and position not in runtime.confirmed_survivors
                ):
                    runtime.confirmed_survivors.add(position)
                    self._confirmed_survivors.add(position)
                    self._record_event(
                        runtime, EventType.SURVIVOR_CONFIRMED, position
                    )
                continue
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
        if self.knowledge_mode == "local":
            return False
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
                    if (
                        self.knowledge_mode == "local"
                        or self._objectives_complete()
                    ):
                        self._start_return(runtime, return_path)
                    else:
                        self._fail_energy(runtime)
                else:
                    self._start_return(runtime, return_path)

    def _allocate_frontiers(self) -> dict[str, FrontierAssignment]:
        if self.knowledge_mode == "local":
            return self._allocate_local_frontiers()
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

    def _allocate_local_frontiers(self) -> dict[str, FrontierAssignment]:
        explorers = {
            runtime.drone.identifier: runtime.drone.position
            for runtime in self._ordered_runtimes()
            if runtime.drone.status is DroneStatus.EXPLORE
        }
        if not explorers:
            return {}

        components = []
        for component in self.shadow_synchronizer.connected_components(
            self.communication_snapshot
        ):
            members = tuple(
                drone_id for drone_id in component if drone_id in explorers
            )
            if members:
                components.append(members)
        peer_connected = any(len(component) == 2 for component in components)
        assignments: dict[str, FrontierAssignment] = {}
        changed_targets: set[str] = set()
        any_frontiers = False

        for component in components:
            # Synchronization has already made every radio-connected member's
            # store identical. Coordination therefore reads one actual local
            # map, never the global evaluation map or an implicit merged view.
            component_map = self.runtimes[min(component)].local_map
            frontiers = component_map.frontiers()
            any_frontiers = any_frontiers or bool(frontiers)
            frontier_set = set(frontiers)
            current_targets = {
                drone_id: self.runtimes[drone_id].active_frontier_target
                for drone_id in component
            }

            claimed_targets: set[Position] = set()
            for drone_id in sorted(component):
                runtime = self.runtimes[drone_id]
                target = runtime.active_frontier_target
                stale = (
                    target is not None
                    and (
                        target not in frontier_set
                        or target in claimed_targets
                    )
                )
                if stale:
                    self._record_event(
                        runtime, EventType.STALE_TARGET_DISCARDED, target
                    )
                    runtime.active_frontier_target = None
                    runtime.planned_path = ()
                    current_targets[drone_id] = None
                    self._local_replanning_by_drone[drone_id] = (
                        self._local_replanning_by_drone.get(drone_id, 0) + 1
                    )
                    if (
                        drone_id in self._pending_reconnect_targets
                        or target in claimed_targets
                    ):
                        self.targets_discarded_after_reconnect += 1
                elif target is not None:
                    claimed_targets.add(target)

            if not frontiers:
                for drone_id in component:
                    runtime = self.runtimes[drone_id]
                    path = self._known_return_path(runtime)
                    if path is None:
                        path = self._known_return_path(
                            runtime, avoid_other_drones=False
                        )
                    if path is None:
                        self._fail_return_path(runtime)
                    else:
                        self._start_return(runtime, path)
                continue

            component_assignments = assign_frontiers(
                {drone_id: explorers[drone_id] for drone_id in component},
                frontiers,
                component_map,
                current_targets,
            )
            assignments.update(component_assignments)
            for drone_id in component:
                runtime = self.runtimes[drone_id]
                old_target = runtime.active_frontier_target
                assignment = component_assignments.get(drone_id)
                new_target = assignment.target if assignment is not None else None
                runtime.active_frontier_target = new_target
                runtime.planned_path = (
                    assignment.path if assignment is not None else ()
                )
                if new_target is not None and new_target != old_target:
                    if old_target is not None:
                        self._local_replanning_by_drone[drone_id] = (
                            self._local_replanning_by_drone.get(drone_id, 0) + 1
                        )
                    runtime.frontier_assignments += 1
                    changed_targets.add(drone_id)
                    self._record_event(
                        runtime,
                        EventType.LOCAL_FRONTIER_SELECTED,
                        new_target,
                    )

        if not peer_connected:
            targets: dict[Position, list[str]] = {}
            for drone_id, runtime in self.runtimes.items():
                if runtime.active_frontier_target is not None:
                    targets.setdefault(
                        runtime.active_frontier_target, []
                    ).append(drone_id)
            for drone_ids in targets.values():
                if len(drone_ids) > 1 and changed_targets.intersection(drone_ids):
                    self.redundant_frontier_assignments += len(drone_ids) - 1

        self._peer_connected_at_last_allocation = peer_connected
        self._pending_reconnect_targets.clear()
        if not any_frontiers:
            self._exploration_complete = True
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
            if (
                runtime.drone.position == self.world.base
                and self.knowledge_mode != "local"
                and not self._objectives_complete()
            ):
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

    def _intent_reservation(
        self,
        runtime: DroneRuntime,
        next_position: Position,
    ) -> tuple[Position, ...]:
        path = (
            runtime.current_return_path
            if runtime.drone.status is DroneStatus.RETURN_HOME
            else runtime.planned_path
        )
        future: list[Position] = []
        if next_position != runtime.drone.position:
            future.append(next_position)
        if runtime.drone.position in path:
            start = path.index(runtime.drone.position) + 1
            for position in path[start:]:
                if not future or position != future[-1]:
                    future.append(position)
        elif next_position in path:
            start = path.index(next_position) + 1
            for position in path[start:]:
                if position != future[-1]:
                    future.append(position)
        if not future:
            future.append(runtime.drone.position)
        return tuple(future[: self.config.intent_reservation_steps])

    def _motion_intent(
        self,
        runtime: DroneRuntime,
        next_position: Position,
    ) -> MotionIntent:
        estimated_return = runtime.estimated_return_energy
        safe_margin = (
            runtime.battery.remaining
            - (estimated_return if estimated_return is not None else 0.0)
            - self.config.energy_safety_reserve
        )
        return MotionIntent(
            drone_id=runtime.drone.identifier,
            current_position=runtime.drone.position,
            next_position=next_position,
            reservation=self._intent_reservation(runtime, next_position),
            status=runtime.drone.status,
            energy_remaining=runtime.battery.remaining,
            safe_energy_margin=safe_margin,
            valid_until_step=self.steps + self.config.motion_intent_ttl,
        )

    def _peer_intents_communicated(self) -> bool:
        return "drone-2" in self._communication_component("drone-1")

    def _record_intent_sharing(self, communicated: bool) -> None:
        for drone_id, intent in sorted(self.motion_intents.items()):
            active = self._intent_sharing_active.get(drone_id, False)
            status_changed = (
                self._last_shared_intent_status.get(drone_id)
                is not intent.status
            )
            if communicated and (not active or status_changed):
                self._record_event(
                    self.runtimes[drone_id],
                    EventType.MOTION_INTENT_SHARED,
                    intent.next_position,
                )
            self._intent_sharing_active[drone_id] = communicated
            if communicated:
                self._last_shared_intent_status[drone_id] = intent.status
                self._last_communicated_intents[drone_id] = intent

    def _avoidance_step(
        self,
        runtime: DroneRuntime,
        raw_next: Position,
        forbidden: set[Position],
    ) -> Position | None:
        decision_map = runtime.local_map
        target = (
            self.world.base
            if runtime.drone.status is DroneStatus.RETURN_HOME
            else runtime.active_frontier_target
        )
        candidates = []
        for candidate in runtime.drone.position.neighbors():
            if (
                candidate == raw_next
                or candidate in forbidden
                or not decision_map.is_known_free(candidate)
            ):
                continue

            path_length = 0
            if target is not None:
                path = astar(
                    candidate,
                    target,
                    lambda position: (
                        decision_map.is_known_free(position)
                        and position not in forbidden
                    ),
                )
                if path is None:
                    continue
                path_length = len(path)
            free_neighbors = sum(
                decision_map.is_known_free(neighbor)
                for neighbor in candidate.neighbors()
            )
            candidates.append(
                (-free_neighbors, path_length, candidate.y, candidate.x, candidate)
            )
        return min(candidates)[-1] if candidates else None

    def _finish_yield_states(
        self,
        yielding_now: set[str],
        delayed_now: set[str],
    ) -> None:
        for runtime in self._ordered_runtimes():
            drone_id = runtime.drone.identifier
            if drone_id in yielding_now:
                if not runtime.yielding:
                    self._record_event(runtime, EventType.YIELD_STARTED)
                runtime.yielding = True
                runtime.consecutive_yield_steps += 1
                if drone_id in delayed_now:
                    runtime.yield_steps += 1
            elif runtime.yielding:
                runtime.yielding = False
                runtime.consecutive_yield_steps = 0
                self._record_event(runtime, EventType.YIELD_ENDED)

    def _deconflict_intentions(
        self,
        intentions: dict[str, Position],
    ) -> dict[str, Position]:
        if (
            self.knowledge_mode != "local"
            or not self.config.distributed_deconfliction_enabled
            or len(intentions) < 2
        ):
            self.motion_intents = {
                drone_id: self._motion_intent(
                    self.runtimes[drone_id], destination
                )
                for drone_id, destination in sorted(intentions.items())
            }
            self._finish_yield_states(set(), set())
            return intentions

        self.motion_intents = {
            drone_id: self._motion_intent(
                self.runtimes[drone_id], destination
            )
            for drone_id, destination in sorted(intentions.items())
        }
        drone_ids = tuple(sorted(self.motion_intents))
        first_id, second_id = drone_ids
        first = self.motion_intents[first_id]
        second = self.motion_intents[second_id]
        communicated = self._peer_intents_communicated()
        self._record_intent_sharing(communicated)

        conflict: IntentConflict | None = None
        source: str | None = None
        proximity_risks: dict[str, str] = {}
        if communicated:
            conflict = detect_intent_conflict(first, second, self.world.base)
            if conflict is not None:
                source = "communication"
        else:
            visible = self.proximity_sensor.can_detect(
                self.world,
                first.current_position,
                second.current_position,
            )
            if visible:
                for intent, other_position in (
                    (first, second.current_position),
                    (second, first.current_position),
                ):
                    risk = proximity_risk(intent, other_position)
                    if risk is not None:
                        proximity_risks[intent.drone_id] = risk
            if proximity_risks:
                source = "proximity"
                kind = sorted(proximity_risks.values())[0]
                position = min(
                    self.motion_intents[drone_id].next_position
                    for drone_id in proximity_risks
                )
                conflict = IntentConflict(kind, position, drone_ids)

        if conflict is None or source is None:
            self._active_deconfliction_signature = None
            self._deconfliction_repeat_count = 0
            self._deadlock_reported_for_signature = False
            self._finish_yield_states(set(), set())
            return intentions

        self.local_motion_conflicts += 1
        if source == "communication":
            self.communication_detected_conflicts += 1
            winner_id = min(
                drone_ids,
                key=lambda drone_id: priority_key(
                    self.motion_intents[drone_id],
                    self.runtimes[drone_id].consecutive_yield_steps,
                ),
            )
            loser_id = next(
                drone_id for drone_id in drone_ids if drone_id != winner_id
            )
        else:
            if source == "proximity":
                self.proximity_detected_conflicts += 1
            if len(proximity_risks) == 1:
                loser_id = next(iter(proximity_risks))
                winner_id = next(
                    drone_id for drone_id in drone_ids if drone_id != loser_id
                )
            else:
                winner_id = min(
                    drone_ids,
                    key=lambda drone_id: priority_key(
                        self.motion_intents[drone_id],
                        self.runtimes[drone_id].consecutive_yield_steps,
                    ),
                )
                loser_id = next(
                    drone_id
                    for drone_id in drone_ids
                    if drone_id != winner_id
                )

        signature = (
            source,
            conflict.kind,
            tuple(
                (
                    drone_id,
                    self.motion_intents[drone_id].current_position,
                    self.motion_intents[drone_id].next_position,
                )
                for drone_id in drone_ids
            ),
        )
        if signature == self._active_deconfliction_signature:
            self._deconfliction_repeat_count += 1
        else:
            self._active_deconfliction_signature = signature
            self._deconfliction_repeat_count = 1
            self._deadlock_reported_for_signature = False
            self._record_event(
                self.runtimes[loser_id],
                EventType.LOCAL_COLLISION_AVOIDED,
                conflict.position,
            )

        resolved = dict(intentions)
        yielding_now = {loser_id}
        delayed_now = {loser_id}
        resolved[loser_id] = self.motion_intents[loser_id].current_position
        # Both drones must stop only for an immediate swap or when the
        # priority winner would enter the loser's currently occupied cell.
        # A future head-on reservation is resolved by yielding the loser now;
        # stopping both sides would recreate the same reservation forever.
        blocking_conflict = (
            conflict.kind == "edge_swap"
            or self.motion_intents[winner_id].next_position
            == self.motion_intents[loser_id].current_position
        )
        if blocking_conflict:
            resolved[winner_id] = self.motion_intents[winner_id].current_position
            yielding_now.add(winner_id)
            delayed_now.add(winner_id)

        if self._deconfliction_repeat_count >= self.config.deadlock_wait_threshold:
            if not self._deadlock_reported_for_signature:
                self.corridor_deadlocks += 1
                self._deadlock_reported_for_signature = True
                self._record_event(
                    self.runtimes[loser_id],
                    EventType.CORRIDOR_DEADLOCK_DETECTED,
                    conflict.position,
                )
            other_intent = self.motion_intents[winner_id]
            forbidden = {
                other_intent.current_position,
                other_intent.next_position,
                *other_intent.reservation,
            }
            loser_runtime = self.runtimes[loser_id]
            avoidance = self._avoidance_step(
                loser_runtime,
                self.motion_intents[loser_id].next_position,
                forbidden,
            )
            self.local_replans_due_to_drones += 1
            if avoidance is not None:
                resolved[loser_id] = avoidance
                delayed_now.discard(loser_id)
                resolved[winner_id] = other_intent.current_position
                yielding_now.add(winner_id)
                delayed_now.add(winner_id)
                self.deadlocks_resolved += 1
                self._record_event(
                    loser_runtime,
                    EventType.DEADLOCK_REPLANNED,
                    avoidance,
                )
            else:
                if loser_runtime.drone.status is DroneStatus.RETURN_HOME:
                    loser_runtime.return_replan_required = True
                else:
                    loser_runtime.active_frontier_target = None
                    loser_runtime.planned_path = ()
                self._record_event(
                    loser_runtime, EventType.DEADLOCK_REPLANNED
                )

        if delayed_now:
            self.deconfliction_delay_steps += 1
        self._finish_yield_states(yielding_now, delayed_now)
        return resolved

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
            if self.knowledge_mode == "local":
                self.safety_shield_interventions += 1
                self._record_event(
                    runtime, EventType.SAFETY_SHIELD_INTERVENTION
                )
            else:
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
            if not self._decision_map(runtime).is_known_free(destination):
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
        intentions = self._deconflict_intentions(intentions)
        previous_step = self.steps
        self._execute_intentions(intentions)
        if self.steps != previous_step:
            self._sample_communication()
            self._sync_shadow_maps()
            self._sync_survivor_knowledge()
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
        reported_detected = (
            self._base_detected_survivors
            if self.knowledge_mode == "local"
            else self._detected_survivors
        )
        reported_confirmed = (
            self._base_confirmed_survivors
            if self.knowledge_mode == "local"
            else self._confirmed_survivors
        )
        survivor_recall = (
            len(reported_confirmed) / survivors_total
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
            len(reported_confirmed) == survivors_total
            and self.collisions == 0
            and self.drone_drone_collisions == 0
            and drones_returned == len(self.runtimes)
        )
        shared_shadow_map = self.shadow_synchronizer.shared_shadow_map()
        evaluation_known_cells = (
            shared_shadow_map.known_cell_count
            if self.knowledge_mode == "local"
            else self.occupancy_map.known_cell_count
        )
        evaluation_explored_percent = (
            shared_shadow_map.known_coverage
            if self.knowledge_mode == "local"
            else self.occupancy_map.explored_percent
        )
        return MultiSimulationResult(
            seed=self.config.seed,
            knowledge_mode=self.knowledge_mode,
            completed=self.completed,
            termination_reason=self.termination_reason,
            steps=self.steps,
            known_cells=evaluation_known_cells,
            explored_percent=evaluation_explored_percent,
            collisions=self.collisions,
            survivors_total=survivors_total,
            survivors_detected=len(reported_detected),
            survivors_confirmed=len(reported_confirmed),
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
            communication_uptime_by_drone={
                drone_id: (
                    self._communication_connected_samples[drone_id]
                    / self._communication_samples
                )
                for drone_id in sorted(self.runtimes)
            },
            direct_base_uptime_by_drone={
                drone_id: (
                    self._communication_direct_samples[drone_id]
                    / self._communication_samples
                )
                for drone_id in sorted(self.runtimes)
            },
            relay_uptime_by_drone={
                drone_id: (
                    self._communication_relay_samples[drone_id]
                    / self._communication_samples
                )
                for drone_id in sorted(self.runtimes)
            },
            communication_outages_by_drone={
                drone_id: self._communication_outages.get(drone_id, 0)
                for drone_id in sorted(self.runtimes)
            },
            longest_outage_by_drone={
                drone_id: self._longest_outage_steps.get(drone_id, 0)
                for drone_id in sorted(self.runtimes)
            },
            local_known_coverage_by_drone={
                drone_id: self.runtimes[drone_id].local_map.known_coverage
                for drone_id in sorted(self.runtimes)
            },
            base_known_coverage=(
                self.base_knowledge_map.known_coverage
                if self.base_knowledge_map is not None
                else 0.0
            ),
            shared_shadow_coverage=shared_shadow_map.known_coverage,
            map_divergence_between_drones=(
                self.shadow_synchronizer.divergence_ratio()
            ),
            peak_map_divergence_between_drones=self._peak_map_divergence,
            stale_cells_by_drone={
                drone_id: len(
                    self.runtimes[drone_id].local_map.stale_against(
                        shared_shadow_map
                    )
                )
                for drone_id in sorted(self.runtimes)
            },
            cells_uploaded_by_drone={
                drone_id: self._cells_uploaded_by_drone.get(drone_id, 0)
                for drone_id in sorted(self.runtimes)
            },
            cells_received_by_drone={
                drone_id: self._cells_received_by_drone.get(drone_id, 0)
                for drone_id in sorted(self.runtimes)
            },
            map_sync_events=self._map_sync_events,
            time_to_map_convergence=self._time_to_map_convergence,
            survivor_recall_at_base=(
                len(self._base_confirmed_survivors) / survivors_total
                if self.knowledge_mode == "local" and survivors_total
                else survivor_recall
            ),
            local_survivors_detected_by_drone={
                drone_id: len(self.runtimes[drone_id].detected_survivors)
                for drone_id in sorted(self.runtimes)
            },
            local_survivors_confirmed_by_drone={
                drone_id: len(self.runtimes[drone_id].confirmed_survivors)
                for drone_id in sorted(self.runtimes)
            },
            base_survivors_detected=(
                len(self._base_detected_survivors)
                if self.knowledge_mode == "local"
                else len(self._detected_survivors)
            ),
            base_survivors_confirmed=(
                len(self._base_confirmed_survivors)
                if self.knowledge_mode == "local"
                else len(self._confirmed_survivors)
            ),
            safety_shield_interventions=self.safety_shield_interventions,
            redundant_frontier_assignments=(
                self.redundant_frontier_assignments
            ),
            targets_discarded_after_reconnect=(
                self.targets_discarded_after_reconnect
            ),
            local_replanning_by_drone=dict(
                self._local_replanning_by_drone
            ),
            unique_cells_transferred=len(self._unique_cells_transferred),
            semantic_cell_changes_transferred=(
                self._semantic_cell_changes_transferred
            ),
            local_motion_conflicts=self.local_motion_conflicts,
            communication_detected_conflicts=(
                self.communication_detected_conflicts
            ),
            proximity_detected_conflicts=self.proximity_detected_conflicts,
            yield_steps_by_drone={
                drone_id: self.runtimes[drone_id].yield_steps
                for drone_id in sorted(self.runtimes)
            },
            corridor_deadlocks=self.corridor_deadlocks,
            deadlocks_resolved=self.deadlocks_resolved,
            local_replans_due_to_drones=self.local_replans_due_to_drones,
            deconfliction_delay_steps=self.deconfliction_delay_steps,
            mission_success=mission_success,
            mission_events=self.mission_log.events,
        )
