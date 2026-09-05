from collections.abc import Callable
from dataclasses import dataclass, field
from itertools import permutations
from typing import cast

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
from echorescue.knowledge import CellKnowledge, KnowledgeMap, ProbabilisticKnowledgeMap
from echorescue.probabilistic import UNCERTAINTY_PROFILES
from echorescue.map_sync import ShadowMapSynchronizer
from echorescue.mapping import KnownMap, OccupancyMap
from echorescue.models import CellState, Drone, DroneStatus, Position
from echorescue.multi_relay import (
    ConnectivityForecast,
    MultiRelayPlan,
    RelayDeployment,
    RelayTargetTopology,
    forecast_connectivity,
    relay_candidate_score,
    relay_target_topologies,
)
from echorescue.network_transport import (
    DeterministicNetworkTransport,
    MessageType,
    NetworkDelivery,
    shortest_route,
)
from echorescue.dynamic_obstacles import (
    DynamicObstacleEvent,
    moderate_schedule,
)
from echorescue.network_aware_relay import (
    RelayUtilityDecision,
    RelayUtilityInput,
    RelayUtilityWeights,
    evaluate_relay_utility,
)
from echorescue.planning import astar
from echorescue.perception import (
    HypothesisStatus,
    SurvivorHypothesisTracker,
    SurvivorHypothesis,
)
from echorescue.relay import RelayPlan, known_radio_link, select_relay_plan
from echorescue.roles import (
    ENERGY_PENALTY_INTERVAL,
    TASK_LOAD_PENALTY,
    AgentRole,
    MissionTask,
    TaskRegistry,
    TaskStatus,
    TaskType,
    initial_roles,
    preferred_role,
    role_mismatch_penalty,
)
from echorescue.sensors import DistanceSensor, uncertain_observations
from echorescue.survivors import SurvivorObservationReport, SurvivorSensor


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
    base_acknowledged_records: dict[Position, CellKnowledge] = field(
        default_factory=dict
    )
    base_acknowledged_survivors: set[Position] = field(default_factory=set)
    relay_target: Position | None = None
    relay_scout_id: str | None = None
    relay_served_ids: set[str] = field(default_factory=set)
    relay_upstream_id: str | None = None
    relay_downstream_id: str | None = None
    relay_scout_position: Position | None = None
    relay_plan: RelayPlan | None = None
    relay_started_step: int | None = None
    relay_role_steps: int = 0
    relay_path_length: int = 0
    relay_link_achieved: bool = False
    relay_payload_forwarded: bool = False
    relay_payload_positions: set[Position] = field(default_factory=set)
    relay_payload_survivors: set[Position] = field(default_factory=set)
    relay_energy_at_start: float | None = None
    relay_path_length_at_start: int = 0
    relay_outage_at_start: int = 0
    relay_cooldown_until_step: int = 0
    holding_for_relay: bool = False
    yield_hold_until_step: int = 0
    network_relay_utility: float | None = None
    network_relay_reason: str | None = None
    network_relay_critical_backlog: int = 0
    network_relay_expected_units: int = 0
    network_relay_forwarded_units: int = 0
    network_relay_backpressure: bool = False
    in_smoke: bool = False
    dynamic_replan_pending: bool = False
    dynamic_replan_target: Position | None = None
    dynamic_replan_old_path_length: int | None = None
    dynamic_replan_return: bool = False
    role: AgentRole = AgentRole.GENERALIST
    base_role: AgentRole = AgentRole.GENERALIST
    role_last_changed_step: int = 0
    recovery_task_id: str | None = None

    @property
    def terminal(self) -> bool:
        return self.drone.status in TERMINAL_STATUSES


@dataclass(frozen=True, slots=True)
class FailureRecoveryClaim:
    task_id: str
    failed_id: str
    assignee_id: str
    target: Position
    path: tuple[Position, ...]
    score: int
    path_cost: int
    task_load_penalty: int
    role_penalty: int
    energy_penalty: int


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
    relay_strategy: str
    relay_deployments: int
    successful_relay_deployments: int
    failed_relay_deployments: int
    relay_steps_by_drone: dict[str, int]
    relay_path_length_by_drone: dict[str, int]
    relay_unique_cells_forwarded: int
    relay_survivor_confirmations_forwarded: int
    relay_outages_shortened: int
    base_known_coverage_over_time: tuple[float, ...]
    time_to_first_base_survivor_confirmation: int | None
    time_to_all_base_survivor_confirmations: int | None
    relay_energy_consumed: float
    relay_mission_delay_steps: int
    network_profile: str
    network_messages_queued: int
    network_messages_delivered: int
    network_fragments_sent: int
    network_fragments_delivered: int
    network_fragments_lost: int
    network_messages_expired: int
    network_messages_dropped: int
    network_delivery_ratio: float | None
    network_transmission_attempts: int
    network_successful_transmission_attempts: int
    network_retransmission_attempts: int
    network_fragments_created: int
    network_fragment_attempt_delivery_ratio: float | None
    network_unique_fragment_eventual_delivery_ratio: float | None
    network_logical_message_completion_ratio: float | None
    network_routes_replanned: int
    network_mean_latency: float | None
    network_max_latency: int | None
    network_payload_units_delivered: int
    network_average_queue_size: float | None
    network_max_queue_size: int
    network_max_backlog_duration: int
    stale_motion_intents: int
    map_sync_mean_latency: float | None
    map_sync_max_latency: int | None
    survivor_knowledge_mean_latency: float | None
    relay_network_fragments: int
    relay_network_mean_latency: float | None
    proximity_avoidances_without_fresh_intent: int
    final_sync_started: bool
    final_sync_duration: int
    final_sync_retransmissions: int
    final_sync_survivor_confirmations_transferred: int
    final_sync_timeout: bool
    network_shield_cause_classification: dict[str, int]
    network_shield_geometry_classification: dict[str, int]
    network_aware_relay_metrics: dict[str, object] | None
    failure_recovery_metrics: dict[str, object] | None
    smoke_metrics: dict[str, object] | None
    perception_metrics: dict[str, object] | None
    dynamic_obstacle_metrics: dict[str, object] | None
    role_failure_metrics: dict[str, object] | None
    multi_relay_metrics: dict[str, object] | None
    mission_success: bool
    mission_events: tuple[MissionEvent, ...]

    def to_dict(self) -> dict[str, object]:
        payload: dict[str, object] = {
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
            "relay_strategy": self.relay_strategy,
            "relay_deployments": self.relay_deployments,
            "successful_relay_deployments": (
                self.successful_relay_deployments
            ),
            "failed_relay_deployments": self.failed_relay_deployments,
            "relay_steps_by_drone": dict(self.relay_steps_by_drone),
            "relay_path_length_by_drone": dict(
                self.relay_path_length_by_drone
            ),
            "relay_unique_cells_forwarded": (
                self.relay_unique_cells_forwarded
            ),
            "relay_survivor_confirmations_forwarded": (
                self.relay_survivor_confirmations_forwarded
            ),
            "relay_outages_shortened": self.relay_outages_shortened,
            "base_known_coverage_over_time": [
                round(coverage, 6)
                for coverage in self.base_known_coverage_over_time
            ],
            "time_to_first_base_survivor_confirmation": (
                self.time_to_first_base_survivor_confirmation
            ),
            "time_to_all_base_survivor_confirmations": (
                self.time_to_all_base_survivor_confirmations
            ),
            "relay_energy_consumed": round(self.relay_energy_consumed, 6),
            "relay_mission_delay_steps": self.relay_mission_delay_steps,
            "network_profile": self.network_profile,
            "network_messages_queued": self.network_messages_queued,
            "network_messages_delivered": self.network_messages_delivered,
            "network_fragments_sent": self.network_fragments_sent,
            "network_fragments_delivered": self.network_fragments_delivered,
            "network_fragments_lost": self.network_fragments_lost,
            "network_messages_expired": self.network_messages_expired,
            "network_messages_dropped": self.network_messages_dropped,
            "network_delivery_ratio": (
                round(self.network_delivery_ratio, 6)
                if self.network_delivery_ratio is not None
                else None
            ),
            "network_transmission_attempts": self.network_transmission_attempts,
            "network_successful_transmission_attempts": (
                self.network_successful_transmission_attempts
            ),
            "network_retransmission_attempts": (
                self.network_retransmission_attempts
            ),
            "network_fragments_created": self.network_fragments_created,
            "network_fragment_attempt_delivery_ratio": (
                round(self.network_fragment_attempt_delivery_ratio, 6)
                if self.network_fragment_attempt_delivery_ratio is not None
                else None
            ),
            "network_unique_fragment_eventual_delivery_ratio": (
                round(self.network_unique_fragment_eventual_delivery_ratio, 6)
                if self.network_unique_fragment_eventual_delivery_ratio is not None
                else None
            ),
            "network_logical_message_completion_ratio": (
                round(self.network_logical_message_completion_ratio, 6)
                if self.network_logical_message_completion_ratio is not None
                else None
            ),
            "network_routes_replanned": self.network_routes_replanned,
            "network_mean_latency": (
                round(self.network_mean_latency, 6)
                if self.network_mean_latency is not None
                else None
            ),
            "network_max_latency": self.network_max_latency,
            "network_payload_units_delivered": (
                self.network_payload_units_delivered
            ),
            "network_average_queue_size": (
                round(self.network_average_queue_size, 6)
                if self.network_average_queue_size is not None
                else None
            ),
            "network_max_queue_size": self.network_max_queue_size,
            "network_max_backlog_duration": self.network_max_backlog_duration,
            "stale_motion_intents": self.stale_motion_intents,
            "map_sync_mean_latency": (
                round(self.map_sync_mean_latency, 6)
                if self.map_sync_mean_latency is not None
                else None
            ),
            "map_sync_max_latency": self.map_sync_max_latency,
            "survivor_knowledge_mean_latency": (
                round(self.survivor_knowledge_mean_latency, 6)
                if self.survivor_knowledge_mean_latency is not None
                else None
            ),
            "relay_network_fragments": self.relay_network_fragments,
            "relay_network_mean_latency": (
                round(self.relay_network_mean_latency, 6)
                if self.relay_network_mean_latency is not None
                else None
            ),
            "proximity_avoidances_without_fresh_intent": (
                self.proximity_avoidances_without_fresh_intent
            ),
            "final_sync_started": self.final_sync_started,
            "final_sync_duration": self.final_sync_duration,
            "final_sync_retransmissions": self.final_sync_retransmissions,
            "final_sync_survivor_confirmations_transferred": (
                self.final_sync_survivor_confirmations_transferred
            ),
            "final_sync_timeout": self.final_sync_timeout,
            "network_shield_cause_classification": dict(
                self.network_shield_cause_classification
            ),
            "network_shield_geometry_classification": dict(
                self.network_shield_geometry_classification
            ),
            "mission_success": self.mission_success,
            "mission_events": [event.to_dict() for event in self.mission_events],
        }
        if self.network_aware_relay_metrics is not None:
            payload["network_aware_relay"] = self.network_aware_relay_metrics
        if self.failure_recovery_metrics is not None:
            payload["failure_recovery"] = self.failure_recovery_metrics
        if self.smoke_metrics is not None:
            metric_key = (
                "perception"
                if self.smoke_metrics.get("sensor_channel") == "thermal"
                else "smoke"
            )
            payload[metric_key] = self.smoke_metrics
        if self.perception_metrics is not None:
            payload["noisy_perception"] = self.perception_metrics
        if self.dynamic_obstacle_metrics is not None:
            payload["dynamic_obstacles"] = self.dynamic_obstacle_metrics
        if self.role_failure_metrics is not None:
            payload["role_failure_resilience"] = self.role_failure_metrics
        if self.multi_relay_metrics is not None:
            payload["multi_relay"] = self.multi_relay_metrics
        if self.network_profile == "ideal":
            for key in tuple(payload):
                if key.startswith("network_") or key in {
                    "stale_motion_intents",
                    "map_sync_mean_latency",
                    "map_sync_max_latency",
                    "survivor_knowledge_mean_latency",
                    "relay_network_fragments",
                    "relay_network_mean_latency",
                    "proximity_avoidances_without_fresh_intent",
                    "final_sync_started",
                    "final_sync_duration",
                    "final_sync_retransmissions",
                    "final_sync_survivor_confirmations_transferred",
                    "final_sync_timeout",
                }:
                    payload.pop(key)
        return payload


FrameCallback = Callable[["MultiDroneSimulation"], None]


class MultiDroneSimulation:
    """Synchronous deterministic fleet simulation over shared knowledge."""

    def __init__(self, config: SimulationConfig) -> None:
        self.config = config
        self.knowledge_mode = config.effective_knowledge_mode
        self.world = GridWorld.generate(config)
        self.occupancy_map: OccupancyMap | ProbabilisticKnowledgeMap = (
            cast(ProbabilisticKnowledgeMap, self._new_knowledge_map(config)) if config.uncertainty_profile != "off"
            else OccupancyMap(config.width, config.height))
        self.sensor = DistanceSensor(config.sensor_range)
        self.survivor_sensor = SurvivorSensor(
            max_range=config.active_survivor_sensor_range,
            channel=config.survivor_sensor,
            base_detection_probability=(
                config.active_survivor_detection_probability
            ),
            smoke_attenuation=config.active_survivor_smoke_attenuation,
        )
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
        self.completed: bool = False
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
        self.relay_deployments = 0
        self.successful_relay_deployments = 0
        self.failed_relay_deployments = 0
        self._relay_unique_cells_forwarded: set[Position] = set()
        self._relay_survivors_forwarded: set[Position] = set()
        self.relay_outages_shortened = 0
        self._relay_energy_consumed = 0.0
        self._relay_mission_delay_steps = 0
        self._base_known_coverage_history: list[float] = []
        self.network_transport = (
            DeterministicNetworkTransport(
                seed=config.seed,
                profile=config.network_profile,
                latency_steps=config.network_latency_steps,
                packet_loss_rate=config.network_packet_loss_rate,
                link_capacity_units=config.network_link_capacity_units,
                max_fragment_units=config.network_max_fragment_units,
                fairness_age_steps=config.network_fairness_age_steps,
            )
            if config.network_profile == "constrained"
            else None
        )
        if self.network_transport is not None and config.relay_strategy in {
            "network-aware",
            "multi-relay",
            "predictive",
        }:
            self.network_transport.maximum_route_hops = (
                config.network_relay_max_hops
                if config.relay_strategy == "network-aware"
                else config.multi_relay_max_active + 1
            )
        self._network_queued_records: dict[
            tuple[str, str, Position], CellKnowledge
        ] = {}
        self._network_queued_survivors: dict[
            tuple[str, str, str, Position], str
        ] = {}
        self._received_motion_intents: dict[
            str, dict[str, MotionIntent]
        ] = {
            f"drone-{index}": {}
            for index in range(1, config.drone_count + 1)
        }
        self._network_map_latencies: list[int] = []
        self._network_survivor_latencies: list[int] = []
        self._network_delivered_links_this_step: set[tuple[str, str]] = set()
        self._network_delivered_units_this_step = 0
        self._network_backlog_warning_active = False
        self._network_finalized = False
        self.proximity_avoidances_without_fresh_intent = 0
        self._final_sync_active = False
        self._final_sync_started_step: int | None = None
        self._final_sync_retransmissions_at_start = 0
        self._final_sync_base_survivors_at_start: set[Position] = set()
        self._final_sync_duration = 0
        self._final_sync_timeout = False
        self._network_shield_cause_classification: dict[str, int] = {}
        self._network_shield_geometry_classification: dict[str, int] = {}
        self._legacy_network_proximity_precedence = False
        self._network_relay_weights = RelayUtilityWeights()
        self._network_relay_evaluations = 0
        self._network_relay_accepted = 0
        self._network_relay_rejected = 0
        self._network_relay_utility_sum = 0.0
        self._network_relay_accepted_utility_sum = 0.0
        self._network_relay_rejection_reasons: dict[str, int] = {}
        self._network_relay_last_signature: tuple[object, ...] | None = None
        self._network_relay_backpressure_steps = 0
        self._network_relay_backpressure_active = False
        self._network_relay_compacted_items = 0
        self._network_relay_route_replans = 0
        self._network_relay_critical_acknowledged = 0
        self._network_relay_first_hop_latencies: list[int] = []
        self._network_relay_end_to_end_latencies: list[int] = []
        self._network_relay_scout_wait_steps = 0
        self._network_relay_scout_exploration_steps = 0
        self._network_relay_unnecessary_deployments = 0
        self._network_relay_backlog_samples: list[int] = []
        self.active_relays: set[str] = set()
        self.relay_deployments_by_agent: dict[str, RelayDeployment] = {}
        self._multi_relay_activations = 0
        self._multi_relay_deactivations = 0
        self._multi_relay_active_samples: list[int] = []
        self._multi_relay_hop_samples: list[int] = []
        self._multi_relay_concurrent_link_samples: list[int] = []
        self._multi_relay_disconnected_agent_steps = 0
        self._multi_relay_connected_via_relay_steps = 0
        self._multi_relay_predictive_forecasts = 0
        self._multi_relay_predictive_activations = 0
        self._multi_relay_prevented_disconnect_steps = 0
        self._multi_relay_unnecessary_predictive_activations = 0
        self._multi_relay_role_changes = 0
        self._multi_relay_replans = 0
        self._multi_relay_failure_recoveries = 0
        self._multi_relay_unsynced_critical_samples = 0
        self._multi_relay_served_totals: dict[str, set[str]] = {}
        self._multi_relay_activation_counts: dict[str, int] = {}
        self._multi_relay_forecasts: dict[str, ConnectivityForecast] = {}
        self._orphaned_relay_recoveries: dict[str, RelayDeployment] = {}
        self._injected_failure_ids: set[str] = set()
        self._released_failure_tasks: dict[Position, str] = {}
        self._claimed_failure_tasks: dict[Position, tuple[str, str]] = {}
        self.task_registry = TaskRegistry()
        self._orphan_failure_tasks: dict[str, str] = {}
        self._role_failure_claims: dict[str, FailureRecoveryClaim] = {}
        self._failure_tasks_released = 0
        self._failure_tasks_reassigned = 0
        self._failed_reassignments = 0
        self._reassignment_latencies: list[int] = []
        self._recovery_latencies: list[int] = []
        self._role_changes = 0
        self._emergency_role_takeovers = 0
        self._role_changes_by_agent: dict[str, int] = {}
        self._tasks_completed_by_reassigned_agent = 0
        self._failed_drone_collision_avoidances = 0
        self._smoke_exposure_samples_by_drone: dict[str, int] = {}
        self._smoke_entries_by_drone: dict[str, int] = {}
        self._survivor_detection_attempts = 0
        self._survivor_detection_successes = 0
        self._survivor_detection_failures = 0
        self._smoke_degraded_detection_attempts = 0
        self._smoke_successful_detection_attempts = 0
        self._smoke_degraded_detection_events = 0
        self.hypothesis_tracker = SurvivorHypothesisTracker(
            minimum_positive_observations=(
                config.survivor_confirmation_observations
            ),
            confirmation_threshold=(
                config.survivor_confirmation_evidence_threshold
            ),
            rejection_threshold=config.survivor_rejection_evidence_threshold,
            negative_evidence_weight=config.survivor_negative_evidence_weight,
        )
        self._perception_tp = 0
        self._perception_fp = 0
        self._perception_tn = 0
        self._perception_fn = 0
        self._perception_calibration: list[tuple[float, bool]] = []
        # Benchmark-only ablation seam.  Public strategy semantics stay intact:
        # normal network-aware runs always keep active Relay roles enabled.
        self._network_aware_relay_roles_enabled = True
        # Equivalent v0.10 benchmark seam: transport behavior stays identical
        # while explicit Relay role assignment is disabled for control A.
        self._multi_relay_roles_enabled = True

        starts = self._resolve_start_positions()
        explicit_dynamic_events = tuple(
            DynamicObstacleEvent(Position(x, y), step, "explicit_injection")
            for x, y, step in config.dynamic_obstacle_schedule
        )
        for event in explicit_dynamic_events:
            if event.position in self.world.walls:
                raise ValueError(
                    "dynamic obstacle cannot duplicate an initial wall"
                )
            if event.position in self.world.survivors:
                raise ValueError("dynamic obstacle cannot block a Survivor")
        profile_events = (
            moderate_schedule(
                self.world,
                seed=config.seed,
                excluded_positions=frozenset(starts),
            )
            if config.dynamic_obstacles == "moderate"
            else ()
        )
        self.dynamic_obstacle_events = tuple(
            sorted(
                explicit_dynamic_events + profile_events,
                key=lambda event: (event.step, event.position),
            )
        )
        self._dynamic_event_index = 0
        self._dynamic_obstacles_injected: set[Position] = set()
        self._dynamic_obstacles_observed: set[Position] = set()
        self._path_invalidations = 0
        self._replans_total = 0
        self._successful_replans = 0
        self._failed_replans = 0
        self._target_invalidations = 0
        self._target_reassignments = 0
        self._rtb_replans = 0
        self._stale_path_safety_interventions = 0
        self._dynamic_replan_path_delta = 0
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
                local_map=self._new_knowledge_map(config),
                position_trace=[position],
            )
            self.runtimes[drone_id] = runtime
            if position != self.world.base:
                self._visited_by_cell.setdefault(position, set()).add(drone_id)

        assigned_roles = initial_roles(
            tuple(self.runtimes), config.role_policy
        )
        for runtime in self._ordered_runtimes():
            role = assigned_roles[runtime.drone.identifier]
            runtime.role = role
            runtime.base_role = role
            self._role_changes_by_agent[runtime.drone.identifier] = 0
            if self.roles_enabled:
                self._record_event(
                    runtime,
                    EventType.ROLE_ASSIGNED,
                    reason="initial_policy",
                    new_role=role.value,
                )

        self.local_hypothesis_trackers = {drone_id: SurvivorHypothesisTracker(
            minimum_positive_observations=config.survivor_confirmation_observations,
            confirmation_threshold=config.survivor_confirmation_evidence_threshold,
            rejection_threshold=config.survivor_rejection_evidence_threshold,
            negative_evidence_weight=config.survivor_negative_evidence_weight)
            for drone_id in self.runtimes}
        self.base_knowledge_map = (
            self._new_knowledge_map(config)
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
        self._inject_scheduled_failures()
        self._sample_communication(record_events=False)
        if self.network_transport is None:
            self._sync_shadow_maps()
            self._sync_survivor_knowledge()
            self._acknowledge_base_uploads()
        else:
            self._queue_network_knowledge()
            self._record_network_events()
        self._record_base_coverage()
        if self.knowledge_mode == "local":
            self._record_base_event(EventType.KNOWLEDGE_MODE_ACTIVATED)
        self._update_completion()

    @staticmethod
    def _new_knowledge_map(config: SimulationConfig) -> ProbabilisticKnowledgeMap | KnowledgeMap:
        if config.uncertainty_profile == "off":
            return KnowledgeMap(config.width, config.height)
        return ProbabilisticKnowledgeMap(config.width, config.height,
            probability_config=config.probability_config,
            reliability=UNCERTAINTY_PROFILES[config.uncertainty_profile].occupancy_reliability,
            planning_variant=config.planning_variant)

    def _advance_probability_maps(self) -> None:
        maps = [self.occupancy_map, self.base_knowledge_map,
                *(runtime.local_map for runtime in self.runtimes.values())]
        for knowledge in maps:
            if isinstance(knowledge, ProbabilisticKnowledgeMap):
                knowledge.advance(self.steps)

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

    @property
    def roles_enabled(self) -> bool:
        return self.config.role_policy != "off"

    @property
    def adaptive_relay_enabled(self) -> bool:
        return (
            self.knowledge_mode == "local"
            and self.config.relay_strategy == "adaptive"
        )

    @property
    def network_aware_relay_enabled(self) -> bool:
        return (
            self.network_aware_transport_enabled
            and self.config.relay_strategy == "network-aware"
            and self._network_aware_relay_roles_enabled
        )

    @property
    def network_aware_transport_enabled(self) -> bool:
        return (
            self.knowledge_mode == "local"
            and self.config.relay_strategy
            in {"network-aware", "multi-relay", "predictive"}
            and self.network_transport is not None
        )

    @property
    def multi_relay_enabled(self) -> bool:
        return (
            self.config.relay_strategy in {"multi-relay", "predictive"}
            and self._multi_relay_roles_enabled
        )

    @property
    def predictive_relay_enabled(self) -> bool:
        return self.config.relay_strategy == "predictive"

    @property
    def constrained_network_enabled(self) -> bool:
        return self.network_transport is not None

    def _record_base_coverage(self) -> None:
        coverage = (
            self.base_knowledge_map.known_coverage
            if self.base_knowledge_map is not None
            else 0.0
        )
        self._base_known_coverage_history.append(coverage)

    def _acknowledge_base_uploads(self) -> None:
        if self.knowledge_mode != "local":
            return
        for runtime in self._ordered_runtimes():
            connection = self.communication_snapshot.connections[
                runtime.drone.identifier
            ]
            if not connection.connected_to_base:
                continue
            runtime.base_acknowledged_records = dict(
                runtime.local_map.records
            )
            runtime.base_acknowledged_survivors = set(
                runtime.confirmed_survivors
            )

    def _network_stores(self) -> dict[str, KnowledgeMap]:
        stores = {
            drone_id: runtime.local_map
            for drone_id, runtime in sorted(self.runtimes.items())
        }
        if self.base_knowledge_map is not None:
            stores["base"] = self.base_knowledge_map
        return stores

    def _network_survivor_store(
        self, node_id: str
    ) -> tuple[set[Position], set[Position]]:
        if node_id == "base":
            return self._base_detected_survivors, self._base_confirmed_survivors
        runtime = self.runtimes[node_id]
        return runtime.detected_survivors, runtime.confirmed_survivors

    def _queue_network_knowledge(self) -> None:
        transport = self.network_transport
        if transport is None:
            return
        stores = self._network_stores()
        node_ids = tuple(sorted(stores))
        for sender in node_ids:
            for recipient in node_ids:
                if sender == recipient:
                    continue
                if (
                    sender in self.runtimes
                    and recipient in self.runtimes
                    and not self.communication_snapshot.has_link(sender, recipient)
                ):
                    continue
                route = shortest_route(
                    self.communication_snapshot, sender, recipient
                )
                if not route:
                    continue
                if (
                    self.network_aware_transport_enabled
                    and len(route) - 1 > self.config.network_relay_max_hops
                ):
                    continue
                if (
                    self.network_aware_transport_enabled
                    and transport.queued_payload_units
                    > self.config.network_relay_max_backlog_units
                ):
                    compacted = transport.compact_unsent_map_messages(
                        sender=sender, recipient=recipient
                    )
                    compacted_positions = {
                        item[0]
                        for item in compacted
                        if isinstance(item, tuple)
                        and len(item) == 2
                        and isinstance(item[0], Position)
                    }
                    for position in compacted_positions:
                        self._network_queued_records.pop(
                            (sender, recipient, position), None
                        )
                    if compacted_positions:
                        self._network_relay_compacted_items += len(
                            compacted_positions
                        )
                        runtime = self.runtimes.get(sender)
                        if runtime is not None:
                            self._record_event(
                                runtime,
                                EventType.RELAY_PAYLOAD_COMPACTED,
                                cell_count=len(compacted_positions),
                                backpressure=True,
                            )
                records = []
                for position, record in stores[sender].records:
                    key = (sender, recipient, position)
                    if self._network_queued_records.get(key) == record:
                        continue
                    records.append((position, record))
                if self.network_aware_transport_enabled and records:
                    records.sort(
                        key=lambda item: (
                            -item[1].observed_step,
                            item[0].x,
                            item[0].y,
                        )
                    )
                    records = records[
                        : self.config.network_relay_map_delta_limit
                    ]
                if records:
                    message_id = transport.enqueue(
                        sender=sender,
                        recipient=recipient,
                        route=route,
                        message_type=MessageType.MAP_UPDATE,
                        payload=[(position, CellKnowledge(record.state, e.observed_step,
                            e.source_id, (e,))) for position, record in records for e in record.evidence]
                            if self.config.uncertainty_profile != "off" else records,
                        created_step=self.steps,
                        ttl=self.config.network_map_ttl,
                        message_key=f"map:{sender}:{recipient}",
                    )
                    if message_id is not None:
                        for position, record in records:
                            self._network_queued_records[
                                (sender, recipient, position)
                            ] = record

                detected, confirmed = self._network_survivor_store(sender)
                recipient_detected, recipient_confirmed = (
                    self._network_survivor_store(recipient)
                )
                for kind, positions, message_type in (
                    (
                        "confirmed",
                        confirmed,
                        MessageType.SURVIVOR_CONFIRMATION,
                    ),
                    (
                        "detected",
                        detected,
                        MessageType.SURVIVOR_DETECTION,
                    ),
                ):
                    if self._legacy_network_proximity_precedence:
                        pending = [
                            position
                            for position in sorted(positions)
                            if (sender, recipient, kind, position)
                            not in self._network_queued_survivors
                        ]
                    else:
                        pending = [
                            position
                            for position in sorted(positions)
                            if position not in (
                                recipient_confirmed
                                if kind == "confirmed"
                                else recipient_detected
                            )
                            and not (
                                (
                                    message_id := self._network_queued_survivors.get(
                                        (sender, recipient, kind, position)
                                    )
                                )
                                and transport.message_is_active(message_id)
                            )
                        ]
                    if not pending:
                        continue
                    message_id = transport.enqueue(
                        sender=sender,
                        recipient=recipient,
                        route=route,
                        message_type=message_type,
                        payload=pending,
                        created_step=self.steps,
                        ttl=self.config.network_survivor_ttl,
                        message_key=f"survivor:{kind}:{sender}:{recipient}",
                    )
                    if message_id is not None:
                        for position in pending:
                            self._network_queued_survivors[
                                (sender, recipient, kind, position)
                            ] = message_id

    def _queue_motion_messages(self) -> None:
        transport = self.network_transport
        if transport is None:
            return
        for sender, intent in sorted(self.motion_intents.items()):
            for recipient in sorted(self.runtimes):
                if recipient == sender:
                    continue
                route = shortest_route(
                    self.communication_snapshot, sender, recipient
                )
                if route:
                    transport.enqueue(
                        sender=sender,
                        recipient=recipient,
                        route=route,
                        message_type=MessageType.MOTION_INTENT,
                        payload=(intent,),
                        created_step=self.steps,
                        ttl=self.config.motion_intent_ttl,
                        message_key=f"intent:{sender}:{recipient}:{self.steps}",
                    )
            route_to_base = shortest_route(
                self.communication_snapshot, sender, "base"
            )
            if route_to_base:
                transport.enqueue(
                    sender=sender,
                    recipient="base",
                    route=route_to_base,
                    message_type=MessageType.DRONE_STATE,
                    payload=(
                        {
                            "position": intent.current_position,
                            "status": intent.status,
                            "energy_remaining": intent.energy_remaining,
                            "returning": intent.status is DroneStatus.RETURN_HOME,
                        },
                    ),
                    created_step=self.steps,
                    ttl=self.config.motion_intent_ttl,
                    message_key=f"state:{sender}:{self.steps}",
                )

    def _apply_network_delivery(self, delivery: NetworkDelivery) -> None:
        self._network_delivered_links_this_step.update(
            zip(delivery.route, delivery.route[1:])
        )
        if self.network_transport is not None:
            self._network_delivered_units_this_step += len(delivery.payload)
        if delivery.message_type is MessageType.MOTION_INTENT:
            intent = delivery.payload[0]
            if (
                isinstance(intent, MotionIntent)
                and intent.valid_until_step >= self.steps
                and delivery.recipient in self.runtimes
            ):
                self._received_motion_intents[delivery.recipient][
                    delivery.sender
                ] = intent
            elif self.network_transport is not None:
                self.network_transport.stale_intents += 1
            return
        if delivery.message_type is MessageType.DRONE_STATE:
            return
        if delivery.message_type is MessageType.MAP_UPDATE:
            target = self._network_stores().get(delivery.recipient)
            if target is None:
                return
            records = cast(
                tuple[tuple[Position, CellKnowledge], ...],
                delivery.payload,
            )
            changed = target.apply(records)
            for position, record in records:
                self._network_queued_records[
                    (delivery.recipient, delivery.sender, position)
                ] = record
            if not changed:
                return
            self._network_map_latencies.append(delivery.latency)
            self._unique_cells_transferred.update(changed)
            semantic_changes = sum(
                target.cell_at(position) is record.state
                for position, record in records
                if position in changed
            )
            self._semantic_cell_changes_transferred += semantic_changes
            self._map_sync_events += 1
            if delivery.sender in self.runtimes:
                self._cells_uploaded_by_drone[delivery.sender] = (
                    self._cells_uploaded_by_drone.get(delivery.sender, 0)
                    + len(changed)
                )
            if delivery.recipient in self.runtimes:
                self._cells_received_by_drone[delivery.recipient] = (
                    self._cells_received_by_drone.get(delivery.recipient, 0)
                    + len(changed)
                )
            if delivery.recipient == "base" and delivery.sender in self.runtimes:
                runtime = self.runtimes[delivery.sender]
                for position, record in records:
                    current = runtime.base_acknowledged_records.get(position)
                    if current is None or current.observed_step <= record.observed_step:
                        runtime.base_acknowledged_records[position] = record
            return
        if delivery.message_type not in {
            MessageType.SURVIVOR_DETECTION,
            MessageType.SURVIVOR_CONFIRMATION,
        }:
            return
        detected, confirmed = self._network_survivor_store(delivery.recipient)
        positions = {item for item in delivery.payload if isinstance(item, Position)}
        knowledge_kind = (
            "confirmed"
            if delivery.message_type is MessageType.SURVIVOR_CONFIRMATION
            else "detected"
        )
        if self._legacy_network_proximity_precedence:
            for position in positions:
                self._network_queued_survivors[
                    (
                        delivery.recipient,
                        delivery.sender,
                        knowledge_kind,
                        position,
                    )
                ] = delivery.message_id
        new_confirmed: set[Position] = set()
        if delivery.message_type is MessageType.SURVIVOR_CONFIRMATION:
            new_confirmed = positions - confirmed
            confirmed.update(positions)
            detected.update(positions)
        else:
            detected.update(positions)
        if positions:
            self._network_survivor_latencies.append(delivery.latency)
        if delivery.recipient == "base" and delivery.sender in self.runtimes:
            self.runtimes[delivery.sender].base_acknowledged_survivors.update(
                positions
            )
        for position in sorted(new_confirmed):
            target_runtime = self.runtimes.get(delivery.recipient)
            if target_runtime is not None:
                self._record_event(
                    target_runtime,
                    EventType.SURVIVOR_KNOWLEDGE_SYNCHRONIZED,
                    position,
                )
            else:
                self.mission_log.record(
                    MissionEvent(
                        position=position,
                        step=self.steps,
                        drone_id="base",
                        event_type=EventType.SURVIVOR_KNOWLEDGE_SYNCHRONIZED,
                    )
                )

    def _record_network_events(self) -> None:
        transport = self.network_transport
        if transport is None:
            return
        mapping: dict[str, EventType] = {
            "message_queued": EventType.MESSAGE_QUEUED,
            "message_delivered": EventType.MESSAGE_DELIVERED,
            "message_lost": EventType.MESSAGE_LOST,
            "message_expired": EventType.MESSAGE_EXPIRED,
            "message_dropped": EventType.MESSAGE_DROPPED,
            "message_fragment_completed": EventType.MESSAGE_FRAGMENT_COMPLETED,
            "relay_message_forwarded": EventType.RELAY_MESSAGE_FORWARDED,
        }
        aggregated: dict[
            tuple[str, str, str, MessageType], list[int]
        ] = {}
        for event in transport.drain_events():
            if (
                event.event_type in {"message_queued", "message_delivered"}
                and event.message_type
                in {MessageType.MAP_UPDATE, MessageType.DRONE_STATE}
            ) or (
                event.event_type == "message_expired"
                and event.message_type is MessageType.DRONE_STATE
            ):
                continue
            key = (
                event.event_type,
                event.sender,
                event.recipient,
                event.message_type,
            )
            item = aggregated.setdefault(key, [0, 0])
            item[0] += event.fragment_count
            item[1] += event.payload_units
        for key, (count, units) in sorted(
            aggregated.items(), key=lambda item: tuple(str(value) for value in item[0])
        ):
            event_name, sender, _recipient, message_type = key
            runtime = self.runtimes.get(str(sender))
            position = runtime.drone.position if runtime is not None else self.world.base
            self.mission_log.record(
                MissionEvent(
                    position=position,
                    step=self.steps,
                    drone_id=str(sender),
                    event_type=mapping[str(event_name)],
                    energy_remaining=(
                        runtime.battery.remaining if runtime is not None else None
                    ),
                    message_type=message_type.value,
                    message_count=count,
                    payload_units=units,
                    queue_size=transport.queue_size,
                )
            )
        overloaded = (
            transport.queue_size
            >= self.config.network_backlog_warning_threshold
        )
        if overloaded and not self._network_backlog_warning_active:
            self.mission_log.record(
                MissionEvent(
                    position=self.world.base,
                    step=self.steps,
                    drone_id="base",
                    event_type=EventType.NETWORK_BACKLOG_WARNING,
                    queue_size=transport.queue_size,
                )
            )
        self._network_backlog_warning_active = overloaded

    def _deliver_network_transport(self) -> None:
        transport = self.network_transport
        if transport is None:
            return
        self._network_delivered_links_this_step = set()
        self._network_delivered_units_this_step = 0
        base_before = (
            dict(self.base_knowledge_map.records)
            if self.base_knowledge_map is not None
            else {}
        )
        survivors_before = set(self._base_confirmed_survivors)
        for delivery in transport.deliver(step=self.steps):
            self._apply_network_delivery(delivery)
        self._record_network_events()
        self._update_relay_after_sync(base_before, survivors_before)
        converged = self.shadow_synchronizer.maps_converged()
        if (
            not self._shadow_maps_converged
            and converged
            and self._time_to_map_convergence is None
        ):
            self._time_to_map_convergence = self.steps
            self._record_base_event(EventType.MAP_CONVERGED)
        self._shadow_maps_converged = converged
        self._peak_map_divergence = max(
            self._peak_map_divergence,
            self.shadow_synchronizer.divergence_ratio(),
        )

    def _transmit_network_transport(self) -> None:
        transport = self.network_transport
        if transport is None:
            return
        if self.network_aware_transport_enabled:
            backlog = transport.queued_payload_units
            self._network_relay_backlog_samples.append(backlog)
            active = backlog > self.config.network_relay_max_backlog_units
            if active:
                self._network_relay_backpressure_steps += 1
            if active != self._network_relay_backpressure_active:
                self.mission_log.record(
                    MissionEvent(
                        position=self.world.base,
                        step=self.steps,
                        drone_id="base",
                        event_type=(
                            EventType.RELAY_BACKPRESSURE_STARTED
                            if active
                            else EventType.RELAY_BACKPRESSURE_ENDED
                        ),
                        queue_size=transport.queue_size,
                        payload_units=backlog,
                        backpressure=active,
                    )
                )
            self._network_relay_backpressure_active = active
        transport.transmit(
            step=self.steps,
            snapshot=self.communication_snapshot,
        )
        self._record_network_events()

    def _finalize_network_transport(self) -> None:
        if self.network_transport is None or self._network_finalized:
            return
        self.network_transport.finalize(self.steps)
        self._record_network_events()
        self._network_finalized = True

    def _pending_critical_survivors(self) -> set[Position]:
        """Return confirmed local knowledge not yet delivered to the base.

        The decision intentionally uses no world survivor positions.  Ground
        truth remains confined to result evaluation.
        """

        locally_confirmed: set[Position] = set()
        for runtime in self._ordered_runtimes():
            locally_confirmed.update(runtime.confirmed_survivors)
        return locally_confirmed - self._base_confirmed_survivors

    def _start_final_sync(self) -> None:
        transport = self.network_transport
        assert transport is not None
        self._final_sync_active = True
        self._final_sync_started_step = self.steps
        self._final_sync_retransmissions_at_start = (
            transport.retransmission_attempts
        )
        self._final_sync_base_survivors_at_start = set(
            self._base_confirmed_survivors
        )
        self.mission_log.record(
            MissionEvent(
                position=self.world.base,
                step=self.steps,
                drone_id="base",
                event_type=EventType.FINAL_SYNC_STARTED,
                survivor_count=len(self._pending_critical_survivors()),
                queue_size=transport.queue_size,
            )
        )

    def _finish_final_sync(self, *, timed_out: bool) -> None:
        self._final_sync_active = False
        self._final_sync_timeout = timed_out
        started = self._final_sync_started_step
        self._final_sync_duration = (
            self.steps - started if started is not None else 0
        )
        self.mission_log.record(
            MissionEvent(
                position=self.world.base,
                step=self.steps,
                drone_id="base",
                event_type=(
                    EventType.FINAL_SYNC_TIMEOUT
                    if timed_out
                    else EventType.FINAL_SYNC_COMPLETED
                ),
                survivor_count=len(
                    self._base_confirmed_survivors
                    - self._final_sync_base_survivors_at_start
                ),
                queue_size=(
                    self.network_transport.queue_size
                    if self.network_transport is not None
                    else 0
                ),
            )
        )
        self.completed = True
        self.termination_reason = (
            "final_sync_timeout"
            if timed_out
            else (
                "failure_recovered"
                if self._injected_failure_ids
                and self._all_survivors_confirmed()
                else (
                    "exploration_complete"
                    if self._exploration_complete
                    else "returned_to_base"
                )
            )
        )
        self._finalize_network_transport()

    def _step_final_sync(self) -> bool:
        """Advance normal constrained transport while landed at powered base.

        Flight batteries are not charged for this phase: landed vehicles are
        modeled as using base-station power for electronics and radio.  Link
        latency, capacity, loss, retries and TTL remain unchanged.
        """

        self._deliver_network_transport()
        if not self._pending_critical_survivors():
            self._finish_final_sync(timed_out=False)
            return False
        assert self._final_sync_started_step is not None
        if (
            self.steps - self._final_sync_started_step
            >= self.config.final_sync_max_steps
        ):
            self._finish_final_sync(timed_out=True)
            return False

        self._queue_network_knowledge()
        self._transmit_network_transport()
        self.steps += 1
        for runtime in self._ordered_runtimes():
            runtime.position_trace.append(runtime.drone.position)
        self._sample_communication()
        self._record_base_coverage()
        return True

    def _ordered_runtimes(self) -> tuple[DroneRuntime, ...]:
        return tuple(self.runtimes[key] for key in sorted(self.runtimes))

    def _compute_communication_snapshot(self) -> CommunicationSnapshot:
        active_positions = {
            runtime.drone.identifier: runtime.drone.position
            for runtime in self._ordered_runtimes()
            if runtime.drone.status is not DroneStatus.FAILED
        }
        snapshot = self.communication_model.compute(
            self.world,
            self.world.base,
            active_positions,
        )
        connections = dict(snapshot.connections)
        for runtime in self._ordered_runtimes():
            connections.setdefault(
                runtime.drone.identifier,
                DroneConnection(False, False),
            )
        return CommunicationSnapshot(
            nodes=snapshot.nodes,
            links=snapshot.links,
            connections=connections,
        )

    def _observable_failed_positions(
        self, runtime: DroneRuntime | None = None
    ) -> frozenset[Position]:
        failed = {
            candidate.drone.position
            for candidate in self._ordered_runtimes()
            if candidate.drone.status is DroneStatus.FAILED
            and candidate.drone.position != self.world.base
        }
        if self.knowledge_mode != "local" or runtime is None:
            return frozenset(failed)
        return frozenset(
            position
            for position in failed
            if self.proximity_sensor.can_detect(
                self.world, runtime.drone.position, position
            )
        )

    def _task_type_for_target(self, target: Position) -> TaskType:
        detected = (
            self._base_detected_survivors
            if self.knowledge_mode == "local"
            else self._detected_survivors
        )
        confirmed = (
            self._base_confirmed_survivors
            if self.knowledge_mode == "local"
            else self._confirmed_survivors
        )
        if target in detected - confirmed:
            return TaskType.SURVIVOR_VERIFICATION
        return TaskType.EXPLORATION

    def _transition_role(
        self,
        runtime: DroneRuntime,
        new_role: AgentRole,
        *,
        reason: str,
        task: MissionTask | None = None,
        emergency_takeover: bool = False,
    ) -> None:
        old_role = runtime.role
        if old_role is new_role:
            return
        runtime.role = new_role
        runtime.role_last_changed_step = self.steps
        self._role_changes += 1
        if self.multi_relay_enabled and (
            old_role is AgentRole.RELAY or new_role is AgentRole.RELAY
        ):
            self._multi_relay_role_changes += 1
        drone_id = runtime.drone.identifier
        self._role_changes_by_agent[drone_id] = (
            self._role_changes_by_agent.get(drone_id, 0) + 1
        )
        if emergency_takeover:
            self._emergency_role_takeovers += 1
        self._record_event(
            runtime,
            EventType.ROLE_CHANGED,
            task.target if task is not None else runtime.drone.position,
            reason=reason,
            task_id=task.identifier if task is not None else None,
            task_type=task.task_type.value if task is not None else None,
            task_owner=runtime.drone.identifier,
            old_role=old_role.value,
            new_role=new_role.value,
        )

    def _finish_runtime_task(
        self,
        runtime: DroneRuntime,
        *,
        completed: bool,
    ) -> MissionTask | None:
        task = (
            self.task_registry.complete_owner(
                runtime.drone.identifier, self.steps
            )
            if completed
            else self.task_registry.cancel_owner(
                runtime.drone.identifier, self.steps
            )
        )
        if task is None:
            return None
        if completed and task.reassigned:
            self._tasks_completed_by_reassigned_agent += 1
            self._record_event(
                runtime,
                EventType.TASK_COMPLETED_AFTER_REASSIGNMENT,
                task.target,
                reason="reassigned_task_completed",
                task_id=task.identifier,
                task_type=task.task_type.value,
                task_status=task.status.value,
                task_owner=runtime.drone.identifier,
            )
        if runtime.recovery_task_id == task.identifier:
            runtime.recovery_task_id = None
            if (
                self.config.role_policy == "generalized"
                and runtime.role is not runtime.base_role
            ):
                self._transition_role(
                    runtime,
                    runtime.base_role,
                    reason="recovery_task_completed",
                    task=task,
                )
        return task

    def _synchronize_role_tasks(self) -> None:
        if not self.roles_enabled:
            return
        for runtime in self._ordered_runtimes():
            current = self.task_registry.current_for(
                runtime.drone.identifier
            )
            desired: tuple[TaskType, Position, int] | None = None
            if (
                runtime.drone.status is DroneStatus.EXPLORE
                and runtime.active_frontier_target is not None
            ):
                task_type = self._task_type_for_target(
                    runtime.active_frontier_target
                )
                desired = (
                    task_type,
                    runtime.active_frontier_target,
                    80 if task_type is TaskType.SURVIVOR_VERIFICATION else 50,
                )
            elif (
                runtime.drone.status is DroneStatus.RELAY
                and runtime.relay_target is not None
            ):
                desired = (TaskType.RELAY_POSITIONING, runtime.relay_target, 70)
            elif runtime.drone.status is DroneStatus.RETURN_HOME:
                desired = (TaskType.RETURN_HOME, self.world.base, 100)

            if desired is None:
                if current is not None:
                    self._finish_runtime_task(
                        runtime,
                        completed=runtime.drone.status is DroneStatus.LANDED,
                    )
                continue
            task_type, target, priority = desired
            if (
                current is not None
                and current.task_type is task_type
                and current.target == target
            ):
                continue
            if current is not None:
                self._finish_runtime_task(runtime, completed=True)
            self.task_registry.create(
                owner_id=runtime.drone.identifier,
                task_type=task_type,
                target=target,
                step=self.steps,
                priority=priority,
            )
        for relay_id, deployment in self.relay_deployments_by_agent.items():
            task = self.task_registry.current_for(relay_id)
            if task is not None and task.task_type is TaskType.RELAY_POSITIONING:
                deployment.task_id = task.identifier

    def _record_recovery_movements(self, movers: dict[str, Position]) -> None:
        if not self.roles_enabled:
            return
        for drone_id in sorted(movers):
            runtime = self.runtimes[drone_id]
            task_id = runtime.recovery_task_id
            if task_id is None:
                continue
            task = self.task_registry.tasks.get(task_id)
            if (
                task is None
                or task.execution_recovered_step is not None
                or task.orphaned_step is None
            ):
                continue
            task.execution_recovered_step = self.steps
            latency = self.steps - task.orphaned_step
            self._recovery_latencies.append(latency)
            self._record_event(
                runtime,
                EventType.FAILURE_RECOVERY_COMPLETED,
                task.target,
                reason="productive_execution_resumed",
                task_id=task.identifier,
                task_type=task.task_type.value,
                task_status=task.status.value,
                task_owner=drone_id,
                assignment_latency=latency,
            )

    def _inject_scheduled_failures(self) -> None:
        due = tuple(
            sorted(
                drone_id
                for drone_id, failure_step in self.config.failure_schedule
                if failure_step == self.steps
                and drone_id not in self._injected_failure_ids
            )
        )
        if not due:
            return
        for drone_id in due:
            runtime = self.runtimes[drone_id]
            if runtime.terminal:
                continue
            released_target = runtime.active_frontier_target
            orphaned_task = (
                self.task_registry.orphan_owner(drone_id, self.steps)
                if self.roles_enabled
                else None
            )
            if orphaned_task is not None:
                released_target = orphaned_task.target
                self._orphan_failure_tasks[orphaned_task.identifier] = drone_id
                self._record_event(
                    runtime,
                    EventType.TASK_ORPHANED,
                    orphaned_task.target,
                    reason="owner_failed",
                    task_id=orphaned_task.identifier,
                    task_type=orphaned_task.task_type.value,
                    task_status=orphaned_task.status.value,
                )
                relay_context = self.relay_deployments_by_agent.get(drone_id)
                if (
                    orphaned_task.task_type is TaskType.RELAY_POSITIONING
                    and relay_context is not None
                ):
                    self._orphaned_relay_recoveries[
                        orphaned_task.identifier
                    ] = relay_context
            if runtime.drone.status is DroneStatus.RELAY:
                self._finish_relay_role(runtime, successful=False)
            for relay in self._ordered_runtimes():
                if (
                    relay.drone.status is DroneStatus.RELAY
                    and relay.relay_scout_id == drone_id
                ):
                    self._finish_relay_role(relay, successful=False)
            runtime.drone.status = DroneStatus.FAILED
            runtime.active_frontier_target = None
            runtime.planned_path = ()
            runtime.current_return_path = ()
            runtime.holding_for_relay = False
            self.motion_intents.pop(drone_id, None)
            self._last_communicated_intents.pop(drone_id, None)
            for received in self._received_motion_intents.values():
                received.pop(drone_id, None)
            self._injected_failure_ids.add(drone_id)
            self._record_event(
                runtime,
                EventType.DRONE_FAILURE_INJECTED,
                reason="scheduled_failure",
            )
            if released_target is not None:
                self._released_failure_tasks[released_target] = drone_id
                self._failure_tasks_released += 1
                self._record_event(
                    runtime,
                    EventType.FAILURE_TASK_RELEASED,
                    released_target,
                    reason=drone_id,
                )
        self.communication_snapshot = self._compute_communication_snapshot()

    def _prepare_failure_task_reassignments(self) -> None:
        if self.roles_enabled:
            self._prepare_role_failure_task_reassignments()
            return
        for target, failed_id in sorted(
            self._released_failure_tasks.items(),
            key=lambda item: (item[0].y, item[0].x, item[1]),
        ):
            candidates: list[tuple[int, str, tuple[Position, ...]]] = []
            for runtime in self._ordered_runtimes():
                if (
                    runtime.drone.status is not DroneStatus.EXPLORE
                    or runtime.holding_for_relay
                ):
                    continue
                decision_map = self._decision_map(runtime)
                if target not in decision_map.frontiers():
                    continue
                blocked = self._observable_failed_positions(runtime)
                path = astar(
                    runtime.drone.position,
                    target,
                    lambda position: decision_map.is_known_free(position)
                    and (
                        position == runtime.drone.position
                        or position not in blocked
                    ),
                )
                if path is not None:
                    candidates.append(
                        (len(path), runtime.drone.identifier, path)
                    )
            if not candidates:
                continue
            _, assignee_id, path = min(candidates)
            assignee = self.runtimes[assignee_id]
            assignee.active_frontier_target = target
            assignee.planned_path = path
            self._claimed_failure_tasks[target] = (failed_id, assignee_id)

    def _prepare_role_failure_task_reassignments(self) -> None:
        claimed_assignees = {
            claim.assignee_id for claim in self._role_failure_claims.values()
        }
        for task_id, failed_id in sorted(self._orphan_failure_tasks.items()):
            if task_id in self._role_failure_claims:
                continue
            task = self.task_registry.tasks[task_id]
            if task.status is not TaskStatus.ORPHANED:
                continue
            if task.task_type not in {
                TaskType.EXPLORATION,
                TaskType.SURVIVOR_VERIFICATION,
            }:
                continue
            candidates: list[
                tuple[
                    int,
                    int,
                    float,
                    str,
                    Position,
                    tuple[Position, ...],
                    int,
                    int,
                    int,
                ]
            ] = []
            fallback_candidates: list[
                tuple[
                    int,
                    int,
                    float,
                    str,
                    Position,
                    tuple[Position, ...],
                    int,
                    int,
                    int,
                ]
            ] = []
            for runtime in self._ordered_runtimes():
                drone_id = runtime.drone.identifier
                if (
                    drone_id in claimed_assignees
                    or runtime.drone.status is not DroneStatus.EXPLORE
                    or runtime.holding_for_relay
                ):
                    continue
                decision_map = self._decision_map(runtime)
                frontiers = decision_map.frontiers()
                if not frontiers:
                    continue
                targets = frontiers
                blocked = self._observable_failed_positions(runtime)
                for target in targets:
                    path = astar(
                        runtime.drone.position,
                        target,
                        lambda position: decision_map.is_known_free(position)
                        and (
                            position == runtime.drone.position
                            or position not in blocked
                        ),
                    )
                    return_path = astar(
                        target,
                        self.world.base,
                        lambda position: decision_map.is_known_free(position)
                        and (position == target or position not in blocked),
                    )
                    if path is None or return_path is None:
                        continue
                    required_energy = (
                        runtime.battery.estimate_path(len(path))
                        + runtime.battery.estimate_path(len(return_path))
                        + self.config.energy_safety_reserve
                    )
                    if runtime.battery.remaining + 1e-9 < required_energy:
                        continue
                    path_cost = max(0, len(path) - 1)
                    retarget_penalty = 0 if target == task.target else 5
                    load_penalty = (
                        self.task_registry.active_load(drone_id)
                        * TASK_LOAD_PENALTY
                    )
                    role_penalty = role_mismatch_penalty(
                        runtime.role, task.task_type
                    )
                    energy_penalty = int(
                        runtime.battery.consumed // ENERGY_PENALTY_INTERVAL
                    )
                    score = (
                        path_cost
                        + retarget_penalty
                        + load_penalty
                        + role_penalty
                        + energy_penalty
                    )
                    candidate = (
                        (
                            score,
                            path_cost,
                            -runtime.battery.remaining,
                            drone_id,
                            target,
                            path,
                            load_penalty,
                            role_penalty,
                            energy_penalty,
                        )
                    )
                    if target == task.target:
                        candidates.append(candidate)
                    else:
                        fallback_candidates.append(candidate)
            if not candidates:
                candidates = fallback_candidates
            if not candidates:
                continue
            (
                score,
                path_cost,
                _,
                assignee_id,
                target,
                path,
                load_penalty,
                role_penalty,
                energy_penalty,
            ) = min(candidates)
            assignee = self.runtimes[assignee_id]
            assignee.active_frontier_target = target
            assignee.planned_path = path
            self._role_failure_claims[task_id] = FailureRecoveryClaim(
                task_id=task_id,
                failed_id=failed_id,
                assignee_id=assignee_id,
                target=target,
                path=path,
                score=score,
                path_cost=path_cost,
                task_load_penalty=load_penalty,
                role_penalty=role_penalty,
                energy_penalty=energy_penalty,
            )
            claimed_assignees.add(assignee_id)

    def _confirm_failure_task_reassignments(
        self, assignments: dict[str, FrontierAssignment]
    ) -> None:
        if self.roles_enabled:
            self._confirm_role_failure_task_reassignments(assignments)
            return
        used_assignees: set[str] = set()
        for target, (failed_id, assignee_id) in tuple(
            self._claimed_failure_tasks.items()
        ):
            assignment = assignments.get(assignee_id)
            if assignment is None or assignment.target != target:
                continue
            assignee = self.runtimes[assignee_id]
            assignee.frontier_assignments += 1
            self._failure_tasks_reassigned += 1
            self._record_event(
                assignee,
                EventType.FAILURE_TASK_REASSIGNED,
                target,
                reason=failed_id,
            )
            used_assignees.add(assignee_id)
            self._released_failure_tasks.pop(target, None)
            self._claimed_failure_tasks.pop(target, None)
        for released_target, failed_id in tuple(
            sorted(
                self._released_failure_tasks.items(),
                key=lambda item: (item[0].y, item[0].x, item[1]),
            )
        ):
            available = tuple(
                (drone_id, assignment)
                for drone_id, assignment in sorted(assignments.items())
                if drone_id not in used_assignees
            )
            if not available:
                continue
            assignee_id, assignment = min(
                available,
                key=lambda item: (
                    len(item[1].path),
                    item[1].target.y,
                    item[1].target.x,
                    item[0],
                ),
            )
            self._failure_tasks_reassigned += 1
            self._record_event(
                self.runtimes[assignee_id],
                EventType.FAILURE_TASK_REASSIGNED,
                assignment.target,
                reason=failed_id,
            )
            used_assignees.add(assignee_id)
            self._released_failure_tasks.pop(released_target, None)
            self._claimed_failure_tasks.pop(released_target, None)

    def _confirm_role_failure_task_reassignments(
        self, assignments: dict[str, FrontierAssignment]
    ) -> None:
        for task_id, claim in tuple(sorted(self._role_failure_claims.items())):
            assignment = assignments.get(claim.assignee_id)
            if assignment is None or assignment.target != claim.target:
                continue
            task = self.task_registry.tasks[task_id]
            original_target = task.target
            reassigned = self.task_registry.assign_orphan(
                task_id,
                owner_id=claim.assignee_id,
                target=claim.target,
                step=self.steps,
            )
            assignee = self.runtimes[claim.assignee_id]
            assignee.recovery_task_id = task_id
            latency = (
                self.steps - reassigned.orphaned_step
                if reassigned.orphaned_step is not None
                else 0
            )
            self._reassignment_latencies.append(latency)
            self._failure_tasks_reassigned += 1
            assignee.frontier_assignments += 1
            preferred = preferred_role(reassigned.task_type)
            if (
                self.config.role_policy == "generalized"
                and assignee.role is not preferred
            ):
                self._transition_role(
                    assignee,
                    preferred,
                    reason="failure_recovery",
                    task=reassigned,
                    emergency_takeover=True,
                )
            self._record_event(
                assignee,
                EventType.TASK_REASSIGNED,
                claim.target,
                reason=claim.failed_id,
                task_id=task_id,
                task_type=reassigned.task_type.value,
                task_status=reassigned.status.value,
                task_owner=claim.assignee_id,
                reassignment_score=claim.score,
                assignment_latency=latency,
            )
            self._record_event(
                assignee,
                EventType.FAILURE_TASK_REASSIGNED,
                claim.target,
                reason=claim.failed_id,
                task_id=task_id,
                task_type=reassigned.task_type.value,
                task_owner=claim.assignee_id,
                reassignment_score=claim.score,
                assignment_latency=latency,
            )
            self._released_failure_tasks.pop(original_target, None)
            self._orphan_failure_tasks.pop(task_id, None)
            self._role_failure_claims.pop(task_id, None)

    def _resolve_start_positions(self) -> tuple[Position, ...]:
        starts: tuple[Position, ...]
        if self.config.drone_start_positions is None:
            candidates = sorted(
                (
                    Position(x, y)
                    for y in range(1, self.config.height - 1)
                    for x in range(1, self.config.width - 1)
                    if self.world.is_free(Position(x, y))
                ),
                key=lambda position: (
                    abs(position.x - self.world.base.x)
                    + abs(position.y - self.world.base.y),
                    position.y,
                    position.x,
                ),
            )
            starts = tuple(candidates[: self.config.drone_count])
        else:
            starts = tuple(Position(x, y) for x, y in self.config.drone_start_positions)
        if (
            len(starts) != self.config.drone_count
            or any(not self.world.is_free(position) for position in starts)
        ):
            raise ValueError("every drone start position must be a free cell")
        non_base_starts = [
            position for position in starts if position != self.world.base
        ]
        if len(non_base_starts) != len(set(non_base_starts)):
            raise ValueError("only the shared docking base may have matching starts")
        return starts

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
                    or (
                        other.drone.status is DroneStatus.FAILED
                        and other.drone.position
                        in self._observable_failed_positions(runtime)
                    )
                )
                and other.drone.status is not DroneStatus.LANDED
                and other.drone.position != self.world.base
            )
        }

    def _decision_map(self, runtime: DroneRuntime) -> KnownMap:
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
        survivor_count: int | None = None,
        utility: float | None = None,
        reason: str | None = None,
        critical_backlog: int | None = None,
        backpressure: bool | None = None,
        route_hops: int | None = None,
        transfer_progress: float | None = None,
        smoke_density: float | None = None,
        detection_attempts: int | None = None,
        detection_successes: int | None = None,
        sensor_channel: str | None = None,
        observation_index: int | None = None,
        survivor_distance: float | None = None,
        detection_success: bool | None = None,
        detection_confidence: float | None = None,
        decision_score: float | None = None,
        noise_profile: str | None = None,
        raw_detection_success: bool | None = None,
        noisy_detection_success: bool | None = None,
        evidence_before: float | None = None,
        evidence_after: float | None = None,
        hypothesis_observations: int | None = None,
        hypothesis_status: str | None = None,
        hypothesis_confirmed: bool | None = None,
        hypothesis_rejected: bool | None = None,
        old_cell_state: str | None = None,
        new_cell_state: str | None = None,
        old_path_length: int | None = None,
        new_path_length: int | None = None,
        task_id: str | None = None,
        task_type: str | None = None,
        task_status: str | None = None,
        task_owner: str | None = None,
        old_role: str | None = None,
        new_role: str | None = None,
        reassignment_score: int | None = None,
        assignment_latency: int | None = None,
        served_agent_ids: tuple[str, ...] | None = None,
        active_relay_ids: tuple[str, ...] | None = None,
        forecast_disconnect_step: int | None = None,
        prediction_horizon: int | None = None,
    ) -> None:
        self.mission_log.record(
            MissionEvent(
                position=position or runtime.drone.position,
                step=self.steps,
                drone_id=runtime.drone.identifier,
                event_type=event_type,
                energy_remaining=runtime.battery.remaining,
                cell_count=cell_count,
                survivor_count=survivor_count,
                utility=utility,
                reason=reason,
                critical_backlog=critical_backlog,
                backpressure=backpressure,
                route_hops=route_hops,
                transfer_progress=transfer_progress,
                smoke_density=smoke_density,
                detection_attempts=detection_attempts,
                detection_successes=detection_successes,
                sensor_channel=sensor_channel,
                observation_index=observation_index,
                survivor_distance=survivor_distance,
                detection_success=detection_success,
                detection_confidence=detection_confidence,
                decision_score=decision_score,
                noise_profile=noise_profile,
                raw_detection_success=raw_detection_success,
                noisy_detection_success=noisy_detection_success,
                evidence_before=evidence_before,
                evidence_after=evidence_after,
                hypothesis_observations=hypothesis_observations,
                hypothesis_status=hypothesis_status,
                hypothesis_confirmed=hypothesis_confirmed,
                hypothesis_rejected=hypothesis_rejected,
                old_cell_state=old_cell_state,
                new_cell_state=new_cell_state,
                old_path_length=old_path_length,
                new_path_length=new_path_length,
                task_id=task_id,
                task_type=task_type,
                task_status=task_status,
                task_owner=task_owner,
                old_role=old_role,
                new_role=new_role,
                reassignment_score=reassignment_score,
                assignment_latency=assignment_latency,
                served_agent_ids=served_agent_ids,
                active_relay_ids=active_relay_ids,
                forecast_disconnect_step=forecast_disconnect_step,
                prediction_horizon=prediction_horizon,
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
        snapshot = self._compute_communication_snapshot()
        self.communication_snapshot = snapshot
        self._communication_samples += 1

        if self.knowledge_mode == "local" and previous is not None:
            def peer_pairs(
                candidate: CommunicationSnapshot,
            ) -> set[tuple[str, str]]:
                pairs: set[tuple[str, str]] = set()
                for component in self.shadow_synchronizer.connected_components(
                    candidate
                ):
                    members = sorted(
                        node_id for node_id in component if node_id in self.runtimes
                    )
                    for index, first in enumerate(members):
                        pairs.update((first, second) for second in members[index + 1 :])
                return pairs

            newly_connected = peer_pairs(snapshot) - peer_pairs(previous)
            if newly_connected:
                reconnected_ids = {item for pair in newly_connected for item in pair}
                for runtime in self._ordered_runtimes():
                    if runtime.drone.identifier not in reconnected_ids:
                        continue
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
        self._sample_multi_relay()

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

    @property
    def dynamic_obstacles_enabled(self) -> bool:
        return bool(self.dynamic_obstacle_events)

    def _inject_dynamic_obstacles(self) -> None:
        while (
            self._dynamic_event_index < len(self.dynamic_obstacle_events)
            and self.dynamic_obstacle_events[self._dynamic_event_index].step
            == self.steps
        ):
            event = self.dynamic_obstacle_events[self._dynamic_event_index]
            self._dynamic_event_index += 1
            occupying = next(
                (
                    runtime.drone.identifier
                    for runtime in self._ordered_runtimes()
                    if runtime.drone.position == event.position
                ),
                None,
            )
            if occupying is not None:
                self.mission_log.record(
                    MissionEvent(
                        position=event.position,
                        step=self.steps,
                        drone_id="environment",
                        event_type=EventType.DYNAMIC_OBSTACLE_REJECTED,
                        reason=f"occupied_by_{occupying}",
                        old_cell_state=CellState.FREE.value,
                        new_cell_state=CellState.FREE.value,
                    )
                )
                continue
            self.world.block_cell(event.position)
            self._dynamic_obstacles_injected.add(event.position)
            self.mission_log.record(
                MissionEvent(
                    position=event.position,
                    step=self.steps,
                    drone_id="environment",
                    event_type=EventType.DYNAMIC_OBSTACLE_INJECTED,
                    reason=event.cause,
                    old_cell_state=CellState.FREE.value,
                    new_cell_state=CellState.OCCUPIED.value,
                )
            )

    def _path_remaining_length(
        self, runtime: DroneRuntime, path: tuple[Position, ...]
    ) -> int:
        if runtime.drone.position in path:
            return len(path) - path.index(runtime.drone.position) - 1
        return max(0, len(path) - 1)

    def _invalidate_dynamic_paths(
        self,
        observer: DroneRuntime,
        observed_positions: set[Position],
    ) -> None:
        candidates = (
            (observer,)
            if self.knowledge_mode == "local"
            else self._ordered_runtimes()
        )
        for runtime in candidates:
            if runtime.terminal or runtime.dynamic_replan_pending:
                continue
            path = (
                runtime.current_return_path
                if runtime.drone.status is DroneStatus.RETURN_HOME
                else runtime.planned_path
            )
            blocked = next(
                (position for position in path[1:] if position in observed_positions),
                None,
            )
            if blocked is None:
                continue
            runtime.dynamic_replan_pending = True
            runtime.dynamic_replan_target = runtime.active_frontier_target
            runtime.dynamic_replan_old_path_length = self._path_remaining_length(
                runtime, path
            )
            runtime.dynamic_replan_return = (
                runtime.drone.status is DroneStatus.RETURN_HOME
            )
            if runtime.dynamic_replan_return:
                runtime.return_replan_required = True
            self._path_invalidations += 1
            self._replans_total += 1
            self._record_event(
                runtime,
                EventType.PATH_INVALIDATED,
                blocked,
                reason=(
                    "return_path_blocked"
                    if runtime.dynamic_replan_return
                    else "planned_path_blocked"
                ),
                old_path_length=runtime.dynamic_replan_old_path_length,
            )
            self._record_event(
                runtime,
                EventType.REPLAN_REQUESTED,
                blocked,
                reason=(
                    "return_path_blocked"
                    if runtime.dynamic_replan_return
                    else "planned_path_blocked"
                ),
                old_path_length=runtime.dynamic_replan_old_path_length,
            )

    def _register_dynamic_observations(
        self,
        runtime: DroneRuntime,
        observations: dict[Position, CellState],
        prior_states: dict[Position, CellState] | None = None,
    ) -> None:
        observed = {
            position
            for position, state in observations.items()
            if state is CellState.OCCUPIED
            and position in self._dynamic_obstacles_injected
        }
        for position in sorted(observed - self._dynamic_obstacles_observed):
            self._dynamic_obstacles_observed.add(position)
            self._record_event(
                runtime,
                EventType.DYNAMIC_OBSTACLE_OBSERVED,
                position,
                reason="distance_sensor",
                old_cell_state=(
                    prior_states.get(position, CellState.UNKNOWN).value
                    if prior_states is not None
                    else CellState.UNKNOWN.value
                ),
                new_cell_state=CellState.OCCUPIED.value,
            )
        self._invalidate_dynamic_paths(runtime, observed)

    def _sense(self, runtime: DroneRuntime) -> None:
        if runtime.terminal:
            return
        if not runtime.battery.consume(self.config.sensor_energy_cost):
            self._fail_energy(runtime)
            return
        self._advance_probability_maps()
        observations = self.sensor.observe(self.world, runtime.drone.position)
        if self.config.uncertainty_profile != "off":
            observations = uncertain_observations(observations, seed=self.config.seed,
                agent_id=runtime.drone.identifier, step=self.steps,
                profile=self.config.uncertainty_profile)
        prior_states = {
            position: self._decision_map(runtime).cell_at(position)
            for position in observations
            if self.config.uncertainty_profile != "off" or position in self._dynamic_obstacles_injected
        }
        if self.knowledge_sync_enabled:
            runtime.local_map.observe(
                observations,
                step=self.steps,
                source_id=runtime.drone.identifier,
            )
        if self.knowledge_mode != "local":
            if isinstance(self.occupancy_map, ProbabilisticKnowledgeMap):
                self.occupancy_map.observe(observations, step=self.steps,
                    source_id=runtime.drone.identifier)
            else:
                self.occupancy_map.update(observations)
        if self.config.uncertainty_profile != "off":
            self._invalidate_dynamic_paths(runtime, {
                position for position, previous in prior_states.items()
                if previous is CellState.FREE and
                self._decision_map(runtime).cell_at(position) is not CellState.FREE})
        elif self.dynamic_obstacles_enabled:
            self._register_dynamic_observations(runtime, observations, prior_states)
        self._observe_smoke_state(runtime)
        self._observe_survivors(runtime)

    def _observe_smoke_state(self, runtime: DroneRuntime) -> None:
        if self.config.smoke_profile == "off":
            return
        density = self.world.smoke.density_at(runtime.drone.position)
        currently_in_smoke = density > 0.0
        if currently_in_smoke:
            drone_id = runtime.drone.identifier
            self._smoke_exposure_samples_by_drone[drone_id] = (
                self._smoke_exposure_samples_by_drone.get(drone_id, 0) + 1
            )
        if currently_in_smoke == runtime.in_smoke:
            return
        runtime.in_smoke = currently_in_smoke
        if currently_in_smoke:
            drone_id = runtime.drone.identifier
            self._smoke_entries_by_drone[drone_id] = (
                self._smoke_entries_by_drone.get(drone_id, 0) + 1
            )
            self._record_event(
                runtime,
                EventType.SMOKE_ENTERED,
                smoke_density=density,
            )
        else:
            self._record_event(
                runtime,
                EventType.SMOKE_EXITED,
                smoke_density=0.0,
            )

    def _observe_survivors(self, runtime: DroneRuntime) -> None:
        report = self.survivor_sensor.observe_report(
            self.world,
            runtime.drone.position,
            smoke_profile=self.config.smoke_profile,
            seed=self.config.seed,
            step=self.steps,
            observer_id=runtime.drone.identifier,
            perception_noise=self.config.perception_noise,
        )
        self._survivor_detection_attempts += report.attempts
        self._survivor_detection_successes += report.successful_observations
        self._survivor_detection_failures += report.failed_observations
        self._smoke_degraded_detection_attempts += report.degraded_attempts
        self._smoke_successful_detection_attempts += (
            report.successful_smoke_observations
        )
        if self.config.perception_noise != "off":
            self._perception_tp += report.true_positives
            self._perception_fp += report.false_positives
            self._perception_tn += report.true_negatives
            self._perception_fn += report.false_negatives
            self._apply_noisy_survivor_evidence(runtime, report)
            return
        detailed_observations = (
            report.sensor_channel == "thermal"
            or self.config.smoke_profile != "off"
        )
        if detailed_observations:
            for index, observation in enumerate(report.observations, start=1):
                self._record_event(
                    runtime,
                    EventType.SURVIVOR_SENSOR_OBSERVATION,
                    reason=observation.failure_reason or "detected",
                    smoke_density=observation.smoke_exposure,
                    sensor_channel=report.sensor_channel,
                    observation_index=index,
                    survivor_distance=observation.distance,
                    detection_success=observation.success,
                    detection_confidence=observation.confidence,
                    decision_score=observation.decision_score,
                )
        if (
            self.config.smoke_profile != "off"
            and report.degraded_attempts
        ):
            self._smoke_degraded_detection_events += 1
            self._record_event(
                runtime,
                EventType.SURVIVOR_DETECTION_DEGRADED,
                smoke_density=report.maximum_smoke_exposure,
                detection_attempts=report.attempts,
                detection_successes=report.successful_observations,
                reason="smoke_visibility",
                sensor_channel=report.sensor_channel,
            )
        visible = report.visible_survivors
        event_channel = (
            report.sensor_channel if detailed_observations else None
        )
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
                        runtime,
                        EventType.SURVIVOR_DETECTED,
                        position,
                        sensor_channel=event_channel,
                    )
                if (
                    count >= self.config.survivor_confirmation_observations
                    and position not in runtime.confirmed_survivors
                ):
                    runtime.confirmed_survivors.add(position)
                    self._confirmed_survivors.add(position)
                    self._record_event(
                        runtime,
                        EventType.SURVIVOR_CONFIRMED,
                        position,
                        sensor_channel=event_channel,
                    )
                continue
            if position not in self._detected_survivors:
                self._detected_survivors.add(position)
                self._record_event(
                    runtime,
                    EventType.SURVIVOR_DETECTED,
                    position,
                    sensor_channel=event_channel,
                )
            if (
                count >= self.config.survivor_confirmation_observations
                and position not in self._confirmed_survivors
            ):
                self._confirmed_survivors.add(position)
                self._record_event(
                    runtime,
                    EventType.SURVIVOR_CONFIRMED,
                    position,
                    sensor_channel=event_channel,
                )

    def _apply_noisy_survivor_evidence(
        self, runtime: DroneRuntime, report: SurvivorObservationReport
    ) -> None:
        tracker = (self.local_hypothesis_trackers[runtime.drone.identifier]
                   if self.config.uncertainty_profile != "off" and self.knowledge_mode == "local"
                   else self.hypothesis_tracker)
        observed_positions: set[Position] = set()
        for index, observation in enumerate(report.observations, start=1):
            if not observation.success:
                continue
            observed_positions.add(observation.position)
            self._perception_calibration.append(
                (
                    observation.confidence,
                    observation.position in self.world.survivors,
                )
            )
            update = tracker.positive(
                observation.position,
                confidence=observation.confidence,
                channel=report.sensor_channel,
                agent_id=runtime.drone.identifier,
                step=self.steps,
            )
            runtime.detected_survivors.add(observation.position)
            self._detected_survivors.add(observation.position)
            self._record_event(
                runtime,
                EventType.SURVIVOR_SENSOR_OBSERVATION,
                observation.position,
                reason="evidence_reported",
                smoke_density=observation.smoke_exposure,
                sensor_channel=report.sensor_channel,
                observation_index=index,
                survivor_distance=observation.distance,
                detection_success=True,
                detection_confidence=observation.confidence,
                decision_score=observation.noise_score,
                noise_profile=self.config.perception_noise,
                raw_detection_success=True,
                noisy_detection_success=True,
                evidence_before=update.evidence_before,
                evidence_after=update.evidence_after,
                hypothesis_observations=update.observation_count,
                hypothesis_status=update.status_after.value,
                hypothesis_confirmed=update.confirmed,
                hypothesis_rejected=False,
            )
            if update.created:
                self._record_event(
                    runtime,
                    EventType.SURVIVOR_HYPOTHESIS_CREATED,
                    observation.position,
                    sensor_channel=report.sensor_channel,
                    detection_confidence=observation.confidence,
                    noise_profile=self.config.perception_noise,
                    evidence_before=update.evidence_before,
                    evidence_after=update.evidence_after,
                    hypothesis_observations=update.observation_count,
                    hypothesis_status=update.status_after.value,
                    hypothesis_confirmed=False,
                    hypothesis_rejected=False,
                )
                self._record_event(
                    runtime,
                    EventType.SURVIVOR_DETECTED,
                    observation.position,
                    sensor_channel=report.sensor_channel,
                )
            self._record_event(
                runtime,
                EventType.SURVIVOR_EVIDENCE_UPDATED,
                observation.position,
                sensor_channel=report.sensor_channel,
                detection_confidence=observation.confidence,
                noise_profile=self.config.perception_noise,
                evidence_before=update.evidence_before,
                evidence_after=update.evidence_after,
                hypothesis_observations=update.observation_count,
                hypothesis_status=update.status_after.value,
                hypothesis_confirmed=update.confirmed,
                hypothesis_rejected=False,
            )
            if update.confirmed:
                runtime.confirmed_survivors.add(observation.position)
                self._confirmed_survivors.add(observation.position)
                self._record_event(
                    runtime,
                    EventType.SURVIVOR_HYPOTHESIS_CONFIRMED,
                    observation.position,
                    sensor_channel=report.sensor_channel,
                    detection_confidence=observation.confidence,
                    noise_profile=self.config.perception_noise,
                    evidence_before=update.evidence_before,
                    evidence_after=update.evidence_after,
                    hypothesis_observations=update.observation_count,
                    hypothesis_status=update.status_after.value,
                    hypothesis_confirmed=True,
                    hypothesis_rejected=False,
                )
                self._record_event(
                    runtime,
                    EventType.SURVIVOR_CONFIRMED,
                    observation.position,
                    sensor_channel=report.sensor_channel,
                )

        for position, hypothesis in sorted(
            tracker.hypotheses.items()
        ):
            if (
                hypothesis.status is not HypothesisStatus.UNCONFIRMED
                or position in observed_positions
                or not self.survivor_sensor.can_observe(
                    self.world, runtime.drone.position, position
                )
            ):
                continue
            negative_confidence = (
                UNCERTAINTY_PROFILES[self.config.uncertainty_profile].survivor_reliability
                if self.config.uncertainty_profile != "off" else
                0.25 * self.survivor_sensor.base_detection_probability)
            negative_update = tracker.negative(
                position,
                confidence=negative_confidence,
                channel=report.sensor_channel,
                agent_id=runtime.drone.identifier,
                step=self.steps,
            )
            if negative_update is None:
                continue
            self._record_event(
                runtime,
                (
                    EventType.SURVIVOR_HYPOTHESIS_REJECTED
                    if negative_update.rejected
                    else EventType.SURVIVOR_EVIDENCE_UPDATED
                ),
                position,
                reason="negative_observation",
                sensor_channel=report.sensor_channel,
                detection_confidence=negative_confidence,
                noise_profile=self.config.perception_noise,
                raw_detection_success=False,
                noisy_detection_success=False,
                evidence_before=negative_update.evidence_before,
                evidence_after=negative_update.evidence_after,
                hypothesis_observations=negative_update.observation_count,
                hypothesis_status=negative_update.status_after.value,
                hypothesis_confirmed=False,
                hypothesis_rejected=negative_update.rejected,
            )
    @staticmethod
    def _relay_payload(
        runtime: DroneRuntime,
    ) -> tuple[set[Position], set[Position]]:
        records = {
            position: record for position, record in runtime.local_map.records
        }
        cells = {
            position
            for position, record in records.items()
            if runtime.base_acknowledged_records.get(position) != record
        }
        survivors = (
            runtime.confirmed_survivors
            - runtime.base_acknowledged_survivors
        )
        return cells, set(survivors)

    def _relay_plan_for(
        self,
        runtime: DroneRuntime,
        scout_position: Position,
    ) -> RelayPlan | None:
        blocked = frozenset(
            {
                scout_position,
            }
            if scout_position != self.world.base
            else set()
        )
        return select_relay_plan(
            runtime.local_map,
            runtime.drone.position,
            self.world.base,
            scout_position,
            max_range=self.config.communication_range,
            energy_remaining=runtime.battery.remaining,
            movement_cycle_cost=runtime.battery.movement_cycle_cost,
            safety_reserve=self.config.energy_safety_reserve,
            energy_margin=self.config.relay_energy_margin,
            blocked=blocked,
        )

    def _set_relay_plan(
        self,
        runtime: DroneRuntime,
        plan: RelayPlan,
    ) -> None:
        target_changed = runtime.relay_target != plan.position
        runtime.relay_target = plan.position
        runtime.relay_plan = plan
        runtime.planned_path = plan.path
        if target_changed:
            self._record_event(
                runtime,
                EventType.RELAY_POSITION_SELECTED,
                plan.position,
            )

    def _forecast_position(
        self, runtime: DroneRuntime, forecast_step: int | None
    ) -> Position:
        if forecast_step is None:
            return runtime.drone.position
        path = (
            runtime.current_return_path
            if runtime.drone.status is DroneStatus.RETURN_HOME
            else runtime.planned_path
        )
        offset = max(0, forecast_step - self.steps)
        if runtime.drone.position in path:
            start = path.index(runtime.drone.position)
            return path[min(len(path) - 1, start + offset)]
        return path[min(len(path) - 1, offset)] if path else runtime.drone.position

    def _multi_relay_service_needs(
        self,
    ) -> list[
        tuple[
            int,
            int,
            int,
            str,
            Position,
            set[Position],
            set[Position],
            ConnectivityForecast | None,
        ]
    ]:
        needs = []
        for runtime in self._ordered_runtimes():
            drone_id = runtime.drone.identifier
            if (
                runtime.terminal
                or runtime.drone.status is DroneStatus.RELAY
                or runtime.drone.status not in {
                    DroneStatus.EXPLORE,
                    DroneStatus.RETURN_HOME,
                }
            ):
                continue
            cells, survivors = self._relay_payload(runtime)
            critical = len(survivors) + int(
                runtime.drone.status is DroneStatus.RETURN_HOME
                or runtime.energy_emergency
            )
            backlog = len(cells) + len(survivors)
            connection = self.communication_snapshot.connections[drone_id]
            outage = self._current_outage_steps.get(drone_id, 0)
            forecast: ConnectivityForecast | None = None
            forecast_step: int | None = None
            if self.predictive_relay_enabled and runtime.planned_path:
                forecast = forecast_connectivity(
                    runtime.local_map,
                    base=self.world.base,
                    # The forecast is deliberately self-contained: another
                    # agent's live position may be stale or unavailable in
                    # constrained/local mode.  Existing Relay topology is
                    # evaluated reactively by the communication snapshot.
                    agent_positions={drone_id: runtime.drone.position},
                    agent_id=drone_id,
                    planned_path=(
                        runtime.current_return_path
                        if runtime.drone.status is DroneStatus.RETURN_HOME
                        else runtime.planned_path
                    ),
                    current_step=self.steps,
                    horizon=self.config.relay_prediction_horizon,
                    max_range=self.config.communication_range,
                )
                self._multi_relay_predictive_forecasts += 1
                previous = self._multi_relay_forecasts.get(drone_id)
                self._multi_relay_forecasts[drone_id] = forecast
                forecast_step = forecast.expected_disconnection_step
                if (
                    forecast_step is not None
                    and (
                        previous is None
                        or previous.expected_disconnection_step is None
                    )
                ):
                    self._record_event(
                        runtime,
                        EventType.PREDICTIVE_LINK_FORECAST,
                        reason="planned_path_link_loss",
                        route_hops=forecast.current_hop_count,
                        forecast_disconnect_step=forecast_step,
                        prediction_horizon=self.config.relay_prediction_horizon,
                    )
            reactive = (
                not connection.connected_to_base
                and outage >= self.config.multi_relay_activation_outage_steps
            )
            predictive = (
                self.predictive_relay_enabled
                and connection.connected_to_base
                and forecast_step is not None
            )
            relevant = (
                critical > 0
                or backlog >= self.config.multi_relay_min_unsynced_cells
            )
            if not relevant or not (reactive or predictive):
                continue
            target_position = self._forecast_position(runtime, forecast_step)
            needs.append(
                (
                    -critical,
                    -int(reactive),
                    -max(outage, backlog),
                    drone_id,
                    target_position,
                    cells,
                    survivors,
                    forecast,
                )
            )
        return sorted(needs)

    def _select_multi_relay_plan(
        self,
        need: tuple[
            int,
            int,
            int,
            str,
            Position,
            set[Position],
            set[Position],
            ConnectivityForecast | None,
        ],
        all_needs: list[
            tuple[
                int,
                int,
                int,
                str,
                Position,
                set[Position],
                set[Position],
                ConnectivityForecast | None,
            ]
        ],
    ) -> MultiRelayPlan | None:
        _, _, _, primary_id, _, _, _, forecast = need
        service_positions = {row[3]: row[4] for row in all_needs}
        primary = self.runtimes[primary_id]
        topologies = relay_target_topologies(
            primary.local_map,
            base=self.world.base,
            service_positions=service_positions,
            primary_agent_id=primary_id,
            max_relays=self.config.multi_relay_max_active,
            max_range=self.config.communication_range,
            candidate_limit=self.config.multi_relay_candidate_limit,
        )
        service_ids = set(service_positions)
        relay_candidates = tuple(
            runtime
            for runtime in self._ordered_runtimes()
            if runtime.drone.identifier not in service_ids
            and runtime.drone.status is DroneStatus.EXPLORE
            and self.steps >= runtime.relay_cooldown_until_step
            and self._multi_relay_activation_counts.get(
                runtime.drone.identifier, 0
            )
            < 2
        )
        transport = self.network_transport
        queue_units = transport.queued_payload_units if transport is not None else 0
        choices: list[
            tuple[
                int,
                int,
                tuple[str, ...],
                tuple[Position, ...],
                MultiRelayPlan,
            ]
        ] = []
        for topology in topologies:
            count = len(topology.targets)
            if count > len(relay_candidates):
                continue
            for ordered_relays in permutations(relay_candidates, count):
                paths: list[tuple[Position, ...]] = []
                return_paths: list[tuple[Position, ...]] = []
                total_score = 0
                valid = True
                for index, (relay, target) in enumerate(
                    zip(ordered_relays, topology.targets)
                ):
                    blocked = self._observable_failed_positions(relay)
                    path = astar(
                        relay.drone.position,
                        target,
                        lambda position: relay.local_map.is_known_free(position)
                        and (
                            position == relay.drone.position
                            or position not in blocked
                        ),
                    )
                    return_path = astar(
                        target,
                        self.world.base,
                        lambda position: relay.local_map.is_known_free(position)
                        and (position == target or position not in blocked),
                    )
                    if path is None or return_path is None:
                        valid = False
                        break
                    required = (
                        relay.battery.estimate_path(len(path))
                        + relay.battery.estimate_path(len(return_path))
                        + self.config.energy_safety_reserve
                        + self.config.relay_energy_margin
                    )
                    if relay.battery.remaining + 1e-9 < required:
                        valid = False
                        break
                    paths.append(path)
                    return_paths.append(return_path)
                    predicted_steps = (
                        self.config.relay_prediction_horizon
                        if index == 0
                        and forecast is not None
                        and forecast.expected_disconnection_step is not None
                        else 0
                    )
                    total_score += relay_candidate_score(
                        travel_steps=max(0, len(path) - 1),
                        task_interrupted=(
                            relay.active_frontier_target is not None
                        ),
                        role=relay.role.value,
                        consumed_energy=relay.battery.consumed,
                        agents_helped=(
                            len(topology.served_agent_ids)
                            if index == 0
                            else 0
                        ),
                        queue_units=queue_units if index == 0 else 0,
                        predicted_steps_prevented=predicted_steps,
                    )
                if not valid:
                    continue
                relay_ids = tuple(
                    runtime.drone.identifier for runtime in ordered_relays
                )
                reason = (
                    "predicted_link_loss"
                    if forecast is not None
                    and forecast.expected_disconnection_step is not None
                    and self.communication_snapshot.connections[
                        primary_id
                    ].connected_to_base
                    else "reactive_connectivity_recovery"
                )
                choices.append(
                    (
                        total_score,
                        count,
                        relay_ids,
                        topology.targets,
                        MultiRelayPlan(
                            relay_ids=relay_ids,
                            targets=topology.targets,
                            paths=tuple(paths),
                            return_paths=tuple(return_paths),
                            served_agent_ids=topology.served_agent_ids,
                            score=total_score,
                            reason=reason,
                            predictive=reason == "predicted_link_loss",
                            forecast_disconnect_step=(
                                forecast.expected_disconnection_step
                                if forecast is not None
                                else None
                            ),
                        ),
                    )
                )
        return min(choices)[-1] if choices else None

    def _activate_multi_relay_plan(self, plan: MultiRelayPlan) -> None:
        for index, (relay_id, target, path, return_path) in enumerate(
            zip(plan.relay_ids, plan.targets, plan.paths, plan.return_paths)
        ):
            runtime = self.runtimes[relay_id]
            runtime.drone.status = DroneStatus.RELAY
            runtime.active_frontier_target = None
            runtime.relay_scout_id = plan.served_agent_ids[0]
            runtime.relay_served_ids = set(plan.served_agent_ids)
            runtime.relay_upstream_id = (
                plan.relay_ids[index - 1] if index else None
            )
            runtime.relay_downstream_id = (
                plan.relay_ids[index + 1]
                if index + 1 < len(plan.relay_ids)
                else None
            )
            runtime.relay_scout_position = self.runtimes[
                plan.served_agent_ids[0]
            ].drone.position
            runtime.relay_started_step = self.steps
            runtime.relay_link_achieved = False
            runtime.relay_payload_forwarded = False
            payload_cells: set[Position] = set()
            payload_survivors: set[Position] = set()
            for served_id in plan.served_agent_ids:
                cells, survivors = self._relay_payload(
                    self.runtimes[served_id]
                )
                payload_cells.update(cells)
                payload_survivors.update(survivors)
            runtime.relay_payload_positions = payload_cells
            runtime.relay_payload_survivors = payload_survivors
            runtime.relay_energy_at_start = runtime.battery.remaining
            runtime.relay_path_length_at_start = runtime.drone.path_length
            runtime.relay_outage_at_start = max(
                (
                    self._current_outage_steps.get(served_id, 0)
                    for served_id in plan.served_agent_ids
                ),
                default=0,
            )
            runtime.network_relay_reason = plan.reason
            runtime.network_relay_critical_backlog = len(payload_survivors)
            runtime.network_relay_expected_units = (
                len(payload_cells) + len(payload_survivors)
            )
            deployment = RelayDeployment(
                relay_id=relay_id,
                target=target,
                served_agent_ids=plan.served_agent_ids,
                upstream_relay_id=runtime.relay_upstream_id,
                downstream_relay_id=runtime.relay_downstream_id,
                activated_step=self.steps,
                minimum_release_step=(
                    self.steps + self.config.multi_relay_min_hold_steps
                ),
                reason=plan.reason,
                predictive=plan.predictive,
                forecast_disconnect_step=plan.forecast_disconnect_step,
                score=plan.score,
                payload_observed_steps={
                    position: max(
                        record.observed_step
                        for served_id in plan.served_agent_ids
                        if (
                            record := self.runtimes[
                                served_id
                            ].local_map.record_at(position)
                        )
                        is not None
                    )
                    for position in payload_cells
                },
            )
            self.relay_deployments_by_agent[relay_id] = deployment
            self.active_relays.add(relay_id)
            self._multi_relay_served_totals.setdefault(relay_id, set()).update(
                plan.served_agent_ids
            )
            self._multi_relay_activation_counts[relay_id] = (
                self._multi_relay_activation_counts.get(relay_id, 0) + 1
            )
            if self.roles_enabled:
                self._transition_role(
                    runtime,
                    AgentRole.RELAY,
                    reason=plan.reason,
                )
            relay_plan = RelayPlan(
                position=target,
                path=path,
                return_path=return_path,
                movement_cost=max(0, len(path) - 1),
                energy_required=(
                    runtime.battery.estimate_path(len(path))
                    + runtime.battery.estimate_path(len(return_path))
                    + self.config.energy_safety_reserve
                    + self.config.relay_energy_margin
                ),
            )
            self._set_relay_plan(runtime, relay_plan)
            self.relay_deployments += 1
            self._multi_relay_activations += 1
            if plan.predictive:
                self._multi_relay_predictive_activations += 1
            self._record_event(
                runtime,
                EventType.RELAY_ROLE_ASSIGNED,
                target,
                reason=plan.reason,
                route_hops=len(plan.relay_ids) + 1,
                served_agent_ids=plan.served_agent_ids,
                active_relay_ids=tuple(sorted(self.active_relays)),
                forecast_disconnect_step=plan.forecast_disconnect_step,
                prediction_horizon=(
                    self.config.relay_prediction_horizon
                    if plan.predictive
                    else None
                ),
            )
            self._record_event(
                runtime,
                (
                    EventType.PREDICTIVE_RELAY_ACTIVATED
                    if plan.predictive
                    else EventType.MULTI_RELAY_ACTIVATED
                ),
                target,
                reason=plan.reason,
                utility=float(-plan.score),
                route_hops=len(plan.relay_ids) + 1,
                served_agent_ids=plan.served_agent_ids,
                active_relay_ids=tuple(sorted(self.active_relays)),
                forecast_disconnect_step=plan.forecast_disconnect_step,
                prediction_horizon=(
                    self.config.relay_prediction_horizon
                    if plan.predictive
                    else None
                ),
            )
        topology_runtime = self.runtimes[plan.relay_ids[0]]
        self._record_event(
            topology_runtime,
            EventType.MULTI_RELAY_TOPOLOGY_CHANGED,
            reason="relay_topology_activated",
            route_hops=len(plan.relay_ids) + 1,
            served_agent_ids=plan.served_agent_ids,
            active_relay_ids=tuple(sorted(self.active_relays)),
            forecast_disconnect_step=plan.forecast_disconnect_step,
            prediction_horizon=(
                self.config.relay_prediction_horizon
                if plan.predictive
                else None
            ),
        )

    def _recover_orphaned_relay_tasks(self) -> None:
        for task_id, context in tuple(
            sorted(self._orphaned_relay_recoveries.items())
        ):
            task = self.task_registry.tasks.get(task_id)
            if task is None or task.status is not TaskStatus.ORPHANED:
                self._orphaned_relay_recoveries.pop(task_id, None)
                continue
            candidates: list[
                tuple[
                    int,
                    int,
                    float,
                    str,
                    Position,
                    tuple[Position, ...],
                    tuple[Position, ...],
                ]
            ] = []
            for runtime in self._ordered_runtimes():
                relay_id = runtime.drone.identifier
                if (
                    runtime.drone.status is not DroneStatus.EXPLORE
                    or relay_id in context.served_agent_ids
                    or self.steps < runtime.relay_cooldown_until_step
                    or self._multi_relay_activation_counts.get(relay_id, 0)
                    >= 2
                ):
                    continue
                blocked = self._observable_failed_positions(runtime)
                nearby_targets = tuple(
                    position
                    for position, record in runtime.local_map.records
                    if record.state is CellState.FREE
                    and position not in blocked
                    and abs(position.x - context.target.x)
                    + abs(position.y - context.target.y)
                    <= 2
                )
                for target in sorted(
                    set((context.target,) + nearby_targets),
                    key=lambda position: (
                        abs(position.x - context.target.x)
                        + abs(position.y - context.target.y),
                        position,
                    ),
                ):
                    if target in blocked or not runtime.local_map.is_known_free(target):
                        continue
                    upstream_position = self.world.base
                    if (
                        context.upstream_relay_id is not None
                        and context.upstream_relay_id in self.runtimes
                        and not self.runtimes[context.upstream_relay_id].terminal
                    ):
                        upstream_position = self.runtimes[
                            context.upstream_relay_id
                        ].drone.position
                    if not known_radio_link(
                        runtime.local_map,
                        upstream_position,
                        target,
                        self.config.communication_range,
                    ):
                        continue
                    downstream_positions: list[Position] = []
                    if (
                        context.downstream_relay_id is not None
                        and context.downstream_relay_id in self.runtimes
                        and not self.runtimes[context.downstream_relay_id].terminal
                    ):
                        downstream_positions.append(
                            self.runtimes[
                                context.downstream_relay_id
                            ].drone.position
                        )
                    else:
                        downstream_positions.extend(
                            self.runtimes[served_id].drone.position
                            for served_id in context.served_agent_ids
                            if served_id in self.runtimes
                            and not self.runtimes[served_id].terminal
                        )
                    if not downstream_positions or not any(
                        known_radio_link(
                            runtime.local_map,
                            target,
                            downstream,
                            self.config.communication_range,
                        )
                        for downstream in downstream_positions
                    ):
                        continue
                    path = astar(
                        runtime.drone.position,
                        target,
                        lambda position: runtime.local_map.is_known_free(position)
                        and (
                            position == runtime.drone.position
                            or position not in blocked
                        ),
                    )
                    return_path = astar(
                        target,
                        self.world.base,
                        lambda position: runtime.local_map.is_known_free(position)
                        and (position == target or position not in blocked),
                    )
                    if path is None or return_path is None:
                        continue
                    required = (
                        runtime.battery.estimate_path(len(path))
                        + runtime.battery.estimate_path(len(return_path))
                        + self.config.energy_safety_reserve
                        + self.config.relay_energy_margin
                    )
                    if runtime.battery.remaining + 1e-9 < required:
                        continue
                    score = relay_candidate_score(
                        travel_steps=max(0, len(path) - 1),
                        task_interrupted=runtime.active_frontier_target is not None,
                        role=runtime.role.value,
                        consumed_energy=runtime.battery.consumed,
                        agents_helped=len(context.served_agent_ids),
                        queue_units=(
                            self.network_transport.queued_payload_units
                            if self.network_transport is not None
                            else 0
                        ),
                        predicted_steps_prevented=0,
                    )
                    candidates.append(
                        (
                            score,
                            max(0, len(path) - 1),
                            -runtime.battery.remaining,
                            relay_id,
                            target,
                            path,
                            return_path,
                        )
                    )
            if not candidates:
                continue
            score, _, _, relay_id, target, path, return_path = min(candidates)
            runtime = self.runtimes[relay_id]
            reassigned = self.task_registry.assign_orphan(
                task_id,
                owner_id=relay_id,
                target=target,
                step=self.steps,
            )
            runtime.drone.status = DroneStatus.RELAY
            runtime.active_frontier_target = None
            runtime.relay_scout_id = context.served_agent_ids[0]
            runtime.relay_served_ids = set(context.served_agent_ids)
            runtime.relay_upstream_id = context.upstream_relay_id
            runtime.relay_downstream_id = context.downstream_relay_id
            runtime.relay_started_step = self.steps
            runtime.relay_payload_positions = set()
            runtime.relay_payload_survivors = set()
            for served_id in context.served_agent_ids:
                cells, survivors = self._relay_payload(self.runtimes[served_id])
                runtime.relay_payload_positions.update(cells)
                runtime.relay_payload_survivors.update(survivors)
            runtime.relay_energy_at_start = runtime.battery.remaining
            runtime.relay_path_length_at_start = runtime.drone.path_length
            runtime.recovery_task_id = task_id
            replacement = RelayDeployment(
                relay_id=relay_id,
                target=target,
                served_agent_ids=context.served_agent_ids,
                upstream_relay_id=context.upstream_relay_id,
                downstream_relay_id=context.downstream_relay_id,
                activated_step=self.steps,
                minimum_release_step=(
                    self.steps + self.config.multi_relay_min_hold_steps
                ),
                reason="relay_failure_recovery",
                predictive=context.predictive,
                forecast_disconnect_step=context.forecast_disconnect_step,
                score=score,
                task_id=task_id,
                payload_observed_steps={
                    position: max(
                        record.observed_step
                        for served_id in context.served_agent_ids
                        if (
                            record := self.runtimes[
                                served_id
                            ].local_map.record_at(position)
                        )
                        is not None
                    )
                    for position in runtime.relay_payload_positions
                },
            )
            self.active_relays.add(relay_id)
            self.relay_deployments_by_agent[relay_id] = replacement
            for other in self.relay_deployments_by_agent.values():
                if other.upstream_relay_id == context.relay_id:
                    other.upstream_relay_id = relay_id
                if other.downstream_relay_id == context.relay_id:
                    other.downstream_relay_id = relay_id
            if self.roles_enabled:
                self._transition_role(
                    runtime,
                    AgentRole.RELAY,
                    reason="failure_recovery",
                    task=reassigned,
                    emergency_takeover=True,
                )
            required = (
                runtime.battery.estimate_path(len(path))
                + runtime.battery.estimate_path(len(return_path))
                + self.config.energy_safety_reserve
                + self.config.relay_energy_margin
            )
            self._set_relay_plan(
                runtime,
                RelayPlan(
                    position=target,
                    path=path,
                    return_path=return_path,
                    movement_cost=max(0, len(path) - 1),
                    energy_required=required,
                ),
            )
            latency = self.steps - (reassigned.orphaned_step or self.steps)
            self._reassignment_latencies.append(latency)
            self._failure_tasks_reassigned += 1
            self.relay_deployments += 1
            self._multi_relay_activations += 1
            self._multi_relay_failure_recoveries += 1
            self._multi_relay_served_totals.setdefault(relay_id, set()).update(
                context.served_agent_ids
            )
            self._multi_relay_activation_counts[relay_id] = (
                self._multi_relay_activation_counts.get(relay_id, 0) + 1
            )
            self._record_event(
                runtime,
                EventType.TASK_REASSIGNED,
                target,
                reason=context.relay_id,
                task_id=task_id,
                task_type=TaskType.RELAY_POSITIONING.value,
                task_status=reassigned.status.value,
                task_owner=relay_id,
                reassignment_score=score,
                assignment_latency=latency,
                served_agent_ids=context.served_agent_ids,
                active_relay_ids=tuple(sorted(self.active_relays)),
            )
            self._record_event(
                runtime,
                EventType.FAILURE_TASK_REASSIGNED,
                target,
                reason=context.relay_id,
                task_id=task_id,
                task_type=TaskType.RELAY_POSITIONING.value,
                task_owner=relay_id,
                reassignment_score=score,
                assignment_latency=latency,
            )
            if len(path) == 1 and reassigned.orphaned_step is not None:
                reassigned.execution_recovered_step = self.steps
                self._recovery_latencies.append(
                    self.steps - reassigned.orphaned_step
                )
            self._orphan_failure_tasks.pop(task_id, None)
            self._orphaned_relay_recoveries.pop(task_id, None)
            self._released_failure_tasks.pop(context.target, None)

    def _assign_multi_relay(self) -> None:
        if not self.multi_relay_enabled or self._exploration_complete:
            return
        self._recover_orphaned_relay_tasks()
        if self._orphaned_relay_recoveries or self.active_relays:
            return
        if (
            self._multi_relay_activations
            >= self.config.multi_relay_max_deployments
        ):
            return
        needs = self._multi_relay_service_needs()
        for need in needs:
            plan = self._select_multi_relay_plan(need, needs)
            if plan is not None:
                self._activate_multi_relay_plan(plan)
                return

    def _maintain_multi_relay(self) -> None:
        if not self.multi_relay_enabled or not self.active_relays:
            return
        active = tuple(sorted(self.active_relays))
        if any(
            self.runtimes[relay_id].dynamic_replan_pending
            for relay_id in active
        ):
            self._multi_relay_replans += 1
            for relay_id in active:
                runtime = self.runtimes[relay_id]
                runtime.dynamic_replan_pending = False
                self._finish_relay_role(runtime, successful=False)
            return
        release_ready = True
        timed_out = False
        for relay_id in active:
            runtime = self.runtimes[relay_id]
            deployment = self.relay_deployments_by_agent[relay_id]
            return_path = self._known_return_path(
                runtime, avoid_other_drones=False
            )
            required = (
                runtime.battery.estimate_path(len(return_path))
                if return_path is not None
                else float("inf")
            )
            if runtime.battery.remaining + 1e-9 < (
                required
                + self.config.energy_safety_reserve
                + self.config.relay_energy_margin
            ):
                self._abort_relay_for_energy(runtime)
                continue
            pending_cells = {
                position
                for position, observed_step in (
                    deployment.payload_observed_steps.items()
                )
                if self.base_knowledge_map is None
                or self.base_knowledge_map.record_at(position) is None
                or cast(
                    CellKnowledge,
                    self.base_knowledge_map.record_at(position),
                ).observed_step
                < observed_step
            }
            pending_survivors = (
                runtime.relay_payload_survivors
                - self._base_confirmed_survivors
            )
            served_connected = all(
                self.communication_snapshot.connections[served_id].connected_to_base
                for served_id in deployment.served_agent_ids
                if served_id in self.communication_snapshot.connections
            )
            stable = served_connected and not pending_cells and not pending_survivors
            deployment.payload_delivered = not pending_cells and not pending_survivors
            if stable and self.steps >= deployment.minimum_release_step:
                deployment.stable_release_steps += 1
            else:
                deployment.stable_release_steps = 0
            release_ready = release_ready and (
                deployment.stable_release_steps
                >= self.config.multi_relay_deactivation_hysteresis_steps
            )
            timed_out = timed_out or (
                self.steps - deployment.activated_step
                >= self.config.multi_relay_max_role_steps
            )
        if not self.active_relays:
            return
        if release_ready or timed_out:
            successful = release_ready
            for relay_id in tuple(sorted(self.active_relays)):
                runtime = self.runtimes[relay_id]
                release_deployment = self.relay_deployments_by_agent.get(
                    relay_id
                )
                self._record_event(
                    runtime,
                    EventType.MULTI_RELAY_DEACTIVATED,
                    reason=(
                        "stable_payload_delivery"
                        if successful
                        else "maximum_role_duration"
                    ),
                    served_agent_ids=(
                        release_deployment.served_agent_ids
                        if release_deployment is not None
                        else ()
                    ),
                    active_relay_ids=tuple(sorted(self.active_relays)),
                )
                self._finish_relay_role(runtime, successful=successful)

    def _sample_multi_relay(self) -> None:
        if not self.multi_relay_enabled:
            return
        active = tuple(sorted(self.active_relays))
        self._multi_relay_active_samples.append(len(active))
        active_links = sum(
            link.first in self.active_relays or link.second in self.active_relays
            for link in self.communication_snapshot.links
        )
        self._multi_relay_concurrent_link_samples.append(active_links)
        for runtime in self._ordered_runtimes():
            if runtime.drone.status is DroneStatus.FAILED:
                continue
            connection = self.communication_snapshot.connections[
                runtime.drone.identifier
            ]
            if not connection.connected_to_base:
                self._multi_relay_disconnected_agent_steps += 1
            else:
                hops = (
                    len(connection.relay_path) - 1
                    if connection.relay_path
                    else 1
                )
                self._multi_relay_hop_samples.append(hops)
                if connection.via_relay:
                    self._multi_relay_connected_via_relay_steps += 1
        if self._pending_critical_survivors():
            self._multi_relay_unsynced_critical_samples += 1
        for relay_id in active:
            deployment = self.relay_deployments_by_agent.get(relay_id)
            if deployment is None:
                continue
            for served_id in deployment.served_agent_ids:
                served_connection = (
                    self.communication_snapshot.connections.get(served_id)
                )
                if (
                    served_connection is None
                    or not served_connection.connected_to_base
                ):
                    continue
                deployment.connected_service_steps += 1
                if served_connection.via_relay:
                    deployment.relayed_service_steps += 1
                if (
                    deployment.predictive
                    and deployment.forecast_disconnect_step is not None
                    and self.steps >= deployment.forecast_disconnect_step
                ):
                    self._multi_relay_prevented_disconnect_steps += 1

    def _finish_relay_role(
        self,
        runtime: DroneRuntime,
        *,
        successful: bool,
    ) -> None:
        if runtime.drone.status is not DroneStatus.RELAY:
            return
        relay_id = runtime.drone.identifier
        multi_deployment = self.relay_deployments_by_agent.get(relay_id)
        scout_id = runtime.relay_scout_id
        if successful:
            self.successful_relay_deployments += 1
        else:
            self.failed_relay_deployments += 1
        if runtime.relay_energy_at_start is not None:
            self._relay_energy_consumed += max(
                0.0,
                runtime.relay_energy_at_start - runtime.battery.remaining,
            )
        self._record_event(runtime, EventType.RELAY_ROLE_RELEASED)
        if self.roles_enabled:
            current_task = self.task_registry.current_for(relay_id)
            if (
                current_task is not None
                and current_task.task_type is TaskType.RELAY_POSITIONING
            ):
                self._finish_runtime_task(runtime, completed=successful)
        runtime.drone.status = DroneStatus.EXPLORE
        if self.roles_enabled and runtime.role is AgentRole.RELAY:
            self._transition_role(
                runtime,
                runtime.base_role,
                reason="relay_task_completed",
            )
        runtime.active_frontier_target = None
        runtime.planned_path = ()
        runtime.relay_target = None
        runtime.relay_scout_id = None
        runtime.relay_served_ids.clear()
        runtime.relay_upstream_id = None
        runtime.relay_downstream_id = None
        runtime.relay_scout_position = None
        runtime.relay_plan = None
        runtime.relay_started_step = None
        runtime.relay_link_achieved = False
        runtime.relay_payload_forwarded = False
        runtime.relay_payload_positions.clear()
        runtime.relay_payload_survivors.clear()
        runtime.relay_energy_at_start = None
        runtime.network_relay_utility = None
        runtime.network_relay_reason = None
        runtime.network_relay_critical_backlog = 0
        runtime.network_relay_expected_units = 0
        runtime.network_relay_forwarded_units = 0
        runtime.network_relay_backpressure = False
        runtime.relay_cooldown_until_step = (
            self.steps + self.config.relay_cooldown_steps
        )
        if relay_id in self.active_relays:
            self.active_relays.discard(relay_id)
            self._multi_relay_deactivations += 1
            if (
                multi_deployment is not None
                and multi_deployment.predictive
                and multi_deployment.relayed_service_steps == 0
            ):
                self._multi_relay_unnecessary_predictive_activations += 1
            self.relay_deployments_by_agent.pop(relay_id, None)
        if scout_id in self.runtimes:
            self.runtimes[scout_id].holding_for_relay = False

    def _abort_relay_for_energy(self, runtime: DroneRuntime) -> None:
        self._record_event(runtime, EventType.RELAY_ABORTED_FOR_ENERGY)
        self._finish_relay_role(runtime, successful=False)
        path = self._known_return_path(runtime, avoid_other_drones=False)
        if path is None:
            self._fail_return_path(runtime)
        else:
            self._start_return(runtime, path)

    def _assign_adaptive_relay(self) -> None:
        if not self.adaptive_relay_enabled:
            return
        # Frontier allocation runs immediately before this method.  Refuse a
        # deployment once the local planners have established that exploration
        # is complete: normal RTB will restore base connectivity more cheaply.
        if self._exploration_complete:
            return
        if self.relay_deployments >= self.config.relay_max_deployments:
            return
        if any(
            runtime.drone.status is DroneStatus.RELAY
            for runtime in self._ordered_runtimes()
        ):
            return
        if not self.communication_snapshot.has_link("drone-1", "drone-2"):
            return
        if any(
            connection.connected_to_base
            for connection in self.communication_snapshot.connections.values()
        ):
            return

        tasks = []
        drone_ids = tuple(sorted(self.runtimes))
        for relay_id in drone_ids:
            scout_id = next(
                drone_id for drone_id in drone_ids if drone_id != relay_id
            )
            relay = self.runtimes[relay_id]
            scout = self.runtimes[scout_id]
            if (
                relay.drone.status is not DroneStatus.EXPLORE
                or scout.drone.status is not DroneStatus.EXPLORE
                or self.steps < relay.relay_cooldown_until_step
            ):
                continue
            outage = self._current_outage_steps.get(scout_id, 0)
            if outage < self.config.relay_min_outage_steps:
                continue
            cells, survivors = self._relay_payload(scout)
            if (
                not survivors
                and (
                    self.base_knowledge_map is None
                    or len(cells) < self.config.relay_min_unsynced_cells
                )
            ):
                continue
            plan = self._relay_plan_for(relay, scout.drone.position)
            if plan is None:
                continue
            benefit = 100 * len(survivors) + len(cells) + outage
            ratio = benefit / max(1, plan.movement_cost)
            if ratio + 1e-9 < self.config.relay_min_benefit_ratio:
                continue
            tasks.append(
                (
                    -len(survivors),
                    -len(cells),
                    -outage,
                    plan.movement_cost,
                    -relay.battery.remaining,
                    relay_id,
                    scout_id,
                    plan,
                    cells,
                    survivors,
                )
            )
        if not tasks:
            return
        (
            _,
            _,
            _,
            _,
            _,
            relay_id,
            scout_id,
            plan,
            cells,
            survivors,
        ) = min(tasks)
        relay = self.runtimes[relay_id]
        relay.drone.status = DroneStatus.RELAY
        if self.roles_enabled:
            self._transition_role(
                relay,
                AgentRole.RELAY,
                reason="relay_needed",
            )
        relay.active_frontier_target = None
        relay.relay_scout_id = scout_id
        relay.relay_scout_position = self.runtimes[scout_id].drone.position
        relay.relay_started_step = self.steps
        relay.relay_link_achieved = False
        relay.relay_payload_forwarded = False
        relay.relay_payload_positions = set(cells)
        relay.relay_payload_survivors = set(survivors)
        relay.relay_energy_at_start = relay.battery.remaining
        relay.relay_path_length_at_start = relay.drone.path_length
        relay.relay_outage_at_start = self._current_outage_steps.get(
            scout_id, 0
        )
        self.runtimes[scout_id].holding_for_relay = True
        self.relay_deployments += 1
        self._record_event(relay, EventType.RELAY_ROLE_ASSIGNED)
        self._set_relay_plan(relay, plan)

    def _network_relay_decision(
        self,
        relay: DroneRuntime,
        scout: DroneRuntime,
        plan: RelayPlan,
        cells: set[Position],
        survivors: set[Position],
    ) -> RelayUtilityDecision:
        records = dict(scout.local_map.records)
        recent_cutoff = self.steps - self.config.network_relay_recent_map_age_steps
        recent = sum(
            1
            for position in cells
            if records[position].observed_step >= recent_cutoff
        )
        general = max(0, len(cells) - recent)
        transport = self.network_transport
        assert transport is not None
        return_path = plan.return_path
        return_energy = relay.battery.estimate_path(len(return_path))
        energy_headroom = (
            relay.battery.remaining
            - return_energy
            - self.config.energy_safety_reserve
            - self.config.relay_energy_margin
            - relay.battery.estimate_path(len(plan.path))
        )
        distance = abs(scout.drone.position.x - self.world.base.x) + abs(
            scout.drone.position.y - self.world.base.y
        )
        expected_reconnect = max(1, distance - self.config.communication_range)
        critical_status = int(
            scout.drone.status is DroneStatus.RETURN_HOME
            or scout.energy_emergency
        )
        return evaluate_relay_utility(
            RelayUtilityInput(
                confirmed_survivors=len(survivors),
                critical_status_units=critical_status,
                recent_map_cells=recent,
                general_map_cells=general,
                queue_units=transport.queued_payload_units,
                route_hops=2,
                link_capacity_units=self.config.network_link_capacity_units,
                observed_delivery_ratio=(
                    transport.fragment_attempt_delivery_ratio
                ),
                ttl_remaining_steps=(
                    self.config.network_survivor_ttl
                    if survivors
                    else self.config.network_map_ttl
                ),
                relay_travel_steps=plan.movement_cost,
                energy_headroom=energy_headroom,
                expected_direct_reconnect_steps=expected_reconnect,
                maximum_hops=self.config.network_relay_max_hops,
                utility_threshold=self.config.network_relay_utility_threshold,
                maximum_low_priority_backlog_units=(
                    self.config.network_relay_max_backlog_units
                ),
            ),
            self._network_relay_weights,
        )

    def _record_network_relay_decision(
        self,
        runtime: DroneRuntime,
        decision: RelayUtilityDecision,
        scout_id: str,
    ) -> None:
        self._network_relay_evaluations += 1
        self._network_relay_utility_sum += decision.utility
        if decision.accepted:
            self._network_relay_accepted += 1
            self._network_relay_accepted_utility_sum += decision.utility
        else:
            self._network_relay_rejected += 1
            self._network_relay_rejection_reasons[decision.reason] = (
                self._network_relay_rejection_reasons.get(decision.reason, 0)
                + 1
            )
        signature = (
            runtime.drone.identifier,
            scout_id,
            decision.accepted,
            decision.reason,
            decision.backpressure_required,
        )
        if signature == self._network_relay_last_signature:
            return
        self._network_relay_last_signature = signature
        self._record_event(
            runtime,
            EventType.NETWORK_RELAY_EVALUATED,
            utility=decision.utility,
            reason=decision.reason,
            critical_backlog=decision.critical_payload_units,
            backpressure=decision.backpressure_required,
            route_hops=2,
        )
        self._record_event(
            runtime,
            (
                EventType.NETWORK_RELAY_ACCEPTED
                if decision.accepted
                else EventType.NETWORK_RELAY_REJECTED
            ),
            utility=decision.utility,
            reason=decision.reason,
            critical_backlog=decision.critical_payload_units,
            backpressure=decision.backpressure_required,
            route_hops=2,
        )

    def _assign_network_aware_relay(self) -> None:
        if not self.network_aware_relay_enabled or self._exploration_complete:
            return
        if self.relay_deployments >= self.config.relay_max_deployments:
            return
        if any(
            runtime.drone.status is DroneStatus.RELAY
            for runtime in self._ordered_runtimes()
        ):
            return
        if not self.communication_snapshot.has_link("drone-1", "drone-2"):
            return
        if any(
            connection.connected_to_base
            for connection in self.communication_snapshot.connections.values()
        ):
            return

        candidates: list[
            tuple[
                int,
                float,
                int,
                str,
                str,
                RelayPlan,
                set[Position],
                set[Position],
                RelayUtilityDecision,
            ]
        ] = []
        for relay_id in sorted(self.runtimes):
            scout_id = "drone-2" if relay_id == "drone-1" else "drone-1"
            relay = self.runtimes[relay_id]
            scout = self.runtimes[scout_id]
            if (
                relay.drone.status is not DroneStatus.EXPLORE
                or scout.drone.status is not DroneStatus.EXPLORE
                or self.steps < relay.relay_cooldown_until_step
                or self._current_outage_steps.get(scout_id, 0)
                < self.config.network_relay_min_outage_steps
            ):
                continue
            cells, survivors = self._relay_payload(scout)
            plan = self._relay_plan_for(relay, scout.drone.position)
            if plan is None:
                continue
            decision = self._network_relay_decision(
                relay, scout, plan, cells, survivors
            )
            self._record_network_relay_decision(relay, decision, scout_id)
            if not decision.accepted:
                continue
            candidates.append(
                (
                    -decision.critical_payload_units,
                    -decision.utility,
                    plan.movement_cost,
                    relay_id,
                    scout_id,
                    plan,
                    cells,
                    survivors,
                    decision,
                )
            )
        if not candidates:
            return
        (
            _, _, _, relay_id, scout_id, plan, cells, survivors, decision
        ) = min(candidates)
        relay = self.runtimes[relay_id]
        scout = self.runtimes[scout_id]
        records = dict(scout.local_map.records)
        selected_cells = sorted(
            cells,
            key=lambda position: (
                -records[position].observed_step,
                position.x,
                position.y,
            ),
        )[: self.config.network_relay_map_delta_limit]
        relay.drone.status = DroneStatus.RELAY
        if self.roles_enabled:
            self._transition_role(
                relay,
                AgentRole.RELAY,
                reason="relay_needed",
            )
        relay.active_frontier_target = None
        relay.relay_scout_id = scout_id
        relay.relay_scout_position = scout.drone.position
        relay.relay_started_step = self.steps
        relay.relay_payload_positions = set(selected_cells)
        relay.relay_payload_survivors = set(survivors)
        relay.relay_energy_at_start = relay.battery.remaining
        relay.relay_path_length_at_start = relay.drone.path_length
        relay.relay_outage_at_start = self._current_outage_steps.get(
            scout_id, 0
        )
        relay.network_relay_utility = decision.utility
        relay.network_relay_reason = decision.reason
        relay.network_relay_critical_backlog = decision.critical_payload_units
        relay.network_relay_expected_units = (
            len(selected_cells) + len(survivors)
        )
        relay.network_relay_forwarded_units = 0
        relay.network_relay_backpressure = decision.backpressure_required
        scout.holding_for_relay = True
        self.relay_deployments += 1
        self._record_event(relay, EventType.RELAY_ROLE_ASSIGNED)
        self._set_relay_plan(relay, plan)

    def _maintain_adaptive_relay(self) -> None:
        if not self.adaptive_relay_enabled:
            return
        for relay in self._ordered_runtimes():
            if relay.drone.status is not DroneStatus.RELAY:
                continue
            scout_id = relay.relay_scout_id
            if scout_id is None:
                self._finish_relay_role(relay, successful=False)
                continue
            scout = self.runtimes[scout_id]
            if scout.terminal or scout.drone.status is DroneStatus.RETURN_HOME:
                self._finish_relay_role(relay, successful=False)
                continue
            if self.communication_snapshot.has_link(
                relay.drone.identifier, scout_id
            ):
                relay.relay_scout_position = scout.drone.position
            if (
                self.communication_snapshot.connections[
                    scout_id
                ].direct_to_base
            ):
                self._finish_relay_role(relay, successful=False)
                continue
            role_steps = (
                self.steps - relay.relay_started_step
                if relay.relay_started_step is not None
                else 0
            )
            if role_steps >= self.config.relay_max_role_steps:
                self._finish_relay_role(relay, successful=False)
                continue
            return_path = self._known_return_path(
                relay, avoid_other_drones=False
            )
            required_return = (
                relay.battery.estimate_path(len(return_path))
                if return_path is not None
                else float("inf")
            )
            if (
                relay.battery.remaining + 1e-9
                < required_return
                + self.config.energy_safety_reserve
                + self.config.relay_energy_margin
            ):
                self._abort_relay_for_energy(relay)
                continue
            scout_position = relay.relay_scout_position
            if scout_position is None:
                self._finish_relay_role(relay, successful=False)
                continue
            plan = self._relay_plan_for(relay, scout_position)
            if plan is None:
                self._finish_relay_role(relay, successful=False)
                continue
            self._set_relay_plan(relay, plan)

    def _maintain_network_aware_relay(self) -> None:
        if not self.network_aware_relay_enabled:
            return
        for relay in self._ordered_runtimes():
            if relay.drone.status is not DroneStatus.RELAY:
                continue
            scout_id = relay.relay_scout_id
            if scout_id is None:
                self._finish_relay_role(relay, successful=False)
                continue
            scout = self.runtimes[scout_id]
            if scout.terminal or scout.drone.status is DroneStatus.RETURN_HOME:
                self._finish_relay_role(relay, successful=False)
                continue
            if self.communication_snapshot.has_link(
                relay.drone.identifier, scout_id
            ):
                relay.relay_scout_position = scout.drone.position
            if self.communication_snapshot.connections[scout_id].direct_to_base:
                if not relay.relay_payload_forwarded:
                    self._network_relay_unnecessary_deployments += 1
                self._finish_relay_role(relay, successful=False)
                continue
            role_steps = (
                self.steps - relay.relay_started_step
                if relay.relay_started_step is not None
                else 0
            )
            if role_steps >= self.config.relay_max_role_steps:
                self._finish_relay_role(relay, successful=False)
                continue
            return_path = self._known_return_path(
                relay, avoid_other_drones=False
            )
            required = (
                relay.battery.estimate_path(len(return_path))
                if return_path is not None
                else float("inf")
            )
            if relay.battery.remaining + 1e-9 < (
                required
                + self.config.energy_safety_reserve
                + self.config.relay_energy_margin
            ):
                self._abort_relay_for_energy(relay)
                continue
            scout_position = relay.relay_scout_position
            if scout_position is None:
                self._finish_relay_role(relay, successful=False)
                continue
            plan = self._relay_plan_for(relay, scout_position)
            if plan is None:
                self._finish_relay_role(relay, successful=False)
                continue
            previous_target = relay.relay_target
            self._set_relay_plan(relay, plan)
            if previous_target is not None and previous_target != plan.position:
                self._network_relay_route_replans += 1
                self._record_event(
                    relay,
                    EventType.RELAY_ROUTE_REPLANNED,
                    plan.position,
                    reason="communication_geometry_changed",
                    route_hops=2,
                )

    def _plan_relay_intention(self, runtime: DroneRuntime) -> Position:
        runtime.relay_role_steps += 1
        plan = runtime.relay_plan
        if plan is None or runtime.relay_target is None:
            return runtime.drone.position
        path = astar(
            runtime.drone.position,
            runtime.relay_target,
            runtime.local_map.is_known_free,
        )
        if path is None:
            self._finish_relay_role(runtime, successful=False)
            return runtime.drone.position
        runtime.planned_path = path
        if len(path) < 2:
            return runtime.drone.position
        return path[1]

    def _update_relay_after_sync(
        self,
        base_before: dict[Position, CellKnowledge],
        survivors_before: set[Position],
    ) -> None:
        if not (
            self.adaptive_relay_enabled or self.network_aware_relay_enabled
        ):
            return
        base_after = (
            dict(self.base_knowledge_map.records)
            if self.base_knowledge_map is not None
            else {}
        )
        new_base_cells = {
            position
            for position, record in base_after.items()
            if base_before.get(position) != record
        }
        new_base_survivors = (
            self._base_confirmed_survivors - survivors_before
        )
        for relay in self._ordered_runtimes():
            if relay.drone.status is not DroneStatus.RELAY:
                continue
            scout_id = relay.relay_scout_id
            if scout_id is None:
                continue
            chain_active = (
                self.communication_snapshot.has_link(
                    "base", relay.drone.identifier
                )
                and self.communication_snapshot.has_link(
                    relay.drone.identifier, scout_id
                )
            )
            if chain_active and not relay.relay_link_achieved:
                relay.relay_link_achieved = True
                self.relay_outages_shortened += 1
                self._record_event(relay, EventType.RELAY_LINK_ACHIEVED)
                if self.network_aware_relay_enabled:
                    started = relay.relay_started_step or self.steps
                    self._network_relay_first_hop_latencies.append(
                        self.steps - started
                    )
                    self.runtimes[scout_id].holding_for_relay = False
            if not chain_active:
                continue
            forwarded_cells = (
                new_base_cells & relay.relay_payload_positions
            )
            forwarded_survivors = (
                new_base_survivors & relay.relay_payload_survivors
            )
            if forwarded_cells or forwarded_survivors:
                self._relay_unique_cells_forwarded.update(forwarded_cells)
                self._relay_survivors_forwarded.update(
                    forwarded_survivors
                )
                relay.relay_payload_forwarded = True
                relay.network_relay_forwarded_units += (
                    len(forwarded_cells) + len(forwarded_survivors)
                )
                self._record_event(
                    relay,
                    EventType.RELAY_PAYLOAD_FORWARDED,
                    cell_count=len(forwarded_cells),
                    survivor_count=len(forwarded_survivors),
                    transfer_progress=(
                        relay.network_relay_forwarded_units
                        / max(1, relay.network_relay_expected_units)
                        if self.network_aware_relay_enabled
                        else None
                    ),
                )
                if self.network_aware_relay_enabled:
                    if forwarded_survivors:
                        self._network_relay_critical_acknowledged += len(
                            forwarded_survivors
                        )
                        self._record_event(
                            relay,
                            EventType.CRITICAL_PAYLOAD_ACKNOWLEDGED,
                            survivor_count=len(forwarded_survivors),
                            critical_backlog=max(
                                0,
                                len(relay.relay_payload_survivors)
                                - len(
                                    self._base_confirmed_survivors
                                    & relay.relay_payload_survivors
                                ),
                            ),
                        )
                    cells_complete = all(
                        base_after.get(position)
                        == dict(
                            self.runtimes[scout_id].local_map.records
                        ).get(position)
                        for position in relay.relay_payload_positions
                    )
                    survivors_complete = relay.relay_payload_survivors <= (
                        self._base_confirmed_survivors
                    )
                    transfer_complete = (
                        survivors_complete
                        if relay.relay_payload_survivors
                        else cells_complete
                    )
                    if transfer_complete:
                        started = relay.relay_started_step or self.steps
                        self._network_relay_end_to_end_latencies.append(
                            self.steps - started
                        )
                        self._finish_relay_role(relay, successful=True)
                else:
                    self._finish_relay_role(relay, successful=True)

    def _objectives_complete(self) -> bool:
        if self.knowledge_mode == "local":
            return False
        return len(self._confirmed_survivors) == len(self.world.survivors)

    def _all_survivors_confirmed(self) -> bool:
        confirmed = (
            self._base_confirmed_survivors
            if self.knowledge_mode == "local"
            else self._confirmed_survivors
        )
        if self.config.perception_noise != "off":
            return confirmed == set(self.world.survivors)
        return len(confirmed) == len(self.world.survivors)

    def _fail_energy(self, runtime: DroneRuntime) -> None:
        if runtime.drone.status is DroneStatus.RELAY:
            self._record_event(
                runtime, EventType.RELAY_ABORTED_FOR_ENERGY
            )
            self._finish_relay_role(runtime, successful=False)
        runtime.drone.status = DroneStatus.ENERGY_EMERGENCY
        runtime.energy_emergency = True
        runtime.active_frontier_target = None
        runtime.planned_path = ()
        runtime.current_return_path = ()
        self._record_event(runtime, EventType.ENERGY_EMERGENCY)

    def _fail_return_path(self, runtime: DroneRuntime) -> None:
        if runtime.drone.status is DroneStatus.RELAY:
            self._finish_relay_role(runtime, successful=False)
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

    def _complete_explore_dynamic_replan(
        self,
        runtime: DroneRuntime,
        assignment: FrontierAssignment | None,
    ) -> None:
        if (
            not runtime.dynamic_replan_pending
            or runtime.dynamic_replan_return
        ):
            return
        old_target = runtime.dynamic_replan_target
        old_length = runtime.dynamic_replan_old_path_length or 0
        if assignment is None:
            self._failed_replans += 1
            if old_target is not None:
                self._target_invalidations += 1
                self._record_event(
                    runtime,
                    EventType.TARGET_UNREACHABLE,
                    old_target,
                    reason="no_reachable_frontier_after_blockage",
                    old_path_length=old_length,
                )
            self._record_event(
                runtime,
                EventType.REPLAN_FAILED,
                old_target or runtime.drone.position,
                reason="no_reachable_frontier_after_blockage",
                old_path_length=old_length,
            )
        else:
            new_length = max(0, len(assignment.path) - 1)
            self._successful_replans += 1
            self._dynamic_replan_path_delta += new_length - old_length
            reason = "target_retained"
            if old_target is not None and assignment.target != old_target:
                reason = "target_reassigned"
                self._target_invalidations += 1
                self._target_reassignments += 1
                self._record_event(
                    runtime,
                    EventType.TARGET_UNREACHABLE,
                    old_target,
                    reason="blocked_route_or_target",
                    old_path_length=old_length,
                )
                self._record_event(
                    runtime,
                    EventType.TARGET_REASSIGNED,
                    assignment.target,
                    reason="replacement_frontier",
                    old_path_length=old_length,
                    new_path_length=new_length,
                )
            self._record_event(
                runtime,
                EventType.REPLAN_SUCCEEDED,
                assignment.target,
                reason=reason,
                old_path_length=old_length,
                new_path_length=new_length,
            )
        runtime.dynamic_replan_pending = False
        runtime.dynamic_replan_target = None
        runtime.dynamic_replan_old_path_length = None
        runtime.dynamic_replan_return = False

    def _allocate_frontiers(self) -> dict[str, FrontierAssignment]:
        self._prepare_failure_task_reassignments()
        if self.knowledge_mode == "local":
            return self._allocate_local_frontiers()
        explorers = {
            runtime.drone.identifier: runtime.drone.position
            for runtime in self._ordered_runtimes()
            if (
                runtime.drone.status is DroneStatus.EXPLORE
                and not runtime.holding_for_relay
            )
        }
        if not explorers:
            return {}
        frontiers = self.occupancy_map.frontiers()
        if (
            not frontiers
            and self.config.survivor_sensor == "visual"
            and self.config.smoke_profile == "off"
            and self.config.perception_noise == "off"
        ):
            # A first sighting is not yet a confirmed rescue observation.
            # Revisit detected positions before declaring exploration done.
            frontiers = tuple(
                sorted(self._detected_survivors - self._confirmed_survivors)
            )
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
            explorers,
            frontiers,
            self.occupancy_map,
            old_targets,
            blocked=self._observable_failed_positions(),
        )
        if self.roles_enabled and not assignments:
            failed_positions = self._observable_failed_positions()
            structurally_reachable = any(
                astar(
                    origin,
                    target,
                    lambda position: self.occupancy_map.is_known_free(position)
                    and (
                        position == origin
                        or position not in failed_positions
                    ),
                )
                is not None
                for origin in explorers.values()
                for target in frontiers
            )
            if not structurally_reachable:
                self._exploration_complete = True
                for runtime in self._ordered_runtimes():
                    if runtime.drone.status is not DroneStatus.EXPLORE:
                        continue
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
            self._complete_explore_dynamic_replan(runtime, assignment)
            if assignment is not None and assignment.target != old_target:
                event_type = (
                    EventType.FRONTIER_ASSIGNED
                    if old_target is None
                    else EventType.FRONTIER_REASSIGNED
                )
                runtime.frontier_assignments += 1
                self._record_event(runtime, event_type, assignment.target)
        self._confirm_failure_task_reassignments(assignments)
        return assignments

    def _allocate_local_frontiers(self) -> dict[str, FrontierAssignment]:
        explorers = {
            runtime.drone.identifier: runtime.drone.position
            for runtime in self._ordered_runtimes()
            if (
                runtime.drone.status is DroneStatus.EXPLORE
                and not runtime.holding_for_relay
            )
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
        peer_connected = any(len(component) > 1 for component in components)
        assignments: dict[str, FrontierAssignment] = {}
        changed_targets: set[str] = set()
        any_frontiers = False

        for component in components:
            # Synchronization has already made every radio-connected member's
            # store identical. Coordination therefore reads one actual local
            # map, never the global evaluation map or an implicit merged view.
            component_map = self.runtimes[min(component)].local_map
            frontiers = component_map.frontiers()
            if (
                not frontiers
                and self.config.survivor_sensor == "visual"
                and self.config.smoke_profile == "off"
            ):
                detected = {
                    position
                    for drone_id in component
                    for position in self.runtimes[drone_id].detected_survivors
                }
                confirmed = {
                    position
                    for drone_id in component
                    for position in self.runtimes[drone_id].confirmed_survivors
                }
                frontiers = tuple(sorted(detected - confirmed))
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
                blocked=frozenset(
                    position
                    for drone_id in component
                    for position in self._observable_failed_positions(
                        self.runtimes[drone_id]
                    )
                ),
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
                self._complete_explore_dynamic_replan(runtime, assignment)
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
        self._confirm_failure_task_reassignments(assignments)
        return assignments

    def _plan_return_intention(self, runtime: DroneRuntime) -> Position:
        previous_path = runtime.current_return_path
        previous_path_invalid = bool(previous_path) and not (
            self._return_path_is_usable(runtime, previous_path)
        )
        replan_required = previous_path_invalid or runtime.return_replan_required
        static_path = self._known_return_path(runtime, avoid_other_drones=False)
        if static_path is None:
            if runtime.dynamic_replan_pending:
                self._failed_replans += 1
                self._record_event(
                    runtime,
                    EventType.REPLAN_FAILED,
                    reason="return_path_unavailable",
                    old_path_length=runtime.dynamic_replan_old_path_length,
                )
                runtime.dynamic_replan_pending = False
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
        if runtime.dynamic_replan_pending:
            old_length = runtime.dynamic_replan_old_path_length or 0
            new_length = max(0, len(path) - 1)
            self._successful_replans += 1
            self._rtb_replans += 1
            self._dynamic_replan_path_delta += new_length - old_length
            self._record_event(
                runtime,
                EventType.REPLAN_SUCCEEDED,
                reason="return_route_replanned",
                old_path_length=old_length,
                new_path_length=new_length,
            )
            runtime.dynamic_replan_pending = False
            runtime.dynamic_replan_target = None
            runtime.dynamic_replan_old_path_length = None
            runtime.dynamic_replan_return = False
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
            # Other agents are transient blockers. The static-map fallback
            # prevents a dense launch formation from deadlocking while the
            # central safety shield still protects the next physical move.
            projected_return = self._known_return_path(
                runtime,
                origin=next_position,
                avoid_other_drones=False,
            )
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
            elif runtime.drone.status is DroneStatus.RELAY:
                destination = self._plan_relay_intention(runtime)
            elif runtime.yield_hold_until_step > self.steps:
                destination = runtime.drone.position
            elif runtime.holding_for_relay:
                self._relay_mission_delay_steps += 1
                if self.network_aware_relay_enabled:
                    self._network_relay_scout_wait_steps += 1
                destination = runtime.drone.position
            else:
                if self.network_aware_relay_enabled and any(
                    relay.drone.status is DroneStatus.RELAY
                    and relay.relay_scout_id == runtime.drone.identifier
                    for relay in self._ordered_runtimes()
                ):
                    self._network_relay_scout_exploration_steps += 1
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
        drone_ids = sorted(self.motion_intents)
        if self.network_transport is not None:
            for recipient in drone_ids:
                for sender in drone_ids:
                    if recipient == sender:
                        continue
                    intent = self._received_motion_intents[recipient].get(sender)
                    if intent is None or intent.valid_until_step < self.steps:
                        if intent is not None:
                            self._received_motion_intents[recipient].pop(sender, None)
                        return False
            return True
        return not drone_ids or set(drone_ids).issubset(
            self._communication_component(drone_ids[0])
        )

    def _deconflict_fleet_intentions(
        self,
        intentions: dict[str, Position],
    ) -> dict[str, Position]:
        """Resolve local N-agent conflicts with stable fleet-wide priority."""

        current = {
            drone_id: self.runtimes[drone_id].drone.position
            for drone_id in intentions
        }
        resolved, conflicts = resolve_movements(current, intentions, self.world.base)
        communicated = self._peer_intents_communicated()
        self._record_intent_sharing(communicated)
        if not conflicts:
            self._finish_yield_states(set(), set())
            return resolved

        waiters = {
            drone_id
            for drone_id in intentions
            if intentions[drone_id] != current[drone_id]
            and resolved[drone_id] == current[drone_id]
        }
        self.local_motion_conflicts += len(conflicts)
        if communicated:
            self.communication_detected_conflicts += len(conflicts)
        else:
            self.proximity_detected_conflicts += len(conflicts)
        if waiters:
            self.deconfliction_delay_steps += 1
        for conflict in conflicts:
            for drone_id in sorted(set(conflict.drone_ids).intersection(waiters)):
                self._record_event(
                    self.runtimes[drone_id],
                    EventType.LOCAL_COLLISION_AVOIDED,
                    conflict.position,
                )
        self._finish_yield_states(waiters, waiters)
        return resolved

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
        clearance_from: Position | None = None,
    ) -> Position | None:
        decision_map = runtime.local_map
        target = (
            self.world.base
            if runtime.drone.status is DroneStatus.RETURN_HOME
            else (
                runtime.relay_target
                if runtime.drone.status is DroneStatus.RELAY
                else runtime.active_frontier_target
            )
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
            clearance_penalty = 0
            if clearance_from is not None:
                peer_dx = clearance_from.x - runtime.drone.position.x
                peer_dy = clearance_from.y - runtime.drone.position.y
                move_dx = candidate.x - runtime.drone.position.x
                move_dy = candidate.y - runtime.drone.position.y
                clearance_penalty = int(
                    peer_dx * move_dx + peer_dy * move_dy != 0
                )
            candidates.append(
                (
                    clearance_penalty,
                    -free_neighbors,
                    path_length,
                    candidate.y,
                    candidate.x,
                    candidate,
                )
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
        if len(self.motion_intents) != 2:
            return self._deconflict_fleet_intentions(intentions)
        first_id, second_id = sorted(self.motion_intents)
        drone_ids = (first_id, second_id)
        first = self.motion_intents[first_id]
        second = self.motion_intents[second_id]
        communicated = self._peer_intents_communicated()
        self._record_intent_sharing(communicated)
        conflict_first = first
        conflict_second = second
        if communicated and self.network_transport is not None:
            conflict_first = self._received_motion_intents[second_id][first_id]
            conflict_second = self._received_motion_intents[first_id][second_id]
        priority_intents = {
            first_id: conflict_first,
            second_id: conflict_second,
        }

        conflict: IntentConflict | None = None
        source: str | None = None
        proximity_risks: dict[str, str] = {}
        # Proximity is a current, local observation and therefore takes
        # precedence over delayed-but-still-valid network intent.
        communicated_positions_current = (
            communicated
            and conflict_first.current_position == first.current_position
            and conflict_second.current_position == second.current_position
        )
        communicated_intents_current = (
            communicated_positions_current
            and conflict_first.next_position == first.next_position
            and conflict_second.next_position == second.next_position
        )
        visible = self.proximity_sensor.can_detect(
            self.world,
            first.current_position,
            second.current_position,
        )
        if self._legacy_network_proximity_precedence and communicated:
            visible = False
        if visible:
            for intent, other_position in (
                (first, second.current_position),
                (second, first.current_position),
            ):
                risk = proximity_risk(intent, other_position)
                if risk is not None:
                    proximity_risks[intent.drone_id] = risk
        immediate_occupied_risk = any(
            risk == "proximity_occupied"
            for risk in proximity_risks.values()
        )
        if proximity_risks and (
            immediate_occupied_risk or not communicated_positions_current
        ):
            source = "proximity"
            priority_intents = {
                first_id: first,
                second_id: second,
            }
            if self.network_transport is not None and not communicated:
                self.proximity_avoidances_without_fresh_intent += 1
            kind = sorted(proximity_risks.values())[0]
            position = min(
                self.motion_intents[drone_id].next_position
                for drone_id in proximity_risks
            )
            conflict = IntentConflict(kind, position, drone_ids)
        elif (
            self.network_aware_transport_enabled
            and visible
            and communicated
            and not communicated_intents_current
        ):
            # A close peer plus stale network intent is treated as an
            # uncertain local reservation.  The deterministic yield happens
            # before the central fail-safe and consumes no global map data.
            source = "proximity"
            priority_intents = {first_id: first, second_id: second}
            conflict = IntentConflict(
                "proximity_uncertain",
                min(first.next_position, second.next_position),
                drone_ids,
            )
        elif communicated:
            conflict = detect_intent_conflict(
                conflict_first, conflict_second, self.world.base
            )
            if conflict is not None:
                source = "communication"

        if conflict is None or source is None:
            self._active_deconfliction_signature = None
            self._deconfliction_repeat_count = 0
            self._deadlock_reported_for_signature = False
            self._finish_yield_states(set(), set())
            return intentions

        self.local_motion_conflicts += 1
        if (
            source == "communication"
            and conflict.kind == "occupied_current_cell"
        ):
            self.communication_detected_conflicts += 1
            if conflict_first.next_position == conflict_second.current_position:
                loser_id, winner_id = first_id, second_id
            else:
                loser_id, winner_id = second_id, first_id
        elif source == "communication":
            self.communication_detected_conflicts += 1
            winner_id = min(
                drone_ids,
                key=lambda drone_id: priority_key(
                    priority_intents[drone_id],
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
                        priority_intents[drone_id],
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

        # When an urgent RTB drone has priority, a non-urgent nearby peer
        # actively clears the reserved corridor instead of repeatedly waiting
        # in the cell that forces the RTB planner to oscillate around it.
        winner_intent = priority_intents[winner_id]
        loser_intent = priority_intents[loser_id]
        priority_clearance = (
            source == "proximity"
            and not self._legacy_network_proximity_precedence
            and winner_intent.status is DroneStatus.RETURN_HOME
            and loser_intent.status is not DroneStatus.RETURN_HOME
            and not blocking_conflict
        )
        if priority_clearance:
            forbidden = {
                winner_intent.current_position,
                winner_intent.next_position,
                *winner_intent.reservation,
            }
            loser_runtime = self.runtimes[loser_id]
            avoidance = self._avoidance_step(
                loser_runtime,
                self.motion_intents[loser_id].next_position,
                forbidden,
                clearance_from=winner_intent.current_position,
            )
            if avoidance is not None:
                resolved[loser_id] = avoidance
                delayed_now.discard(loser_id)
                loser_runtime.yield_hold_until_step = max(
                    loser_runtime.yield_hold_until_step,
                    self.steps + self.config.intent_reservation_steps + 1,
                )
                self.local_replans_due_to_drones += 1
                self._record_event(
                    loser_runtime,
                    EventType.DEADLOCK_REPLANNED,
                    avoidance,
                )

        if self._deconfliction_repeat_count >= self.config.deadlock_wait_threshold:
            if not self._deadlock_reported_for_signature:
                self.corridor_deadlocks += 1
                self._deadlock_reported_for_signature = True
                self._record_event(
                    self.runtimes[loser_id],
                    EventType.CORRIDOR_DEADLOCK_DETECTED,
                    conflict.position,
                )
            other_intent = priority_intents[winner_id]
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
        failed_positions = self._observable_failed_positions()
        for drone_id, destination in tuple(intentions.items()):
            runtime = self.runtimes[drone_id]
            if (
                destination != runtime.drone.position
                and destination in failed_positions
            ):
                intentions[drone_id] = runtime.drone.position
                if runtime.drone.status is DroneStatus.RETURN_HOME:
                    runtime.return_replan_required = True
                self._failed_drone_collision_avoidances += 1
                self._record_event(
                    runtime,
                    EventType.FAILED_DRONE_COLLISION_AVOIDED,
                    destination,
                )
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
        if conflict_waiters and self.network_transport is not None:
            current_events = tuple(
                event
                for event in self.mission_log.events
                if event.step == self.steps
            )
            for drone_id in sorted(conflict_waiters):
                conflict = next(
                    item for item in conflicts if drone_id in item.drone_ids
                )
                geometry = {
                    "vertex": "vertex_conflict",
                    "edge_swap": "edge_swap",
                    "occupied_current_cell": "corridor_encounter",
                }.get(conflict.reason, conflict.reason)
                self._network_shield_geometry_classification[geometry] = (
                    self._network_shield_geometry_classification.get(geometry, 0)
                    + 1
                )
                peer_id = next(
                    candidate
                    for candidate in conflict.drone_ids
                    if candidate != drone_id
                )
                received = self._received_motion_intents[drone_id].get(peer_id)
                if any(
                    event.event_type is EventType.MESSAGE_EXPIRED
                    and event.message_type == MessageType.MOTION_INTENT.value
                    for event in current_events
                ):
                    cause = "expired_intent"
                elif any(
                    event.event_type is EventType.MESSAGE_LOST
                    and event.message_type == MessageType.MOTION_INTENT.value
                    for event in current_events
                ):
                    cause = "lost_intent"
                elif received is not None and (
                    received.current_position
                    != self.runtimes[peer_id].drone.position
                    or received.valid_until_step - self.config.motion_intent_ttl
                    < self.steps
                ):
                    cause = "delayed_intent"
                elif any(
                    fragment.message_type is MessageType.MOTION_INTENT
                    and fragment.recipient == drone_id
                    for fragment in (
                        self.network_transport._queued
                        + self.network_transport._in_flight
                    )
                ):
                    cause = "queue_displacement"
                elif (
                    self.proximity_sensor.can_detect(
                        self.world,
                        self.runtimes[drone_id].drone.position,
                        self.runtimes[peer_id].drone.position,
                    )
                    and not self.communication_snapshot.has_link(
                        drone_id, peer_id
                    )
                ):
                    cause = "contact_without_radio"
                else:
                    cause = "lost_intent"
                self._network_shield_cause_classification[cause] = (
                    self._network_shield_cause_classification.get(cause, 0) + 1
                )
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
                if self.config.uncertainty_profile != "off":
                    # Privileged execution veto only: no truth injected into mapping.
                    self.safety_shield_interventions += 1
                    self._stale_path_safety_interventions += 1
                    waiting.add(drone_id)
                    continue
                if destination in self.world.dynamic_obstacles:
                    self.safety_shield_interventions += 1
                    self._stale_path_safety_interventions += 1
                    self._record_event(
                        runtime,
                        EventType.STALE_PATH_SAFETY_INTERVENTION,
                        destination,
                        reason="dynamic_obstacle_on_stale_path",
                        old_cell_state=CellState.FREE.value,
                        new_cell_state=CellState.OCCUPIED.value,
                    )
                    contact_observation = {
                        destination: CellState.OCCUPIED
                    }
                    if self.knowledge_sync_enabled:
                        runtime.local_map.observe(
                            contact_observation,
                            step=self.steps,
                            source_id=runtime.drone.identifier,
                        )
                    if self.knowledge_mode != "local":
                        if isinstance(self.occupancy_map, OccupancyMap):
                            self.occupancy_map.update(contact_observation)
                    self._register_dynamic_observations(
                        runtime,
                        contact_observation,
                        {destination: CellState.FREE},
                    )
                    waiting.add(drone_id)
                    continue
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
            if runtime.drone.status is DroneStatus.RELAY:
                runtime.relay_path_length += 1
            if runtime.drone.status is DroneStatus.RETURN_HOME:
                runtime.return_path_length += 1
                if runtime.current_return_path:
                    runtime.current_return_path = runtime.current_return_path[1:]
            if destination != self.world.base:
                self._visited_by_cell.setdefault(destination, set()).add(drone_id)

        self.steps += 1
        self._record_recovery_movements(movers)
        if self.dynamic_obstacles_enabled:
            self._inject_dynamic_obstacles()
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
        all_landed = all(
            runtime.drone.status is DroneStatus.LANDED
            for runtime in self._ordered_runtimes()
        )
        operational_landed = all(
            runtime.drone.status is DroneStatus.LANDED
            for runtime in self._ordered_runtimes()
            if runtime.drone.identifier not in self._injected_failure_ids
        )
        if (
            operational_landed
            and self.network_transport is not None
            and self._pending_critical_survivors()
        ):
            if not self._final_sync_active:
                self._start_final_sync()
            return
        self.completed = True
        if self._injected_failure_ids and operational_landed:
            self.termination_reason = (
                "failure_recovered"
                if self._all_survivors_confirmed()
                else "mission_failed"
            )
        elif all_landed:
            self.termination_reason = (
                "exploration_complete"
                if self._exploration_complete
                else "returned_to_base"
            )
        else:
            self.termination_reason = "mission_failed"

    def _is_completed(self) -> bool:
        """Return completion without retaining a stale type narrowing."""

        return self.completed

    def step(self) -> bool:
        self._advance_probability_maps()
        if self._is_completed():
            return False
        if self._final_sync_active:
            return self._step_final_sync()
        if self.steps >= self.config.max_steps:
            for runtime in self._ordered_runtimes():
                if not runtime.terminal:
                    if runtime.drone.status is DroneStatus.RELAY:
                        self._finish_relay_role(
                            runtime, successful=False
                        )
                    runtime.drone.status = DroneStatus.FAILED
                    runtime.active_frontier_target = None
                    runtime.planned_path = ()
                    runtime.current_return_path = ()
            self.completed = True
            self.termination_reason = "max_steps"
            self._synchronize_role_tasks()
            self._finalize_network_transport()
            return False

        if self.network_transport is not None:
            self._deliver_network_transport()
        self._prepare_energy_states()
        if self.multi_relay_enabled:
            self._maintain_multi_relay()
        elif self.network_aware_relay_enabled:
            self._maintain_network_aware_relay()
        else:
            self._maintain_adaptive_relay()
        assignments = self._allocate_frontiers()
        if self.multi_relay_enabled:
            self._assign_multi_relay()
        elif self.network_aware_relay_enabled:
            self._assign_network_aware_relay()
        else:
            self._assign_adaptive_relay()
        self._synchronize_role_tasks()
        intentions = self._plan_intentions(assignments)
        intentions = self._deconflict_intentions(intentions)
        if self.network_transport is not None:
            self._queue_motion_messages()
            self._transmit_network_transport()
        previous_step = self.steps
        self._execute_intentions(intentions)
        if self.steps != previous_step:
            self._inject_scheduled_failures()
            self._synchronize_role_tasks()
            base_before = (
                dict(self.base_knowledge_map.records)
                if self.base_knowledge_map is not None
                else {}
            )
            survivors_before = set(self._base_confirmed_survivors)
            self._sample_communication()
            if self.network_transport is None:
                self._sync_shadow_maps()
                self._sync_survivor_knowledge()
                self._update_relay_after_sync(
                    base_before, survivors_before
                )
                self._acknowledge_base_uploads()
            else:
                self._queue_network_knowledge()
                self._record_network_events()
            self._record_base_coverage()
        self._update_completion()
        if self._is_completed():
            self._finalize_network_transport()
        return not self._is_completed()

    def run(self, on_frame: FrameCallback | None = None) -> MultiSimulationResult:
        if on_frame is not None:
            on_frame(self)
        while self.step():
            if on_frame is not None:
                on_frame(self)
        if on_frame is not None:
            on_frame(self)
        return self.result()

    def _multi_relay_metrics(self) -> dict[str, object] | None:
        if not self.multi_relay_enabled:
            return None
        transport = self.network_transport
        active_relay_steps = sum(self._multi_relay_active_samples)
        relay_energy = self._relay_energy_consumed + sum(
            max(0.0, runtime.relay_energy_at_start - runtime.battery.remaining)
            for runtime in self._ordered_runtimes()
            if runtime.relay_energy_at_start is not None
        )
        communication_uptimes = [
            self._communication_connected_samples.get(drone_id, 0)
            / max(1, self._communication_samples)
            for drone_id in sorted(self.runtimes)
        ]
        served_counts = {
            relay_id: len(served)
            for relay_id, served in sorted(
                self._multi_relay_served_totals.items()
            )
        }
        return {
            "strategy": self.config.relay_strategy,
            "architecture_version": "1.0",
            "candidate_score_version": "1.0",
            "maximum_active_relays": self.config.multi_relay_max_active,
            "prediction_horizon": self.config.relay_prediction_horizon,
            "relay_activations": self._multi_relay_activations,
            "relay_deactivations": self._multi_relay_deactivations,
            "active_relay_steps": active_relay_steps,
            "mean_active_relays": (
                sum(self._multi_relay_active_samples)
                / len(self._multi_relay_active_samples)
                if self._multi_relay_active_samples
                else 0.0
            ),
            "max_active_relays": max(
                self._multi_relay_active_samples, default=0
            ),
            "relay_role_changes": self._multi_relay_role_changes,
            "relay_role_changes_by_agent": {
                drone_id: self._role_changes_by_agent.get(drone_id, 0)
                for drone_id in sorted(self.runtimes)
            },
            "relay_thrashing_detected": any(
                count > 2
                for count in self._multi_relay_activation_counts.values()
            ),
            "relay_activation_counts": dict(
                sorted(self._multi_relay_activation_counts.items())
            ),
            "relay_distance_travelled": sum(
                runtime.relay_path_length
                for runtime in self._ordered_runtimes()
            ),
            "relay_energy_consumed": relay_energy,
            "exploration_opportunity_cost_steps": active_relay_steps,
            "scout_hold_steps": sum(
                runtime.wait_steps
                for runtime in self._ordered_runtimes()
                if runtime.holding_for_relay
            ),
            "agents_served_per_relay": served_counts,
            "mean_agents_served_per_relay": (
                sum(served_counts.values()) / len(served_counts)
                if served_counts
                else 0.0
            ),
            "concurrent_relay_links_mean": (
                sum(self._multi_relay_concurrent_link_samples)
                / len(self._multi_relay_concurrent_link_samples)
                if self._multi_relay_concurrent_link_samples
                else 0.0
            ),
            "concurrent_relay_links_max": max(
                self._multi_relay_concurrent_link_samples, default=0
            ),
            "communication_uptime": (
                sum(communication_uptimes) / len(communication_uptimes)
                if communication_uptimes
                else 0.0
            ),
            "base_connectivity_uptime": (
                sum(communication_uptimes) / len(communication_uptimes)
                if communication_uptimes
                else 0.0
            ),
            "disconnected_agent_steps": (
                self._multi_relay_disconnected_agent_steps
            ),
            "mean_hop_count": (
                sum(self._multi_relay_hop_samples)
                / len(self._multi_relay_hop_samples)
                if self._multi_relay_hop_samples
                else None
            ),
            "max_hop_count": max(self._multi_relay_hop_samples, default=0),
            "connectivity_gain_steps": (
                self._multi_relay_connected_via_relay_steps
            ),
            "relay_efficiency": (
                self._multi_relay_connected_via_relay_steps
                / active_relay_steps
                if active_relay_steps
                else 0.0
            ),
            "queue_size_mean": (
                transport.average_queue_size
                if transport is not None
                else 0.0
            ),
            "queue_size_max": (
                transport.maximum_queue_size if transport is not None else 0
            ),
            "messages_sent": (
                transport.sent_fragments if transport is not None else 0
            ),
            "messages_delivered": (
                transport.delivered_fragments if transport is not None else 0
            ),
            "messages_expired": (
                transport.expired_fragments if transport is not None else 0
            ),
            "ttl_expirations": (
                transport.expired_fragments if transport is not None else 0
            ),
            "map_sync_mean_latency": (
                sum(self._network_map_latencies)
                / len(self._network_map_latencies)
                if self._network_map_latencies
                else None
            ),
            "map_sync_max_latency": max(
                self._network_map_latencies, default=0
            ),
            "final_sync_steps": self._final_sync_duration,
            "final_sync_timeout": self._final_sync_timeout,
            "unsynced_critical_events": (
                self._multi_relay_unsynced_critical_samples
            ),
            "predictive_forecasts": self._multi_relay_predictive_forecasts,
            "predictive_activations": self._multi_relay_predictive_activations,
            "prevented_disconnection_steps": (
                self._multi_relay_prevented_disconnect_steps
            ),
            "unnecessary_predictive_activations": (
                self._multi_relay_unnecessary_predictive_activations
            ),
            "relay_failure_recoveries": self._multi_relay_failure_recoveries,
            "relay_replans": self._multi_relay_replans,
        }

    def _network_aware_relay_metrics(self) -> dict[str, object] | None:
        if self.config.relay_strategy != "network-aware":
            return None
        transport = self.network_transport
        assert transport is not None

        def mean(values: list[int]) -> float | None:
            return sum(values) / len(values) if values else None

        accepted = self._network_relay_accepted
        evaluations = self._network_relay_evaluations
        return {
            "utility_model_version": "1.0",
            "utility_weights": self._network_relay_weights.to_dict(),
            "utility_threshold": self.config.network_relay_utility_threshold,
            "maximum_route_hops": self.config.network_relay_max_hops,
            "maximum_low_priority_backlog_units": (
                self.config.network_relay_max_backlog_units
            ),
            "evaluations": evaluations,
            "accepted_decisions": accepted,
            "rejected_decisions": self._network_relay_rejected,
            "acceptance_ratio": accepted / evaluations if evaluations else None,
            "average_utility": (
                self._network_relay_utility_sum / evaluations
                if evaluations
                else None
            ),
            "average_accepted_utility": (
                self._network_relay_accepted_utility_sum / accepted
                if accepted
                else None
            ),
            "rejection_reasons": dict(
                sorted(self._network_relay_rejection_reasons.items())
            ),
            "critical_payloads_relayed": (
                self._network_relay_critical_acknowledged
            ),
            "compacted_or_superseded_map_items": (
                self._network_relay_compacted_items
            ),
            "backpressure_steps": self._network_relay_backpressure_steps,
            "average_relay_backlog_units": (
                sum(self._network_relay_backlog_samples)
                / len(self._network_relay_backlog_samples)
                if self._network_relay_backlog_samples
                else 0.0
            ),
            "maximum_relay_backlog_units": max(
                self._network_relay_backlog_samples, default=0
            ),
            "first_hop_mean_latency": mean(
                transport.critical_first_hop_latencies
            ),
            "critical_first_hop_establishment_mean_latency": mean(
                transport.critical_first_hop_latencies
            ),
            "second_hop_mean_latency": mean(
                transport.critical_second_hop_latencies
            ),
            "critical_second_hop_transport_mean_latency": mean(
                transport.critical_second_hop_latencies
            ),
            "end_to_end_critical_mean_latency": mean(
                transport.critical_end_to_end_latencies
            ),
            "route_replans": self._network_relay_route_replans,
            "unnecessary_deployments": (
                self._network_relay_unnecessary_deployments
            ),
            "scout_wait_steps": self._network_relay_scout_wait_steps,
            "scout_exploration_steps_during_relay": (
                self._network_relay_scout_exploration_steps
            ),
            "ttl_losses": transport.expired_fragments,
            "queue_backlog_by_type": transport.backlog_by_type(),
        }

    def smoke_detection_metrics(self) -> dict[str, object]:
        """Return aggregate perception telemetry without target locations."""

        drone_ids = sorted(self.runtimes)
        payload: dict[str, object] = {
            "profile": self.config.smoke_profile,
            "smoke_cells": len(self.world.smoke.cells),
            "maximum_density": round(self.world.smoke.maximum_density, 6),
            "exposure_samples_by_drone": {
                drone_id: self._smoke_exposure_samples_by_drone.get(
                    drone_id, 0
                )
                for drone_id in drone_ids
            },
            "entries_by_drone": {
                drone_id: self._smoke_entries_by_drone.get(drone_id, 0)
                for drone_id in drone_ids
            },
            "detection_attempts": self._survivor_detection_attempts,
            "successful_observations": self._survivor_detection_successes,
            "degraded_detection_attempts": (
                self._smoke_degraded_detection_attempts
            ),
            "degraded_detection_events": self._smoke_degraded_detection_events,
            "successful_smoke_observations": (
                self._smoke_successful_detection_attempts
            ),
        }
        if self.config.survivor_sensor == "thermal":
            payload.update(
                {
                    "sensor_channel": "thermal",
                    "failed_observations": self._survivor_detection_failures,
                    "thermal_attempts": self._survivor_detection_attempts,
                    "thermal_successful_observations": (
                        self._survivor_detection_successes
                    ),
                    "thermal_failed_observations": (
                        self._survivor_detection_failures
                    ),
                }
            )
        return payload

    def _smoke_metrics(self) -> dict[str, object] | None:
        if (
            self.config.smoke_profile == "off"
            and self.config.survivor_sensor == "visual"
        ):
            return None
        return self.smoke_detection_metrics()

    def _perception_metrics(self) -> dict[str, object] | None:
        if self.config.perception_noise == "off":
            return None
        hypotheses = tuple(self.hypothesis_tracker.hypotheses.values())
        if self.config.uncertainty_profile != "off" and self.knowledge_mode == "local":
            by_location: dict[Position, SurvivorHypothesis] = {}
            for tracker in self.local_hypothesis_trackers.values():
                for location, hypothesis in tracker.hypotheses.items():
                    current = by_location.get(location)
                    if current is None or hypothesis.status is HypothesisStatus.CONFIRMED:
                        by_location[location] = hypothesis
            hypotheses = tuple(by_location.values())
        confirmed = tuple(
            item for item in hypotheses if item.status is HypothesisStatus.CONFIRMED
        )
        rejected = tuple(
            item for item in hypotheses if item.status is HypothesisStatus.REJECTED
        )
        true_confirmed = sum(
            item.location in self.world.survivors for item in confirmed
        )
        false_confirmed = len(confirmed) - true_confirmed
        confidence_values = [
            item.confirmation_confidence
            for item in confirmed
            if item.confirmation_confidence is not None
        ]
        bucket_edges = tuple(index / 5 for index in range(6))
        calibration = []
        for lower, upper in zip(bucket_edges, bucket_edges[1:]):
            samples = [
                (confidence, correct)
                for confidence, correct in self._perception_calibration
                if lower <= confidence < upper
                or (upper == 1.0 and confidence == 1.0)
            ]
            calibration.append(
                {
                    "range": [lower, upper],
                    "count": len(samples),
                    "mean_confidence": (
                        round(sum(value for value, _ in samples) / len(samples), 6)
                        if samples
                        else None
                    ),
                    "empirical_true_positive_rate": (
                        round(sum(correct for _, correct in samples) / len(samples), 6)
                        if samples
                        else None
                    ),
                }
            )
        first_true_steps = [
            event.step
            for event in self.mission_log.events
            if event.event_type is EventType.SURVIVOR_HYPOTHESIS_CONFIRMED
            and event.position in self.world.survivors
        ]
        positive_denominator = self._perception_tp + self._perception_fn
        negative_denominator = self._perception_fp + self._perception_tn
        return {
            "profile": self.config.perception_noise,
            "perception_attempts": positive_denominator + negative_denominator,
            "true_positive_observations": self._perception_tp,
            "false_positive_observations": self._perception_fp,
            "true_negative_observations": self._perception_tn,
            "false_negative_observations": self._perception_fn,
            "false_positive_rate": round(
                self._perception_fp / negative_denominator, 6
            ) if negative_denominator else 0.0,
            "false_negative_rate": round(
                self._perception_fn / positive_denominator, 6
            ) if positive_denominator else 0.0,
            "hypotheses_created": len(hypotheses),
            "hypotheses_confirmed": len(confirmed),
            "hypotheses_rejected": len(rejected),
            "false_survivor_confirmations": false_confirmed,
            "true_survivor_confirmations": true_confirmed,
            "mean_confirmation_confidence": (
                round(sum(confidence_values) / len(confidence_values), 6)
                if confidence_values
                else None
            ),
            "mean_observations_per_confirmation": (
                round(
                    sum(item.observation_count for item in confirmed)
                    / len(confirmed),
                    6,
                )
                if confirmed
                else None
            ),
            "time_to_first_true_survivor": (
                min(first_true_steps) if first_true_steps else None
            ),
            "calibration": calibration,
        }

    def _dynamic_obstacle_metrics(self) -> dict[str, object] | None:
        if not self.dynamic_obstacles_enabled:
            return None
        return {
            "profile": (
                "explicit"
                if self.config.dynamic_obstacle_schedule
                else self.config.dynamic_obstacles
            ),
            "scheduled": len(self.dynamic_obstacle_events),
            "dynamic_obstacles_injected": len(
                self._dynamic_obstacles_injected
            ),
            "dynamic_obstacles_observed": len(
                self._dynamic_obstacles_observed
            ),
            "path_invalidations": self._path_invalidations,
            "replans_total": self._replans_total,
            "successful_replans": self._successful_replans,
            "failed_replans": self._failed_replans,
            "target_invalidations": self._target_invalidations,
            "target_reassignments": self._target_reassignments,
            "rtb_replans": self._rtb_replans,
            "stale_path_safety_interventions": (
                self._stale_path_safety_interventions
            ),
            "additional_path_length_due_to_replanning": (
                self._dynamic_replan_path_delta
            ),
            "additional_mission_duration_due_to_obstacles": None,
        }

    def _failure_recovery_metrics(self) -> dict[str, object] | None:
        if not self.config.failure_schedule:
            return None
        operational = tuple(
            runtime
            for runtime in self._ordered_runtimes()
            if runtime.drone.identifier not in self._injected_failure_ids
        )
        operational_returned = sum(
            runtime.drone.status is DroneStatus.LANDED
            for runtime in operational
        )
        unexpected_failures = tuple(
            runtime.drone.identifier
            for runtime in operational
            if runtime.drone.status in {
                DroneStatus.ENERGY_EMERGENCY,
                DroneStatus.RETURN_PATH_UNAVAILABLE,
                DroneStatus.FAILED,
            }
        )
        recovery_success = (
            bool(self._injected_failure_ids)
            and operational_returned == len(operational)
            and not unexpected_failures
            and self._all_survivors_confirmed()
            and self.collisions == 0
            and self.drone_drone_collisions == 0
        )
        return {
            "scheduled": [
                {"drone_id": drone_id, "step": step}
                for drone_id, step in self.config.failure_schedule
            ],
            "injected_drone_ids": sorted(self._injected_failure_ids),
            "failures_triggered": len(self._injected_failure_ids),
            "tasks_released": self._failure_tasks_released,
            "tasks_reassigned": self._failure_tasks_reassigned,
            "tasks_pending": len(self._released_failure_tasks),
            "failed_drone_collision_avoidances": (
                self._failed_drone_collision_avoidances
            ),
            "operational_drones": len(operational),
            "operational_drones_returned": operational_returned,
            "unexpected_failure_ids": list(unexpected_failures),
            "recovery_success": recovery_success,
        }

    def _role_failure_metrics(
        self,
        *,
        mission_success: bool,
        survivor_recall: float,
    ) -> dict[str, object] | None:
        if not self.roles_enabled:
            return None
        orphaned = tuple(
            task
            for task in self.task_registry.tasks.values()
            if task.orphaned_step is not None
        )
        reassigned = tuple(task for task in orphaned if task.reassigned)
        failed_reassignments = len(orphaned) - len(reassigned)
        operational = tuple(
            runtime
            for runtime in self._ordered_runtimes()
            if runtime.drone.identifier not in self._injected_failure_ids
        )

        def average(values: list[int]) -> float | None:
            return sum(values) / len(values) if values else None

        return {
            "role_policy": self.config.role_policy,
            "initial_roles": {
                runtime.drone.identifier: runtime.base_role.value
                for runtime in self._ordered_runtimes()
            },
            "final_roles": {
                runtime.drone.identifier: runtime.role.value
                for runtime in self._ordered_runtimes()
            },
            "failures_injected": len(self._injected_failure_ids),
            "failed_agents": len(self._injected_failure_ids),
            "failed_agent_ids": sorted(self._injected_failure_ids),
            "tasks_orphaned": len(orphaned),
            "tasks_reassigned": len(reassigned),
            "successful_reassignments": len(reassigned),
            "failed_reassignments": failed_reassignments,
            "reassignment_success_rate": (
                len(reassigned) / len(orphaned) if orphaned else None
            ),
            "mean_reassignment_latency": average(
                self._reassignment_latencies
            ),
            "max_reassignment_latency": (
                max(self._reassignment_latencies)
                if self._reassignment_latencies
                else None
            ),
            "mean_recovery_latency": average(self._recovery_latencies),
            "max_recovery_latency": (
                max(self._recovery_latencies)
                if self._recovery_latencies
                else None
            ),
            "role_changes": self._role_changes,
            "role_changes_per_agent": dict(
                sorted(self._role_changes_by_agent.items())
            ),
            "role_thrashing_detected": any(
                changes > 2
                for changes in self._role_changes_by_agent.values()
            ),
            "emergency_role_takeovers": self._emergency_role_takeovers,
            "tasks_completed_by_reassigned_agent": (
                self._tasks_completed_by_reassigned_agent
            ),
            "mission_success_after_failure": (
                mission_success if self._injected_failure_ids else None
            ),
            "survivor_recall_after_failure": (
                survivor_recall if self._injected_failure_ids else None
            ),
            "operational_agents": len(operational),
            "operational_agents_returned": sum(
                runtime.drone.status is DroneStatus.LANDED
                for runtime in operational
            ),
        }

    def result(self) -> MultiSimulationResult:
        self._finalize_network_transport()
        transport = self.network_transport
        detection_steps = [
            event.step
            for event in self.mission_log.events
            if event.event_type is EventType.SURVIVOR_DETECTED
        ]
        base_confirmation_steps = [
            event.step
            for event in self.mission_log.events
            if (
                event.event_type
                is EventType.SURVIVOR_KNOWLEDGE_SYNCHRONIZED
                and event.drone_id == "base"
            )
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
        true_reported_confirmed = reported_confirmed & self.world.survivors
        survivor_recall = (
            len(true_reported_confirmed) / survivors_total
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
        failure_recovery_metrics = self._failure_recovery_metrics()
        mission_success = (
            bool(failure_recovery_metrics["recovery_success"])
            if self._injected_failure_ids
            and failure_recovery_metrics is not None
            else (
                (
                    reported_confirmed == set(self.world.survivors)
                    if self.config.perception_noise != "off"
                    else len(reported_confirmed) == survivors_total
                )
                and self.collisions == 0
                and self.drone_drone_collisions == 0
                and drones_returned == len(self.runtimes)
            )
        )
        role_failure_metrics = self._role_failure_metrics(
            mission_success=mission_success,
            survivor_recall=survivor_recall,
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
            relay_strategy=self.config.relay_strategy,
            relay_deployments=self.relay_deployments,
            successful_relay_deployments=(
                self.successful_relay_deployments
            ),
            failed_relay_deployments=self.failed_relay_deployments,
            relay_steps_by_drone={
                drone_id: self.runtimes[drone_id].relay_role_steps
                for drone_id in sorted(self.runtimes)
            },
            relay_path_length_by_drone={
                drone_id: self.runtimes[drone_id].relay_path_length
                for drone_id in sorted(self.runtimes)
            },
            relay_unique_cells_forwarded=len(
                self._relay_unique_cells_forwarded
            ),
            relay_survivor_confirmations_forwarded=len(
                self._relay_survivors_forwarded
            ),
            relay_outages_shortened=self.relay_outages_shortened,
            base_known_coverage_over_time=tuple(
                self._base_known_coverage_history
            ),
            time_to_first_base_survivor_confirmation=(
                min(base_confirmation_steps)
                if base_confirmation_steps
                else None
            ),
            time_to_all_base_survivor_confirmations=(
                max(base_confirmation_steps)
                if survivors_total
                and len(base_confirmation_steps) == survivors_total
                else None
            ),
            relay_energy_consumed=self._relay_energy_consumed,
            relay_mission_delay_steps=self._relay_mission_delay_steps,
            network_profile=self.config.network_profile,
            network_messages_queued=(
                transport.queued_messages if transport is not None else 0
            ),
            network_messages_delivered=(
                transport.delivered_messages if transport is not None else 0
            ),
            network_fragments_sent=(
                transport.sent_fragments if transport is not None else 0
            ),
            network_fragments_delivered=(
                transport.delivered_fragments if transport is not None else 0
            ),
            network_fragments_lost=(
                transport.lost_fragments if transport is not None else 0
            ),
            network_messages_expired=(
                transport.expired_fragments if transport is not None else 0
            ),
            network_messages_dropped=(
                transport.dropped_fragments if transport is not None else 0
            ),
            network_delivery_ratio=(
                transport.delivery_ratio if transport is not None else None
            ),
            network_transmission_attempts=(
                transport.transmission_attempts if transport is not None else 0
            ),
            network_successful_transmission_attempts=(
                transport.successful_transmission_attempts
                if transport is not None
                else 0
            ),
            network_retransmission_attempts=(
                transport.retransmission_attempts
                if transport is not None
                else 0
            ),
            network_fragments_created=(
                transport.created_fragments if transport is not None else 0
            ),
            network_fragment_attempt_delivery_ratio=(
                transport.fragment_attempt_delivery_ratio
                if transport is not None
                else None
            ),
            network_unique_fragment_eventual_delivery_ratio=(
                transport.unique_fragment_eventual_delivery_ratio
                if transport is not None
                else None
            ),
            network_logical_message_completion_ratio=(
                transport.logical_message_completion_ratio
                if transport is not None
                else None
            ),
            network_routes_replanned=(
                transport.routes_replanned if transport is not None else 0
            ),
            network_mean_latency=(
                transport.mean_latency if transport is not None else None
            ),
            network_max_latency=(
                transport.max_latency if transport is not None else None
            ),
            network_payload_units_delivered=(
                transport.payload_units_delivered if transport is not None else 0
            ),
            network_average_queue_size=(
                transport.average_queue_size if transport is not None else None
            ),
            network_max_queue_size=(
                transport.maximum_queue_size if transport is not None else 0
            ),
            network_max_backlog_duration=(
                transport.max_backlog_duration if transport is not None else 0
            ),
            stale_motion_intents=(
                transport.stale_intents if transport is not None else 0
            ),
            map_sync_mean_latency=(
                sum(self._network_map_latencies) / len(self._network_map_latencies)
                if self._network_map_latencies
                else None
            ),
            map_sync_max_latency=(
                max(self._network_map_latencies)
                if self._network_map_latencies
                else None
            ),
            survivor_knowledge_mean_latency=(
                sum(self._network_survivor_latencies)
                / len(self._network_survivor_latencies)
                if self._network_survivor_latencies
                else None
            ),
            relay_network_fragments=(
                transport.relay_fragments_forwarded
                if transport is not None
                else 0
            ),
            relay_network_mean_latency=(
                sum(transport.relay_latencies) / len(transport.relay_latencies)
                if transport is not None and transport.relay_latencies
                else None
            ),
            proximity_avoidances_without_fresh_intent=(
                self.proximity_avoidances_without_fresh_intent
            ),
            final_sync_started=(self._final_sync_started_step is not None),
            final_sync_duration=self._final_sync_duration,
            final_sync_retransmissions=(
                (
                    transport.retransmission_attempts
                    - self._final_sync_retransmissions_at_start
                )
                if transport is not None
                and self._final_sync_started_step is not None
                else 0
            ),
            final_sync_survivor_confirmations_transferred=(
                len(
                    self._base_confirmed_survivors
                    - self._final_sync_base_survivors_at_start
                )
                if self._final_sync_started_step is not None
                else 0
            ),
            final_sync_timeout=self._final_sync_timeout,
            network_shield_cause_classification=dict(
                sorted(self._network_shield_cause_classification.items())
            ),
            network_shield_geometry_classification=dict(
                sorted(self._network_shield_geometry_classification.items())
            ),
            network_aware_relay_metrics=self._network_aware_relay_metrics(),
            failure_recovery_metrics=failure_recovery_metrics,
            smoke_metrics=self._smoke_metrics(),
            perception_metrics=self._perception_metrics(),
            dynamic_obstacle_metrics=self._dynamic_obstacle_metrics(),
            role_failure_metrics=role_failure_metrics,
            multi_relay_metrics=self._multi_relay_metrics(),
            mission_success=mission_success,
            mission_events=self.mission_log.events,
        )
