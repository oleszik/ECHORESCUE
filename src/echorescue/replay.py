import json
from collections.abc import Callable
from dataclasses import asdict
from pathlib import Path

from echorescue.communication import BASE_NODE_ID, CommunicationLink
from echorescue.config import SimulationConfig
from echorescue.events import EventType
from echorescue.knowledge import KnowledgeMap
from echorescue.models import CellState, DroneStatus, Position
from echorescue.multi_simulation import (
    DroneRuntime,
    MultiDroneSimulation,
    MultiSimulationResult,
)


REPLAY_SCHEMA_VERSION = "1.5"
CONSTRAINED_REPLAY_SCHEMA_VERSION = "1.7"
NETWORK_AWARE_REPLAY_SCHEMA_VERSION = "1.8"
FAILURE_RECOVERY_REPLAY_SCHEMA_VERSION = "1.9"
SMOKE_REPLAY_SCHEMA_VERSION = "2.0"
NOISY_PERCEPTION_REPLAY_SCHEMA_VERSION = "2.1"
DYNAMIC_OBSTACLE_REPLAY_SCHEMA_VERSION = "2.2"
ROLE_FAILURE_REPLAY_SCHEMA_VERSION = "2.3"
CELL_SYMBOLS = {
    CellState.UNKNOWN: "?",
    CellState.FREE: ".",
    CellState.OCCUPIED: "#",
}


def _position(position: Position) -> list[int]:
    return [position.x, position.y]


def _map_rows(simulation: MultiDroneSimulation) -> list[str]:
    if simulation.knowledge_mode == "local":
        return _knowledge_rows(
            simulation.shadow_synchronizer.shared_shadow_map()
        )
    return [
        "".join(
            CELL_SYMBOLS[
                simulation.occupancy_map.cell_at(Position(x, y))
            ]
            for x in range(simulation.config.width)
        )
        for y in range(simulation.config.height)
    ]


def _knowledge_rows(knowledge_map: KnowledgeMap) -> list[str]:
    return [
        "".join(
            CELL_SYMBOLS[knowledge_map.cell_at(Position(x, y))]
            for x in range(knowledge_map.width)
        )
        for y in range(knowledge_map.height)
    ]


def _remaining_path(runtime: DroneRuntime) -> tuple[Position, ...]:
    drone = runtime.drone
    path = (
        runtime.current_return_path
        if drone.status is DroneStatus.RETURN_HOME
        else runtime.planned_path
    )
    if drone.position in path:
        return path[path.index(drone.position) :]
    return path


class ReplayRecorder:
    """Read-only observer that snapshots public multi-drone simulation state."""

    def __init__(self, *, include_debug_smoke: bool = False) -> None:
        self._frames: list[dict[str, object]] = []
        self._include_debug_smoke = include_debug_smoke

    def capture(self, simulation: MultiDroneSimulation) -> None:
        shared_shadow_map = simulation.shadow_synchronizer.shared_shadow_map()
        drones = {}
        for drone_id, runtime in sorted(simulation.runtimes.items()):
            current_task = (
                simulation.task_registry.current_for(drone_id)
                if simulation.roles_enabled
                else None
            )
            path = _remaining_path(runtime)
            target = (
                runtime.relay_target
                if runtime.drone.status is DroneStatus.RELAY
                else runtime.active_frontier_target
            )
            connection = simulation.communication_snapshot.connections[drone_id]
            intent = (
                simulation.motion_intents.get(drone_id)
                if simulation.knowledge_mode == "local"
                else None
            )
            drone_payload: dict[str, object] = {
                "position": _position(runtime.drone.position),
                "state": runtime.drone.status.value,
                "energy_remaining": round(runtime.battery.remaining, 6),
                "energy_remaining_percent": round(
                    runtime.battery.remaining_percent, 3
                ),
                "target": _position(target) if target is not None else None,
                **(
                    {
                        "role": runtime.role.value,
                        "task_id": (
                            current_task.identifier
                            if current_task is not None
                            else None
                        ),
                    }
                    if simulation.roles_enabled
                    else {}
                ),
                "planned_path": [_position(position) for position in path],
                "path_kind": (
                    "return"
                    if runtime.drone.status is DroneStatus.RETURN_HOME
                    else (
                        "relay"
                        if runtime.drone.status is DroneStatus.RELAY
                        else "frontier"
                    )
                ),
                "relay": {
                    "active": runtime.drone.status is DroneStatus.RELAY,
                    "strategy": simulation.config.relay_strategy,
                    "position": (
                        _position(runtime.relay_target)
                        if runtime.relay_target is not None
                        else None
                    ),
                    "scout_id": runtime.relay_scout_id,
                    "link_achieved": runtime.relay_link_achieved,
                    "role_steps": runtime.relay_role_steps,
                    "holding_for_relay": runtime.holding_for_relay,
                    **(
                        {
                            "utility": runtime.network_relay_utility,
                            "decision_reason": runtime.network_relay_reason,
                            "critical_backlog": (
                                runtime.network_relay_critical_backlog
                            ),
                            "backpressure": (
                                runtime.network_relay_backpressure
                            ),
                            "expected_payload_units": (
                                runtime.network_relay_expected_units
                            ),
                            "forwarded_payload_units": (
                                runtime.network_relay_forwarded_units
                            ),
                            "transfer_progress": round(
                                runtime.network_relay_forwarded_units
                                / max(1, runtime.network_relay_expected_units),
                                6,
                            ),
                        }
                        if simulation.config.relay_strategy
                        == "network-aware"
                        else {}
                    ),
                },
                "yielding": runtime.yielding,
                "motion_intent": (
                    {
                        "current_position": _position(
                            intent.current_position
                        ),
                        "next_position": _position(intent.next_position),
                        "reservation": [
                            _position(position)
                            for position in intent.reservation
                        ],
                        "state": intent.status.value,
                        "energy_remaining": round(
                            intent.energy_remaining, 6
                        ),
                        "safe_energy_margin": round(
                            intent.safe_energy_margin, 6
                        ),
                        "valid_until_step": intent.valid_until_step,
                    }
                    if intent is not None
                    else None
                ),
                "communication": {
                    "connected_to_base": connection.connected_to_base,
                    "direct_to_base": connection.direct_to_base,
                    "via_relay": connection.via_relay,
                    "relay_path": list(connection.relay_path),
                },
                "knowledge": {
                    "known_coverage": round(
                        runtime.local_map.known_coverage, 6
                    ),
                    "stale_cells": len(
                        runtime.local_map.stale_against(shared_shadow_map)
                    ),
                    "average_data_age": round(
                        runtime.local_map.average_data_age(simulation.steps),
                        3,
                    ),
                    "oldest_data_age": runtime.local_map.oldest_data_age(
                        simulation.steps
                    ),
                    "detected_survivors": len(
                        runtime.detected_survivors
                    ),
                    "confirmed_survivors": len(
                        runtime.confirmed_survivors
                    ),
                },
            }
            if simulation.config.smoke_profile != "off":
                density = simulation.world.smoke.density_at(
                    runtime.drone.position
                )
                drone_payload["smoke"] = {
                    "density": round(density, 6),
                    "in_smoke": density > 0.0,
                }
            drones[drone_id] = drone_payload
        relay_edges = {
            CommunicationLink.between(first, second)
            for connection in (
                simulation.communication_snapshot.connections.values()
            )
            for first, second in zip(
                connection.relay_path, connection.relay_path[1:]
            )
        }
        communication_links = []
        for link in simulation.communication_snapshot.links:
            if BASE_NODE_ID in (link.first, link.second):
                kind = "direct_base"
            elif link in relay_edges:
                kind = "relay"
            else:
                kind = "peer"
            communication_links.append(
                {"from": link.first, "to": link.second, "kind": kind}
            )
        operator_rows = _map_rows(simulation)
        base_map = simulation.base_knowledge_map
        knowledge_maps = {
            "operator": {
                "occupancy": operator_rows,
                "known_coverage": round(
                    (
                        shared_shadow_map.known_coverage
                        if simulation.knowledge_mode == "local"
                        else simulation.occupancy_map.explored_percent
                    ),
                    6,
                ),
                "differences_from_shadow": [],
                "confirmed_survivors": [
                    _position(position)
                    for position in sorted(simulation.confirmed_survivors)
                ],
                "purpose": "evaluation_aggregate",
            },
            **{
                drone_id: {
                    "occupancy": _knowledge_rows(runtime.local_map),
                    "known_coverage": round(
                        runtime.local_map.known_coverage, 6
                    ),
                    "differences_from_shadow": [
                        _position(position)
                        for position in runtime.local_map.differs_from(
                            shared_shadow_map
                        )
                    ],
                    "confirmed_survivors": [
                        _position(position)
                        for position in sorted(
                            runtime.confirmed_survivors
                            if simulation.knowledge_mode == "local"
                            else simulation.confirmed_survivors
                        )
                    ],
                    "purpose": "local_decision_knowledge",
                }
                for drone_id, runtime in sorted(simulation.runtimes.items())
            },
            "base": {
                "occupancy": (
                    _knowledge_rows(base_map)
                    if base_map is not None
                    else ["?" * simulation.config.width]
                    * simulation.config.height
                ),
                "known_coverage": round(
                    base_map.known_coverage if base_map is not None else 0.0,
                    6,
                ),
                "differences_from_shadow": [
                    _position(position)
                    for position in (
                        base_map.differs_from(shared_shadow_map)
                        if base_map is not None
                        else tuple(
                            position for position, _ in shared_shadow_map.records
                        )
                    )
                ],
                "confirmed_survivors": [
                    _position(position)
                    for position in sorted(
                        simulation.confirmed_survivors
                    )
                ],
                "purpose": "base_operational_knowledge",
            },
        }
        frame = {
            "step": simulation.steps,
            "drones": drones,
            "occupancy": operator_rows,
            "knowledge_maps": knowledge_maps,
            "shadow_knowledge": {
                "shared_coverage": round(
                    shared_shadow_map.known_coverage, 6
                ),
                "map_divergence_between_drones": round(
                    simulation.shadow_synchronizer.divergence_ratio(), 6
                ),
            },
            "confirmed_survivors": [
                _position(position)
                for position in sorted(simulation.confirmed_survivors)
            ],
            "events": [],
            "communication": {
                "base_station": {
                    "id": BASE_NODE_ID,
                    "position": _position(simulation.world.base),
                },
                "nodes": {
                    node_id: _position(position)
                    for node_id, position in sorted(
                        simulation.communication_snapshot.nodes.items()
                    )
                },
                "links": communication_links,
            },
            "explored_percent": round(
                (
                    shared_shadow_map.known_coverage
                    if simulation.knowledge_mode == "local"
                    else simulation.occupancy_map.explored_percent
                ),
                3,
            ),
        }
        if simulation.network_transport is not None:
            transport = simulation.network_transport
            frame["network"] = {
                "profile": "constrained",
                "physical_links": communication_links,
                "successful_transfer_links": [
                    {"from": first, "to": second}
                    for first, second in sorted(
                        simulation._network_delivered_links_this_step
                    )
                ],
                "delivered_payload_units": (
                    simulation._network_delivered_units_this_step
                ),
                "queue_size": transport.queue_size,
                "average_queue_size": round(
                    transport.average_queue_size, 3
                ),
                "maximum_queue_size": transport.maximum_queue_size,
                "lost_fragments": transport.lost_fragments,
                "expired_fragments": transport.expired_fragments,
                "relay_fragments_forwarded": (
                    transport.relay_fragments_forwarded
                ),
                "relay_forwarding_active": any(
                    len(fragment.route) > 2
                    for fragment in transport._queued + transport._in_flight
                ),
            }
        if simulation.config.perception_noise != "off":
            frame["survivor_hypotheses"] = [
                hypothesis.to_dict()
                for _, hypothesis in sorted(
                    simulation.hypothesis_tracker.hypotheses.items()
                )
            ]
        if simulation.dynamic_obstacles_enabled:
            frame["dynamic_obstacles_observed"] = [
                _position(position)
                for position in sorted(
                    simulation._dynamic_obstacles_observed
                )
            ]
        if simulation.roles_enabled:
            frame["task_ownership"] = [
                task.to_dict()
                for task in simulation.task_registry.active_or_orphaned()
            ]
        if self._frames and self._frames[-1]["step"] == simulation.steps:
            self._frames[-1] = frame
        else:
            self._frames.append(frame)

    def build(
        self,
        simulation: MultiDroneSimulation,
        result: MultiSimulationResult,
    ) -> dict[str, object]:
        events_by_step: dict[int, list[dict[str, object]]] = {}
        for event in result.mission_events:
            if event.event_type in {
                EventType.DYNAMIC_OBSTACLE_INJECTED,
                EventType.DYNAMIC_OBSTACLE_REJECTED,
            }:
                continue
            events_by_step.setdefault(event.step, []).append(event.to_dict())
        frames = []
        for captured in self._frames:
            frame = dict(captured)
            step = frame["step"]
            if not isinstance(step, int):
                raise TypeError("captured replay step must be an integer")
            frame["events"] = events_by_step.get(step, [])
            frames.append(frame)
        configuration = asdict(simulation.config)
        if simulation.network_transport is None:
            for key in tuple(configuration):
                if key.startswith("network_") or key == "final_sync_max_steps":
                    configuration.pop(key)
        if simulation.config.relay_strategy != "network-aware":
            for key in tuple(configuration):
                if key.startswith("network_relay_"):
                    configuration.pop(key)
        if not simulation.config.failure_schedule:
            configuration.pop("failure_schedule", None)
        if simulation.config.smoke_profile == "off":
            configuration.pop("smoke_profile", None)
        if simulation.config.perception_noise == "off":
            for key in (
                "perception_noise",
                "survivor_confirmation_evidence_threshold",
                "survivor_rejection_evidence_threshold",
                "survivor_negative_evidence_weight",
            ):
                configuration.pop(key, None)
        if not simulation.dynamic_obstacles_enabled:
            configuration.pop("dynamic_obstacles", None)
            configuration.pop("dynamic_obstacle_schedule", None)
        if not simulation.roles_enabled:
            configuration.pop("role_policy", None)
        if simulation.config.survivor_sensor == "visual":
            for key in (
                "survivor_sensor",
                "thermal_survivor_sensor_range",
                "thermal_detection_probability",
                "thermal_smoke_attenuation",
            ):
                configuration.pop(key, None)
        mission = {
            "seed": simulation.config.seed,
            "knowledge_mode": simulation.knowledge_mode,
            "relay_strategy": simulation.config.relay_strategy,
            "configuration": configuration,
        }
        if simulation.config.perception_noise != "off":
            mission["perception_noise"] = simulation.config.perception_noise
        if simulation.dynamic_obstacles_enabled:
            mission["dynamic_obstacles"] = (
                "explicit"
                if simulation.config.dynamic_obstacle_schedule
                else simulation.config.dynamic_obstacles
            )
        if simulation.roles_enabled:
            mission["role_policy"] = simulation.config.role_policy
        if simulation.network_transport is not None:
            mission["network_profile"] = simulation.config.network_profile
        if simulation.config.survivor_sensor != "visual":
            mission["survivor_sensor"] = simulation.config.survivor_sensor
        map_payload: dict[str, object] = {
            "width": simulation.config.width,
            "height": simulation.config.height,
            "base": _position(simulation.world.base),
            "cell_encoding": {
                "?": "unknown",
                ".": "free",
                "#": "occupied",
            },
            "initial_known_occupancy": (
                frames[0]["occupancy"] if frames else []
            ),
        }
        if (
            self._include_debug_smoke
            and simulation.config.smoke_profile != "off"
        ):
            map_payload["smoke_debug"] = {
                "debug_only": True,
                "profile": simulation.config.smoke_profile,
                "density": simulation.world.smoke.to_debug_rows(
                    simulation.config.width, simulation.config.height
                ),
            }
        metrics = result.to_dict()
        if simulation.dynamic_obstacles_enabled:
            mission_events = metrics.get("mission_events")
            if isinstance(mission_events, list):
                metrics["mission_events"] = [
                    event
                    for event in mission_events
                    if isinstance(event, dict)
                    and event.get("event_type")
                    not in {
                        EventType.DYNAMIC_OBSTACLE_INJECTED.value,
                        EventType.DYNAMIC_OBSTACLE_REJECTED.value,
                    }
                ]
        return {
            "schema_version": (
                ROLE_FAILURE_REPLAY_SCHEMA_VERSION
                if simulation.roles_enabled
                else DYNAMIC_OBSTACLE_REPLAY_SCHEMA_VERSION
                if simulation.dynamic_obstacles_enabled
                else NOISY_PERCEPTION_REPLAY_SCHEMA_VERSION
                if simulation.config.perception_noise != "off"
                else SMOKE_REPLAY_SCHEMA_VERSION
                if (
                    simulation.config.smoke_profile != "off"
                    or simulation.config.survivor_sensor != "visual"
                )
                else (
                    FAILURE_RECOVERY_REPLAY_SCHEMA_VERSION
                    if simulation.config.failure_schedule
                    else (
                        NETWORK_AWARE_REPLAY_SCHEMA_VERSION
                        if simulation.config.relay_strategy == "network-aware"
                        else (
                            CONSTRAINED_REPLAY_SCHEMA_VERSION
                            if simulation.network_transport is not None
                            else REPLAY_SCHEMA_VERSION
                        )
                    )
                )
            ),
            "mission": {
                **mission,
            },
            "map": map_payload,
            "frames": frames,
            "metrics": metrics,
        }


FrameObserver = Callable[[MultiDroneSimulation], None]


def record_simulation(
    simulation: MultiDroneSimulation,
    observer: FrameObserver | None = None,
    *,
    include_debug_smoke: bool = False,
) -> tuple[dict[str, object], MultiSimulationResult]:
    recorder = ReplayRecorder(include_debug_smoke=include_debug_smoke)

    def capture(simulation_state: MultiDroneSimulation) -> None:
        recorder.capture(simulation_state)
        if observer is not None:
            observer(simulation_state)

    result = simulation.run(capture)
    return recorder.build(simulation, result), result


def generate_replay(
    config: SimulationConfig, *, include_debug_smoke: bool = False
) -> dict[str, object]:
    replay, _ = record_simulation(
        MultiDroneSimulation(config),
        include_debug_smoke=include_debug_smoke,
    )
    return replay


def replay_json_bytes(replay: dict[str, object]) -> bytes:
    return (
        json.dumps(
            replay,
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
        )
        + "\n"
    ).encode("utf-8")


def write_replay(replay: dict[str, object], output: str | Path) -> Path:
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(replay_json_bytes(replay))
    return path
