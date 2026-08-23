import json
from collections.abc import Callable
from dataclasses import asdict
from pathlib import Path

from echorescue.communication import BASE_NODE_ID, CommunicationLink
from echorescue.config import SimulationConfig
from echorescue.knowledge import KnowledgeMap
from echorescue.models import CellState, DroneStatus, Position
from echorescue.multi_simulation import (
    MultiDroneSimulation,
    MultiSimulationResult,
)


REPLAY_SCHEMA_VERSION = "1.5"
CONSTRAINED_REPLAY_SCHEMA_VERSION = "1.7"
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


def _remaining_path(runtime: object) -> tuple[Position, ...]:
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

    def __init__(self) -> None:
        self._frames: list[dict[str, object]] = []

    def capture(self, simulation: MultiDroneSimulation) -> None:
        shared_shadow_map = simulation.shadow_synchronizer.shared_shadow_map()
        drones = {}
        for drone_id, runtime in sorted(simulation.runtimes.items()):
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
            drones[drone_id] = {
                "position": _position(runtime.drone.position),
                "state": runtime.drone.status.value,
                "energy_remaining": round(runtime.battery.remaining, 6),
                "energy_remaining_percent": round(
                    runtime.battery.remaining_percent, 3
                ),
                "target": _position(target) if target is not None else None,
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
            events_by_step.setdefault(event.step, []).append(event.to_dict())
        frames = []
        for captured in self._frames:
            frame = dict(captured)
            frame["events"] = events_by_step.get(int(frame["step"]), [])
            frames.append(frame)
        configuration = asdict(simulation.config)
        if simulation.network_transport is None:
            for key in tuple(configuration):
                if key.startswith("network_") or key == "final_sync_max_steps":
                    configuration.pop(key)
        mission = {
            "seed": simulation.config.seed,
            "knowledge_mode": simulation.knowledge_mode,
            "relay_strategy": simulation.config.relay_strategy,
            "configuration": configuration,
        }
        if simulation.network_transport is not None:
            mission["network_profile"] = simulation.config.network_profile
        return {
            "schema_version": (
                CONSTRAINED_REPLAY_SCHEMA_VERSION
                if simulation.network_transport is not None
                else REPLAY_SCHEMA_VERSION
            ),
            "mission": {
                **mission,
            },
            "map": {
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
            },
            "frames": frames,
            "metrics": result.to_dict(),
        }


FrameObserver = Callable[[MultiDroneSimulation], None]


def record_simulation(
    simulation: MultiDroneSimulation,
    observer: FrameObserver | None = None,
) -> tuple[dict[str, object], MultiSimulationResult]:
    recorder = ReplayRecorder()

    def capture(simulation_state: MultiDroneSimulation) -> None:
        recorder.capture(simulation_state)
        if observer is not None:
            observer(simulation_state)

    result = simulation.run(capture)
    return recorder.build(simulation, result), result


def generate_replay(config: SimulationConfig) -> dict[str, object]:
    if config.drone_count != 2:
        raise ValueError("portfolio replay generation requires drone_count=2")
    replay, _ = record_simulation(MultiDroneSimulation(config))
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
