import json
from collections.abc import Callable
from dataclasses import asdict
from pathlib import Path

from echorescue.config import SimulationConfig
from echorescue.models import CellState, DroneStatus, Position
from echorescue.multi_simulation import (
    MultiDroneSimulation,
    MultiSimulationResult,
)


REPLAY_SCHEMA_VERSION = "1.0"
CELL_SYMBOLS = {
    CellState.UNKNOWN: "?",
    CellState.FREE: ".",
    CellState.OCCUPIED: "#",
}


def _position(position: Position) -> list[int]:
    return [position.x, position.y]


def _map_rows(simulation: MultiDroneSimulation) -> list[str]:
    return [
        "".join(
            CELL_SYMBOLS[
                simulation.occupancy_map.cell_at(Position(x, y))
            ]
            for x in range(simulation.config.width)
        )
        for y in range(simulation.config.height)
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
        drones = {}
        for drone_id, runtime in sorted(simulation.runtimes.items()):
            path = _remaining_path(runtime)
            target = runtime.active_frontier_target
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
                    else "frontier"
                ),
            }
        frame = {
            "step": simulation.steps,
            "drones": drones,
            "occupancy": _map_rows(simulation),
            "confirmed_survivors": [
                _position(position)
                for position in sorted(simulation.confirmed_survivors)
            ],
            "events": [],
            "explored_percent": round(
                simulation.occupancy_map.explored_percent, 3
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
        return {
            "schema_version": REPLAY_SCHEMA_VERSION,
            "mission": {
                "seed": simulation.config.seed,
                "configuration": configuration,
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
