import os
import sys
import time
from dataclasses import dataclass

from echorescue.models import CellState, DroneStatus, Position
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.simulation import Simulation


SimulationView = Simulation | MultiDroneSimulation


def _base_grid(simulation: SimulationView) -> list[list[str]]:
    symbols = {
        CellState.UNKNOWN: "?",
        CellState.FREE: ".",
        CellState.OCCUPIED: "#",
    }
    return [
        [
            symbols[simulation.occupancy_map.cell_at(Position(x, y))]
            for x in range(simulation.config.width)
        ]
        for y in range(simulation.config.height)
    ]


def _render_single(simulation: Simulation) -> str:
    grid = _base_grid(simulation)
    for position in simulation.current_return_path[1:]:
        grid[position.y][position.x] = "r"
    for position in simulation.confirmed_survivors:
        grid[position.y][position.x] = "S"
    base = simulation.world.base
    grid[base.y][base.x] = "B"
    drone = simulation.drone
    if drone.position == base and drone.status is DroneStatus.LANDED:
        grid[base.y][base.x] = "L"
    else:
        grid[drone.position.y][drone.position.x] = "D"

    rows = ["".join(row) for row in grid]
    rows.append(
        f"step={simulation.steps}  known={simulation.occupancy_map.explored_percent:.1f}%  "
        f"survivors={len(simulation.confirmed_survivors)}/{len(simulation.world.survivors)}  "
        f"battery={simulation.battery.remaining:.1f}/{simulation.battery.capacity:.1f} "
        f"({simulation.battery.remaining_percent:.1f}%)"
    )
    rows.append(
        f"state={drone.status.value}  collisions={simulation.collisions}  "
        f"status={simulation.termination_reason}"
    )
    rows.append(
        "legend: D=drone L=landed@base B=base r=return path "
        "S=confirmed survivor #=wall .=free ?=unknown"
    )
    return "\n".join(rows)


def _render_multi(simulation: MultiDroneSimulation) -> str:
    grid = _base_grid(simulation)
    path_symbols = {"drone-1": "r", "drone-2": "q"}
    target_symbols = {"drone-1": "a", "drone-2": "b"}
    drone_symbols = {"drone-1": "1", "drone-2": "2"}

    for drone_id, runtime in sorted(simulation.runtimes.items()):
        for position in runtime.current_return_path[1:]:
            current = grid[position.y][position.x]
            symbol = path_symbols[drone_id]
            grid[position.y][position.x] = "+" if current in {"r", "q"} else symbol
    for drone_id, runtime in sorted(simulation.runtimes.items()):
        target = runtime.active_frontier_target
        if target is not None:
            grid[target.y][target.x] = target_symbols[drone_id]
    for position in simulation.confirmed_survivors:
        grid[position.y][position.x] = "S"

    base = simulation.world.base
    grid[base.y][base.x] = "B"
    visible_drones: dict[Position, list[str]] = {}
    for drone_id, runtime in sorted(simulation.runtimes.items()):
        if runtime.drone.status is not DroneStatus.LANDED:
            visible_drones.setdefault(runtime.drone.position, []).append(drone_id)
    for position, drone_ids in visible_drones.items():
        grid[position.y][position.x] = (
            drone_symbols[drone_ids[0]] if len(drone_ids) == 1 else "*"
        )

    rows = ["".join(row) for row in grid]
    rows.append(
        f"step={simulation.steps}  known={simulation.occupancy_map.explored_percent:.1f}%  "
        f"survivors={len(simulation.confirmed_survivors)}/{len(simulation.world.survivors)}  "
        f"wall_collisions={simulation.collisions}  "
        f"drone_collisions={simulation.drone_drone_collisions}"
    )
    for drone_id, runtime in sorted(simulation.runtimes.items()):
        target = runtime.active_frontier_target
        target_text = f"({target.x},{target.y})" if target is not None else "-"
        rows.append(
            f"{drone_id}: state={runtime.drone.status.value}  "
            f"battery={runtime.battery.remaining:.1f}/{runtime.battery.capacity:.1f} "
            f"({runtime.battery.remaining_percent:.1f}%)  target={target_text}"
        )
    rows.append(f"mission_status={simulation.termination_reason}")
    rows.append(
        "legend: 1/2=drones B=shared dock a/b=frontier targets "
        "r/q=return paths +=shared return cell S=confirmed survivor "
        "#=wall .=free ?=unknown"
    )
    return "\n".join(rows)


def render_text(
    simulation: SimulationView, show_ground_truth: bool = False
) -> str:
    # Retain the argument for API compatibility. Operator output never reveals
    # unknown ground-truth geometry.
    _ = show_ground_truth
    if isinstance(simulation, MultiDroneSimulation):
        return _render_multi(simulation)
    return _render_single(simulation)


@dataclass(slots=True)
class TerminalRenderer:
    delay: float = 0.03
    show_ground_truth: bool = False
    _first_frame: bool = True

    def __call__(self, simulation: SimulationView) -> None:
        if not self._first_frame and sys.stdout.isatty():
            print("\033[H\033[2J", end="")
        elif self._first_frame and sys.stdout.isatty() and os.name == "nt":
            os.system("")
        print(render_text(simulation, self.show_ground_truth), flush=True)
        self._first_frame = False
        if self.delay > 0:
            time.sleep(self.delay)
