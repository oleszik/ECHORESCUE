import os
import sys
import time
from dataclasses import dataclass

from echorescue.models import CellState, Position
from echorescue.simulation import Simulation


def render_text(simulation: Simulation, show_ground_truth: bool = False) -> str:
    symbols = {
        CellState.UNKNOWN: "?",
        CellState.FREE: ".",
        CellState.OCCUPIED: "#",
    }
    rows = []
    for y in range(simulation.config.height):
        row = []
        for x in range(simulation.config.width):
            position = Position(x, y)
            state = (
                simulation.world.cell_at(position)
                if show_ground_truth
                else simulation.occupancy_map.cell_at(position)
            )
            symbol = symbols[state]
            if position in simulation.confirmed_survivors:
                symbol = "S"
            if position == simulation.world.base:
                symbol = "B"
            if position == simulation.drone.position:
                symbol = "D"
            row.append(symbol)
        rows.append("".join(row))
    rows.append(
        f"step={simulation.steps}  known={simulation.occupancy_map.explored_percent:.1f}%  "
        f"survivors={len(simulation.confirmed_survivors)}/{len(simulation.world.survivors)}  "
        f"collisions={simulation.collisions}  status={simulation.termination_reason}"
    )
    rows.append("legend: D=drone B=base S=confirmed survivor #=wall .=free ?=unknown")
    return "\n".join(rows)


@dataclass(slots=True)
class TerminalRenderer:
    delay: float = 0.03
    show_ground_truth: bool = False
    _first_frame: bool = True

    def __call__(self, simulation: Simulation) -> None:
        if not self._first_frame and sys.stdout.isatty():
            # ANSI clear keeps the visualizer dependency-free and portable to
            # modern terminals. Captured output remains a sequence of frames.
            print("\033[H\033[2J", end="")
        elif self._first_frame and sys.stdout.isatty() and os.name == "nt":
            os.system("")  # Enable ANSI processing on supported Windows terminals.
        print(render_text(simulation, self.show_ground_truth), flush=True)
        self._first_frame = False
        if self.delay > 0:
            time.sleep(self.delay)
