# EchoRescue

EchoRescue is a deterministic, grid-based search-and-rescue simulation. This
first vertical slice demonstrates one autonomous drone exploring an initially
unknown floor with local sensing, an occupancy map, frontier selection, A* path
planning, and deterministic survivor detection events.
The drone continuously budgets energy for a known-free A* route back to base
and transitions into a dedicated return state before its safety reserve is at
risk.

> Status: Phase 1 foundation. This is a software simulation, not evidence of
> real-world flight safety. Multi-drone coordination, communication, replay
> files, and full mission reports belong to the later MVP phase.

## Architecture

Ground truth and the drone's discovered knowledge are deliberately separate:

```text
GridWorld -> DistanceSensor -> OccupancyMap -> Frontier selection -> A* -> Drone
     |                                                               |
     +-> SurvivorSensor -> MissionLog -> confirmed-only terminal view +
                                  Battery -> safe known return path ---+
```

All simulation decisions use only the discovered occupancy map. The seed is
owned by `SimulationConfig`, and the simulation uses deterministic ordering for
frontier and path choices.

## Quick start

Python 3.10 or newer is required. The runtime has no third-party dependencies.

```bash
python -m pip install -e .
python -m echorescue --visualize --delay 0.03
```

For a fast headless run:

```bash
python -m echorescue --seed 7
```

The command prints a machine-readable JSON summary. Operator output never
reveals unknown geometry or unconfirmed survivor positions. Survivor sensing
can be configured with `--survivor-range` and `--confirmation-observations`.
Energy settings are exposed as `--battery-capacity`, `--movement-energy`,
`--sensor-energy`, and `--energy-reserve`.

## Tests

```bash
python -m unittest discover -s tests -v
```

The suite covers seeded world generation, sensor occlusion, occupancy updates,
frontier detection, A* planning, survivor event deduplication, hidden-state
visualization, movement safety, and deterministic regression.
It also verifies deterministic energy accounting, reserve-aware return
decisions, known-free return paths, landing, and explicit emergency states.

## Current limitations

- one floor and one drone
- cardinal, noise-free distance readings
- static obstacles and a terminal visualization
- survivor confirmation uses two unobstructed observations but does not yet
  trigger a dedicated verification state
- energy is an abstract deterministic budget, not a physical discharge model
- return paths assume static, accurately mapped occupied/free cells
- no communication or multi-drone coordination model yet
- no claim of hardware validation

The next safe increment is target deconfliction before adding the second drone.
