# EchoRescue

EchoRescue is a deterministic, grid-based search-and-rescue simulation. This
first vertical slice demonstrates one autonomous drone exploring an initially
unknown floor with local sensing, an occupancy map, frontier selection, A* path
planning, and deterministic survivor detection events.

> Status: Phase 1 foundation. This is a software simulation, not evidence of
> real-world flight safety. Multi-drone coordination, survivors, batteries,
> replay files, and mission reports belong to the later MVP phase.

## Architecture

Ground truth and the drone's discovered knowledge are deliberately separate:

```text
GridWorld -> DistanceSensor -> OccupancyMap -> Frontier selection -> A* -> Drone
     |                                                               |
     +-> SurvivorSensor -> MissionLog -> confirmed-only terminal view +
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

The command prints a machine-readable JSON summary. Use
`--show-ground-truth` only as a debug view; autonomous decisions never receive
that hidden geometry. Even the debug view does not reveal unconfirmed survivor
positions. Survivor sensing can be configured with `--survivor-range` and
`--confirmation-observations`.

## Tests

```bash
python -m unittest discover -s tests -v
```

The suite covers seeded world generation, sensor occlusion, occupancy updates,
frontier detection, A* planning, survivor event deduplication, hidden-state
visualization, movement safety, and deterministic regression.

## Current limitations

- one floor and one drone
- cardinal, noise-free distance readings
- static obstacles and a terminal visualization
- survivor confirmation uses two unobstructed observations but does not yet
  trigger a dedicated verification state
- no energy model, return-to-base logic, or communication model yet
- no claim of hardware validation

The next safe increment is energy-aware return-to-base behavior before adding
the second drone.
