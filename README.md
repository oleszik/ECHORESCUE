# EchoRescue

EchoRescue is a deterministic, grid-based search-and-rescue simulation. This
current slice demonstrates two autonomous drones exploring an initially
unknown floor with local sensing, a shared occupancy map, centrally decoupled
frontier targets, A* path planning, deterministic survivor events, and
independent energy-aware return-to-base behavior.

> Status: Phase 1 foundation. This is a software simulation, not evidence of
> real-world flight safety. Communication, roles, failures, relay behavior,
> replay files, and full mission reports belong to later phases.

## Architecture

Ground truth and the drones' discovered knowledge are deliberately separate:

```text
GridWorld -> local sensors -> shared OccupancyMap -> central assignments
     |                                      |                 |
     +-> per-drone survivor observations    +-> A* paths -> drone-1
                    |                       +-> A* paths -> drone-2
                    +-> shared confirmations and MissionLog
```

All simulation decisions use only the discovered occupancy map. The seed is
owned by `SimulationConfig`, and the simulation uses deterministic ordering for
frontier and path choices. Unconfirmed survivor observations are per drone: two
single sightings by different drones do not combine into a confirmation.

The default two-drone start uses the base and its free eastern neighbor. A
configurable shared-base start is also supported. The base represents a docking
zone with virtual slots: landed drones no longer block the base or adjacent
cells, so both can finish at the same base without a grid collision.

## Quick start

Python 3.10 or newer is required. The runtime has no third-party dependencies.

```bash
python -m pip install -e .
python -m echorescue --visualize --delay 0.03
```

For a fast headless run:

```bash
python -m echorescue --seed 7 --drones 2
```

The command prints a machine-readable JSON summary. Operator output never
reveals unknown geometry or unconfirmed survivor positions. Survivor sensing
can be configured with `--survivor-range` and `--confirmation-observations`.
Energy settings are exposed as `--battery-capacity`, `--movement-energy`,
`--sensor-energy`, `--wait-energy`, and `--energy-reserve`. Use `--drones 1`
for the preserved single-drone regression mode, or `--start-mode shared-base`
for two virtual launch slots at the base.

## Tests

```bash
python -m unittest discover -s tests -v
```

The suite covers seeded world generation, sensor occlusion, occupancy updates,
frontier detection, A* planning, survivor event deduplication, hidden-state
visualization, movement safety, and deterministic regression.
It also verifies deterministic energy accounting, reserve-aware return
decisions, known-free return paths, landing, and explicit emergency states.
Multi-drone tests cover deterministic distinct target assignment, route
blocking, vertex and edge-swap conflicts, per-drone event deduplication,
independent return states, and joint mission termination.

## Current limitations

- one floor and at most two drones
- cardinal, noise-free distance readings
- static obstacles and a terminal visualization
- survivor confirmation uses two unobstructed observations but does not yet
  trigger a dedicated verification state
- energy is an abstract deterministic budget, not a physical discharge model
- return paths assume static, accurately mapped occupied/free cells
- duplicate exploration is defined as the fraction of non-base visited cells
  visited by both drones; sensor-footprint overlap is not yet counted
- shared state is instantaneous; communication loss is not modeled
- no roles, failures, relays, or task redistribution
- no claim of hardware validation

The next safe increment is benchmark-driven refinement of target allocation;
communication and roles remain intentionally out of scope.
