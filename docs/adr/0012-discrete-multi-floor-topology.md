# ADR 0012: Discrete multi-floor topology instead of continuous 3D

## Status

Accepted

## Context

EchoRescue needs to study coordination across building floors without turning
the deterministic grid simulator into a flight-dynamics or voxel project. The
existing world, mapping, sensing, planning, safety, and replay contracts are all
two-dimensional.

## Decision

Represent a mission position as the ordered value `(floor, row, col)`. A
`MultiFloorEnvironment` owns independent, potentially differently sized 2D
`GridWorld` instances and explicit weighted `FloorTransition` edges. Horizontal
neighbors remain cardinal 2D cells. A floor change is possible only through an
enabled transition edge.

Transition topology is mission-known; occupancy, Survivors, Smoke, and dynamic
obstacles remain floor-local and initially unknown. Weighted A* uses a zero
heuristic while source and goal are on different floors, retaining
admissibility for arbitrary connector geometry. Single-floor missions continue
through the unchanged legacy `Position` and simulator path.

## Consequences

- The 2D implementation and all legacy replay fingerprints remain isolated.
- Transition traversal has explicit path, step, energy, reservation, and
  telemetry semantics.
- Continuous altitude, free vertical motion, voxel occupancy, RF propagation,
  and physical stair/elevator simulation are deliberately excluded.
- Cross-floor coordination can be evaluated without implying real 3D flight.
