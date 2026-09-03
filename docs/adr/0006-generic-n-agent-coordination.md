# ADR 0006: Generic N-agent coordination

## Status

Accepted

## Context

EchoRescue's public CLI and runtime were historically split between a legacy
single-agent loop and a two-agent coordination loop. Extending that split with
separate four- and eight-agent algorithms would duplicate mission semantics and
make deterministic comparisons unreliable.

## Decision

Use one collection-oriented `MultiDroneSimulation` core for every supported
fleet size. Generate stable `drone-N` identities, place default starts by a
stable nearest-free-cell ordering, preserve the deterministic greedy frontier
allocator, and resolve physical movements over the complete active-agent set.
The virtual base remains the sole legal co-location cell.

The final central movement resolver is the authoritative simulator safety
boundary. Pairwise checks may detect edge and occupancy conflicts, but winners
and waiters are computed from stable global ordering. Replay and telemetry
continue to encode agents as ID-keyed collections, avoiding a schema revision.

Adaptive and Network-aware Relay remain explicitly limited to their validated
two-agent role semantics. That limitation is rejected at configuration time
rather than silently deriving a fixed partner in a larger fleet.

## Consequences

- 1, 2, 4, and 8 agents execute identical mission lifecycle code.
- Excess agents are valid and may remain without a frontier assignment.
- Existing two-agent behavior and replay compatibility are preserved.
- Total path, overlap, and conflict interventions can rise even when duration
  falls; fleet size is therefore an experimental variable, not a monotonic
  performance guarantee.
- Generalized Relay roles and richer fleet-wide distributed reservations remain
  future work.
