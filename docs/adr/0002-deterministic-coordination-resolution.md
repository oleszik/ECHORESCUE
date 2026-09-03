# ADR 0002: Deterministic coordination resolution

## Status

Accepted

## Context

Multiple agents can select competing frontiers or propose vertex, edge-swap,
and corridor-conflicting movements. Random winner selection would make safety
events and benchmark outcomes depend on execution order.

## Decision

Frontier candidates are ranked by explicit cost, coordinates, and stable drone
identifier. Movement and distributed-intent conflicts use deterministic
priority keys based on return urgency, safe energy margin, waiting time, and
identifier. A central movement resolver remains the final simulation safety
shield even in active-local mode.

The implemented rules are in
[`coordination.py`](../../src/echorescue/coordination.py),
[`deconfliction.py`](../../src/echorescue/deconfliction.py), and
[`multi_simulation.py`](../../src/echorescue/multi_simulation.py).

## Consequences

- Identical inputs resolve conflicts identically across runs.
- Starvation pressure is observable through accumulated waiting time.
- The final shield prevents simulator-level collisions but is not a proof of
  decentralized real-world collision avoidance.
- Generalizing beyond two agents requires validating that pairwise and global
  ordering rules remain fair at fleet scale.
