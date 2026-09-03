# ADR 0003: Separate Ground Truth from agent knowledge

## Status

Accepted

## Context

The generated world must know every wall, Survivor, and Smoke cell to evaluate a
mission. An autonomous agent should only act on information it has sensed or
received. Mixing those views would create an oracle and invalidate exploration,
communication, and failure-recovery experiments.

## Decision

`GridWorld` owns Ground Truth. `OccupancyMap` and `KnowledgeMap` contain only
observed records. Active-local planning reads the observing drone's local map;
radio-connected components exchange explicitly represented knowledge. Ground
Truth is used for sensing, collision evaluation, and final metrics, not as an
implicit planner input. Operator replays omit unknown walls, unconfirmed
Survivor positions, and the full Smoke field unless an explicit debug option is
used.

The boundary is implemented in
[`environment.py`](../../src/echorescue/environment.py),
[`mapping.py`](../../src/echorescue/mapping.py),
[`knowledge.py`](../../src/echorescue/knowledge.py), and
[`replay.py`](../../src/echorescue/replay.py).

## Consequences

- Coverage and Survivor knowledge can diverge between drones and the base.
- Communication experiments measure actual information transfer rather than
  access to a shared oracle.
- Debug artifacts that expose Ground Truth must remain explicit and unsuitable
  for ordinary operator views.
- New planners and sensors must state which knowledge boundary they consume.
