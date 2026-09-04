# ADR 0009: Role and task ownership recovery semantics

## Status

Accepted

## Context

EchoRescue's N-agent allocator and fail-stop behavior were deterministic, but
failure recovery represented work as implicit frontier ownership. That was
insufficient to prove that a task was orphaned once, reassigned once, executed
by one winner, or recovered within a measurable interval. Historical Relay
experiments also used role state designed for exactly two agents.

## Decision

Represent roles as data (`GENERALIST`, `SCOUT`, `VERIFIER`, `RELAY`) and put
role preference above the shared frontier, A*, safety, energy, RTB, and
communication mechanisms. Introduce a small task registry with deterministic
identity, type, single owner, lifecycle status, target, priority, and timing.

On fail-stop, orphan the active task before marking its owner failed. Select
one operational replacement with a documented additive score over known path
cost, retargeting, task load, role compatibility, and consumed energy. Require
a known task path and energy-safe return path. Resolve ties by stable data only.
Keep any emergency role takeover sticky until task completion and emit every
transition. Measure both registry assignment latency and first productive
execution latency.

Keep this model opt-in so role policy `off` retains legacy results and replay
serialization. Treat existing two-agent Relay planning as a bounded consumer
of the generic role state, not as a Multi-Relay algorithm.

## Consequences

- An orphan retains its task ID and can have only one replacement owner.
- Candidate choice is deterministic, explainable, role-aware, reachable, and
  constrained by the existing safe-return energy estimate.
- Failed bodies remain obstacles while mission success and return rates use the
  remaining operational fleet.
- Replay schema 2.3 can reconstruct ownership and recovery transitions; older
  schema 2.2 remains supported.
- Shared-mode role/task state is authoritative. Distributed role consensus and
  N-agent Multi-Relay planning remain future work.
