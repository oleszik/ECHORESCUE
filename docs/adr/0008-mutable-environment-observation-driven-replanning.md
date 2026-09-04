# ADR 0008: Mutable environment with observation-driven replanning

## Status

Accepted

## Context

EchoRescue originally represented walls as immutable generated geometry. Agent
knowledge could be incomplete but traversability itself never changed, so a
cached path could only become temporarily unusable because of another agent.
Dynamic closures require mutable Ground Truth without granting planners unfair
access to that state.

## Decision

Keep initial geometry immutable and add a persistent dynamic-obstacle layer to
`GridWorld`. Deterministic events mutate only that Ground-Truth layer. Agent
maps learn `FREE → OCCUPIED` exclusively through sensor or safety-contact
observations. Timestamped knowledge accepts newer occupied evidence and its
safety-first merge prevents stale free reports from reopening a closure.

When an observed closure intersects any future cell of a cached exploration or
return path, explicitly mark that path invalid. Replanning first preserves the
same reachable frontier through the existing deterministic allocator, then
releases and replaces an unreachable target. Return paths use the existing RTB,
energy-reserve, and `RETURN_PATH_UNAVAILABLE` semantics. A final motion shield
turns attempted entry into an unobserved dynamic obstacle into a contact
observation and wait, never a collision.

Operator replay exposes only observed closures. Injection events remain
internal evaluator telemetry. The non-adversarial profile is derived from seed
and static map structure and never reads agent paths.

## Consequences

- The generated map remains reproducible while mission traversability can
  change deterministically.
- Ground Truth, agent knowledge, and operator knowledge remain separate.
- Exploration, A*, coordination, energy, communication, and failure recovery
  retain one N-agent implementation.
- Reopening requires a future explicit conflict/version policy; v0.8 closures
  are permanent.
- A paired baseline is required to measure total mission-duration and path
  overhead; a single replay cannot supply that counterfactual.
