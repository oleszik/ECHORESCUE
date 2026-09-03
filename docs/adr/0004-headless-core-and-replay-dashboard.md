# ADR 0004: Headless core and replay dashboard

## Status

Accepted

## Context

Mission logic must be testable in CI and benchmarkable without a browser or
display. Visualization should not change timing, ordering, sensing, planning,
or control decisions.

## Decision

The simulation runs as a headless Python core. It exposes result objects and an
optional read-only frame callback. `ReplayRecorder` captures already-computed
operator-visible state into JSON. A small HTTP server serves static dashboard
assets plus selected replay and benchmark files; browser JavaScript renders
those artifacts but never calls back into mission control.

The boundary is implemented in
[`simulation.py`](../../src/echorescue/simulation.py),
[`multi_simulation.py`](../../src/echorescue/multi_simulation.py),
[`replay.py`](../../src/echorescue/replay.py), and
[`dashboard.py`](../../src/echorescue/dashboard.py).

## Consequences

- Tests and benchmarks run without UI dependencies.
- A saved mission can be inspected without rerunning it.
- Dashboard defects cannot alter the recorded mission outcome.
- Live control, streaming simulation, authentication, and production HTTP
  serving are outside the built-in dashboard's responsibility.
