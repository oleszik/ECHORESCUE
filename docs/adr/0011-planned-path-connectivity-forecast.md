# ADR 0011: Planned-path connectivity forecast

- Status: Accepted
- Date: 2026-09-04

## Context

Reactive Relay placement waits for an outage. The simulator already has a
short, deterministic planned path and local map knowledge, so a useful near-term
forecast is possible without RF probability models, future obstacle knowledge,
or hidden map access.

## Decision

For the next configurable `K` path positions, evaluate deterministic base
connectivity, hop count, link margin, and the first expected disconnect. LOS is
evaluated exclusively against the forecasting agent's `KnowledgeMap`; unknown
intervening cells are conservative. Other agents' live positions are not used
by the simulator forecast because they may be stale in constrained/local mode.

A predicted loss may trigger the same bounded Relay target and candidate
planner used by reactive Multi-Relay. Prediction changes timing, not routing,
safety, task ownership, or success semantics. The default and frozen v0.10
horizon is four steps.

## Consequences

- Forecasts are reproducible, inspectable, and do not leak future dynamic
  obstacles or unknown static geometry.
- The model can miss connectivity supplied by a future moving peer because it
  deliberately avoids assuming that peer's trajectory.
- Predictive activation can be unnecessary or costlier than reactive behavior;
  both outcomes are measured rather than hidden.
