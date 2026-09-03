# ADR 0001: Deterministic simulation

## Status

Accepted

## Context

EchoRescue compares coordination, communication, failure, and perception
strategies over repeated mission seeds. Mutable global randomness would make a
behavior change difficult to distinguish from sampling noise and would weaken
replay regression tests.

## Decision

Mission generation is controlled by `SimulationConfig.seed`. Probabilistic-like
transport loss and perception outcomes are derived from stable identities and
cryptographic hashes rather than an unseeded global random stream. Ordering and
tie-breaking use stable identifiers and coordinates. Benchmarks repeat candidate
runs and compare their complete results.

The implementation is visible in
[`environment.py`](../../src/echorescue/environment.py),
[`network_transport.py`](../../src/echorescue/network_transport.py), and
[`survivors.py`](../../src/echorescue/survivors.py).

## Consequences

- A seed and configuration reproduce the same world, mission events, metrics,
  and replay.
- Paired experiments can attribute differences to the selected strategy or
  sensor profile.
- Adding a decision input requires updating its stable identity deliberately.
- Determinism is a software property of this simulator, not a claim that real
  sensors or networks are deterministic.
