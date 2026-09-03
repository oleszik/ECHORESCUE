# ADR 0007: Evidence-based Survivor hypotheses

## Status

Accepted

## Context

The original Survivor pipeline exposed successful range/LOS sensor observations
as exact grid cells and confirmed a cell after two successful observations. It
could express Thermal misses and Smoke attenuation, but not false-positive
beliefs, negative evidence, or confidence-based confirmation without mixing
agent knowledge with simulator Ground Truth.

## Decision

Represent noisy reports as location-keyed hypotheses containing accumulated
evidence, positive/negative counts, source channels and agents, timestamps, and
an `unconfirmed`, `confirmed`, or `rejected` status. The tracker consumes only
reported positions and confidence and has no world dependency.

Positive evidence adds `0.5 × confidence`. Confirmation requires at least two
positive observations and evidence `>= 0.65`. An observable hypothesis absent
from a sweep loses `0.55 × (0.25 × channel detection probability)` evidence.
After at least two negatives, evidence `<= 0.12` rejects it. Mission evaluation
separately compares confirmed locations with Ground Truth; a false confirmation
makes success false. Default `perception_noise = "off"` takes the unchanged
legacy counting path.

## Consequences

- Confidence is a decision input rather than telemetry only.
- False positives can be rejected without acquiring Survivor IDs.
- Operator replay exposes belief state, never hypothesis correctness.
- No verification task, navigation rule, sensor fusion, or planner input is
  introduced.
- Shared-knowledge noisy perception is the validated v0.7 experiment; richer
  belief synchronization under constrained local knowledge remains future work.
