# Phase 5 smoke-perception baseline

## Scope and research question

This first Phase-5 vertical slice asks: **How strongly does degraded visibility
reduce Survivor detection and mission performance for the existing strategy?**
It intentionally adds no thermal, acoustic, ultrasonic, fused, or compensating
sensor. Occupancy mapping and movement physics remain unchanged so the
perception effect can be interpreted independently.

## Model and determinism

`SmokeField` is immutable environmental Ground Truth. The `off` profile has no
cells. `moderate` deterministically shuffles free interior candidates with a
smoke-specific random stream derived from the mission seed, chooses three zone
centres, and fills free cells within Manhattan radius 3 at density 0.65. It
never consumes the obstacle or Survivor generator's random streams.

For an otherwise visible Survivor, the sensor averages smoke density along the
conservative line between drone and target. Visibility is `1 - exposure`, with
a lower bound of 0.05. That value first scales effective range and then, for an
in-range exposed observation, controls a deterministic Bernoulli decision. The
decision is a SHA-256-derived value keyed by seed, profile, observer, step,
origin, and Survivor position. Repeated runs do not depend on mutable or
unseeded random state.

The model does not change walls, free cells, the occupancy DistanceSensor,
frontier assignment, A*, energy, communication, or collision avoidance.

## Telemetry and hidden-state boundary

The mission log adds:

- `smoke_entered` and `smoke_exited`, located at the observing drone;
- `survivor_detection_degraded`, aggregating attempts, successful observations,
  and maximum line exposure for one drone and step.

A failed detection does not expose the target position. Result metrics report
profile, zone-cell count, maximum density, exposure samples and entries per
drone, eligible and successful observations, degraded attempts/events, and
successful observations through smoke.

Smoke missions use replay schema 2.0. Each drone frame may expose only its
current local density and `in_smoke` state. Standard operator replays omit the
complete smoke field. `--replay-debug-smoke` explicitly adds a `debug_only`
density grid and enables the separate **Smoke debug** dashboard view. Schemas
1.5, 1.7, 1.8, and 1.9 remain accepted and unchanged.

## Reproduction

```bash
python -m echorescue --drones 2 --seed 1 --smoke-profile moderate --replay-out replays/seed_1_smoke.json --replay-debug-smoke
python -m echorescue.dashboard --replay replays/seed_1_smoke.json --benchmark benchmarks/smoke_perception_50_seeds.json
python -m echorescue.smoke_benchmark --seeds 50 --output benchmarks/smoke_perception_50_seeds.json
```

Seed 1 is the compact visual case: drones enter and exit smoke, degraded
detections are visible in telemetry, one of three Survivors remains
unconfirmed, both drones return, and no collision occurs. This deliberately
shows the new failure mode rather than presenting a cherry-picked success.

## Verified 50-seed result

The stored artifact runs seeds 0–49 with two drones, shared knowledge, and
otherwise default `SimulationConfig` values. Every moderate run is repeated.

| Metric | Smoke off | Moderate smoke | Difference |
| --- | ---: | ---: | ---: |
| Average Survivor Recall | 100.00% | 79.33% | -20.67 pp |
| Mission success | 100.00% | 50.00% | -50.00 pp |
| Mean first detection | 7.16 | 10.74 | +3.58 steps |
| Mean mission duration | 72.10 | 72.10 | 0.00 |
| Mean explored area | 96.95% | 96.95% | 0.00 pp |
| Mean drones returned | 2.00 | 2.00 | 0.00 |
| Eligible detection attempts | 1,897 | 1,897 | 0 |
| Successful confirmation observations | 1,897 | 1,274 | -623 |
| Degraded attempts / events | 0 / 0 | 623 / 556 | +623 / +556 |
| Wall / drone collisions | 0 / 0 | 0 / 0 | 0 / 0 |

Deterministic repeats and all collision gates passed. “Successful confirmation
observations” counts sensor observations that can increment a Survivor's
confirmation counter; it is not the count of distinct confirmed Survivors.
Mean time to first detection is over missions with a detection; all 50 missions
in both profiles detected at least one Survivor.

## Observed failure modes and next experiment

Smoke can prevent the two observations required for confirmation even after
the map is nearly completely explored. It can also delay the first detection.
Because the exploration policy does not revisit cells specifically for a
missed Survivor, the fleet can return safely with incomplete Recall. Conversely,
some smoke missions retain full Recall when geometry provides enough clear or
successful exposed observations; degradation is spatial and seed-dependent.

The next highest-value experiment is an abstract **thermal Survivor channel**.
The evidence isolates visibility-dependent missed confirmations while movement
safety remains intact. A thermal channel with independently modeled smoke
sensitivity would therefore test genuine complementary perception. It should
be benchmarked alone before confidence fusion is introduced. Acoustic sensing
is a reasonable later localization cue; dynamic paths and ultrasonic proximity
do not directly address the measured Recall failure.

## Limits

This is static, cell-based smoke—not CFD, camera physics, real hardware, or a
claim about operational flight safety. Density does not diffuse or change with
time. Drones know only density at their present cell for telemetry; they do not
build or plan from a smoke map. The deterministic hash is an experimental
repeatability mechanism, not a calibrated detector model.
