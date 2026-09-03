# Phase 5 abstracted Thermal Survivor baseline

## Motivation and research question

The smoke baseline showed that the existing Visual Survivor channel can finish
mapping and return both drones while failing to collect the two observations
needed to confirm every Survivor. This slice asks: **How robust is an
abstracted Thermal Survivor channel against smoke visibility degradation
compared with the existing Visual channel?**

Thermal is evaluated as an isolated alternative. There is no Visual/Thermal
fusion, cross-channel confirmation, Survivor-directed re-exploration, or
sensor-dependent task allocation in this slice.

## Sensor abstraction

Both channels reuse the same conservative supercover line-of-sight test and
therefore cannot see through walls. Both ignore Survivors outside their
configured Euclidean range and use the existing two-observation confirmation
rule.

| Parameter | Visual | Thermal |
| --- | ---: | ---: |
| Range | 3 cells | 3 cells |
| Clear-air base detection probability | 1.00 | 0.60 |
| Smoke attenuation coefficient | 1.00 | 0.15 |
| Smoke effect on effective range | yes | no |
| Wall occlusion | yes | yes |

For Thermal, detection confidence is `0.60 × (1 - 0.15 × line exposure)`.
The same deterministic SHA-256-derived score is used for Thermal with smoke on
and off, so a Smoke-moderate success is always a subset of the corresponding
clear-air successes. Thermal's lower base probability makes it a constrained
baseline rather than an automatic Ground-Truth oracle.

The Visual calculation and its hash identity remain unchanged. Visual Smoke-off
still reproduces the previously stored replay bytes; Visual Smoke-moderate
reproduces the existing 50-seed Smoke benchmark metrics.

## Telemetry and replay

Smoke missions and Thermal missions emit compact
`survivor_sensor_observation` events at the observing drone—not at an
undetected target. Each event reports:

- `sensor_channel` (`visual` or `thermal`);
- target distance without target direction or position;
- line-averaged smoke exposure;
- detection success or failure;
- resulting confidence and deterministic decision score;
- a failure reason when applicable.

Successful first detections and confirmations retain the existing Survivor
events and are channel-labeled in detailed-perception missions. Failed events
never contain a Survivor coordinate. Result metrics separate all failed
observations from causally smoke-degraded observations: a Thermal failure in a
smoke cell counts as smoke-degraded only when the same score would have passed
the clear-air threshold.

Replay schema 2.0 already supports the required mission configuration and
extensible event payload, so no schema increase was needed. Thermal missions
use schema 2.0 even with Smoke off. Schemas 1.5, 1.7, 1.8, and 1.9 and existing
2.0 Smoke replays remain accepted. The dashboard shows the selected Survivor
sensor and renders channel, outcome, range, smoke exposure, and confidence in
the event feed. The full smoke density grid remains available only in an
explicit `--replay-debug-smoke` replay.

## CLI and reproduction

```bash
# Isolated channels
python -m echorescue --drones 2 --seed 3 --survivor-sensor visual --smoke-profile moderate
python -m echorescue --drones 2 --seed 3 --survivor-sensor thermal --smoke-profile moderate

# Comparable debug replays
python -m echorescue --drones 2 --seed 3 --survivor-sensor visual --smoke-profile moderate --replay-debug-smoke --replay-out replays/seed_3_visual_smoke.json
python -m echorescue --drones 2 --seed 3 --survivor-sensor thermal --smoke-profile moderate --replay-debug-smoke --replay-out replays/seed_3_thermal_smoke.json

# Four-profile benchmark and dashboard
python -m echorescue.thermal_benchmark --seeds 50 --output benchmarks/thermal_perception_50_seeds.json
python -m echorescue.dashboard --replay replays/seed_3_thermal_smoke.json --benchmark benchmarks/thermal_perception_50_seeds.json
```

The Thermal range, base probability, and smoke attenuation are centralized in
`SimulationConfig` and also exposed as `--thermal-survivor-range`,
`--thermal-detection-probability`, and `--thermal-smoke-attenuation`.

## Benchmark method

The stored artifact runs seeds 0–49 with two drones and shared knowledge under
four profiles. Every mission is executed twice. Only Survivor sensor channel
and Smoke profile change; all navigation, mapping, energy, communication, and
safety parameters are identical.

| Metric | Visual off | Visual moderate | Thermal off | Thermal moderate |
| --- | ---: | ---: | ---: | ---: |
| Survivor Recall | 100.00% | 79.33% | 96.67% | 96.67% |
| Mission Success | 100.00% | 50.00% | 90.00% | 90.00% |
| Mean first detection | 7.16 | 10.74 | 8.12 | 8.14 |
| Mean mission duration | 72.10 | 72.10 | 72.10 | 72.10 |
| Mean returned drones | 2.00 | 2.00 | 2.00 | 2.00 |
| Mean explored area | 96.95% | 96.95% | 96.95% | 96.95% |
| Successful Survivor observations | 1,897 | 1,274 | 1,130 | 1,097 |
| Failed detection attempts | 0 | 623 | 767 | 800 |
| Causally Smoke-degraded attempts | 0 | 623 | 0 | 33 |
| Smoke-degraded events | 0 | 556 | 0 | 33 |
| Survivor confirmations | 150 | 119 | 145 | 145 |
| Thermal attempts | 0 | 0 | 1,897 | 1,897 |
| Thermal successful observations | 0 | 0 | 1,130 | 1,097 |
| Thermal failed observations | 0 | 0 | 767 | 800 |
| Wall / drone collisions | 0 / 0 | 0 / 0 | 0 / 0 |

All deterministic-repeat, collision, and navigation-equivalence gates passed.
The lower number of successful Thermal observations is not itself a lower
Recall: confirmation depends on whether successes cover each distinct
Survivor at least twice, not only on the aggregate observation count.

## Analysis

Visual's moderate-Smoke penalty is **−20.67 Recall points**, −50 mission-success
points, and +3.58 steps to first detection. Thermal's Recall and mission-success
penalties are **0 points**; first detection changes by only +0.02 steps. Smoke
causes 33 additional Thermal failures, but those failures do not change the
number of confirmed Survivors in this suite.

Without Smoke, Thermal is weaker than Visual: −3.33 Recall points, −10
mission-success points, and +0.96 steps to first detection. Under moderate
Smoke, however, Thermal gains **17.33 Recall points** and 40 mission-success
points over Visual and detects the first Survivor 2.60 steps earlier.

The current Visual baseline has exactly 25 incomplete moderate-Smoke seeds.
Thermal fully recovers **22/25**. Seeds `1`, `4`, and `25` remain incomplete.
Thermal has five incomplete moderate-Smoke seeds in total: `1`, `4`, `21`,
`25`, and `43`. Seeds `21` and `43` are new failures where Visual happened to
succeed, exposing Thermal's lower base probability rather than a smoke problem.

Navigation is invariant across all four profiles: 72.10 mean steps, 2.00
returned drones, 96.95% explored area, and zero wall or drone collisions.
This is expected because Survivor observations do not alter planning.

## Demo seed

Seed 3 is the selected deterministic comparison:

| Seed 3 + moderate Smoke | Visual | Thermal |
| --- | ---: | ---: |
| Survivor Recall | 66.67% | 100.00% |
| Mission Success | no | yes |
| First detection | step 16 | step 10 |
| Mission duration | 60 | 60 |
| Returned drones | 2 | 2 |
| Explored area | 97.07% | 97.07% |
| Wall / drone collisions | 0 / 0 | 0 / 0 |

Visual misses one Survivor confirmation; Thermal confirms all three under the
same world, smoke field, routes, and duration. The paired debug replays are
`replays/seed_3_visual_smoke.json` and
`replays/seed_3_thermal_smoke.json`.

## Failure modes and limits

- Thermal can still miss the two observations required for confirmation due
  to its 0.60 clear-air base probability.
- Aggregate success count does not guarantee per-Survivor confirmation.
- Smoke is static and cell-based; Thermal attenuation is an experimental
  coefficient, not calibrated infrared physics.
- There are no images, heat fields, false-positive heat sources, body
  temperature dynamics, sensor noise calibration, or real camera data.
- A single mission selects exactly one channel. No event from one channel can
  confirm evidence from the other in this slice.
- Results are simulation evidence for this grid, exploration policy, and
  parameter set—not evidence of real-world detector or flight safety.

## Next research question

The evidence supports **Visual + Thermal confidence fusion** as the next
isolated experiment: Thermal recovers 22 Visual failures but introduces two
different failures, demonstrating complementary rather than universally
superior evidence. A fusion experiment should define channel-specific evidence
and confirmation rules, then test whether it recovers both failure sets without
leaking Ground Truth or changing navigation. Fusion is not implemented here.
