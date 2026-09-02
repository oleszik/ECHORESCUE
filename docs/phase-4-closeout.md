# Phase 4 closeout

Phase 4 is complete for the current two-drone simulation scope. This conclusion
is based on the committed implementation, versioned replay and benchmark
artifacts, compatibility tests, and the full automated suite. It is not a claim
about real radio or flight hardware.

## Capability coverage

| Planned area | Implemented evidence |
| --- | --- |
| Communication range and wall attenuation | Deterministic Euclidean range, conservative wall line-of-sight, direct/peer/base graph telemetry, Active Local knowledge exchange, and constrained delay/loss/capacity/TTL transport. |
| Relay behavior | Opt-in `adaptive` and `network-aware` Relay strategies with local waypoint planning, energy guards, bounded role duration, store-and-forward transport, and replay telemetry. |
| Dynamic roles | Deterministic Scout/Relay assignment, release, cooldown, RTB priority, and return to exploration after the Relay task. No persistent hierarchy is claimed. |
| Task redistribution after failure | Opt-in fail-stop injection, offline failed radio, released frontier responsibility, deterministic reassignment from current knowledge, static-obstacle handling, degraded-success metrics, benchmark, and replay events. |

The defaults remain the pre-Phase-4 behavior: `knowledge_mode=shared`,
`network_profile=ideal`, `relay_strategy=off`, and an empty failure schedule.
Network-aware Relay additionally requires Active Local knowledge and the
constrained network profile. Failure injection is absent unless
`--inject-failure` is supplied.

## Status semantics

- **FAILED** is a terminal simulated vehicle state. An injected failed drone
  stops moving, loses its radio links, relinquishes its frontier responsibility,
  remains a physical obstacle, is counted in `drones_failed`, and is never
  counted in `drones_returned`.
- **Operational** means a drone that was not the target of a triggered injected
  failure. Failure-mode success requires every operational drone to land, no
  unexpected terminal failure, full reachable-Survivor confirmation, and zero
  wall/drone collisions.
- **LANDED / returned** means the drone physically reached the base and entered
  `LANDED`. Only this state contributes to `drones_returned`.
- **LOST** is not a separate state in the current simulator. The project does
  not infer physical loss from radio disconnection. Communication loss is a
  link condition; `FAILED` is a simulated fail-stop condition. Real lost-vehicle
  detection and recovery remain out of scope.

A successful failure mission is therefore a degraded success, not a recovered
failed vehicle. The result still reports one failed drone and only the
operational return count.

## Replay and dashboard compatibility

Replay schemas remain additive and explicitly accepted by the dashboard:

- `1.5`: ideal/shared, Shadow, Active Local, and Adaptive Relay
- `1.7`: constrained transport
- `1.8`: network-aware Relay
- `1.9`: deterministic failure and task reassignment

Schema 1.9 records `drone_failure_injected`, `failure_task_released`,
`failure_task_reassigned`, and `failed_drone_collision_avoided` events plus the
`failure_recovery` metric block. The dashboard marks the failed drone with a
distinct FAILED state and marker, shows its radio offline, and reports returned
operational drones separately. Regression fixtures for older schemas remain
loadable and their no-feature behavior is unchanged.

## Reproducible evidence

```bash
python -m echorescue.network_aware_benchmark --train-seeds 50 --holdout-seeds 50 --base-coverage-quality-target 60 --output benchmarks/network_aware_relay_100_seeds.json --analysis-output benchmarks/network_aware_relay_analysis.json
python -m echorescue.failure_benchmark --seeds 50 --failure-drone drone-2 --failure-step 4 --output benchmarks/failure_reassignment_50_seeds.json
python -m echorescue --drones 2 --seed 7 --inject-failure drone-2:4 --replay-out replays/seed_7_failure.json
python -m pytest -q
python -m compileall -q src tests
```

The network-aware artifact separates training seeds 0-49 and holdout seeds
50-99 and repeats every strategy. The failure artifact repeats all 50 injected
missions. Stored results show 100% mission success and Survivor Recall for the
reported Phase-4 profiles, all required operational returns, and zero wall or
drone collisions. The failure suite records 50 failures, 50 released tasks, 50
reassignments, and zero pending tasks. Network-aware transport improves queue
behavior and duration relative to legacy constrained strategies but trades away
base-map coverage; adding Relay recovers part of that coverage at a duration,
path, and energy cost. Those trade-offs remain visible in the README and source
artifacts.

## Demo assessment

`replays/seed_7_failure.json` is retained as the Phase-4 demo. It shows normal
exploration through step 3, a `drone-2` failure and task reassignment at step 4,
the failed vehicle fixed at `[5, 2]`, `drone-1` passing an adjacent cell without
collision, continued Survivor confirmation and exploration, and the operational
drone landing at step 120. The replay ends with `failure_recovered`, one failed
drone, one returned operational drone, and zero collisions. No larger replay
collection is needed.

## Known limits and residual risks

- Radio and loss models are deterministic abstractions, not RF physics.
- Failure is scheduled and fail-stop; there is no fault diagnosis, partial
  degradation, repair, or real LOST-vehicle state.
- Failure reassignment uses the currently available map. An unreachable or
  stale former target is not forced; responsibility shifts to a reachable
  current frontier.
- Failed-airframe avoidance is validated in the grid simulator only and is not
  evidence of decentralized real-flight safety.
- The system remains limited to one floor and two drones.

Within those declared limits, there is no missing Phase-4 capability that
blocks the first Phase-5 vertical slice.
