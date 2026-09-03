# EchoRescue

[![CI](https://github.com/oleszik/ECHORESCUE/actions/workflows/tests.yml/badge.svg)](https://github.com/oleszik/ECHORESCUE/actions/workflows/tests.yml)
[![Python 3.10–3.13](https://img.shields.io/badge/python-3.10--3.13-blue.svg)](pyproject.toml)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

EchoRescue is a deterministic, grid-based search-and-rescue simulator for
studying how autonomous agents explore an unknown environment, coordinate over
imperfect communication, confirm Survivors, recover from failures, and return
safely to base.

![EchoRescue deterministic mission replay](docs/assets/echorescue-mission-replay.gif)

The simulation core is headless and dependency-free. A browser dashboard
replays its versioned, operator-safe telemetry without participating in mission
decisions.

## Verified results

Every result below comes from a committed JSON artifact and can be regenerated
with the documented benchmark commands.

| Experiment | Stored result over 50 seeds |
| --- | --- |
| Noisy Survivor perception | Visual/moderate: 96.67% Recall, 90% mission success, 7.98% FPR, 18.22% FNR; 100/100 agents returned and zero collisions |
| N-agent scaling | 1/2/4/8-agent fleets: 200/200 successful and collision-free missions; mean duration 121.52/72.10/65.36/62.04 steps |
| Two-agent search | 50/50 successful missions, 100% Survivor Recall, both drones returned, zero wall/drone collisions; 40.67% shorter mean duration than one drone |
| Failure reassignment | 50/50 injected failures recovered, 50/50 released tasks reassigned, 100% Recall, zero collisions |
| Constrained communication | 100% base-known Recall and mission success for Relay-off and Adaptive Relay, zero collisions and Final-Sync timeouts |
| Visual perception under moderate Smoke | Recall falls from 100% to 79.33%; navigation and collision metrics remain unchanged |
| Isolated Thermal perception under moderate Smoke | 96.67% Recall; 22 of 25 incomplete Visual-Smoke seeds recovered, zero collisions |

The headline mission comparison is stored in
[`benchmarks/two_drone_50_seeds.json`](benchmarks/two_drone_50_seeds.json).
Failure, network, Smoke, Thermal, knowledge, deconfliction, and Relay artifacts
are available in [`benchmarks/`](benchmarks/). Experiment-specific analysis is
documented in [`docs/`](docs/).
The N-agent results, variance, efficiency, and per-seed regressions are analyzed
in [`docs/v0.6-n-agent-scaling.md`](docs/v0.6-n-agent-scaling.md).
The confidence model, holdout results, reliability analysis, and failure seeds
are in
[`docs/v0.7-noisy-perception-confidence.md`](docs/v0.7-noisy-perception-confidence.md).

## Architecture

EchoRescue keeps world truth, autonomous decisions, and presentation separated:

```text
Seeded environment
       │
       ▼
Sensors ──► discovered knowledge / occupancy maps
                         │
                         ▼
              frontier allocation ──► A* planning
                         │
                         ▼
       energy + agent state machine + deconfliction shield
                         │
                         ▼
             telemetry ──► versioned replay ──► dashboard

Communication graph ──► map/Survivor sync ──► constrained transport
                     └─► optional Relay roles
Failure events ───────► task release and deterministic reassignment
```

The core components are:

- deterministic environment generation, range sensors, Visual/Thermal Survivor
  observations, and strict line-of-sight rules;
- shared, shadow, and active local knowledge modes with explicit provenance;
- deterministic frontier allocation, A* path planning, energy-aware return, and
  collision prevention;
- communication graphs, constrained store-and-forward transport, Relay
  experiments, and failure reassignment;
- versioned JSON telemetry and a read-only HTML/CSS/Canvas replay dashboard.

Architecture decisions and their trade-offs are recorded in
[`docs/adr/`](docs/adr/). Ground Truth never becomes an implicit planning input,
and ordinary operator replays do not expose unknown walls, unconfirmed Survivor
positions, or the full Smoke field.

## Quick start

Python 3.10 or newer is required. Runtime code has no third-party dependencies.

```bash
python -m pip install -e .
python -m echorescue --drones 4 --seed 44 --replay-out replays/seed_44_4_agents.json
python -m echorescue.dashboard --replay replays/seed_44_4_agents.json
```

Open <http://127.0.0.1:8000>. The dashboard provides timeline playback,
operator/shared/local map views, paths, battery and agent state, communication
links, events, confirmed Survivors, and final mission metrics.

For a headless result only:

```bash
python -m echorescue --drones 8 --seed 7
python -m echorescue --drones 2 --seed 50 --survivor-sensor visual --perception-noise moderate
```

The public Python entry points are also importable:

```python
from echorescue import MultiDroneSimulation, SimulationConfig

result = MultiDroneSimulation(
    SimulationConfig(seed=7, drone_count=4)
).run()
print(result.to_dict())
```

## Demo and deployment readiness

The committed network-aware replay is a high-signal demonstration of mapping,
local knowledge, constrained transport, Relay decisions, and safe return:

```bash
python -m echorescue.dashboard \
  --replay replays/seed_7_network_aware.json \
  --benchmark benchmarks/network_aware_relay_100_seeds.json \
  --host 0.0.0.0 \
  --port 8000
```

`--host` and `--port` make the built-in HTTP server suitable for a small
provider-neutral demonstration environment. It resolves package assets and
repository-relative replay paths at runtime; no local machine path is embedded
in public configuration. External hosting should add TLS, access controls,
resource limits, and production-grade HTTP serving as appropriate. No
provider-specific deployment manifest or account credential is required by the
repository.

For a compact fleet-scaling demo, use `replays/seed_44_4_agents.json` with
`benchmarks/n_agent_scaling_50_seeds.json`; it shows four concurrent agents,
distinct targets, Survivor confirmation, movement interventions, and safe
return.

## Reproducible benchmarks

The main baseline and selected resilience/perception experiments can be rebuilt
with:

```bash
python -m echorescue.benchmark --seeds 50 --output benchmarks/two_drone_50_seeds.json
python -m echorescue.scaling_benchmark --seeds 50 --output benchmarks/n_agent_scaling_50_seeds.json
python -m echorescue.failure_benchmark --seeds 50 --failure-drone drone-2 --failure-step 4 --output benchmarks/failure_reassignment_50_seeds.json
python -m echorescue.network_benchmark --seeds 50 --output benchmarks/constrained_network_50_seeds.json
python -m echorescue.smoke_benchmark --seeds 50 --output benchmarks/smoke_perception_50_seeds.json
python -m echorescue.thermal_benchmark --seeds 50 --output benchmarks/thermal_perception_50_seeds.json
python -m echorescue.noisy_perception_benchmark --output benchmarks/noisy_perception_50_holdout_seeds.json
```

Benchmark modules use identical seed ranges and deterministic repeat checks
where their experiment requires them. Additional commands and interpretation
are documented alongside the corresponding artifacts and in
[`docs/phase-4-closeout.md`](docs/phase-4-closeout.md),
[`docs/phase-5-smoke-baseline.md`](docs/phase-5-smoke-baseline.md), and
[`docs/phase-5-thermal-baseline.md`](docs/phase-5-thermal-baseline.md).

## Tests and quality checks

Install the pinned development checker and run the same core checks as CI:

```bash
python -m pip install -e ".[dev]"
python -m unittest discover -s tests -v
python -m mypy src
python -m compileall -q src tests
```

The mypy policy checks all production simulation, planning, mapping,
coordination, sensing, networking, replay, dashboard, and CLI modules. Legacy
benchmark JSON aggregation modules are a documented incremental exception; see
[`docs/type-checking.md`](docs/type-checking.md).

## Limitations

- EchoRescue is a software simulation, not evidence of real-world flight safety.
- The world is a static two-dimensional grid; there are no dynamic obstacles,
  floors, vehicle dynamics, aerodynamics, or flight-controller integration.
- Sensors are abstract range/LOS/probability models. Thermal is not an infrared
  camera, and there is no Visual/Thermal fusion, acoustic sensing, ML, or CV.
- The radio model uses grid LOS and deterministic transport abstractions, not
  measured RF propagation, interference, or a complete routing protocol.
- The mission runtime is validated for 1–8 agents. Adaptive and Network-aware
  Relay roles remain intentionally limited to two-agent experiments.
- Failure injection is deterministic fail-stop behavior without diagnosis,
  repair, or probabilistic component reliability.
- There is no ROS 2, hardware-in-the-loop, real sensor dataset, or hardware
  validation.
- The built-in dashboard server is intended for development and demonstrations,
  not as a hardened public production server.

## Roadmap

Implemented experimental capabilities such as Smoke, Thermal sensing, Relay,
and failure reassignment remain available. The roadmap below defines future
engineering priority rather than the historical order in which experiments
were added.

- **v0.5.x — Portfolio Hardening:** landing page, ADRs, typing, CI, and demo
  readiness.
- **v0.6 — N-Agent Generalization:** one deterministic core validated for
  1/2/4/8 agents, with scaling statistics and fleet-safe replay/dashboard data.
- **v0.7 — Noisy Perception & Confidence:** generalized uncertain observations
  and confidence semantics.
- **v0.8 — Dynamic Obstacles / Dynamic Replanning:** changing traversability and
  replanning behavior.
- **v0.9 — Generalized Roles + Failure Reassignment:** fleet-scale role and task
  recovery policies.
- **v0.10 — Predictive Communication / Multi-Relay:** route-quality prediction
  and multi-hop Relay coordination.
- **v0.11 — Multi-floor / 2.5D:** connected floor plans and vertical transitions.

Development stops at completed v0.6 in this slice. v0.7 begins only as a
separate task.

## License

EchoRescue is available under the [MIT License](LICENSE).
