# EchoRescue

EchoRescue is a deterministic, grid-based search-and-rescue simulation with a
browser replay dashboard. Two drones explore an initially unknown floor, share
an occupancy map, confirm survivors, and return independently to base. Mission
decisions remain headless; the dashboard only renders a versioned replay.

## Mission replay

![EchoRescue dashboard at mission step 42](docs/assets/echorescue-dashboard-step-42.png)

<details>
<summary>Watch the final mission phase through the safe return of both drones</summary>

![EchoRescue deterministic mission replay](docs/assets/echorescue-mission-replay.gif)

</details>

## Quick start

Python 3.10 or newer is required. The runtime and dashboard have no third-party
dependencies.

```bash
python -m pip install -e .
python -m echorescue --drones 2 --seed 7 --replay-out replays/seed_7.json
python -m echorescue --drones 2 --seed 7 --knowledge-mode local --replay-out replays/seed_7_local.json
python -m echorescue.dashboard --replay replays/seed_7.json
```

Open <http://127.0.0.1:8000>. The dashboard provides playback, single-step
navigation, click/drag timeline scrubbing, 0.25× through 8× speed, drone trails,
planned paths, battery and state telemetry, confirmed survivors, event history,
coverage, direct and relay radio links, communication state, final metrics, and
the verified single-/two-drone comparison. The map selector switches between
the shared operator map, both local drone maps, and the base knowledge store.

To reproduce the benchmark artifact:

```bash
python -m echorescue.benchmark --seeds 50 --output benchmarks/two_drone_50_seeds.json
python -m echorescue.communication_benchmark --seeds 50 --output benchmarks/communication_50_seeds.json
python -m echorescue.shadow_benchmark --seeds 50 --output benchmarks/shadow_mode_50_seeds.json
python -m echorescue.knowledge_benchmark --seeds 50 --output benchmarks/knowledge_modes_50_seeds.json
python -m echorescue.deconfliction_benchmark --seeds 50 --output benchmarks/distributed_deconfliction_50_seeds.json
```

The server automatically loads that default benchmark file when it exists. A
different artifact can be selected explicitly:

```bash
python -m echorescue.dashboard --replay replays/seed_7.json --benchmark benchmarks/two_drone_50_seeds.json
```

## Architecture

Simulation and presentation have a one-way boundary:

```text
SimulationConfig + seed
          |
          v
 MultiDroneSimulation  --> JSON result
          |
    read-only observer
          v
 versioned replay JSON --> local HTTP server --> HTML/CSS/Canvas dashboard
```

The browser does not generate maps, plan paths, assign frontiers, detect
survivors, account for energy, or decide movements. Each replay frame is a
snapshot of the simulation's already-computed operator-visible state. It
contains both drone states, batteries, targets and paths; the known occupancy
map; confirmed survivors; coverage; communication graph; and the events emitted
at that step. In the default `shared` and opt-in `shadow` modes, the radio graph
remains observational. Only the explicit `local` mode consumes communication
state for map and survivor synchronization and component-scoped coordination.

Local-map Shadow Mode runs beside that unchanged control path. Sensor readings
enter the observing drone's local knowledge first and are then mirrored into
the legacy shared operator map exactly as before. Connected graph components
exchange local records immediately; connected drones also upload to and receive
from the base knowledge store. None of these shadow stores is read by frontier
allocation, A*, survivor decisions, energy logic, or movement.

Ground truth and discovered knowledge remain separate. Standard replays never
contain the full wall set, unconfirmed survivor positions, or a ground-truth
map. Unknown cells stay unknown until the simulation's sensors map them.
Unconfirmed survivor observations remain per drone. In Active Local mode the
operator-safe survivor layer and top-level mission metrics show only knowledge
that reached the base. Selecting a drone's local map deliberately exposes only
that drone's confirmed knowledge; the global operator map is labeled as an
evaluation aggregate and is never a local planning input.

### Communication telemetry

The base station is a separate graph node at the base cell. At frame zero and
after every simulation step, radio links are recomputed from the configured
Euclidean range and a conservative grid line-of-sight test: an intervening wall
blocks the link. A drone is classified as direct, connected through the other
drone, or disconnected. Transition events are emitted only when that state
changes.

Uptime values are connected frame samples divided by all frame samples,
including frame zero. Relay uptime counts only samples where no direct base link
exists and a peer path does; outage length is the number of consecutive
disconnected samples. The reproducible 50-seed artifact is available at
[`benchmarks/communication_50_seeds.json`](benchmarks/communication_50_seeds.json).

### Local-map Shadow Mode

Knowledge records contain only an observed cell state, observation step, and
source node. Merge order is deterministic and independent of ground truth:
occupied outranks free as the conservative conflict rule; equal states prefer
the newer observation and then the lexicographically stable source ID. There is
no synchronization between disconnected graph components. Transfers are
instantaneous in this slice, with per-step cell counts aggregated into one
upload and one receive event per participating drone.

Coverage is reported for each drone, the base, and the union of both local
maps. Divergence is the fraction of grid cells whose local states differ;
staleness also includes older copies of otherwise equal knowledge.
`map_sync_events` counts transfer rounds, while `time_to_map_convergence` is the
first frame-level reconvergence after a persisted divergence. The 50-seed
artifact is available at
[`benchmarks/shadow_mode_50_seeds.json`](benchmarks/shadow_mode_50_seeds.json).

### Knowledge modes

`shared` is the unchanged default and preserves the verified two-drone control
path. `shadow` simulates distributed maps and radio synchronization as telemetry
without changing decisions. `local` is opt-in: each drone performs frontier
detection, reachability checks, A*, and return-to-base planning against its own
map only. Observations enter only the observing drone's store; cells and
confirmed survivors cross nodes only within the current direct/relay radio
component. Connected drones coordinate distinct targets, while disconnected
drones choose independently and reconcile stale or duplicate goals
deterministically after reconnect.

The global occupancy map remains an evaluation/rendering aggregate in Active
Local mode. It is not passed to frontier allocation, path planning, RTB, or
Survivor decisions. The central movement resolver is retained solely as a
last-resort safety shield against vertex and edge-swap collisions. Every time
it blocks a locally planned movement, the replay records a
`safety_shield_intervention`; ordinary component-aware route planning produces
no such event. This is simulation safety containment, not decentralized proof
of collision avoidance.

Replay schema `1.4` stores the active mode, labels operator, drone-local, and
base knowledge explicitly, and includes the short motion intent/reservation
that was available for distributed deconfliction. Full map snapshots remain
intentionally simple and auditable; future large maps may benefit from delta
encoding.

### Distributed deconfliction

Active Local mode shares position, state, remaining energy, next movement, and
a two- or three-step reservation only inside the current radio component. When
radio is unavailable, a configurable short-range proximity sensor propagates
only through free cells; it can see around an open corner but never through a
wall. Conflict priority is deterministic: urgent RTB, lower safe energy margin,
longer waiting time, then stable drone ID. Repeated blocks trigger a local
deadlock replan. The central movement resolver remains unchanged as the final
shield and is not consulted by normal planning.

The reproducible 50-seed artifact is
[`benchmarks/distributed_deconfliction_50_seeds.json`](benchmarks/distributed_deconfliction_50_seeds.json).
It reports 100% base-known Survivor Recall, zero collisions, safe return of both
drones in every mission, and zero central shield interventions. Mean duration
changes from 75.10 to 75.30 steps (+0.27%); targeted corridor tests cover vertex,
edge-swap, wall-occluded proximity, repeated blocking, deterministic replanning,
and starvation prevention.

## Verified benchmark

[`benchmarks/two_drone_50_seeds.json`](benchmarks/two_drone_50_seeds.json) is
machine-generated from seeds 0–49. Every two-drone seed is executed twice for a
determinism check, alongside the existing single-drone baseline.

| Metric | Single drone | Two drones |
| --- | ---: | ---: |
| Average mission duration | 121.52 steps | 72.10 steps |
| Survivor recall | 100% | 100% |
| Wall collisions | 0 | 0 |
| Drone collisions | n/a | 0 |
| Duplicate exploration | n/a | 12.56% |

The reported **40.67% shorter mission duration** is calculated as
`(121.52 - 72.10) / 121.52 × 100`, rounded to two decimals. All 50 two-drone
missions returned both drones, with no failures or timeouts. The trade-off is
explicit: combined fleet path length averaged 134.64 cells versus 121.52 for
one drone, an increase of about 10.8%.

The Shadow Mode suite also executes seeds 0–49 twice. Local maps diverged by an
average peak of 26.07% of grid cells (maximum 52.75%) while radio links were
unavailable or partitioned. All 50 missions later converged; the base finished
with 96.95% average known coverage (minimum 95.60%). Its complete path-and-core-
metric SHA-256 fingerprint matches the pre-Shadow baseline exactly.

The mode comparison at
[`benchmarks/knowledge_modes_50_seeds.json`](benchmarks/knowledge_modes_50_seeds.json)
runs every seed twice in all three modes at radio range 8. Shared and Shadow
retain the verified fingerprint exactly. Active Local completed all 50 missions
with 100% base-known Survivor Recall, both drones safely returned, zero wall or
drone collisions, and zero timeouts. It averaged 75.30 steps versus 72.10 for
Shared/Shadow, with zero safety-shield interventions, 30 redundant
frontier assignments, an average peak map divergence of 24.29%, and no final
divergence. There were no failed seeds to classify; the measurable cost is
3.20 additional average steps and substantially more local replanning caused by
partial knowledge, disconnection, and deterministic target reconciliation.

## Simulation controls

For a headless JSON summary without a replay:

```bash
python -m echorescue --seed 7 --drones 2
```

The deterministic model exposes sensor, survivor, battery, reserve, wait-cost,
radio-range, knowledge mode, base-store, map-size, obstacle-density and
maximum-step options through `--help`. Use
`--drones 1` for the preserved single-drone regression mode and `--start-mode
shared-base` for two virtual launch slots at the base.

## Verification

```bash
python -m unittest discover -s tests -v
python -m compileall -q src tests
```

The suite covers seeded generation, sensing and occlusion, mapping, A* planning,
survivor, energy, communication, and map-sync events, return safety,
deterministic multi-drone coordination, collision avoidance, replay
determinism, hidden-state protection, event/metric fidelity, dashboard assets,
and a local HTTP smoke test.

## Current limitations

- one static 2D floor and at most two drones
- cardinal, noise-free sensing and abstract deterministic energy units
- replay schema compatibility is version-checked but has no migration layer
- full occupancy snapshots favor transparency over file-size efficiency
- the local server is intended for development and portfolio demos, not public
  production hosting
- the dashboard is desktop-first; it remains usable at narrow widths but has no
  touch-specific gestures beyond the native range control
- communication and map synchronization are instantaneous and lossless; there
  is no delay, bandwidth limit, packet loss, relay role, or behavior adaptation
- Active Local mode is opt-in and has only a central simulator safety shield;
  it does not claim fully decentralized collision avoidance
- no roles, injected failures, dynamic obstacles, ROS 2, hardware integration,
  or 3D visualization

This is a software simulation and not evidence of real-world flight safety.
