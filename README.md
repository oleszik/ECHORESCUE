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
python -m echorescue --drones 2 --seed 7 --knowledge-mode local --relay-strategy adaptive --replay-out replays/seed_7_relay.json
python -m echorescue --drones 2 --seed 7 --knowledge-mode local --network-profile constrained --replay-out replays/seed_7_constrained.json
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
python -m echorescue.relay_benchmark --seeds 50 --output benchmarks/adaptive_relay_50_seeds.json
python -m echorescue.network_benchmark --seeds 50 --output benchmarks/constrained_network_50_seeds.json
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

Replay schema `1.5` stores the active mode and Relay strategy, labels operator,
drone-local, and base knowledge explicitly, and includes the short motion
intent/reservation that was available for distributed deconfliction. Relay
frames also expose the locally selected waypoint, designated Scout, held/active
state, and achieved communication chain. Full map snapshots remain
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

### Adaptive Relay role

`--relay-strategy adaptive` is available only with Active Local Knowledge and
is deliberately opt-in; `off` remains the default. After a sustained outage,
one drone may temporarily enter `RELAY` when the disconnected peer has
unacknowledged map or Survivor knowledge. Candidate waypoints are selected and
replanned exclusively from the Relay drone's local free cells. Each plan must
predict local line of sight to both base and the last communicated Scout
position, remain locally reachable, and preserve the configured energy reserve
plus Relay margin. A bounded role duration and deployment count prevent Relay
starvation. RTB priority and distributed deconfliction remain in force.

The versioned comparison at
[`benchmarks/adaptive_relay_50_seeds.json`](benchmarks/adaptive_relay_50_seeds.json)
runs seeds 0–49 at radio range 8, repeats every adaptive run, and verifies the
`off` behavior fingerprint. Adaptive Relay completed 25/25 deployments and
forwarded 4,353 unique per-mission cell positions plus 44 Survivor
confirmations. Average communication uptime improved from 32.94% to 34.33%
(+1.40 percentage points), and mean time to first base-known Survivor improved
slightly from 21.44 to 21.24 steps. The explicit cost was 75.30 to 77.92 mean
mission steps (+3.48%), 1.83% more combined path, and 1.99% more fleet energy.
Both variants retained 100% Survivor Recall, safe return in all 50 missions,
zero collisions, zero timeouts, and zero central Safety-Shield interventions.
This measurable communication gain passes the benchmark acceptance rule but is
not sufficient reason to change the default strategy automatically.

### Constrained network transport

`--network-profile ideal` remains the default and preserves instantaneous
knowledge exchange and all verified fingerprints. The opt-in `constrained`
profile is available only with Active Local Knowledge. It separates physical
radio reachability from successful data delivery through the standalone
`network_transport` module. The documented moderate defaults are one-step
link latency, 5% deterministic packet loss, 36 payload units per physical link
and step, fragments of at most 12 units, and age-based queue fairness every 8
steps. CLI flags expose latency, loss, capacity, fragment size, knowledge TTLs,
fairness age, backlog warning threshold, and the bounded Final-Sync budget.

Safety-critical motion intent and drone/RTB state precede Survivor confirmation,
Survivor detection, map data, and decision-free telemetry. Loss is derived from
the mission seed, profile, directed hop, stable fragment ID, retry attempt, and
send step; it never consumes a mutable random stream. A Relay route is genuine
store-and-forward transport: Scout-to-Relay delivery only queues the second
Relay-to-Base hop. Replay schema 1.7 exposes physical links, successful transfer
links, queue/backlog state, loss, expiry, and Relay forwarding without leaking
ground truth. Older schema 1.5 and 1.6 replays remain supported.

Transport quality is reported with three explicit denominators. Fragment-attempt
delivery counts every link-hop attempt and every retry. Unique-fragment eventual
delivery counts each created end fragment once. Logical-message completion counts
only messages whose complete fragment set reached the recipient. Packet losses,
TTL expiry, and fragments discarded at mission close are separate counters; no
missing value is replaced by zero.

When both drones have landed but locally confirmed Survivor knowledge is still
missing at the base, the constrained profile enters a bounded `FINAL_SYNC` data
drain. It uses the normal latency, capacity, deterministic loss, retransmission,
TTL, and routing machinery—there is no queue flush and no Ground-Truth fallback.
Landed radios are explicitly abstracted as base-powered, so flight-battery values
do not change. Once all confirmed Survivor data has arrived, remaining map traffic
may be discarded; if the configured `--final-sync-max-steps` budget expires first,
the mission remains failed with `final_sync_timeout`.

Queued messages whose next hop becomes invalid are reconsidered deterministically
against the currently observed communication graph. This prevents a stale fixed
route from indefinitely blocking safety or Survivor traffic. Predictive routing,
route-quality optimization, and a complete dynamic-routing protocol remain future
work.

The reproducible 50-seed comparison is stored at
[`benchmarks/constrained_network_50_seeds.json`](benchmarks/constrained_network_50_seeds.json).
Every seed is executed twice. Ideal Relay-off retains fingerprint
`db80668469f645f5133b2c5bc53bfbeeefe91108d9e0103dc8a6b8369761b5bb` and
averages 75.30 steps with 100% base-known Survivor Recall. Hardened constrained
Relay-off averages 152.64 steps; Adaptive Relay averages 160.26. Both now reach
100% mission success and base Recall, return both drones in every mission, and
have zero wall/drone collisions, timeouts, Final-Sync timeouts, and central
Safety-Shield interventions. Three missions per constrained profile enter Final
Sync. Relay-off averages 4.00 Final-Sync steps (maximum 7), while Adaptive Relay
averages 17.33 (maximum 33).

The aggregate Relay-off ratios are 95.03% successful fragment attempts, 64.01%
eventual unique-fragment delivery, and 65.11% logical-message completion.
Adaptive Relay reaches 95.01%, 62.94%, and 63.03%. The apparent gap is not
unexplained packet loss: low-priority map snapshots are intentionally discarded
after critical Survivor delivery and mission close, while stale traffic also
expires by TTL. Exact loss, TTL, close-drop, retransmission, and before/after
Shield classifications are versioned in the artifact. The old 46 Relay-off and
51 Adaptive Shield interventions reproduce as delayed/lost Intent cases; the
hardened variants reduce every category to zero. Adaptive Relay remains slower
and has lower completion ratios, so it is still not accepted as an improvement.

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
  is no delay, bandwidth limit, packet loss, or multi-hop role negotiation
- Adaptive Relay uses only two drones, freezes the designated Scout briefly,
  and optimizes a deterministic local benefit heuristic rather than a learned
  or globally optimal policy
- Active Local mode is opt-in and retains a central simulator Safety Shield as
  a final fail-safe; it does not claim real-world decentralized flight safety
- no persistent role hierarchy, injected failures, dynamic obstacles, ROS 2,
  hardware integration, or 3D visualization

This is a software simulation and not evidence of real-world flight safety.
