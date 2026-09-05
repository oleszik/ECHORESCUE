# ADR 0013: Observation-union probabilistic mapping

Status: accepted for v0.12. Legacy perception remains the default.

## Inventory and implementation plan

The starting branch was `feature/failure-reassignment`, with a clean working
 tree. No AGENTS.md was found in the repository or ancestor directories.
The project brief describes the old MVP; the explicit v0.12 task supersedes it.
Existing v0.11 has N-agent 2D missions, role/failure logic, constrained transport,
and a separate centralized 2.5D runner. Its floor communication graph is
telemetry, not distributed knowledge transport. README roadmap/limits lag code.
Existing occupancy uses deterministic occupied-first merge. Survivor evidence
is an uncalibrated bounded heuristic; confirmation messages transmit sets.

Commit blocks: (1) evidence algebra and sensor profiles; (2) simulation,
transport, planning and floor integration; (3) replay/dashboard and tests;
(4) frozen protocol, benchmark execution and documentation/results.
Risks: duplicated relay evidence, decay breaking RTB connectivity, stale path
execution, old replay compatibility, and misleading claims from reused seeds.

## Decision

Extend CellKnowledge with immutable raw observations. Identity is enclosing
cell plus floor, source agent and original simulation step. Fusion unions these
identities, never sums fused map scores. A conflicting payload with the same
identity is rejected. Within each timestamp evidence is summed before clipping
log odds to +/-4. Chronological updates decay the offset from prior log odds
by 2**(-age/80), then add signed log(reliability/(1-reliability)). Prior is .5;
free <= .35, occupied >= .65, intermediate is uncertain. Unknown has no retained
evidence. These are model probabilities, not empirically calibrated estimates.
ProbabilityConfig exposes the prior, thresholds, bound, half life and retention.

Only observations younger than 640 ticks are retained. Late older messages are
ignored, including after their identity was discarded. At most one observation
per agent/tick/cell gives O(cells * agents * retention) map evidence storage.
At expiry the small remaining contribution is discarded and the cell returns
to prior. Read/receive time never replaces observation time. There is no
obstacle-type-dependent decay. Opposite observations can reopen a cell.

Sensor draws use the existing SHA-256 deterministic helper, with a separate
occupancy namespace and seed/agent/tick/floor/cell keys. Survivor sensing extends
the existing profiles and tracker. Detection is P(report | target present),
false positive is P(report | absent); reliability is independent evidence
strength, not another random dropout. Occupancy false negatives report free;
false positives report occupied. Rays still terminate at the physical first
surface (a sensor-generation use of truth). Temporal and inter-agent independence
is an approximation; spatially correlated occlusion and systematic bias are not
modeled. Relay copies never count as independent measurements.

Survivor evidence remains distinct: positive score increment .5*reliability,
confirmation requires >=2 unique positive observations and score >=.65.
Negative evidence reduces the heuristic score; rejected and confirmed findings
remain latched for the mission, preserving existing semantics. Repeated or older
same-agent/channel observations do not increment it. Confirmation set messages
never increase confidence. This is not a calibrated posterior.

Naive scoring is the existing path-length ordering on the thresholded map.
Uncertainty-aware subtracts the sum of the four adjacent cell binary entropies
(0..4 path-cost units), including unknown cells at the prior. This is a local
entropy proxy, NOT expected sensor information gain. Existing target retention,
coordinate/agent tie breaks, safety, roles and energy checks still apply.

Ground truth is permitted in sensor generation/evaluation. The execution shield
may veto wall entry but must not write a truth-derived cell into probabilistic
mapping. Zero collisions under this shield do not prove robust perception.
The topology/endpoints of stairs remain mission-provided prior knowledge.
