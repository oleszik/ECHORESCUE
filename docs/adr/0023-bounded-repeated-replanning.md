# ADR 0023: Bounded repeated replanning and route generations

- Status: Accepted
- Milestone: v0.15.2

## Decision

The sensor-owned map is monotone within one MAVLink session. Every effective
batch has a strictly increasing map revision and lists newly occupied and newly
inflated cells. Hits already covered by the inflated map are duplicates; an
absent return never clears occupancy. A hit of unknown thickness marks its
endpoint plus one map cell behind the observed surface before applying the
existing vehicle-radius and margin inflation.

A replan is requested only when a sufficiently large effective change
intersects the unexecuted route. Requests are bounded by mission and active
target budgets, a monotonic planning timeout, cooldown, and consecutive-failure
limit. Irrelevant changes and their reasons are retained without consuming a
budget. A relevant change that cannot be safely reconsidered fails closed into
the existing LAND recovery rather than continuing toward the unsafe target.

Each successful replacement increments the route generation. Its target IDs
carry that generation, the previous active target is invalidated atomically,
and only post-transmission telemetry can settle the new target. A reconnect
invalidates sensor ordering and route correlation and uses the existing
fail-closed recovery; the old route is never resumed.

## Consequences

Identical pose, map and observation inputs produce identical cardinal A* paths.
The policy prevents duplicate-trigger storms while permitting multiple genuine
discoveries. It remains static-obstacle, fixed-altitude grid planning—not SLAM,
probabilistic clearing, dynamic tracking or real-aircraft safety logic.
