# ADR 0010: Generic multi-Relay task topology

- Status: Accepted
- Date: 2026-09-04

## Context

The communication graph and constrained store-and-forward transport already
supported arbitrary agents and deterministic multi-hop routes. The adaptive
Relay policies above that graph nevertheless assumed exactly two drones, one
Scout, one Relay, one holding point, and a fixed two-hop route. Extending those
branches for every fleet size would duplicate routing logic and preserve the
wrong 1:1 relationship between a Relay and a Scout.

## Decision

Represent active Relay work as a collection keyed by agent ID. Each
`RelayDeployment` records its target, served agents, optional upstream and
downstream Relay, task ID, activation reason, release state, and payload
obligation. A bounded deterministic planner may select one or two targets, but
execution, telemetry, task ownership, and routing iterate over collections and
contain no relay-count branches.

The existing communication graph remains authoritative. Operational agents and
the base are graph nodes, valid range/LOS links are edges, and deterministic BFS
selects shortest paths. A Relay may serve multiple agents whenever the graph
permits it. `RELAY_POSITIONING` remains an ordinary owned task, so failure uses
the v0.9 orphan/reassignment lifecycle.

Activation requires a measured connectivity need plus relevant unsynchronized
data. Deactivation requires payload delivery and stable connectivity after a
minimum hold, or a bounded maximum role duration. Energy, A* reachability,
deconfliction, the safety shield, and RTB are unchanged.

## Consequences

- 4- and 8-agent fleets can use concurrent Relay agents and three-hop
  Base–Relay–Relay–Scout paths.
- Shared Relays are natural graph topology rather than special 1:1 bindings.
- Candidate search is intentionally bounded to at most two active Relays; this
  is not a general mesh optimizer.
- Legacy adaptive/network-aware strategies retain their two-agent validation
  and replay behavior for compatibility.
