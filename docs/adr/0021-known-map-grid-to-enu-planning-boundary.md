# ADR 0021: Known-map grid planning before telemetry-only flight

Status: Accepted

## Context

The deterministic EchoRescue planner operates on integer `Position` cells,
while the simulation flight boundary accepts launch-relative ENU targets. The
v0.14.5 route was manually configured and therefore did not prove that the
existing planner could drive the continuous stack.

## Decision

v0.15.0 defines a versioned, fully known 2D occupancy map for the static indoor
world. It calls `echorescue.planning.astar` unchanged for outbound and return
paths. Columns increase east, rows increase north, and cell centres are mapped
from a launch-relative southwest origin at the configured resolution. The
runtime launch telemetry anchors those relative coordinates; `(0,0,0)` is
never assumed to be the ArduPilot local origin.

Occupied grid cells are inflated before planning by the 0.35 m conservative
vehicle radius plus 0.20 m margin and a cell half-diagonal, because an occupied
cell represents its full square. Only the existing four-connected planner is
allowed, so diagonal corner cutting cannot occur. Compaction removes only
collinear cardinal cells and rechecks every covered inflated-map cell.

Planning and all safety validation finish before the flight harness is called.
The v0.14.4 executor receives the generated targets and retains its telemetry,
session, progress, arrival, settling and bounded LAND semantics. Gazebo pose
and contacts remain confined to the independent evaluator and cannot create,
alter or approve a route.

## Consequences

The result is deterministic known-map navigation, not exploration or obstacle
avoidance. A bad map, blocked goal or unsafe passage prevents any flight
process or MAVLink command. No in-flight replanning occurs. The 2D map and one
fixed cruise altitude are specific to this versioned simulation world and do
not establish real-aircraft safety.
