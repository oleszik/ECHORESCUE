# ADR 0024: Sensor-owned autonomous frontier exploration

- Status: Accepted
- Milestone: v0.16.0

## Decision

The production explorer starts with an entirely `UNKNOWN` bounded ENU grid. It
may learn `FREE` cells only from validated LiDAR ray traversal and `OCCUPIED`
cells only from valid hit endpoints associated with a fresh pose in the same
MAVLink session. Occupancy is monotone. Planning uses a separately inflated map
and the existing cardinal A* implementation; `UNKNOWN` is never traversable.

A frontier is a safe `FREE` cell with a cardinally adjacent `UNKNOWN` cell.
Cardinally connected frontiers form clusters. Within each cluster the
representative is the reachable cell with highest local information gain, then
stable row/column order. Candidates are ordered by
`2 × information_gain − path_cost − target_switch_penalty`, then row/column.
One-metre receding-horizon route prefixes limit exposure to newly discovered
occupancy. A selected frontier remains active until telemetry settles in its
cell or new evidence makes it unreachable.

Completion requires no reachable safe frontier, a quiet map interval and the
configured minimum controller-known ratio. It never uses evaluator coverage.
The final return is generated from the controller-owned map to the recorded
launch cell. All command transitions retain the ACK-plus-later-telemetry
contract, and success still requires `ON_GROUND` plus disarmed telemetry.

Any session change invalidates pose/scan, target and route correlation and
cannot resume exploration. Recovery obtains fresh extended-state evidence in
the new session when needed. LiDAR/telemetry loss and exhausted bounds remain
failed outcomes even after a verified recovery landing.

## Trust boundary and consequences

Gazebo poses, collision geometry, contact topics and evaluator results exist
only in the independent acceptance harness. Configuration supplied to the
controller contains resolution, bounds, sensor contract and policy values but
no wall, doorway, obstacle or route coordinates.

The result is deterministic static, planar, fixed-altitude exploration in one
simulation vehicle. It is not SLAM, loop closure, probabilistic clearing,
dynamic-object tracking, semantic/survivor detection, multi-drone exploration
or hardware safety.
