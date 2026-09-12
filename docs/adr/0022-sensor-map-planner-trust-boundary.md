# ADR 0022: Sensor-to-map-to-planner trust boundary

- Status: Accepted
- Milestone: v0.15.1

## Decision

Flight planning consumes only the repository prior map, typed MAVLink-derived
vehicle telemetry and a typed range observation. A Gazebo-only adapter may read
the modeled lidar topic, but it publishes no pose, collision, world geometry or
contact data. Every scan carries its sensor frame, simulator timestamp, local
receipt time, telemetry session and strict sequence number.

The ROS-independent mapper accepts a scan only with a fresh, same-session pose.
It rejects stale, cross-session, duplicate, out-of-order, non-finite and
out-of-range data without changing occupancy. Valid finite hit endpoints are
converted from the horizontal sensor frame through telemetry heading into
launch-relative ENU grid cells. No-return beams are represented by the declared
maximum range and do not create occupancy.

New cells are added monotonically and inflated with the existing 0.35 m vehicle
radius plus 0.20 m safety margin contract. A route is replaced only when new
inflated occupancy intersects its remaining cells. Replacement uses
`echorescue.planning.astar` directly, is limited to the configured attempt count
and monotonic-time budget, and replaces the active target before another
setpoint can be issued. No route triggers the existing bounded LAND recovery.

Gazebo pose and contact topics remain exclusive inputs to the independent
evaluator. They prove trajectory, clearance and collision outcomes but cannot
affect mapping, planning or control.

## Consequences

This is deterministic single-drone, static-obstacle, horizontal range mapping;
it is not SLAM or dynamic tracking. A single endpoint cell is a conservative
occupied observation without probabilistic clearing. Simulator evidence does
not imply real-aircraft safety.
