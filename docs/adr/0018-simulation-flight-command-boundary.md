# ADR 0018: Simulation flight command boundary

- Status: Accepted
- Milestone: v0.14.3

## Context

v0.14.2 ends at a receive-only continuous ENU state. The next milestone needs
one real closed-loop flight without turning the telemetry adapter into a
general navigation API or allowing simulator Ground Truth into control.

## Decision

Keep `/echorescue_mavlink_telemetry_bridge` receive-only. Add a separate
`/echorescue_mavlink_flight_mission` executable that reuses the same parsing,
health, frame-conversion, typed-topic and single-connection implementation.
Only one telemetry-setup command and four flight `COMMAND_LONG` operations are
permitted:

- `MAV_CMD_SET_MESSAGE_INTERVAL` for `EXTENDED_SYS_STATE`;
- `MAV_CMD_DO_SET_MODE` for Copter `GUIDED`;
- `MAV_CMD_COMPONENT_ARM_DISARM` with arm set to true;
- `MAV_CMD_NAV_TAKEOFF` with the configured local ENU-up altitude;
- `MAV_CMD_NAV_LAND` at the current location.

Each command has two ordered gates: an accepted `COMMAND_ACK`, and a later
telemetry observation appropriate to the operation. Those observations are,
respectively, an extended-state sample, a GUIDED heartbeat, an armed heartbeat,
local position reaching the ENU altitude band, and both
`EXTENDED_SYS_STATE.ON_GROUND` plus a disarmed heartbeat. No ACK mutates
observed state.

The state machine aborts on stale/disconnected telemetry, command rejection,
transition timeout, unexpected session replacement, or interruption. If the
vehicle is armed or an arm outcome is uncertain, it waits for fresh telemetry
and uses the same ACK-and-state-gated LAND path for bounded recovery. A report
records command issue, ACK, and telemetry-transition times.

The controller never issues LAND twice in one MAVLink session. If telemetry
becomes stale after LAND is sent, bounded recovery continues against that
outstanding command; only a newly observed connection session may receive a
new recovery LAND. This prevents a delayed ACK from an older command from
satisfying a newer command with the same MAVLink command ID.

An independent ROS process observes only typed topics and must separately see
GUIDED, armed, target altitude, the hover interval, LAND, landed and disarmed
states within one MAVLink session. Gazebo world/model/pose data is absent from
both controller and observer.

## Consequences

The receive-only executable remains suitable for passive integrations, while
the command authority is conspicuous and opt-in. The runner supports one SITL
vehicle and one preconfigured vertical mission only. It is not a reusable
navigation controller and exposes no waypoint, velocity, position-target, RC,
servo, obstacle, swarm, indoor-world, HIL, or real-vehicle behavior.
