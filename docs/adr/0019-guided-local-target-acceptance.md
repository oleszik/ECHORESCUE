# ADR 0019: Telemetry-gated GUIDED local-target acceptance

- Status: Accepted
- Date: 2026-09-10
- Milestone: v0.14.4

## Context

v0.14.3 safely gates `MAV_CMD` requests with both `COMMAND_ACK` and later
vehicle telemetry. v0.14.4 must command local continuous positions without
claiming an acknowledgement that MAVLink does not provide. ArduCopter supports
`SET_POSITION_TARGET_LOCAL_NED` in GUIDED mode for position, velocity,
acceleration and yaw fields. MAVLink defines it as message 84, while
`COMMAND_ACK` acknowledges a `MAV_CMD` identifier through the command
microservice.

References:

- [ArduPilot Copter commands in Guided mode](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)
- [MAVLink common message set](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)
- [MAVLink command protocol](https://mavlink.io/en/services/command.html)

## Decision

The v0.14.4 command process sends absolute `MAV_FRAME_LOCAL_NED` position
setpoints with the ArduPilot position-only mask `0x0DF8`. EchoRescue owns ENU
targets; the boundary reuses the v0.14.2 mapping:

```text
ENU (east, north, up) -> NED (north, east, -up)
ENU heading CCW from East -> NED yaw clockwise from North
```

Optional heading clears the yaw-ignore bit while retaining all three position
axes. The sender-time field is zero because ArduCopter's absolute position
handling does not require a synchronized sender boot clock; acceptance never
depends on that field.

Mode, arm, takeoff and LAND remain `COMMAND_LONG` operations. Each needs an
accepted ACK from the active session and a later independently received state
transition. A local setpoint is explicitly not ACK-gated. Its acceptance chain
is:

1. validate the immutable target ID and target against the launch-relative ENU
   geofence;
2. require fresh local state in the active MAVLink session;
3. transmit exactly once and record local send success, receipt time and the
   latest vehicle boot time;
4. reject pre-transmission, wrong-session, duplicate or regressing telemetry;
5. require a bounded measurable decrease in total 3D target error unless the
   vehicle is already within both arrival tolerances;
6. require horizontal and vertical arrival together, followed by continuous
   residence for the configured settling interval.

One target is authoritative at a time. A reconnect invalidates its correlation
and the old route is never resumed while airborne. Normal mission failure while
potentially airborne enters the v0.14.3 bounded recovery sequencer, which may
send at most one ACK-and-telemetry-gated LAND per session. Final completion
still requires `ON_GROUND` and disarmed telemetry.

## Consequences

- Reports distinguish command ACK evidence from setpoint telemetry evidence.
- The independent observer can reproduce ordered arrival and settling from ROS
  vehicle state plus typed target/event topics.
- Navigation cannot consume simulator Ground Truth and the legacy bridge stays
  command-free.
- The interface is intentionally limited to three configured local targets. It
  is not a mission-upload protocol, planner, RC interface or arbitrary MAVLink
  command proxy.
- Local coordinates retain ArduPilot's startup EKF-origin limitations; no GPS
  projection, transform publication or real-vehicle support is implied.
