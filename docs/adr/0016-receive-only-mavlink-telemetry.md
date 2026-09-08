# ADR 0016: Receive-only MAVLink telemetry boundary

- Status: Accepted
- Date: 2026-09-08
- Milestone: v0.14.1

## Context

The v0.14.0 stack proves real Gazebo–ArduPilot state exchange and advancing
MAVLink position telemetry. EchoRescue needs a first real ROS 2 boundary, but
the discrete v0.13 `MoveGrid` contract is not a continuous-flight contract and
must not be stretched into one.

## Decision

A repository-owned `echorescue_mavlink_telemetry_bridge` node connects directly
to the configured ArduPilot MAVLink endpoint. It sends only the MAVLink data
stream request needed to enable position telemetry. It never sends arming,
mode, mission, servo, RC, waypoint, takeoff, landing, or other flight commands.

MAVLink parsing, ordering, sessions, freshness, stale transitions, disconnects,
and deterministic serialization live in a ROS- and pymavlink-independent core.
The ROS node adapts accepted immutable values to four versioned custom message
types. Position and velocity remain in MAVLink local NED, while global position
remains WGS84. No ENU conversion is performed.

Status uses four explicit states: `DEGRADED` after heartbeat but before usable
position, `CONNECTED` while advancing position is fresh, `STALE` once its age
exceeds the configured threshold, and `DISCONNECTED` after transport failure or
heartbeat timeout. A reconnect creates a new session and resets source ordering.

## Ground Truth boundary

The bridge reads only MAVLink packets from ArduPilot. It does not subscribe to
Gazebo pose, world, model, or sensor topics. Gazebo Ground Truth may be checked
only by the independent lifecycle/evaluation harness that proves the external
stack is live.

## Consequences

Typed telemetry can now be observed by ROS 2 consumers without granting them a
flight-control path. Best-effort volatile QoS is used for high-rate telemetry;
reliable transient-local QoS exposes the latest health state to late joiners.
Gazebo physics and receipt times are nondeterministic, while core transition and
serialization tests remain deterministic. Command/control and NED-to-ENU
contracts require a later, separate decision.
