# ADR 0017: Explicit NED/FRD to ENU/FLU continuous-state boundary

- Status: Accepted
- Date: 2026-09-09
- Milestone: v0.14.2

## Context

v0.14.1 publishes receive-only ArduPilot telemetry in its native local NED
frame. EchoRescue needs a continuous 3D state, but silently relabeling axes,
using GPS as local position, or equating local receipt time with vehicle boot
time would make the state ambiguous. The SITL EKF origin is not proven to be
the same transform as Gazebo's world origin.

## Decision

One ROS-independent boundary converts authoritative `LOCAL_POSITION_NED`
position and velocity into EchoRescue ENU: `x=east`, `y=north`, `z=-down`.
ArduPilot's FRD body convention maps to FLU as `forward=forward`,
`left=-right`, `up=-down`. Full ZYX attitude conversion changes both world and
body bases. Heading changes from degrees clockwise from north to degrees
counter-clockwise from east; public angles are normalized to `[-pi, pi)` or
`[-180, 180)`.

The output preserves MAVLink `time_boot_ms` and the local monotonic receive
timestamp as separate fields. Attitude and heading retain their own source boot
timestamps. Optional attitude, heading, and landed state have explicit validity
flags. A changed MAVLink session clears all joined metadata and source-time
ordering so a reboot cannot be mistaken for monotonic progress.

The local ENU origin policy is named
`ardupilot_local_ned_at_sitl_startup`: `(0,0,0)` is the same startup-local
origin reported by ArduPilot, expressed with ENU axes. GPS is metadata only.
No general geodetic projection is performed.

No TF is published in this milestone. A frame name and coordinate conversion
are valid, but the absolute transform from the ArduPilot EKF origin to Gazebo's
world origin has not been established. Gazebo pose remains available only to
independent validation, never to the runtime bridge or autonomous logic.

## Consequences

- Consumers receive one typed, explicit continuous 3D state without depending
  on ROS in the domain model.
- Native v0.14.1 topics remain unchanged and available for audit/regression.
- Attitude/heading/landed values may be unavailable until their corresponding
  MAVLink messages arrive; zeros never imply validity.
- The result is receive-only telemetry, not localization, sensor fusion,
  navigation, command, or flight-safety evidence.
