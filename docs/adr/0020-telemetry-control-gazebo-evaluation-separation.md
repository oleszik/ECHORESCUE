# ADR 0020: Separate telemetry control from Gazebo safety evaluation

Status: Accepted

## Context

v0.14.5 needs simulator pose and contact evidence to assess a predefined indoor
flight, but allowing that privileged state into the mission would turn an
environment validation milestone into ground-truth navigation.

## Decision

The existing v0.14.4 mission controller remains the only navigation
implementation. Its ROS adapter consumes MAVLink-derived state and configured
launch-relative ENU targets and emits only the existing GUIDED flight commands
and local-NED position targets. It has no Gazebo dependency.

A separate process observes the versioned world's pose and per-solid contact
topics. It records world-ENU trajectory, conservative clearance and contact
evidence. Its output can fail and interrupt the integration run, causing the
mission process to use its existing bounded LAND recovery, but it cannot send
targets, advance mission state or generate a maneuver. The aggregate report
keeps telemetry mission evidence and Gazebo evaluation evidence separate.

Static dependency tests enforce both directions of this boundary. Absence of a
contact message is insufficient: every configured solid must expose its
contact topic before an accepted run can report zero prohibited contacts.

## Consequences

Gazebo pose is evaluation-only and is not an autonomous localization source.
Clearance uses sampled model origin and configured axis-aligned geometry with
a 0.35 m conservative spherical vehicle radius; it is not a swept-volume or
real-aircraft safety proof. A detected collision fails the route and requests
bounded landing rather than avoidance.
