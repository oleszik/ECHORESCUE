# ADR 0015: External 3D simulator integration boundary

- Status: Accepted
- Date: 2026-09-07
- Milestone: v0.14.0

## Context

v0.13 separates the ROS 2 mission process from a deterministic grid-simulator
process. v0.14.x will eventually connect EchoRescue to continuous vehicle
dynamics, but the first milestone must establish a reproducible external stack
without introducing flight commands or weakening deterministic acceptance.

## Decision

The v0.14.0 baseline uses Ubuntu 24.04 x86-64, ROS 2 Jazzy, Gazebo Harmonic
(gz-sim 8), ArduPilot Copter 4.6.3 and a pinned official
`ardupilot_gazebo` revision. The official Iris runway example connects Gazebo
to SITL through the plugin's JSON interface. This path does not require ROS at
runtime; ROS readiness is diagnosed independently for later EchoRescue
adapters.

Repository code owns configuration, diagnostics, lifecycle supervision and
health interpretation only. Gazebo owns continuous world and model state;
ArduPilot owns flight-controller state. v0.14.0 issues no flight command.
MAVLink endpoints and ENU/NED/FRD frame names are configuration, while frame
conversion remains explicitly unimplemented.

A smoke result passes only after real Gazebo and SITL processes start, the Iris
world appears, and at least two distinct MAVLink position messages arrive.
Every exit path terminates owned process groups and reports cleanup status.

## Determinism boundary

The grid simulator retains exact seeded fingerprints and byte-level replay
gates. Gazebo physics and ArduPilot telemetry are not expected to be
bit-identical across machines. Future 3D gates will use identical inputs,
event-level invariants, numeric tolerances and repeated-run distributions.
Grid and physics results will be reported separately; adding physics cannot
relax any deterministic grid gate.

## Consequences

Ubuntu 26.04 / ROS 2 Lyrical is not the supported Harmonic binary baseline.
Gazebo Harmonic officially targets Ubuntu 22.04 and 24.04, and the recommended
ROS pairing is Jazzy on 24.04. A separate Ubuntu 24.04 environment is therefore
required on the audited host. Lyrical remains useful for the existing v0.13
workspace but does not satisfy this milestone's external-stack preflight.
