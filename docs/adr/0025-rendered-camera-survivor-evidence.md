# ADR 0025: Rendered-camera survivor evidence boundary

- Status: Accepted
- Milestone: v0.16.1

## Decision

Survivor evidence is derived only from rendered Gazebo RGB frames, documented
fixed camera calibration, and fresh same-session MAVLink-derived vehicle poses.
The controlled marker detector thresholds HSV color, applies a bounded 3 px
open/close filter, extracts connected components, and gates area, aspect ratio,
fill and the explicit confidence formula documented in the milestone guide.
This is an explainable simulation marker detector, not human recognition.

Each image candidate is projected onto the configured horizontal marker plane
with a pinhole ray and fixed downward-oblique camera extrinsic. Invalid rays,
stale or cross-session associations, unsupported images and estimates outside
the exploration boundary are rejected with stable reason codes. Confirmation
requires two distinct, increasing frames in one MAVLink session, compatible
locations, adequate mean confidence, and time or viewpoint separation. Stable
mission-local IDs are allocated only on confirmation.

The production path has no survivor count, coordinates, Gazebo model/state,
contact, or evaluator input. Ground-truth matching runs only after flight in the
acceptance harness and never publishes into control. Camera loss invalidates
visual-search acceptance but does not make LiDAR navigation unsafe; flight
continues to its bounded return and telemetry-gated landing.

## Consequences

The result demonstrates controlled static rescue-marker discovery in one
versioned simulated environment. It does not establish general person
detection, arbitrary 3D pose, semantic SLAM, moving-target tracking, thermal
fusion, hardware readiness, or real-aircraft safety.
