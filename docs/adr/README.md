# Architecture decision records

EchoRescue records durable architecture choices as short ADRs. An accepted ADR
describes the current implementation; later changes should supersede an ADR
rather than silently rewriting its decision.

| ADR | Status | Decision |
| --- | --- | --- |
| [0001](0001-deterministic-simulation.md) | Accepted | Seeded and hash-derived deterministic simulation |
| [0002](0002-deterministic-coordination-resolution.md) | Accepted | Stable conflict and assignment resolution |
| [0003](0003-ground-truth-and-agent-knowledge.md) | Accepted | Separation of Ground Truth from agent/operator knowledge |
| [0004](0004-headless-core-and-replay-dashboard.md) | Accepted | Headless simulation with read-only visualization |
| [0005](0005-versioned-replay-interface.md) | Accepted | Replay JSON as a versioned compatibility boundary |
| [0006](0006-generic-n-agent-coordination.md) | Accepted | One deterministic collection-oriented core for 1–8 agents |
| [0007](0007-evidence-based-survivor-hypotheses.md) | Accepted | Confidence-weighted Survivor hypotheses without Ground-Truth leakage |
| [0008](0008-mutable-environment-observation-driven-replanning.md) | Accepted | Mutable Ground Truth with observation-driven path invalidation and replanning |
| [0009](0009-role-task-ownership-recovery.md) | Accepted | Deterministic N-agent role and single-owner task recovery semantics |
| [0010](0010-generic-multi-relay-topology.md) | Accepted | Collection-oriented Relay deployments over the existing communication graph |
| [0011](0011-planned-path-connectivity-forecast.md) | Accepted | Known-map planned-path forecasts for near-term communication loss |
| [0012](0012-discrete-multi-floor-topology.md) | Accepted | Discrete floor graphs and explicit vertical transitions instead of continuous 3D |
| [0013](0013-observation-union-probabilistic-mapping.md) | Accepted | Bounded observation-union occupancy, decay and uncertainty planning |
| [0014](0014-ros2-closed-loop-boundary.md) | Accepted | ROS-independent closed-loop core behind typed ROS 2 adapters |
| [0015](0015-external-3d-simulator-boundary.md) | Accepted | Pinned external 3D stack behind diagnostic and process-lifecycle gates |
| [0016](0016-receive-only-mavlink-telemetry.md) | Accepted | Receive-only native-frame MAVLink telemetry behind typed ROS 2 adapters |
| [0017](0017-continuous-3d-coordinate-frame-boundary.md) | Accepted | Explicit NED/FRD to ENU/FLU continuous-state boundary |
| [0018](0018-simulation-flight-command-boundary.md) | Accepted | ACK-and-telemetry-gated single-vehicle simulation flight command boundary |
