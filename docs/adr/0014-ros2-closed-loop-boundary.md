# ADR 0014: ROS-independent closed-loop core behind typed ROS 2 adapters

- Status: Accepted
- Date: 2026-09-06

## Context

EchoRescue's simulator previously called mission logic in one Python process.
v0.13 must demonstrate the same observation-driven mapping and planning policy
across a real ROS 2 process boundary without making ROS a dependency of the
normal package.  Ground Truth must remain confined to the simulator, while
commands need acknowledgement, expiry, deduplication and fail-closed behavior.

## Decision

The canonical bridge domain types and both state machines are ordinary Python
modules.  They do not import `rclpy` or generated messages.  A separate
`ros2_ws` contains:

- `echorescue_interfaces`, with typed messages and the `MoveGrid` action;
- `echorescue_ros`, with one mission node and one simulator node; and
- a launch file which starts those nodes as separate processes.

The simulator process alone owns `GridWorld`.  It publishes bounded sensor
observations and actual state.  The mission process owns only its probabilistic
knowledge map, Survivor hypotheses and planner.  It treats a move as committed
only after both a completed action result and a matching, newer state report.

The action's `(session_id, command_id)` is the execution idempotency key.  The
simulator caches terminal results, rejects stale preconditions and expired or
foreign-session goals, and uses a monotonic heartbeat lease to stop accepting
commands if the mission process disappears.  DDS data uses reliable, volatile,
keep-last QoS: reliability is useful inside this small local demo, while
volatile durability prevents a restarted session from inheriting retained
sensor or state samples.

Grid steps are atomic.  Cancellation is honored before execution starts; a
step which has started is completed.  This is not a continuous emergency-stop
claim.  An execution-side occupancy check is a simulator Safety Shield and its
interventions are telemetry, not evidence of physical collision safety.

## Consequences

The existing package remains installable and testable without ROS 2.  A direct
in-process transport can exercise the identical mission/backend classes as a
deterministic reference.  ROS-specific generation and tests require ROS 2
Lyrical in the separate workspace.  This milestone deliberately remains one
agent, one floor and discrete motion; it adds no MAVLink, autopilot, physics or
hardware integration.
