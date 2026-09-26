# EchoRescue Repository Instructions

EchoRescue is a civilian autonomous search-and-rescue robotics research
and portfolio project.

## Engineering principles

- Preserve deterministic and reproducible behavior wherever possible.
- Safety takes priority over mission completion and exploration performance.
- Never weaken a safety condition merely to make an acceptance test pass.
- Never special-case a benchmark, seed, map, survivor position, or acceptance scenario.
- Do not silently increase retry, replan, timeout, or recovery budgets to hide failures.
- Prefer root-cause fixes over symptom suppression.
- Keep modules typed, testable, and clearly separated.
- Preserve existing behavior unless a change intentionally modifies it.
- Add or update tests when behavior changes.
- Do not introduce unnecessary dependencies.

## Ground truth boundary

Gazebo/world ground truth is evaluation-only.

Production controllers, planners, mapping, exploration, perception, survivor
detection, and recovery logic must never consume privileged Gazebo ground-truth
state unless the component is explicitly an evaluator or test oracle.

Do not derive controller-visible information indirectly from ground truth.

## ROS 2 / Gazebo

- Target ROS 2 Jazzy.
- Keep simulation, controller, observer, and evaluator responsibilities separated.
- Production perception must consume realistic sensor outputs.
- Ensure launched processes terminate cleanly.
- Verify relevant network ports/resources are released after integration runs.

## Validation

Before declaring implementation work complete:

1. Run relevant focused tests while developing.
2. Run regression tests for affected existing functionality.
3. Run static/type checks required by the repository.
4. Run ROS/colcon tests when ROS code changes.
5. Run the appropriate integration/acceptance scenario.
6. Inspect generated evidence rather than assuming success from process exit.
7. Report failures truthfully.

A recovery LAND can demonstrate safe failure handling while the overall mission
still remains FAIL.

Never describe an unvalidated run as successful.

## Git workflow

- Inspect the current branch and working tree before modifying files.
- Do not discard unrelated user changes.
- Keep changes scoped to the requested version/task.
- Do not begin a later version while the current requested acceptance gate is open.
- Do not commit, push, merge, or tag unless the task explicitly authorizes it.
- Never rewrite history or force-push unless explicitly instructed.

## Evidence and claims

Only publish benchmark or mission claims backed by reproducible evidence.

Preserve useful failure evidence when investigating regressions.

Distinguish clearly between:
- implemented,
- unit tested,
- integration tested,
- acceptance validated,
- experimental,
- planned.

## Hardware direction

The long-term physical target is a 3–3.5 inch micro-UAV.

Do not make architectural decisions that require desktop-class compute onboard.

Heavy perception may initially run on a ground station while onboard systems
retain safety-critical local behavior.

## Project scope

EchoRescue is for civilian search-and-rescue research.

Do not add weapon, attack, targeting, pursuit, or harmful capabilities.
