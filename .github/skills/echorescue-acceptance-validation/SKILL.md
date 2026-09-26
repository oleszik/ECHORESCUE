---
name: echorescue-acceptance-validation
description: Validate EchoRescue changes and releases through layered tests, ROS 2/Gazebo integration checks, regression missions, safety/recovery scenarios, end-to-end acceptance runs, artifact inspection, and cleanup verification. Use when asked to validate, verify, accept, release, regression-test, or prove an EchoRescue implementation or version.
---

# EchoRescue Acceptance Validation

Use this skill to determine whether an EchoRescue change, milestone, or release
is actually acceptance-ready.

The purpose is not to make tests pass at any cost. The purpose is to collect
reproducible evidence that the requested implementation works without
regressing existing safety or functionality.

Repository instructions and the requirements of the current task take
precedence over generic examples in this skill.

## Core rule

A version is accepted only when every required acceptance gate passes.

A successful build is not sufficient.
Passing unit tests is not sufficient.
A safe recovery LAND after a mission failure is not mission success.

Never convert BLOCKED or FAIL into PASS by weakening acceptance criteria.

## 1. Establish validation scope

Before running tests:

1. Inspect the current Git branch.
2. Inspect `git status`.
3. Inspect the current diff when changes are present.
4. Read repository instructions and relevant documentation.
5. Identify the version/change being validated.
6. Identify its explicit acceptance criteria.
7. Identify the previous stable version that must remain functional.
8. Locate relevant scripts, tests, configs, reports, and artifacts.

Do not assume test counts, benchmark values, paths, launch commands, ports,
seeds, or acceptance thresholds from old reports.

Discover them from the current repository.

Record the working-tree state before validation.

## 2. Focused validation

Start with tests directly related to the modified subsystem.

Examples include:

- mapping;
- frontier exploration;
- A* planning;
- return routing;
- survivor detection;
- perception;
- ROS message handling;
- recovery;
- process cleanup.

Run focused tests before expensive end-to-end missions.

If focused tests fail, investigate the root cause before continuing.

## 3. Python regression suite

Run the repository's complete Python test suite using the documented project
command.

Record:

- passed;
- failed;
- skipped;
- errors;
- duration when useful.

Do not hard-code an expected number of tests in this skill.

The expected count evolves with the repository.

Any unexplained failure blocks acceptance.

## 4. Static and type validation

Run the static checks applicable to the current repository.

This may include:

- mypy;
- compileall;
- formatting/lint checks;
- JSON validation;
- XML validation;
- shell syntax validation;
- `git diff --check`.

Use repository-provided commands where available.

Record the result of each executed check.

## 5. ROS 2 validation

When ROS 2 code, configuration, launch files, messages, sensors, or integration
behavior is affected:

1. perform the repository's documented clean ROS 2 build;
2. source the resulting workspace as required;
3. run the applicable colcon/ROS tests;
4. inspect failures rather than relying only on command exit status.

Target the ROS distribution defined by the repository.

Do not claim ROS integration success based solely on compilation.

## 6. Previous-version regression

Run the relevant acceptance/regression scenario from the last stable version
when the current change could affect existing behavior.

Verify the important existing guarantees applicable to that version.

Examples may include:

- autonomous exploration;
- map coverage;
- door traversal;
- collision-free operation;
- safe return;
- LAND;
- ON_GROUND;
- disarm;
- deterministic behavior;
- process cleanup.

Use the actual previous-version acceptance criteria rather than inventing new
ones.

A regression in an existing required behavior blocks acceptance.

## 7. Ground-truth isolation

For production-path integration runs, verify that controller-visible behavior
does not consume privileged Gazebo/world ground truth.

Ground truth may be used by a separate evaluator or test oracle.

If a change introduces direct or indirect ground-truth leakage into production
perception, mapping, planning, exploration, recovery, or control, acceptance
fails even if mission metrics improve.

## 8. Failure and recovery scenarios

Run the failure/recovery scenarios required by the current change.

Depending on the subsystem, these may include:

- sensor dropout;
- camera dropout;
- stale sensor data;
- planning/replanning budget exhaustion;
- unreachable frontier;
- communication loss;
- timeout;
- malformed input.

Verify bounded behavior.

Where applicable, verify:

- controlled transition to recovery;
- LAND;
- ON_GROUND;
- disarm;
- no uncontrolled continued exploration.

Recovery success and mission success must be reported separately.

## 9. End-to-end acceptance mission

Run the exact end-to-end scenario required for the version/change.

If graphical Gazebo validation is required, use an actual graphical Gazebo run.
Do not substitute a headless run.

During the mission, collect the evidence required by the acceptance criteria.

Possible evidence includes:

- mission status;
- controller-known map coverage;
- evaluator coverage;
- survivor confirmations;
- survivor precision;
- survivor recall;
- duplicate detections;
- generated routes;
- replans;
- frontier failures;
- collisions;
- prohibited contacts;
- return distance;
- LAND state;
- ON_GROUND state;
- disarm state;
- mission duration;
- recovery reason.

Only report metrics actually produced by the run.

## 10. Perception-specific validation

When validating survivor detection, vision, depth estimation, or sensor fusion:

Verify that production perception consumes actual configured sensor data.

For survivor detection, check applicable criteria such as:

- required temporal/multi-frame evidence;
- all required survivors confirmed;
- no duplicate survivor confirmations;
- precision;
- recall;
- camera dropout behavior.

For future depth/sensor-fusion work, validate perception independently before
allowing it to influence autonomous control.

Do not use evaluator ground truth as production perception input.

## 11. Cleanup verification

After every bounded integration or acceptance run, verify that the test
environment is clean.

Check for:

- remaining EchoRescue processes;
- Gazebo processes;
- SITL/autopilot processes;
- ROS nodes/processes started by the run;
- occupied network ports used by the scenario;
- orphaned subprocesses.

Use the ports and process expectations defined by the current repository.

Do not hard-code old port numbers unless they remain part of the current
configuration.

Unexpected leftovers block acceptance until understood or cleaned up.

## 12. Artifact inspection

Inspect generated reports, logs, telemetry, and evaluation artifacts.

Do not infer mission success solely from console output.

Confirm that artifacts agree with the reported result.

Preserve useful evidence from failed runs when it helps reproduce or diagnose a
real failure.

Do not replace failed-run evidence with stale successful artifacts.

## 13. Final Git hygiene

Before acceptance:

1. run `git status`;
2. run `git diff --check`;
3. inspect the final diff;
4. check for accidental generated artifacts;
5. check for machine-specific absolute paths;
6. check for credentials or personal data;
7. verify unrelated user changes were not removed.

Do not commit, push, merge, or tag unless the current task explicitly
authorizes it.

If authorization is conditional on acceptance, Git actions must wait until all
acceptance gates pass.

## 14. Decision

Return exactly one overall validation state:

### PASS

Use only when every required acceptance gate passed.

### BLOCKED

Use when the implementation may be substantially correct but at least one
required gate cannot currently be satisfied or a reproducible acceptance
blocker remains.

### FAIL

Use when required behavior is demonstrably incorrect or a required validation
gate fails.

Never use PASS with exceptions such as "PASS except for..."

## 15. Validation report

At the end, report:

### Status

PASS / BLOCKED / FAIL

### Scope

Version, branch, and change validated.

### Automated checks

Commands/check categories and their results.

### Regression

Previous stable behavior and whether it passed.

### End-to-end mission

Relevant measured mission results.

### Safety and recovery

Safety behavior, LAND/ON_GROUND/disarm results, and failure scenarios.

### Ground-truth boundary

Whether production logic remained isolated from evaluator ground truth.

### Cleanup

Whether processes and resources were released.

### Evidence

Important reports/logs/artifacts used to establish the result.

### Remaining blockers

Exact reproducible blockers, if any.

### Git state

Working-tree status and commit hash only if a commit actually exists.

## Example use

A suitable request for this skill is:

"Validate v0.16.1 for release. Run all required tests, the v0.16.0 regression,
the v0.16.1 recovery scenarios, and the final graphical Gazebo acceptance
mission. Inspect the artifacts and only report PASS if every gate succeeds."

Another suitable request is:

"Before committing this planner fix, run the EchoRescue acceptance validation
appropriate to the affected autonomy stack and report any regression."