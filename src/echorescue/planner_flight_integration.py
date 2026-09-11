"""v0.15.0 known-map planner to v0.14.5 owned flight orchestration."""

from __future__ import annotations

import argparse
import copy
from dataclasses import asdict
import json
from pathlib import Path
import sys
import tempfile
from typing import Any, Mapping, Sequence

from echorescue.indoor_integration import indoor_diagnose, indoor_smoke
from echorescue.planner_flight import KnownMapPlan, plan_known_map, serialize_planning_report
from echorescue.sim_integration import Check, REPOSITORY_ROOT, Status, check, serialize_report


DEFAULT_CONFIG = REPOSITORY_ROOT / "config" / "planner-flight-v0.15.0.json"


def load_config(path: Path = DEFAULT_CONFIG) -> dict[str, Any]:
    config: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
    plan_known_map(config)
    return config


def _base_path(config: Mapping[str, Any]) -> Path:
    return REPOSITORY_ROOT / str(config["base_indoor_config"])


def _indoor_configuration(config: Mapping[str, Any], plan: KnownMapPlan) -> dict[str, Any]:
    raw: dict[str, Any] = json.loads(_base_path(config).read_text(encoding="utf-8"))
    flight = config["flight"]
    raw["mission"] = {
        "takeoff_altitude_m": plan.cruise_altitude_m,
        **{name: value for name, value in flight.items()},
        "targets": [
            {name: value for name, value in target.items() if name not in {"source_cell", "leg"}}
            for target in plan.targets
        ],
    }
    return raw


def planner_diagnose(path: Path = DEFAULT_CONFIG) -> dict[str, Any]:
    try:
        config = load_config(path)
        plan = plan_known_map(config)
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        return {
            "schema_version": "echorescue-planner-flight-diagnostic/1.0", "milestone": "v0.15.0",
            "ready": False, "checks": [asdict(check("planner.known_map", Status.FAIL, str(error)))],
        }
    indoor = indoor_diagnose(_base_path(config))
    checks = [
        Check(item["name"], Status(item["status"]), item.get("version"), item["detail"])
        for item in indoor["checks"]
    ]
    checks.append(check(
        "planner.known_map", Status.PASS,
        f"existing A* produced {len(plan.raw_outbound) - 1}+{len(plan.raw_return) - 1} cost with {len(plan.targets)} flight targets",
    ))
    return {
        "schema_version": "echorescue-planner-flight-diagnostic/1.0", "milestone": "v0.15.0",
        "ready": not any(item.status is Status.FAIL for item in checks),
        "planning": plan.report(), "checks": [asdict(item) for item in sorted(checks, key=lambda item: item.name)],
    }


def _agreement_checks(plan: KnownMapPlan, execution: Mapping[str, Any]) -> list[Check]:
    expected = [target["target_id"] for target in plan.targets]
    mission = execution.get("mission") or {}
    observer = execution.get("observer") or {}
    actual = [item.get("target_id") for item in mission.get("targets", [])]
    observed = observer.get("target_ids")
    evaluator = execution.get("gazebo_evaluation") or {}
    ordered = actual == expected and observed == expected
    safety = (
        evaluator.get("status") == "PASS"
        and evaluator.get("prohibited_contact_detected") is False
        and evaluator.get("planner_goal_region_entered") is True
        and evaluator.get("return_region_entered") is True
        and [item.get("direction") for item in evaluator.get("doorway_crossings", []) if item.get("inside_opening")] == ["outbound", "inbound"]
    )
    return [
        check("planner.executor_target_agreement", Status.PASS if ordered else Status.FAIL, f"planned={expected}; executed={actual}; observed={observed}"),
        check("planner.evaluator_route_agreement", Status.PASS if safety else Status.FAIL, "goal-side entry, return, doorway crossings and collision result must agree"),
    ]


def _annotate_goal_evaluation(
    plan: KnownMapPlan, execution: dict[str, Any], tolerance_m: float,
) -> None:
    evaluator = execution.get("gazebo_evaluation") or {}
    trajectory = evaluator.get("trajectory", [])
    goal_east, goal_north = plan.transform.cell_center_to_enu(plan.goal)
    entered = False
    minimum_error: float | None = None
    if trajectory:
        origin_east = float(trajectory[0]["east_m"])
        origin_north = float(trajectory[0]["north_m"])
        for sample in trajectory:
            error = (
                (float(sample["east_m"]) - origin_east - goal_east) ** 2
                + (float(sample["north_m"]) - origin_north - goal_north) ** 2
            ) ** 0.5
            minimum_error = error if minimum_error is None else min(minimum_error, error)
        entered = minimum_error is not None and minimum_error <= tolerance_m
    evaluator["planner_goal_region"] = {
        "east_offset_m": goal_east, "north_offset_m": goal_north,
        "horizontal_tolerance_m": tolerance_m,
    }
    evaluator["planner_goal_minimum_horizontal_error_m"] = minimum_error
    evaluator["planner_goal_region_entered"] = entered


def planner_smoke(
    path: Path, output_path: Path | None, timeout_s: float, graphical: bool,
) -> dict[str, Any]:
    config = load_config(path)
    plan = plan_known_map(config)
    planning = plan.report()
    with tempfile.TemporaryDirectory(prefix="echorescue-v0150-") as temporary:
        generated_config = Path(temporary) / "generated-indoor-config.json"
        generated_config.write_text(json.dumps(_indoor_configuration(config, plan)), encoding="utf-8")
        execution = indoor_smoke(generated_config, None, timeout_s, graphical, "owned")
    _annotate_goal_evaluation(
        plan, execution, float(config["planning"]["evaluation_goal_tolerance_m"]),
    )
    mission = execution.get("mission") or {}
    planning["observed_launch_local_enu"] = mission.get("launch_enu")
    checks = _agreement_checks(plan, execution)
    passed = execution.get("status") == Status.PASS and all(item.status is Status.PASS for item in checks)
    report = {
        "schema_version": "echorescue-planner-flight-smoke/1.0", "milestone": "v0.15.0",
        "status": Status.PASS if passed else Status.FAIL,
        "mode": "graphical" if graphical else "headless", "stack_mode": "owned",
        "planning": planning, "execution": execution,
        "checks": [asdict(item) for item in checks],
        "failure_reason": None if passed else "planner, executor, observer or evaluator disagreement",
    }
    if output_path:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(serialize_report(report), encoding="utf-8")
    return report


def expected_rejection(path: Path, kind: str) -> dict[str, Any]:
    config = load_config(path)
    modified = copy.deepcopy(config)
    if kind == "blocked-goal":
        modified["planning"]["outbound_goal_cell"] = [8, 17]
        expected_text = "goal cell is occupied"
    else:
        rows = modified["map"]["occupancy_rows"]
        for row_index in (7, 9):
            rows[row_index] = rows[row_index][:12] + "#" + rows[row_index][13:]
        expected_text = "no deterministic path"
    failure = None
    try:
        plan_known_map(modified)
    except ValueError as error:
        failure = str(error)
    passed = failure is not None and expected_text in failure
    return {
        "schema_version": "echorescue-planner-flight-prearm-rejection/1.0",
        "milestone": "v0.15.0", "case": kind,
        "status": Status.PASS if passed else Status.FAIL,
        "planning_outcome": "REJECTED" if failure else "UNEXPECTEDLY_ACCEPTED",
        "failure_reason": failure, "flight_mission_started": False,
        "processes_started": [], "commands_sent": [],
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the v0.15.0 known-map planner flight bridge")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("diagnose")
    smoke = sub.add_parser("smoke")
    smoke.add_argument("--timeout", type=float, default=260.0)
    smoke.add_argument("--graphical", action="store_true")
    sub.add_parser("negative-blocked-goal")
    sub.add_parser("negative-clearance")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "diagnose":
        report = planner_diagnose(args.config)
        passed = report["ready"]
    elif args.command == "smoke":
        report = planner_smoke(args.config, args.output, args.timeout, args.graphical)
        passed = report["status"] == Status.PASS
    else:
        kind = "blocked-goal" if args.command == "negative-blocked-goal" else "unsafe-clearance"
        report = expected_rejection(args.config, kind)
        passed = report["status"] == Status.PASS
    rendered = serialize_planning_report(report)
    if args.output and args.command != "smoke":
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    sys.stdout.write(rendered)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
