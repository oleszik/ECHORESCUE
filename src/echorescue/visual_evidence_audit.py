"""Offline validation of v0.16.1 visual-search acceptance reports.

This never launches or controls a vehicle. It only checks saved evidence.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping, Sequence


SCHEMA = "echorescue-visual-search-smoke/1.0"
CASES = frozenset({
    "success", "partial_occlusion", "distractor", "no_survivor",
    "camera_dropout", "session_change", "budget",
})
REQUIRED_CHECKS = frozenset({
    "perception.controller_prior_hidden",
    "perception.real_frames",
    "perception.multi_observation_confirmation",
    "perception.evaluator_separate",
    "mission.final_safe",
    "gazebo.no_prohibited_contact",
})


def audit_report(report: Mapping[str, Any], expected_case: str | None = None) -> list[str]:
    """Return independent, human-readable reasons evidence cannot be accepted."""
    errors: list[str] = []
    if report.get("schema_version") != SCHEMA:
        errors.append("unsupported or missing schema_version")
    if report.get("milestone") != "v0.16.1":
        errors.append("unexpected milestone")
    case = report.get("case")
    if case not in CASES:
        errors.append("invalid scenario")
    if expected_case is not None and case != expected_case:
        errors.append(f"expected scenario {expected_case!r}, got {case!r}")
    if report.get("status") != "PASS":
        errors.append("top-level status is not PASS")
    if report.get("mode") not in ("headless", "graphical"):
        errors.append("missing or invalid run mode")
    digest = report.get("configuration_digest")
    if not isinstance(digest, str) or len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
        errors.append("invalid configuration digest")

    for section in ("checks", "cleanup"):
        entries = report.get(section)
        if not isinstance(entries, list) or not entries:
            errors.append(f"{section} must be a nonempty list")
            continue
        seen: set[str] = set()
        for index, entry in enumerate(entries):
            if not isinstance(entry, dict) or not isinstance(entry.get("name"), str):
                errors.append(f"{section}[{index}] lacks a check name")
                continue
            name = entry["name"]
            if name in seen:
                errors.append(f"duplicate {section} check {name}")
            seen.add(name)
            if entry.get("status") != "PASS":
                errors.append(f"{section} check {name} is not PASS")
        if section == "checks" and case in CASES:
            for name in sorted((REQUIRED_CHECKS | {f"visual_search.{case}"}) - seen):
                errors.append(f"missing acceptance check {name}")
        if section == "cleanup":
            if "cleanup.ros_nodes" not in seen:
                errors.append("missing ROS node cleanup check")
            if not any(name.startswith("cleanup.") and name != "cleanup.ros_nodes" for name in seen):
                errors.append("missing resource cleanup checks")
    for section in ("controller_mission", "observer", "perception", "gazebo_evaluation"):
        if not isinstance(report.get(section), dict) or not report[section]:
            errors.append(f"missing {section} evidence")
    truth = report.get("evaluator_only_survivor_truth")
    if not isinstance(truth, dict) or truth.get("evaluator_only") is not True:
        errors.append("missing separately marked evaluator truth")
    return errors


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reports", nargs="+", type=Path, help="v0.16.1 saved JSON reports")
    parser.add_argument("--case", choices=sorted(CASES), help="Require one specific scenario")
    args = parser.parse_args(argv)
    failed = False
    for path in args.reports:
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
            errors = audit_report(value, args.case) if isinstance(value, dict) else ["report root must be an object"]
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            errors = [f"cannot read JSON: {exc}"]
        print(f"{path}: {'FAIL' if errors else 'PASS'}")
        for issue in errors:
            print(f"  - {issue}")
        failed |= bool(errors)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
