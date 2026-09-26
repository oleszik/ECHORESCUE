"""Evaluator-only matching for v0.16.1 visual survivor reports."""

from __future__ import annotations

import json
from math import atan2, pi
from pathlib import Path
from typing import Any

from echorescue.visual_survivor import match_estimates


def _angle_difference(left: float, right: float) -> float:
    return abs((left - right + pi) % (2 * pi) - pi)


def evaluate(observer: dict[str, Any], truth: dict[str, Any], horizontal_fov_rad: float) -> dict[str, Any]:
    confirmed = observer.get("confirmed_survivors", [])
    truths = truth["survivors"]
    estimates = [(float(item["east_m"]), float(item["north_m"])) for item in confirmed]
    positions = [
        (float(item["marker_enu_m"][0]),
         float(item["marker_enu_m"][1]))
        for item in truths
    ]
    result = match_estimates(estimates, positions, float(truth["matching_tolerance_m"]))
    matches = [{
        "survivor_id": confirmed[estimate]["survivor_id"],
        "truth_id": truths[target]["truth_id"], "localization_error_m": distance,
    } for estimate, target, distance in result.matches]
    visible: list[dict[str, object]] = []
    candidates = observer.get("candidates", [])
    for candidate in candidates:
        origin = (float(candidate["vehicle_east_m"]), float(candidate["vehicle_north_m"]))
        yaw = float(candidate["vehicle_yaw_rad"])
        any_visible = any(
            _angle_difference(atan2(y - origin[1], x - origin[0]), yaw)
            <= horizontal_fov_rad / 2
            for x, y in positions
        )
        visible.append({"observation_id": candidate["observation_id"], "truth_in_horizontal_fov": any_visible})
    tolerance = float(truth["matching_tolerance_m"])
    duplicate_count = sum(max(0, sum(
        ((estimate[0] - target[0]) ** 2 + (estimate[1] - target[1]) ** 2) ** 0.5 <= tolerance
        for estimate in estimates
    ) - 1) for target in positions)
    return {
        "schema_version": "echorescue-visual-survivor-evaluation/1.0",
        "milestone": "v0.16.1", "evaluator_only": True,
        "ground_truth_survivor_count": len(truths),
        "confirmed_report_count": len(confirmed), "true_positives": len(matches),
        "false_positives": result.false_positives, "false_negatives": result.false_negatives,
        "precision": result.precision, "recall": result.recall,
        "duplicate_report_count": duplicate_count, "matches": matches,
        "candidate_visibility_checks": visible,
        "status": "PASS" if result.false_positives == 0 and result.false_negatives == 0 else "FAIL",
    }


def evaluate_files(observer_path: Path, truth_path: Path, output: Path,
                   horizontal_fov_rad: float) -> dict[str, Any]:
    report = evaluate(json.loads(observer_path.read_text()), json.loads(truth_path.read_text()), horizontal_fov_rad)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report
