import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from echorescue.planner_flight import plan_known_map
from echorescue.planner_flight_integration import (
    _agreement_checks, _annotate_goal_evaluation, _indoor_configuration, expected_rejection, load_config,
    planner_smoke,
)
from echorescue.sim_integration import Status


CONFIG = Path(__file__).parents[1] / "config/planner-flight-v0.15.0.json"


class PlannerIntegrationTests(unittest.TestCase):
    def test_generated_indoor_config_contains_only_planner_targets(self) -> None:
        config = load_config(CONFIG)
        plan = plan_known_map(config)
        indoor = _indoor_configuration(config, plan)
        self.assertEqual(
            [item["target_id"] for item in indoor["mission"]["targets"]],
            [item["target_id"] for item in plan.targets],
        )
        self.assertNotIn("source_cell", indoor["mission"]["targets"][0])

    def test_blocked_goal_rejects_before_process_or_command(self) -> None:
        report = expected_rejection(CONFIG, "blocked-goal")
        self.assertEqual(report["status"], Status.PASS)
        self.assertFalse(report["flight_mission_started"])
        self.assertEqual(report["processes_started"], [])
        self.assertEqual(report["commands_sent"], [])

    def test_unsafe_clearance_rejects_before_process_or_command(self) -> None:
        report = expected_rejection(CONFIG, "unsafe-clearance")
        self.assertEqual(report["status"], Status.PASS)
        self.assertIn("no deterministic path", report["failure_reason"])

    def test_observer_disagreement_fails_gate(self) -> None:
        plan = plan_known_map(load_config(CONFIG))
        execution = {
            "mission": {"targets": [{"target_id": item["target_id"]} for item in plan.targets]},
            "observer": {"target_ids": []},
            "gazebo_evaluation": {},
        }
        checks = {item.name: item for item in _agreement_checks(plan, execution)}
        self.assertEqual(checks["planner.executor_target_agreement"].status, Status.FAIL)

    def test_evaluator_goal_region_uses_world_displacement_only_for_scoring(self) -> None:
        plan = plan_known_map(load_config(CONFIG))
        execution = {"gazebo_evaluation": {"trajectory": [
            {"east_m": -4.0, "north_m": 0.0},
            {"east_m": 4.0, "north_m": 2.0},
        ]}}
        _annotate_goal_evaluation(plan, execution, 0.75)
        self.assertTrue(execution["gazebo_evaluation"]["planner_goal_region_entered"])

    def test_planning_failure_never_calls_flight_harness(self) -> None:
        config = load_config(CONFIG)
        config["planning"]["outbound_goal_cell"] = [8, 17]
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "bad.json"
            path.write_text(json.dumps(config))
            with patch("echorescue.planner_flight_integration.indoor_smoke") as flight:
                with self.assertRaisesRegex(ValueError, "occupied"):
                    planner_smoke(path, None, 1, False)
            flight.assert_not_called()
