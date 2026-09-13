import json
from pathlib import Path
import tempfile
import unittest

from echorescue.multi_obstacle_integration import _generated_configuration, _world, load_scenarios
from echorescue.planner_flight import plan_known_map


ROOT = Path(__file__).parents[1]
SCENARIOS = ROOT / "config/multi-obstacle-scenarios-v0.15.2.json"


class MultiObstacleScenarioTests(unittest.TestCase):
    def test_multi_scenario_has_three_distinct_unknown_obstacles(self) -> None:
        raw = load_scenarios(SCENARIOS)
        obstacles = raw["scenarios"]["multi"]["obstacles"]
        self.assertGreaterEqual(len(obstacles), 3)
        self.assertEqual(len({tuple(item["center"]) for item in obstacles}), len(obstacles))
        world = _world(raw["scenarios"]["multi"])
        for item in obstacles:
            self.assertEqual(world.count(f'model name="{item["name"]}"'), 1)

    def test_generated_planner_preserves_prior_map_and_applies_bounded_policy(self) -> None:
        raw = load_scenarios(SCENARIOS)
        base = json.loads((ROOT / raw["planner_config"]).read_text())
        with tempfile.TemporaryDirectory() as name:
            generated = _generated_configuration(raw, "multi", Path(name))
            configured = json.loads(generated.read_text())
            self.assertEqual(configured["map"], base["map"])
            self.assertEqual(configured["sensor_replanning"]["maximum_replan_attempts"], 4)
            self.assertTrue(configured["sensor_replanning"]["generation_target_ids"])
            self.assertEqual(plan_known_map(configured).occupied, plan_known_map(base).occupied)

    def test_budget_and_later_unreachable_are_distinct_general_policy_cases(self) -> None:
        raw = load_scenarios(SCENARIOS)
        self.assertEqual(raw["scenarios"]["budget"]["maximum_replans_per_mission"], 1)
        later = raw["scenarios"]["later_unreachable"]["obstacles"]
        self.assertTrue(any(item["name"].startswith("unknown_barrier") for item in later))
        self.assertEqual(raw["policy"]["map_change_threshold_cells"], 1)
