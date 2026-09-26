import unittest

from echorescue.visual_survivor_evaluation import evaluate


class VisualSurvivorEvaluationTests(unittest.TestCase):
    def truth(self):
        return {
            "matching_tolerance_m": 1.0,
            "survivors": [
                {"truth_id": "a", "marker_enu_m": [1, 0, 0]},
                {"truth_id": "b", "marker_enu_m": [3, 0, 0]},
            ],
        }

    def test_exact_one_to_one_precision_recall(self):
        observer = {"confirmed_survivors": [
            {"survivor_id": "survivor-001", "east_m": 1.1, "north_m": 0},
            {"survivor_id": "survivor-002", "east_m": 3.1, "north_m": 0},
        ], "candidates": []}
        result = evaluate(observer, self.truth(), 1.0)
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["precision"], 1.0)
        self.assertEqual(result["recall"], 1.0)

    def test_duplicate_report_is_false_positive(self):
        observer = {"confirmed_survivors": [
            {"survivor_id": "survivor-001", "east_m": 1, "north_m": 0},
            {"survivor_id": "survivor-002", "east_m": 1.1, "north_m": 0},
        ], "candidates": []}
        result = evaluate(observer, self.truth(), 1.0)
        self.assertEqual(result["false_positives"], 1)
        self.assertEqual(result["duplicate_report_count"], 1)
        self.assertEqual(result["status"], "FAIL")

    def test_visibility_is_evaluator_only(self):
        observer = {"confirmed_survivors": [], "candidates": [{
            "observation_id": "obs", "vehicle_east_m": 0,
            "vehicle_north_m": 0, "vehicle_yaw_rad": 0,
        }]}
        result = evaluate(observer, self.truth(), 1.0)
        self.assertTrue(result["evaluator_only"])
        self.assertTrue(result["candidate_visibility_checks"][0]["truth_in_horizontal_fov"])


if __name__ == "__main__":
    unittest.main()
