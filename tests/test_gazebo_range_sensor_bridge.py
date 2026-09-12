from pathlib import Path
import unittest


class RangeBridgeSourceTests(unittest.TestCase):
    def test_bridge_consumes_sensor_topic_without_ground_truth_pose(self) -> None:
        source = (Path(__file__).parents[1] / "ros2_ws/src/echorescue_ros/echorescue_ros/gazebo_range_sensor_bridge.py").read_text()
        self.assertIn('"gz", "topic"', source)
        for prohibited in ("/world/", "pose/info", "contact", "set_pose"):
            self.assertNotIn(prohibited, source)

    def test_planner_and_mission_do_not_import_evaluator(self) -> None:
        root = Path(__file__).parents[1]
        for relative in (
            "src/echorescue/sensor_replanning.py",
            "ros2_ws/src/echorescue_ros/echorescue_ros/mavlink_sensor_replanning_mission.py",
        ):
            source = (root / relative).read_text()
            self.assertNotIn("gazebo_indoor_evaluator", source)
            self.assertNotIn("indoor_evaluation", source)
