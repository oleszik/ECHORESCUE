import json
from pathlib import Path
import tempfile
import unittest

from echorescue.indoor_evaluation import load_indoor_config
from echorescue.planner_flight import plan_known_map
from echorescue.sensor_replanning_integration import _generated_indoor, _sensor_mission_command


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "config/sensor-replanning-v0.15.1.json"


class SensorReplanningIntegrationContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.raw = json.loads(CONFIG.read_text(encoding="utf-8"))
        self.plan = plan_known_map(self.raw)

    def test_unknown_obstacle_is_not_in_prior_map_and_generated_routes_are_used(self) -> None:
        generated = _generated_indoor(self.raw, self.plan, False)
        unknown = next(item for item in generated["geometry"]["entities"] if item["name"] == "unknown_obstacle")
        relative = (
            float(unknown["center"][0]) - float(generated["world"]["launch_pose_world_enu"][0]),
            float(unknown["center"][1]) - float(generated["world"]["launch_pose_world_enu"][1]),
        )
        cell = self.plan.transform.enu_to_cell(relative)
        self.assertNotIn(cell, self.plan.occupied)
        self.assertEqual(
            [item["target_id"] for item in generated["mission"]["targets"]],
            [item["target_id"] for item in self.plan.targets],
        )

    def test_unreachable_world_uses_a_sensor_visible_unknown_barrier(self) -> None:
        generated = _generated_indoor(self.raw, self.plan, True)
        barrier = next(item for item in generated["geometry"]["entities"] if item["name"] == "unknown_barrier")
        self.assertEqual(barrier["size"], [0.7, 5.0, 2.4])
        self.assertTrue(str(generated["world"]["sdf_path"]).endswith("_unreachable.sdf"))

    def test_command_selects_only_the_narrow_sensor_mission_adapter(self) -> None:
        generated = _generated_indoor(self.raw, self.plan, False)
        with tempfile.TemporaryDirectory() as temporary:
            command = _sensor_mission_command(generated, Path(temporary) / "report.json", CONFIG)
        self.assertIn("echorescue_ros.mavlink_sensor_replanning_mission", command)
        self.assertIn("--planner-config", command)
        self.assertNotIn("echorescue_ros.mavlink_waypoint_mission", command)


class SensorWorldContractTests(unittest.TestCase):
    def test_both_worlds_and_configured_sensor_model_parse(self) -> None:
        _, config = load_indoor_config(ROOT / "config/sensor-replanning-indoor-v0.15.1.json")
        self.assertEqual(config.milestone, "v0.15.1")
        for name in ("echorescue_indoor_v0_15_1.sdf", "echorescue_indoor_v0_15_1_unreachable.sdf"):
            text = (ROOT / "worlds" / name).read_text(encoding="utf-8")
            self.assertIn("model://echorescue_iris_range_sensor", text)
            self.assertIn("/echorescue/sensors/obstacle_scan", (ROOT / "models/echorescue_iris_range_sensor/model.sdf").read_text())
