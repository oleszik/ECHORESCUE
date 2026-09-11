import copy
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from echorescue.indoor_evaluation import (
    AxisAlignedBox, IndoorRunEvaluator, load_indoor_config,
    parse_contact_message, parse_pose_message, point_box_distance,
    segment_box_distance, serialize_indoor_report,
)


CONFIG = Path(__file__).parents[1] / "config/indoor-reference-v0.14.5.json"


class IndoorGeometryTests(unittest.TestCase):
    def test_repository_reference_route_validates(self) -> None:
        raw, config = load_indoor_config(CONFIG)
        self.assertEqual(raw["milestone"], "v0.14.5")
        self.assertEqual(len(config.targets), 9)
        self.assertEqual(config.required_center_clearance_m, 0.55)

    def test_sdf_geometry_and_contact_topics_match_numeric_config(self) -> None:
        _, config = load_indoor_config(CONFIG)
        root = ET.parse(Path(__file__).parents[1] / config.world_sdf).getroot()
        world = root.find("world")
        assert world is not None
        self.assertEqual(world.get("name"), config.world_name)
        models = {model.get("name"): model for model in world.findall("model")}
        for entity in config.entities:
            model = models[entity.name]
            self.assertEqual(tuple(map(float, model.findtext("pose", "").split()[:3])), entity.center)
            self.assertEqual(tuple(map(float, model.findtext("link/collision/geometry/box/size", "").split())), entity.size)
            self.assertEqual(model.findtext("link/sensor/contact/topic"), entity.contact_topic)

    def test_exact_point_and_segment_distance(self) -> None:
        box = AxisAlignedBox("box", "obstacle", (0, 0, 0), (2, 2, 2), "/box")
        self.assertEqual(point_box_distance((0, 0, 0), box), 0)
        self.assertAlmostEqual(point_box_distance((2, 2, 1), box), 2 ** 0.5)
        self.assertEqual(segment_box_distance((-2, 0, 0), (2, 0, 0), box), 0)
        self.assertAlmostEqual(segment_box_distance((-2, 2, 0), (2, 2, 0), box), 1)

    def test_duplicate_topic_is_rejected(self) -> None:
        raw = json.loads(CONFIG.read_text())
        raw["geometry"]["entities"][1]["contact_topic"] = raw["geometry"]["entities"][0]["contact_topic"]
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "bad.json"
            path.write_text(json.dumps(raw))
            with self.assertRaisesRegex(ValueError, "unique"):
                load_indoor_config(path)

    def test_narrow_doorway_is_rejected(self) -> None:
        raw = json.loads(CONFIG.read_text())
        raw["geometry"]["doorway"]["width_m"] = 1.0
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "bad.json"
            path.write_text(json.dumps(raw))
            with self.assertRaisesRegex(ValueError, "not traversable"):
                load_indoor_config(path)

    def test_unsafe_route_segment_is_rejected(self) -> None:
        raw = json.loads(CONFIG.read_text())
        next(item for item in raw["mission"]["targets"] if item["target_id"] == "far-room")["north_offset_m"] = 0
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "bad.json"
            path.write_text(json.dumps(raw))
            with self.assertRaisesRegex(ValueError, "clearance"):
                load_indoor_config(path)


class IndoorEvaluationTests(unittest.TestCase):
    def setUp(self) -> None:
        _, self.config = load_indoor_config(CONFIG)

    def test_gazebo_json_parsers(self) -> None:
        pose = {"header": {"stamp": {"sec": 2, "nsec": 500_000_000}}, "pose": [
            {"name": "other", "position": {}},
            {"name": self.config.model_name, "position": {"x": 1, "y": 2, "z": 3}},
        ]}
        self.assertEqual(parse_pose_message(pose, self.config.model_name), (2.5, (1.0, 2.0, 3.0)))
        contacts = {"contact": {"collision1": "vehicle", "collision2": "wall"}}
        self.assertEqual(parse_contact_message(contacts)[1], [("vehicle", "wall")])

    def test_reference_trajectory_passes_and_records_crossings(self) -> None:
        evaluator = IndoorRunEvaluator(self.config)
        evaluator.set_available_topics(self.config.contact_topics)
        launch = self.config.launch_world_enu
        points = [launch, (launch[0], launch[1], launch[2] + self.config.takeoff_altitude_m)]
        points.extend(target.absolute(launch) for target in self.config.targets)
        points.append(launch)
        for index, point in enumerate(points):
            evaluator.observe_pose(point, float(index))
        report = evaluator.report()
        self.assertEqual(report["status"], "PASS")
        self.assertEqual([item["direction"] for item in report["doorway_crossings"]], ["outbound", "inbound"])
        self.assertFalse(report["prohibited_contact_detected"])

    def test_deliberate_obstacle_collision_fails_immediately(self) -> None:
        evaluator = IndoorRunEvaluator(self.config)
        evaluator.set_available_topics(self.config.contact_topics)
        evaluator.observe_pose(self.config.launch_world_enu, 0)
        evaluator.observe_pose(self.config.obstacle.center, 1)
        self.assertTrue(evaluator.collision_detected)
        self.assertEqual(evaluator.report()["status"], "FAIL")

    def test_non_launch_floor_contact_is_prohibited(self) -> None:
        evaluator = IndoorRunEvaluator(self.config)
        evaluator.observe_contacts("floor", [("iris", "floor")], 1.0, (2.0, 0.0, 0.2))
        self.assertTrue(evaluator.collision_detected)

    def test_missing_contact_topic_coverage_fails(self) -> None:
        evaluator = IndoorRunEvaluator(self.config)
        evaluator.set_available_topics(self.config.contact_topics[:-1])
        self.assertFalse(evaluator.report()["contact_topic_coverage_complete"])
        self.assertEqual(evaluator.report()["status"], "FAIL")

    def test_report_serialization_is_deterministic(self) -> None:
        evaluator = IndoorRunEvaluator(self.config)
        self.assertEqual(serialize_indoor_report(evaluator.report()), serialize_indoor_report(evaluator.report()))


class IndoorSeparationTests(unittest.TestCase):
    def test_command_node_has_no_gazebo_ground_truth_dependency(self) -> None:
        source = (Path(__file__).parents[1] / "ros2_ws/src/echorescue_ros/echorescue_ros/mavlink_waypoint_mission.py").read_text()
        for prohibited in ("gz.msgs", "gz topic", "/world/", "indoor_evaluation", "gazebo_indoor_evaluator"):
            self.assertNotIn(prohibited, source)

    def test_evaluator_has_no_ros_or_mavlink_dependency(self) -> None:
        source = (Path(__file__).parents[1] / "src/echorescue/gazebo_indoor_evaluator.py").read_text()
        for prohibited in ("rclpy", "pymavlink", "set_position_target", "command_long"):
            self.assertNotIn(prohibited, source)
