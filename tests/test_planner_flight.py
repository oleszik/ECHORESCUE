import copy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from echorescue.planner_flight import (
    GridTransform, compact_path, inflate_occupied, load_planner_flight_config,
    plan_known_map, segment_cells, serialize_planning_report,
)


CONFIG = Path(__file__).parents[1] / "config/planner-flight-v0.15.0.json"


def configuration():
    return json.loads(CONFIG.read_text())


class TransformTests(unittest.TestCase):
    def test_launch_anchor_is_exact_start_cell_center(self) -> None:
        plan = plan_known_map(configuration())
        self.assertEqual(plan.transform.enu_to_cell((0, 0)), plan.start)
        self.assertEqual(plan.transform.cell_center_to_enu(plan.start), (0, 0))

    def test_row_and_column_axes_are_explicit(self) -> None:
        transform = plan_known_map(configuration()).transform
        self.assertEqual(transform.cell_center_to_enu((9, 5)), (0.5, 0.5))
        self.assertEqual(transform.enu_to_cell((0.5, 0.5)), (9, 5))

    def test_unsupported_axis_direction_is_rejected(self) -> None:
        config = configuration()
        config["map"]["row_axis"] = "south_positive"
        with self.assertRaisesRegex(ValueError, "north-positive"):
            plan_known_map(config)

    def test_transform_rejects_out_of_bounds(self) -> None:
        transform = plan_known_map(configuration()).transform
        with self.assertRaisesRegex(ValueError, "outside map"):
            transform.enu_to_cell((-100, 0))


class InflationAndPlanningTests(unittest.TestCase):
    def test_inflation_uses_radius_margin_and_cell_extent(self) -> None:
        plan = plan_known_map(configuration())
        self.assertEqual(plan.footprint_m, 0.55)
        self.assertIn((8, 15), plan.inflated_occupied)
        self.assertNotIn((8, 14), plan.inflated_occupied)

    def test_valid_doorway_remains_traversable(self) -> None:
        plan = plan_known_map(configuration())
        self.assertIn((9, 12), plan.raw_outbound)
        self.assertEqual(plan.raw_outbound[0], plan.start)
        self.assertEqual(plan.raw_outbound[-1], plan.goal)

    def test_unsafe_narrow_doorway_is_unreachable(self) -> None:
        config = configuration()
        rows = config["map"]["occupancy_rows"]
        for index in (7, 9):
            rows[index] = rows[index][:12] + "#" + rows[index][13:]
        with self.assertRaisesRegex(ValueError, "no deterministic path"):
            plan_known_map(config)

    def test_occupied_goal_is_rejected(self) -> None:
        config = configuration()
        config["planning"]["outbound_goal_cell"] = [8, 17]
        with self.assertRaisesRegex(ValueError, "goal cell is occupied"):
            plan_known_map(config)

    def test_out_of_bounds_goal_is_rejected(self) -> None:
        config = configuration()
        config["planning"]["outbound_goal_cell"] = [12, 25]
        with self.assertRaisesRegex(ValueError, "goal cell is outside"):
            plan_known_map(config)

    def test_launch_and_return_policies_are_enforced(self) -> None:
        for field, value, message in (
            ("start_cell_policy", "fixed_origin", "observed-launch"),
            ("return_goal_policy", "reverse_waypoints", "recorded launch"),
        ):
            with self.subTest(field=field):
                config = configuration()
                config["planning"][field] = value
                with self.assertRaisesRegex(ValueError, message):
                    plan_known_map(config)

    def test_radius_and_margin_are_validated_independently(self) -> None:
        config = configuration()
        config["planning"]["conservative_vehicle_radius_m"] = 0
        with self.assertRaisesRegex(ValueError, "vehicle radius"):
            plan_known_map(config)
        config = configuration()
        config["planning"]["safety_margin_m"] = -0.01
        with self.assertRaisesRegex(ValueError, "safety margin"):
            plan_known_map(config)

    def test_existing_astar_is_called_for_outbound_and_return(self) -> None:
        config = configuration()
        from echorescue import planner_flight
        real = planner_flight.astar
        with patch("echorescue.planner_flight.astar", wraps=real) as reused:
            plan_known_map(config)
        self.assertEqual(reused.call_count, 2)

    def test_identical_input_produces_identical_plan(self) -> None:
        self.assertEqual(plan_known_map(configuration()).report(), plan_known_map(configuration()).report())

    def test_diagonal_connectivity_and_corner_cutting_are_rejected(self) -> None:
        config = configuration()
        config["planning"]["connectivity"] = "eight"
        config["planning"]["allow_diagonal_corner_cutting"] = True
        with self.assertRaisesRegex(ValueError, "cardinal"):
            plan_known_map(config)


class CompactionAndRouteTests(unittest.TestCase):
    def test_collinear_compaction_is_deterministic(self) -> None:
        path = ((1, 1), (1, 2), (1, 3), (2, 3), (3, 3))
        self.assertEqual(compact_path(path, frozenset()), ((1, 1), (1, 3), (3, 3)))

    def test_compaction_rechecks_every_segment_cell(self) -> None:
        with self.assertRaisesRegex(ValueError, "intersects"):
            compact_path(((1, 1), (1, 2), (1, 3)), frozenset({(1, 2)}))

    def test_diagonal_compacted_segment_is_forbidden(self) -> None:
        with self.assertRaisesRegex(ValueError, "axis-aligned"):
            segment_cells((0, 0), (1, 1))

    def test_generated_targets_are_cell_centres_in_leg_order(self) -> None:
        plan = plan_known_map(configuration())
        self.assertEqual(plan.targets[0]["target_id"], "launch-stabilize")
        self.assertEqual(plan.targets[-1]["target_id"], "return-launch")
        self.assertEqual(plan.targets[-1]["east_offset_m"], 0)
        self.assertEqual(plan.targets[-1]["north_offset_m"], 0)
        self.assertIn("outbound-goal", [target["target_id"] for target in plan.targets])

    def test_geofence_rejects_generated_route(self) -> None:
        config = configuration()
        config["flight"]["geofence_horizontal_radius_m"] = 5
        with self.assertRaisesRegex(ValueError, "geofence"):
            plan_known_map(config)

    def test_report_serialization_is_deterministic(self) -> None:
        report = plan_known_map(configuration()).report()
        self.assertEqual(serialize_planning_report(report), serialize_planning_report(report))

    def test_repository_configuration_loads_and_validates(self) -> None:
        self.assertEqual(load_planner_flight_config(CONFIG)["map_version"], "echorescue-indoor-known-map-v1")


class TrustBoundaryTests(unittest.TestCase):
    def test_planner_and_command_node_do_not_read_gazebo_ground_truth(self) -> None:
        root = Path(__file__).parents[1]
        sources = [
            (root / "src/echorescue/planner_flight.py").read_text(),
            (root / "ros2_ws/src/echorescue_ros/echorescue_ros/mavlink_waypoint_mission.py").read_text(),
        ]
        for source in sources:
            for prohibited in ("gz.msgs", "gz topic", "/world/", "parse_pose_message", "parse_contact_message"):
                self.assertNotIn(prohibited, source)
