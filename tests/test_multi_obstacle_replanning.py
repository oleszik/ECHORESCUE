import json
from math import atan2, hypot
from pathlib import Path
import unittest
from unittest.mock import patch

from echorescue.planner_flight import plan_known_map
from echorescue.sensor_replanning import PoseSample, RangeObservation, ReplanningLimits, SensorMapReplanner


CONFIG = Path(__file__).parents[1] / "config/sensor-replanning-v0.15.1.json"


class RepeatedReplanningTests(unittest.TestCase):
    def setUp(self) -> None:
        self.plan = plan_known_map(json.loads(CONFIG.read_text()))
        self.value = SensorMapReplanner(self.plan, ReplanningLimits(
            maximum_attempts=3, maximum_replans_per_target=2, cooldown_s=0.0,
        ))
        self.value.observe_pose(PoseSample("s", 100, 1_000_000_000, 10, 20, 0))

    def scan(self, cell: tuple[int, int], sequence: int) -> RangeObservation:
        east, north = self.plan.transform.cell_center_to_enu(cell)
        assert self.value.pose is not None
        east -= self.value.pose.east_m - 10
        north -= self.value.pose.north_m - 20
        return RangeObservation(
            "s", sequence, sequence * 100, 1_000_000_000 + sequence,
            "iris_range_link", atan2(north, east), 0.1, 0.25, 4.0,
            (hypot(east, north),),
        )

    def process(self, cell: tuple[int, int], sequence: int, route: tuple[tuple[int, int], ...],
                target: str = "target-a"):
        return self.value.process(
            self.scan(cell, sequence), launch_east_m=10, launch_north_m=20,
            remaining_route=route, now_ns=1_100_000_000 + sequence,
            target_id=target, route_generation=self.value.route_generation,
        )

    def test_multiple_updates_have_stable_revisions_and_route_generations(self) -> None:
        first = self.process((9, 10), 1, self.plan.raw_outbound + self.plan.raw_return)
        self.assertTrue(first)
        route = tuple(tuple(cell) for cell in self.value.replans[-1]["raw_outbound"] + self.value.replans[-1]["raw_return"])
        self.value.observe_pose(PoseSample("s", 101, 1_000_000_002, 15, 22, 0))
        second = self.process((12, 17), 2, route, "target-b")
        self.assertTrue(second)
        self.assertEqual([item["map_revision"] for item in self.value.map_updates], [1, 2])
        self.assertEqual([item["new_route_generation"] for item in self.value.replans], [2, 3])
        self.assertTrue(all(target.target_id.startswith("g003-") for target in second or ()))

    def test_duplicate_cells_do_not_mutate_revision_or_consume_budget(self) -> None:
        self.process((9, 10), 1, ())
        revision, attempts = self.value.map_revision, self.value.attempts
        self.process((9, 10), 2, self.plan.raw_outbound)
        self.assertEqual((self.value.map_revision, self.value.attempts), (revision, attempts))
        self.assertEqual(self.value.observations[-1]["discovered_cells"], [])

    def test_irrelevant_update_does_not_replan(self) -> None:
        result = self.process((4, 5), 1, self.plan.raw_outbound)
        self.assertIsNone(result)
        self.assertEqual(self.value.attempts, 0)
        self.assertIn("irrelevant", self.value.skipped_replans[-1]["reason"])

    def test_quantised_return_immediately_before_known_surface_is_not_discovered(self) -> None:
        scan = RangeObservation(
            "s", 1, 100, 1_000_000_001, "iris_range_link",
            3.141592653589793, 0.1, 0.25, 4.0, (1.1,),
        )
        result = self.value.process(
            scan, launch_east_m=10, launch_north_m=20,
            remaining_route=self.plan.raw_outbound, now_ns=1_100_000_001,
        )
        self.assertIsNone(result)
        self.assertEqual(self.value.map_revision, 0)
        self.assertEqual(self.value.observations[-1]["discovered_cells"], [])

    def test_mission_budget_exhaustion_is_stable(self) -> None:
        self.value.limits = ReplanningLimits(maximum_attempts=1, maximum_replans_per_target=2, cooldown_s=0)
        self.assertTrue(self.process((9, 10), 1, self.plan.raw_outbound, "a"))
        self.value.observe_pose(PoseSample("s", 101, 1_000_000_002, 15, 22, 0))
        result = self.process((12, 17), 2, ((12, 17),), "b")
        self.assertEqual(result, ())
        self.assertEqual(self.value.failure_reason, "replan budget exhausted")
        self.assertEqual(self.value.attempts, 1)

    def test_per_target_budget_and_cooldown_fail_closed(self) -> None:
        self.value.limits = ReplanningLimits(maximum_attempts=3, maximum_replans_per_target=1, cooldown_s=0)
        self.assertTrue(self.process((9, 10), 1, self.plan.raw_outbound, "same"))
        self.value.observe_pose(PoseSample("s", 101, 1_000_000_002, 15, 22, 0))
        self.assertEqual(self.process((12, 17), 2, ((12, 17),), "same"), ())
        self.assertEqual(self.value.failure_reason, "per-target replan budget exhausted")

        other = SensorMapReplanner(self.plan, ReplanningLimits(cooldown_s=10, maximum_replans_per_target=2))
        other.observe_pose(PoseSample("s", 100, 1_000_000_000, 10, 20, 0))
        first_east, first_north = self.plan.transform.cell_center_to_enu((9, 10))
        first_scan = RangeObservation("s", 1, 100, 1_000_000_001, "iris_range_link",
                                      atan2(first_north, first_east), 0.1, 0.25, 4.0,
                                      (hypot(first_east, first_north),))
        other.process(first_scan, launch_east_m=10, launch_north_m=20,
                      remaining_route=self.plan.raw_outbound, now_ns=1_100_000_001, target_id="a")
        other.observe_pose(PoseSample("s", 101, 1_000_000_002, 15, 22, 0))
        second_scan = RangeObservation("s", 2, 200, 1_000_000_002, "iris_range_link", 0.0, 0.1, 0.25, 4.0, (1.5,))
        result = other.process(second_scan, launch_east_m=10, launch_north_m=20,
                               remaining_route=((12, 17),), now_ns=1_100_000_002, target_id="b")
        self.assertEqual(result, ())
        self.assertIn("cooldown", other.failure_reason or "")

    def test_timeout_and_reconnect_reject_old_session(self) -> None:
        with patch("echorescue.sensor_replanning.monotonic_ns", side_effect=(0, 1_000_000_000)):
            self.value.limits = ReplanningLimits(timeout_s=0.01, cooldown_s=0)
            self.assertEqual(self.process((9, 10), 1, self.plan.raw_outbound), ())
        self.assertEqual(self.value.failure_reason, "bounded replanning timeout")

        fresh = SensorMapReplanner(self.plan, ReplanningLimits())
        fresh.observe_pose(PoseSample("old", 100, 1, 10, 20, 0))
        fresh.observe_pose(PoseSample("new", 1, 2, 10, 20, 0))
        old = self.scan((9, 10), 1)
        result = fresh.process(old, launch_east_m=10, launch_north_m=20,
                               remaining_route=self.plan.raw_outbound, now_ns=3)
        self.assertIsNone(result)
        self.assertEqual(fresh.observations[-1]["reason"], "cross-session observation")

    def test_vehicle_time_regression_is_rejected(self) -> None:
        self.assertFalse(self.value.observe_pose(PoseSample("s", 99, 1_000_000_001, 10, 20, 0)))
        self.assertEqual(self.value.pose_rejections[-1]["reason"], "vehicle-time regression")

    def test_map_threshold_and_consecutive_failure_budget_fail_closed(self) -> None:
        self.value.limits = ReplanningLimits(map_change_threshold_cells=99, cooldown_s=0)
        self.assertEqual(self.process((9, 10), 1, self.plan.raw_outbound), ())
        self.assertIn("threshold", self.value.failure_reason or "")

        other = SensorMapReplanner(self.plan, ReplanningLimits(maximum_consecutive_failures=1, cooldown_s=0))
        other.observe_pose(PoseSample("s", 100, 1_000_000_000, 10, 20, 0))
        other.consecutive_failures = 1
        scan = self.scan((9, 10), 2)
        self.assertEqual(other.process(scan, launch_east_m=10, launch_north_m=20,
                                      remaining_route=self.plan.raw_outbound,
                                      now_ns=1_100_000_002), ())
        self.assertEqual(other.failure_reason, "consecutive planning failure budget exhausted")
