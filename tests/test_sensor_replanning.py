import json
from math import atan2, hypot, inf, nan
from pathlib import Path
import unittest
from unittest.mock import patch

from echorescue.planner_flight import plan_known_map
from echorescue.sensor_replanning import PoseSample, RangeObservation, ReplanningLimits, SensorMapReplanner


CONFIG = Path(__file__).parents[1] / "config/sensor-replanning-v0.15.1.json"


def replanner() -> SensorMapReplanner:
    plan = plan_known_map(json.loads(CONFIG.read_text()))
    value = SensorMapReplanner(plan, ReplanningLimits(maximum_attempts=1))
    value.observe_pose(PoseSample("session-a", 1000, 1_000_000_000, 10.0, 20.0, 0.0))
    return value


def observation(**changes: object) -> RangeObservation:
    east, north = 3.0, 0.5
    values = {
        "session_id": "session-a", "sequence": 1, "sensor_time_ns": 500,
        "receipt_monotonic_ns": 1_050_000_000, "sensor_frame": "iris_range_link",
        "angle_min_rad": atan2(north, east), "angle_increment_rad": 0.1,
        "range_min_m": 0.25, "range_max_m": 4.0,
        "ranges_m": (hypot(east, north),),
    }
    values.update(changes)
    return RangeObservation(**values)  # type: ignore[arg-type]


class SensorReplanningTests(unittest.TestCase):
    def test_sensor_discovery_invalidates_and_replans_with_existing_astar(self) -> None:
        value = replanner()
        remaining = value.plan.raw_outbound + value.plan.raw_return
        with patch("echorescue.sensor_replanning.astar", wraps=__import__("echorescue.planning", fromlist=["astar"]).astar) as reused:
            replacement = value.process(
                observation(), launch_east_m=10, launch_north_m=20,
                remaining_route=remaining, now_ns=1_100_000_000,
            )
        self.assertIsNotNone(replacement)
        self.assertTrue(replacement)
        self.assertEqual(value.attempts, 1)
        self.assertGreaterEqual(reused.call_count, 2)
        self.assertNotEqual(value.replans[0]["compacted_outbound"], [list(cell) for cell in value.plan.compacted_outbound])

    def test_discovery_not_intersecting_remaining_route_does_not_replan(self) -> None:
        value = replanner()
        result = value.process(
            observation(angle_min_rad=-1.57, ranges_m=(1.0,)),
            launch_east_m=10, launch_north_m=20,
            remaining_route=value.plan.raw_outbound + value.plan.raw_return,
            now_ns=1_100_000_000,
        )
        self.assertIsNone(result)
        self.assertEqual(value.attempts, 0)

    def test_unreachable_after_discovery_reports_explicit_failure(self) -> None:
        value = replanner()
        with patch("echorescue.sensor_replanning._grid_path", return_value=None):
            result = value.process(
                observation(), launch_east_m=10, launch_north_m=20,
                remaining_route=value.plan.raw_outbound, now_ns=1_100_000_000,
            )
        self.assertEqual(result, ())
        self.assertEqual(value.failure_reason, "no safe route after sensor discovery")

    def test_invalid_observations_never_change_map_or_route(self) -> None:
        cases = (
            {"receipt_monotonic_ns": 1, "sequence": 1},
            {"session_id": "old", "sequence": 1},
            {"ranges_m": (nan,), "sequence": 1},
            {"ranges_m": (inf,), "sequence": 1},
            {"ranges_m": (0.1,), "sequence": 1},
            {"ranges_m": (4.1,), "sequence": 1},
            {"range_min_m": 0.1, "sequence": 1},
            {"range_max_m": 5.0, "sequence": 1},
            {"angle_increment_rad": -0.1, "sequence": 1},
        )
        for changes in cases:
            with self.subTest(changes=changes):
                value = replanner()
                before = set(value.occupied)
                result = value.process(
                    observation(**changes), launch_east_m=10, launch_north_m=20,
                    remaining_route=value.plan.raw_outbound, now_ns=1_100_000_000,
                )
                self.assertIsNone(result)
                self.assertEqual(value.occupied, before)
                self.assertFalse(value.observations[-1]["accepted"])

    def test_out_of_order_sequence_and_sensor_time_are_rejected(self) -> None:
        value = replanner()
        value.process(observation(), launch_east_m=10, launch_north_m=20, remaining_route=(), now_ns=1_100_000_000)
        before = set(value.occupied)
        value.process(observation(sequence=1, sensor_time_ns=499), launch_east_m=10, launch_north_m=20, remaining_route=(), now_ns=1_100_000_000)
        self.assertEqual(value.occupied, before)
        self.assertFalse(value.observations[-1]["accepted"])

    def test_pose_observation_skew_and_stale_pose_are_rejected(self) -> None:
        value = replanner()
        result = value.process(
            observation(receipt_monotonic_ns=1_400_000_000),
            launch_east_m=10, launch_north_m=20,
            remaining_route=value.plan.raw_outbound, now_ns=1_450_000_000,
        )
        self.assertIsNone(result)
        self.assertIn("pose", value.observations[-1]["reason"])
