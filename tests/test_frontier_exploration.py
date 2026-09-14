from math import nan, pi
import unittest

from echorescue.frontier_exploration import FrontierOccupancyMap, FrontierPolicy, MappingPolicy
from echorescue.models import CellState
from echorescue.planner_flight import GridTransform
from echorescue.sensor_replanning import PoseSample, RangeObservation


class FrontierExplorationTests(unittest.TestCase):
    def setUp(self) -> None:
        transform = GridTransform(9, 9, 1.0, -4.5, -4.5, "east_positive", "north_positive")
        self.value = FrontierOccupancyMap(MappingPolicy(
            transform, 0.4, range_min_m=0.25, range_max_m=4.0,
            observation_max_age_s=1.0, pose_max_age_s=1.0,
            pose_observation_max_skew_s=1.0,
        ))
        self.value.observe_pose(PoseSample("s1", 100, 1_000_000_000, 10.0, 20.0, 0.0))

    def scan(self, ranges: tuple[float, ...], sequence: int = 1,
             angle: float = 0.0, frame: str = "iris_range_link",
             session: str = "s1", receipt: int = 1_000_000_001,
             increment: float = 0.1) -> RangeObservation:
        return RangeObservation(
            session, sequence, sequence * 100, receipt, frame, angle,
            increment, 0.25, 4.0, ranges,
        )

    def process(self, scan: RangeObservation, now: int = 1_100_000_000) -> bool:
        return self.value.process(scan, launch_east_m=10.0, launch_north_m=20.0, now_ns=now)

    def test_hit_marks_traversal_free_and_endpoint_occupied(self) -> None:
        self.assertTrue(self.process(self.scan((2.2,))))
        self.assertIs(self.value.state((4, 4)), CellState.FREE)
        self.assertIs(self.value.state((4, 5)), CellState.FREE)
        self.assertIs(self.value.state((4, 6)), CellState.OCCUPIED)
        self.assertEqual(self.value.map_revision, 1)

    def test_maximum_range_marks_free_without_occupied_endpoint(self) -> None:
        self.process(self.scan((4.0,)))
        self.assertEqual(self.value.counts()["occupied_cell_count"], 0)
        self.assertIs(self.value.state((4, 8)), CellState.FREE)

    def test_out_of_bounds_beam_clips_free_and_never_invents_boundary_hit(self) -> None:
        self.process(self.scan((3.0,), angle=pi))
        self.value.observe_pose(PoseSample("s1", 101, 1_000_000_010, 6.0, 20.0, 180.0))
        self.process(self.scan((3.8,), sequence=2, receipt=1_000_000_011))
        self.assertEqual(self.value.counts()["occupied_cell_count"], 1)

    def test_occupied_cell_is_never_cleared_by_later_no_return(self) -> None:
        self.process(self.scan((2.2,)))
        occupied = (4, 6)
        self.value.observe_pose(PoseSample("s1", 101, 1_000_000_010, 10.0, 20.0, 0.0))
        self.process(self.scan((4.0,), sequence=2, receipt=1_000_000_011))
        self.assertIs(self.value.state(occupied), CellState.OCCUPIED)

    def test_duplicate_effective_scan_does_not_increment_revision(self) -> None:
        self.process(self.scan((2.2,)))
        self.value.observe_pose(PoseSample("s1", 101, 1_000_000_010, 10.0, 20.0, 0.0))
        self.assertFalse(self.process(self.scan((2.2,), sequence=2, receipt=1_000_000_011)))
        self.assertEqual(self.value.map_revision, 1)

    def test_revision_records_raw_transitions_and_counts(self) -> None:
        self.process(self.scan((2.2,)))
        revision = self.value.revisions[0]
        self.assertEqual(revision["new_occupied_cells"], [[4, 6]])
        self.assertGreater(len(revision["new_free_cells"]), 0)
        self.assertEqual(revision["unknown_cell_count"] + revision["free_cell_count"] + revision["occupied_cell_count"], 81)

    def test_inflation_is_distinct_from_raw_occupancy(self) -> None:
        self.process(self.scan((2.2,)))
        self.assertEqual(len(self.value.occupied), 1)
        self.assertGreater(len(self.value.inflated), 1)

    def test_frontiers_are_safe_free_and_clustered_deterministically(self) -> None:
        self.process(self.scan((4.0, 4.0, 4.0), angle=-0.1))
        frontiers = self.value.frontier_cells()
        self.assertTrue(frontiers)
        self.assertTrue(all(self.value.safe_free(cell) for cell in frontiers))
        clusters = self.value.frontier_clusters()
        self.assertEqual(clusters, tuple(sorted(clusters, key=lambda item: item[0])))

    def test_selection_uses_gain_cost_and_stable_row_column_tie_break(self) -> None:
        self.process(self.scan((4.0, 4.0, 4.0), angle=-0.1))
        policy = FrontierPolicy(information_gain_weight=2.0, path_cost_weight=1.0)
        first = self.value.select_frontier((4, 4), policy)
        second = self.value.select_frontier((4, 4), policy)
        self.assertIsNotNone(first)
        self.assertEqual(first, second)
        assert first is not None
        self.assertEqual(first.score, 2.0 * first.information_gain - first.path_cost)

    def test_astar_rejects_unknown_shortcut_and_unreachable_frontier(self) -> None:
        self.process(self.scan((2.2,)))
        self.assertIsNone(self.value.path((4, 4), (3, 8)))
        self.assertTrue(all(candidate.path for candidate in self.value.candidates((4, 4), FrontierPolicy())))

    def test_no_frontier_means_controller_owned_completion(self) -> None:
        tiny = FrontierOccupancyMap(MappingPolicy(
            GridTransform(1, 1, 1.0, -0.5, -0.5, "east_positive", "north_positive"),
            0.1, range_max_m=4.0,
        ))
        tiny.observe_pose(PoseSample("s", 1, 1, 0, 0, 0))
        tiny.process(self.scan((4.0,), session="s", receipt=1), launch_east_m=0,
                     launch_north_m=0, now_ns=2)
        self.assertTrue(tiny.exploration_complete((0, 0), FrontierPolicy()))

    def test_cross_session_stale_frame_and_order_rejections_do_not_mutate_map(self) -> None:
        invalid = (
            self.scan((1.0,), session="old"),
            self.scan((1.0,), receipt=1, sequence=2),
            self.scan((1.0,), frame="wrong", sequence=3),
        )
        for scan in invalid:
            self.assertFalse(self.process(scan))
        self.assertEqual(self.value.map_revision, 0)
        self.assertEqual(self.value.rejected_scans, 3)

    def test_invalid_numeric_range_and_angle_inputs_are_rejected(self) -> None:
        scans = (
            self.scan((nan,), sequence=1),
            self.scan((0.1,), sequence=2),
            self.scan((4.1,), sequence=3),
            self.scan((1.0,), sequence=4, increment=0.0),
        )
        for scan in scans:
            self.assertFalse(self.process(scan))
        self.assertEqual(self.value.map_revision, 0)

    def test_pose_regression_and_stale_pose_are_rejected(self) -> None:
        self.assertFalse(self.value.observe_pose(PoseSample("s1", 99, 1_000_000_001, 10, 20, 0)))
        self.assertFalse(self.process(self.scan((1.0,)), now=3_000_000_000))
        self.assertEqual(self.value.map_revision, 0)

    def test_new_session_resets_scan_order_but_old_session_scan_is_rejected(self) -> None:
        self.process(self.scan((1.0,), sequence=8))
        self.value.observe_pose(PoseSample("s2", 1, 2_000_000_000, 10, 20, 0))
        self.assertFalse(self.process(self.scan((1.0,), sequence=9, session="s1", receipt=2_000_000_001), 2_100_000_000))
        accepted = self.value.accepted_scans
        self.process(self.scan((1.0,), sequence=1, session="s2", receipt=2_000_000_001), 2_100_000_000)
        self.assertEqual(self.value.accepted_scans, accepted + 1)


if __name__ == "__main__":
    unittest.main()
