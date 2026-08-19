import unittest

from echorescue.mapping import OccupancyMap
from echorescue.models import CellState, Position


class OccupancyMapTests(unittest.TestCase):
    def test_observations_update_only_reported_cells(self) -> None:
        occupancy_map = OccupancyMap(5, 5)
        occupancy_map.update(
            {
                Position(2, 2): CellState.FREE,
                Position(3, 2): CellState.OCCUPIED,
            }
        )

        self.assertEqual(occupancy_map.cell_at(Position(2, 2)), CellState.FREE)
        self.assertEqual(occupancy_map.cell_at(Position(3, 2)), CellState.OCCUPIED)
        self.assertEqual(occupancy_map.cell_at(Position(1, 2)), CellState.UNKNOWN)

    def test_frontier_is_known_free_cell_next_to_unknown(self) -> None:
        occupancy_map = OccupancyMap(5, 5)
        occupancy_map.update(
            {
                Position(2, 2): CellState.FREE,
                Position(1, 2): CellState.OCCUPIED,
                Position(3, 2): CellState.OCCUPIED,
                Position(2, 1): CellState.OCCUPIED,
            }
        )

        self.assertEqual(occupancy_map.frontiers(), (Position(2, 2),))
        occupancy_map.update({Position(2, 3): CellState.OCCUPIED})
        self.assertEqual(occupancy_map.frontiers(), ())


if __name__ == "__main__":
    unittest.main()

