import unittest

from echorescue.models import Position
from echorescue.planning import astar


class PlanningTests(unittest.TestCase):
    def test_astar_routes_around_obstacle(self) -> None:
        blocked = {Position(2, 1), Position(2, 2)}

        def passable(position: Position) -> bool:
            return (
                0 <= position.x < 5
                and 0 <= position.y < 5
                and position not in blocked
            )

        path = astar(Position(1, 1), Position(3, 1), passable)

        self.assertIsNotNone(path)
        assert path is not None
        self.assertEqual(path[0], Position(1, 1))
        self.assertEqual(path[-1], Position(3, 1))
        self.assertTrue(blocked.isdisjoint(path))
        self.assertEqual(len(path), 5)

    def test_astar_rejects_unknown_or_blocked_goal(self) -> None:
        self.assertIsNone(
            astar(
                Position(0, 0),
                Position(1, 0),
                lambda position: position == Position(0, 0),
            )
        )


if __name__ == "__main__":
    unittest.main()
