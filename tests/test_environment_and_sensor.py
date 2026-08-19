import unittest

from echorescue.config import SimulationConfig
from echorescue.environment import GridWorld
from echorescue.models import CellState, Position
from echorescue.sensors import DistanceSensor


class EnvironmentAndSensorTests(unittest.TestCase):
    def test_world_generation_is_seeded(self) -> None:
        first = GridWorld.generate(SimulationConfig(seed=42))
        second = GridWorld.generate(SimulationConfig(seed=42))
        different = GridWorld.generate(SimulationConfig(seed=43))

        self.assertEqual(first.walls, second.walls)
        self.assertNotEqual(first.walls, different.walls)

    def test_sensor_stops_at_wall(self) -> None:
        world = GridWorld(
            width=7,
            height=7,
            base=Position(1, 1),
            walls=frozenset({Position(3, 2)}),
        )
        observations = DistanceSensor(max_range=4).observe(world, Position(1, 2))

        self.assertEqual(observations[Position(2, 2)], CellState.FREE)
        self.assertEqual(observations[Position(3, 2)], CellState.OCCUPIED)
        self.assertNotIn(Position(4, 2), observations)


if __name__ == "__main__":
    unittest.main()

