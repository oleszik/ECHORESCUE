import unittest

from echorescue.config import SimulationConfig
from echorescue.simulation import Simulation


class SimulationTests(unittest.TestCase):
    def test_vertical_slice_completes_without_collisions(self) -> None:
        simulation = Simulation(SimulationConfig(seed=7))
        result = simulation.run()

        self.assertTrue(result.completed)
        self.assertEqual(result.termination_reason, "exploration_complete")
        self.assertEqual(result.collisions, 0)
        self.assertGreater(result.path_length, 0)
        self.assertTrue(all(simulation.world.is_free(p) for p in result.position_trace))

    def test_same_seed_has_identical_trace_and_metrics(self) -> None:
        config = SimulationConfig(seed=19)
        first = Simulation(config).run()
        second = Simulation(config).run()

        self.assertEqual(first, second)

    def test_max_steps_is_reported(self) -> None:
        result = Simulation(SimulationConfig(seed=7, max_steps=1)).run()

        self.assertEqual(result.termination_reason, "max_steps")
        self.assertEqual(result.steps, 1)


if __name__ == "__main__":
    unittest.main()

