import unittest

from echorescue.config import SimulationConfig
from echorescue.events import EventType
from echorescue.simulation import Simulation
from echorescue.visualization import render_text


class SimulationTests(unittest.TestCase):
    def test_vertical_slice_completes_without_collisions(self) -> None:
        simulation = Simulation(SimulationConfig(seed=7))
        result = simulation.run()

        self.assertTrue(result.completed)
        self.assertEqual(result.termination_reason, "exploration_complete")
        self.assertEqual(result.collisions, 0)
        self.assertGreater(result.path_length, 0)
        self.assertTrue(all(simulation.world.is_free(p) for p in result.position_trace))

    def test_survivors_are_detected_then_confirmed_once(self) -> None:
        result = Simulation(SimulationConfig(seed=7)).run()

        self.assertEqual(result.survivors_total, 3)
        self.assertEqual(result.survivors_detected, 3)
        self.assertEqual(result.survivors_confirmed, 3)
        self.assertEqual(result.survivor_recall, 1.0)
        self.assertIsNotNone(result.time_to_first_detection)
        payload = result.to_dict()
        self.assertEqual(payload["survivors_total"], 3)
        self.assertEqual(payload["survivors_detected"], 3)
        self.assertEqual(payload["survivors_confirmed"], 3)
        self.assertEqual(payload["survivor_recall"], 1.0)
        self.assertEqual(payload["time_to_first_detection"], result.time_to_first_detection)
        self.assertTrue(
            all(
                set(event) == {"position", "step", "drone_id", "event_type"}
                | {"energy_remaining"}
                for event in payload["mission_events"]
            )
        )

        for survivor in {
            event.position
            for event in result.mission_events
            if event.event_type is EventType.SURVIVOR_DETECTED
        }:
            survivor_events = [
                event for event in result.mission_events if event.position == survivor
            ]
            self.assertEqual(
                [event.event_type for event in survivor_events],
                [EventType.SURVIVOR_DETECTED, EventType.SURVIVOR_CONFIRMED],
            )
            self.assertLess(survivor_events[0].step, survivor_events[1].step)
            self.assertTrue(all(event.drone_id == "drone-1" for event in survivor_events))

    def test_visualization_never_reveals_unconfirmed_survivors(self) -> None:
        simulation = Simulation(
            SimulationConfig(seed=7, survivor_confirmation_observations=3)
        )
        initial_grid = render_text(simulation, show_ground_truth=True).splitlines()[
            : simulation.config.height
        ]

        self.assertNotIn("S", "".join(initial_grid))

        simulation.run()
        simulation.drone.position = simulation.world.base
        final_grid = render_text(simulation, show_ground_truth=False).splitlines()[
            : simulation.config.height
        ]
        self.assertEqual(
            "".join(final_grid).count("S"), len(simulation.confirmed_survivors)
        )

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
