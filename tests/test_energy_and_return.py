import unittest

from echorescue.config import SimulationConfig
from echorescue.events import EventType
from echorescue.models import CellState, DroneStatus
from echorescue.simulation import Simulation
from echorescue.visualization import render_text


class EnergyAndReturnTests(unittest.TestCase):
    def test_energy_consumption_is_deterministic_per_movement_and_sensing(self) -> None:
        config = SimulationConfig(
            seed=7,
            max_steps=1,
            battery_capacity=50.0,
            movement_energy_cost=2.0,
            sensor_energy_cost=0.25,
            energy_safety_reserve=5.0,
        )

        first = Simulation(config).run()
        second = Simulation(config).run()

        self.assertEqual(first, second)
        self.assertAlmostEqual(first.energy_consumed, 2.5)
        self.assertAlmostEqual(first.energy_remaining, 47.5)

    def test_return_starts_before_reserve_is_at_risk(self) -> None:
        config = SimulationConfig(
            seed=7,
            battery_capacity=40.0,
            movement_energy_cost=1.0,
            sensor_energy_cost=0.0,
            energy_safety_reserve=5.0,
        )
        simulation = Simulation(config)

        while simulation.drone.status is DroneStatus.EXPLORE:
            self.assertTrue(simulation.step())

        self.assertEqual(simulation.drone.status, DroneStatus.RETURN_HOME)
        self.assertIsNotNone(simulation.return_started_step)
        self.assertIsNone(simulation.active_frontier_target)
        self.assertIsNotNone(simulation.estimated_return_energy)
        assert simulation.estimated_return_energy is not None
        self.assertGreaterEqual(
            simulation.battery.remaining,
            simulation.estimated_return_energy + config.energy_safety_reserve,
        )

    def test_projected_energy_limit_triggers_early_but_safe_return(self) -> None:
        """Regression: RTB must start before frontier exhaustion."""

        config = SimulationConfig(
            seed=7,
            battery_capacity=40.0,
            movement_energy_cost=1.0,
            sensor_energy_cost=0.0,
            energy_safety_reserve=5.0,
        )
        simulation = Simulation(config)

        while simulation.drone.status is DroneStatus.EXPLORE:
            self.assertTrue(simulation.step())

        self.assertEqual(simulation.drone.status, DroneStatus.RETURN_HOME)
        self.assertTrue(simulation.occupancy_map.frontiers())
        self.assertLess(
            simulation.occupancy_map.explored_percent,
            100.0,
        )
        result = simulation.run()
        self.assertEqual(result.termination_reason, "returned_to_base")
        self.assertTrue(result.returned_to_base)
        self.assertFalse(result.energy_emergency)
        self.assertGreaterEqual(result.energy_remaining, config.energy_safety_reserve)

    def test_return_path_uses_only_known_free_cells_and_no_frontier_targets(self) -> None:
        simulation = Simulation(
            SimulationConfig(
                seed=7,
                battery_capacity=40.0,
                sensor_energy_cost=0.0,
                energy_safety_reserve=5.0,
            )
        )
        while simulation.drone.status is DroneStatus.EXPLORE:
            simulation.step()

        return_positions = []
        rendered_return = render_text(simulation)
        self.assertIn("state=RETURN_HOME", rendered_return)
        self.assertIn("battery=", rendered_return)
        self.assertIn("r", "".join(rendered_return.splitlines()[: simulation.config.height]))

        while simulation.drone.status is DroneStatus.RETURN_HOME:
            self.assertIsNone(simulation.active_frontier_target)
            self.assertTrue(
                all(
                    simulation.occupancy_map.cell_at(position) is CellState.FREE
                    for position in simulation.current_return_path
                )
            )
            previous_position = simulation.drone.position
            simulation.step()
            if simulation.drone.position != previous_position:
                return_positions.append(simulation.drone.position)

        self.assertEqual(len(return_positions), simulation.return_path_length)
        self.assertTrue(
            all(simulation.occupancy_map.is_known_free(p) for p in return_positions)
        )

    def test_success_requires_survivors_collision_free_flight_and_landing(self) -> None:
        simulation = Simulation(SimulationConfig(seed=7))
        result = simulation.run()

        self.assertEqual(result.drone_status, DroneStatus.LANDED)
        self.assertEqual(simulation.drone.position, simulation.world.base)
        self.assertTrue(result.returned_to_base)
        self.assertTrue(result.mission_success)
        self.assertFalse(result.energy_emergency)
        self.assertGreaterEqual(
            result.energy_remaining, simulation.config.energy_safety_reserve
        )
        event_types = [event.event_type for event in result.mission_events]
        self.assertEqual(event_types.count(EventType.RETURN_STARTED), 1)
        self.assertEqual(event_types.count(EventType.BASE_REACHED), 1)
        self.assertTrue(
            all(event.energy_remaining is not None for event in result.mission_events)
        )
        payload = result.to_dict()
        required_metrics = {
            "battery_capacity",
            "energy_consumed",
            "energy_remaining",
            "energy_remaining_percent",
            "return_started_step",
            "returned_to_base",
            "return_path_length",
            "energy_emergency",
            "mission_success",
        }
        self.assertTrue(required_metrics.issubset(payload))

    def test_changed_return_path_is_replanned(self) -> None:
        simulation = Simulation(
            SimulationConfig(
                seed=7,
                battery_capacity=40.0,
                sensor_energy_cost=0.0,
                energy_safety_reserve=5.0,
            )
        )
        while simulation.drone.status is DroneStatus.EXPLORE:
            simulation.step()

        blocked_next_step = simulation.current_return_path[1]
        simulation.occupancy_map.update(
            {blocked_next_step: CellState.OCCUPIED}
        )

        self.assertTrue(simulation.step())
        self.assertEqual(simulation.drone.status, DroneStatus.RETURN_HOME)
        self.assertNotEqual(simulation.drone.position, blocked_next_step)
        self.assertIn(
            EventType.RETURN_REPLANNED,
            [event.event_type for event in simulation.mission_log.events],
        )

    def test_early_safe_return_keeps_survivor_success_separate(self) -> None:
        result = Simulation(
            SimulationConfig(
                seed=7,
                battery_capacity=40.0,
                sensor_energy_cost=0.0,
                energy_safety_reserve=5.0,
            )
        ).run()

        self.assertTrue(result.returned_to_base)
        self.assertFalse(result.energy_emergency)
        self.assertLess(result.survivors_confirmed, result.survivors_total)
        self.assertFalse(result.mission_success)

    def test_too_little_energy_produces_explicit_emergency(self) -> None:
        result = Simulation(
            SimulationConfig(
                seed=7,
                battery_capacity=2.0,
                movement_energy_cost=1.0,
                sensor_energy_cost=0.0,
                energy_safety_reserve=1.0,
            )
        ).run()

        self.assertEqual(result.drone_status, DroneStatus.ENERGY_EMERGENCY)
        self.assertEqual(result.termination_reason, "energy_emergency")
        self.assertTrue(result.energy_emergency)
        self.assertFalse(result.returned_to_base)
        self.assertFalse(result.mission_success)
        self.assertEqual(
            [event.event_type for event in result.mission_events],
            [EventType.ENERGY_EMERGENCY],
        )

    def test_missing_known_return_path_has_explicit_status(self) -> None:
        simulation = Simulation(SimulationConfig(seed=7))
        simulation.occupancy_map.update(
            {simulation.world.base: CellState.OCCUPIED}
        )

        self.assertFalse(simulation.step())
        result = simulation.result()
        self.assertEqual(
            result.drone_status, DroneStatus.RETURN_PATH_UNAVAILABLE
        )
        self.assertEqual(result.termination_reason, "return_path_unavailable")
        self.assertFalse(result.mission_success)
        self.assertEqual(
            result.mission_events[-1].event_type,
            EventType.RETURN_PATH_UNAVAILABLE,
        )


if __name__ == "__main__":
    unittest.main()
