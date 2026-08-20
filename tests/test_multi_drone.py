import unittest

from echorescue.config import SimulationConfig
from echorescue.coordination import assign_frontiers, resolve_movements
from echorescue.events import EventType
from echorescue.mapping import OccupancyMap
from echorescue.models import CellState, DroneStatus, Position
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.visualization import render_text


def known_free_map(width: int = 7, height: int = 7) -> OccupancyMap:
    occupancy_map = OccupancyMap(width, height)
    occupancy_map.update(
        {
            Position(x, y): (
                CellState.OCCUPIED
                if x in (0, width - 1) or y in (0, height - 1)
                else CellState.FREE
            )
            for y in range(height)
            for x in range(width)
        }
    )
    return occupancy_map


class CoordinationTests(unittest.TestCase):
    def test_frontier_assignment_is_deterministic_and_distinct(self) -> None:
        occupancy_map = known_free_map()
        positions = {
            "drone-1": Position(1, 1),
            "drone-2": Position(5, 1),
        }
        frontiers = (Position(1, 5), Position(5, 5), Position(3, 5))

        first = assign_frontiers(
            positions, frontiers, occupancy_map, {key: None for key in positions}
        )
        second = assign_frontiers(
            positions, frontiers, occupancy_map, {key: None for key in positions}
        )

        self.assertEqual(first, second)
        self.assertEqual(first["drone-1"].target, Position(1, 5))
        self.assertEqual(first["drone-2"].target, Position(5, 5))
        self.assertEqual(len({item.target for item in first.values()}), 2)

    def test_route_does_not_cross_other_drones_current_cell(self) -> None:
        occupancy_map = known_free_map(width=5, height=5)
        positions = {
            "drone-1": Position(1, 2),
            "drone-2": Position(2, 2),
        }
        assignments = assign_frontiers(
            positions,
            (Position(3, 2),),
            occupancy_map,
            {key: None for key in positions},
        )

        if "drone-1" in assignments:
            self.assertNotIn(Position(2, 2), assignments["drone-1"].path)

    def test_vertex_conflict_has_stable_drone_one_priority(self) -> None:
        current = {
            "drone-1": Position(1, 1),
            "drone-2": Position(3, 1),
        }
        intended = {
            "drone-1": Position(2, 1),
            "drone-2": Position(2, 1),
        }

        resolved, conflicts = resolve_movements(
            current, intended, base=Position(1, 2)
        )

        self.assertEqual(resolved["drone-1"], Position(2, 1))
        self.assertEqual(resolved["drone-2"], Position(3, 1))
        self.assertEqual(conflicts[0].reason, "vertex")

    def test_edge_swap_makes_both_drones_wait(self) -> None:
        current = {
            "drone-1": Position(1, 1),
            "drone-2": Position(2, 1),
        }
        intended = {
            "drone-1": Position(2, 1),
            "drone-2": Position(1, 1),
        }

        resolved, conflicts = resolve_movements(
            current, intended, base=Position(1, 2)
        )

        self.assertEqual(resolved, current)
        self.assertEqual(conflicts[0].reason, "edge_swap")


class MultiDroneSimulationTests(unittest.TestCase):
    def test_targets_remain_distinct_and_mission_is_collision_free(self) -> None:
        simulation = MultiDroneSimulation(
            SimulationConfig(seed=7, drone_count=2)
        )

        while not simulation.completed:
            targets = [
                runtime.active_frontier_target
                for runtime in simulation.runtimes.values()
                if runtime.drone.status is DroneStatus.EXPLORE
                and runtime.active_frontier_target is not None
            ]
            self.assertEqual(len(targets), len(set(targets)))
            simulation.step()

        result = simulation.result()
        self.assertEqual(result.collisions, 0)
        self.assertEqual(result.drone_drone_collisions, 0)
        self.assertEqual(result.drones_returned, 2)
        self.assertTrue(result.mission_success)

    def test_traces_have_no_vertex_or_edge_swap_collisions(self) -> None:
        result = MultiDroneSimulation(
            SimulationConfig(seed=11, drone_count=2)
        ).run()
        first = result.position_trace_by_drone["drone-1"]
        second = result.position_trace_by_drone["drone-2"]

        for step in range(min(len(first), len(second))):
            if first[step] != Position(1, 1):
                self.assertNotEqual(first[step], second[step])
        for step in range(1, min(len(first), len(second))):
            swapped = (
                first[step] == second[step - 1]
                and second[step] == first[step - 1]
                and first[step] != first[step - 1]
            )
            self.assertFalse(swapped)

    def test_batteries_and_return_states_are_independent(self) -> None:
        simulation = MultiDroneSimulation(
            SimulationConfig(seed=7, drone_count=2)
        )
        for _ in range(5):
            simulation.step()
        first = simulation.runtimes["drone-1"]
        second = simulation.runtimes["drone-2"]
        path = simulation._refresh_return_estimate(first)
        assert path is not None
        assert first.estimated_return_energy is not None
        first.battery.remaining = (
            first.estimated_return_energy
            + simulation.config.energy_safety_reserve
            - 0.1
        )

        assignments_before = first.frontier_assignments
        simulation.step()

        self.assertEqual(first.drone.status, DroneStatus.RETURN_HOME)
        self.assertEqual(second.drone.status, DroneStatus.EXPLORE)
        self.assertNotEqual(first.battery.remaining, second.battery.remaining)
        self.assertIsNone(first.active_frontier_target)
        simulation.step()
        self.assertEqual(first.frontier_assignments, assignments_before)

    def test_mission_completion_waits_for_both_drones(self) -> None:
        simulation = MultiDroneSimulation(
            SimulationConfig(seed=7, drone_count=2)
        )

        while not any(
            runtime.drone.status is DroneStatus.LANDED
            for runtime in simulation.runtimes.values()
        ):
            simulation.step()

        self.assertFalse(simulation.completed)
        self.assertTrue(
            any(
                runtime.drone.status is DroneStatus.RETURN_HOME
                for runtime in simulation.runtimes.values()
            )
        )
        result = simulation.run()
        self.assertEqual(result.drones_returned, 2)

    def test_shared_base_launch_and_virtual_docking_are_safe(self) -> None:
        simulation = MultiDroneSimulation(
            SimulationConfig(
                seed=7,
                drone_count=2,
                drone_start_positions=((1, 1), (1, 1)),
            )
        )
        result = simulation.run()

        self.assertEqual(result.drone_drone_collisions, 0)
        self.assertEqual(result.drones_returned, 2)
        self.assertTrue(result.mission_success)
        self.assertTrue(
            all(
                runtime.drone.position == simulation.world.base
                for runtime in simulation.runtimes.values()
            )
        )

    def test_same_seed_repeats_paths_events_and_metrics(self) -> None:
        config = SimulationConfig(seed=19, drone_count=2)

        first = MultiDroneSimulation(config).run()
        second = MultiDroneSimulation(config).run()

        self.assertEqual(first, second)

    def test_normal_return_progress_does_not_emit_replan_spam(self) -> None:
        result = MultiDroneSimulation(
            SimulationConfig(seed=7, drone_count=2)
        ).run()

        replans = [
            event
            for event in result.mission_events
            if event.event_type is EventType.RETURN_REPLANNED
        ]
        self.assertEqual(replans, [])

    def test_new_metrics_and_events_include_both_stable_ids(self) -> None:
        result = MultiDroneSimulation(
            SimulationConfig(seed=7, drone_count=2)
        ).run()
        payload = result.to_dict()

        self.assertEqual(result.drones_total, 2)
        self.assertEqual(set(result.path_length_by_drone), {"drone-1", "drone-2"})
        self.assertEqual(
            set(result.energy_remaining_by_drone), {"drone-1", "drone-2"}
        )
        self.assertIn("duplicate_exploration_ratio", payload)
        return_ids = {
            event.drone_id
            for event in result.mission_events
            if event.event_type is EventType.RETURN_STARTED
        }
        self.assertEqual(return_ids, {"drone-1", "drone-2"})

    def test_visualization_distinguishes_drones_without_ground_truth_leaks(self) -> None:
        simulation = MultiDroneSimulation(
            SimulationConfig(seed=7, drone_count=2)
        )
        initial = render_text(simulation, show_ground_truth=True)
        initial_grid = initial.splitlines()[: simulation.config.height]

        self.assertIn("1", "".join(initial_grid))
        self.assertIn("2", "".join(initial_grid))
        self.assertNotIn("S", "".join(initial_grid))
        self.assertIn("drone-1: state=EXPLORE", initial)
        self.assertIn("drone-2: state=EXPLORE", initial)

        while not any(
            runtime.drone.status is DroneStatus.RETURN_HOME
            for runtime in simulation.runtimes.values()
        ):
            simulation.step()
        returning = render_text(simulation)
        self.assertTrue("r" in returning or "q" in returning)


if __name__ == "__main__":
    unittest.main()
