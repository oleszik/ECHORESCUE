import unittest
from pathlib import Path

from echorescue.config import SimulationConfig
from echorescue.dynamic_obstacles import DynamicObstacleEvent
from echorescue.dynamic_obstacle_benchmark import run_benchmark
from echorescue.environment import GridWorld
from echorescue.events import EventType
from echorescue.dashboard import _validate_replay
from echorescue.knowledge import KnowledgeMap
from echorescue.mapping import OccupancyMap
from echorescue.models import CellState, DroneStatus, Position
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.replay import (
    DYNAMIC_OBSTACLE_REPLAY_SCHEMA_VERSION,
    generate_replay,
    replay_json_bytes,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def far_free_cell(simulation: MultiDroneSimulation) -> Position:
    return max(
        (
            Position(x, y)
            for y in range(1, simulation.config.height - 1)
            for x in range(1, simulation.config.width - 1)
            if simulation.world.is_free(Position(x, y))
            and Position(x, y) not in simulation.world.survivors
        ),
        key=lambda position: (
            abs(position.x - simulation.world.base.x)
            + abs(position.y - simulation.world.base.y),
            position,
        ),
    )


class DynamicObstacleConfigurationTests(unittest.TestCase):
    def test_off_preserves_legacy_replay_bytes(self) -> None:
        generated = replay_json_bytes(
            generate_replay(SimulationConfig(seed=7, drone_count=2))
        )
        expected = (
            REPOSITORY_ROOT / "replays" / "seed_7.json"
        ).read_bytes().replace(b"\r\n", b"\n")
        self.assertEqual(generated, expected)

    def test_invalid_injections_are_rejected(self) -> None:
        invalid = (
            ((0, 2, 4),),
            ((1, 1, 4),),
            ((2, 2, 0),),
            ((2, 2, 4), (2, 2, 8)),
        )
        for schedule in invalid:
            with self.subTest(schedule=schedule), self.assertRaises(ValueError):
                SimulationConfig(dynamic_obstacle_schedule=schedule)
        with self.assertRaises(ValueError):
            SimulationConfig(
                dynamic_obstacles="moderate",
                dynamic_obstacle_schedule=((3, 3, 5),),
            )

    def test_initial_wall_and_survivor_injections_are_rejected(self) -> None:
        template = MultiDroneSimulation(SimulationConfig(seed=7, drone_count=2))
        wall = next(
            position
            for position in template.world.walls
            if 0 < position.x < template.config.width - 1
            and 0 < position.y < template.config.height - 1
        )
        survivor = next(iter(template.world.survivors))
        for position in (wall, survivor):
            config = SimulationConfig(
                seed=7,
                drone_count=2,
                dynamic_obstacle_schedule=((position.x, position.y, 5),),
            )
            with self.subTest(position=position), self.assertRaises(ValueError):
                MultiDroneSimulation(config)


class MutableWorldAndKnowledgeTests(unittest.TestCase):
    def test_ground_truth_changes_at_the_configured_step_only(self) -> None:
        template = MultiDroneSimulation(SimulationConfig(seed=12, drone_count=1))
        location = far_free_cell(template)
        simulation = MultiDroneSimulation(
            SimulationConfig(
                seed=12,
                drone_count=1,
                dynamic_obstacle_schedule=((location.x, location.y, 2),),
            )
        )
        self.assertTrue(simulation.world.is_free(location))
        simulation.step()
        self.assertTrue(simulation.world.is_free(location))
        simulation.step()
        self.assertFalse(simulation.world.is_free(location))
        injected = [
            event
            for event in simulation.mission_log.events
            if event.event_type is EventType.DYNAMIC_OBSTACLE_INJECTED
        ]
        self.assertEqual([(event.position, event.step) for event in injected], [(location, 2)])

    def test_agent_does_not_know_blockage_until_observation(self) -> None:
        template = MultiDroneSimulation(SimulationConfig(seed=13, drone_count=1))
        location = far_free_cell(template)
        simulation = MultiDroneSimulation(
            SimulationConfig(
                seed=13,
                drone_count=1,
                dynamic_obstacle_schedule=((location.x, location.y, 1),),
            )
        )
        simulation.step()
        self.assertFalse(simulation.world.is_free(location))
        self.assertIsNot(
            simulation.occupancy_map.cell_at(location), CellState.OCCUPIED
        )
        while location not in simulation._dynamic_obstacles_observed:
            self.assertTrue(simulation.step())
        self.assertIs(
            simulation.occupancy_map.cell_at(location), CellState.OCCUPIED
        )

    def test_free_to_occupied_overwrites_older_knowledge(self) -> None:
        occupancy = OccupancyMap(7, 7)
        location = Position(3, 3)
        occupancy.update({location: CellState.FREE})
        occupancy.update({location: CellState.OCCUPIED})
        self.assertIs(occupancy.cell_at(location), CellState.OCCUPIED)

        knowledge = KnowledgeMap(7, 7)
        knowledge.observe(
            {location: CellState.FREE}, step=2, source_id="drone-1"
        )
        knowledge.observe(
            {location: CellState.OCCUPIED}, step=7, source_id="drone-2"
        )
        record = knowledge.record_at(location)
        self.assertIsNotNone(record)
        assert record is not None
        self.assertIs(record.state, CellState.OCCUPIED)
        self.assertEqual(record.observed_step, 7)


class DynamicReplanningIntegrationTests(unittest.TestCase):
    def test_small_paired_benchmark_is_deterministic_and_safe(self) -> None:
        first = run_benchmark(seeds=(50, 51), four_agent_seeds=(0,), repeat=True)
        second = run_benchmark(seeds=(50, 51), four_agent_seeds=(0,), repeat=True)
        self.assertEqual(first, second)
        self.assertTrue(first["acceptance"]["deterministic_dynamic_repeats"])
        self.assertTrue(first["acceptance"]["collision_free"])
        self.assertEqual(len(first["per_seed"]), 2)

    def test_future_path_cells_invalidate_and_replan(self) -> None:
        result = MultiDroneSimulation(
            SimulationConfig(seed=3, drone_count=2, dynamic_obstacles="moderate")
        ).run()
        metrics = result.dynamic_obstacle_metrics
        self.assertIsNotNone(metrics)
        assert metrics is not None
        self.assertGreaterEqual(metrics["path_invalidations"], 1)
        self.assertEqual(metrics["failed_replans"], 0)
        self.assertEqual(
            metrics["successful_replans"], metrics["replans_total"]
        )
        self.assertGreaterEqual(metrics["target_reassignments"], 1)
        self.assertGreaterEqual(metrics["rtb_replans"], 1)
        self.assertTrue(result.mission_success)

    def test_no_return_path_uses_existing_failure_state(self) -> None:
        simulation = MultiDroneSimulation(
            SimulationConfig(
                width=7,
                height=7,
                seed=4,
                drone_count=1,
                survivor_count=0,
            )
        )
        corridor = {Position(1, 1), Position(2, 1), Position(3, 1)}
        boundary = {
            Position(x, y)
            for y in range(7)
            for x in range(7)
            if x in (0, 6) or y in (0, 6)
        }
        interior_walls = {
            Position(x, y)
            for y in range(1, 6)
            for x in range(1, 6)
            if Position(x, y) not in corridor
        }
        simulation.world = GridWorld(
            7, 7, Position(1, 1), frozenset(boundary | interior_walls)
        )
        runtime = simulation.runtimes["drone-1"]
        runtime.drone.position = Position(3, 1)
        runtime.drone.status = DroneStatus.RETURN_HOME
        runtime.current_return_path = (
            Position(3, 1),
            Position(2, 1),
            Position(1, 1),
        )
        runtime.local_map = KnowledgeMap(7, 7)
        runtime.local_map.observe(
            {position: CellState.FREE for position in corridor},
            step=0,
            source_id="drone-1",
        )
        simulation.occupancy_map = OccupancyMap(7, 7)
        simulation.occupancy_map.update(
            {position: CellState.FREE for position in corridor}
        )
        simulation.world.block_cell(Position(2, 1))
        simulation._dynamic_obstacles_injected.add(Position(2, 1))
        simulation.dynamic_obstacle_events = (
            DynamicObstacleEvent(Position(2, 1), 99, "stress_test"),
        )
        simulation.occupancy_map.update(
            {Position(2, 1): CellState.OCCUPIED}
        )
        simulation._invalidate_dynamic_paths(
            runtime, {Position(2, 1)}
        )
        simulation._plan_return_intention(runtime)
        self.assertIs(runtime.drone.status, DroneStatus.RETURN_PATH_UNAVAILABLE)
        self.assertEqual(simulation._failed_replans, 1)

    def test_controlled_replanning_stress_test_finds_return_detour(self) -> None:
        simulation = MultiDroneSimulation(
            SimulationConfig(
                width=7,
                height=7,
                seed=4,
                drone_count=1,
                survivor_count=0,
            )
        )
        old_route = {Position(1, 1), Position(2, 1), Position(3, 1)}
        detour = {
            Position(3, 2),
            Position(2, 2),
            Position(1, 2),
        }
        free = old_route | detour
        boundary = {
            Position(x, y)
            for y in range(7)
            for x in range(7)
            if x in (0, 6) or y in (0, 6)
        }
        interior_walls = {
            Position(x, y)
            for y in range(1, 6)
            for x in range(1, 6)
            if Position(x, y) not in free
        }
        simulation.world = GridWorld(
            7, 7, Position(1, 1), frozenset(boundary | interior_walls)
        )
        runtime = simulation.runtimes["drone-1"]
        runtime.drone.position = Position(3, 1)
        runtime.drone.status = DroneStatus.RETURN_HOME
        runtime.current_return_path = (
            Position(3, 1),
            Position(2, 1),
            Position(1, 1),
        )
        simulation.occupancy_map = OccupancyMap(7, 7)
        simulation.occupancy_map.update(
            {position: CellState.FREE for position in free}
        )
        blocked = Position(2, 1)
        simulation.world.block_cell(blocked)
        simulation._dynamic_obstacles_injected.add(blocked)
        simulation.dynamic_obstacle_events = (
            DynamicObstacleEvent(blocked, 99, "controlled_stress_test"),
        )
        simulation.occupancy_map.update({blocked: CellState.OCCUPIED})
        simulation._invalidate_dynamic_paths(runtime, {blocked})
        intention = simulation._plan_return_intention(runtime)
        self.assertEqual(intention, Position(3, 2))
        self.assertEqual(simulation._successful_replans, 1)
        self.assertEqual(simulation._failed_replans, 0)
        self.assertEqual(simulation._rtb_replans, 1)
        self.assertIs(runtime.drone.status, DroneStatus.RETURN_HOME)

    def test_stale_path_contact_is_blocked_without_collision(self) -> None:
        simulation = MultiDroneSimulation(
            SimulationConfig(seed=7, drone_count=1, survivor_count=0)
        )
        runtime = simulation.runtimes["drone-1"]
        destination = next(
            neighbor
            for neighbor in runtime.drone.position.neighbors()
            if simulation.world.is_free(neighbor)
            and neighbor not in simulation.world.survivors
        )
        simulation.dynamic_obstacle_events = (
            DynamicObstacleEvent(destination, 99, "stress_test"),
        )
        simulation.world.block_cell(destination)
        simulation._dynamic_obstacles_injected.add(destination)
        simulation.occupancy_map.update({destination: CellState.FREE})
        runtime.planned_path = (runtime.drone.position, destination)
        runtime.active_frontier_target = destination
        simulation._execute_intentions({"drone-1": destination})
        self.assertNotEqual(runtime.drone.position, destination)
        self.assertEqual(simulation.collisions, 0)
        self.assertEqual(simulation._stale_path_safety_interventions, 1)
        self.assertIs(
            simulation.occupancy_map.cell_at(destination), CellState.OCCUPIED
        )

    def test_dynamic_runs_are_deterministic_and_n_agent_safe(self) -> None:
        for drone_count in (1, 2, 4, 8):
            config = SimulationConfig(
                seed=3,
                drone_count=drone_count,
                dynamic_obstacles="moderate",
            )
            first = MultiDroneSimulation(config).run()
            second = MultiDroneSimulation(config).run()
            with self.subTest(drone_count=drone_count):
                self.assertEqual(first, second)
                self.assertEqual(first.collisions, 0)
                self.assertEqual(first.drone_drone_collisions, 0)
                self.assertEqual(first.drones_returned, drone_count)

    def test_failure_and_dynamic_obstacles_coexist(self) -> None:
        result = MultiDroneSimulation(
            SimulationConfig(
                seed=3,
                drone_count=2,
                failure_schedule=(("drone-2", 4),),
                dynamic_obstacles="moderate",
            )
        ).run()
        self.assertEqual(result.collisions, 0)
        self.assertEqual(result.drone_drone_collisions, 0)
        self.assertIsNotNone(result.dynamic_obstacle_metrics)
        self.assertIsNotNone(result.failure_recovery_metrics)

    def test_replay_is_operator_safe_and_dashboard_ready(self) -> None:
        replay = generate_replay(
            SimulationConfig(seed=3, drone_count=2, dynamic_obstacles="moderate")
        )
        self.assertEqual(
            replay["schema_version"], DYNAMIC_OBSTACLE_REPLAY_SCHEMA_VERSION
        )
        observed = set()
        for frame in replay["frames"]:
            observed.update(
                tuple(position)
                for position in frame.get("dynamic_obstacles_observed", [])
            )
            self.assertFalse(
                any(
                    event["event_type"] == "dynamic_obstacle_injected"
                    for event in frame["events"]
                )
            )
        self.assertTrue(observed)
        app = (
            REPOSITORY_ROOT
            / "src"
            / "echorescue"
            / "dashboard_assets"
            / "app.js"
        ).read_text(encoding="utf-8")
        self.assertIn("dynamic_obstacles_observed", app)
        self.assertIn("replan_succeeded", app)
        replay_path = REPOSITORY_ROOT / "replays" / "seed_51_dynamic_obstacles.json"
        if replay_path.exists():
            _validate_replay(replay_path)


if __name__ == "__main__":
    unittest.main()
