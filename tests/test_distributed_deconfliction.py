import json
import unittest
from pathlib import Path
from unittest.mock import patch

from echorescue.config import SimulationConfig
from echorescue.deconfliction import ProximitySensor
from echorescue.environment import GridWorld
from echorescue.events import EventType
from echorescue.mapping import OccupancyMap
from echorescue.models import CellState, DroneStatus, Position
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.replay import REPLAY_SCHEMA_VERSION, generate_replay


def local_simulation(**overrides: object) -> MultiDroneSimulation:
    values = {
        "width": 9,
        "height": 7,
        "seed": 7,
        "drone_count": 2,
        "obstacle_density": 0.0,
        "communication_range": 20,
        "knowledge_mode": "local",
    }
    values.update(overrides)
    return MultiDroneSimulation(SimulationConfig(**values))


def prepare_conflict(
    simulation: MultiDroneSimulation,
    first: Position,
    second: Position,
    known: set[Position],
) -> None:
    simulation.runtimes["drone-1"].drone.position = first
    simulation.runtimes["drone-2"].drone.position = second
    for runtime in simulation.runtimes.values():
        runtime.local_map.observe(
            {position: CellState.FREE for position in known},
            step=1,
            source_id=runtime.drone.identifier,
        )
    simulation.steps = 1
    simulation._sample_communication(record_events=False)


class ProximitySensorTests(unittest.TestCase):
    def test_detects_around_an_open_corner_without_crossing_a_wall(self) -> None:
        boundary = {
            Position(x, y)
            for y in range(7)
            for x in range(7)
            if x in (0, 6) or y in (0, 6)
        }
        world = GridWorld(
            7,
            7,
            Position(1, 1),
            frozenset(boundary | {Position(2, 3)}),
        )

        self.assertTrue(
            ProximitySensor(2).can_detect(
                world, Position(2, 2), Position(3, 3)
            )
        )

    def test_does_not_detect_through_a_wall(self) -> None:
        boundary = {
            Position(x, y)
            for y in range(7)
            for x in range(7)
            if x in (0, 6) or y in (0, 6)
        }
        world = GridWorld(
            7,
            7,
            Position(1, 1),
            frozenset(boundary | {Position(3, 2)}),
        )

        self.assertFalse(
            ProximitySensor(2).can_detect(
                world, Position(2, 2), Position(4, 2)
            )
        )


class DistributedDeconflictionTests(unittest.TestCase):
    def test_versioned_50_seed_benchmark_meets_acceptance(self) -> None:
        artifact = (
            Path(__file__).resolve().parents[1]
            / "benchmarks"
            / "distributed_deconfliction_50_seeds.json"
        )
        payload = json.loads(artifact.read_text(encoding="utf-8"))

        self.assertEqual(payload["schema_version"], "1.0")
        self.assertEqual(payload["suite"]["seed_count"], 50)
        self.assertEqual(payload["suite"]["determinism_check"], "passed")
        self.assertTrue(all(payload["suite"]["acceptance"].values()))
        self.assertEqual(payload["remaining_shield_cases"], [])

    def test_replay_exposes_intents_and_yield_state(self) -> None:
        replay = generate_replay(
            SimulationConfig(
                seed=16,
                drone_count=2,
                communication_range=8,
                knowledge_mode="local",
            )
        )

        self.assertEqual(replay["schema_version"], REPLAY_SCHEMA_VERSION)
        self.assertTrue(
            any(
                drone["motion_intent"] is not None
                for frame in replay["frames"]
                for drone in frame["drones"].values()
            )
        )
        for frame in replay["frames"]:
            for drone in frame["drones"].values():
                self.assertIn("yielding", drone)
                self.assertIn("motion_intent", drone)

    def test_vertex_conflict_is_prevented_before_the_safety_shield(self) -> None:
        simulation = local_simulation()
        destination = Position(3, 2)
        prepare_conflict(
            simulation,
            Position(2, 2),
            Position(4, 2),
            {Position(2, 2), destination, Position(4, 2)},
        )

        resolved = simulation._deconflict_intentions(
            {"drone-1": destination, "drone-2": destination}
        )
        simulation._execute_intentions(resolved)

        self.assertEqual(simulation.safety_shield_interventions, 0)
        self.assertEqual(simulation.local_motion_conflicts, 1)
        self.assertEqual(simulation.communication_detected_conflicts, 1)
        self.assertEqual(
            sum(runtime.drone.position == destination for runtime in simulation.runtimes.values()),
            1,
        )

    def test_edge_swap_is_prevented_locally(self) -> None:
        simulation = local_simulation()
        first = Position(2, 2)
        second = Position(3, 2)
        prepare_conflict(simulation, first, second, {first, second})

        resolved = simulation._deconflict_intentions(
            {"drone-1": second, "drone-2": first}
        )
        simulation._execute_intentions(resolved)

        self.assertEqual(
            {drone_id: runtime.drone.position for drone_id, runtime in simulation.runtimes.items()},
            {"drone-1": first, "drone-2": second},
        )
        self.assertEqual(simulation.safety_shield_interventions, 0)

    def test_return_home_with_critical_margin_receives_priority(self) -> None:
        simulation = local_simulation()
        destination = Position(3, 2)
        prepare_conflict(
            simulation,
            Position(2, 2),
            Position(4, 2),
            {Position(2, 2), destination, Position(4, 2)},
        )
        returning = simulation.runtimes["drone-2"]
        returning.drone.status = DroneStatus.RETURN_HOME
        returning.current_return_path = (
            returning.drone.position,
            destination,
            simulation.world.base,
        )
        returning.estimated_return_energy = returning.battery.remaining

        resolved = simulation._deconflict_intentions(
            {"drone-1": destination, "drone-2": destination}
        )

        self.assertEqual(resolved["drone-2"], destination)
        self.assertEqual(
            resolved["drone-1"],
            simulation.runtimes["drone-1"].drone.position,
        )

    def test_proximity_prevents_conflict_without_radio(self) -> None:
        simulation = local_simulation(communication_range=1)
        boundary = {
            Position(x, y)
            for y in range(simulation.config.height)
            for x in range(simulation.config.width)
            if x in (0, simulation.config.width - 1)
            or y in (0, simulation.config.height - 1)
        }
        simulation.world = GridWorld(
            simulation.config.width,
            simulation.config.height,
            Position(1, 1),
            frozenset(boundary | {Position(2, 3)}),
        )
        destination = Position(3, 2)
        first = Position(2, 2)
        second = Position(3, 3)
        prepare_conflict(
            simulation, first, second, {first, destination, second}
        )

        resolved = simulation._deconflict_intentions(
            {"drone-1": destination, "drone-2": destination}
        )

        self.assertFalse(simulation._peer_intents_communicated())
        self.assertEqual(simulation.proximity_detected_conflicts, 1)
        self.assertEqual(
            sum(position == destination for position in resolved.values()), 1
        )

    def test_repeated_corridor_block_is_replanned_deterministically(self) -> None:
        def exercise() -> tuple[dict[str, Position], tuple[EventType, ...]]:
            simulation = local_simulation(deadlock_wait_threshold=3)
            first = Position(2, 2)
            second = Position(3, 2)
            known = {
                first,
                second,
                Position(2, 1),
                Position(2, 3),
                Position(3, 1),
                Position(3, 3),
                Position(4, 2),
            }
            prepare_conflict(simulation, first, second, known)
            resolved = {}
            for _ in range(3):
                resolved = simulation._deconflict_intentions(
                    {"drone-1": second, "drone-2": first}
                )
            return resolved, tuple(
                event.event_type for event in simulation.mission_log.events
            )

        first = exercise()
        second = exercise()

        self.assertEqual(first, second)
        self.assertIn(EventType.CORRIDOR_DEADLOCK_DETECTED, first[1])
        self.assertIn(EventType.DEADLOCK_REPLANNED, first[1])

    def test_narrow_corridor_with_passing_bay_resolves_without_shield(self) -> None:
        simulation = local_simulation(deadlock_wait_threshold=3)
        first = Position(3, 3)
        second = Position(4, 3)
        corridor = {Position(x, 3) for x in range(1, 8)}
        passing_bay = Position(4, 2)
        free = corridor | {passing_bay, simulation.world.base}
        walls = {
            Position(x, y)
            for y in range(simulation.config.height)
            for x in range(simulation.config.width)
            if Position(x, y) not in free
        }
        simulation.world = GridWorld(
            simulation.config.width,
            simulation.config.height,
            simulation.world.base,
            frozenset(walls),
        )
        prepare_conflict(simulation, first, second, free)
        simulation.runtimes["drone-1"].planned_path = (
            first,
            second,
            Position(5, 3),
        )
        simulation.runtimes["drone-2"].planned_path = (
            second,
            first,
            Position(2, 3),
        )
        for runtime in simulation.runtimes.values():
            runtime.active_frontier_target = None
            runtime.estimated_return_energy = 0.0

        resolved = {}
        for _ in range(3):
            resolved = simulation._deconflict_intentions(
                {"drone-1": second, "drone-2": first}
            )
        simulation._execute_intentions(resolved)

        positions = {
            runtime.drone.position for runtime in simulation.runtimes.values()
        }
        self.assertEqual(len(positions), 2)
        self.assertEqual(simulation.corridor_deadlocks, 1)
        self.assertEqual(simulation.deadlocks_resolved, 1)
        self.assertEqual(simulation.safety_shield_interventions, 0)

    def test_wait_priority_prevents_permanent_drone_two_starvation(self) -> None:
        simulation = local_simulation(deadlock_wait_threshold=4)
        destination = Position(3, 2)
        prepare_conflict(
            simulation,
            Position(2, 2),
            Position(4, 2),
            {Position(2, 2), destination, Position(4, 2)},
        )
        for runtime in simulation.runtimes.values():
            runtime.estimated_return_energy = 0.0

        first = simulation._deconflict_intentions(
            {"drone-1": destination, "drone-2": destination}
        )
        second = simulation._deconflict_intentions(
            {"drone-1": destination, "drone-2": destination}
        )

        self.assertEqual(first["drone-2"], Position(4, 2))
        self.assertEqual(second["drone-2"], destination)

    def test_intent_events_are_transition_deduplicated(self) -> None:
        simulation = local_simulation(deadlock_wait_threshold=4)
        destination = Position(3, 2)
        prepare_conflict(
            simulation,
            Position(2, 2),
            Position(4, 2),
            {Position(2, 2), destination, Position(4, 2)},
        )
        simulation.runtimes["drone-1"].estimated_return_energy = 10.0
        for _ in range(2):
            simulation._deconflict_intentions(
                {"drone-1": destination, "drone-2": destination}
            )

        events = simulation.mission_log.events
        self.assertEqual(
            sum(event.event_type is EventType.MOTION_INTENT_SHARED for event in events),
            2,
        )
        self.assertEqual(
            sum(
                event.event_type is EventType.LOCAL_COLLISION_AVOIDED
                for event in events
            ),
            1,
        )

    def test_local_deconfliction_never_reads_the_global_operator_map(self) -> None:
        simulation = local_simulation()
        destination = Position(3, 2)
        prepare_conflict(
            simulation,
            Position(2, 2),
            Position(4, 2),
            {Position(2, 2), destination, Position(4, 2)},
        )

        with patch.object(
            OccupancyMap,
            "is_known_free",
            side_effect=AssertionError("global operator map was read"),
        ), patch.object(
            OccupancyMap,
            "frontiers",
            side_effect=AssertionError("global operator map was read"),
        ):
            simulation._deconflict_intentions(
                {"drone-1": destination, "drone-2": destination}
            )

    def test_shared_shadow_fingerprints_and_active_local_determinism(self) -> None:
        behavior_fields = (
            "steps",
            "position_trace_by_drone",
            "path_length_by_drone",
            "mission_success",
            "survivor_recall",
            "collisions",
            "drone_drone_collisions",
            "duplicate_exploration_ratio",
        )
        for seed in range(3):
            shared = local_simulation(
                seed=seed, knowledge_mode="shared"
            ).run().to_dict()
            shadow = local_simulation(
                seed=seed, knowledge_mode="shadow"
            ).run().to_dict()
            for field in behavior_fields:
                self.assertEqual(shared[field], shadow[field])

        config = SimulationConfig(
            seed=16,
            drone_count=2,
            communication_range=8,
            knowledge_mode="local",
        )
        self.assertEqual(
            MultiDroneSimulation(config).run(),
            MultiDroneSimulation(config).run(),
        )


if __name__ == "__main__":
    unittest.main()
