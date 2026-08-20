import json
import unittest
from pathlib import Path

from echorescue.communication import BASE_NODE_ID, CommunicationModel
from echorescue.communication_benchmark import run_communication_benchmark
from echorescue.config import SimulationConfig
from echorescue.environment import GridWorld
from echorescue.events import EventType
from echorescue.models import Position
from echorescue.multi_simulation import MultiDroneSimulation


def open_world(*, wall: Position | None = None) -> GridWorld:
    width = 10
    height = 7
    boundary = {
        Position(x, y)
        for y in range(height)
        for x in range(width)
        if x in (0, width - 1) or y in (0, height - 1)
    }
    if wall is not None:
        boundary.add(wall)
    return GridWorld(
        width=width,
        height=height,
        base=Position(1, 1),
        walls=frozenset(boundary),
    )


class CommunicationModelTests(unittest.TestCase):
    def test_range_boundary_is_inclusive_and_deterministic(self) -> None:
        model = CommunicationModel(max_range=3)
        world = open_world()

        at_boundary = model.compute(
            world, world.base, {"drone-1": Position(4, 1)}
        )
        outside = model.compute(
            world, world.base, {"drone-1": Position(5, 1)}
        )

        self.assertTrue(at_boundary.has_link(BASE_NODE_ID, "drone-1"))
        self.assertFalse(outside.has_link(BASE_NODE_ID, "drone-1"))

    def test_wall_blocks_line_of_sight(self) -> None:
        model = CommunicationModel(max_range=8)
        world = open_world(wall=Position(3, 1))

        snapshot = model.compute(
            world, world.base, {"drone-1": Position(5, 1)}
        )

        self.assertFalse(snapshot.has_link(BASE_NODE_ID, "drone-1"))
        self.assertFalse(snapshot.connections["drone-1"].connected_to_base)

    def test_direct_base_link_is_classified(self) -> None:
        snapshot = CommunicationModel(max_range=3).compute(
            open_world(), Position(1, 1), {"drone-1": Position(4, 1)}
        )

        connection = snapshot.connections["drone-1"]
        self.assertTrue(connection.connected_to_base)
        self.assertTrue(connection.direct_to_base)
        self.assertFalse(connection.via_relay)
        self.assertEqual(connection.relay_path, ())

    def test_indirect_connection_uses_deterministic_relay_path(self) -> None:
        snapshot = CommunicationModel(max_range=3).compute(
            open_world(),
            Position(1, 1),
            {
                "drone-1": Position(4, 1),
                "drone-2": Position(7, 1),
            },
        )

        connection = snapshot.connections["drone-2"]
        self.assertTrue(connection.connected_to_base)
        self.assertFalse(connection.direct_to_base)
        self.assertTrue(connection.via_relay)
        self.assertEqual(
            connection.relay_path, ("base", "drone-1", "drone-2")
        )


class CommunicationIntegrationTests(unittest.TestCase):
    def test_versioned_50_seed_artifact_is_present(self) -> None:
        artifact = (
            Path(__file__).resolve().parents[1]
            / "benchmarks"
            / "communication_50_seeds.json"
        )
        payload = json.loads(artifact.read_text(encoding="utf-8"))

        self.assertEqual(payload["suite"]["seed_count"], 50)
        self.assertEqual(payload["suite"]["determinism_check"], "passed")

    def test_small_communication_benchmark_is_deterministic(self) -> None:
        first = run_communication_benchmark(seed_count=2)
        second = run_communication_benchmark(seed_count=2)

        self.assertEqual(first, second)
        self.assertEqual(first["suite"]["determinism_check"], "passed")

    def test_lost_and_restored_events_only_occur_on_transitions(self) -> None:
        simulation = MultiDroneSimulation(
            SimulationConfig(
                drone_count=2,
                communication_range=2,
                obstacle_density=0.0,
            )
        )
        runtime = simulation.runtimes["drone-2"]

        runtime.drone.position = Position(6, 1)
        simulation.steps = 1
        simulation._sample_communication()
        simulation.steps = 2
        simulation._sample_communication()
        runtime.drone.position = Position(2, 1)
        simulation.steps = 3
        simulation._sample_communication()
        simulation.steps = 4
        simulation._sample_communication()

        transitions = [
            event.event_type
            for event in simulation.mission_log.events
            if event.drone_id == "drone-2"
            and event.event_type
            in {EventType.COMMUNICATION_LOST, EventType.COMMUNICATION_RESTORED}
        ]
        self.assertEqual(
            transitions,
            [EventType.COMMUNICATION_LOST, EventType.COMMUNICATION_RESTORED],
        )

    def test_relay_events_only_occur_when_relay_state_changes(self) -> None:
        simulation = MultiDroneSimulation(
            SimulationConfig(
                width=10,
                height=7,
                drone_count=2,
                communication_range=3,
                obstacle_density=0.0,
            )
        )
        simulation.runtimes["drone-1"].drone.position = Position(4, 1)
        simulation.runtimes["drone-2"].drone.position = Position(7, 1)
        simulation.steps = 1
        simulation._sample_communication()
        simulation.steps = 2
        simulation._sample_communication()
        simulation.runtimes["drone-1"].drone.position = Position(8, 1)
        simulation.steps = 3
        simulation._sample_communication()

        relay_events = [
            event.event_type
            for event in simulation.mission_log.events
            if event.drone_id == "drone-2"
            and event.event_type
            in {EventType.RELAY_LINK_ESTABLISHED, EventType.RELAY_LINK_LOST}
        ]
        self.assertEqual(
            relay_events,
            [EventType.RELAY_LINK_ESTABLISHED, EventType.RELAY_LINK_LOST],
        )

    def test_communication_metrics_are_deterministic(self) -> None:
        config = SimulationConfig(seed=17, drone_count=2, communication_range=8)

        first = MultiDroneSimulation(config).run().to_dict()
        second = MultiDroneSimulation(config).run().to_dict()

        metric_names = (
            "communication_uptime_by_drone",
            "direct_base_uptime_by_drone",
            "relay_uptime_by_drone",
            "communication_outages_by_drone",
            "longest_outage_by_drone",
        )
        for name in metric_names:
            self.assertEqual(first[name], second[name])

    def test_radio_range_does_not_change_existing_mission_behavior(self) -> None:
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
        for seed in range(5):
            short_range = MultiDroneSimulation(
                SimulationConfig(seed=seed, drone_count=2, communication_range=1)
            ).run().to_dict()
            long_range = MultiDroneSimulation(
                SimulationConfig(seed=seed, drone_count=2, communication_range=50)
            ).run().to_dict()
            for field in behavior_fields:
                self.assertEqual(short_range[field], long_range[field])
