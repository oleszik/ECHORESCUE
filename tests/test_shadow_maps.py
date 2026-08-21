import json
import unittest
from pathlib import Path

from echorescue.communication import CommunicationModel
from echorescue.config import SimulationConfig
from echorescue.environment import GridWorld
from echorescue.events import EventType
from echorescue.knowledge import (
    CellKnowledge,
    KnowledgeMap,
    merge_cell_knowledge,
)
from echorescue.map_sync import ShadowMapSynchronizer
from echorescue.models import CellState, Position
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.shadow_benchmark import (
    EXPECTED_BEHAVIOR_FINGERPRINT_50_SEEDS,
    run_shadow_benchmark,
)


def open_world() -> GridWorld:
    width = 10
    height = 7
    walls = frozenset(
        Position(x, y)
        for y in range(height)
        for x in range(width)
        if x in (0, width - 1) or y in (0, height - 1)
    )
    return GridWorld(width, height, Position(1, 1), walls)


def knowledge_maps() -> tuple[dict[str, KnowledgeMap], KnowledgeMap]:
    return (
        {
            "drone-1": KnowledgeMap(10, 7),
            "drone-2": KnowledgeMap(10, 7),
        },
        KnowledgeMap(10, 7),
    )


class KnowledgeMergeTests(unittest.TestCase):
    def test_conflict_merge_is_order_independent_and_safety_first(self) -> None:
        free = CellKnowledge(CellState.FREE, 10, "drone-2")
        occupied = CellKnowledge(CellState.OCCUPIED, 2, "drone-1")

        self.assertEqual(merge_cell_knowledge(free, occupied), occupied)
        self.assertEqual(merge_cell_knowledge(occupied, free), occupied)

    def test_merge_uses_only_supplied_knowledge(self) -> None:
        local = KnowledgeMap(10, 7)
        position = Position(4, 3)

        local.observe(
            {position: CellState.FREE}, step=1, source_id="drone-1"
        )

        self.assertIs(local.cell_at(position), CellState.FREE)
        self.assertIs(local.cell_at(Position(5, 3)), CellState.UNKNOWN)


class ShadowSynchronizationTests(unittest.TestCase):
    def test_local_observation_stays_local_without_connection(self) -> None:
        local_maps, base_map = knowledge_maps()
        local_maps["drone-1"].observe(
            {Position(6, 3): CellState.FREE},
            step=1,
            source_id="drone-1",
        )
        world = open_world()
        snapshot = CommunicationModel(max_range=1).compute(
            world,
            world.base,
            {"drone-1": Position(6, 3), "drone-2": Position(8, 5)},
        )

        report = ShadowMapSynchronizer(local_maps, base_map).sync(snapshot)

        self.assertFalse(report.transfer_occurred)
        self.assertIs(
            local_maps["drone-2"].cell_at(Position(6, 3)), CellState.UNKNOWN
        )
        self.assertIs(base_map.cell_at(Position(6, 3)), CellState.UNKNOWN)

    def test_direct_connection_synchronizes_drones_and_base(self) -> None:
        local_maps, base_map = knowledge_maps()
        position = Position(3, 2)
        local_maps["drone-1"].observe(
            {position: CellState.OCCUPIED},
            step=2,
            source_id="drone-1",
        )
        world = open_world()
        snapshot = CommunicationModel(max_range=4).compute(
            world,
            world.base,
            {"drone-1": Position(3, 1), "drone-2": Position(4, 1)},
        )

        report = ShadowMapSynchronizer(local_maps, base_map).sync(snapshot)

        self.assertTrue(report.transfer_occurred)
        self.assertIs(local_maps["drone-2"].cell_at(position), CellState.OCCUPIED)
        self.assertIs(base_map.cell_at(position), CellState.OCCUPIED)

    def test_relay_path_enables_map_exchange(self) -> None:
        local_maps, base_map = knowledge_maps()
        position = Position(7, 2)
        local_maps["drone-2"].observe(
            {position: CellState.FREE}, step=3, source_id="drone-2"
        )
        world = open_world()
        snapshot = CommunicationModel(max_range=3).compute(
            world,
            world.base,
            {"drone-1": Position(4, 1), "drone-2": Position(7, 1)},
        )

        ShadowMapSynchronizer(local_maps, base_map).sync(snapshot)

        self.assertTrue(snapshot.connections["drone-2"].via_relay)
        self.assertIs(local_maps["drone-1"].cell_at(position), CellState.FREE)
        self.assertIs(base_map.cell_at(position), CellState.FREE)

    def test_no_synchronization_after_link_loss(self) -> None:
        local_maps, base_map = knowledge_maps()
        synchronizer = ShadowMapSynchronizer(local_maps, base_map)
        world = open_world()
        connected = CommunicationModel(max_range=4).compute(
            world,
            world.base,
            {"drone-1": Position(3, 1), "drone-2": Position(4, 1)},
        )
        synchronizer.sync(connected)
        position = Position(6, 4)
        local_maps["drone-1"].observe(
            {position: CellState.FREE}, step=4, source_id="drone-1"
        )
        disconnected = CommunicationModel(max_range=1).compute(
            world,
            world.base,
            {"drone-1": Position(6, 4), "drone-2": Position(8, 5)},
        )

        report = synchronizer.sync(disconnected)

        self.assertFalse(report.transfer_occurred)
        self.assertIs(local_maps["drone-2"].cell_at(position), CellState.UNKNOWN)


class ShadowModeIntegrationTests(unittest.TestCase):
    def test_versioned_50_seed_shadow_artifact_is_present(self) -> None:
        artifact = (
            Path(__file__).resolve().parents[1]
            / "benchmarks"
            / "shadow_mode_50_seeds.json"
        )
        payload = json.loads(artifact.read_text(encoding="utf-8"))

        self.assertEqual(payload["suite"]["seed_count"], 50)
        self.assertEqual(payload["suite"]["determinism_check"], "passed")
        self.assertTrue(payload["suite"]["matches_pre_shadow_behavior"])
        self.assertEqual(
            payload["suite"]["behavior_fingerprint_sha256"],
            EXPECTED_BEHAVIOR_FINGERPRINT_50_SEEDS,
        )

    def test_small_shadow_benchmark_is_deterministic(self) -> None:
        first = run_shadow_benchmark(seed_count=2)
        second = run_shadow_benchmark(seed_count=2)

        self.assertEqual(first, second)
        self.assertEqual(first["suite"]["determinism_check"], "passed")

    def test_same_seed_has_identical_shadow_metrics_and_events(self) -> None:
        config = SimulationConfig(
            seed=19, drone_count=2, knowledge_mode="shadow"
        )

        first = MultiDroneSimulation(config).run()
        second = MultiDroneSimulation(config).run()

        self.assertEqual(first, second)

    def test_map_transfer_events_are_aggregated_per_step_and_drone(self) -> None:
        result = MultiDroneSimulation(
            SimulationConfig(
                seed=7, drone_count=2, knowledge_mode="shadow"
            )
        ).run()
        map_events = {
            EventType.MAP_SYNC_STARTED,
            EventType.MAP_CELLS_UPLOADED,
            EventType.MAP_CELLS_RECEIVED,
            EventType.MAP_CONVERGED,
        }
        seen = set()
        for event in result.mission_events:
            if event.event_type not in map_events:
                continue
            key = (event.step, event.drone_id, event.event_type)
            self.assertNotIn(key, seen)
            seen.add(key)
            if event.event_type in {
                EventType.MAP_CELLS_UPLOADED,
                EventType.MAP_CELLS_RECEIVED,
            }:
                self.assertIsNotNone(event.cell_count)
                self.assertGreater(event.cell_count, 0)

    def test_shadow_mode_does_not_change_mission_behavior(self) -> None:
        behavior_fields = (
            "termination_reason",
            "steps",
            "collisions",
            "survivor_recall",
            "drones_returned",
            "drone_drone_collisions",
            "movement_conflicts",
            "wait_steps_by_drone",
            "path_length_by_drone",
            "frontier_assignments_by_drone",
            "drone_status_by_drone",
            "return_started_step_by_drone",
            "return_path_length_by_drone",
            "position_trace_by_drone",
            "duplicate_exploration_ratio",
            "mission_success",
        )
        map_events = {
            EventType.MAP_SYNC_STARTED,
            EventType.MAP_CELLS_UPLOADED,
            EventType.MAP_CELLS_RECEIVED,
            EventType.MAP_CONVERGED,
        }
        for seed in range(5):
            enabled = MultiDroneSimulation(
                SimulationConfig(
                    seed=seed,
                    drone_count=2,
                    local_map_shadow_mode=True,
                )
            ).run()
            disabled = MultiDroneSimulation(
                SimulationConfig(
                    seed=seed,
                    drone_count=2,
                    local_map_shadow_mode=False,
                )
            ).run()
            enabled_payload = enabled.to_dict()
            disabled_payload = disabled.to_dict()
            for field in behavior_fields:
                self.assertEqual(enabled_payload[field], disabled_payload[field])
            self.assertEqual(
                [
                    event
                    for event in enabled.mission_events
                    if event.event_type not in map_events
                ],
                list(disabled.mission_events),
            )
