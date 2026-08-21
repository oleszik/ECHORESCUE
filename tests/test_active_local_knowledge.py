import json
import unittest
from pathlib import Path
from unittest.mock import patch

from echorescue.config import SimulationConfig
from echorescue.events import EventType
from echorescue.knowledge import KnowledgeMap
from echorescue.mapping import OccupancyMap
from echorescue.models import CellState, Position
from echorescue.multi_simulation import MultiDroneSimulation


BEHAVIOR_FIELDS = (
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


def local_config(**overrides: object) -> SimulationConfig:
    values = {
        "width": 10,
        "height": 7,
        "seed": 7,
        "drone_count": 2,
        "obstacle_density": 0.0,
        "communication_range": 1,
        "knowledge_mode": "local",
    }
    values.update(overrides)
    return SimulationConfig(**values)


class ActiveLocalKnowledgeTests(unittest.TestCase):
    def test_versioned_50_seed_mode_comparison_is_present(self) -> None:
        artifact = (
            Path(__file__).resolve().parents[1]
            / "benchmarks"
            / "knowledge_modes_50_seeds.json"
        )
        payload = json.loads(artifact.read_text(encoding="utf-8"))

        self.assertEqual(payload["suite"]["seed_count"], 50)
        self.assertEqual(payload["suite"]["determinism_check"], "passed")
        self.assertTrue(
            payload["suite"]["matches_verified_shared_shadow_baseline"]
        )

    def test_local_decision_path_never_calls_global_frontiers(self) -> None:
        simulation = MultiDroneSimulation(local_config())
        self.assertEqual(simulation.occupancy_map.known_cell_count, 0)
        self.assertGreater(
            simulation.runtimes["drone-1"].local_map.known_cell_count, 0
        )

        with patch.object(
            OccupancyMap,
            "frontiers",
            side_effect=AssertionError("global map entered local decision path"),
        ), patch.object(
            OccupancyMap,
            "is_known_free",
            side_effect=AssertionError("global map entered local A* path"),
        ):
            simulation.step()

    def test_no_map_transfer_without_a_communication_path(self) -> None:
        simulation = MultiDroneSimulation(local_config())
        first = simulation.runtimes["drone-1"]
        second = simulation.runtimes["drone-2"]
        position = Position(6, 4)
        first.drone.position = Position(6, 4)
        second.drone.position = Position(8, 5)
        first.local_map.observe(
            {position: CellState.FREE},
            step=1,
            source_id="drone-1",
        )

        simulation.steps = 1
        simulation._sample_communication()
        simulation._sync_shadow_maps()

        self.assertIs(second.local_map.cell_at(position), CellState.UNKNOWN)
        self.assertIs(
            simulation.base_knowledge_map.cell_at(position), CellState.UNKNOWN
        )

    def test_disconnected_drones_select_from_their_local_frontiers(self) -> None:
        simulation = MultiDroneSimulation(local_config())
        simulation.runtimes["drone-1"].drone.position = Position(6, 4)
        simulation.runtimes["drone-2"].drone.position = Position(8, 5)
        for runtime in simulation.runtimes.values():
            runtime.active_frontier_target = None
            runtime.planned_path = ()
            simulation._sense(runtime)
        simulation.steps = 1
        simulation._sample_communication()
        local_frontiers = {
            drone_id: set(runtime.local_map.frontiers())
            for drone_id, runtime in simulation.runtimes.items()
        }

        assignments = simulation._allocate_frontiers()

        self.assertTrue(assignments)
        for drone_id, assignment in assignments.items():
            self.assertIn(assignment.target, local_frontiers[drone_id])
        self.assertTrue(
            any(
                event.event_type is EventType.LOCAL_FRONTIER_SELECTED
                for event in simulation.mission_log.events
            )
        )

    def test_duplicate_target_is_reconciled_after_reconnection(self) -> None:
        simulation = MultiDroneSimulation(local_config())
        first = simulation.runtimes["drone-1"]
        second = simulation.runtimes["drone-2"]
        first.drone.position = Position(6, 4)
        second.drone.position = Position(8, 5)
        simulation.steps = 1
        simulation._sample_communication()
        target = first.local_map.frontiers()[0]
        first.active_frontier_target = target
        second.active_frontier_target = target

        first.drone.position = Position(2, 1)
        second.drone.position = Position(3, 1)
        simulation.steps = 2
        simulation._sample_communication()
        simulation._sync_shadow_maps()
        simulation._allocate_frontiers()

        targets = {
            runtime.active_frontier_target
            for runtime in simulation.runtimes.values()
            if runtime.active_frontier_target is not None
        }
        self.assertEqual(len(targets), 2)
        self.assertGreaterEqual(simulation.targets_discarded_after_reconnect, 1)
        stale_events = [
            event
            for event in simulation.mission_log.events
            if event.event_type is EventType.STALE_TARGET_DISCARDED
        ]
        self.assertTrue(stale_events)

    def test_survivor_knowledge_reaches_base_only_over_radio(self) -> None:
        simulation = MultiDroneSimulation(local_config())
        first = simulation.runtimes["drone-1"]
        second = simulation.runtimes["drone-2"]
        survivor = Position(4, 4)
        first.drone.position = Position(7, 5)
        second.drone.position = Position(8, 5)
        second.detected_survivors.add(survivor)
        second.confirmed_survivors.add(survivor)
        simulation.steps = 1
        simulation._sample_communication()
        simulation._sync_survivor_knowledge()

        self.assertNotIn(survivor, simulation.confirmed_survivors)

        first.drone.position = Position(2, 1)
        second.drone.position = Position(3, 1)
        simulation.steps = 2
        simulation._sample_communication()
        simulation._sync_survivor_knowledge()

        self.assertIn(survivor, simulation.confirmed_survivors)
        self.assertTrue(
            any(
                event.event_type
                is EventType.SURVIVOR_KNOWLEDGE_SYNCHRONIZED
                and event.drone_id == "base"
                and event.position == survivor
                for event in simulation.mission_log.events
            )
        )

    def test_return_path_uses_only_locally_known_free_cells(self) -> None:
        simulation = MultiDroneSimulation(local_config())
        runtime = simulation.runtimes["drone-1"]
        runtime.local_map = KnowledgeMap(
            simulation.config.width, simulation.config.height
        )
        runtime.drone.position = Position(3, 1)
        local_route = (
            Position(3, 1),
            Position(3, 2),
            Position(2, 2),
            Position(1, 2),
            Position(1, 1),
        )
        runtime.local_map.observe(
            {position: CellState.FREE for position in local_route},
            step=1,
            source_id="drone-1",
        )
        simulation.occupancy_map.update({Position(2, 1): CellState.FREE})

        path = simulation._known_return_path(
            runtime, avoid_other_drones=False
        )

        self.assertEqual(path, local_route)
        self.assertNotIn(Position(2, 1), path)
        self.assertTrue(
            all(runtime.local_map.is_known_free(position) for position in path)
        )

    def test_central_safety_shield_counts_every_blocked_intention(self) -> None:
        simulation = MultiDroneSimulation(local_config())
        first = simulation.runtimes["drone-1"]
        second = simulation.runtimes["drone-2"]
        first.drone.position = Position(1, 2)
        second.drone.position = Position(3, 2)
        destination = Position(2, 2)
        for runtime in (first, second):
            runtime.local_map.observe(
                {destination: CellState.FREE},
                step=1,
                source_id=runtime.drone.identifier,
            )

        simulation._execute_intentions(
            {"drone-1": destination, "drone-2": destination}
        )

        events = [
            event
            for event in simulation.mission_log.events
            if event.event_type is EventType.SAFETY_SHIELD_INTERVENTION
        ]
        self.assertEqual(simulation.safety_shield_interventions, 1)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].drone_id, "drone-2")
        self.assertEqual(simulation.drone_drone_collisions, 0)

    def test_shared_and_shadow_behavior_remain_identical(self) -> None:
        for seed in range(3):
            shared = MultiDroneSimulation(
                SimulationConfig(
                    seed=seed, drone_count=2, knowledge_mode="shared"
                )
            ).run().to_dict()
            shadow = MultiDroneSimulation(
                SimulationConfig(
                    seed=seed, drone_count=2, knowledge_mode="shadow"
                )
            ).run().to_dict()
            for field in BEHAVIOR_FIELDS:
                self.assertEqual(shared[field], shadow[field])

    def test_active_local_mode_is_deterministic(self) -> None:
        config = SimulationConfig(
            seed=19,
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
