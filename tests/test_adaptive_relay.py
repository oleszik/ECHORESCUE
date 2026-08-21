import json
import unittest
from pathlib import Path
from unittest.mock import patch

from echorescue.config import SimulationConfig
from echorescue.environment import GridWorld
from echorescue.events import EventType
from echorescue.models import CellState, DroneStatus, Position
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.relay import known_radio_link, select_relay_plan
from echorescue.replay import REPLAY_SCHEMA_VERSION, generate_replay


def relay_config(**overrides: object) -> SimulationConfig:
    values = {
        "width": 9,
        "height": 7,
        "seed": 7,
        "drone_count": 2,
        "obstacle_density": 0.0,
        "communication_range": 3,
        "knowledge_mode": "local",
        "relay_strategy": "adaptive",
        "relay_min_outage_steps": 1,
        "relay_min_unsynced_cells": 1,
        "relay_min_benefit_ratio": 0.1,
    }
    values.update(overrides)
    return SimulationConfig(**values)


def prepare_relay_opportunity() -> MultiDroneSimulation:
    simulation = MultiDroneSimulation(relay_config())
    relay = simulation.runtimes["drone-1"]
    scout = simulation.runtimes["drone-2"]
    relay.drone.position = Position(5, 1)
    scout.drone.position = Position(7, 1)
    known_free = {
        Position(x, 1): CellState.FREE for x in range(1, 8)
    }
    for runtime in (relay, scout):
        runtime.local_map.observe(
            known_free, step=1, source_id=runtime.drone.identifier
        )
        runtime.base_acknowledged_records = dict(runtime.local_map.records)
        runtime.base_acknowledged_survivors = set(
            runtime.confirmed_survivors
        )
    scout.local_map.observe(
        {Position(7, 2): CellState.FREE},
        step=2,
        source_id="drone-2",
    )
    simulation.steps = 2
    simulation._sample_communication(record_events=False)
    simulation._current_outage_steps = {"drone-1": 2, "drone-2": 2}
    return simulation


class RelayPlanningTests(unittest.TestCase):
    def test_known_radio_link_uses_only_local_cell_states(self) -> None:
        simulation = prepare_relay_opportunity()
        knowledge = simulation.runtimes["drone-1"].local_map

        self.assertTrue(
            known_radio_link(
                knowledge, Position(1, 1), Position(4, 1), 3
            )
        )
        knowledge.observe(
            {Position(3, 1): CellState.OCCUPIED},
            step=3,
            source_id="drone-1",
        )
        self.assertFalse(
            known_radio_link(
                knowledge, Position(1, 1), Position(4, 1), 3
            )
        )

    def test_relay_position_is_local_free_reachable_and_energy_safe(self) -> None:
        simulation = prepare_relay_opportunity()
        runtime = simulation.runtimes["drone-1"]
        plan = select_relay_plan(
            runtime.local_map,
            runtime.drone.position,
            simulation.world.base,
            simulation.runtimes["drone-2"].drone.position,
            max_range=3,
            energy_remaining=runtime.battery.remaining,
            movement_cycle_cost=runtime.battery.movement_cycle_cost,
            safety_reserve=simulation.config.energy_safety_reserve,
            energy_margin=simulation.config.relay_energy_margin,
        )

        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertTrue(runtime.local_map.is_known_free(plan.position))
        self.assertEqual(plan.path[0], runtime.drone.position)
        self.assertEqual(plan.return_path[-1], simulation.world.base)
        self.assertLessEqual(plan.energy_required, runtime.battery.remaining)


class AdaptiveRelayTests(unittest.TestCase):
    def test_adaptive_relay_is_restricted_to_active_local_mode(self) -> None:
        self.assertEqual(SimulationConfig().relay_strategy, "off")
        with self.assertRaisesRegex(ValueError, "knowledge_mode=local"):
            SimulationConfig(
                drone_count=2,
                knowledge_mode="shared",
                relay_strategy="adaptive",
            )

    def test_relay_activates_only_for_unsynchronized_relevant_data(self) -> None:
        simulation = prepare_relay_opportunity()
        simulation._assign_adaptive_relay()

        relay = simulation.runtimes["drone-1"]
        self.assertIs(relay.drone.status, DroneStatus.RELAY)
        self.assertEqual(relay.relay_scout_id, "drone-2")
        self.assertIsNotNone(relay.relay_target)
        self.assertTrue(simulation.runtimes["drone-2"].holding_for_relay)

        no_payload = prepare_relay_opportunity()
        for runtime in no_payload.runtimes.values():
            runtime.base_acknowledged_records = dict(
                runtime.local_map.records
            )
        no_payload._assign_adaptive_relay()
        self.assertEqual(no_payload.relay_deployments, 0)

        no_base_store = prepare_relay_opportunity()
        no_base_store.base_knowledge_map = None
        no_base_store.shadow_synchronizer.base_map = None
        no_base_store._assign_adaptive_relay()
        self.assertEqual(no_base_store.relay_deployments, 0)

    def test_relay_assignment_does_not_query_ground_truth(self) -> None:
        simulation = prepare_relay_opportunity()
        with patch.object(
            GridWorld,
            "is_free",
            side_effect=AssertionError("ground truth entered relay planning"),
        ):
            simulation._assign_adaptive_relay()

        self.assertEqual(simulation.relay_deployments, 1)

    def test_insufficient_energy_prevents_role_assignment(self) -> None:
        simulation = prepare_relay_opportunity()
        simulation.runtimes["drone-1"].battery.remaining = 10.0
        simulation._assign_adaptive_relay()

        self.assertEqual(simulation.relay_deployments, 0)

    def test_critical_energy_aborts_relay_and_starts_safe_return(self) -> None:
        simulation = prepare_relay_opportunity()
        simulation._assign_adaptive_relay()
        relay = simulation.runtimes["drone-1"]
        relay.battery.remaining = 6.0

        simulation._maintain_adaptive_relay()

        self.assertIs(relay.drone.status, DroneStatus.RETURN_HOME)
        event_types = [event.event_type for event in simulation.mission_log.events]
        self.assertIn(EventType.RELAY_ABORTED_FOR_ENERGY, event_types)
        self.assertIn(EventType.RELAY_ROLE_RELEASED, event_types)

    def test_survivor_payload_reaches_base_and_role_is_released(self) -> None:
        result = MultiDroneSimulation(
            SimulationConfig(
                seed=1,
                drone_count=2,
                communication_range=8,
                knowledge_mode="local",
                relay_strategy="adaptive",
            )
        ).run()

        self.assertGreater(result.relay_survivor_confirmations_forwarded, 0)
        self.assertEqual(result.successful_relay_deployments, 1)
        self.assertEqual(result.failed_relay_deployments, 0)
        relay_events = [
            event.event_type
            for event in result.mission_events
            if event.event_type
            in {
                EventType.RELAY_ROLE_ASSIGNED,
                EventType.RELAY_LINK_ACHIEVED,
                EventType.RELAY_PAYLOAD_FORWARDED,
                EventType.RELAY_ROLE_RELEASED,
            }
        ]
        self.assertEqual(
            relay_events,
            [
                EventType.RELAY_ROLE_ASSIGNED,
                EventType.RELAY_LINK_ACHIEVED,
                EventType.RELAY_PAYLOAD_FORWARDED,
                EventType.RELAY_ROLE_RELEASED,
            ],
        )
        self.assertEqual(result.base_survivors_confirmed, result.survivors_total)

    def test_role_duration_is_bounded_and_does_not_starve_scout(self) -> None:
        config = SimulationConfig(
            seed=7,
            drone_count=2,
            communication_range=8,
            knowledge_mode="local",
            relay_strategy="adaptive",
        )
        result = MultiDroneSimulation(config).run()

        self.assertLessEqual(result.relay_deployments, config.relay_max_deployments)
        self.assertLessEqual(
            max(result.relay_steps_by_drone.values()),
            config.relay_max_role_steps,
        )
        self.assertEqual(result.drones_returned, 2)
        self.assertNotEqual(
            result.drone_status_by_drone["drone-2"], DroneStatus.FAILED
        )

    def test_relay_participates_in_distributed_deconfliction(self) -> None:
        simulation = prepare_relay_opportunity()
        relay = simulation.runtimes["drone-1"]
        scout = simulation.runtimes["drone-2"]
        relay.drone.status = DroneStatus.RELAY
        destination = Position(6, 1)

        resolved = simulation._deconflict_intentions(
            {"drone-1": destination, "drone-2": destination}
        )
        simulation._execute_intentions(resolved)

        self.assertEqual(simulation.safety_shield_interventions, 0)
        self.assertEqual(simulation.drone_drone_collisions, 0)
        self.assertEqual(
            sum(
                runtime.drone.position == destination
                for runtime in (relay, scout)
            ),
            1,
        )

    def test_adaptive_mode_is_deterministic(self) -> None:
        config = SimulationConfig(
            seed=3,
            drone_count=2,
            communication_range=8,
            knowledge_mode="local",
            relay_strategy="adaptive",
        )
        self.assertEqual(
            MultiDroneSimulation(config).run(),
            MultiDroneSimulation(config).run(),
        )

    def test_replay_and_dashboard_expose_relay_state_without_new_logic(self) -> None:
        replay = generate_replay(
            SimulationConfig(
                seed=7,
                drone_count=2,
                communication_range=8,
                knowledge_mode="local",
                relay_strategy="adaptive",
            )
        )

        self.assertEqual(replay["schema_version"], REPLAY_SCHEMA_VERSION)
        self.assertEqual(replay["mission"]["relay_strategy"], "adaptive")
        relay_frames = [
            drone
            for frame in replay["frames"]
            for drone in frame["drones"].values()
            if drone["relay"]["active"]
        ]
        self.assertTrue(relay_frames)
        self.assertTrue(all(drone["relay"]["position"] for drone in relay_frames))
        assets = Path(__file__).resolve().parents[1] / "src" / "echorescue" / "dashboard_assets"
        self.assertIn(
            "Adaptive relay impact",
            (assets / "index.html").read_text(encoding="utf-8"),
        )
        self.assertIn(
            "relay-active",
            (assets / "app.js").read_text(encoding="utf-8"),
        )

    def test_versioned_50_seed_artifact_meets_acceptance(self) -> None:
        artifact = (
            Path(__file__).resolve().parents[1]
            / "benchmarks"
            / "adaptive_relay_50_seeds.json"
        )
        payload = json.loads(artifact.read_text(encoding="utf-8"))

        self.assertEqual(payload["schema_version"], "1.0")
        self.assertEqual(payload["suite"]["seed_count"], 50)
        self.assertEqual(payload["suite"]["determinism_check"], "passed")
        self.assertTrue(payload["suite"]["off_matches_verified_fingerprint"])
        self.assertTrue(payload["suite"]["acceptance"]["accepted"])


if __name__ == "__main__":
    unittest.main()
