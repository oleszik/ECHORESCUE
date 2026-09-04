import json
import unittest
from pathlib import Path

from echorescue.config import SimulationConfig
from echorescue.events import EventType
from echorescue.knowledge import KnowledgeMap
from echorescue.models import CellState, DroneStatus, Position
from echorescue.multi_relay import (
    MultiRelayPlan,
    forecast_connectivity,
    relay_candidate_score,
    relay_target_topologies,
)
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.multi_relay_benchmark import run_benchmark
from echorescue.replay import MULTI_RELAY_REPLAY_SCHEMA_VERSION, ReplayRecorder
from echorescue.roles import AgentRole, TaskStatus, TaskType


def known_corridor(width: int = 11, height: int = 7) -> KnowledgeMap:
    knowledge = KnowledgeMap(width, height)
    knowledge.observe(
        {Position(x, 1): CellState.FREE for x in range(1, width - 1)},
        step=1,
        source_id="fixture",
    )
    return knowledge


def relay_config(**overrides: object) -> SimulationConfig:
    values: dict[str, object] = {
        "width": 11,
        "height": 7,
        "seed": 4,
        "drone_count": 4,
        "drone_start_positions": ((2, 1), (3, 1), (2, 2), (8, 1)),
        "obstacle_density": 0.0,
        "survivor_count": 0,
        "communication_range": 3,
        "knowledge_mode": "local",
        "network_profile": "constrained",
        "relay_strategy": "multi-relay",
        "role_policy": "generalized",
        "multi_relay_max_active": 2,
        "multi_relay_activation_outage_steps": 1,
        "multi_relay_min_unsynced_cells": 1,
        "multi_relay_min_hold_steps": 2,
        "multi_relay_max_role_steps": 12,
        "multi_relay_deactivation_hysteresis_steps": 2,
        "max_steps": 80,
    }
    values.update(overrides)
    return SimulationConfig(**values)


def prepared_simulation(**overrides: object) -> MultiDroneSimulation:
    simulation = MultiDroneSimulation(relay_config(**overrides))
    corridor = KnowledgeMap(11, 7)
    corridor.observe(
        {
            Position(x, y): CellState.FREE
            for x in range(1, 10)
            for y in range(1, 6)
        },
        step=1,
        source_id="fixture",
    )
    for runtime in simulation.runtimes.values():
        runtime.local_map = KnowledgeMap(corridor.width, corridor.height)
        runtime.local_map.apply(corridor.records)
        runtime.base_acknowledged_records = dict(runtime.local_map.records)
    simulation.shadow_synchronizer.local_maps = {
        drone_id: runtime.local_map
        for drone_id, runtime in simulation.runtimes.items()
    }
    assert simulation.base_knowledge_map is not None
    simulation.base_knowledge_map.apply(corridor.records)
    simulation._sample_communication(record_events=False)
    return simulation


def two_relay_plan() -> MultiRelayPlan:
    return MultiRelayPlan(
        relay_ids=("drone-1", "drone-2"),
        targets=(Position(4, 1), Position(6, 1)),
        paths=(
            (Position(2, 1), Position(3, 1), Position(4, 1)),
            (
                Position(3, 1),
                Position(4, 1),
                Position(5, 1),
                Position(6, 1),
            ),
        ),
        return_paths=(
            (Position(4, 1), Position(3, 1), Position(2, 1), Position(1, 1)),
            (
                Position(6, 1),
                Position(5, 1),
                Position(4, 1),
                Position(3, 1),
                Position(2, 1),
                Position(1, 1),
            ),
        ),
        served_agent_ids=("drone-4",),
        score=-20,
        reason="reactive_connectivity_recovery",
        predictive=False,
        forecast_disconnect_step=None,
    )


class PredictiveConnectivityTests(unittest.TestCase):
    def test_prediction_detects_loss_at_horizon_boundary(self) -> None:
        forecast = forecast_connectivity(
            known_corridor(),
            base=Position(1, 1),
            agent_positions={"scout": Position(3, 1)},
            agent_id="scout",
            planned_path=tuple(Position(x, 1) for x in range(3, 8)),
            current_step=10,
            horizon=2,
            max_range=3,
        )
        self.assertTrue(forecast.current_connected)
        self.assertEqual(forecast.projected_connected, (True, False))
        self.assertEqual(forecast.expected_disconnection_step, 12)
        self.assertEqual(forecast.projected_hop_counts, (1, None))

    def test_horizon_and_stable_route_do_not_false_predict(self) -> None:
        short = forecast_connectivity(
            known_corridor(),
            base=Position(1, 1),
            agent_positions={"scout": Position(2, 1)},
            agent_id="scout",
            planned_path=(Position(2, 1), Position(3, 1), Position(4, 1)),
            current_step=5,
            horizon=1,
            max_range=3,
        )
        self.assertIsNone(short.expected_disconnection_step)
        self.assertEqual(short.projected_connected, (True,))

    def test_unknown_geometry_is_conservative_and_deterministic(self) -> None:
        knowledge = KnowledgeMap(11, 7)
        knowledge.observe(
            {Position(1, 1): CellState.FREE, Position(2, 1): CellState.FREE},
            step=1,
            source_id="scout",
        )
        arguments = dict(
            base=Position(1, 1),
            agent_positions={"scout": Position(2, 1)},
            agent_id="scout",
            planned_path=(Position(2, 1), Position(4, 1)),
            current_step=2,
            horizon=1,
            max_range=4,
        )
        first = forecast_connectivity(knowledge, **arguments)
        second = forecast_connectivity(knowledge, **arguments)
        self.assertEqual(first, second)
        self.assertEqual(first.expected_disconnection_step, 3)


class RelayTopologyTests(unittest.TestCase):
    def test_two_relay_and_shared_relay_topologies_are_available(self) -> None:
        two_hop = relay_target_topologies(
            known_corridor(),
            base=Position(1, 1),
            service_positions={"scout": Position(9, 1)},
            primary_agent_id="scout",
            max_relays=2,
            max_range=3,
            candidate_limit=20,
        )
        self.assertTrue(any(len(item.targets) == 2 for item in two_hop))

        shared_knowledge = known_corridor()
        shared_knowledge.observe(
            {
                Position(x, y): CellState.FREE
                for x in range(1, 10)
                for y in range(1, 3)
            },
            step=2,
            source_id="fixture",
        )
        shared = relay_target_topologies(
            shared_knowledge,
            base=Position(1, 1),
            service_positions={
                "scout-a": Position(7, 1),
                "scout-b": Position(7, 2),
            },
            primary_agent_id="scout-a",
            max_relays=1,
            max_range=4,
            candidate_limit=20,
        )
        self.assertTrue(
            any(
                item.served_agent_ids == ("scout-a", "scout-b")
                for item in shared
            )
        )

    def test_candidate_score_is_transparent_and_stable(self) -> None:
        preferred = relay_candidate_score(
            travel_steps=2,
            task_interrupted=False,
            role="GENERALIST",
            consumed_energy=0.0,
            agents_helped=2,
            queue_units=40,
            predicted_steps_prevented=0,
        )
        expensive = relay_candidate_score(
            travel_steps=5,
            task_interrupted=True,
            role="SCOUT",
            consumed_energy=40.0,
            agents_helped=1,
            queue_units=0,
            predicted_steps_prevented=0,
        )
        self.assertEqual(preferred, -90)
        self.assertEqual(expensive, 12)
        self.assertLess(preferred, expensive)


class MultiRelayIntegrationTests(unittest.TestCase):
    def test_relay_collection_is_generic_for_supported_fleets(self) -> None:
        for fleet_size in (1, 2, 4, 8):
            with self.subTest(fleet_size=fleet_size):
                simulation = MultiDroneSimulation(
                    SimulationConfig(seed=2, drone_count=fleet_size)
                )
                self.assertEqual(simulation.active_relays, set())
                self.assertEqual(simulation.relay_deployments_by_agent, {})

    def test_two_relays_activate_without_teleporting_and_serve_one_scout(self) -> None:
        simulation = prepared_simulation()
        simulation._activate_multi_relay_plan(two_relay_plan())
        self.assertEqual(simulation.active_relays, {"drone-1", "drone-2"})
        self.assertEqual(
            simulation.runtimes["drone-1"].drone.position, Position(2, 1)
        )
        self.assertEqual(
            simulation.runtimes["drone-2"].drone.position, Position(3, 1)
        )
        self.assertEqual(
            simulation.runtimes["drone-1"].relay_served_ids,
            {"drone-4"},
        )
        self.assertTrue(
            all(
                runtime.drone.status is DroneStatus.RELAY
                for runtime in (
                    simulation.runtimes["drone-1"],
                    simulation.runtimes["drone-2"],
                )
            )
        )

    def test_selection_is_deterministic_and_rejects_low_energy_candidate(self) -> None:
        simulation = prepared_simulation(
            multi_relay_max_active=1,
            communication_range=4,
        )
        primary = simulation.runtimes["drone-4"]
        primary.base_acknowledged_records = {}
        simulation.runtimes["drone-1"].battery.remaining = 1.0
        simulation._current_outage_steps["drone-4"] = 2
        simulation.communication_snapshot = simulation._compute_communication_snapshot()
        needs = simulation._multi_relay_service_needs()
        self.assertTrue(needs)
        first = simulation._select_multi_relay_plan(needs[0], needs)
        second = simulation._select_multi_relay_plan(needs[0], needs)
        self.assertEqual(first, second)
        self.assertIsNotNone(first)
        assert first is not None
        self.assertNotIn("drone-1", first.relay_ids)

    def test_hysteresis_releases_relays_and_restores_base_roles(self) -> None:
        simulation = prepared_simulation()
        simulation._activate_multi_relay_plan(two_relay_plan())
        simulation._synchronize_role_tasks()
        base_roles = {
            relay_id: simulation.runtimes[relay_id].base_role
            for relay_id in simulation.active_relays
        }
        simulation.runtimes["drone-1"].drone.position = Position(4, 1)
        simulation.runtimes["drone-2"].drone.position = Position(6, 1)
        simulation._sample_communication(record_events=False)
        simulation.steps = 2
        simulation._maintain_multi_relay()
        self.assertEqual(len(simulation.active_relays), 2)
        simulation.steps = 3
        simulation._maintain_multi_relay()
        self.assertEqual(simulation.active_relays, set())
        for relay_id, base_role in base_roles.items():
            self.assertIs(simulation.runtimes[relay_id].role, base_role)
        self.assertEqual(simulation._multi_relay_deactivations, 2)

    def test_relay_failure_reuses_orphan_task_reassignment(self) -> None:
        simulation = prepared_simulation(
            failure_schedule=(("drone-1", 1),),
            multi_relay_max_active=1,
            communication_range=4,
        )
        plan = two_relay_plan()
        single = MultiRelayPlan(
            relay_ids=("drone-1",),
            targets=(Position(4, 1),),
            paths=(plan.paths[0],),
            return_paths=(plan.return_paths[0],),
            served_agent_ids=plan.served_agent_ids,
            score=plan.score,
            reason=plan.reason,
            predictive=False,
            forecast_disconnect_step=None,
        )
        simulation._activate_multi_relay_plan(single)
        simulation.runtimes["drone-1"].drone.position = Position(4, 1)
        simulation._synchronize_role_tasks()
        failed_task = simulation.task_registry.current_for("drone-1")
        self.assertIsNotNone(failed_task)
        assert failed_task is not None
        simulation.steps = 1
        simulation._inject_scheduled_failures()
        self.assertIs(failed_task.status, TaskStatus.ORPHANED)
        simulation._recover_orphaned_relay_tasks()
        self.assertIs(failed_task.status, TaskStatus.ASSIGNED)
        self.assertNotEqual(failed_task.owner_id, "drone-1")
        self.assertEqual(simulation._multi_relay_failure_recoveries, 1)

    def test_dynamic_target_invalidation_triggers_relay_replan(self) -> None:
        simulation = prepared_simulation(
            dynamic_obstacle_schedule=((4, 1, 2),)
        )
        simulation._activate_multi_relay_plan(two_relay_plan())
        relay = simulation.runtimes["drone-1"]
        relay.local_map.observe(
            {Position(4, 1): CellState.OCCUPIED},
            step=2,
            source_id="drone-1",
        )
        simulation._dynamic_obstacles_injected.add(Position(4, 1))
        simulation._invalidate_dynamic_paths(relay, {Position(4, 1)})
        self.assertTrue(relay.dynamic_replan_pending)
        simulation._maintain_multi_relay()
        self.assertEqual(simulation._multi_relay_replans, 1)
        self.assertEqual(simulation._path_invalidations, 1)
        self.assertEqual(simulation.active_relays, set())
        simulation.world.block_cell(Position(4, 1))
        for runtime in simulation.runtimes.values():
            runtime.local_map.observe(
                {Position(4, 1): CellState.OCCUPIED},
                step=2,
                source_id="shared-observation",
            )
        simulation.runtimes["drone-4"].base_acknowledged_records = {}
        simulation._sample_communication(record_events=False)
        simulation.steps = 8
        simulation._current_outage_steps["drone-4"] = 3
        simulation._assign_multi_relay()
        self.assertTrue(simulation.active_relays)
        self.assertNotIn(
            Position(4, 1),
            {
                deployment.target
                for deployment in simulation.relay_deployments_by_agent.values()
            },
        )

    def test_replay_serializes_multiple_relays_and_forecast_reason(self) -> None:
        simulation = prepared_simulation(relay_strategy="predictive")
        base_plan = two_relay_plan()
        plan = MultiRelayPlan(
            relay_ids=base_plan.relay_ids,
            targets=base_plan.targets,
            paths=base_plan.paths,
            return_paths=base_plan.return_paths,
            served_agent_ids=base_plan.served_agent_ids,
            score=base_plan.score,
            reason="predicted_link_loss",
            predictive=True,
            forecast_disconnect_step=4,
        )
        simulation._activate_multi_relay_plan(plan)
        recorder = ReplayRecorder()
        recorder.capture(simulation)
        frame = recorder._frames[0]
        topology = frame["multi_relay"]
        assert isinstance(topology, dict)
        self.assertEqual(
            topology["active_relay_ids"], ["drone-1", "drone-2"]
        )
        self.assertEqual(len(topology["deployments"]), 2)
        self.assertEqual(MULTI_RELAY_REPLAY_SCHEMA_VERSION, "2.4")
        replay = recorder.build(simulation, simulation.result())
        self.assertEqual(
            replay["schema_version"], MULTI_RELAY_REPLAY_SCHEMA_VERSION
        )
        self.assertIn(
            EventType.PREDICTIVE_RELAY_ACTIVATED,
            {event.event_type for event in simulation.mission_log.events},
        )

    def test_multi_relay_does_not_hold_scouts(self) -> None:
        simulation = prepared_simulation()
        simulation._activate_multi_relay_plan(two_relay_plan())
        self.assertFalse(simulation.runtimes["drone-4"].holding_for_relay)
        self.assertEqual(
            simulation.runtimes["drone-4"].role,
            simulation.runtimes["drone-4"].base_role,
        )

    def test_final_sync_waits_for_pending_critical_knowledge(self) -> None:
        simulation = prepared_simulation(relay_strategy="predictive")
        survivor = Position(8, 2)
        simulation.runtimes["drone-4"].confirmed_survivors.add(survivor)
        for runtime in simulation.runtimes.values():
            runtime.drone.status = DroneStatus.LANDED
        simulation._exploration_complete = True
        simulation._update_completion()
        self.assertFalse(simulation.completed)
        self.assertTrue(simulation._final_sync_active)

    def test_stable_predictive_path_does_not_activate_relay(self) -> None:
        simulation = prepared_simulation(relay_strategy="predictive")
        runtime = simulation.runtimes["drone-1"]
        runtime.planned_path = (
            Position(2, 1),
            Position(3, 1),
            Position(4, 1),
        )
        runtime.base_acknowledged_records = {}
        simulation._assign_multi_relay()
        self.assertEqual(simulation.active_relays, set())

    def test_dashboard_has_generic_multi_relay_rendering(self) -> None:
        source = (
            __import__("pathlib")
            .Path("src/echorescue/dashboard_assets/app.js")
            .read_text(encoding="utf-8")
        )
        self.assertIn("Predictive relay", source)
        self.assertIn("multi_relay_deactivated", source)
        self.assertIn("served_agent_ids", source)


class MultiRelayBenchmarkTests(unittest.TestCase):
    def test_small_paired_benchmark_is_deterministic_and_safe(self) -> None:
        payload = run_benchmark((52,), repeat=True)
        self.assertTrue(payload["acceptance"]["deterministic_repeats"])
        self.assertTrue(payload["acceptance"]["relay_profiles_collision_free"])
        self.assertEqual(len(payload["profiles"]), 4)

    def test_committed_holdout_contains_all_seeds_and_statistics(self) -> None:
        path = Path("benchmarks/predictive_multi_relay_50_seeds.json")
        payload = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(payload["seeds"], list(range(50, 100)))
        self.assertTrue(payload["acceptance"]["relay_profiles_complete"])
        self.assertTrue(
            payload["acceptance"]["relay_profiles_collision_free"]
        )
        for rows in payload["per_seed"].values():
            self.assertEqual(len(rows), 50)
        for profile in payload["profiles"].values():
            for statistics in profile["metrics"].values():
                self.assertEqual(
                    set(statistics),
                    {"mean", "population_std", "median", "min", "max"},
                )


if __name__ == "__main__":
    unittest.main()
