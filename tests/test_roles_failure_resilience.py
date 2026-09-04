import json
import unittest
from pathlib import Path

from echorescue.config import SimulationConfig
from echorescue.events import EventType
from echorescue.models import DroneStatus, Position
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.replay import (
    ROLE_FAILURE_REPLAY_SCHEMA_VERSION,
    record_simulation,
)
from echorescue.role_failure_benchmark import run_benchmark
from echorescue.roles import (
    AgentRole,
    TaskRegistry,
    TaskStatus,
    TaskType,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class RoleAndTaskModelTests(unittest.TestCase):
    def test_generalized_roles_initialize_for_supported_fleet_sizes(self) -> None:
        expected = {
            1: ["GENERALIST"],
            2: ["SCOUT", "VERIFIER"],
            4: ["SCOUT", "SCOUT", "VERIFIER", "GENERALIST"],
            8: [
                "SCOUT",
                "SCOUT",
                "SCOUT",
                "SCOUT",
                "VERIFIER",
                "VERIFIER",
                "GENERALIST",
                "GENERALIST",
            ],
        }
        for drone_count, roles in expected.items():
            simulation = MultiDroneSimulation(
                SimulationConfig(
                    seed=7,
                    drone_count=drone_count,
                    role_policy="generalized",
                )
            )
            with self.subTest(drone_count=drone_count):
                self.assertEqual(
                    [runtime.role.value for runtime in simulation._ordered_runtimes()],
                    roles,
                )

    def test_role_enabled_fleet_sizes_are_deterministic_safe_and_complete(self) -> None:
        for drone_count in (1, 2, 4, 8):
            config = SimulationConfig(
                seed=7, drone_count=drone_count, role_policy="generalized"
            )
            first = MultiDroneSimulation(config).run()
            second = MultiDroneSimulation(config).run()
            with self.subTest(drone_count=drone_count):
                self.assertEqual(first, second)
                self.assertTrue(first.mission_success)
                self.assertEqual(first.drones_returned, drone_count)
                self.assertEqual(first.collisions, 0)
                self.assertEqual(first.drone_drone_collisions, 0)

    def test_task_registry_has_unique_identity_and_single_owner(self) -> None:
        registry = TaskRegistry()
        first = registry.create(
            owner_id="drone-1",
            task_type=TaskType.EXPLORATION,
            target=Position(3, 3),
            step=2,
            priority=50,
        )
        orphan = registry.orphan_owner("drone-1", step=5)
        self.assertIs(orphan, first)
        self.assertIs(first.status, TaskStatus.ORPHANED)
        registry.assign_orphan(
            first.identifier,
            owner_id="drone-2",
            target=Position(3, 3),
            step=6,
        )
        self.assertIsNone(registry.current_for("drone-1"))
        self.assertIs(registry.current_for("drone-2"), first)
        self.assertEqual(first.owner_id, "drone-2")

    def test_generalist_policy_assigns_only_generalists(self) -> None:
        simulation = MultiDroneSimulation(
            SimulationConfig(
                seed=7, drone_count=4, role_policy="generalist"
            )
        )
        self.assertTrue(
            all(
                runtime.role is AgentRole.GENERALIST
                for runtime in simulation._ordered_runtimes()
            )
        )

    def test_invalid_role_policy_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "role_policy"):
            SimulationConfig(role_policy="commander")


class FailureRecoveryIntegrationTests(unittest.TestCase):
    def config(self, **overrides: object) -> SimulationConfig:
        values: dict[str, object] = {
            "seed": 7,
            "drone_count": 4,
            "role_policy": "generalized",
            "failure_schedule": (("drone-2", 16),),
        }
        values.update(overrides)
        return SimulationConfig(**values)

    def test_failed_owner_orphans_and_reassigns_task_exactly_once(self) -> None:
        result = MultiDroneSimulation(self.config()).run()
        metrics = result.role_failure_metrics
        self.assertIsNotNone(metrics)
        assert metrics is not None
        self.assertEqual(metrics["failures_injected"], 1)
        self.assertEqual(metrics["tasks_orphaned"], 1)
        self.assertEqual(metrics["successful_reassignments"], 1)
        self.assertEqual(metrics["failed_reassignments"], 0)
        self.assertEqual(metrics["tasks_completed_by_reassigned_agent"], 1)
        self.assertEqual(metrics["mean_reassignment_latency"], 0.0)
        self.assertEqual(metrics["mean_recovery_latency"], 1.0)
        events = result.mission_events
        self.assertEqual(
            sum(event.event_type is EventType.TASK_ORPHANED for event in events),
            1,
        )
        self.assertEqual(
            sum(event.event_type is EventType.TASK_REASSIGNED for event in events),
            1,
        )

    def test_multiple_candidates_have_stable_role_aware_winner(self) -> None:
        generalist = MultiDroneSimulation(
            self.config(role_policy="generalist")
        ).run()
        generalized = MultiDroneSimulation(self.config()).run()
        generalist_event = next(
            event
            for event in generalist.mission_events
            if event.event_type is EventType.TASK_REASSIGNED
        )
        generalized_event = next(
            event
            for event in generalized.mission_events
            if event.event_type is EventType.TASK_REASSIGNED
        )
        self.assertEqual(generalist_event.drone_id, "drone-3")
        self.assertEqual(generalized_event.drone_id, "drone-1")
        self.assertLess(
            generalized_event.reassignment_score,
            generalist_event.reassignment_score,
        )

    def test_energy_unsafe_best_candidate_is_rejected(self) -> None:
        simulation = MultiDroneSimulation(self.config())
        while simulation.steps < 16:
            simulation.step()
        simulation._prepare_role_failure_task_reassignments()
        first_claim = next(iter(simulation._role_failure_claims.values()))
        simulation._role_failure_claims.clear()
        unsafe = simulation.runtimes[first_claim.assignee_id]
        unsafe.battery.remaining = 0.0
        simulation._prepare_role_failure_task_reassignments()
        safe_claim = next(iter(simulation._role_failure_claims.values()))
        self.assertNotEqual(safe_claim.assignee_id, unsafe.drone.identifier)
        self.assertGreater(simulation.runtimes[safe_claim.assignee_id].battery.remaining, 0)

    def test_sequential_failures_trigger_sticky_role_takeover_without_thrashing(self) -> None:
        result = MultiDroneSimulation(
            self.config(
                failure_schedule=(("drone-1", 16), ("drone-2", 24))
            )
        ).run()
        metrics = result.role_failure_metrics
        assert metrics is not None
        self.assertEqual(metrics["failures_injected"], 2)
        self.assertEqual(metrics["successful_reassignments"], 2)
        self.assertEqual(metrics["emergency_role_takeovers"], 1)
        self.assertEqual(metrics["role_changes_per_agent"]["drone-3"], 2)
        self.assertFalse(metrics["role_thrashing_detected"])
        self.assertTrue(result.mission_success)
        self.assertEqual(result.drones_returned, 2)

    def test_failed_agent_is_offline_static_obstacle_and_operational_agents_return(self) -> None:
        result = MultiDroneSimulation(self.config()).run()
        failed_trace = result.position_trace_by_drone["drone-2"]
        failure_step = 16
        self.assertEqual(len(set(failed_trace[failure_step:])), 1)
        self.assertIs(result.drone_status_by_drone["drone-2"], DroneStatus.FAILED)
        self.assertEqual(result.drones_returned, 3)
        self.assertEqual(result.collisions, 0)
        self.assertEqual(result.drone_drone_collisions, 0)

    def test_unreachable_frontier_behind_failed_agent_does_not_stall_mission(self) -> None:
        config = self.config(seed=96, failure_schedule=(("drone-2", 12),))
        result = MultiDroneSimulation(config).run()
        metrics = result.role_failure_metrics
        assert metrics is not None
        self.assertTrue(result.mission_success)
        self.assertLess(result.steps, config.max_steps)
        self.assertEqual(metrics["successful_reassignments"], 1)
        self.assertEqual(metrics["tasks_completed_by_reassigned_agent"], 1)
        self.assertEqual(metrics["operational_agents_returned"], 3)
        self.assertEqual(result.collisions, 0)
        self.assertEqual(result.drone_drone_collisions, 0)

    def test_failure_combines_with_dynamic_obstacles(self) -> None:
        result = MultiDroneSimulation(
            self.config(dynamic_obstacles="moderate")
        ).run()
        self.assertTrue(result.mission_success)
        self.assertEqual(result.collisions, 0)
        self.assertEqual(result.drone_drone_collisions, 0)
        self.assertIsNotNone(result.dynamic_obstacle_metrics)
        self.assertEqual(result.role_failure_metrics["successful_reassignments"], 1)  # type: ignore[index]

    def test_failure_combines_with_noisy_hypotheses(self) -> None:
        first = MultiDroneSimulation(
            self.config(perception_noise="moderate")
        ).run()
        second = MultiDroneSimulation(
            self.config(perception_noise="moderate")
        ).run()
        self.assertEqual(first, second)
        self.assertIsNotNone(first.perception_metrics)
        self.assertIsNotNone(first.role_failure_metrics)
        self.assertEqual(first.collisions, 0)
        self.assertEqual(first.drone_drone_collisions, 0)

    def test_role_task_recovery_is_regression_safe_in_constrained_local_mode(self) -> None:
        result = MultiDroneSimulation(
            self.config(
                knowledge_mode="local",
                network_profile="constrained",
                failure_schedule=(("drone-2", 12),),
            )
        ).run()
        metrics = result.role_failure_metrics
        assert metrics is not None
        self.assertTrue(result.mission_success)
        self.assertEqual(metrics["successful_reassignments"], 1)
        self.assertEqual(metrics["operational_agents_returned"], 3)
        self.assertEqual(result.collisions, 0)
        self.assertEqual(result.drone_drone_collisions, 0)

    def test_replay_and_dashboard_expose_roles_tasks_and_recovery(self) -> None:
        replay, result = record_simulation(MultiDroneSimulation(self.config()))
        self.assertEqual(replay["schema_version"], ROLE_FAILURE_REPLAY_SCHEMA_VERSION)
        self.assertEqual(replay["mission"]["role_policy"], "generalized")
        self.assertTrue(all("role" in drone for drone in replay["frames"][0]["drones"].values()))
        self.assertTrue(any(frame.get("task_ownership") for frame in replay["frames"]))
        events = [event for frame in replay["frames"] for event in frame["events"]]
        self.assertTrue(any(event["event_type"] == "task_orphaned" for event in events))
        self.assertTrue(any(event["event_type"] == "task_reassigned" for event in events))
        self.assertTrue(any(event["event_type"] == "failure_recovery_completed" for event in events))
        self.assertEqual(
            replay["metrics"]["role_failure_resilience"],
            result.role_failure_metrics,
        )
        javascript = (
            REPOSITORY_ROOT
            / "src"
            / "echorescue"
            / "dashboard_assets"
            / "app.js"
        ).read_text(encoding="utf-8")
        self.assertIn("droneRole", javascript)
        self.assertIn("droneTask", javascript)
        self.assertIn("task_reassigned", javascript)

    def test_small_four_scenario_benchmark_is_deterministic_and_safe(self) -> None:
        first = run_benchmark(seeds=(50, 51), repeat=True)
        second = run_benchmark(seeds=(50, 51), repeat=True)
        self.assertEqual(first, second)
        self.assertTrue(all(first["acceptance"].values()))
        self.assertEqual(len(first["per_seed"]), 2)

    def test_committed_holdout_artifact_meets_acceptance(self) -> None:
        artifact = (
            REPOSITORY_ROOT
            / "benchmarks"
            / "generalized_roles_failure_50_holdout_seeds.json"
        )
        payload = json.loads(artifact.read_text(encoding="utf-8"))
        self.assertEqual(payload["schema_version"], "1.0")
        self.assertEqual(
            payload["configuration"]["holdout_seeds"], list(range(50, 100))
        )
        self.assertEqual(len(payload["per_seed"]), 50)
        self.assertTrue(all(payload["acceptance"].values()))
        self.assertEqual(payload["representative_demo_seed"], 50)
        self.assertTrue(
            all(
                scenario["missions"] == 50
                for scenario in payload["scenarios"].values()
            )
        )


if __name__ == "__main__":
    unittest.main()
