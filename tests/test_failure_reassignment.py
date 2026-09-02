import json
import unittest
from pathlib import Path

from echorescue.config import SimulationConfig
from echorescue.coordination import assign_frontiers
from echorescue.dashboard import create_server
from echorescue.events import EventType
from echorescue.failure_benchmark import run_benchmark
from echorescue.mapping import OccupancyMap
from echorescue.models import CellState, DroneStatus, Position
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.replay import (
    FAILURE_RECOVERY_REPLAY_SCHEMA_VERSION,
    record_simulation,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class FailureConfigurationTests(unittest.TestCase):
    def test_failure_schedule_is_validated(self) -> None:
        with self.assertRaisesRegex(ValueError, "requires drone_count=2"):
            SimulationConfig(failure_schedule=(("drone-1", 4),))
        with self.assertRaisesRegex(ValueError, "drone-1 or drone-2"):
            SimulationConfig(
                drone_count=2,
                failure_schedule=(("drone-3", 4),),
            )
        with self.assertRaisesRegex(ValueError, "only one injected failure"):
            SimulationConfig(
                drone_count=2,
                failure_schedule=(("drone-1", 4), ("drone-1", 8)),
            )


class FailureReassignmentTests(unittest.TestCase):
    def config(self, **overrides: object) -> SimulationConfig:
        values: dict[str, object] = {
            "seed": 7,
            "drone_count": 2,
            "failure_schedule": (("drone-2", 4),),
        }
        values.update(overrides)
        return SimulationConfig(**values)

    def test_failed_drone_goes_offline_at_the_scheduled_step(self) -> None:
        simulation = MultiDroneSimulation(self.config())
        while simulation.steps < 4:
            simulation.step()

        failed = simulation.runtimes["drone-2"]
        self.assertIs(failed.drone.status, DroneStatus.FAILED)
        self.assertFalse(
            simulation.communication_snapshot.connections[
                "drone-2"
            ].connected_to_base
        )
        self.assertTrue(
            all(
                "drone-2" not in (link.first, link.second)
                for link in simulation.communication_snapshot.links
            )
        )

    def test_surviving_drone_inherits_search_responsibility_and_recovers(self) -> None:
        result = MultiDroneSimulation(self.config()).run()
        metrics = result.failure_recovery_metrics

        self.assertIsNotNone(metrics)
        assert metrics is not None
        self.assertEqual(result.termination_reason, "failure_recovered")
        self.assertTrue(result.mission_success)
        self.assertEqual(result.survivors_confirmed, result.survivors_total)
        self.assertEqual(result.drones_returned, 1)
        self.assertEqual(result.drones_failed, 1)
        self.assertEqual(metrics["failures_triggered"], 1)
        self.assertEqual(metrics["tasks_released"], 1)
        self.assertEqual(metrics["tasks_reassigned"], 1)
        self.assertEqual(metrics["tasks_pending"], 0)
        self.assertTrue(metrics["recovery_success"])

        released = next(
            event
            for event in result.mission_events
            if event.event_type is EventType.FAILURE_TASK_RELEASED
        )
        reassigned = next(
            event
            for event in result.mission_events
            if event.event_type is EventType.FAILURE_TASK_REASSIGNED
        )
        self.assertEqual(reassigned.drone_id, "drone-1")
        self.assertEqual(released.reason, "drone-2")
        self.assertEqual(reassigned.reason, "drone-2")

    def test_failure_recovery_is_deterministic_in_active_local_mode(self) -> None:
        config = self.config(knowledge_mode="local")

        first = MultiDroneSimulation(config).run()
        second = MultiDroneSimulation(config).run()

        self.assertEqual(first, second)
        self.assertTrue(first.mission_success)
        self.assertEqual(
            first.failure_recovery_metrics["tasks_reassigned"],  # type: ignore[index]
            1,
        )

    def test_failure_replay_uses_versioned_schema_and_metrics(self) -> None:
        replay, result = record_simulation(MultiDroneSimulation(self.config()))

        self.assertEqual(
            replay["schema_version"], FAILURE_RECOVERY_REPLAY_SCHEMA_VERSION
        )
        self.assertEqual(
            replay["mission"]["configuration"]["failure_schedule"],
            (("drone-2", 4),),
        )
        self.assertEqual(
            replay["metrics"]["failure_recovery"],
            result.failure_recovery_metrics,
        )
        failure_frame = next(
            frame
            for frame in replay["frames"]
            if frame["drones"]["drone-2"]["state"] == "FAILED"
        )
        self.assertFalse(
            failure_frame["drones"]["drone-2"]["communication"][
                "connected_to_base"
            ]
        )

    def test_frontier_paths_can_treat_failed_vehicle_as_static_obstacle(self) -> None:
        occupancy = OccupancyMap(7, 7)
        occupancy.update(
            {
                Position(x, y): (
                    CellState.OCCUPIED
                    if x in {0, 6} or y in {0, 6}
                    else CellState.FREE
                )
                for y in range(7)
                for x in range(7)
            }
        )
        assignment = assign_frontiers(
            {"drone-1": Position(1, 1)},
            (Position(5, 1),),
            occupancy,
            {"drone-1": None},
            blocked=frozenset({Position(3, 1)}),
        )["drone-1"]

        self.assertNotIn(Position(3, 1), assignment.path)

    def test_failure_benchmark_reports_independent_acceptance_gates(self) -> None:
        benchmark = run_benchmark(seeds=3)

        self.assertEqual(benchmark["benchmark_type"], "failure_reassignment")
        self.assertEqual(benchmark["failure_recovery"]["missions"], 3)
        self.assertEqual(benchmark["failure_recovery"]["failures_triggered"], 3)
        self.assertEqual(benchmark["failure_recovery"]["tasks_reassigned"], 3)
        self.assertTrue(all(benchmark["acceptance"].values()))

    def test_dashboard_accepts_failure_replay_and_benchmark(self) -> None:
        server = create_server(
            REPOSITORY_ROOT / "replays" / "seed_7_failure.json",
            REPOSITORY_ROOT
            / "benchmarks"
            / "failure_reassignment_50_seeds.json",
            port=0,
        )
        try:
            self.assertGreater(server.server_address[1], 0)
        finally:
            server.server_close()

    def test_demo_replay_shows_one_failure_and_one_reassignment(self) -> None:
        replay = json.loads(
            (REPOSITORY_ROOT / "replays" / "seed_7_failure.json").read_text(
                encoding="utf-8"
            )
        )
        events = [
            event
            for frame in replay["frames"]
            for event in frame["events"]
        ]
        failures = [
            event
            for event in events
            if event["event_type"] == "drone_failure_injected"
        ]
        reassignments = [
            event
            for event in events
            if event["event_type"] == "failure_task_reassigned"
        ]

        self.assertEqual(len(failures), 1)
        self.assertEqual(len(reassignments), 1)
        self.assertEqual(reassignments[0]["drone_id"], "drone-1")
        failure_step = failures[0]["step"]
        failed_position = failures[0]["position"]
        post_failure = [
            frame for frame in replay["frames"] if frame["step"] >= failure_step
        ]
        self.assertGreater(failure_step, 0)
        self.assertTrue(
            all(
                frame["drones"]["drone-2"]["state"] == "FAILED"
                and frame["drones"]["drone-2"]["position"] == failed_position
                and frame["drones"]["drone-2"]["target"] is None
                for frame in post_failure
            )
        )
        self.assertTrue(
            all(
                frame["drones"]["drone-1"]["position"] != failed_position
                for frame in post_failure
            )
        )
        self.assertEqual(replay["frames"][-1]["drones"]["drone-1"]["state"], "LANDED")
        self.assertEqual(replay["metrics"]["drones_returned"], 1)
        self.assertEqual(replay["metrics"]["drones_failed"], 1)
        self.assertTrue(replay["metrics"]["failure_recovery"]["recovery_success"])

    def test_dashboard_visually_distinguishes_failed_and_operational_drones(self) -> None:
        javascript = (
            REPOSITORY_ROOT
            / "src"
            / "echorescue"
            / "dashboard_assets"
            / "app.js"
        ).read_text(encoding="utf-8")
        stylesheet = (
            REPOSITORY_ROOT
            / "src"
            / "echorescue"
            / "dashboard_assets"
            / "styles.css"
        ).read_text(encoding="utf-8")

        self.assertIn('drone.state === "FAILED"', javascript)
        self.assertIn('"Offline · failed"', javascript)
        self.assertIn("operational_drones_returned", javascript)
        self.assertIn(".drone-card.failed", stylesheet)


if __name__ == "__main__":
    unittest.main()
