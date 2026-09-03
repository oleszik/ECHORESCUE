import unittest
from pathlib import Path

from echorescue.config import SimulationConfig
from echorescue.environment import GridWorld
from echorescue.models import DroneStatus, Position
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.perception import HypothesisStatus, SurvivorHypothesisTracker
from echorescue.replay import (
    NOISY_PERCEPTION_REPLAY_SCHEMA_VERSION,
    generate_replay,
)
from echorescue.survivors import SurvivorSensor


def open_world() -> GridWorld:
    walls = frozenset(
        Position(x, y)
        for y in range(7)
        for x in range(7)
        if x in (0, 6) or y in (0, 6)
    )
    return GridWorld(
        width=7,
        height=7,
        base=Position(1, 1),
        walls=walls,
        survivors=frozenset({Position(3, 3)}),
    )


class NoisySurvivorSensorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.world = open_world()
        self.sensor = SurvivorSensor(max_range=4)
        self.origin = Position(2, 3)

    def observe(self, seed: int, profile: str = "moderate"):
        return self.sensor.observe_report(
            self.world,
            self.origin,
            smoke_profile="off",
            seed=seed,
            step=0,
            observer_id="drone-1",
            perception_noise=profile,
        )

    def test_noise_off_preserves_legacy_report(self) -> None:
        legacy = self.sensor.observe_report(
            self.world,
            self.origin,
            smoke_profile="off",
            seed=29,
            step=0,
            observer_id="drone-1",
        )
        self.assertEqual(legacy, self.observe(29, "off"))

    def test_noise_decisions_are_deterministic(self) -> None:
        self.assertEqual(self.observe(29), self.observe(29))

    def test_visible_survivor_can_be_false_negative(self) -> None:
        report = self.observe(0)
        self.assertEqual(report.false_negatives, 1)
        self.assertNotIn(Position(3, 3), report.visible_survivors)

    def test_false_positive_is_plausible_empty_cell_without_identity(self) -> None:
        report = self.observe(29)
        false_positive = next(
            item for item in report.observations if item.is_false_positive
        )
        self.assertNotIn(false_positive.position, self.world.survivors)
        self.assertTrue(self.world.is_free(false_positive.position))
        self.assertTrue(
            self.sensor.can_observe(
                self.world, self.origin, false_positive.position
            )
        )
        self.assertFalse(hasattr(false_positive, "survivor_id"))
        self.assertGreaterEqual(false_positive.confidence, 0.0)
        self.assertLessEqual(false_positive.confidence, 1.0)
        self.assertEqual(report.false_positives, 1)


class SurvivorEvidenceTests(unittest.TestCase):
    def tracker(self) -> SurvivorHypothesisTracker:
        return SurvivorHypothesisTracker(
            minimum_positive_observations=2,
            confirmation_threshold=0.65,
            rejection_threshold=0.12,
            negative_evidence_weight=0.55,
        )

    def test_one_observation_never_confirms_normal_evidence(self) -> None:
        tracker = self.tracker()
        update = tracker.positive(
            Position(2, 2),
            confidence=0.9,
            channel="visual",
            agent_id="drone-1",
            step=1,
        )
        self.assertFalse(update.confirmed)
        self.assertEqual(update.status_after, HypothesisStatus.UNCONFIRMED)

    def test_consistent_evidence_confirms(self) -> None:
        tracker = self.tracker()
        for step in (1, 2):
            update = tracker.positive(
                Position(2, 2),
                confidence=0.8,
                channel="visual",
                agent_id=f"drone-{step}",
                step=step,
            )
        self.assertTrue(update.confirmed)
        self.assertEqual(update.status_after, HypothesisStatus.CONFIRMED)

    def test_repeated_negative_evidence_rejects(self) -> None:
        tracker = self.tracker()
        tracker.positive(
            Position(2, 2),
            confidence=0.4,
            channel="visual",
            agent_id="drone-1",
            step=1,
        )
        tracker.negative(
            Position(2, 2),
            confidence=0.6,
            channel="visual",
            agent_id="drone-1",
            step=2,
        )
        update = tracker.negative(
            Position(2, 2),
            confidence=0.6,
            channel="visual",
            agent_id="drone-1",
            step=3,
        )
        self.assertIsNotNone(update)
        assert update is not None
        self.assertTrue(update.rejected)


class NoisyPerceptionIntegrationTests(unittest.TestCase):
    def test_result_metrics_and_replay_hypotheses(self) -> None:
        config = SimulationConfig(
            seed=7, drone_count=2, perception_noise="moderate"
        )
        first = MultiDroneSimulation(config).run()
        second = MultiDroneSimulation(config).run()
        self.assertEqual(first, second)
        self.assertIsNotNone(first.perception_metrics)
        assert first.perception_metrics is not None
        self.assertGreater(first.perception_metrics["perception_attempts"], 0)
        self.assertGreater(
            first.perception_metrics["false_positive_observations"], 0
        )
        replay = generate_replay(config)
        self.assertEqual(
            replay["schema_version"], NOISY_PERCEPTION_REPLAY_SCHEMA_VERSION
        )
        self.assertEqual(replay["mission"]["perception_noise"], "moderate")
        self.assertIn("survivor_hypotheses", replay["frames"][-1])
        self.assertTrue(
            all(
                "status" in hypothesis
                for hypothesis in replay["frames"][-1]["survivor_hypotheses"]
            )
        )

    def test_false_confirmation_prevents_mission_success(self) -> None:
        simulation = MultiDroneSimulation(
            SimulationConfig(seed=7, drone_count=2, perception_noise="moderate")
        )
        false_location = next(
            Position(x, y)
            for y in range(1, simulation.config.height - 1)
            for x in range(1, simulation.config.width - 1)
            if simulation.world.is_free(Position(x, y))
            and Position(x, y) not in simulation.world.survivors
        )
        simulation._confirmed_survivors = set(simulation.world.survivors) | {
            false_location
        }
        for runtime in simulation.runtimes.values():
            runtime.drone.position = simulation.world.base
            runtime.drone.status = DroneStatus.LANDED
        simulation.completed = True
        self.assertFalse(simulation.result().mission_success)

    def test_dashboard_assets_render_all_hypothesis_states(self) -> None:
        app = (
            Path(__file__).resolve().parents[1]
            / "src"
            / "echorescue"
            / "dashboard_assets"
            / "app.js"
        ).read_text(encoding="utf-8")
        self.assertIn('hypothesis.status === "confirmed"', app)
        self.assertIn('hypothesis.status === "rejected"', app)
        self.assertIn("event.evidence_after", app)
        self.assertIn("perceptionNoise.toUpperCase()", app)


if __name__ == "__main__":
    unittest.main()
