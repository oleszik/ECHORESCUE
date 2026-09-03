import json
import unittest
from pathlib import Path

from echorescue.cli import build_parser
from echorescue.config import SimulationConfig
from echorescue.environment import GridWorld
from echorescue.events import EventType
from echorescue.models import Position
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.replay import SMOKE_REPLAY_SCHEMA_VERSION, generate_replay
from echorescue.simulation import Simulation
from echorescue.smoke import SmokeCell, SmokeField
from echorescue.survivors import SurvivorSensor
from echorescue.thermal_benchmark import run_benchmark


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def make_world(
    *,
    survivor: Position,
    internal_walls: frozenset[Position] = frozenset(),
    smoke: SmokeField = SmokeField(),
) -> GridWorld:
    width = height = 7
    boundary = {
        Position(x, y)
        for y in range(height)
        for x in range(width)
        if x in (0, width - 1) or y in (0, height - 1)
    }
    return GridWorld(
        width=width,
        height=height,
        base=Position(1, 1),
        walls=frozenset(boundary | set(internal_walls)),
        survivors=frozenset({survivor}),
        smoke=smoke,
    )


class ThermalSensorTests(unittest.TestCase):
    def test_cli_defaults_to_visual_and_exposes_thermal(self) -> None:
        defaults = build_parser().parse_args([])
        thermal = build_parser().parse_args(
            ["--survivor-sensor", "thermal"]
        )

        self.assertEqual(defaults.survivor_sensor, "visual")
        self.assertEqual(thermal.survivor_sensor, "thermal")
        self.assertEqual(thermal.thermal_survivor_range, 3)
        self.assertEqual(thermal.thermal_detection_probability, 0.6)
        self.assertEqual(thermal.thermal_smoke_attenuation, 0.15)

    def test_thermal_respects_wall_occlusion(self) -> None:
        world = make_world(
            survivor=Position(3, 1),
            internal_walls=frozenset({Position(2, 1)}),
        )
        sensor = SurvivorSensor(
            max_range=3,
            channel="thermal",
            base_detection_probability=1.0,
            smoke_attenuation=0.15,
        )

        report = sensor.observe_report(
            world,
            Position(1, 1),
            smoke_profile="off",
            seed=1,
            step=0,
            observer_id="drone-1",
        )

        self.assertEqual(report.attempts, 0)
        self.assertEqual(report.visible_survivors, ())

    def test_thermal_respects_configured_range(self) -> None:
        world = make_world(survivor=Position(4, 1))
        sensor = SurvivorSensor(
            max_range=2,
            channel="thermal",
            base_detection_probability=1.0,
            smoke_attenuation=0.15,
        )

        report = sensor.observe_report(
            world,
            Position(1, 1),
            smoke_profile="off",
            seed=1,
            step=0,
            observer_id="drone-1",
        )

        self.assertEqual(report.attempts, 0)
        self.assertEqual(report.visible_survivors, ())

    def test_thermal_is_less_smoke_sensitive_than_visual(self) -> None:
        smoke = SmokeField(
            profile="moderate",
            cells=tuple(
                SmokeCell(Position(x, 1), 0.65) for x in range(1, 5)
            ),
        )
        world = make_world(survivor=Position(4, 1), smoke=smoke)
        visual = SurvivorSensor(max_range=3)
        thermal = SurvivorSensor(
            max_range=3,
            channel="thermal",
            base_detection_probability=1.0,
            smoke_attenuation=0.15,
        )
        arguments = {
            "world": world,
            "origin": Position(1, 1),
            "smoke_profile": "moderate",
            "seed": 1,
            "step": 0,
            "observer_id": "drone-1",
        }

        visual_report = visual.observe_report(**arguments)
        thermal_report = thermal.observe_report(**arguments)

        self.assertEqual(visual_report.successful_observations, 0)
        self.assertEqual(thermal_report.successful_observations, 1)
        self.assertGreater(
            thermal_report.observations[0].confidence,
            visual_report.observations[0].confidence,
        )

    def test_thermal_confirmation_still_requires_two_observations(self) -> None:
        simulation = Simulation(
            SimulationConfig(
                seed=2,
                survivor_count=0,
                survivor_sensor="thermal",
            )
        )
        survivor = Position(2, 1)
        simulation.world = make_world(survivor=survivor)
        simulation.survivor_sensor = SurvivorSensor(
            max_range=3,
            channel="thermal",
            base_detection_probability=1.0,
            smoke_attenuation=0.15,
        )

        simulation._observe_survivors()
        self.assertIn(survivor, simulation.detected_survivors)
        self.assertNotIn(survivor, simulation.confirmed_survivors)

        simulation.steps = 1
        simulation._observe_survivors()
        self.assertIn(survivor, simulation.confirmed_survivors)
        confirmation = [
            event
            for event in simulation.mission_log.events
            if event.event_type is EventType.SURVIVOR_CONFIRMED
        ]
        self.assertEqual(len(confirmation), 1)
        self.assertEqual(confirmation[0].sensor_channel, "thermal")


class ThermalSimulationTests(unittest.TestCase):
    def test_four_profile_artifact_preserves_visual_smoke_baseline(self) -> None:
        smoke = json.loads(
            (
                REPOSITORY_ROOT
                / "benchmarks"
                / "smoke_perception_50_seeds.json"
            ).read_text(encoding="utf-8")
        )
        thermal = json.loads(
            (
                REPOSITORY_ROOT
                / "benchmarks"
                / "thermal_perception_50_seeds.json"
            ).read_text(encoding="utf-8")
        )

        for old_name, new_name in (
            ("smoke_off", "visual_smoke_off"),
            ("smoke_moderate", "visual_smoke_moderate"),
        ):
            old = smoke[old_name]
            new = thermal["profiles"][new_name]
            self.assertEqual(
                new["average_survivor_recall"],
                old["average_survivor_recall"],
            )
            self.assertEqual(
                new["average_time_to_first_detection"],
                old["average_time_to_first_detection"],
            )
            self.assertEqual(
                new["successful_survivor_observations"],
                old["survivor_confirmation_attempts"],
            )

    def test_small_four_profile_benchmark_is_deterministic(self) -> None:
        first = run_benchmark(seeds=2)
        second = run_benchmark(seeds=2)

        self.assertEqual(first, second)
        self.assertTrue(first["acceptance"]["deterministic_repeats"])
        self.assertTrue(first["acceptance"]["all_profiles_collision_free"])
        self.assertTrue(first["acceptance"]["navigation_metrics_identical"])
        self.assertEqual(
            set(first["profiles"]),
            {
                "visual_smoke_off",
                "visual_smoke_moderate",
                "thermal_smoke_off",
                "thermal_smoke_moderate",
            },
        )

    def test_thermal_mission_is_deterministic(self) -> None:
        config = SimulationConfig(
            seed=3,
            drone_count=2,
            survivor_sensor="thermal",
            smoke_profile="moderate",
        )

        first = MultiDroneSimulation(config).run()
        second = MultiDroneSimulation(config).run()

        self.assertEqual(first, second)

    def test_thermal_observation_telemetry_is_labeled_and_safe(self) -> None:
        config = SimulationConfig(
            seed=3,
            drone_count=2,
            survivor_sensor="thermal",
            smoke_profile="moderate",
        )
        result = MultiDroneSimulation(config).run()
        events = [
            event
            for event in result.mission_events
            if event.event_type is EventType.SURVIVOR_SENSOR_OBSERVATION
        ]

        self.assertTrue(events)
        for event in events:
            self.assertEqual(event.sensor_channel, "thermal")
            self.assertIsNotNone(event.survivor_distance)
            self.assertIsNotNone(event.smoke_density)
            self.assertIsNotNone(event.detection_success)
            self.assertIsNotNone(event.detection_confidence)
            self.assertEqual(
                event.position,
                result.position_trace_by_drone[event.drone_id][event.step],
            )

    def test_thermal_replay_uses_schema_2_without_ground_truth_leak(self) -> None:
        replay = generate_replay(
            SimulationConfig(
                seed=3,
                drone_count=2,
                survivor_sensor="thermal",
                smoke_profile="moderate",
            )
        )
        serialized = json.dumps(replay, sort_keys=True)

        self.assertEqual(
            replay["schema_version"], SMOKE_REPLAY_SCHEMA_VERSION
        )
        self.assertEqual(replay["mission"]["survivor_sensor"], "thermal")
        self.assertNotIn("smoke_debug", replay["map"])
        self.assertNotIn('"survivor_positions"', serialized)
        self.assertNotIn('"ground_truth"', serialized)
        failed_observations = [
            event
            for frame in replay["frames"]
            for event in frame["events"]
            if event["event_type"] == "survivor_sensor_observation"
            and not event["detection_success"]
        ]
        self.assertTrue(failed_observations)
        self.assertTrue(
            all("survivor_position" not in event for event in failed_observations)
        )

    def test_visual_moderate_reference_seed_is_unchanged(self) -> None:
        simulation = MultiDroneSimulation(
            SimulationConfig(
                seed=1,
                drone_count=2,
                survivor_sensor="visual",
                smoke_profile="moderate",
            )
        )
        result = simulation.run()
        metrics = simulation.smoke_detection_metrics()

        self.assertAlmostEqual(result.survivor_recall, 2 / 3)
        self.assertEqual(result.time_to_first_detection, 12)
        self.assertEqual(metrics["detection_attempts"], 28)
        self.assertEqual(metrics["successful_observations"], 11)
        self.assertEqual(metrics["degraded_detection_attempts"], 17)
        observations = [
            event
            for event in result.mission_events
            if event.event_type is EventType.SURVIVOR_SENSOR_OBSERVATION
        ]
        self.assertEqual(len(observations), 28)
        self.assertTrue(
            all(event.sensor_channel == "visual" for event in observations)
        )
        self.assertEqual(
            sum(event.detection_success is False for event in observations),
            17,
        )


if __name__ == "__main__":
    unittest.main()
