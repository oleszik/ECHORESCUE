import unittest
from pathlib import Path

from echorescue.cli import build_parser
from echorescue.config import SimulationConfig
from echorescue.dashboard import _validate_replay
from echorescue.environment import GridWorld
from echorescue.events import EventType
from echorescue.models import Position
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.replay import (
    SMOKE_REPLAY_SCHEMA_VERSION,
    generate_replay,
    replay_json_bytes,
    write_replay,
)
from echorescue.simulation import Simulation
from echorescue.smoke import SmokeCell, SmokeField
from echorescue.smoke_benchmark import run_benchmark
from echorescue.survivors import SurvivorSensor


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class SmokeModelTests(unittest.TestCase):
    def test_cli_exposes_opt_in_smoke_profile_and_debug_replay(self) -> None:
        defaults = build_parser().parse_args([])
        enabled = build_parser().parse_args(
            ["--smoke-profile", "moderate", "--replay-debug-smoke"]
        )

        self.assertEqual(defaults.smoke_profile, "off")
        self.assertFalse(defaults.replay_debug_smoke)
        self.assertEqual(enabled.smoke_profile, "moderate")
        self.assertTrue(enabled.replay_debug_smoke)

    def test_smoke_generation_is_seeded_and_opt_in(self) -> None:
        off_world = GridWorld.generate(SimulationConfig(seed=4))
        config = SimulationConfig(seed=4, smoke_profile="moderate")

        first = GridWorld.generate(config)
        second = GridWorld.generate(config)

        self.assertEqual(first.smoke, second.smoke)
        self.assertEqual(off_world.smoke, SmokeField())
        self.assertTrue(first.smoke.cells)
        self.assertTrue(
            all(cell.position not in first.walls for cell in first.smoke.cells)
        )

    def test_unknown_smoke_profile_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "smoke_profile"):
            SimulationConfig(smoke_profile="opaque")

    def test_smoke_reduces_only_survivor_detection(self) -> None:
        width = height = 7
        walls = frozenset(
            Position(x, y)
            for y in range(height)
            for x in range(width)
            if x in (0, width - 1) or y in (0, height - 1)
        )
        origin = Position(1, 1)
        survivor = Position(4, 1)
        smoke = SmokeField(
            profile="moderate",
            cells=tuple(
                SmokeCell(Position(x, 1), 0.65) for x in range(1, 5)
            ),
        )
        world = GridWorld(
            width,
            height,
            origin,
            walls,
            frozenset({survivor}),
            smoke,
        )
        sensor = SurvivorSensor(max_range=3)

        clear = sensor.observe_report(
            world,
            origin,
            smoke_profile="off",
            seed=1,
            step=0,
            observer_id="drone-1",
        )
        degraded = sensor.observe_report(
            world,
            origin,
            smoke_profile="moderate",
            seed=1,
            step=0,
            observer_id="drone-1",
        )

        self.assertEqual(clear.visible_survivors, (survivor,))
        self.assertEqual(degraded.visible_survivors, ())
        self.assertEqual(degraded.degraded_attempts, 1)
        # Smoke does not alter occupancy or movement physics.
        self.assertTrue(world.is_free(survivor))


class SmokeSimulationTests(unittest.TestCase):
    def test_single_drone_mode_reports_smoke_metrics(self) -> None:
        result = Simulation(
            SimulationConfig(seed=1, drone_count=1, smoke_profile="moderate")
        ).run()

        self.assertIsNotNone(result.smoke_metrics)
        self.assertIn("smoke", result.to_dict())

    def test_small_smoke_benchmark_is_deterministic(self) -> None:
        first = run_benchmark(seeds=2)
        second = run_benchmark(seeds=2)

        self.assertEqual(first, second)
        self.assertTrue(first["acceptance"]["deterministic_repeats"])
        self.assertTrue(first["acceptance"]["moderate_collision_free"])

    def test_explicit_off_matches_previous_default_exactly(self) -> None:
        default = SimulationConfig(seed=7, drone_count=2)
        explicit = SimulationConfig(
            seed=7, drone_count=2, smoke_profile="off"
        )

        default_result = MultiDroneSimulation(default).run()
        explicit_result = MultiDroneSimulation(explicit).run()

        self.assertEqual(default_result, explicit_result)
        self.assertEqual(
            replay_json_bytes(generate_replay(default)),
            replay_json_bytes(generate_replay(explicit)),
        )
        self.assertEqual(
            replay_json_bytes(generate_replay(explicit)),
            (REPOSITORY_ROOT / "replays" / "seed_7.json")
            .read_bytes()
            .replace(b"\r\n", b"\n"),
        )
        self.assertNotIn("smoke", default_result.to_dict())

    def test_moderate_profile_is_deterministic_and_emits_telemetry(self) -> None:
        config = SimulationConfig(
            seed=1, drone_count=2, smoke_profile="moderate"
        )

        first = MultiDroneSimulation(config).run()
        second = MultiDroneSimulation(config).run()

        self.assertEqual(first, second)
        self.assertIsNotNone(first.smoke_metrics)
        assert first.smoke_metrics is not None
        self.assertGreater(
            first.smoke_metrics["degraded_detection_attempts"], 0
        )
        event_types = {event.event_type for event in first.mission_events}
        self.assertIn(EventType.SMOKE_ENTERED, event_types)
        self.assertIn(EventType.SURVIVOR_DETECTION_DEGRADED, event_types)
        for event in first.mission_events:
            if event.event_type is EventType.SURVIVOR_DETECTION_DEGRADED:
                self.assertEqual(
                    event.position,
                    first.position_trace_by_drone[event.drone_id][event.step],
                )

    def test_smoke_replay_hides_ground_truth_unless_debug_is_requested(
        self,
    ) -> None:
        config = SimulationConfig(
            seed=0, drone_count=2, smoke_profile="moderate"
        )

        operator_replay = generate_replay(config)
        debug_replay = generate_replay(config, include_debug_smoke=True)

        self.assertEqual(
            operator_replay["schema_version"], SMOKE_REPLAY_SCHEMA_VERSION
        )
        self.assertNotIn("smoke_debug", operator_replay["map"])
        self.assertNotIn("smoke", operator_replay["frames"][0])
        self.assertNotIn(
            "smoke", operator_replay["frames"][0]["knowledge_maps"]["operator"]
        )
        self.assertTrue(debug_replay["map"]["smoke_debug"]["debug_only"])
        densities = debug_replay["map"]["smoke_debug"]["density"]
        self.assertEqual(len(densities), config.height)
        self.assertEqual(len(densities[0]), config.width)
        self.assertTrue(any(value > 0 for row in densities for value in row))

    def test_dashboard_accepts_smoke_and_legacy_replays(self) -> None:
        config = SimulationConfig(
            seed=0, drone_count=2, smoke_profile="moderate"
        )
        temporary = REPOSITORY_ROOT / "smoke-dashboard-test.json"
        try:
            write_replay(generate_replay(config), temporary)
            _validate_replay(temporary)
            _validate_replay(REPOSITORY_ROOT / "replays" / "seed_7.json")
        finally:
            temporary.unlink(missing_ok=True)

        javascript = (
            REPOSITORY_ROOT
            / "src"
            / "echorescue"
            / "dashboard_assets"
            / "app.js"
        ).read_text(encoding="utf-8")
        self.assertIn('state.mapView === "smoke-debug"', javascript)
        self.assertIn("replay.map.smoke_debug?.debug_only", javascript)

    def test_failure_scenario_stays_functional_with_smoke_off(self) -> None:
        result = MultiDroneSimulation(
            SimulationConfig(
                seed=7,
                drone_count=2,
                smoke_profile="off",
                failure_schedule=(("drone-2", 4),),
            )
        ).run()

        self.assertTrue(result.mission_success)
        self.assertEqual(result.collisions, 0)
        self.assertEqual(result.drone_drone_collisions, 0)
        self.assertEqual(result.drones_returned, 1)


if __name__ == "__main__":
    unittest.main()
