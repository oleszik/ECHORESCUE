import json
import tempfile
import threading
import unittest
from pathlib import Path
from urllib.request import urlopen

from echorescue.benchmark import BENCHMARK_SCHEMA_VERSION, run_benchmark
from echorescue.config import SimulationConfig
from echorescue.dashboard import ASSET_DIRECTORY, create_server
from echorescue.environment import GridWorld
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.replay import (
    REPLAY_SCHEMA_VERSION,
    generate_replay,
    record_simulation,
    replay_json_bytes,
    write_replay,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class ReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = SimulationConfig(seed=7, drone_count=2)
        cls.replay, cls.result = record_simulation(
            MultiDroneSimulation(cls.config)
        )

    def test_same_seed_produces_byte_identical_replay(self) -> None:
        first = generate_replay(self.config)
        second = generate_replay(self.config)

        self.assertEqual(replay_json_bytes(first), replay_json_bytes(second))

    def test_schema_is_versioned_and_every_frame_has_both_drones(self) -> None:
        self.assertEqual(
            self.replay["schema_version"], REPLAY_SCHEMA_VERSION
        )
        self.assertTrue(self.replay["frames"])
        for frame in self.replay["frames"]:
            self.assertEqual(
                set(frame["drones"]), {"drone-1", "drone-2"}
            )

    def test_replay_exposes_only_discovered_world_state(self) -> None:
        payload = json.dumps(self.replay, sort_keys=True)
        for forbidden_key in (
            '"ground_truth"',
            '"walls"',
            '"survivor_positions"',
            '"unconfirmed_survivors"',
        ):
            self.assertNotIn(forbidden_key, payload)

        world = GridWorld.generate(self.config)
        confirmed: set[tuple[int, int]] = set()
        for frame in self.replay["frames"]:
            for event in frame["events"]:
                if event["event_type"] == "survivor_confirmed":
                    confirmed.add(tuple(event["position"]))
            visible = {
                tuple(position) for position in frame["confirmed_survivors"]
            }
            self.assertEqual(visible, confirmed)
            self.assertTrue(visible.issubset({(p.x, p.y) for p in world.survivors}))

        initial_map = self.replay["map"]["initial_known_occupancy"]
        self.assertIn("?", "".join(initial_map))
        self.assertNotIn("S", "".join(initial_map))

    def test_event_order_matches_simulation_log(self) -> None:
        replay_events = [
            event
            for frame in self.replay["frames"]
            for event in frame["events"]
        ]
        expected = [event.to_dict() for event in self.result.mission_events]

        self.assertEqual(replay_events, expected)

    def test_final_metrics_match_existing_result_json(self) -> None:
        self.assertEqual(self.replay["metrics"], self.result.to_dict())


class DashboardAndBenchmarkTests(unittest.TestCase):
    def test_dashboard_assets_and_versioned_artifacts_are_present(self) -> None:
        for name in ("index.html", "styles.css", "app.js"):
            self.assertTrue((ASSET_DIRECTORY / name).is_file())
        replay_path = REPOSITORY_ROOT / "replays" / "seed_7.json"
        benchmark_path = (
            REPOSITORY_ROOT / "benchmarks" / "two_drone_50_seeds.json"
        )
        self.assertEqual(
            json.loads(replay_path.read_text(encoding="utf-8"))[
                "schema_version"
            ],
            REPLAY_SCHEMA_VERSION,
        )
        self.assertEqual(
            json.loads(benchmark_path.read_text(encoding="utf-8"))[
                "schema_version"
            ],
            BENCHMARK_SCHEMA_VERSION,
        )

    def test_dashboard_server_serves_assets_and_selected_replay(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            replay_path = write_replay(
                generate_replay(SimulationConfig(seed=3, drone_count=2)),
                Path(temporary_directory) / "replay.json",
            )
            benchmark_path = (
                REPOSITORY_ROOT
                / "benchmarks"
                / "two_drone_50_seeds.json"
            )
            server = create_server(replay_path, benchmark_path, port=0)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                host, port = server.server_address[:2]
                with urlopen(f"http://{host}:{port}/", timeout=5) as response:
                    html = response.read().decode("utf-8")
                with urlopen(
                    f"http://{host}:{port}/replay.json", timeout=5
                ) as response:
                    replay = json.load(response)
                with urlopen(
                    f"http://{host}:{port}/benchmark.json", timeout=5
                ) as response:
                    benchmark = json.load(response)
                with urlopen(
                    f"http://{host}:{port}/app.js", timeout=5
                ) as response:
                    javascript = response.read().decode("utf-8")
                with urlopen(
                    f"http://{host}:{port}/styles.css", timeout=5
                ) as response:
                    stylesheet = response.read().decode("utf-8")

                self.assertIn("EchoRescue", html)
                self.assertIn("drawMission", javascript)
                self.assertIn(".dashboard-grid", stylesheet)
                self.assertEqual(
                    replay["schema_version"], REPLAY_SCHEMA_VERSION
                )
                self.assertEqual(
                    benchmark["schema_version"], BENCHMARK_SCHEMA_VERSION
                )
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=5)

    def test_small_benchmark_is_deterministic_and_versioned(self) -> None:
        first = run_benchmark(seed_count=2)
        second = run_benchmark(seed_count=2)

        self.assertEqual(first, second)
        self.assertEqual(first["schema_version"], BENCHMARK_SCHEMA_VERSION)
        self.assertEqual(first["suite"]["determinism_check"], "passed")


if __name__ == "__main__":
    unittest.main()
