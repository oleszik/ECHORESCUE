import json
import re
import subprocess
import tempfile
import threading
import unittest
from pathlib import Path
from urllib.error import HTTPError
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
STYLESHEET_PATH = ASSET_DIRECTORY / "styles.css"
APP_PATH = ASSET_DIRECTORY / "app.js"


def benchmark_view(payload: object) -> dict[str, object]:
    script = f"""
const app = require({json.dumps(str(APP_PATH))});
let input = "";
process.stdin.setEncoding("utf8");
process.stdin.on("data", chunk => input += chunk);
process.stdin.on("end", () => process.stdout.write(
  JSON.stringify(app.safeBenchmarkView(JSON.parse(input)))
));
"""
    completed = subprocess.run(
        ["node", "-e", script],
        input=json.dumps(payload),
        text=True,
        capture_output=True,
        check=True,
    )
    return json.loads(completed.stdout)


def css_block(stylesheet: str, selector: str) -> str:
    match = re.search(
        rf"(?ms)^\s*{re.escape(selector)}\s*\{{(.*?)\}}",
        stylesheet,
    )
    if match is None:
        raise AssertionError(f"missing CSS selector: {selector}")
    return match.group(1)


def relative_luminance(hex_color: str) -> float:
    channels = [
        int(hex_color[index : index + 2], 16) / 255
        for index in (1, 3, 5)
    ]
    linear = [
        channel / 12.92
        if channel <= 0.04045
        else ((channel + 0.055) / 1.055) ** 2.4
        for channel in channels
    ]
    return 0.2126 * linear[0] + 0.7152 * linear[1] + 0.0722 * linear[2]


def contrast_ratio(first: str, second: str) -> float:
    light, dark = sorted(
        (relative_luminance(first), relative_luminance(second)), reverse=True
    )
    return (light + 0.05) / (dark + 0.05)


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
            self.assertEqual(
                set(frame["communication"]["nodes"]),
                {"base", "drone-1", "drone-2"},
            )
            self.assertEqual(
                frame["communication"]["nodes"]["base"],
                self.replay["map"]["base"],
            )
            for drone_id, drone in frame["drones"].items():
                self.assertIn("communication", drone)
                self.assertIn("knowledge", drone)
                self.assertEqual(
                    frame["communication"]["nodes"][drone_id],
                    drone["position"],
                )
            for link in frame["communication"]["links"]:
                self.assertIn(link["from"], frame["communication"]["nodes"])
                self.assertIn(link["to"], frame["communication"]["nodes"])
                self.assertIn(link["kind"], {"direct_base", "relay", "peer"})
            self.assertEqual(
                set(frame["knowledge_maps"]),
                {"operator", "drone-1", "drone-2", "base"},
            )
            for view in frame["knowledge_maps"].values():
                self.assertEqual(len(view["occupancy"]), self.config.height)

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

    def test_active_local_replay_separates_operator_and_base_knowledge(self) -> None:
        replay = generate_replay(
            SimulationConfig(
                seed=7,
                drone_count=2,
                communication_range=8,
                knowledge_mode="local",
            )
        )

        self.assertEqual(replay["mission"]["knowledge_mode"], "local")
        for frame in replay["frames"]:
            maps = frame["knowledge_maps"]
            self.assertEqual(
                maps["operator"]["purpose"], "evaluation_aggregate"
            )
            self.assertEqual(
                maps["base"]["purpose"], "base_operational_knowledge"
            )
            self.assertEqual(
                frame["confirmed_survivors"],
                maps["base"]["confirmed_survivors"],
            )
            self.assertEqual(
                maps["operator"]["confirmed_survivors"],
                maps["base"]["confirmed_survivors"],
            )


class DashboardAndBenchmarkTests(unittest.TestCase):
    def test_dashboard_assets_and_versioned_artifacts_are_present(self) -> None:
        for name in ("index.html", "styles.css", "app.js"):
            self.assertTrue((ASSET_DIRECTORY / name).is_file())
        replay_path = REPOSITORY_ROOT / "replays" / "seed_7.json"
        local_replay_path = (
            REPOSITORY_ROOT / "replays" / "seed_7_local.json"
        )
        benchmark_path = (
            REPOSITORY_ROOT / "benchmarks" / "two_drone_50_seeds.json"
        )
        self.assertEqual(
            json.loads(replay_path.read_text(encoding="utf-8"))[
                "schema_version"
            ],
            REPLAY_SCHEMA_VERSION,
        )
        local_replay = json.loads(
            local_replay_path.read_text(encoding="utf-8")
        )
        self.assertEqual(local_replay["schema_version"], REPLAY_SCHEMA_VERSION)
        self.assertEqual(local_replay["mission"]["knowledge_mode"], "local")
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
                self.assertIn("drawCommunicationLinks", javascript)
                self.assertIn("droneCommunication1", html)
                self.assertIn("mapViewSelect", html)
                self.assertIn("knowledgeMode", html)
                self.assertIn("selectedKnowledgeMap", javascript)
                self.assertIn("evaluation aggregate", javascript.lower())
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

    def test_relay_replay_works_without_a_benchmark(self) -> None:
        replay_path = REPOSITORY_ROOT / "replays" / "seed_7_relay.json"
        server = create_server(replay_path, benchmark_path=None, port=0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            host, port = server.server_address[:2]
            with urlopen(f"http://{host}:{port}/replay.json", timeout=5) as response:
                replay = json.load(response)
            with self.assertRaises(HTTPError) as error:
                urlopen(f"http://{host}:{port}/benchmark.json", timeout=5)
            self.assertEqual(error.exception.code, 404)
            self.assertEqual(replay["mission"]["relay_strategy"], "adaptive")
            self.assertEqual(
                benchmark_view(None),
                {
                    "status": "unavailable",
                    "message": "Benchmark artifact unavailable.",
                },
            )
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=5)

    def test_adaptive_relay_benchmark_schema_is_mapped_correctly(self) -> None:
        payload = json.loads(
            (
                REPOSITORY_ROOT
                / "benchmarks"
                / "adaptive_relay_50_seeds.json"
            ).read_text(encoding="utf-8")
        )

        view = benchmark_view(payload)

        self.assertEqual(view["status"], "ready")
        self.assertEqual(view["format"], "adaptive_relay")
        self.assertEqual(view["baselineLabel"], "Relay off")
        self.assertEqual(view["candidateLabel"], "Adaptive relay")
        self.assertEqual(view["baselineSteps"], 75.3)
        self.assertEqual(view["candidateSteps"], 77.92)
        self.assertEqual(view["improvementValue"], "+1.40 pp")

    def test_older_versioned_benchmark_formats_remain_supported(self) -> None:
        expected = {
            "two_drone_50_seeds.json": "parallel_exploration",
            "knowledge_modes_50_seeds.json": "knowledge_modes",
            "distributed_deconfliction_50_seeds.json": (
                "distributed_deconfliction"
            ),
            "communication_50_seeds.json": "communication",
            "shadow_mode_50_seeds.json": "shadow_mode",
        }
        for filename, format_name in expected.items():
            with self.subTest(filename=filename):
                payload = json.loads(
                    (REPOSITORY_ROOT / "benchmarks" / filename).read_text(
                        encoding="utf-8"
                    )
                )
                view = benchmark_view(payload)
                self.assertEqual(view["status"], "ready")
                self.assertEqual(view["format"], format_name)

    def test_missing_average_mission_steps_is_rendered_as_unavailable(self) -> None:
        payload = json.loads(
            (
                REPOSITORY_ROOT
                / "benchmarks"
                / "adaptive_relay_50_seeds.json"
            ).read_text(encoding="utf-8")
        )
        del payload["active_local_adaptive_relay"]["average_mission_steps"]

        view = benchmark_view(payload)

        self.assertEqual(view["status"], "ready")
        self.assertEqual(view["baselineSteps"], 75.3)
        self.assertIsNone(view["candidateSteps"])

    def test_unknown_and_incomplete_benchmarks_report_precise_errors(self) -> None:
        unknown = benchmark_view({"schema_version": "9.9", "metrics": {}})
        incomplete = benchmark_view(
            {
                "schema_version": "1.0",
                "active_local_relay_off": {},
            }
        )

        self.assertEqual(unknown["status"], "invalid")
        self.assertIn("Unrecognized benchmark format", unknown["message"])
        self.assertIn("schema_version: 9.9", unknown["message"])
        self.assertEqual(incomplete["status"], "invalid")
        self.assertIn(
            'missing required object "active_local_adaptive_relay"',
            incomplete["message"],
        )
        malformed = benchmark_view(
            {"__benchmark_load_error": "Unable to parse /benchmark.json as JSON"}
        )
        self.assertEqual(malformed["status"], "invalid")
        self.assertIn("Unable to parse /benchmark.json", malformed["message"])


class DashboardStyleRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.stylesheet = STYLESHEET_PATH.read_text(encoding="utf-8")
        root = css_block(cls.stylesheet, ":root")
        cls.variables = dict(
            re.findall(r"--([\w-]+)\s*:\s*(#[0-9a-fA-F]{6})\s*;", root)
        )

    def test_required_contrast_variables_exist(self) -> None:
        required = {
            "background",
            "panel",
            "panel-raised",
            "border",
            "text-primary",
            "text-secondary",
            "text-muted",
            "drone-one",
            "drone-two",
            "success",
            "danger",
        }

        self.assertTrue(required.issubset(self.variables))

    def test_foreground_and_background_colors_are_distinct_and_readable(self) -> None:
        backgrounds = {
            self.variables["background"],
            self.variables["panel"],
            self.variables["panel-raised"],
        }
        foregrounds = {
            self.variables["text-primary"],
            self.variables["text-secondary"],
        }

        self.assertTrue(backgrounds.isdisjoint(foregrounds))
        for background in backgrounds:
            self.assertGreaterEqual(
                contrast_ratio(self.variables["text-primary"], background),
                7.0,
            )
            self.assertGreaterEqual(
                contrast_ratio(self.variables["text-secondary"], background),
                4.5,
            )

    def test_large_layout_containers_do_not_dim_the_interface(self) -> None:
        for selector in ("body", ".app-shell", ".dashboard-grid"):
            declarations = css_block(self.stylesheet, selector)
            opacity = re.search(r"(?:^|;)\s*opacity\s*:\s*([\d.]+)", declarations)
            if opacity is not None:
                self.assertGreaterEqual(float(opacity.group(1)), 0.8)
            self.assertNotRegex(declarations, r"(?:^|;)\s*filter\s*:")

    def test_fullscreen_error_overlay_respects_hidden_state(self) -> None:
        overlay = css_block(self.stylesheet, ".fatal-error")
        self.assertIn("position: fixed", overlay)
        self.assertNotRegex(overlay, r"(?:^|;)\s*display\s*:")
        self.assertRegex(
            css_block(self.stylesheet, ".fatal-error[hidden]"),
            r"display\s*:\s*none\s*!important",
        )
        self.assertRegex(
            css_block(self.stylesheet, ".fatal-error:not([hidden])"),
            r"display\s*:\s*grid",
        )
        self.assertNotRegex(
            self.stylesheet,
            r"(?s)(?:body|\.app-shell|\.dashboard-grid)::(?:before|after)"
            r"\s*\{[^}]*position\s*:\s*fixed[^}]*z-index",
        )

    def test_map_uses_replay_aspect_ratio_without_fixed_tall_viewport(self) -> None:
        canvas = css_block(self.stylesheet, ".canvas-wrap")
        javascript = (ASSET_DIRECTORY / "app.js").read_text(encoding="utf-8")

        self.assertIn("aspect-ratio: 21 / 13", canvas)
        self.assertIn("replay.map.width", javascript)
        self.assertNotRegex(canvas, r"height\s*:\s*min\(")

    def test_landed_shared_base_drones_use_separate_docking_markers(self) -> None:
        javascript = (ASSET_DIRECTORY / "app.js").read_text(encoding="utf-8")

        self.assertIn("function dockingMarkerOffset", javascript)
        self.assertIn('drone.state === "LANDED"', javascript)
        self.assertIn(
            "dockingMarkerOffset(frame, droneId, geometry)", javascript
        )


if __name__ == "__main__":
    unittest.main()
