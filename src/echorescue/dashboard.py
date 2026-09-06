import argparse
import json
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from socket import socket
from socketserver import BaseServer
from urllib.parse import unquote

from echorescue.replay import (
    CONSTRAINED_REPLAY_SCHEMA_VERSION,
    DYNAMIC_OBSTACLE_REPLAY_SCHEMA_VERSION,
    FAILURE_RECOVERY_REPLAY_SCHEMA_VERSION,
    NETWORK_AWARE_REPLAY_SCHEMA_VERSION,
    MULTI_RELAY_REPLAY_SCHEMA_VERSION,
    MULTI_FLOOR_REPLAY_SCHEMA_VERSION,
    NOISY_PERCEPTION_REPLAY_SCHEMA_VERSION,
    ROLE_FAILURE_REPLAY_SCHEMA_VERSION,
    REPLAY_SCHEMA_VERSION,
    SMOKE_REPLAY_SCHEMA_VERSION,
)


ASSET_DIRECTORY = Path(__file__).with_name("dashboard_assets")
REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REPLAY = Path("replays/seed_44_4_agents.json")

SCENARIOS: tuple[dict[str, object], ...] = (
    {
        "id": "multi-agent",
        "name": "Multi-Agent Exploration",
        "description": "Four drones coordinate frontier exploration and return to base.",
        "replay": "replays/seed_44_4_agents.json",
        "benchmark": "benchmarks/n_agent_scaling_50_seeds.json",
        "mode": "Recorded simulation",
    },
    {
        "id": "multi-floor",
        "name": "Multi-Floor Mission",
        "description": "Four drones explore three connected floors with explicit transitions.",
        "replay": "replays/seed_68_multi_floor.json",
        "benchmark": "benchmarks/multi_floor_50_holdout_seeds.json",
        "mode": "Recorded simulation",
    },
    {
        "id": "network-relay",
        "name": "Network & Relay",
        "description": "Two drones operate over constrained transport with a network-aware Relay.",
        "replay": "replays/seed_7_network_aware.json",
        "benchmark": "benchmarks/network_aware_relay_100_seeds.json",
        "mode": "Recorded simulation",
    },
    {
        "id": "uncertain-perception",
        "name": "Uncertain Perception",
        "description": "Two drones use probabilistic occupancy and uncertainty-aware planning under medium noise.",
        "replay": "replays/seed_11000_uncertain.json",
        "benchmark": None,
        "mode": "Recorded simulation",
    },
    {
        "id": "ros2-closed-loop",
        "name": "ROS 2 Closed Loop",
        "description": "One-drone simulator-integration proof across a validated ROS 2 process boundary.",
        "replay": "replays/v0.13-ros2-closed-loop.json",
        "benchmark": None,
        "mode": "Recorded ROS 2 reference transport",
    },
)


def _repository_file(relative_path: str | Path) -> Path:
    candidate = REPOSITORY_ROOT / relative_path
    if candidate.is_file():
        return candidate
    return Path(relative_path).resolve()


def _scenario_public_entry(scenario: dict[str, object]) -> dict[str, object]:
    replay_path = _repository_file(str(scenario["replay"]))
    available = replay_path.is_file()
    details: dict[str, object] = {
        key: value for key, value in scenario.items() if key not in {"replay", "benchmark"}
    }
    details["available"] = available
    if available:
        replay = json.loads(replay_path.read_text(encoding="utf-8"))
        first_frame = replay["frames"][0]
        details.update(
            {
                "drones": len(first_frame.get("drones", {})),
                "floors": replay["mission"].get("floor_count", 1),
                "seed": replay["mission"].get("seed"),
                "schema_version": replay.get("schema_version"),
            }
        )
    return details


def _project_summary() -> dict[str, object]:
    artifact = json.loads(
        _repository_file("benchmarks/uncertainty_holdout.json").read_text(encoding="utf-8")
    )
    profiles = ("clean", "low_noise", "medium_noise", "high_noise")
    return {
        "source": "benchmarks/uncertainty_holdout.json",
        "runs": len(artifact["rows"]),
        "profiles": [
            {
                "id": profile,
                "label": profile.replace("_", " ").title(),
                "naive": artifact["groups"][profile]["naive"]["success"]["mean"],
                "aware": artifact["groups"][profile]["uncertainty-aware"]["success"]["mean"],
                "n": artifact["groups"][profile]["naive"]["success"]["n"],
            }
            for profile in profiles
        ],
        "clean_duration_difference": artifact["paired_candidate_minus_naive"]["clean"][
            "both_successful_duration"
        ],
    }


def _validate_replay(path: Path) -> None:
    with path.open("r", encoding="utf-8") as replay_file:
        replay = json.load(replay_file)
    if replay.get("schema_version") not in {
        REPLAY_SCHEMA_VERSION,
        CONSTRAINED_REPLAY_SCHEMA_VERSION,
        NETWORK_AWARE_REPLAY_SCHEMA_VERSION,
        FAILURE_RECOVERY_REPLAY_SCHEMA_VERSION,
        SMOKE_REPLAY_SCHEMA_VERSION,
        NOISY_PERCEPTION_REPLAY_SCHEMA_VERSION,
        DYNAMIC_OBSTACLE_REPLAY_SCHEMA_VERSION,
        ROLE_FAILURE_REPLAY_SCHEMA_VERSION,
        MULTI_RELAY_REPLAY_SCHEMA_VERSION,
        MULTI_FLOOR_REPLAY_SCHEMA_VERSION,
    }:
        raise ValueError(
            f"unsupported replay schema: {replay.get('schema_version')!r}"
        )


def create_server(
    replay_path: str | Path,
    benchmark_path: str | Path | None = None,
    host: str = "127.0.0.1",
    port: int = 8000,
) -> ThreadingHTTPServer:
    replay = Path(replay_path).resolve()
    if not replay.is_file():
        raise FileNotFoundError(f"replay not found: {replay}")
    _validate_replay(replay)
    benchmark = Path(benchmark_path).resolve() if benchmark_path else None
    if benchmark is not None and not benchmark.is_file():
        raise FileNotFoundError(f"benchmark not found: {benchmark}")

    class DashboardHandler(SimpleHTTPRequestHandler):
        def __init__(
            self,
            request: socket | tuple[bytes, socket],
            client_address: tuple[str, int],
            server: BaseServer,
        ) -> None:
            super().__init__(
                request,
                client_address,
                server,
                directory=str(ASSET_DIRECTORY),
            )

        def _send_json_file(self, path: Path) -> None:
            body = path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            request_path = self.path.split("?", 1)[0]
            if request_path == "/replay.json":
                self._send_json_file(replay)
                return
            if request_path == "/benchmark.json":
                if benchmark is None:
                    self.send_error(404, "No benchmark artifact configured")
                else:
                    self._send_json_file(benchmark)
                return
            if request_path == "/api/scenarios":
                self._send_json({"default": "multi-agent", "scenarios": [_scenario_public_entry(item) for item in SCENARIOS]})
                return
            if request_path == "/api/project-summary":
                self._send_json(_project_summary())
                return
            parts = request_path.strip("/").split("/")
            if len(parts) == 4 and parts[:2] == ["api", "scenarios"]:
                scenario_id, artifact_type = unquote(parts[2]), parts[3]
                scenario = next((item for item in SCENARIOS if item["id"] == scenario_id), None)
                if scenario is None or artifact_type not in {"replay", "benchmark"}:
                    self.send_error(404, "Unknown scenario artifact")
                    return
                relative = scenario.get(artifact_type)
                if not relative:
                    self.send_error(404, f"No {artifact_type} artifact for this scenario")
                    return
                path = _repository_file(str(relative))
                if not path.is_file():
                    self.send_error(404, f"Scenario {artifact_type} is unavailable")
                    return
                self._send_json_file(path)
                return
            super().do_GET()

        def _send_json(self, payload: dict[str, object]) -> None:
            body = (json.dumps(payload, sort_keys=True) + "\n").encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format: str, *args: object) -> None:
            return

    return ThreadingHTTPServer((host, port), DashboardHandler)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Serve the local EchoRescue replay dashboard."
    )
    parser.add_argument(
        "--replay",
        default=str(DEFAULT_REPLAY),
        help="Replay to open initially (default: four-drone portfolio demo)",
    )
    parser.add_argument("--benchmark")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    replay_path = (
        _repository_file(DEFAULT_REPLAY)
        if Path(args.replay) == DEFAULT_REPLAY
        else Path(args.replay)
    )
    benchmark = args.benchmark
    default_benchmark = Path("benchmarks/n_agent_scaling_50_seeds.json")
    if benchmark is None:
        resolved_benchmark = _repository_file(default_benchmark)
        benchmark = resolved_benchmark if resolved_benchmark.is_file() else None
    server = create_server(replay_path, benchmark, args.host, args.port)
    print(
        f"EchoRescue dashboard: http://{args.host}:{server.server_port}",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
