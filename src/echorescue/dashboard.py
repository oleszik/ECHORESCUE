import argparse
import json
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from echorescue.replay import REPLAY_SCHEMA_VERSION


ASSET_DIRECTORY = Path(__file__).with_name("dashboard_assets")


def _validate_replay(path: Path) -> None:
    with path.open("r", encoding="utf-8") as replay_file:
        replay = json.load(replay_file)
    if replay.get("schema_version") != REPLAY_SCHEMA_VERSION:
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
        def __init__(self, *args: object, **kwargs: object) -> None:
            super().__init__(*args, directory=str(ASSET_DIRECTORY), **kwargs)

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
            super().do_GET()

        def log_message(self, format: str, *args: object) -> None:
            return

    return ThreadingHTTPServer((host, port), DashboardHandler)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Serve the local EchoRescue replay dashboard."
    )
    parser.add_argument("--replay", required=True)
    parser.add_argument("--benchmark")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    benchmark = args.benchmark
    default_benchmark = Path("benchmarks/two_drone_50_seeds.json")
    if benchmark is None and default_benchmark.is_file():
        benchmark = default_benchmark
    server = create_server(args.replay, benchmark, args.host, args.port)
    address, port = server.server_address[:2]
    print(f"EchoRescue dashboard: http://{address}:{port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
