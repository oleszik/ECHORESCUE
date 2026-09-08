"""v0.14.0 diagnostics and real external-simulator orchestration.

This module intentionally contains no flight-control behavior.  It validates
and starts the official ArduPilot Gazebo example only when the configured
external dependencies are genuinely available.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from enum import Enum
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
from typing import Any, Callable, Iterable, Mapping, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = REPOSITORY_ROOT / "config" / "simulator-stack-v0.14.0.json"


class Status(str, Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"


@dataclass(frozen=True)
class Check:
    name: str
    status: Status
    version: str | None
    detail: str


def check(name: str, status: Status, detail: str, version: str | None = None) -> Check:
    return Check(name=name, status=status, version=version, detail=detail)


def serialize_report(report: Mapping[str, Any]) -> str:
    """Return byte-stable, human-readable JSON for repository-owned reports."""
    return json.dumps(report, indent=2, sort_keys=True, ensure_ascii=True) + "\n"


def parse_version(text: str, pattern: str) -> str | None:
    match = re.search(pattern, text, flags=re.IGNORECASE | re.MULTILINE)
    return match.group(1) if match else None


def expected_lines(output: str, expected: Iterable[str]) -> tuple[bool, list[str]]:
    actual = {line.strip() for line in output.splitlines() if line.strip()}
    missing = sorted(item for item in expected if item not in actual)
    return not missing, missing


def telemetry_progressed(samples: Sequence[Mapping[str, Any]]) -> bool:
    """Changing clocks or values establish a live, advancing MAVLink stream."""
    if len(samples) < 2:
        return False
    normalized = [serialize_report(dict(sample)) for sample in samples]
    return len(set(normalized)) > 1


def _run_version(command: Sequence[str]) -> tuple[int, str]:
    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return 1, str(error)
    return completed.returncode, (completed.stdout + completed.stderr).strip()


def _package_version(name: str) -> str | None:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None


def _ubuntu_version() -> str | None:
    path = Path("/etc/os-release")
    if not path.exists():
        return None
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key] = value.strip('"')
    return values.get("VERSION_ID") if values.get("ID") == "ubuntu" else None


def _executable_check(name: str, executable: str, version_args: Sequence[str]) -> Check:
    path = shutil.which(executable)
    if path is None:
        return check(name, Status.FAIL, f"missing executable: {executable}")
    _, output = _run_version([path, *version_args])
    first_line = next((line.strip() for line in output.splitlines() if line.strip()), "version unreported")
    return check(name, Status.PASS, path, first_line[:160])


def _ros_package_check(ros_path: str, package_name: str) -> Check:
    code, output = _run_version([ros_path, "pkg", "prefix", package_name])
    if code == 0 and output:
        return check(f"ros_package.{package_name}", Status.PASS, output.splitlines()[0])
    return check(f"ros_package.{package_name}", Status.SKIP, "not installed; optional for the direct JSON baseline")


def _port_check(name: str, host: str, port: int, protocol: str) -> Check:
    sock_type = socket.SOCK_DGRAM if protocol == "udp" else socket.SOCK_STREAM
    with socket.socket(socket.AF_INET, sock_type) as probe:
        try:
            probe.bind((host, port))
        except OSError as error:
            return check(name, Status.FAIL, f"{protocol} {host}:{port} unavailable: {error}")
    return check(name, Status.PASS, f"{protocol} {host}:{port} is available")


def _rendering_check() -> Check:
    display = os.environ.get("WAYLAND_DISPLAY") or os.environ.get("DISPLAY")
    if not display:
        return check("rendering", Status.SKIP, "no DISPLAY or WAYLAND_DISPLAY; headless mode remains available")
    glxinfo = shutil.which("glxinfo")
    if glxinfo is None:
        return check("rendering", Status.SKIP, "display is present but glxinfo is missing; install mesa-utils to verify acceleration")
    code, output = _run_version([glxinfo, "-B"])
    renderer = parse_version(output, r"OpenGL renderer string:\s*(.+)")
    if code != 0 or renderer is None:
        return check("rendering", Status.FAIL, "glxinfo could not identify an OpenGL renderer")
    software = any(marker in renderer.lower() for marker in ("llvmpipe", "softpipe", "software rasterizer"))
    status = Status.FAIL if software else Status.PASS
    detail = "software rendering detected" if software else "hardware-accelerated OpenGL renderer detected"
    return check("rendering", status, detail, renderer)


def load_config(path: Path = DEFAULT_CONFIG) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def diagnose(config: Mapping[str, Any], repository_root: Path = REPOSITORY_ROOT) -> dict[str, Any]:
    baseline = config["baseline"]
    expected_platform = baseline["platform"]
    checks: list[Check] = []

    current_system = platform.system()
    ubuntu = _ubuntu_version()
    architecture = platform.machine()
    platform_ok = (
        current_system == expected_platform["system"]
        and ubuntu == expected_platform["version"]
        and architecture in (expected_platform["architecture"], "amd64")
    )
    checks.append(check(
        "platform",
        Status.PASS if platform_ok else Status.FAIL,
        f"required Ubuntu {expected_platform['version']} {expected_platform['architecture']}; detected {current_system} {ubuntu or platform.release()} {architecture}",
        ubuntu or platform.release(),
    ))

    ros_path = shutil.which("ros2")
    ros_distro = os.environ.get("ROS_DISTRO")
    if ros_path is None:
        checks.append(check("ros2", Status.FAIL, "ros2 is not on PATH; source /opt/ros/jazzy/setup.bash"))
    elif ros_distro != baseline["ros2"]["distribution"]:
        checks.append(check("ros2", Status.FAIL, f"expected ROS_DISTRO=jazzy, detected {ros_distro or 'unsourced'}", ros_distro))
    else:
        checks.append(check("ros2", Status.PASS, ros_path, ros_distro))
    if ros_path is not None:
        graph_code, graph_output = _run_version([ros_path, "node", "list"])
        checks.append(check(
            "ros2.graph_health",
            Status.PASS if graph_code == 0 else Status.FAIL,
            "ROS graph query succeeded" if graph_code == 0 else f"ROS graph query failed: {graph_output[:160]}",
            ros_distro,
        ))
        for package_name in ("ardupilot_msgs", "mavros", "mavros_msgs"):
            checks.append(_ros_package_check(ros_path, package_name))

    gz_path = shutil.which("gz")
    if gz_path is None:
        checks.append(check("gazebo", Status.FAIL, "missing executable: gz"))
    else:
        _, gz_output = _run_version([gz_path, "sim", "--version"])
        gz_version = parse_version(gz_output, r"Gazebo Sim,?\s+version\s+([0-9.]+)") or parse_version(gz_output, r"\b([0-9]+\.[0-9.]+)")
        wanted_major = str(baseline["gazebo"]["gz_sim_major"])
        compatible = gz_version is not None and gz_version.split(".", 1)[0] == wanted_major
        checks.append(check("gazebo", Status.PASS if compatible else Status.FAIL, f"expected Gazebo Harmonic / gz-sim {wanted_major}.x; {gz_output[:160] or 'version unreported'}", gz_version))

    colcon_path = shutil.which("colcon")
    colcon_version = _package_version("colcon-core")
    checks.append(check(
        "colcon",
        Status.PASS if colcon_path else Status.FAIL,
        colcon_path or "missing executable: colcon",
        colcon_version or ("installed; package version unreported" if colcon_path else None),
    ))
    checks.append(_executable_check("mavproxy", "mavproxy.py", ["--version"]))
    pymavlink = _package_version("pymavlink")
    checks.append(check("pymavlink", Status.PASS if pymavlink else Status.FAIL, "Python MAVLink telemetry probe dependency" if pymavlink else "missing Python package: pymavlink", pymavlink))

    ardupilot_home = os.environ.get("ARDUPILOT_HOME")
    sim_vehicle = Path(ardupilot_home, "Tools", "autotest", "sim_vehicle.py") if ardupilot_home else None
    if sim_vehicle is None or not sim_vehicle.is_file():
        checks.append(check("ardupilot_sitl", Status.FAIL, "ARDUPILOT_HOME does not contain Tools/autotest/sim_vehicle.py"))
    else:
        assert ardupilot_home is not None
        code, output = _run_version(["git", "-C", ardupilot_home, "rev-parse", "HEAD"])
        commit = output.strip() if code == 0 else None
        wanted = baseline["ardupilot"]["commit"]
        checks.append(check("ardupilot_sitl", Status.PASS if commit == wanted else Status.FAIL, f"expected {baseline['ardupilot']['ref']} ({wanted}); checkout at {ardupilot_home}", commit))

    plugin_home = os.environ.get("ARDUPILOT_GAZEBO_HOME")
    plugin_library = Path(plugin_home, "build") if plugin_home else None
    if plugin_library is None or not plugin_library.is_dir():
        checks.append(check("ardupilot_gazebo", Status.FAIL, "ARDUPILOT_GAZEBO_HOME does not contain a build directory"))
    else:
        assert plugin_home is not None
        code, output = _run_version(["git", "-C", plugin_home, "rev-parse", "HEAD"])
        commit = output.strip() if code == 0 else None
        wanted = baseline["ardupilot_gazebo"]["ref"]
        checks.append(check("ardupilot_gazebo", Status.PASS if commit == wanted else Status.FAIL, f"expected plugin commit {wanted}", commit))

    required_environment = config["environment"]["required"]
    for variable in sorted(required_environment):
        value = os.environ.get(variable)
        if variable == "GZ_VERSION":
            valid = value == required_environment[variable]
        elif variable == "GZ_SIM_SYSTEM_PLUGIN_PATH" and plugin_home:
            valid = str(Path(plugin_home, "build")) in (value or "").split(os.pathsep)
        elif variable == "GZ_SIM_RESOURCE_PATH" and plugin_home:
            entries = set((value or "").split(os.pathsep))
            valid = {str(Path(plugin_home, "models")), str(Path(plugin_home, "worlds"))}.issubset(entries)
        else:
            valid = bool(value)
        checks.append(check(f"environment.{variable}", Status.PASS if valid else Status.FAIL, value or f"unset; {required_environment[variable]}", value))

    checks.append(check(
        "echorescue_ros_workspace",
        Status.PASS if (repository_root / "ros2_ws" / "src" / "echorescue_ros" / "package.xml").is_file() else Status.FAIL,
        str(repository_root / "ros2_ws"),
    ))
    for port_name, endpoint in sorted(config["network"].items()):
        checks.append(_port_check(f"port.{port_name}", endpoint["host"], int(endpoint["port"]), endpoint["protocol"]))
    checks.append(_rendering_check())
    checks.sort(key=lambda item: item.name)
    return {
        "schema_version": "echorescue-simulator-diagnostic/1.0",
        "milestone": config["milestone"],
        "ready": not any(item.status is Status.FAIL for item in checks),
        "checks": [asdict(item) for item in checks],
    }


class StartupTimeout(RuntimeError):
    pass


class UnexpectedProcessExit(RuntimeError):
    pass


class ProcessSupervisor:
    """Own external process groups and terminate their descendants."""

    def __init__(self) -> None:
        self.processes: list[tuple[str, subprocess.Popen[str]]] = []

    def start(self, name: str, command: Sequence[str], cwd: Path, log: Any) -> subprocess.Popen[str]:
        kwargs: dict[str, Any] = {"cwd": cwd, "stdout": log, "stderr": subprocess.STDOUT, "text": True}
        if os.name == "nt":
            kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            kwargs["start_new_session"] = True
        process = subprocess.Popen(list(command), **kwargs)
        self.processes.append((name, process))
        return process

    def wait_until(self, name: str, process: subprocess.Popen[str], predicate: Callable[[], bool], timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            return_code = process.poll()
            if return_code is not None:
                raise UnexpectedProcessExit(f"{name} exited unexpectedly with code {return_code}")
            if predicate():
                return
            time.sleep(0.1)
        raise StartupTimeout(f"{name} readiness timed out after {timeout_s:.1f}s")

    def cleanup(self, grace_s: float = 5.0) -> list[Check]:
        results: list[Check] = []
        for name, process in reversed(self.processes):
            if os.name == "nt":
                if process.poll() is None:
                    try:
                        process.send_signal(signal.CTRL_BREAK_EVENT)
                        process.wait(timeout=grace_s)
                    except (OSError, subprocess.TimeoutExpired):
                        subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"], capture_output=True, check=False)
                        try:
                            process.wait(timeout=grace_s)
                        except subprocess.TimeoutExpired:
                            pass
                stopped = process.poll() is not None
            else:
                try:
                    self._kill_process_group(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                deadline = time.monotonic() + grace_s
                while time.monotonic() < deadline:
                    process.poll()
                    if not self._group_alive(process.pid):
                        break
                    time.sleep(0.05)
                if self._group_alive(process.pid):
                    try:
                        self._kill_process_group(process.pid, getattr(signal, "SIGKILL", signal.SIGTERM))
                    except ProcessLookupError:
                        pass
                try:
                    process.wait(timeout=grace_s)
                except subprocess.TimeoutExpired:
                    pass
                stopped = process.poll() is not None and not self._group_alive(process.pid)
            results.append(check(f"cleanup.{name}", Status.PASS if stopped else Status.FAIL, f"pid {process.pid} {'stopped' if stopped else 'remains alive'}"))
        return sorted(results, key=lambda item: item.name)

    @staticmethod
    def _kill_process_group(process_group_id: int, requested_signal: int) -> None:
        killpg: Callable[[int, int], None] = getattr(os, "killpg")
        killpg(process_group_id, requested_signal)

    @staticmethod
    def _group_alive(process_group_id: int) -> bool:
        try:
            ProcessSupervisor._kill_process_group(process_group_id, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True


def _tcp_ready(host: str, port: int) -> bool:
    try:
        with socket.create_connection((host, port), timeout=0.25):
            return True
    except OSError:
        return False


def _gazebo_ready(expected_topic: str) -> bool:
    code, output = _run_version(["gz", "topic", "-l"])
    return code == 0 and expected_topic in output


def _mavlink_samples(endpoint: str, minimum: int, timeout_s: float) -> list[dict[str, Any]]:
    from pymavlink import mavutil  # type: ignore[import-not-found]

    connection = mavutil.mavlink_connection(endpoint)
    samples: list[dict[str, Any]] = []
    deadline = time.monotonic() + timeout_s
    try:
        connection.wait_heartbeat(timeout=min(timeout_s, 10.0))
        while time.monotonic() < deadline and len(samples) < minimum:
            message = connection.recv_match(type=["LOCAL_POSITION_NED", "GLOBAL_POSITION_INT"], blocking=True, timeout=1.0)
            if message is not None:
                samples.append(message.to_dict())
    finally:
        connection.close()
    return samples


def _expand_command(parts: Sequence[str], values: Mapping[str, str]) -> list[str]:
    return [part.format_map(values) for part in parts]


def smoke(config: Mapping[str, Any], timeout_s: float, headless: bool) -> dict[str, Any]:
    preflight = diagnose(config)
    if not preflight["ready"]:
        return {
            "schema_version": "echorescue-simulator-smoke/1.0",
            "milestone": config["milestone"],
            "status": Status.SKIP,
            "detail": "preflight failed; no external process was started",
            "checks": preflight["checks"],
            "cleanup": [],
        }

    integration = config["integration"]
    values = {
        "world": integration["world"],
        "model": integration["model"],
        "ardupilot_home": os.environ["ARDUPILOT_HOME"],
    }
    gazebo_command = _expand_command(integration["gazebo_command"], values)
    if headless:
        gazebo_command.extend(integration["gazebo_headless_arguments"])
    sitl_command = _expand_command(integration["sitl_command"], values)
    supervisor = ProcessSupervisor()
    checks: list[Check] = []
    interruption = False
    with tempfile.TemporaryDirectory(prefix="echorescue-v014-") as temp_dir:
        temporary = Path(temp_dir)
        try:
            with (temporary / "gazebo.log").open("w", encoding="utf-8") as gazebo_log:
                gazebo = supervisor.start("gazebo", gazebo_command, REPOSITORY_ROOT, gazebo_log)
                supervisor.wait_until("gazebo", gazebo, lambda: _gazebo_ready(config["health"]["gazebo_topic_contains"]), timeout_s)
                checks.append(check("health.gazebo", Status.PASS, "real Gazebo process exposes the configured Iris world"))
                with (temporary / "sitl.log").open("w", encoding="utf-8") as sitl_log:
                    sitl = supervisor.start("ardupilot_sitl", sitl_command, Path(values["ardupilot_home"]), sitl_log)
                    endpoint = config["network"]["sitl_mavlink"]
                    supervisor.wait_until("ardupilot_sitl", sitl, lambda: _tcp_ready(endpoint["host"], int(endpoint["port"])), timeout_s)
                    samples = _mavlink_samples(endpoint["endpoint"], int(config["health"]["minimum_telemetry_samples"]), timeout_s)
                    advancing = telemetry_progressed(samples)
                    checks.append(check("health.ardupilot_sitl", Status.PASS if advancing else Status.FAIL, f"received {len(samples)} MAVLink position samples; advancing={advancing}"))
                    checks.append(check("health.gazebo_sitl_exchange", Status.PASS if advancing else Status.FAIL, "Gazebo world is live and SITL emits advancing telemetry"))
                    if integration["requires_ros_runtime"]:
                        node_code, nodes = _run_version(["ros2", "node", "list"])
                        topic_code, topics = _run_version(["ros2", "topic", "list"])
                        nodes_ok, missing_nodes = expected_lines(nodes, config["health"]["ros_nodes"])
                        topics_ok, missing_topics = expected_lines(topics, config["health"]["ros_topics"])
                        ros_ok = node_code == 0 and topic_code == 0 and nodes_ok and topics_ok
                        checks.append(check("health.ros2", Status.PASS if ros_ok else Status.FAIL, f"missing nodes={missing_nodes}; missing topics={missing_topics}"))
                    else:
                        checks.append(check("health.ros2", Status.SKIP, "official direct JSON plugin does not require a ROS runtime; ROS readiness is covered by diagnostics"))
        except KeyboardInterrupt:
            interruption = True
            checks.append(check("smoke.interruption", Status.FAIL, "interrupted; cleanup initiated"))
        except Exception as error:
            checks.append(check("smoke.execution", Status.FAIL, str(error)))
        finally:
            cleanup = supervisor.cleanup()
    passed = bool(checks) and not interruption and not any(item.status is Status.FAIL for item in checks + cleanup)
    return {
        "schema_version": "echorescue-simulator-smoke/1.0",
        "milestone": config["milestone"],
        "status": Status.PASS if passed else Status.FAIL,
        "detail": "real external stack verified" if passed else "real external stack was not verified",
        "checks": [asdict(item) for item in sorted(checks, key=lambda item: item.name)],
        "cleanup": [asdict(item) for item in cleanup],
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Diagnose or smoke-test the EchoRescue v0.14.0 simulator baseline")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("diagnose", help="run deterministic environment diagnostics")
    smoke_parser = subparsers.add_parser("smoke", help="opt in to starting the real Gazebo and ArduPilot processes")
    smoke_parser.add_argument("--mode", choices=("headless", "graphical"), default="headless")
    smoke_parser.add_argument("--timeout", type=float, default=60.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = load_config(args.config)
    if args.command == "diagnose":
        report = diagnose(config)
        exit_code = 0 if report["ready"] else 1
    else:
        if args.timeout <= 0:
            raise SystemExit("--timeout must be positive")
        report = smoke(config, args.timeout, args.mode == "headless")
        exit_code = 0 if report["status"] == Status.PASS else (2 if report["status"] == Status.SKIP else 1)
    rendered = serialize_report(report)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    sys.stdout.write(rendered)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
