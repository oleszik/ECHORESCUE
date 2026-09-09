"""Opt-in orchestration for the real v0.14.1 receive-only telemetry path."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Any, Mapping, Sequence

from echorescue.sim_integration import (
    Check,
    DEFAULT_CONFIG,
    ProcessSupervisor,
    REPOSITORY_ROOT,
    StartupTimeout,
    Status,
    UnexpectedProcessExit,
    _expand_command,
    _gazebo_ready,
    _port_check,
    _retain_failure_logs,
    _run_version,
    _tcp_ready,
    check,
    diagnose,
    load_config,
    serialize_report,
)


DEFAULT_TELEMETRY_CONFIG = REPOSITORY_ROOT / "config" / "mavlink-telemetry-v0.14.1.json"


def _diagnostic_schema(config: Mapping[str, Any]) -> str:
    return "echorescue-coordinate-frame-diagnostic/1.0" if config.get("require_converted_state") else "echorescue-mavlink-telemetry-diagnostic/1.0"


def _smoke_schema(config: Mapping[str, Any]) -> str:
    return "echorescue-coordinate-frame-smoke/1.1" if config.get("require_converted_state") else "echorescue-mavlink-telemetry-smoke/1.0"


def _timestamp_domains_progress(report: Mapping[str, Any]) -> bool:
    """Require both explicitly named clocks to progress in their own units."""
    source_first = report.get("converted_source_time_boot_ms_first")
    source_last = report.get("converted_source_time_boot_ms_last")
    receipt_first = report.get("converted_receipt_monotonic_ns_first")
    receipt_last = report.get("converted_receipt_monotonic_ns_last")
    return (
        report.get("source_clock") == "mavlink_system_boot_ms"
        and report.get("receipt_clock") == "host_monotonic_ns"
        and isinstance(source_first, int)
        and isinstance(source_last, int)
        and isinstance(receipt_first, int)
        and isinstance(receipt_last, int)
        and source_last > source_first
        and receipt_last > receipt_first
    )


def load_telemetry_config(path: Path = DEFAULT_TELEMETRY_CONFIG) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def telemetry_diagnose(
    config: Mapping[str, Any],
    base_config: Mapping[str, Any],
) -> dict[str, Any]:
    base = diagnose(base_config)
    checks = [
        Check(
            name=item["name"],
            status=Status(item["status"]),
            version=item["version"],
            detail=item["detail"],
        )
        for item in base["checks"]
    ]
    for package_name in ("echorescue_interfaces", "echorescue_ros"):
        code, output = _run_version(["ros2", "pkg", "prefix", package_name])
        checks.append(check(
            f"ros_package.{package_name}",
            Status.PASS if code == 0 and bool(output) else Status.FAIL,
            output.splitlines()[0] if code == 0 and output else f"build and source ROS package: {package_name}",
        ))
    interface_name = str(config.get("diagnostic_interface", "echorescue_interfaces/msg/MavlinkTelemetryStatus"))
    expected_field = "output_frame" if config.get("require_converted_state") else "telemetry_age_s"
    message_code, message_output = _run_version(["ros2", "interface", "show", interface_name])
    checks.append(check(
        "ros_interface.mavlink_telemetry",
        Status.PASS if message_code == 0 and expected_field in message_output else Status.FAIL,
        f"typed {config['milestone']} telemetry interfaces are available"
        if message_code == 0 and expected_field in message_output
        else f"build and source the {config['milestone']} ROS interfaces",
    ))
    checks.sort(key=lambda item: item.name)
    return {
        "schema_version": _diagnostic_schema(config),
        "milestone": config["milestone"],
        "ready": base["ready"] and not any(item.status is Status.FAIL for item in checks),
        "checks": [asdict(item) for item in checks],
    }


def _ros_graph_ready(node_name: str, topics: Mapping[str, str]) -> bool:
    node_code, nodes = _run_version(["ros2", "node", "list"])
    topic_code, topic_output = _run_version(["ros2", "topic", "list", "-t"])
    if node_code != 0 or topic_code != 0 or node_name not in nodes.splitlines():
        return False
    lines = set(topic_output.splitlines())
    return all(f"{name} [{message_type}]" in lines for name, message_type in topics.items())


def _wait_for_exit(name: str, process: Any, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        return_code = process.poll()
        if return_code is not None:
            if return_code != 0:
                raise UnexpectedProcessExit(f"{name} exited unexpectedly with code {return_code}")
            return
        time.sleep(0.1)
    raise StartupTimeout(f"{name} timed out after {timeout_s:.1f}s")


def _cleanup_port_checks(base_config: Mapping[str, Any], timeout_s: float = 5.0) -> list[Check]:
    deadline = time.monotonic() + timeout_s
    results: list[Check] = []
    while True:
        results = []
        for port_name, endpoint in sorted(base_config["network"].items()):
            result = _port_check(
                f"cleanup.port.{port_name}",
                endpoint["host"],
                int(endpoint["port"]),
                endpoint["protocol"],
            )
            results.append(result)
        if all(item.status is Status.PASS for item in results) or time.monotonic() >= deadline:
            return results
        time.sleep(0.1)


def _observer_checks(report: Mapping[str, Any], freshness_threshold_s: float, require_converted_state: bool = False) -> list[Check]:
    count = int(report.get("local_position_samples", 0))
    first = report.get("local_source_time_boot_ms_first")
    last = report.get("local_source_time_boot_ms_last")
    advancing = count >= 2 and isinstance(first, int) and isinstance(last, int) and last > first
    age = report.get("telemetry_age_s")
    fresh = isinstance(age, (int, float)) and 0.0 <= float(age) <= freshness_threshold_s
    graph = not report.get("missing_nodes") and not report.get("missing_topics")
    checks = [
        check("health.ros_graph", Status.PASS if graph else Status.FAIL, f"missing nodes={report.get('missing_nodes', [])}; missing topics={report.get('missing_topics', [])}"),
        check("health.vehicle_state", Status.PASS if report.get("vehicle_state_received") else Status.FAIL, "heartbeat-derived typed vehicle state received" if report.get("vehicle_state_received") else "no heartbeat-derived vehicle state received"),
        check("health.local_position", Status.PASS if advancing else Status.FAIL, f"received {count} local NED samples; source time {first} -> {last}"),
        check("health.global_position", Status.PASS if report.get("global_position_received") else Status.FAIL, "typed global position received" if report.get("global_position_received") else "no global position received"),
        check("health.telemetry_age", Status.PASS if fresh else Status.FAIL, f"age={age}; threshold={freshness_threshold_s}"),
    ]
    if require_converted_state:
        converted_count = int(report.get("converted_state_samples", 0))
        converted_first = report.get("converted_source_time_boot_ms_first")
        converted_last = report.get("converted_source_time_boot_ms_last")
        converted_advancing = (
            converted_count >= 2 and isinstance(converted_first, int)
            and isinstance(converted_last, int) and converted_last > converted_first
        )
        timestamps_distinct = _timestamp_domains_progress(report)
        checks.extend([
            check("health.converted_enu_state", Status.PASS if converted_advancing else Status.FAIL, f"received {converted_count} converted ENU samples; source time {converted_first} -> {converted_last}"),
            check("health.coordinate_conversion", Status.PASS if report.get("coordinate_conversion_consistent") else Status.FAIL, f"independently matched {report.get('coordinate_conversion_matches', 0)} NED/ENU position and velocity samples"),
            check("health.timestamp_separation", Status.PASS if timestamps_distinct else Status.FAIL, "MAVLink boot time and host monotonic receipt time each advance in their declared clock domain"),
        ])
    return checks


def telemetry_smoke(
    config: Mapping[str, Any],
    base_config: Mapping[str, Any],
    timeout_s: float,
    output_path: Path | None,
) -> dict[str, Any]:
    preflight = telemetry_diagnose(config, base_config)
    if not preflight["ready"]:
        failed = [item["name"] for item in preflight["checks"] if item["status"] == Status.FAIL]
        return {
            "schema_version": _smoke_schema(config),
            "milestone": config["milestone"],
            "status": Status.SKIP,
            "detail": f"preflight unavailable: {', '.join(failed)}",
            "checks": preflight["checks"],
            "cleanup": [],
            "logs": [],
            "observer": None,
        }

    integration = base_config["integration"]
    bridge = config["bridge"]
    values = {
        "world": integration["world"],
        "model": integration["model"],
        "ardupilot_home": os.environ["ARDUPILOT_HOME"],
        "mavlink_endpoint": bridge["endpoint"],
        "stream_rate_hz": str(bridge["stream_rate_hz"]),
        "freshness_threshold_s": str(bridge["freshness_threshold_s"]),
        "disconnect_threshold_s": str(bridge["disconnect_threshold_s"]),
        "reconnect_interval_s": str(bridge["reconnect_interval_s"]),
    }
    gazebo_command = _expand_command(integration["gazebo_command"], values)
    gazebo_command.extend(integration["gazebo_headless_arguments"])
    sitl_command = _expand_command(integration["sitl_command"], values)
    bridge_command = _expand_command(bridge["command"], values)
    supervisor = ProcessSupervisor()
    checks: list[Check] = []
    observer_report: dict[str, Any] | None = None
    interrupted = False
    with tempfile.TemporaryDirectory(prefix="echorescue-v0141-") as temp_dir:
        temporary = Path(temp_dir)
        observer_output = temporary / "observer.json"
        observer_values = {
            "observer_output": str(observer_output),
            "observer_timeout_s": str(timeout_s),
            "minimum_position_samples": str(config["observer"]["minimum_position_samples"]),
        }
        observer_command = _expand_command(config["observer"]["command"], observer_values)
        log_sources = {
            "gazebo": temporary / "gazebo.log",
            "ardupilot_sitl": temporary / "sitl.log",
            "telemetry_bridge": temporary / "bridge.log",
            "telemetry_observer": temporary / "observer.log",
        }
        try:
            with (
                log_sources["gazebo"].open("w", encoding="utf-8") as gazebo_log,
                log_sources["ardupilot_sitl"].open("w", encoding="utf-8") as sitl_log,
                log_sources["telemetry_bridge"].open("w", encoding="utf-8") as bridge_log,
                log_sources["telemetry_observer"].open("w", encoding="utf-8") as observer_log,
            ):
                gazebo_process = supervisor.start("gazebo", gazebo_command, REPOSITORY_ROOT, gazebo_log)
                supervisor.wait_until(
                    "gazebo", gazebo_process,
                    lambda: _gazebo_ready(base_config["health"]["gazebo_topic_contains"]),
                    timeout_s,
                )
                checks.append(check("health.gazebo", Status.PASS, "real Gazebo Iris world is live"))
                sitl_process = supervisor.start("ardupilot_sitl", sitl_command, Path(values["ardupilot_home"]), sitl_log)
                endpoint = base_config["network"]["sitl_mavlink"]
                supervisor.wait_until(
                    "ardupilot_sitl", sitl_process,
                    lambda: _tcp_ready(endpoint["host"], int(endpoint["port"])),
                    timeout_s,
                )
                checks.append(check("health.ardupilot_sitl", Status.PASS, "real SITL MAVLink endpoint is accepting connections"))
                bridge_process = supervisor.start("telemetry_bridge", bridge_command, REPOSITORY_ROOT, bridge_log)
                supervisor.wait_until(
                    "telemetry_bridge", bridge_process,
                    lambda: _ros_graph_ready(bridge["node_name"], config["topics"]),
                    timeout_s,
                )
                observer_process = supervisor.start("telemetry_observer", observer_command, REPOSITORY_ROOT, observer_log)
                _wait_for_exit("telemetry_observer", observer_process, timeout_s + 5.0)
                if not observer_output.is_file():
                    raise RuntimeError("telemetry observer exited without a report")
                observer_report = json.loads(observer_output.read_text(encoding="utf-8"))
                checks.extend(_observer_checks(
                    observer_report,
                    float(bridge["freshness_threshold_s"]),
                    bool(config.get("require_converted_state", False)),
                ))
        except KeyboardInterrupt:
            interrupted = True
            checks.append(check("smoke.interruption", Status.FAIL, "interrupted; cleanup initiated"))
        except Exception as error:
            checks.append(check("smoke.execution", Status.FAIL, str(error)))
        finally:
            cleanup = supervisor.cleanup()
            cleanup.extend(_cleanup_port_checks(base_config))
            cleanup.sort(key=lambda item: item.name)
        passed = bool(checks) and not interrupted and not any(
            item.status is Status.FAIL for item in checks + cleanup
        )
        retained_logs = _retain_failure_logs(
            log_sources,
            output_path,
            int(config["logging"]["failure_log_max_bytes"]),
        ) if not passed else []
    return {
        "schema_version": _smoke_schema(config),
        "milestone": config["milestone"],
        "status": Status.PASS if passed else Status.FAIL,
        "detail": "real receive-only Gazebo-SITL-MAVLink-ROS telemetry path verified"
        if passed else "real receive-only telemetry path was not verified",
        "checks": [asdict(item) for item in sorted(checks, key=lambda item: item.name)],
        "cleanup": [asdict(item) for item in cleanup],
        "logs": retained_logs,
        "observer": observer_report,
    }


def _base_config_path(config: Mapping[str, Any]) -> Path:
    return REPOSITORY_ROOT / str(config.get("base_stack_config", DEFAULT_CONFIG))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Diagnose or test EchoRescue v0.14.1 MAVLink telemetry")
    parser.add_argument("--config", type=Path, default=DEFAULT_TELEMETRY_CONFIG)
    parser.add_argument("--output", type=Path)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("diagnose")
    smoke_parser = subparsers.add_parser("smoke")
    smoke_parser.add_argument("--timeout", type=float, default=90.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = load_telemetry_config(args.config)
    base_config = load_config(_base_config_path(config))
    if args.command == "diagnose":
        report = telemetry_diagnose(config, base_config)
        exit_code = 0 if report["ready"] else 1
    else:
        if args.timeout <= 0:
            raise SystemExit("--timeout must be positive")
        report = telemetry_smoke(config, base_config, args.timeout, args.output)
        exit_code = 0 if report["status"] == Status.PASS else (2 if report["status"] == Status.SKIP else 1)
    rendered = serialize_report(report)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    sys.stdout.write(rendered)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
