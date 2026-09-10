"""Opt-in orchestration for the real v0.14.3 simulation flight milestone."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys
import tempfile
from typing import Any, Mapping, Sequence

from echorescue.sim_integration import (
    Check,
    ProcessSupervisor,
    REPOSITORY_ROOT,
    StartupTimeout,
    Status,
    UnexpectedProcessExit,
    _expand_command,
    _gazebo_ready,
    _retain_failure_logs,
    _tcp_ready,
    check,
    load_config,
    serialize_report,
)
from echorescue.telemetry_integration import (
    _cleanup_port_checks,
    _ros_graph_ready,
    _wait_for_exit,
    telemetry_diagnose,
)


DEFAULT_FLIGHT_CONFIG = REPOSITORY_ROOT / "config" / "flight-mission-v0.14.3.json"


def load_flight_config(path: Path = DEFAULT_FLIGHT_CONFIG) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def flight_diagnose(config: Mapping[str, Any], base_config: Mapping[str, Any]) -> dict[str, Any]:
    report = telemetry_diagnose(config, base_config)
    report["schema_version"] = "echorescue-flight-mission-diagnostic/1.0"
    return report


def _mission_checks(mission: Mapping[str, Any], observer: Mapping[str, Any]) -> list[Check]:
    commands = mission.get("commands", [])
    expected = ["extended_state_stream", "guided", "arm", "takeoff", "land"]
    kinds = [item.get("kind") for item in commands]
    acknowledgements = (
        kinds == expected
        and all(item.get("ack_result") == 0 for item in commands)
        and all(
            isinstance(item.get("ack_monotonic_ns"), int)
            and item["ack_monotonic_ns"] > item["issued_monotonic_ns"]
            for item in commands
        )
    )
    transitions = (
        kinds == expected
        and all(item.get("telemetry_transition") for item in commands)
        and all(
            isinstance(item.get("telemetry_transition_monotonic_ns"), int)
            and isinstance(item.get("ack_monotonic_ns"), int)
            and item["telemetry_transition_monotonic_ns"] > item["ack_monotonic_ns"]
            for item in commands
        )
    )
    final_state = (
        mission.get("status") == "PASS"
        and mission.get("final_landed") is True
        and mission.get("final_armed") is False
    )
    independent = observer.get("status") == "PASS" and not observer.get("session_changed", True)
    return [
        check(
            "mission.command_acknowledgements",
            Status.PASS if acknowledgements else Status.FAIL,
            f"command order={kinds}; every command must have MAV_RESULT_ACCEPTED",
        ),
        check(
            "mission.telemetry_transitions",
            Status.PASS if transitions else Status.FAIL,
            "each accepted ACK is followed by an independently received telemetry transition",
        ),
        check(
            "mission.final_state",
            Status.PASS if final_state else Status.FAIL,
            f"status={mission.get('status')}; landed={mission.get('final_landed')}; armed={mission.get('final_armed')}",
        ),
        check(
            "mission.independent_ros_observer",
            Status.PASS if independent else Status.FAIL,
            f"observer status={observer.get('status')}; session_changed={observer.get('session_changed')}",
        ),
    ]


def flight_smoke(
    config: Mapping[str, Any],
    base_config: Mapping[str, Any],
    timeout_s: float,
    output_path: Path | None,
) -> dict[str, Any]:
    preflight = flight_diagnose(config, base_config)
    if not preflight["ready"]:
        failed = [item["name"] for item in preflight["checks"] if item["status"] == Status.FAIL]
        return {
            "schema_version": "echorescue-flight-mission-smoke/1.0",
            "milestone": "v0.14.3",
            "status": Status.SKIP,
            "detail": f"preflight unavailable: {', '.join(failed)}",
            "checks": preflight["checks"],
            "cleanup": [],
            "logs": [],
            "mission": None,
            "observer": None,
        }

    integration = base_config["integration"]
    runner = config["runner"]
    mission = config["mission"]
    values = {
        "world": integration["world"],
        "model": integration["model"],
        "ardupilot_home": os.environ["ARDUPILOT_HOME"],
        "mavlink_endpoint": runner["endpoint"],
        "stream_rate_hz": str(runner["stream_rate_hz"]),
        "freshness_threshold_s": str(runner["freshness_threshold_s"]),
        "disconnect_threshold_s": str(runner["disconnect_threshold_s"]),
        "reconnect_interval_s": str(runner["reconnect_interval_s"]),
        **{name: str(value) for name, value in mission.items()},
    }
    gazebo_command = _expand_command(integration["gazebo_command"], values)
    gazebo_command.extend(integration["gazebo_headless_arguments"])
    sitl_command = _expand_command(integration["sitl_command"], values)
    supervisor = ProcessSupervisor()
    checks: list[Check] = []
    mission_report: dict[str, Any] | None = None
    observer_report: dict[str, Any] | None = None
    interrupted = False
    with tempfile.TemporaryDirectory(prefix="echorescue-v0143-") as temp_dir:
        temporary = Path(temp_dir)
        mission_output = temporary / "mission.json"
        observer_output = temporary / "observer.json"
        command_values = {
            **values,
            "mission_output": str(mission_output),
            "observer_output": str(observer_output),
            "observer_timeout_s": str(timeout_s),
        }
        mission_command = _expand_command(runner["command"], command_values)
        observer_command = _expand_command(config["observer"]["command"], command_values)
        log_sources = {
            "gazebo": temporary / "gazebo.log",
            "ardupilot_sitl": temporary / "sitl.log",
            "mission": temporary / "mission.log",
            "observer": temporary / "observer.log",
        }
        try:
            with (
                log_sources["gazebo"].open("w", encoding="utf-8") as gazebo_log,
                log_sources["ardupilot_sitl"].open("w", encoding="utf-8") as sitl_log,
                log_sources["mission"].open("w", encoding="utf-8") as mission_log,
                log_sources["observer"].open("w", encoding="utf-8") as observer_log,
            ):
                gazebo_process = supervisor.start("gazebo", gazebo_command, REPOSITORY_ROOT, gazebo_log)
                supervisor.wait_until(
                    "gazebo",
                    gazebo_process,
                    lambda: _gazebo_ready(base_config["health"]["gazebo_topic_contains"]),
                    timeout_s,
                )
                checks.append(check("health.gazebo", Status.PASS, "real Gazebo Iris world is live"))
                sitl_process = supervisor.start(
                    "ardupilot_sitl", sitl_command, Path(values["ardupilot_home"]), sitl_log
                )
                endpoint = base_config["network"]["sitl_mavlink"]
                supervisor.wait_until(
                    "ardupilot_sitl",
                    sitl_process,
                    lambda: _tcp_ready(endpoint["host"], int(endpoint["port"])),
                    timeout_s,
                )
                checks.append(check("health.ardupilot_sitl", Status.PASS, "real SITL endpoint is accepting connections"))
                observer_process = supervisor.start(
                    "flight_observer", observer_command, REPOSITORY_ROOT, observer_log
                )
                mission_process = supervisor.start(
                    "flight_mission", mission_command, REPOSITORY_ROOT, mission_log
                )
                supervisor.wait_until(
                    "flight_mission",
                    mission_process,
                    lambda: _ros_graph_ready(runner["node_name"], config["topics"]),
                    timeout_s,
                )
                _wait_for_exit("flight_mission", mission_process, timeout_s)
                _wait_for_exit("flight_observer", observer_process, timeout_s)
                if not mission_output.is_file() or not observer_output.is_file():
                    raise RuntimeError("mission or independent observer exited without a report")
                mission_report = json.loads(mission_output.read_text(encoding="utf-8"))
                observer_report = json.loads(observer_output.read_text(encoding="utf-8"))
                checks.extend(_mission_checks(mission_report, observer_report))
        except KeyboardInterrupt:
            interrupted = True
            checks.append(check("smoke.interruption", Status.FAIL, "interrupted; cleanup initiated"))
        except (StartupTimeout, UnexpectedProcessExit, RuntimeError, OSError, ValueError) as error:
            checks.append(check("smoke.execution", Status.FAIL, str(error)))
        finally:
            if mission_report is None and mission_output.is_file():
                try:
                    mission_report = json.loads(mission_output.read_text(encoding="utf-8"))
                except (OSError, ValueError):
                    pass
            if observer_report is None and observer_output.is_file():
                try:
                    observer_report = json.loads(observer_output.read_text(encoding="utf-8"))
                except (OSError, ValueError):
                    pass
            cleanup = supervisor.cleanup()
            cleanup.extend(_cleanup_port_checks(base_config))
            cleanup.sort(key=lambda item: item.name)
        passed = bool(checks) and not interrupted and not any(
            item.status is Status.FAIL for item in checks + cleanup
        )
        retained_logs = (
            _retain_failure_logs(
                log_sources,
                output_path,
                int(config["logging"]["failure_log_max_bytes"]),
            )
            if not passed
            else []
        )
    return {
        "schema_version": "echorescue-flight-mission-smoke/1.0",
        "milestone": "v0.14.3",
        "status": Status.PASS if passed else Status.FAIL,
        "detail": "real Gazebo-ArduPilot-ROS arm/takeoff/hover/land mission verified"
        if passed
        else "real v0.14.3 flight mission was not verified",
        "checks": [asdict(item) for item in sorted(checks, key=lambda item: item.name)],
        "cleanup": [asdict(item) for item in cleanup],
        "logs": retained_logs,
        "mission": mission_report,
        "observer": observer_report,
    }


def _base_config_path(config: Mapping[str, Any]) -> Path:
    return REPOSITORY_ROOT / str(config["base_stack_config"])


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Diagnose or run EchoRescue v0.14.3 flight integration")
    parser.add_argument("--config", type=Path, default=DEFAULT_FLIGHT_CONFIG)
    parser.add_argument("--output", type=Path)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("diagnose")
    smoke_parser = subparsers.add_parser("smoke")
    smoke_parser.add_argument("--timeout", type=float, default=150.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = load_flight_config(args.config)
    base_config = load_config(_base_config_path(config))
    if args.command == "diagnose":
        report = flight_diagnose(config, base_config)
        exit_code = 0 if report["ready"] else 1
    else:
        if args.timeout <= 0:
            raise SystemExit("--timeout must be positive")
        report = flight_smoke(config, base_config, args.timeout, args.output)
        exit_code = 0 if report["status"] == Status.PASS else (2 if report["status"] == Status.SKIP else 1)
    rendered = serialize_report(report)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    sys.stdout.write(rendered)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
