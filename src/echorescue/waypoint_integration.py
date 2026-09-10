"""Attached- or owned-stack orchestration for the real v0.14.4 mission."""

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
    Check, ProcessSupervisor, REPOSITORY_ROOT, StartupTimeout, Status,
    UnexpectedProcessExit, _expand_command, _gazebo_ready, _retain_failure_logs,
    _tcp_ready, check, load_config, serialize_report,
)
from echorescue.flight_mission import validate_simulation_endpoint
from echorescue.telemetry_integration import (
    _cleanup_port_checks, _ros_graph_ready, _wait_for_exit, telemetry_diagnose,
)
from echorescue.waypoint_mission import RelativeTarget, WaypointMissionConfig


DEFAULT_WAYPOINT_CONFIG = REPOSITORY_ROOT / "config" / "waypoint-mission-v0.14.4.json"


def load_waypoint_config(path: Path = DEFAULT_WAYPOINT_CONFIG) -> dict[str, Any]:
    config: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
    validate_waypoint_config(config)
    return config


def validate_waypoint_config(config: Mapping[str, Any]) -> None:
    if config.get("milestone") != "v0.14.4" or config.get("schema_version") != "echorescue-waypoint-mission-stack/1.0":
        raise ValueError("waypoint config must declare the v0.14.4 stack schema")
    runner = config["runner"]
    mission = config["mission"]
    validate_simulation_endpoint(str(runner["endpoint"]))
    targets = (
        RelativeTarget("waypoint-a", float(mission["waypoint_a_east_m"]), float(mission["waypoint_a_north_m"]), float(mission["waypoint_a_up_m"])),
        RelativeTarget("waypoint-b", float(mission["waypoint_b_east_m"]), float(mission["waypoint_b_north_m"]), float(mission["waypoint_b_up_m"])),
        RelativeTarget("return-launch", float(mission["return_east_m"]), float(mission["return_north_m"]), float(mission["return_up_m"])),
    )
    WaypointMissionConfig(
        targets=targets,
        takeoff_altitude_m=float(mission["takeoff_altitude_m"]),
        horizontal_tolerance_m=float(mission["horizontal_tolerance_m"]),
        vertical_tolerance_m=float(mission["vertical_tolerance_m"]),
        settling_time_s=float(mission["settling_time_s"]),
        target_timeout_s=float(mission["target_timeout_s"]),
        progress_timeout_s=float(mission["progress_timeout_s"]),
        progress_epsilon_m=float(mission["progress_epsilon_m"]),
        mission_timeout_s=float(mission["mission_timeout_s"]),
        geofence_horizontal_radius_m=float(mission["geofence_horizontal_radius_m"]),
        geofence_min_altitude_m=float(mission["geofence_min_altitude_m"]),
        geofence_max_altitude_m=float(mission["geofence_max_altitude_m"]),
        preflight_hold_s=float(mission["preflight_hold_s"]),
        ready_timeout_s=float(mission["ready_timeout_s"]),
        transition_timeout_s=float(mission["transition_timeout_s"]),
        takeoff_timeout_s=float(mission["takeoff_timeout_s"]),
        landing_timeout_s=float(mission["landing_timeout_s"]),
        cleanup_timeout_s=float(mission["cleanup_timeout_s"]),
    )
    required_topics = {
        "/echorescue/vehicle/state_3d": "echorescue_interfaces/msg/EchoRescueVehicleState3D",
        "/echorescue/mission/active_target": "echorescue_interfaces/msg/WaypointTarget",
        "/echorescue/mission/event": "echorescue_interfaces/msg/WaypointMissionEvent",
    }
    if any(config["topics"].get(name) != message_type for name, message_type in required_topics.items()):
        raise ValueError("waypoint config is missing a required typed ROS topic")


def waypoint_diagnose(config: Mapping[str, Any], base_config: Mapping[str, Any]) -> dict[str, Any]:
    report = telemetry_diagnose(config, base_config)
    report["schema_version"] = "echorescue-waypoint-mission-diagnostic/1.0"
    return report


def _attached_preflight(config: Mapping[str, Any], base_config: Mapping[str, Any]) -> dict[str, Any]:
    report = waypoint_diagnose(config, base_config)
    replaced: list[dict[str, Any]] = []
    for item in report["checks"]:
        if item["name"] == "port.gazebo_json":
            ready = _gazebo_ready(base_config["health"]["gazebo_topic_contains"])
            item = asdict(check(item["name"], Status.PASS if ready else Status.FAIL, "attached Gazebo stack is live" if ready else "attached Gazebo stack not detected"))
        elif item["name"] == "port.sitl_mavlink":
            endpoint = base_config["network"]["sitl_mavlink"]
            ready = _tcp_ready(endpoint["host"], int(endpoint["port"]))
            item = asdict(check(item["name"], Status.PASS if ready else Status.FAIL, "attached SITL endpoint is live" if ready else "attached SITL endpoint not detected"))
        replaced.append(item)
    report["checks"] = replaced
    report["ready"] = not any(item["status"] == Status.FAIL for item in replaced)
    return report


def _mission_checks(mission: Mapping[str, Any], observer: Mapping[str, Any], config: Mapping[str, Any]) -> list[Check]:
    commands = mission.get("commands", [])
    kinds = [item.get("kind") for item in commands]
    expected_commands = ["extended_state_stream", "guided", "arm", "takeoff", "land"]
    ack_ok = kinds == expected_commands and all(item.get("ack_result") == 0 for item in commands)
    transition_ok = ack_ok and all(
        isinstance(item.get("telemetry_transition_monotonic_ns"), int)
        and isinstance(item.get("ack_monotonic_ns"), int)
        and item["telemetry_transition_monotonic_ns"] > item["ack_monotonic_ns"]
        for item in commands
    )
    targets = mission.get("targets", [])
    expected_targets = ["waypoint-a", "waypoint-b", "return-launch"]
    target_ids = [item.get("target_id") for item in targets]
    post_command = target_ids == expected_targets and all(
        item.get("outcome") == "settled"
        and isinstance(item.get("transmit_monotonic_ns"), int)
        and isinstance(item.get("first_post_command_monotonic_ns"), int)
        and item["first_post_command_monotonic_ns"] > item["transmit_monotonic_ns"]
        and item["first_post_command_source_time_boot_ms"] > item["transmit_source_time_boot_ms"]
        for item in targets
    )
    horizontal_tolerance = float(config["mission"]["horizontal_tolerance_m"])
    vertical_tolerance = float(config["mission"]["vertical_tolerance_m"])
    arrival = post_command and all(
        item.get("horizontal_error_m", float("inf")) <= horizontal_tolerance
        and item.get("vertical_error_m", float("inf")) <= vertical_tolerance
        and item.get("settling_duration_s", 0.0) >= float(config["mission"]["settling_time_s"])
        for item in targets
    )
    observer_ok = (
        observer.get("status") == "PASS"
        and observer.get("target_ids") == expected_targets
        and observer.get("settled_event_ids") == expected_targets
        and not observer.get("session_changed", True)
        and observer.get("session_id") == mission.get("session_id")
    )
    final_ok = (
        mission.get("status") == "PASS" and mission.get("final_landed") is True
        and mission.get("final_armed") is False
        and mission.get("final_distance_from_launch_m", float("inf")) <= 0.75
    )
    return [
        check("mission.command_ack_and_transitions", Status.PASS if transition_ok else Status.FAIL, f"command order={kinds}; ACKs and later telemetry required"),
        check("mission.target_post_command_evidence", Status.PASS if post_command else Status.FAIL, f"target order={target_ids}; send/session/receipt/vehicle-time evidence required"),
        check("mission.target_arrival_and_settling", Status.PASS if arrival else Status.FAIL, f"horizontal <= {horizontal_tolerance} m; vertical <= {vertical_tolerance} m"),
        check("mission.independent_observer_agreement", Status.PASS if observer_ok else Status.FAIL, f"observer targets={observer.get('target_ids')}; events={observer.get('settled_event_ids')}"),
        check("mission.return_land_disarm", Status.PASS if final_ok else Status.FAIL, f"distance={mission.get('final_distance_from_launch_m')}; landed={mission.get('final_landed')}; armed={mission.get('final_armed')}"),
    ]


def waypoint_smoke(
    config: Mapping[str, Any], base_config: Mapping[str, Any], timeout_s: float,
    output_path: Path | None, *, stack_mode: str,
) -> dict[str, Any]:
    preflight = _attached_preflight(config, base_config) if stack_mode == "attached" else waypoint_diagnose(config, base_config)
    if not preflight["ready"]:
        failed = [item["name"] for item in preflight["checks"] if item["status"] == Status.FAIL]
        return {
            "schema_version": "echorescue-waypoint-mission-smoke/1.0", "milestone": "v0.14.4",
            "stack_mode": stack_mode, "status": Status.SKIP,
            "detail": f"preflight unavailable: {', '.join(failed)}", "checks": preflight["checks"],
            "cleanup": [], "process_exit_statuses": {}, "logs": [], "mission": None, "observer": None,
        }
    integration, runner, mission_config = base_config["integration"], config["runner"], config["mission"]
    values = {
        "world": integration["world"], "model": integration["model"],
        "ardupilot_home": os.environ["ARDUPILOT_HOME"],
        "mavlink_endpoint": runner["endpoint"],
        "stream_rate_hz": str(runner["stream_rate_hz"]),
        "freshness_threshold_s": str(runner["freshness_threshold_s"]),
        "disconnect_threshold_s": str(runner["disconnect_threshold_s"]),
        "reconnect_interval_s": str(runner["reconnect_interval_s"]),
        **{name: str(value) for name, value in mission_config.items()},
    }
    supervisor = ProcessSupervisor()
    checks: list[Check] = []
    mission_report: dict[str, Any] | None = None
    observer_report: dict[str, Any] | None = None
    interrupted = False
    process_exit_statuses: dict[str, int | None] = {}
    with tempfile.TemporaryDirectory(prefix="echorescue-v0144-") as raw_temp:
        temporary = Path(raw_temp)
        mission_output, observer_output = temporary / "mission.json", temporary / "observer.json"
        command_values = {
            **values, "mission_output": str(mission_output), "observer_output": str(observer_output),
            "observer_timeout_s": str(timeout_s),
        }
        mission_command = _expand_command(runner["command"], command_values)
        observer_command = _expand_command(config["observer"]["command"], command_values)
        log_sources = {
            "gazebo": temporary / "gazebo.log", "ardupilot_sitl": temporary / "sitl.log",
            "mission": temporary / "mission.log", "observer": temporary / "observer.log",
        }
        try:
            with (
                log_sources["gazebo"].open("w", encoding="utf-8") as gazebo_log,
                log_sources["ardupilot_sitl"].open("w", encoding="utf-8") as sitl_log,
                log_sources["mission"].open("w", encoding="utf-8") as mission_log,
                log_sources["observer"].open("w", encoding="utf-8") as observer_log,
            ):
                if stack_mode == "owned":
                    gazebo_command = _expand_command(integration["gazebo_command"], values)
                    gazebo_command.extend(integration["gazebo_headless_arguments"])
                    gazebo_process = supervisor.start("gazebo", gazebo_command, REPOSITORY_ROOT, gazebo_log)
                    supervisor.wait_until("gazebo", gazebo_process, lambda: _gazebo_ready(base_config["health"]["gazebo_topic_contains"]), timeout_s)
                    checks.append(check("health.gazebo", Status.PASS, "owned real Gazebo Iris world is live"))
                    sitl_command = _expand_command(integration["sitl_command"], values)
                    sitl_process = supervisor.start("ardupilot_sitl", sitl_command, Path(values["ardupilot_home"]), sitl_log)
                    endpoint = base_config["network"]["sitl_mavlink"]
                    supervisor.wait_until("ardupilot_sitl", sitl_process, lambda: _tcp_ready(endpoint["host"], int(endpoint["port"])), timeout_s)
                    checks.append(check("health.ardupilot_sitl", Status.PASS, "owned real SITL endpoint is accepting connections"))
                else:
                    checks.extend([
                        check("health.gazebo", Status.PASS, "pre-existing Gazebo stack preserved"),
                        check("health.ardupilot_sitl", Status.PASS, "pre-existing SITL stack preserved"),
                    ])
                observer_process = supervisor.start("waypoint_observer", observer_command, REPOSITORY_ROOT, observer_log)
                supervisor.wait_until(
                    "waypoint_observer", observer_process,
                    lambda: _ros_graph_ready(runner["observer_node_name"], {}), timeout_s,
                )
                mission_process = supervisor.start("waypoint_mission", mission_command, REPOSITORY_ROOT, mission_log)
                supervisor.wait_until("waypoint_mission", mission_process, lambda: _ros_graph_ready(runner["node_name"], config["topics"]), timeout_s)
                _wait_for_exit("waypoint_mission", mission_process, timeout_s)
                _wait_for_exit("waypoint_observer", observer_process, timeout_s)
                if not mission_output.is_file() or not observer_output.is_file():
                    raise RuntimeError("mission or independent observer exited without a report")
                mission_report = json.loads(mission_output.read_text(encoding="utf-8"))
                observer_report = json.loads(observer_output.read_text(encoding="utf-8"))
                checks.extend(_mission_checks(mission_report, observer_report, config))
        except KeyboardInterrupt:
            interrupted = True
            checks.append(check("smoke.interruption", Status.FAIL, "interrupted; bounded cleanup initiated"))
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
            process_exit_statuses = {name: process.poll() for name, process in supervisor.processes}
            if stack_mode == "owned":
                cleanup.extend(_cleanup_port_checks(base_config))
            else:
                cleanup.append(check("cleanup.external_stack_preserved", Status.PASS, "pre-existing Gazebo and SITL were not owned or terminated"))
            cleanup.sort(key=lambda item: item.name)
        passed = bool(checks) and not interrupted and not any(item.status is Status.FAIL for item in checks + cleanup)
        retained_logs = _retain_failure_logs(log_sources, output_path, int(config["logging"]["failure_log_max_bytes"])) if not passed else []
    return {
        "schema_version": "echorescue-waypoint-mission-smoke/1.0", "milestone": "v0.14.4",
        "stack_mode": stack_mode, "status": Status.PASS if passed else Status.FAIL,
        "detail": "real Gazebo-ArduPilot-ROS continuous waypoint-return-land mission verified" if passed else "real v0.14.4 waypoint mission was not verified",
        "checks": [asdict(item) for item in sorted(checks, key=lambda item: item.name)],
        "cleanup": [asdict(item) for item in cleanup], "process_exit_statuses": process_exit_statuses,
        "logs": retained_logs, "mission": mission_report, "observer": observer_report,
    }


def _base_config_path(config: Mapping[str, Any]) -> Path:
    return REPOSITORY_ROOT / str(config["base_stack_config"])


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Diagnose or run EchoRescue v0.14.4 waypoint integration")
    parser.add_argument("--config", type=Path, default=DEFAULT_WAYPOINT_CONFIG)
    parser.add_argument("--output", type=Path)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("diagnose")
    smoke = subparsers.add_parser("smoke")
    smoke.add_argument("--timeout", type=float, default=210.0)
    smoke.add_argument("--stack-mode", choices=("owned", "attached"), default="owned")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = load_waypoint_config(args.config)
    base = load_config(_base_config_path(config))
    if args.command == "diagnose":
        report = waypoint_diagnose(config, base)
        exit_code = 0 if report["ready"] else 1
    else:
        if args.timeout <= 0:
            raise SystemExit("--timeout must be positive")
        report = waypoint_smoke(config, base, args.timeout, args.output, stack_mode=args.stack_mode)
        exit_code = 0 if report["status"] == Status.PASS else (2 if report["status"] == Status.SKIP else 1)
    rendered = serialize_report(report)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    sys.stdout.write(rendered)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
