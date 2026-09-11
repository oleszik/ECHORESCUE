"""Lifecycle and evidence gates for the v0.14.5 indoor reference mission."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from typing import Any, Mapping, Sequence

from echorescue.flight_mission import validate_simulation_endpoint
from echorescue.indoor_evaluation import load_indoor_config
from echorescue.sim_integration import (
    Check, ProcessSupervisor, REPOSITORY_ROOT, StartupTimeout, Status,
    UnexpectedProcessExit, _gazebo_ready, _retain_failure_logs, _tcp_ready,
    check, load_config, serialize_report,
)
from echorescue.telemetry_integration import _cleanup_port_checks, _ros_graph_ready, _wait_for_exit


DEFAULT_CONFIG = REPOSITORY_ROOT / "config" / "indoor-reference-v0.14.5.json"


def load_configured_stack(path: Path = DEFAULT_CONFIG) -> tuple[dict[str, Any], Any]:
    return load_indoor_config(path)


def indoor_diagnose(path: Path = DEFAULT_CONFIG) -> dict[str, Any]:
    raw, config = load_configured_stack(path)
    checks: list[Check] = []
    validate_simulation_endpoint(str(raw["runner"]["endpoint"]))
    checks.append(check("config.indoor_reference", Status.PASS, f"{len(config.entities)} solids and {len(config.targets)} ordered targets validated"))
    sdf = REPOSITORY_ROOT / config.world_sdf
    checks.append(check("world.sdf", Status.PASS if sdf.is_file() else Status.FAIL, str(sdf)))
    for executable in ("gz", "ros2"):
        found = shutil.which(executable)
        checks.append(check(f"executable.{executable}", Status.PASS if found else Status.FAIL, found or "not found"))
    runtime = subprocess.run(
        [sys.executable, "-c", "import pymavlink, rclpy, yaml"],
        capture_output=True, text=True, check=False, timeout=10,
    )
    checks.append(check(
        "python.runtime", Status.PASS if runtime.returncode == 0 else Status.FAIL,
        sys.executable if runtime.returncode == 0 else (runtime.stderr or runtime.stdout).strip()[:300],
    ))
    for variable in ("ARDUPILOT_HOME", "ARDUPILOT_GAZEBO_HOME", "GZ_SIM_SYSTEM_PLUGIN_PATH", "GZ_SIM_RESOURCE_PATH"):
        value = os.environ.get(variable, "")
        checks.append(check(f"environment.{variable}", Status.PASS if value else Status.FAIL, value or "unset"))
    revisions = raw["dependencies"]
    for name, variable, expected in (
        ("ardupilot", "ARDUPILOT_HOME", revisions["ardupilot_commit"]),
        ("ardupilot_gazebo", "ARDUPILOT_GAZEBO_HOME", revisions["ardupilot_gazebo_commit"]),
    ):
        root = os.environ.get(variable)
        actual = ""
        if root:
            result = subprocess.run(["git", "-C", root, "rev-parse", "HEAD"], capture_output=True, text=True, check=False, timeout=10)
            actual = result.stdout.strip() if result.returncode == 0 else ""
        checks.append(check(f"revision.{name}", Status.PASS if actual == expected else Status.FAIL, f"expected {expected}; actual {actual or 'unavailable'}"))
    return {
        "schema_version": "echorescue-indoor-diagnostic/1.0", "milestone": "v0.14.5",
        "ready": not any(item.status is Status.FAIL for item in checks),
        "checks": [asdict(item) for item in sorted(checks, key=lambda item: item.name)],
    }


def _mission_command(raw: Mapping[str, Any], mission_output: Path) -> list[str]:
    mission = raw["mission"]
    runner = raw["runner"]
    target_json = json.dumps(mission["targets"], separators=(",", ":"))
    parameters = {
        "endpoint": runner["endpoint"], "stream_rate_hz": runner["stream_rate_hz"],
        "freshness_threshold_s": runner["freshness_threshold_s"],
        "disconnect_threshold_s": runner["disconnect_threshold_s"],
        "reconnect_interval_s": runner["reconnect_interval_s"], "targets_json": f"'{target_json}'",
        **{key: value for key, value in mission.items() if key != "targets"},
    }
    command = [sys.executable, "-m", "echorescue_ros.mavlink_waypoint_mission", "--output", str(mission_output), "--ros-args"]
    for name, value in parameters.items():
        command.extend(["-p", f"{name}:={value}"])
    return command


def _checks(
    raw: Mapping[str, Any], mission: Mapping[str, Any], observer: Mapping[str, Any],
    gazebo: Mapping[str, Any], evaluator_exit: int | None,
) -> list[Check]:
    expected = [item["target_id"] for item in raw["mission"]["targets"]]
    commands = mission.get("commands", [])
    transitions = bool(commands) and all(
        item.get("ack_result") == 0
        and isinstance(item.get("ack_monotonic_ns"), int)
        and isinstance(item.get("telemetry_transition_monotonic_ns"), int)
        and item["telemetry_transition_monotonic_ns"] > item["ack_monotonic_ns"]
        for item in commands
    )
    mission_targets = [item.get("target_id") for item in mission.get("targets", [])]
    ordered = mission_targets == expected and observer.get("target_ids") == expected and observer.get("settled_event_ids") == expected
    final = mission.get("status") == "PASS" and mission.get("final_landed") is True and mission.get("final_armed") is False
    clearance = gazebo.get("minimum_obstacle_surface_clearance_m")
    doorway_clearance = gazebo.get("minimum_doorway_boundary_surface_clearance_m")
    clearance_ok = (
        isinstance(clearance, (int, float)) and clearance >= float(raw["safety"]["safety_margin_m"])
        and isinstance(doorway_clearance, (int, float)) and doorway_clearance >= float(raw["safety"]["safety_margin_m"])
    )
    gazebo_ok = evaluator_exit == 0 and gazebo.get("status") == "PASS" and not gazebo.get("prohibited_contact_detected", True) and clearance_ok
    agreement = False
    trajectory = gazebo.get("trajectory", [])
    final_position = mission.get("final_position_enu")
    launch_position = mission.get("launch_enu")
    if len(trajectory) >= 2 and isinstance(final_position, Mapping) and isinstance(launch_position, Mapping):
        start, end = trajectory[0], trajectory[-1]
        telemetry_displacement = (
            float(final_position["east_m"]) - float(launch_position["east_m"]),
            float(final_position["north_m"]) - float(launch_position["north_m"]),
            float(final_position["up_m"]) - float(launch_position["up_m"]),
        )
        gazebo_displacement = (
            float(end["east_m"]) - float(start["east_m"]),
            float(end["north_m"]) - float(start["north_m"]),
            float(end["up_m"]) - float(start["up_m"]),
        )
        error = sum((a - b) ** 2 for a, b in zip(telemetry_displacement, gazebo_displacement)) ** 0.5
        agreement = error <= float(raw["evaluation"]["telemetry_world_agreement_tolerance_m"])
    return [
        check("mission.command_ack_then_telemetry", Status.PASS if transitions else Status.FAIL, "every COMMAND_ACK requires a later same-session telemetry transition"),
        check("mission.ordered_predefined_route", Status.PASS if ordered else Status.FAIL, f"expected={expected}; mission={mission_targets}"),
        check("mission.return_land_disarm", Status.PASS if final else Status.FAIL, f"landed={mission.get('final_landed')}; armed={mission.get('final_armed')}"),
        check("gazebo.collision_free_route", Status.PASS if gazebo_ok else Status.FAIL, f"evaluator exit={evaluator_exit}; contacts={gazebo.get('contact_count')}; obstacle clearance={clearance}; doorway clearance={doorway_clearance}"),
        check("evidence.telemetry_gazebo_agreement", Status.PASS if agreement else Status.FAIL, "final displacement must agree within configured tolerance"),
    ]


def _interrupt(process: subprocess.Popen[str]) -> None:
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)


def _ros_nodes_absent(names: Sequence[str], timeout_s: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout_s
    expected = set(names)
    while time.monotonic() < deadline:
        result = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True, check=False, timeout=5)
        actual = {line.strip() for line in result.stdout.splitlines()}
        if result.returncode == 0 and not expected & actual:
            return True
        time.sleep(0.2)
    return False


def indoor_smoke(
    path: Path, output_path: Path | None, timeout_s: float, graphical: bool,
    stack_mode: str = "owned",
) -> dict[str, Any]:
    raw, config = load_configured_stack(path)
    diagnostic = indoor_diagnose(path)
    if not diagnostic["ready"]:
        return {"schema_version": "echorescue-indoor-smoke/1.0", "milestone": "v0.14.5", "status": Status.SKIP, "detail": "dependency diagnostic failed", "checks": diagnostic["checks"], "cleanup": []}
    base = load_config(REPOSITORY_ROOT / raw["base_stack_config"])
    supervisor = ProcessSupervisor()
    checks: list[Check] = []
    mission_report: dict[str, Any] = {}
    observer_report: dict[str, Any] = {}
    gazebo_report: dict[str, Any] = {}
    exits: dict[str, int | None] = {}
    logs_retained: list[dict[str, Any]] = []
    with tempfile.TemporaryDirectory(prefix="echorescue-v0145-") as raw_temp:
        temp = Path(raw_temp)
        paths = {name: temp / f"{name}.log" for name in ("gazebo", "sitl", "mission", "observer", "evaluator")}
        mission_output, observer_output, evaluator_output, stop_file = (temp / "mission.json", temp / "observer.json", temp / "gazebo.json", temp / "stop")
        handles = [value.open("w", encoding="utf-8") for value in paths.values()]
        mission_process = observer_process = evaluator_process = None
        try:
            if stack_mode == "owned":
                world = str((REPOSITORY_ROOT / config.world_sdf).resolve())
                gazebo_command = ["gz", "sim", "-v4", "-r", world] + ([] if graphical else ["-s"])
                gazebo_process = supervisor.start("gazebo", gazebo_command, REPOSITORY_ROOT, handles[0])
                supervisor.wait_until("gazebo", gazebo_process, lambda: _gazebo_ready(f"/world/{config.world_name}"), 30.0)
                sitl_command = [
                    str(Path(os.environ["ARDUPILOT_HOME"]) / "Tools/autotest/sim_vehicle.py"), "-v", "ArduCopter", "-f", "gazebo-iris",
                    "--model", "JSON", "--no-rebuild", "--no-mavproxy",
                ]
                sitl_process = supervisor.start("ardupilot_sitl", sitl_command, Path(os.environ["ARDUPILOT_HOME"]), handles[1])
                supervisor.wait_until("ardupilot_sitl", sitl_process, lambda: _tcp_ready("127.0.0.1", 5760), 40.0)
            else:
                if not _gazebo_ready(f"/world/{config.world_name}") or not _tcp_ready("127.0.0.1", 5760):
                    raise RuntimeError("attached indoor Gazebo world and SITL must already be live")
            evaluator_process = supervisor.start("gazebo_evaluator", [
                sys.executable, "-m", "echorescue.gazebo_indoor_evaluator", "--config", str(path), "--output", str(evaluator_output), "--stop-file", str(stop_file),
            ], REPOSITORY_ROOT, handles[4])
            expected = ",".join(target.target_id for target in config.targets)
            observer_process = supervisor.start("waypoint_observer", [
                "ros2", "run", "echorescue_ros", "waypoint_mission_observer", "--output", str(observer_output), "--timeout", str(timeout_s),
                "--horizontal-tolerance", str(raw["mission"]["horizontal_tolerance_m"]), "--vertical-tolerance", str(raw["mission"]["vertical_tolerance_m"]),
                "--settling-time", str(raw["mission"]["settling_time_s"]), "--expected-targets", expected,
            ], REPOSITORY_ROOT, handles[3])
            supervisor.wait_until("waypoint_observer", observer_process, lambda: _ros_graph_ready(raw["runner"]["observer_node_name"], {}), 20.0)
            mission_process = supervisor.start("waypoint_mission", _mission_command(raw, mission_output), REPOSITORY_ROOT, handles[2])
            supervisor.wait_until("waypoint_mission", mission_process, lambda: _ros_graph_ready(raw["runner"]["node_name"], {}), 20.0)
            deadline = time.monotonic() + timeout_s
            while mission_process.poll() is None and time.monotonic() < deadline:
                if evaluator_process.poll() is not None:
                    _interrupt(mission_process)
                    break
                time.sleep(0.2)
            _wait_for_exit("waypoint_mission", mission_process, float(raw["mission"]["cleanup_timeout_s"]) + 2.0)
            _wait_for_exit("waypoint_observer", observer_process, 10.0)
            stop_file.touch()
            _wait_for_exit("gazebo_evaluator", evaluator_process, 10.0)
            if not all(item.is_file() for item in (mission_output, observer_output, evaluator_output)):
                raise RuntimeError("one or more evidence producers exited without a report")
            mission_report = json.loads(mission_output.read_text(encoding="utf-8"))
            observer_report = json.loads(observer_output.read_text(encoding="utf-8"))
            gazebo_report = json.loads(evaluator_output.read_text(encoding="utf-8"))
            checks.extend(_checks(raw, mission_report, observer_report, gazebo_report, evaluator_process.poll()))
        except (KeyboardInterrupt, OSError, RuntimeError, StartupTimeout, UnexpectedProcessExit) as error:
            if mission_process is not None:
                _interrupt(mission_process)
                try:
                    mission_process.wait(timeout=float(raw["mission"]["cleanup_timeout_s"]) + 2.0)
                except subprocess.TimeoutExpired:
                    pass
            checks.append(check("smoke.execution", Status.FAIL, str(error) or "interrupted"))
        finally:
            stop_file.touch()
            for handle in handles:
                handle.close()
            cleanup = supervisor.cleanup()
            exits = {name: process.poll() for name, process in supervisor.processes}
            ros_clean = _ros_nodes_absent((raw["runner"]["node_name"], raw["runner"]["observer_node_name"]))
            cleanup.append(check(
                "cleanup.ros_nodes", Status.PASS if ros_clean else Status.FAIL,
                "mission and observer nodes absent from the ROS graph" if ros_clean else "mission or observer remains in the ROS graph",
            ))
            if stack_mode == "owned":
                cleanup.extend(_cleanup_port_checks(base))
            else:
                cleanup.append(check("cleanup.external_stack_preserved", Status.PASS, "pre-existing graphical Gazebo and SITL were not owned or terminated"))
            passed = bool(checks) and not any(item.status is Status.FAIL for item in checks + cleanup)
            if not passed:
                logs_retained = _retain_failure_logs(paths, output_path, int(raw["logging"]["failure_log_max_bytes"]))
    return {
        "schema_version": "echorescue-indoor-smoke/1.0", "milestone": "v0.14.5",
        "status": Status.PASS if passed else Status.FAIL, "mode": "graphical" if graphical else "headless", "stack_mode": stack_mode,
        "world_version": config.world_version, "dependencies": raw["dependencies"],
        "checks": [asdict(item) for item in sorted(checks, key=lambda item: item.name)],
        "cleanup": [asdict(item) for item in sorted(cleanup, key=lambda item: item.name)],
        "process_exit_statuses": exits, "mission": mission_report, "observer": observer_report,
        "gazebo_evaluation": gazebo_report, "logs": logs_retained,
    }


def negative_collision(path: Path, output_path: Path | None) -> dict[str, Any]:
    """Teleport the unarmed model into the obstacle and require explicit failure."""
    raw, config = load_configured_stack(path)
    base = load_config(REPOSITORY_ROOT / raw["base_stack_config"])
    supervisor = ProcessSupervisor()
    checks: list[Check] = []
    evaluation: dict[str, Any] = {}
    with tempfile.TemporaryDirectory(prefix="echorescue-v0145-negative-") as raw_temp:
        temp = Path(raw_temp)
        gazebo_log_path, evaluator_log_path = temp / "gazebo.log", temp / "evaluator.log"
        evaluator_output, stop_file = temp / "evaluation.json", temp / "stop"
        with gazebo_log_path.open("w", encoding="utf-8") as gazebo_log, evaluator_log_path.open("w", encoding="utf-8") as evaluator_log:
            try:
                world = str((REPOSITORY_ROOT / config.world_sdf).resolve())
                gazebo = supervisor.start("gazebo", ["gz", "sim", "-v4", "-r", "-s", world], REPOSITORY_ROOT, gazebo_log)
                supervisor.wait_until("gazebo", gazebo, lambda: _gazebo_ready(f"/world/{config.world_name}"), 30.0)
                evaluator = supervisor.start("gazebo_evaluator", [
                    sys.executable, "-m", "echorescue.gazebo_indoor_evaluator", "--config", str(path),
                    "--output", str(evaluator_output), "--stop-file", str(stop_file),
                ], REPOSITORY_ROOT, evaluator_log)
                service = f"/world/{config.world_name}/set_pose"
                request = (
                    f'name: "{config.model_name}" position {{ x: {config.obstacle.center[0]} '
                    f'y: {config.obstacle.center[1]} z: {config.obstacle.center[2]} }} orientation {{ w: 1 }}'
                )
                result = subprocess.run([
                    "gz", "service", "-s", service, "--reqtype", "gz.msgs.Pose",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", request,
                ], capture_output=True, text=True, check=False, timeout=10)
                if result.returncode != 0 or "true" not in result.stdout:
                    raise RuntimeError(f"controlled collision setup failed: {result.stderr or result.stdout}")
                try:
                    evaluator.wait(timeout=15.0)
                except subprocess.TimeoutExpired as error:
                    raise RuntimeError("negative evaluator did not fail within 15 seconds") from error
                if not evaluator_output.is_file():
                    raise RuntimeError("negative evaluator produced no report")
                evaluation = json.loads(evaluator_output.read_text(encoding="utf-8"))
                detected = (
                    evaluator.poll() != 0 and evaluation.get("status") == "FAIL"
                    and evaluation.get("prohibited_contact_detected") is True
                    and any(
                        item.get("entity") == "blocking_obstacle"
                        and item.get("source") == "gazebo_contact_sensor"
                        for item in evaluation.get("prohibited_contacts", [])
                    )
                )
                checks.append(check("negative.obstacle_collision_detected", Status.PASS if detected else Status.FAIL, f"evaluator exit={evaluator.poll()}; sources={evaluation.get('prohibited_contacts')}"))
            except (OSError, RuntimeError, StartupTimeout, UnexpectedProcessExit) as error:
                checks.append(check("negative.execution", Status.FAIL, str(error)))
            finally:
                stop_file.touch()
        cleanup = supervisor.cleanup()
        cleanup.extend(_cleanup_port_checks(base))
        passed = bool(checks) and not any(item.status is Status.FAIL for item in checks + cleanup)
        logs = _retain_failure_logs(
            {"gazebo": gazebo_log_path, "evaluator": evaluator_log_path}, output_path,
            int(raw["logging"]["failure_log_max_bytes"]),
        ) if not passed else []
    return {
        "schema_version": "echorescue-indoor-negative-collision/1.0", "milestone": "v0.14.5",
        "status": Status.PASS if passed else Status.FAIL, "evaluation": evaluation,
        "checks": [asdict(item) for item in checks], "cleanup": [asdict(item) for item in cleanup], "logs": logs,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate the EchoRescue v0.14.5 indoor reference stack")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("diagnose")
    smoke = sub.add_parser("smoke")
    smoke.add_argument("--timeout", type=float, default=260.0)
    smoke.add_argument("--graphical", action="store_true")
    smoke.add_argument("--stack-mode", choices=("owned", "attached"), default="owned")
    sub.add_parser("negative-collision")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "diagnose":
        report = indoor_diagnose(args.config)
    elif args.command == "negative-collision":
        report = negative_collision(args.config, args.output)
    else:
        report = indoor_smoke(args.config, args.output, args.timeout, args.graphical, args.stack_mode)
    rendered = serialize_report(report)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    sys.stdout.write(rendered)
    return 0 if report.get("ready", report.get("status") == Status.PASS) else 1


if __name__ == "__main__":
    raise SystemExit(main())
