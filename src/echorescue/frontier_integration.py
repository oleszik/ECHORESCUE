"""Owned-stack v0.16.0 autonomous frontier exploration evidence harness."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from math import hypot
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any, Mapping, Sequence

from echorescue.indoor_evaluation import load_indoor_config, segment_box_distance
from echorescue.indoor_integration import _interrupt, _ros_nodes_absent
from echorescue.sim_integration import (
    Check, ProcessSupervisor, REPOSITORY_ROOT, Status, _gazebo_ready,
    _retain_failure_logs, _tcp_ready, check, load_config, serialize_report,
)
from echorescue.telemetry_integration import _cleanup_port_checks, _ros_graph_ready


DEFAULT_CONFIG = REPOSITORY_ROOT / "config/frontier-exploration-v0.16.0.json"


def _wait(process: Any, timeout_s: float) -> int | None:
    try:
        return process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        return None


def _scenario_config(raw: Mapping[str, Any], case: str, temporary: Path) -> Path:
    generated = json.loads(json.dumps(raw))
    if case == "budget":
        generated["limits"]["maximum_frontier_targets"] = 1
    elif case == "unreachable":
        generated["frontier_policy"]["minimum_cluster_size"] = 999
        generated["completion"]["minimum_known_ratio"] = 0.95
    path = temporary / "frontier.json"
    path.write_text(json.dumps(generated), encoding="utf-8")
    return path


def _mission_command(indoor: Mapping[str, Any], output: Path, config: Path) -> list[str]:
    runner, mission = indoor["runner"], indoor["mission"]
    command = [
        sys.executable, "-m", "echorescue_ros.mavlink_frontier_exploration",
        "--output", str(output), "--config", str(config), "--ros-args",
        "-p", f"endpoint:={runner['endpoint']}",
        "-p", f"stream_rate_hz:={runner['stream_rate_hz']}",
        "-p", f"freshness_threshold_s:={runner['freshness_threshold_s']}",
        "-p", f"disconnect_threshold_s:={runner['disconnect_threshold_s']}",
        "-p", f"reconnect_interval_s:={runner['reconnect_interval_s']}",
    ]
    for name in (
        "takeoff_altitude_m", "horizontal_tolerance_m", "vertical_tolerance_m",
        "settling_time_s", "target_timeout_s", "progress_timeout_s",
        "progress_epsilon_m", "mission_timeout_s", "geofence_horizontal_radius_m",
        "geofence_min_altitude_m", "geofence_max_altitude_m", "preflight_hold_s",
        "transition_timeout_s", "takeoff_timeout_s", "landing_timeout_s",
        "cleanup_timeout_s",
    ):
        if name in mission:
            command.extend(("-p", f"{name}:={mission[name]}"))
    return command


def _truth_coverage(gazebo: Mapping[str, Any], indoor_path: Path, range_m: float) -> dict[str, Any]:
    _, config = load_indoor_config(indoor_path)
    trajectory = gazebo.get("trajectory", [])
    obstacles = tuple(item for item in config.entities if item.classification in {"wall", "obstacle"})
    resolution = 0.5
    samples: list[tuple[float, float]] = []
    x = config.allowed_minimum[0] + resolution / 2
    while x < config.allowed_maximum[0]:
        y = config.allowed_minimum[1] + resolution / 2
        while y < config.allowed_maximum[1]:
            point = (x, y, config.launch_world_enu[2] + config.takeoff_altitude_m)
            if all(segment_box_distance(point, point, item) > config.vehicle_radius_m for item in obstacles):
                samples.append((x, y))
            y += resolution
        x += resolution
    def visible(point: tuple[float, float]) -> bool:
        end = (point[0], point[1], config.launch_world_enu[2] + config.takeoff_altitude_m)
        for pose in trajectory:
            start = (float(pose["east_m"]), float(pose["north_m"]), float(pose["up_m"]))
            if hypot(end[0] - start[0], end[1] - start[1]) > range_m:
                continue
            if all(segment_box_distance(start, end, item) > 0.0 for item in obstacles):
                return True
        return False
    covered = {point for point in samples if visible(point)}
    first = {point for point in samples if point[0] < config.doorway.plane_coordinate_m}
    second = set(samples) - first
    ratio = lambda group: len(covered & group) / len(group) if group else 0.0
    final = trajectory[-1] if trajectory else None
    return {
        "schema_version": "echorescue-frontier-gazebo-coverage/1.0",
        "source": "evaluator-only Gazebo trajectory plus configured geometry",
        "navigation_input": False, "sample_resolution_m": resolution,
        "explorable_sample_count": len(samples), "covered_sample_count": len(covered),
        "true_coverage_ratio": ratio(set(samples)),
        "first_room_coverage_ratio": ratio(first),
        "second_room_coverage_ratio": ratio(second),
        "final_distance_from_launch_m": None if final is None else hypot(
            float(final["east_m"]) - config.launch_world_enu[0],
            float(final["north_m"]) - config.launch_world_enu[1],
        ),
        "final_landed_height_observed": bool(final and float(final["up_m"]) <= 0.35),
    }


def _checks(case: str, mission: Mapping[str, Any], observer: Mapping[str, Any],
            gazebo: Mapping[str, Any], coverage: Mapping[str, Any],
            mission_exit: int | None, observer_exit: int | None) -> list[Check]:
    commands = mission.get("commands", [])
    acknowledged = [item for item in commands if item.get("ack_monotonic_ns") is not None]
    gated = bool(acknowledged) and all(
        item.get("ack_result") == 0
        and isinstance(item.get("telemetry_transition_monotonic_ns"), int)
        and item["telemetry_transition_monotonic_ns"] > item["ack_monotonic_ns"]
        for item in acknowledged
    )
    safe_routes = all(item.get("known_safe_only") for item in mission.get("route_history", []))
    mission_targets = list(dict.fromkeys(
        item["target_id"] for item in mission.get("targets", [])
        if item.get("transmitted") is True
    ))
    observer_targets = observer.get("target_ids", [])
    mission_generations = {
        int(item["route_generation"]) for item in mission.get("route_history", [])
    }
    observer_generations = {int(value) for value in observer.get("route_generations", [])}
    observer_sessions = observer.get("sessions", [])
    expected_session_count = 2 if case == "telemetry_reconnect" else 1
    agreement = (
        mission.get("session_id") in observer_sessions
        and len(observer_sessions) == expected_session_count
        and mission_targets == observer_targets
        and mission_generations <= observer_generations
        and observer.get("on_ground_observed") is True
        and observer.get("disarmed_observed") is True
    )
    common = [
        check("mission.command_ack_then_telemetry", Status.PASS if gated else Status.FAIL, "all command ACKs precede independently observed transitions"),
        check("mission.controller_prior_unknown", Status.PASS if mission.get("controller_prior_interior_geometry") is False else Status.FAIL, "controller configuration contains bounds but no interior geometry"),
        check("mission.routes_known_safe", Status.PASS if safe_routes else Status.FAIL, "all route generations contain controller-known safe cells only"),
        check(
            "mission.observer_agreement", Status.PASS if agreement else Status.FAIL,
            f"sessions={observer_sessions}; targets={len(mission_targets)}; routes={len(mission_generations)}",
        ),
        check("gazebo.no_prohibited_contact", Status.PASS if not gazebo.get("prohibited_contact_detected", True) else Status.FAIL, f"contacts={gazebo.get('prohibited_contacts')}"),
    ]
    final_safe = mission.get("final_landed") is True and mission.get("final_armed") is False
    if case == "success":
        passed = (
            mission_exit == 0 and observer_exit == 0 and mission.get("status") == "PASS"
            and observer.get("status") == "PASS" and gazebo.get("status") == "PASS"
            and int(mission.get("map_counts", {}).get("free_cell_count", 0)) > 0
            and len(mission.get("map_revisions", [])) > 1
            and len(mission.get("frontier_history", [])) > 1
            and mission.get("exploration_completion_reason")
            and coverage.get("second_room_coverage_ratio", 0) >= 0.5
            and final_safe
        )
        common.append(check("exploration.autonomous_success", Status.PASS if passed else Status.FAIL, f"mission={mission_exit}; observer={observer_exit}; coverage={coverage.get('true_coverage_ratio')}"))
    else:
        reason = str(mission.get("exploration_failure_reason") or mission.get("failure_reason") or "")
        expected = {
            "budget": "frontier target budget exhausted",
            "unreachable": "no reachable frontier",
            "sensor_dropout": "LiDAR telemetry became stale",
            "telemetry_reconnect": "session",
        }[case]
        passed = mission_exit == 1 and observer_exit == 0 and expected.lower() in reason.lower() and final_safe
        common.append(check(f"exploration.{case}_bounded", Status.PASS if passed else Status.FAIL, f"reason={reason}; landed={final_safe}"))
    return common


def smoke(config_path: Path, output: Path | None, timeout_s: float, graphical: bool,
          case: str = "success") -> dict[str, Any]:
    raw: dict[str, Any] = json.loads(config_path.read_text(encoding="utf-8"))
    indoor_path = REPOSITORY_ROOT / str(raw["base_indoor_config"])
    indoor, indoor_config = load_indoor_config(indoor_path)
    base = load_config(REPOSITORY_ROOT / str(indoor["base_stack_config"]))
    supervisor, checks = ProcessSupervisor(), []
    mission: dict[str, Any] = {}
    observer: dict[str, Any] = {}
    gazebo: dict[str, Any] = {}
    coverage: dict[str, Any] = {}
    exits: dict[str, int | None] = {}
    with tempfile.TemporaryDirectory(prefix="echorescue-v0160-") as name:
        temporary = Path(name)
        scenario = _scenario_config(raw, case, temporary)
        outputs = {item: temporary / f"{item}.json" for item in ("mission", "observer", "gazebo")}
        logs = {item: temporary / f"{item}.log" for item in ("gazebo", "sitl", "sensor", "mission", "observer", "evaluator")}
        handles = {item: path.open("w", encoding="utf-8") for item, path in logs.items()}
        stop = temporary / "stop"
        mission_process = observer_process = evaluator_process = sensor_process = sitl_process = None
        try:
            resources = [str((REPOSITORY_ROOT / "models").resolve())]
            if os.environ.get("ARDUPILOT_GAZEBO_HOME"):
                resources.append(str((Path(os.environ["ARDUPILOT_GAZEBO_HOME"]) / "models").resolve()))
            resources.extend(item for item in os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(":") if item)
            os.environ["GZ_SIM_RESOURCE_PATH"] = ":".join(dict.fromkeys(resources))
            world = REPOSITORY_ROOT / str(indoor["world"]["sdf_path"])
            gazebo_process = supervisor.start("gazebo", ["gz", "sim", "-v4", "-r", str(world)] + ([] if graphical else ["-s"]), REPOSITORY_ROOT, handles["gazebo"])
            supervisor.wait_until("gazebo", gazebo_process, lambda: _gazebo_ready(f"/world/{indoor_config.world_name}"), 30)
            sitl_command = [str(Path(os.environ["ARDUPILOT_HOME"]) / "Tools/autotest/sim_vehicle.py"), "-v", "ArduCopter", "-f", "gazebo-iris", "--model", "JSON", "--no-rebuild", "--no-mavproxy"]
            sitl_process = supervisor.start("ardupilot_sitl", sitl_command, Path(os.environ["ARDUPILOT_HOME"]), handles["sitl"])
            supervisor.wait_until("ardupilot_sitl", sitl_process, lambda: _tcp_ready("127.0.0.1", 5760), 40)
            evaluator_process = supervisor.start("gazebo_evaluator", [sys.executable, "-m", "echorescue.gazebo_indoor_evaluator", "--config", str(indoor_path), "--output", str(outputs["gazebo"]), "--stop-file", str(stop)], REPOSITORY_ROOT, handles["evaluator"])
            sensor_process = supervisor.start("lidar_bridge", ["ros2", "run", "echorescue_ros", "gazebo_range_sensor_bridge", "--topic", str(raw["sensor"]["topic"])], REPOSITORY_ROOT, handles["sensor"])
            supervisor.wait_until("lidar_bridge", sensor_process, lambda: _ros_graph_ready("/echorescue_gazebo_range_sensor_bridge", {}), 20)
            observer_command = ["ros2", "run", "echorescue_ros", "frontier_exploration_observer", "--output", str(outputs["observer"]), "--timeout", str(timeout_s)]
            if case != "success":
                observer_command.append("--expect-failure")
            observer_process = supervisor.start("exploration_observer", observer_command, REPOSITORY_ROOT, handles["observer"])
            supervisor.wait_until("exploration_observer", observer_process, lambda: _ros_graph_ready("/echorescue_frontier_exploration_observer", {}), 20)
            mission_process = supervisor.start("exploration_mission", _mission_command(indoor, outputs["mission"], scenario), REPOSITORY_ROOT, handles["mission"])
            supervisor.wait_until("exploration_mission", mission_process, lambda: _ros_graph_ready("/echorescue_mavlink_waypoint_mission", {}), 20)
            if case == "sensor_dropout":
                time.sleep(48)
                _interrupt(sensor_process)
            elif case == "telemetry_reconnect":
                time.sleep(48)
                _interrupt(sitl_process)
                time.sleep(4)
                sitl_process = supervisor.start("ardupilot_reconnect", sitl_command, Path(os.environ["ARDUPILOT_HOME"]), handles["sitl"])
                supervisor.wait_until("ardupilot_reconnect", sitl_process, lambda: _tcp_ready("127.0.0.1", 5760), 40)
            mission_exit = _wait(mission_process, timeout_s)
            if mission_exit is None:
                _interrupt(mission_process)
                mission_exit = _wait(mission_process, float(raw["flight"]["cleanup_timeout_s"]) + 2)
            observer_exit = _wait(observer_process, 15)
            stop.touch()
            evaluator_exit = _wait(evaluator_process, 15)
            for key, path in outputs.items():
                if path.is_file():
                    value = json.loads(path.read_text(encoding="utf-8"))
                    if key == "mission": mission = value
                    elif key == "observer": observer = value
                    else: gazebo = value
            if not all(path.is_file() for path in outputs.values()):
                raise RuntimeError(f"independent reports missing; mission={mission_exit}; observer={observer_exit}; evaluator={evaluator_exit}")
            coverage = _truth_coverage(gazebo, indoor_path, float(raw["sensor"]["range_max_m"]))
            checks.extend(_checks(case, mission, observer, gazebo, coverage, mission_exit, observer_exit))
        except (KeyboardInterrupt, OSError, RuntimeError, KeyError, ValueError) as error:
            checks.append(check("smoke.execution", Status.FAIL, str(error) or "interrupted"))
        finally:
            stop.touch()
            if mission_process is not None:
                _interrupt(mission_process)
            for handle in handles.values():
                handle.close()
            cleanup = supervisor.cleanup()
            exits = {process_name: process.poll() for process_name, process in supervisor.processes}
            cleanup.extend(_cleanup_port_checks(base))
            nodes = ("/echorescue_mavlink_waypoint_mission", "/echorescue_frontier_exploration_observer", "/echorescue_gazebo_range_sensor_bridge")
            cleanup.append(check("cleanup.ros_nodes", Status.PASS if _ros_nodes_absent(nodes) else Status.FAIL, "exploration ROS nodes absent"))
            passed = bool(checks) and not any(item.status is Status.FAIL for item in checks + cleanup)
            retained = _retain_failure_logs(logs, output, int(indoor["logging"]["failure_log_max_bytes"])) if not passed else []
    report = {
        "schema_version": "echorescue-frontier-exploration-smoke/1.0",
        "milestone": "v0.16.0", "case": case,
        "status": Status.PASS if passed else Status.FAIL,
        "mode": "graphical" if graphical else "headless",
        "configuration_digest": hashlib.sha256(config_path.read_bytes()).hexdigest(),
        "dependencies": indoor["dependencies"], "mission": mission,
        "observer": observer, "gazebo_evaluation": gazebo,
        "evaluator_only_coverage": coverage,
        "checks": [asdict(item) for item in checks],
        "process_exit_statuses": exits,
        "cleanup": [asdict(item) for item in cleanup], "logs": retained,
    }
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(serialize_report(report), encoding="utf-8")
    return report


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path)
    parser.add_argument("scenario", choices=("success", "unreachable", "sensor_dropout", "telemetry_reconnect", "budget"))
    parser.add_argument("--timeout", type=float, default=320)
    parser.add_argument("--graphical", action="store_true")
    args = parser.parse_args(argv)
    report = smoke(args.config, args.output, args.timeout, args.graphical, args.scenario)
    sys.stdout.write(serialize_report(report))
    return 0 if report["status"] == Status.PASS else 1


if __name__ == "__main__":
    raise SystemExit(main())
