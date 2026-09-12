"""Owned-stack v0.15.1 sensor discovery and bounded-replanning evidence."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any, Mapping, Sequence

from echorescue.indoor_evaluation import load_indoor_config
from echorescue.indoor_integration import _interrupt, _mission_command, _ros_nodes_absent
from echorescue.planner_flight import KnownMapPlan, plan_known_map
from echorescue.sim_integration import (
    Check, ProcessSupervisor, REPOSITORY_ROOT, Status, _gazebo_ready,
    _retain_failure_logs, _tcp_ready, check, load_config, serialize_report,
)
from echorescue.telemetry_integration import _cleanup_port_checks, _ros_graph_ready


DEFAULT_CONFIG = REPOSITORY_ROOT / "config" / "sensor-replanning-v0.15.1.json"


def _generated_indoor(raw: Mapping[str, Any], plan: KnownMapPlan, unreachable: bool) -> dict[str, Any]:
    indoor_path = REPOSITORY_ROOT / str(raw["base_indoor_config"])
    indoor: dict[str, Any] = json.loads(indoor_path.read_text(encoding="utf-8"))
    indoor["mission"] = {
        "takeoff_altitude_m": plan.cruise_altitude_m,
        **raw["flight"],
        "targets": [
            {key: value for key, value in target.items() if key not in {"source_cell", "leg"}}
            for target in plan.targets
        ],
    }
    if unreachable:
        indoor["world"].update({
            "version": "echorescue-sensor-indoor-unreachable-v1",
            "name": "echorescue_indoor_v0_15_1_unreachable",
            "sdf_path": "worlds/echorescue_indoor_v0_15_1_unreachable.sdf",
        })
        indoor["evaluation"]["pose_topic"] = "/world/echorescue_indoor_v0_15_1_unreachable/pose/info"
        unknown = next(item for item in indoor["geometry"]["entities"] if item["name"] == "unknown_obstacle")
        unknown.update({"name": "unknown_barrier", "center": [-1.0, 0.0, 1.2], "size": [0.7, 5.0, 2.4],
                        "contact_topic": "/echorescue/indoor/contacts/unknown_barrier"})
    load_indoor_config_data(indoor)
    return indoor


def load_indoor_config_data(raw: Mapping[str, Any]) -> None:
    """Validate generated configuration through the canonical file loader."""
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", encoding="utf-8") as handle:
        json.dump(raw, handle)
        handle.flush()
        load_indoor_config(Path(handle.name))


def _sensor_mission_command(indoor: Mapping[str, Any], output: Path, planner_config: Path) -> list[str]:
    command = _mission_command(indoor, output)
    command[2] = "echorescue_ros.mavlink_sensor_replanning_mission"
    ros_index = command.index("--ros-args")
    command[ros_index:ros_index] = ["--planner-config", str(planner_config)]
    return command


def _wait(process: Any, timeout_s: float) -> int | None:
    try:
        return process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        return None


def _checks(mission: Mapping[str, Any], observer: Mapping[str, Any], gazebo: Mapping[str, Any],
            mission_exit: int | None, observer_exit: int | None, unreachable: bool,
            original_outbound: tuple[tuple[int, int], ...]) -> list[Check]:
    commands = mission.get("commands", [])
    gated = bool(commands) and all(
        item.get("ack_result") == 0
        and isinstance(item.get("ack_monotonic_ns"), int)
        and isinstance(item.get("telemetry_transition_monotonic_ns"), int)
        and item["telemetry_transition_monotonic_ns"] > item["ack_monotonic_ns"]
        for item in commands
    )
    updates = mission.get("map_updates", [])
    replans = mission.get("replans", [])
    common = [
        check("mission.command_ack_then_telemetry", Status.PASS if gated else Status.FAIL,
              "every command ACK has a later independently observed transition"),
        check("sensor.accepted_and_map_updated", Status.PASS if updates else Status.FAIL,
              f"observations={len(mission.get('sensor_observations', []))}; updates={len(updates)}"),
        check("observer.independent_chain", Status.PASS if observer_exit == 0 and observer.get("status") == "PASS" else Status.FAIL,
              f"observer exit={observer_exit}; events={observer.get('events')}"),
        check("gazebo.no_prohibited_contact", Status.PASS if gazebo and not gazebo.get("prohibited_contact_detected", True) else Status.FAIL,
              f"contacts={gazebo.get('prohibited_contacts')}"),
    ]
    if unreachable:
        safe = (mission_exit == 1 and mission.get("status") == "FAIL"
                and mission.get("replanning_failure_reason") == "no safe route after sensor discovery"
                and mission.get("final_landed") is True and mission.get("final_armed") is False
                and len(replans) == 1 and replans[0].get("status") == "FAIL")
        common.append(check("replan.unreachable_bounded_land", Status.PASS if safe else Status.FAIL,
                            f"exit={mission_exit}; reason={mission.get('replanning_failure_reason')}"))
    else:
        replacement_differs = bool(replans) and replans[0].get("compacted_outbound") != [list(cell) for cell in original_outbound]
        targets = observer.get("target_ids", [])
        success = (mission_exit == 0 and mission.get("status") == "PASS" and len(replans) == 1
                   and replans[0].get("status") == "PASS" and mission.get("final_landed") is True
                   and mission.get("final_armed") is False and gazebo.get("status") == "PASS"
                   and replacement_differs and any(str(item).endswith("outbound-goal") for item in targets)
                   and any(str(item).endswith("return-launch") for item in targets))
        common.append(check("replan.single_successful_replacement", Status.PASS if success else Status.FAIL,
                            f"exit={mission_exit}; replans={len(replans)}; gazebo={gazebo.get('status')}"))
    return common


def smoke(path: Path, output: Path | None, timeout_s: float, graphical: bool,
          unreachable: bool = False) -> dict[str, Any]:
    raw: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
    plan = plan_known_map(raw)
    indoor = _generated_indoor(raw, plan, unreachable)
    _, config = load_indoor_config(REPOSITORY_ROOT / str(raw["base_indoor_config"]))
    if unreachable:
        config_world = "echorescue_indoor_v0_15_1_unreachable"
    else:
        config_world = config.world_name
    supervisor = ProcessSupervisor()
    checks: list[Check] = []
    mission: dict[str, Any] = {}
    observer: dict[str, Any] = {}
    gazebo: dict[str, Any] = {}
    base = load_config(REPOSITORY_ROOT / str(indoor["base_stack_config"]))
    with tempfile.TemporaryDirectory(prefix="echorescue-v0151-") as temporary:
        temp = Path(temporary)
        generated = temp / "indoor.json"
        generated.write_text(json.dumps(indoor), encoding="utf-8")
        outputs = {name: temp / f"{name}.json" for name in ("mission", "observer", "gazebo")}
        logs = {name: temp / f"{name}.log" for name in ("gazebo", "sitl", "sensor", "mission", "observer", "evaluator")}
        handles = {name: file.open("w", encoding="utf-8") for name, file in logs.items()}
        stop = temp / "stop"
        mission_process = observer_process = evaluator_process = None
        try:
            resource_paths = [str((REPOSITORY_ROOT / "models").resolve())]
            gazebo_home = os.environ.get("ARDUPILOT_GAZEBO_HOME")
            if gazebo_home:
                resource_paths.append(str((Path(gazebo_home) / "models").resolve()))
            existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
            resource_paths.extend(item for item in existing.split(":") if item)
            os.environ["GZ_SIM_RESOURCE_PATH"] = ":".join(dict.fromkeys(resource_paths))
            world = REPOSITORY_ROOT / str(indoor["world"]["sdf_path"])
            gazebo_process = supervisor.start("gazebo", ["gz", "sim", "-v4", "-r", str(world)] + ([] if graphical else ["-s"]), REPOSITORY_ROOT, handles["gazebo"])
            supervisor.wait_until("gazebo", gazebo_process, lambda: _gazebo_ready(f"/world/{config_world}"), 30)
            sitl = [str(Path(os.environ["ARDUPILOT_HOME"]) / "Tools/autotest/sim_vehicle.py"), "-v", "ArduCopter", "-f", "gazebo-iris", "--model", "JSON", "--no-rebuild", "--no-mavproxy"]
            sitl_process = supervisor.start("ardupilot_sitl", sitl, Path(os.environ["ARDUPILOT_HOME"]), handles["sitl"])
            supervisor.wait_until("ardupilot_sitl", sitl_process, lambda: _tcp_ready("127.0.0.1", 5760), 40)
            evaluator_process = supervisor.start("gazebo_evaluator", [sys.executable, "-m", "echorescue.gazebo_indoor_evaluator", "--config", str(generated), "--output", str(outputs["gazebo"]), "--stop-file", str(stop)], REPOSITORY_ROOT, handles["evaluator"])
            sensor_process = supervisor.start("sensor_bridge", ["ros2", "run", "echorescue_ros", "gazebo_range_sensor_bridge", "--topic", str(raw["sensor_replanning"]["topic"])], REPOSITORY_ROOT, handles["sensor"])
            supervisor.wait_until("sensor_bridge", sensor_process, lambda: _ros_graph_ready("/echorescue_gazebo_range_sensor_bridge", {}), 20)
            observer_command = ["ros2", "run", "echorescue_ros", "sensor_replanning_observer", "--output", str(outputs["observer"]), "--timeout", str(timeout_s)]
            if unreachable:
                observer_command.append("--expect-failure")
            observer_process = supervisor.start("sensor_observer", observer_command, REPOSITORY_ROOT, handles["observer"])
            supervisor.wait_until("sensor_observer", observer_process, lambda: _ros_graph_ready("/echorescue_sensor_replanning_observer", {}), 20)
            mission_process = supervisor.start("sensor_mission", _sensor_mission_command(indoor, outputs["mission"], path), REPOSITORY_ROOT, handles["mission"])
            supervisor.wait_until("sensor_mission", mission_process, lambda: _ros_graph_ready("/echorescue_mavlink_waypoint_mission", {}), 20)
            mission_exit = _wait(mission_process, timeout_s)
            if mission_exit is None:
                _interrupt(mission_process)
                mission_exit = _wait(mission_process, float(raw["flight"]["cleanup_timeout_s"]) + 2)
            observer_exit = _wait(observer_process, 15)
            stop.touch()
            evaluator_exit = _wait(evaluator_process, 15)
            if outputs["mission"].is_file():
                mission = json.loads(outputs["mission"].read_text(encoding="utf-8"))
            if outputs["observer"].is_file():
                observer = json.loads(outputs["observer"].read_text(encoding="utf-8"))
            if outputs["gazebo"].is_file():
                gazebo = json.loads(outputs["gazebo"].read_text(encoding="utf-8"))
            if not all(item.is_file() for item in outputs.values()):
                missing = [name for name, item in outputs.items() if not item.is_file()]
                raise RuntimeError(
                    f"independent evidence reports missing={missing}; "
                    f"mission_exit={mission_exit}; observer_exit={observer_exit}; evaluator_exit={evaluator_exit}"
                )
            checks.extend(_checks(mission, observer, gazebo, mission_exit, observer_exit, unreachable, plan.compacted_outbound))
            checks.append(check("evaluator.exit", Status.PASS if (evaluator_exit == (1 if unreachable else 0)) else Status.FAIL,
                                f"exit={evaluator_exit}; expected={1 if unreachable else 0}"))
        except (KeyboardInterrupt, OSError, RuntimeError, KeyError, ValueError) as error:
            checks.append(check("smoke.execution", Status.FAIL, str(error) or "interrupted"))
        finally:
            stop.touch()
            if mission_process is not None:
                _interrupt(mission_process)
            for handle in handles.values():
                handle.close()
            cleanup = supervisor.cleanup()
            cleanup.extend(_cleanup_port_checks(base))
            nodes_clean = _ros_nodes_absent(("/echorescue_mavlink_waypoint_mission", "/echorescue_sensor_replanning_observer", "/echorescue_gazebo_range_sensor_bridge"))
            cleanup.append(check("cleanup.ros_nodes", Status.PASS if nodes_clean else Status.FAIL, "v0.15.1 ROS nodes absent"))
            passed = bool(checks) and not any(item.status is Status.FAIL for item in checks + cleanup)
            retained = _retain_failure_logs(logs, output, int(indoor["logging"]["failure_log_max_bytes"])) if not passed else []
    report = {
        "schema_version": "echorescue-sensor-replanning-smoke/1.0", "milestone": "v0.15.1",
        "status": Status.PASS if passed else Status.FAIL, "mode": "graphical" if graphical else "headless",
        "case": "unreachable-after-discovery" if unreachable else "unknown-obstacle-replan",
        "planning": plan.report(), "mission": mission, "observer": observer, "gazebo_evaluation": gazebo,
        "checks": [asdict(item) for item in checks], "cleanup": [asdict(item) for item in cleanup], "logs": retained,
    }
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(serialize_report(report), encoding="utf-8")
    return report


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("smoke", "negative-unreachable"):
        item = sub.add_parser(name)
        item.add_argument("--timeout", type=float, default=280)
        item.add_argument("--graphical", action="store_true")
    args = parser.parse_args(argv)
    report = smoke(args.config, args.output, args.timeout, args.graphical, args.command == "negative-unreachable")
    sys.stdout.write(serialize_report(report))
    return 0 if report["status"] == Status.PASS else 1


if __name__ == "__main__":
    raise SystemExit(main())
