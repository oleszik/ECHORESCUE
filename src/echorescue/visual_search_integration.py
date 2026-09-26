"""Owned-stack v0.16.1 camera survivor-search evidence harness."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
from typing import Any, Mapping, Sequence

from echorescue.frontier_integration import (
    _mission_command, _scenario_config, _truth_coverage, _wait,
)
from echorescue.indoor_evaluation import load_indoor_config
from echorescue.indoor_integration import _interrupt, _ros_nodes_absent
from echorescue.sim_integration import (
    Check, ProcessSupervisor, REPOSITORY_ROOT, Status, _gazebo_ready,
    _retain_failure_logs, _tcp_ready, check, load_config, serialize_report,
)
from echorescue.telemetry_integration import _cleanup_port_checks, _ros_graph_ready
from echorescue.visual_survivor_evaluation import evaluate


DEFAULT_CONFIG = REPOSITORY_ROOT / "config/visual-survivor-v0.16.1.json"


def _scenario_files(raw: Mapping[str, Any], case: str, temporary: Path) -> tuple[Path, Path, Path]:
    exploration_path = REPOSITORY_ROOT / str(raw["base_exploration_config"])
    exploration: dict[str, Any] = json.loads(exploration_path.read_text())
    scenario = _scenario_config(exploration, "budget" if case == "budget" else "success", temporary)
    indoor_path = REPOSITORY_ROOT / str(exploration["base_indoor_config"])
    truth_path = REPOSITORY_ROOT / "config/visual-survivor-ground-truth-v0.16.1.json"
    if case != "no_survivor":
        return scenario, indoor_path, truth_path
    indoor = json.loads(indoor_path.read_text())
    world_source = REPOSITORY_ROOT / str(indoor["world"]["sdf_path"])
    text = world_source.read_text()
    text = re.sub(r'\s*<model name="survivor_(?:alpha|bravo|charlie)">.*?</model>', "", text, flags=re.DOTALL)
    world = temporary / "no-survivor.sdf"
    world.write_text(text)
    indoor["world"]["sdf_path"] = str(world)
    generated_indoor = temporary / "indoor.json"
    generated_indoor.write_text(json.dumps(indoor))
    exploration["base_indoor_config"] = str(generated_indoor)
    scenario.write_text(json.dumps(exploration))
    truth = json.loads(truth_path.read_text())
    truth["survivors"] = []
    generated_truth = temporary / "truth.json"
    generated_truth.write_text(json.dumps(truth))
    return scenario, generated_indoor, generated_truth


def _checks(case: str, mission: Mapping[str, Any], observer: Mapping[str, Any],
            perception: Mapping[str, Any], visual: Mapping[str, Any],
            gazebo: Mapping[str, Any], mission_exit: int | None,
            observer_exit: int | None) -> list[Check]:
    safe = mission.get("final_landed") is True and mission.get("final_armed") is False
    confirmations = observer.get("confirmed_survivors", [])
    multi = all(int(item["supporting_observation_count"]) >= 2
                and int(item["confirmed_ns"]) > int(item["first_seen_ns"])
                for item in confirmations)
    checks = [
        check("perception.controller_prior_hidden", Status.PASS if perception.get("survivor_prior_unavailable") is True else Status.FAIL, "survivor positions/count unavailable to production"),
        check("perception.real_frames", Status.PASS if int(perception.get("counts", {}).get("accepted", 0)) > 0 else Status.FAIL, f"counts={perception.get('counts') }"),
        check("perception.multi_observation_confirmation", Status.PASS if multi else Status.FAIL, f"confirmations={len(confirmations)}"),
        check("perception.evaluator_separate", Status.PASS if visual.get("evaluator_only") is True else Status.FAIL, "Ground Truth evaluated after execution"),
        check("mission.final_safe", Status.PASS if safe else Status.FAIL, f"landed={mission.get('final_landed')}; armed={mission.get('final_armed')}"),
        check("gazebo.no_prohibited_contact", Status.PASS if not gazebo.get("prohibited_contact_detected", True) else Status.FAIL, f"contacts={gazebo.get('prohibited_contacts')}"),
    ]
    if case in {"success", "partial_occlusion", "distractor"}:
        passed = (mission_exit == 0 and observer_exit == 0 and mission.get("status") == "PASS"
                  and observer.get("status") == "PASS" and visual.get("status") == "PASS"
                  and visual.get("precision") == 1.0 and visual.get("recall") == 1.0 and safe)
    elif case == "no_survivor":
        passed = (mission_exit == 0 and observer_exit == 0 and len(confirmations) == 0
                  and visual.get("false_positives") == 0 and safe)
    elif case == "camera_dropout":
        passed = (mission_exit == 0 and perception.get("camera_dropout_observed") is True and safe)
    else:
        passed = mission_exit == 1 and safe
    checks.append(check(f"visual_search.{case}", Status.PASS if passed else Status.FAIL,
                        f"mission={mission_exit}; observer={observer_exit}; precision={visual.get('precision')}; recall={visual.get('recall')}"))
    return checks


def _wait_until_frontier_selected(timeout_s: float) -> bool:
    try:
        result = subprocess.run(
            [
                "ros2", "topic", "echo", "--no-daemon", "--once",
                "--timeout", str(timeout_s),
                "--filter", "m.map_revision > 0 and m.route_generation > 1 and m.has_selected_frontier",
                "/echorescue/exploration/state",
                "echorescue_interfaces/msg/ExplorationState",
            ],
            capture_output=True, text=True, check=False, timeout=timeout_s + 5.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return (
        result.returncode == 0
        and re.search(r"(?m)^map_revision: [1-9]\d*$", result.stdout) is not None
        and re.search(r"(?m)^route_generation: [2-9]\d*$", result.stdout) is not None
        and "has_selected_frontier: true" in result.stdout
    )


def smoke(config_path: Path, output: Path | None, timeout_s: float,
          graphical: bool, case: str) -> dict[str, Any]:
    raw: dict[str, Any] = json.loads(config_path.read_text())
    supervisor = ProcessSupervisor()
    checks: list[Check] = []
    mission: dict[str, Any] = {}; observer: dict[str, Any] = {}
    perception: dict[str, Any] = {}; gazebo: dict[str, Any] = {}
    visual: dict[str, Any] = {}; coverage: dict[str, Any] = {}
    exits: dict[str, int | None] = {}
    with tempfile.TemporaryDirectory(prefix="echorescue-v0161-") as name:
        temporary = Path(name)
        scenario, indoor_path, truth_path = _scenario_files(raw, case, temporary)
        indoor, indoor_config = load_indoor_config(indoor_path)
        base = load_config(REPOSITORY_ROOT / str(indoor["base_stack_config"]))
        outputs = {item: temporary / f"{item}.json" for item in ("mission", "observer", "perception", "gazebo")}
        log_names = ("gazebo", "gazebo_gui", "sitl", "lidar", "camera", "perception", "mission", "observer", "evaluator")
        logs = {item: temporary / f"{item}.log" for item in log_names}
        handles = {item: path.open("w", encoding="utf-8") for item, path in logs.items()}
        stop = temporary / "stop"
        processes: dict[str, Any] = {}
        try:
            resources = [str((REPOSITORY_ROOT / "models").resolve())]
            if os.environ.get("ARDUPILOT_GAZEBO_HOME"):
                gazebo_home = Path(os.environ["ARDUPILOT_GAZEBO_HOME"])
                resources.append(str((gazebo_home / "models").resolve()))
                plugin_paths = [str((gazebo_home / "build").resolve())]
                plugin_paths.extend(
                    item for item in os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "").split(":") if item
                )
                os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(dict.fromkeys(plugin_paths))
            resources.extend(item for item in os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(":") if item)
            os.environ["GZ_SIM_RESOURCE_PATH"] = ":".join(dict.fromkeys(resources))
            world = Path(str(indoor["world"]["sdf_path"]))
            if not world.is_absolute(): world = REPOSITORY_ROOT / world
            processes["gazebo"] = supervisor.start(
                "gazebo", ["gz", "sim", "-v4", "-r", "-s", str(world)],
                REPOSITORY_ROOT, handles["gazebo"],
            )
            supervisor.wait_until("gazebo", processes["gazebo"], lambda: _gazebo_ready(f"/world/{indoor_config.world_name}"), 30)
            sitl_command = [str(Path(os.environ["ARDUPILOT_HOME"]) / "Tools/autotest/sim_vehicle.py"), "-v", "ArduCopter", "-f", "gazebo-iris", "--model", "JSON", "--no-rebuild", "--no-mavproxy"]
            processes["sitl"] = supervisor.start("ardupilot_sitl", sitl_command, Path(os.environ["ARDUPILOT_HOME"]), handles["sitl"])
            supervisor.wait_until("sitl", processes["sitl"], lambda: _tcp_ready("127.0.0.1", 5760), 40)
            processes["evaluator"] = supervisor.start("gazebo_evaluator", [sys.executable, "-m", "echorescue.gazebo_indoor_evaluator", "--config", str(indoor_path), "--output", str(outputs["gazebo"]), "--stop-file", str(stop)], REPOSITORY_ROOT, handles["evaluator"])
            processes["lidar"] = supervisor.start("lidar_bridge", ["ros2", "run", "echorescue_ros", "gazebo_range_sensor_bridge", "--topic", str(json.loads(scenario.read_text())["sensor"]["topic"])], REPOSITORY_ROOT, handles["lidar"])
            camera = raw["camera"]
            processes["camera"] = supervisor.start("camera_bridge", ["ros2", "run", "echorescue_ros", "gazebo_camera_bridge", "--topic", camera["gazebo_topic"], "--frame", camera["frame"], "--fx", str(camera["fx"]), "--fy", str(camera["fy"]), "--cx", str(camera["cx"]), "--cy", str(camera["cy"])], REPOSITORY_ROOT, handles["camera"])
            processes["perception"] = supervisor.start("visual_perception", ["ros2", "run", "echorescue_ros", "visual_survivor_perception", "--config", str(config_path), "--output", str(outputs["perception"])], REPOSITORY_ROOT, handles["perception"])
            processes["observer"] = supervisor.start("visual_observer", ["ros2", "run", "echorescue_ros", "visual_search_observer", "--output", str(outputs["observer"]), "--timeout", str(timeout_s)] + (["--expect-failure"] if case in {"session_change", "budget"} else []), REPOSITORY_ROOT, handles["observer"])
            processes["mission"] = supervisor.start("exploration_mission", _mission_command(indoor, outputs["mission"], scenario), REPOSITORY_ROOT, handles["mission"])
            supervisor.wait_until("mission", processes["mission"], lambda: _ros_graph_ready("/echorescue_mavlink_waypoint_mission", {}), 20)
            if graphical:
                # Start visualization only after the server-side rendering and
                # flight graph are initialized. This avoids racing two Ogre
                # contexts while keeping GUI failure outside the sensor path.
                time.sleep(10)
                processes["gazebo_gui"] = supervisor.start(
                    "gazebo_gui", ["gz", "sim", "-v4", "-g"],
                    REPOSITORY_ROOT, handles["gazebo_gui"],
                )
                time.sleep(5)
                if processes["gazebo_gui"].poll() is not None:
                    raise RuntimeError("Gazebo GUI exited before graphical mission observation")
            if case == "camera_dropout":
                time.sleep(48); _interrupt(processes["camera"])
            elif case == "session_change":
                exploration_config = REPOSITORY_ROOT / str(raw["base_exploration_config"])
                exploration = json.loads(exploration_config.read_text())
                exploration_ready_timeout = sum(
                    float(exploration["flight"][key]) for key in (
                        "initialization_timeout_s", "preflight_hold_s", "takeoff_timeout_s",
                    )
                )
                if not _wait_until_frontier_selected(exploration_ready_timeout):
                    raise RuntimeError(
                        "no mapped active frontier was observed before session-change injection"
                    )
                _interrupt(processes["sitl"])
                if _wait(processes["sitl"], 10.0) is None:
                    raise RuntimeError("original SITL process did not stop before session reconnect")
                processes["sitl_reconnect"] = supervisor.start("ardupilot_reconnect", sitl_command, Path(os.environ["ARDUPILOT_HOME"]), handles["sitl"])
                supervisor.wait_until(
                    "ardupilot_reconnect", processes["sitl_reconnect"],
                    lambda: _tcp_ready("127.0.0.1", 5760), 40,
                )
            mission_exit = _wait(processes["mission"], timeout_s)
            if mission_exit is None:
                _interrupt(processes["mission"]); mission_exit = _wait(processes["mission"], 47)
            observer_exit = _wait(processes["observer"], 15)
            _interrupt(processes["perception"]); _wait(processes["perception"], 10)
            stop.touch(); _wait(processes["evaluator"], 15)
            for key, path in outputs.items():
                if path.is_file():
                    value = json.loads(path.read_text())
                    if key == "mission": mission = value
                    elif key == "observer": observer = value
                    elif key == "perception": perception = value
                    else: gazebo = value
            visual = evaluate(observer, json.loads(truth_path.read_text()), float(camera["horizontal_fov_rad"]))
            coverage = _truth_coverage(gazebo, indoor_path, 4.0)
            checks.extend(_checks(case, mission, observer, perception, visual, gazebo, mission_exit, observer_exit))
        except (KeyboardInterrupt, OSError, RuntimeError, KeyError, ValueError) as error:
            checks.append(check("smoke.execution", Status.FAIL, str(error) or "interrupted"))
        finally:
            stop.touch()
            for process in processes.values(): _interrupt(process)
            for handle in handles.values(): handle.close()
            cleanup = supervisor.cleanup()
            exits = {process_name: process.poll() for process_name, process in supervisor.processes}
            cleanup.extend(_cleanup_port_checks(base))
            nodes = ("/echorescue_mavlink_waypoint_mission", "/echorescue_visual_survivor_perception", "/echorescue_visual_search_observer", "/echorescue_gazebo_camera_bridge", "/echorescue_gazebo_range_sensor_bridge")
            cleanup.append(check("cleanup.ros_nodes", Status.PASS if _ros_nodes_absent(nodes) else Status.FAIL, "visual-search ROS nodes absent"))
            passed = bool(checks) and not any(item.status is Status.FAIL for item in checks + cleanup)
            retained = _retain_failure_logs(logs, output, int(indoor["logging"]["failure_log_max_bytes"])) if not passed else []
    report = {
        "schema_version": "echorescue-visual-search-smoke/1.0", "milestone": "v0.16.1",
        "case": case, "status": Status.PASS if passed else Status.FAIL,
        "mode": "graphical" if graphical else "headless",
        "configuration_digest": hashlib.sha256(config_path.read_bytes()).hexdigest(),
        "dependencies": indoor["dependencies"], "controller_mission": mission,
        "observer": observer, "perception": perception,
        "evaluator_only_survivor_truth": visual, "gazebo_evaluation": gazebo,
        "evaluator_only_coverage": coverage, "checks": [asdict(item) for item in checks],
        "process_exit_statuses": exits, "cleanup": [asdict(item) for item in cleanup], "logs": retained,
    }
    if output:
        output.parent.mkdir(parents=True, exist_ok=True); output.write_text(serialize_report(report))
    return report


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", choices=("success", "partial_occlusion", "distractor", "no_survivor", "camera_dropout", "session_change", "budget"))
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path); parser.add_argument("--timeout", type=float, default=340)
    parser.add_argument("--graphical", action="store_true")
    args = parser.parse_args(argv)
    report = smoke(args.config, args.output, args.timeout, args.graphical, args.scenario)
    sys.stdout.write(serialize_report(report)); return 0 if report["status"] == Status.PASS else 1


if __name__ == "__main__":
    raise SystemExit(main())
