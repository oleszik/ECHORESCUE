"""Versioned v0.15.2 multi-obstacle owned-stack scenarios."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import re
import sys
import tempfile
from typing import Any, Mapping, Sequence

from echorescue.sensor_replanning_integration import smoke
from echorescue.sim_integration import REPOSITORY_ROOT, Status, serialize_report


DEFAULT_SCENARIOS = REPOSITORY_ROOT / "config/multi-obstacle-scenarios-v0.15.2.json"


def load_scenarios(path: Path = DEFAULT_SCENARIOS) -> dict[str, Any]:
    raw: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
    if raw.get("schema_version") != "echorescue-multi-obstacle-scenarios/1.0":
        raise ValueError("unsupported v0.15.2 scenario schema")
    return raw


def _obstacle_model(item: Mapping[str, Any]) -> str:
    name = str(item["name"])
    center = " ".join(str(value) for value in item["center"])
    size = " ".join(str(value) for value in item["size"])
    topic = f"/echorescue/indoor/contacts/{name}"
    return f'''    <model name="{name}"><static>true</static><pose>{center} 0 0 0</pose><link name="link">
      <collision name="collision"><geometry><box><size>{size}</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>{size}</size></box></geometry><material><ambient>0.65 0.12 0.12 1</ambient><diffuse>0.9 0.18 0.18 1</diffuse></material></visual>
      <sensor name="contact" type="contact"><always_on>true</always_on><update_rate>50</update_rate><contact><collision>collision</collision><topic>{topic}</topic></contact></sensor>
    </link></model>
'''


def _world(scenario: Mapping[str, Any]) -> str:
    source = (REPOSITORY_ROOT / "worlds/echorescue_indoor_v0_15_1.sdf").read_text(encoding="utf-8")
    source = source.replace('world name="echorescue_indoor_v0_15_1"', f'world name="{scenario["world_name"]}"')
    source = re.sub(r'\n    <model name="unknown_obstacle">.*?</link></model>\n', "\n", source, flags=re.DOTALL)
    models = "\n".join(_obstacle_model(item) for item in scenario["obstacles"])
    return source.replace("\n    <include>\n", f"\n{models}\n    <include>\n")


def _generated_configuration(raw: Mapping[str, Any], scenario_name: str, temporary: Path) -> Path:
    scenario = raw["scenarios"][scenario_name]
    planner: dict[str, Any] = json.loads((REPOSITORY_ROOT / str(raw["planner_config"])).read_text(encoding="utf-8"))
    indoor: dict[str, Any] = json.loads((REPOSITORY_ROOT / str(planner["base_indoor_config"])).read_text(encoding="utf-8"))
    world_path = temporary / f"{scenario['world_name']}.sdf"
    world_path.write_text(_world(scenario), encoding="utf-8")
    indoor["milestone"] = "v0.15.2"
    # LAND recovery may intentionally finish away from launch. Floor contact
    # below the configured landed-height remains allowed throughout the bounded
    # indoor flight volume; walls and obstacles remain prohibited.
    indoor["safety"]["launch_ground_contact_radius_m"] = 10.0
    indoor["world"].update({"version": f"{scenario['world_name']}-v1", "name": scenario["world_name"], "sdf_path": str(world_path)})
    indoor["evaluation"]["pose_topic"] = f"/world/{scenario['world_name']}/pose/info"
    indoor["geometry"]["entities"] = [item for item in indoor["geometry"]["entities"] if not item["name"].startswith("unknown_")]
    for obstacle in scenario["obstacles"]:
        item = copy.deepcopy(obstacle)
        item.update({"classification": "obstacle", "contact_topic": f"/echorescue/indoor/contacts/{item['name']}"})
        indoor["geometry"]["entities"].append(item)
    indoor_path = temporary / "indoor.json"
    indoor_path.write_text(json.dumps(indoor), encoding="utf-8")
    planner["base_indoor_config"] = str(indoor_path)
    planner["sensor_replanning"].update(raw["policy"])
    planner["sensor_replanning"]["generation_target_ids"] = True
    planner["sensor_replanning"]["maximum_replan_attempts"] = scenario["maximum_replans_per_mission"]
    planner_path = temporary / "planner.json"
    planner_path.write_text(json.dumps(planner), encoding="utf-8")
    return planner_path


def run_scenario(path: Path, scenario_name: str, output: Path | None, timeout_s: float,
                 graphical: bool) -> dict[str, Any]:
    raw = load_scenarios(path)
    with tempfile.TemporaryDirectory(prefix="echorescue-v0152-config-") as name:
        planner = _generated_configuration(raw, scenario_name, Path(name))
        report = smoke(planner, None, timeout_s, graphical, case=scenario_name)
    gazebo = report.get("gazebo_evaluation", {})
    trajectory = gazebo.get("trajectory", [])
    final_pose = trajectory[-1] if trajectory else None
    evaluator_landed = bool(final_pose and float(final_pose["up_m"]) <= 0.35)
    gazebo["final_position_world_enu"] = final_pose
    gazebo["final_landed_height_observed"] = evaluator_landed
    report["checks"].append({
        "name": "gazebo.final_landed_height", "status": "PASS" if evaluator_landed else "FAIL",
        "version": None, "detail": f"final evaluator height={final_pose.get('up_m') if final_pose else None}",
    })
    if not evaluator_landed:
        report["status"] = Status.FAIL
    report.update({
        "schema_version": "echorescue-multi-obstacle-smoke/2.0", "milestone": "v0.15.2",
        "case": scenario_name, "scenario_schema_version": raw["schema_version"],
        "scenario_configuration": raw["scenarios"][scenario_name],
        "replanning_policy": raw["policy"],
        "source_revisions": report.get("dependencies", {}),
    })
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(serialize_report(report), encoding="utf-8")
    return report


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenarios", type=Path, default=DEFAULT_SCENARIOS)
    parser.add_argument("--output", type=Path)
    parser.add_argument("scenario", choices=("multi", "budget", "later_unreachable"))
    parser.add_argument("--timeout", type=float, default=300)
    parser.add_argument("--graphical", action="store_true")
    args = parser.parse_args(argv)
    report = run_scenario(args.scenarios, args.scenario, args.output, args.timeout, args.graphical)
    sys.stdout.write(serialize_report(report))
    return 0 if report["status"] == Status.PASS else 1


if __name__ == "__main__":
    raise SystemExit(main())
