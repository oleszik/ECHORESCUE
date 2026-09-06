"""Machine-readable report and dashboard-compatible replay helpers."""

import json
from pathlib import Path
from typing import cast

from echorescue.closed_loop import ClosedLoopMission
from echorescue.replay import REPLAY_SCHEMA_VERSION


def build_closed_loop_replay(mission: ClosedLoopMission) -> dict[str, object]:
    frames = []
    for source in mission.frames:
        position = cast(list[int], source["position"])
        occupancy = cast(list[str], source["occupancy"])
        confirmed = cast(list[list[int]], source["confirmed_survivors"])
        energy = cast(float, source["energy_remaining"])
        state = cast(str, source["state"])
        returning = cast(bool, source["returning"])
        capacity = mission.config.battery_capacity
        drone = {
            "position": position,
            "state": state,
            "energy_remaining": energy,
            "energy_remaining_percent": (
                round(100.0 * energy / capacity, 3) if capacity is not None else 0.0
            ),
            "target": None,
            "planned_path": [],
            "path_kind": "return" if returning else "frontier",
            "relay": {"active": False, "strategy": "off", "position": None, "scout_id": None, "link_achieved": False, "role_steps": 0, "holding_for_relay": False},
            "yielding": False,
            "motion_intent": None,
            "communication": {"connected_to_base": True, "direct_to_base": True, "via_relay": False, "relay_path": [mission.agent_id, "base"]},
            "knowledge": {"known_coverage": 0.0, "stale_cells": 0, "average_data_age": 0.0, "oldest_data_age": 0, "detected_survivors": len(confirmed), "confirmed_survivors": len(confirmed)},
        }
        known = sum(character != "?" for row in occupancy for character in row)
        coverage = round(100.0 * known / (mission.config.width * mission.config.height), 3)
        knowledge = {
            "occupancy": occupancy,
            "known_coverage": coverage,
            "differences_from_shadow": [],
            "confirmed_survivors": confirmed,
            "purpose": "local_decision_knowledge",
        }
        frames.append(
            {
                "step": source["step"],
                "drones": {mission.agent_id: drone},
                "occupancy": occupancy,
                "knowledge_maps": {"operator": knowledge, mission.agent_id: knowledge, "base": knowledge},
                "shadow_knowledge": {"shared_coverage": coverage, "map_divergence_between_drones": 0.0},
                "confirmed_survivors": confirmed,
                "events": [],
                "communication": {
                    "base_station": {"id": "base", "position": [mission.config.base.x, mission.config.base.y]},
                    "nodes": {"base": [mission.config.base.x, mission.config.base.y], mission.agent_id: position},
                    "links": [{"from": mission.agent_id, "to": "base", "kind": "ros2"}],
                },
                "explored_percent": coverage,
                "bridge": source["adapter"],
            }
        )
    report = mission.report()
    return {
        "schema_version": REPLAY_SCHEMA_VERSION,
        "mission": {
            "seed": None,
            "knowledge_mode": "ros2-local",
            "relay_strategy": "off",
            "configuration": {
                "width": mission.config.width,
                "height": mission.config.height,
                "drone_count": 1,
                "floors": 1,
                "transport": "ros2",
                "session_id": mission.session_id,
            },
        },
        "map": {
            "width": mission.config.width,
            "height": mission.config.height,
            "base": [mission.config.base.x, mission.config.base.y],
            "cell_encoding": {"?": "unknown", ".": "free", "#": "occupied"},
            "initial_known_occupancy": frames[0]["occupancy"] if frames else [],
        },
        "frames": frames,
        "metrics": {**report, "mission_events": []},
    }


def write_json(payload: dict[str, object], output: str | Path) -> Path:
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path
