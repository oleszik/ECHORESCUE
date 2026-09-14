"""Independent typed-ROS observer for v0.16.0 frontier exploration."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from time import monotonic

import rclpy
from rclpy.node import Node

from echorescue_interfaces.msg import (
    EchoRescueVehicleState3D, ExplorationState, MavlinkVehicleState,
    WaypointMissionEvent, WaypointTarget,
)
from echorescue_ros.qos import MISSION_QOS, TELEMETRY_QOS


class FrontierExplorationObserver(Node):
    def __init__(self, output: Path, timeout_s: float, expect_success: bool) -> None:
        super().__init__("echorescue_frontier_exploration_observer")
        self.output, self.timeout_s, self.expect_success = output, timeout_s, expect_success
        self.started = monotonic()
        self.finished = self.passed = False
        self.session_id = ""
        self.sessions: list[str] = []
        self.guided = self.armed = self.land = self.landed = self.disarmed = False
        self.events: list[str] = []
        self.exploration_events: list[dict[str, object]] = []
        self.targets: list[str] = []
        self.route_generations: set[int] = set()
        self.maximum_map_revision = 0
        self.maximum_frontier_clusters = 0
        self.return_observed = False
        self.completion_observed = False
        self.recovery_observed = False
        self.create_subscription(MavlinkVehicleState, "/echorescue/mavlink/vehicle_state", self._vehicle, TELEMETRY_QOS)
        self.create_subscription(EchoRescueVehicleState3D, "/echorescue/vehicle/state_3d", self._state, TELEMETRY_QOS)
        self.create_subscription(ExplorationState, "/echorescue/exploration/state", self._exploration, MISSION_QOS)
        self.create_subscription(WaypointMissionEvent, "/echorescue/mission/event", self._event, MISSION_QOS)
        self.create_subscription(WaypointTarget, "/echorescue/mission/active_target", self._target, MISSION_QOS)
        self.create_timer(0.1, self._evaluate)

    def _session(self, value: str) -> None:
        if value and value not in self.sessions:
            self.sessions.append(value)
        if value:
            self.session_id = value

    def _vehicle(self, message: MavlinkVehicleState) -> None:
        self._session(message.session_id)
        self.guided |= message.flight_mode == "GUIDED"
        self.armed |= bool(message.armed)
        self.land |= self.armed and message.flight_mode == "LAND"
        self.disarmed |= self.armed and not message.armed

    def _state(self, message: EchoRescueVehicleState3D) -> None:
        self._session(message.session_id)
        self.landed |= self.armed and message.has_landed_state and message.landed

    def _exploration(self, message: ExplorationState) -> None:
        self._session(message.session_id)
        self.maximum_map_revision = max(self.maximum_map_revision, int(message.map_revision))
        self.maximum_frontier_clusters = max(self.maximum_frontier_clusters, int(message.frontier_cluster_count))
        self.route_generations.add(int(message.route_generation))
        self.completion_observed |= message.event == "exploration_complete"
        self.recovery_observed |= message.event in {"recovery", "budget_exhausted"}
        self.return_observed |= message.state == "returning" or "return" in message.reason
        self.exploration_events.append({
            "sequence": int(message.sequence), "state": message.state,
            "event": message.event, "reason": message.reason,
            "map_revision": int(message.map_revision),
            "route_generation": int(message.route_generation),
            "active_target_id": message.active_target_id,
        })

    def _event(self, message: WaypointMissionEvent) -> None:
        self._session(message.session_id)
        self.events.append(message.event)
        self.land |= message.phase == "landing" or "LAND" in message.detail
        self.recovery_observed |= "recovery" in message.detail.lower()

    def _target(self, message: WaypointTarget) -> None:
        self._session(message.session_id)
        if message.target_id not in self.targets:
            self.targets.append(message.target_id)
        self.return_observed |= "return" in message.target_id

    def _evaluate(self) -> None:
        land_chain = self.land if self.expect_success else True
        flight = self.guided and self.armed and land_chain and self.landed and self.disarmed
        evidence = self.maximum_map_revision > 0 and bool(self.targets)
        outcome = self.completion_observed if self.expect_success else self.recovery_observed
        if flight and evidence and outcome:
            self._finish(True, "independent exploration, target, LAND, ON_GROUND and disarm chain complete")
        elif monotonic() - self.started > self.timeout_s:
            self._finish(False, "observer timed out")

    def _finish(self, passed: bool, detail: str) -> None:
        report = {
            "schema_version": "echorescue-frontier-exploration-observer/1.0",
            "milestone": "v0.16.0", "status": "PASS" if passed else "FAIL",
            "detail": detail, "session_id": self.session_id, "sessions": self.sessions,
            "guided_observed": self.guided, "armed_observed": self.armed,
            "land_observed": self.land, "on_ground_observed": self.landed,
            "disarmed_observed": self.disarmed, "events": self.events,
            "exploration_events": self.exploration_events,
            "target_ids": self.targets,
            "route_generations": sorted(self.route_generations),
            "maximum_map_revision": self.maximum_map_revision,
            "maximum_frontier_clusters": self.maximum_frontier_clusters,
            "exploration_complete_observed": self.completion_observed,
            "return_observed": self.return_observed,
            "recovery_observed": self.recovery_observed,
        }
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.passed, self.finished = passed, True


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=300)
    parser.add_argument("--expect-failure", action="store_true")
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = FrontierExplorationObserver(parsed.output, parsed.timeout, not parsed.expect_failure)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if not node.finished:
            node._finish(False, "observer interrupted")
    finally:
        passed = node.passed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
