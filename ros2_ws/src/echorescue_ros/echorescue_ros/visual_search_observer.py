"""Independent typed-ROS observer for v0.16.1 visual search."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from time import monotonic

import rclpy
from rclpy.executors import ExternalShutdownException

from echorescue_interfaces.msg import (
    ConfirmedSurvivor, PerceptionStatus, SurvivorEvidenceUpdate,
    VisualCandidateObservation,
)
from echorescue_ros.frontier_exploration_observer import FrontierExplorationObserver
from echorescue_ros.qos import MISSION_QOS, STATUS_QOS


class VisualSearchObserver(FrontierExplorationObserver):
    def __init__(self, output: Path, timeout_s: float, expect_success: bool) -> None:
        self.perception_ready = False
        self.received_frames = self.accepted_frames = self.candidate_count = 0
        self.candidates: list[dict[str, object]] = []
        self.evidence: list[dict[str, object]] = []
        self.confirmations: dict[str, dict[str, object]] = {}
        super().__init__(output, timeout_s, expect_success)
        self.create_subscription(PerceptionStatus, "/echorescue/perception/status", self._perception, STATUS_QOS)
        self.create_subscription(VisualCandidateObservation, "/echorescue/perception/candidate", self._candidate, MISSION_QOS)
        self.create_subscription(SurvivorEvidenceUpdate, "/echorescue/perception/evidence", self._evidence, MISSION_QOS)
        self.create_subscription(ConfirmedSurvivor, "/echorescue/perception/confirmed", self._confirmed, MISSION_QOS)

    def _session(self, value: str) -> None:
        previous = getattr(self, "session_id", "")
        super()._session(value)
        if previous and value and value != previous:
            self.candidates.clear()
            self.evidence.clear()
            self.confirmations.clear()

    def _perception(self, message: PerceptionStatus) -> None:
        self._session(message.session_id)
        self.perception_ready |= bool(message.ready)
        self.received_frames = max(self.received_frames, int(message.received_frame_count))
        self.accepted_frames = max(self.accepted_frames, int(message.accepted_frame_count))
        self.candidate_count = max(self.candidate_count, int(message.candidate_count))

    def _candidate(self, message: VisualCandidateObservation) -> None:
        self._session(message.session_id)
        self.candidates.append({
            "observation_id": message.observation_id,
            "frame_sequence": int(message.frame_sequence),
            "image_time_ns": int(message.image_time_ns),
            "vehicle_time_ns": int(message.vehicle_time_ns),
            "vehicle_pose_age_ns": int(message.vehicle_pose_age_ns),
            "confidence": float(message.confidence),
            "east_m": float(message.east_m), "north_m": float(message.north_m),
            "uncertainty_radius_m": float(message.uncertainty_radius_m),
            "vehicle_east_m": float(message.vehicle_east_m),
            "vehicle_north_m": float(message.vehicle_north_m),
            "vehicle_up_m": float(message.vehicle_up_m),
            "vehicle_yaw_rad": float(message.vehicle_yaw_rad),
        })

    def _evidence(self, message: SurvivorEvidenceUpdate) -> None:
        self._session(message.session_id)
        self.evidence.append({
            "observation_id": message.observation_id, "cluster_id": message.cluster_id,
            "supporting_observation_count": int(message.supporting_observation_count),
            "confirmation_state": message.confirmation_state,
            "first_seen_ns": int(message.first_seen_ns), "last_seen_ns": int(message.last_seen_ns),
        })

    def _confirmed(self, message: ConfirmedSurvivor) -> None:
        self._session(message.session_id)
        self.confirmations.setdefault(message.survivor_id, {
            "survivor_id": message.survivor_id,
            "supporting_observation_count": int(message.supporting_observation_count),
            "first_seen_ns": int(message.first_seen_ns), "confirmed_ns": int(message.confirmed_ns),
            "east_m": float(message.east_m), "north_m": float(message.north_m),
            "uncertainty_radius_m": float(message.uncertainty_radius_m),
            "confidence": float(message.confidence),
        })

    def _evaluate(self) -> None:
        land_chain = self.land if self.expect_success else True
        flight = self.guided and self.armed and land_chain and self.landed and self.disarmed
        exploration = self.maximum_map_revision > 0 and bool(self.targets)
        outcome = self.completion_observed if self.expect_success else self.recovery_observed
        valid_confirmations = all(
            item["supporting_observation_count"] >= 2
            and item["confirmed_ns"] > item["first_seen_ns"]
            for item in self.confirmations.values()
        )
        perception = self.perception_ready and self.accepted_frames > 0 and valid_confirmations
        if flight and exploration and outcome and perception:
            self._finish(True, "typed exploration, perception, LAND, ON_GROUND and disarm chain complete")
        elif monotonic() - self.started > self.timeout_s:
            self._finish(False, "visual search observer timed out")

    def _finish(self, passed: bool, detail: str) -> None:
        report = {
            "schema_version": "echorescue-visual-search-observer/1.0",
            "milestone": "v0.16.1", "status": "PASS" if passed else "FAIL",
            "detail": detail, "session_id": self.session_id, "sessions": self.sessions,
            "guided_observed": self.guided, "armed_observed": self.armed,
            "land_observed": self.land, "on_ground_observed": self.landed,
            "disarmed_observed": self.disarmed, "perception_ready": self.perception_ready,
            "received_frame_count": self.received_frames,
            "accepted_frame_count": self.accepted_frames,
            "candidate_count": self.candidate_count,
            "candidates": self.candidates, "evidence": self.evidence,
            "confirmed_survivors": list(self.confirmations.values()),
            "exploration_events": self.exploration_events, "target_ids": self.targets,
            "route_generations": sorted(self.route_generations),
            "maximum_map_revision": self.maximum_map_revision,
            "exploration_complete_observed": self.completion_observed,
            "return_observed": self.return_observed, "recovery_observed": self.recovery_observed,
        }
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.passed, self.finished = passed, True


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=320)
    parser.add_argument("--expect-failure", action="store_true")
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = VisualSearchObserver(parsed.output, parsed.timeout, not parsed.expect_failure)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
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
