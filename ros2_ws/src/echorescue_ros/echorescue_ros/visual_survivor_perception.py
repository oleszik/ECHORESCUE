"""Typed ROS camera perception node for controlled v0.16.1 survivor markers."""

from __future__ import annotations

import argparse
import json
from collections import Counter, deque
from math import radians
from pathlib import Path
from time import monotonic_ns

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from echorescue.visual_survivor import (
    CameraCalibration, CandidateObservation, ConfirmationPolicy, DetectorPolicy,
    MarkerDetector, VehiclePose, VisualEvidenceTracker, project_to_plane,
)
from echorescue_interfaces.msg import (
    ConfirmedSurvivor, EchoRescueVehicleState3D, PerceptionMissionSummary,
    PerceptionStatus, SurvivorEvidenceUpdate, VisualCandidateObservation,
)
from echorescue_ros.qos import IMAGE_QOS, MISSION_QOS, STATUS_QOS, TELEMETRY_QOS


def _stamp_ns(message: Image) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class VisualSurvivorPerception(Node):
    def __init__(self, config: Path, output: Path | None = None) -> None:
        super().__init__("echorescue_visual_survivor_perception")
        self.raw = json.loads(config.read_text(encoding="utf-8"))
        camera, detector, evidence = self.raw["camera"], self.raw["detector"], self.raw["evidence"]
        self.calibration = CameraCalibration(
            camera["width"], camera["height"], camera["fx"], camera["fy"],
            camera["cx"], camera["cy"], camera["frame"],
            camera["extrinsics_flu_m"][0], camera["extrinsics_flu_m"][1],
            camera["extrinsics_flu_m"][2], camera["pitch_down_rad"],
            camera["estimation_plane_up_m"],
        )
        self.detector = MarkerDetector(DetectorPolicy(**detector))
        self.tracker = VisualEvidenceTracker(ConfirmationPolicy(**evidence))
        self.output = output
        self.mission_id = self.raw["mission_id"]
        self.poses: deque[VehiclePose] = deque(maxlen=32)
        self.calibration_ready = False
        self.pending: tuple[Image, int] | None = None
        self.last_image_time_ns = -1
        self.last_processed_ns = 0
        self.last_frame_receipt_ns = 0
        self.camera_dropout_observed = False
        self.frame_sequence = 0
        self.counts: Counter[str] = Counter()
        self.rejections: Counter[str] = Counter()
        self.confirmed_ids: list[str] = []
        self.candidate_publisher = self.create_publisher(
            VisualCandidateObservation, "/echorescue/perception/candidate", MISSION_QOS)
        self.evidence_publisher = self.create_publisher(
            SurvivorEvidenceUpdate, "/echorescue/perception/evidence", MISSION_QOS)
        self.confirmed_publisher = self.create_publisher(
            ConfirmedSurvivor, "/echorescue/perception/confirmed", MISSION_QOS)
        self.status_publisher = self.create_publisher(
            PerceptionStatus, "/echorescue/perception/status", STATUS_QOS)
        self.summary_publisher = self.create_publisher(
            PerceptionMissionSummary, "/echorescue/perception/summary", STATUS_QOS)
        self.create_subscription(EchoRescueVehicleState3D, "/echorescue/vehicle/state_3d", self._pose, TELEMETRY_QOS)
        self.create_subscription(CameraInfo, "/echorescue/camera/camera_info", self._camera_info, IMAGE_QOS)
        self.create_subscription(Image, "/echorescue/camera/image_raw", self._image, IMAGE_QOS)
        self.create_timer(1.0 / float(camera["maximum_processing_rate_hz"]), self._process)
        self.create_timer(0.5, self._status)

    def _reject(self, reason: str) -> None:
        self.counts["rejected"] += 1
        self.rejections[reason] += 1

    def _pose(self, message: EchoRescueVehicleState3D) -> None:
        if not message.position_valid or not (message.has_attitude or message.has_heading) or not message.session_id:
            return
        pose = VehiclePose(
            message.session_id, int(message.sequence),
            int(message.source_time_boot_ms) * 1_000_000,
            int(message.receipt_monotonic_ns), float(message.x_m), float(message.y_m),
            float(message.z_m), float(message.roll_rad) if message.has_attitude else 0.0,
            float(message.pitch_rad) if message.has_attitude else 0.0,
            float(message.yaw_rad) if message.has_attitude else radians(float(message.heading_deg)),
        )
        if self.tracker.session_id and pose.session_id != self.tracker.session_id:
            self.rejections["session_reset"] += 1
        self.tracker.reset_session(pose.session_id)
        self.poses.append(pose)

    def _camera_info(self, message: CameraInfo) -> None:
        expected = self.calibration
        self.calibration_ready = (
            message.width == expected.width and message.height == expected.height
            and message.header.frame_id == expected.frame
            and len(message.k) == 9 and message.k[0] > 0 and message.k[4] > 0
            and abs(message.k[0] - expected.fx) < 1e-3
            and abs(message.k[4] - expected.fy) < 1e-3
        )
        if not self.calibration_ready:
            self.rejections["invalid_calibration"] += 1

    def _image(self, message: Image) -> None:
        self.counts["received"] += 1
        receipt = monotonic_ns()
        self.last_frame_receipt_ns = receipt
        if self.pending is not None:
            self.counts["dropped"] += 1
        self.pending = message, receipt

    def _process(self) -> None:
        if self.pending is None:
            return
        message, receipt = self.pending
        self.pending = None
        image_time = _stamp_ns(message)
        if not self.calibration_ready:
            self._reject("missing_calibration")
            return
        if message.width != self.calibration.width or message.height != self.calibration.height:
            self._reject("invalid_dimensions")
            return
        if image_time <= self.last_image_time_ns:
            self._reject("out_of_order_frame")
            return
        if monotonic_ns() - receipt > int(float(self.raw["camera"]["maximum_image_age_s"]) * 1e9):
            self._reject("stale_frame")
            return
        if not self.poses:
            self._reject("missing_vehicle_pose")
            return
        pose = self.poses[-1]
        pose_age = receipt - pose.receipt_monotonic_ns
        if pose_age < 0 or pose_age > int(float(self.raw["camera"]["maximum_pose_age_s"]) * 1e9):
            self._reject("stale_vehicle_pose")
            return
        if pose.session_id != self.tracker.session_id:
            self._reject("cross_session_pose")
            return
        started = monotonic_ns()
        try:
            candidates = self.detector.detect(bytes(message.data), message.width, message.height, message.encoding)
        except ValueError as error:
            self._reject(str(error))
            return
        if monotonic_ns() - started > int(float(self.raw["camera"]["maximum_processing_latency_s"]) * 1e9):
            self._reject("processing_latency_exceeded")
            return
        self.last_image_time_ns = image_time
        self.frame_sequence += 1
        self.counts["accepted"] += 1
        for index, candidate in enumerate(candidates, 1):
            try:
                location = project_to_plane(candidate, self.calibration, pose)
                bounds = self.raw["exploration_bounds_enu_m"]
                if not (bounds[0] <= location.east_m <= bounds[1]
                        and bounds[2] <= location.north_m <= bounds[3]):
                    raise ValueError("estimate_outside_exploration_bounds")
                observation = CandidateObservation(
                    f"obs-{self.frame_sequence:06d}-{index:02d}", pose.session_id,
                    self.frame_sequence, image_time, receipt, pose.source_time_ns,
                    pose.east_m, pose.north_m, candidate, location,
                )
                cluster = self.tracker.observe(observation)
            except ValueError as error:
                self._reject(str(error))
                continue
            self.counts["candidates"] += 1
            self._publish_candidate(observation)
            self._publish_evidence(observation, cluster)
            if cluster.confirmed_id and cluster.confirmed_id not in self.confirmed_ids:
                self.confirmed_ids.append(cluster.confirmed_id)
                self._publish_confirmation(cluster)

    def _publish_candidate(self, observation: CandidateObservation) -> None:
        message = VisualCandidateObservation()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.calibration.frame
        message.schema_version = "echorescue-visual-candidate/1.0"
        message.mission_id, message.session_id = self.mission_id, observation.session_id
        message.observation_id, message.frame_sequence = observation.observation_id, observation.frame_sequence
        message.image_time_ns, message.receipt_monotonic_ns = observation.image_time_ns, observation.receipt_monotonic_ns
        message.vehicle_time_ns, message.camera_frame = observation.vehicle_time_ns, self.calibration.frame
        message.vehicle_pose_age_ns = max(0, observation.receipt_monotonic_ns - self.poses[-1].receipt_monotonic_ns)
        candidate, location = observation.candidate, observation.location
        message.bbox_x, message.bbox_y = candidate.x, candidate.y
        message.bbox_width, message.bbox_height = candidate.width, candidate.height
        message.centroid_x, message.centroid_y = candidate.centroid_x, candidate.centroid_y
        message.confidence, message.fill_ratio = candidate.confidence, candidate.fill_ratio
        message.aspect_ratio, message.color_score = candidate.aspect_ratio, candidate.color_score
        message.bearing_rad, message.estimated_range_m = location.bearing_rad, location.range_m
        message.east_m, message.north_m = location.east_m, location.north_m
        message.up_m, message.uncertainty_radius_m = self.calibration.estimation_plane_up_m, location.uncertainty_radius_m
        message.vehicle_east_m, message.vehicle_north_m = observation.vehicle_east_m, observation.vehicle_north_m
        message.vehicle_up_m, message.vehicle_yaw_rad = self.poses[-1].up_m, self.poses[-1].yaw_rad
        message.rejection_reason = ""
        self.candidate_publisher.publish(message)

    def _publish_evidence(self, observation: CandidateObservation, cluster: object) -> None:
        from echorescue.visual_survivor import EvidenceCluster
        assert isinstance(cluster, EvidenceCluster)
        message = SurvivorEvidenceUpdate()
        message.header.stamp = self.get_clock().now().to_msg()
        message.schema_version = "echorescue-survivor-evidence/1.0"
        message.mission_id, message.session_id = self.mission_id, observation.session_id
        message.observation_id = observation.observation_id
        message.cluster_id = cluster.confirmed_id or f"candidate-{self.tracker.clusters.index(cluster) + 1:03d}"
        message.supporting_observation_count = len(cluster.observations)
        message.accumulated_confidence = sum(item.candidate.confidence for item in cluster.observations)
        message.confirmation_state = "confirmed" if cluster.confirmed_id else "unconfirmed"
        message.first_seen_ns = cluster.observations[0].image_time_ns
        message.last_seen_ns = cluster.observations[-1].image_time_ns
        message.east_m, message.north_m = cluster.east_m, cluster.north_m
        message.uncertainty_radius_m = cluster.uncertainty_radius_m
        message.reason = "multi_observation_gate_satisfied" if cluster.confirmed_id else "awaiting_independent_observation"
        self.evidence_publisher.publish(message)

    def _publish_confirmation(self, cluster: object) -> None:
        from echorescue.visual_survivor import EvidenceCluster
        assert isinstance(cluster, EvidenceCluster) and cluster.confirmed_id
        message = ConfirmedSurvivor()
        message.header.stamp = self.get_clock().now().to_msg()
        message.schema_version = "echorescue-confirmed-survivor/1.0"
        message.mission_id, message.session_id = self.mission_id, self.tracker.session_id
        message.survivor_id = cluster.confirmed_id
        message.supporting_observation_count = len(cluster.observations)
        message.first_seen_ns = cluster.observations[0].image_time_ns
        message.confirmed_ns = cluster.observations[-1].image_time_ns
        message.east_m, message.north_m = cluster.east_m, cluster.north_m
        message.up_m, message.uncertainty_radius_m = self.calibration.estimation_plane_up_m, cluster.uncertainty_radius_m
        message.confidence = sum(item.candidate.confidence for item in cluster.observations) / len(cluster.observations)
        self.confirmed_publisher.publish(message)

    def _status(self) -> None:
        message = PerceptionStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.schema_version = "echorescue-perception-status/1.0"
        message.mission_id, message.session_id = self.mission_id, self.tracker.session_id
        message.ready = self.calibration_ready and bool(self.poses)
        maximum_gap = int(float(self.raw["camera"]["dropout_timeout_s"]) * 1e9)
        message.camera_fresh = bool(self.last_frame_receipt_ns) and monotonic_ns() - self.last_frame_receipt_ns <= maximum_gap
        self.camera_dropout_observed |= bool(self.last_frame_receipt_ns) and not message.camera_fresh
        message.received_frame_count, message.accepted_frame_count = self.counts["received"], self.counts["accepted"]
        message.dropped_frame_count, message.rejected_frame_count = self.counts["dropped"], self.counts["rejected"]
        message.candidate_count, message.confirmed_survivor_count = self.counts["candidates"], len(self.confirmed_ids)
        message.state = "ready" if message.ready else "initializing"
        message.reason = "" if message.ready else "awaiting_calibration_and_pose"
        self.status_publisher.publish(message)
        self._publish_summary("running")

    def _publish_summary(self, outcome: str) -> None:
        message = PerceptionMissionSummary()
        message.header.stamp = self.get_clock().now().to_msg()
        message.schema_version = "echorescue-perception-summary/1.0"
        message.mission_id, message.session_id = self.mission_id, self.tracker.session_id
        message.outcome = outcome
        message.survivor_prior_unavailable = True
        message.received_frame_count, message.accepted_frame_count = self.counts["received"], self.counts["accepted"]
        message.dropped_frame_count, message.rejected_frame_count = self.counts["dropped"], self.counts["rejected"]
        message.candidate_count, message.confirmed_survivor_count = self.counts["candidates"], len(self.confirmed_ids)
        message.survivor_ids = self.confirmed_ids
        message.rejection_reasons = [f"{reason}:{count}" for reason, count in sorted(self.rejections.items())]
        self.summary_publisher.publish(message)

    def summary(self, outcome: str) -> dict[str, object]:
        return {
            "schema_version": "echorescue-visual-perception-report/1.0",
            "milestone": "v0.16.1", "mission_id": self.mission_id,
            "session_id": self.tracker.session_id, "outcome": outcome,
            "survivor_prior_unavailable": True, "counts": dict(self.counts),
            "camera_dropout_observed": self.camera_dropout_observed,
            "rejection_reasons": dict(self.rejections),
            "confirmed_survivor_ids": self.confirmed_ids,
            "clusters": [{
                "survivor_id": cluster.confirmed_id,
                "east_m": cluster.east_m, "north_m": cluster.north_m,
                "uncertainty_radius_m": cluster.uncertainty_radius_m,
                "observations": [item.observation_id for item in cluster.observations],
            } for cluster in self.tracker.clusters],
        }

    def close(self) -> None:
        if rclpy.ok():
            self._publish_summary("complete")
        report = self.summary("complete")
        if self.output:
            self.output.parent.mkdir(parents=True, exist_ok=True)
            self.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = VisualSurvivorPerception(parsed.config, parsed.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
