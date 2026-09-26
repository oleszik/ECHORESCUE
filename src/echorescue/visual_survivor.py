"""Deterministic controlled-environment visual survivor evidence.

This module has no ROS, Gazebo, MAVLink, or evaluator dependency.  It detects a
versioned high-visibility marker from rendered RGB pixels, projects observations
with documented camera geometry, and accumulates same-session evidence.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from math import atan2, cos, hypot, isfinite, sin, sqrt
from typing import Iterable

import cv2  # type: ignore[import-not-found]
import numpy as np


SUPPORTED_ENCODINGS = ("rgb8", "bgr8")


@dataclass(frozen=True, slots=True)
class FramePolicy:
    width: int
    height: int
    maximum_image_age_ns: int
    maximum_pose_age_ns: int
    maximum_processing_latency_ns: int


@dataclass(frozen=True, slots=True)
class FrameMetadata:
    session_id: str
    pose_session_id: str
    sequence: int
    previous_sequence: int
    image_time_ns: int
    previous_image_time_ns: int
    receipt_monotonic_ns: int
    now_monotonic_ns: int
    pose_receipt_monotonic_ns: int
    processing_latency_ns: int
    width: int
    height: int
    encoding: str
    calibrated: bool


def validate_frame_metadata(value: FrameMetadata, policy: FramePolicy) -> None:
    """Validate ordering, freshness, calibration and same-session association."""
    if value.width != policy.width or value.height != policy.height:
        raise ValueError("invalid_dimensions")
    if value.encoding not in SUPPORTED_ENCODINGS:
        raise ValueError("unsupported_encoding")
    if not value.calibrated:
        raise ValueError("missing_calibration")
    if not value.session_id or value.session_id != value.pose_session_id:
        raise ValueError("cross_session_pose")
    if value.sequence <= value.previous_sequence or value.image_time_ns <= value.previous_image_time_ns:
        raise ValueError("out_of_order_frame")
    if value.now_monotonic_ns - value.receipt_monotonic_ns > policy.maximum_image_age_ns:
        raise ValueError("stale_frame")
    pose_age = value.receipt_monotonic_ns - value.pose_receipt_monotonic_ns
    if pose_age < 0 or pose_age > policy.maximum_pose_age_ns:
        raise ValueError("stale_vehicle_pose")
    if value.processing_latency_ns > policy.maximum_processing_latency_ns:
        raise ValueError("processing_latency_exceeded")


@dataclass(frozen=True, slots=True)
class DetectorPolicy:
    hue_min: int = 35
    hue_max: int = 85
    saturation_min: int = 150
    value_min: int = 100
    minimum_area_px: int = 80
    maximum_area_fraction: float = 0.25
    minimum_aspect_ratio: float = 0.55
    maximum_aspect_ratio: float = 1.8
    minimum_fill_ratio: float = 0.55
    morphology_kernel_px: int = 3
    minimum_confidence: float = 0.60

    def __post_init__(self) -> None:
        if not (0 <= self.hue_min <= self.hue_max <= 179):
            raise ValueError("invalid hue interval")
        if not (0 <= self.saturation_min <= 255 and 0 <= self.value_min <= 255):
            raise ValueError("invalid saturation/value threshold")
        if self.minimum_area_px < 1 or not 0 < self.maximum_area_fraction <= 1:
            raise ValueError("invalid area limits")
        if not 0 < self.minimum_aspect_ratio <= self.maximum_aspect_ratio:
            raise ValueError("invalid aspect-ratio limits")
        if not 0 < self.minimum_fill_ratio <= 1 or not 0 <= self.minimum_confidence <= 1:
            raise ValueError("invalid fill/confidence threshold")
        if self.morphology_kernel_px < 1 or self.morphology_kernel_px % 2 == 0:
            raise ValueError("morphology kernel must be positive and odd")


@dataclass(frozen=True, slots=True)
class VisualCandidate:
    x: int
    y: int
    width: int
    height: int
    centroid_x: float
    centroid_y: float
    area_px: int
    fill_ratio: float
    aspect_ratio: float
    color_score: float
    confidence: float

    def report(self) -> dict[str, object]:
        return asdict(self)


class MarkerDetector:
    """HSV/component detector for the controlled green rescue panel."""

    def __init__(self, policy: DetectorPolicy = DetectorPolicy()) -> None:
        self.policy = policy

    def detect(self, data: bytes, width: int, height: int, encoding: str) -> tuple[VisualCandidate, ...]:
        if width < 1 or height < 1:
            raise ValueError("invalid_dimensions")
        if encoding not in SUPPORTED_ENCODINGS:
            raise ValueError("unsupported_encoding")
        if len(data) != width * height * 3:
            raise ValueError("truncated_image_buffer")
        image = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
        rgb = image if encoding == "rgb8" else image[:, :, ::-1]
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        lower = np.array((self.policy.hue_min, self.policy.saturation_min, self.policy.value_min), dtype=np.uint8)
        upper = np.array((self.policy.hue_max, 255, 255), dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((self.policy.morphology_kernel_px,) * 2, dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        count, _, statistics, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        candidates: list[VisualCandidate] = []
        maximum_area = width * height * self.policy.maximum_area_fraction
        for component in range(1, count):
            x, y, box_width, box_height, area = (int(value) for value in statistics[component])
            if area < self.policy.minimum_area_px or area > maximum_area or box_height == 0:
                continue
            aspect = box_width / box_height
            fill = area / (box_width * box_height)
            if not self.policy.minimum_aspect_ratio <= aspect <= self.policy.maximum_aspect_ratio:
                continue
            if fill < self.policy.minimum_fill_ratio:
                continue
            pixels = hsv[y:y + box_height, x:x + box_width]
            selected = mask[y:y + box_height, x:x + box_width] > 0
            color_score = float(np.mean(pixels[:, :, 1][selected]) / 255.0)
            aspect_score = max(0.0, 1.0 - abs(1.0 - aspect) / max(1.0, self.policy.maximum_aspect_ratio - 1.0))
            confidence = min(1.0, 0.40 * fill + 0.35 * color_score + 0.25 * aspect_score)
            if confidence < self.policy.minimum_confidence:
                continue
            candidates.append(VisualCandidate(
                x, y, box_width, box_height, float(centroids[component][0]),
                float(centroids[component][1]), area, fill, aspect, color_score, confidence,
            ))
        return tuple(sorted(candidates, key=lambda item: (item.x, item.y, -item.area_px)))


@dataclass(frozen=True, slots=True)
class CameraCalibration:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    frame: str
    forward_m: float
    left_m: float
    up_m: float
    pitch_down_rad: float
    estimation_plane_up_m: float = 0.0

    def __post_init__(self) -> None:
        values = (self.fx, self.fy, self.cx, self.cy, self.forward_m, self.left_m,
                  self.up_m, self.pitch_down_rad, self.estimation_plane_up_m)
        if self.width < 1 or self.height < 1 or self.fx <= 0 or self.fy <= 0:
            raise ValueError("invalid_camera_calibration")
        if not self.frame or not all(isfinite(value) for value in values):
            raise ValueError("invalid_camera_calibration")


@dataclass(frozen=True, slots=True)
class VehiclePose:
    session_id: str
    sequence: int
    source_time_ns: int
    receipt_monotonic_ns: int
    east_m: float
    north_m: float
    up_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float


@dataclass(frozen=True, slots=True)
class LocalizedObservation:
    east_m: float
    north_m: float
    range_m: float
    bearing_rad: float
    uncertainty_radius_m: float


def project_to_plane(candidate: VisualCandidate, calibration: CameraCalibration,
                     pose: VehiclePose) -> LocalizedObservation:
    """Intersect the centroid ray with a horizontal ENU plane.

    Camera optical axes are x=right, y=down, z=forward.  The fixed mount has
    zero roll/yaw and a documented downward pitch from body-forward.
    """
    if not all(isfinite(value) for value in (candidate.centroid_x, candidate.centroid_y,
                                             pose.east_m, pose.north_m, pose.up_m,
                                             pose.roll_rad, pose.pitch_rad, pose.yaw_rad)):
        raise ValueError("non_finite_projection")
    right = (candidate.centroid_x - calibration.cx) / calibration.fx
    down = (candidate.centroid_y - calibration.cy) / calibration.fy
    # Optical ray -> body FLU, followed by fixed downward camera pitch.
    cp, sp = cos(calibration.pitch_down_rad), sin(calibration.pitch_down_rad)
    forward = cp - sp * down
    left = -right
    up = -sp - cp * down
    # Conservative: full vehicle roll/pitch are rejected rather than silently
    # approximated; accepted exploration imagery is near-level.
    if abs(pose.roll_rad) > 0.35 or abs(pose.pitch_rad) > 0.35:
        raise ValueError("vehicle_attitude_out_of_projection_contract")
    cy, sy = cos(pose.yaw_rad), sin(pose.yaw_rad)
    ray_east = cy * forward - sy * left
    ray_north = sy * forward + cy * left
    ray_up = up
    camera_east = pose.east_m + cy * calibration.forward_m - sy * calibration.left_m
    camera_north = pose.north_m + sy * calibration.forward_m + cy * calibration.left_m
    camera_up = pose.up_m + calibration.up_m
    if ray_up >= -1e-6:
        raise ValueError("ray_does_not_intersect_estimation_plane")
    scale = (calibration.estimation_plane_up_m - camera_up) / ray_up
    if not isfinite(scale) or scale <= 0:
        raise ValueError("ray_does_not_intersect_estimation_plane")
    east, north = camera_east + scale * ray_east, camera_north + scale * ray_north
    distance = hypot(east - pose.east_m, north - pose.north_m)
    bearing = atan2(north - pose.north_m, east - pose.east_m)
    uncertainty = 0.20 + 0.06 * distance + distance / max(calibration.fx, calibration.fy)
    if not all(isfinite(value) for value in (east, north, distance, bearing, uncertainty)):
        raise ValueError("non_finite_projection")
    return LocalizedObservation(east, north, distance, bearing, uncertainty)


@dataclass(frozen=True, slots=True)
class CandidateObservation:
    observation_id: str
    session_id: str
    frame_sequence: int
    image_time_ns: int
    receipt_monotonic_ns: int
    vehicle_time_ns: int
    vehicle_east_m: float
    vehicle_north_m: float
    candidate: VisualCandidate
    location: LocalizedObservation


@dataclass(slots=True)
class EvidenceCluster:
    east_m: float
    north_m: float
    uncertainty_radius_m: float
    observations: list[CandidateObservation] = field(default_factory=list)
    confirmed_id: str | None = None


@dataclass(frozen=True, slots=True)
class ConfirmationPolicy:
    minimum_observations: int = 2
    minimum_confidence: float = 0.65
    minimum_time_separation_ns: int = 300_000_000
    minimum_viewpoint_separation_m: float = 0.20
    clustering_radius_m: float = 0.80


class VisualEvidenceTracker:
    """Mission-local spatial evidence with v0.7-style multi-observation gating."""

    def __init__(self, policy: ConfirmationPolicy = ConfirmationPolicy()) -> None:
        self.policy = policy
        self.session_id = ""
        self.clusters: list[EvidenceCluster] = []
        self.confirmations: list[EvidenceCluster] = []
        self._observations: set[str] = set()

    def reset_session(self, session_id: str) -> None:
        if session_id != self.session_id:
            self.session_id = session_id
            self.clusters.clear()
            self.confirmations.clear()
            self._observations.clear()

    def observe(self, observation: CandidateObservation) -> EvidenceCluster:
        if observation.session_id != self.session_id:
            raise ValueError("cross_session_observation")
        if observation.observation_id in self._observations:
            raise ValueError("duplicate_frame")
        self._observations.add(observation.observation_id)
        compatible = [cluster for cluster in self.clusters if hypot(
            cluster.east_m - observation.location.east_m,
            cluster.north_m - observation.location.north_m,
        ) <= max(self.policy.clustering_radius_m, cluster.uncertainty_radius_m,
                 observation.location.uncertainty_radius_m)]
        cluster = min(compatible, key=lambda item: (hypot(
            item.east_m - observation.location.east_m,
            item.north_m - observation.location.north_m,
        ), item.east_m, item.north_m)) if compatible else EvidenceCluster(
            observation.location.east_m, observation.location.north_m,
            observation.location.uncertainty_radius_m,
        )
        if not compatible:
            self.clusters.append(cluster)
        if cluster.confirmed_id is not None:
            return cluster
        if any(item.frame_sequence == observation.frame_sequence for item in cluster.observations):
            raise ValueError("duplicate_frame")
        if cluster.observations and observation.image_time_ns <= cluster.observations[-1].image_time_ns:
            raise ValueError("out_of_order_evidence")
        cluster.observations.append(observation)
        weight = sum(item.candidate.confidence for item in cluster.observations)
        distinct = any(
            observation.image_time_ns - prior.image_time_ns >= self.policy.minimum_time_separation_ns
            or hypot(observation.vehicle_east_m - prior.vehicle_east_m,
                     observation.vehicle_north_m - prior.vehicle_north_m)
            >= self.policy.minimum_viewpoint_separation_m
            for prior in cluster.observations[:-1]
        )
        if (len(cluster.observations) >= self.policy.minimum_observations and distinct
                and weight / len(cluster.observations) >= self.policy.minimum_confidence):
            cluster.confirmed_id = f"survivor-{len(self.confirmations) + 1:03d}"
            self.confirmations.append(cluster)
        return cluster


@dataclass(frozen=True, slots=True)
class MatchResult:
    matches: tuple[tuple[int, int, float], ...]
    false_positives: int
    false_negatives: int
    precision: float | None
    recall: float | None


def match_estimates(estimates: Iterable[tuple[float, float]], truths: Iterable[tuple[float, float]],
                    tolerance_m: float) -> MatchResult:
    estimate_list, truth_list = list(estimates), list(truths)
    possibilities = sorted(
        (hypot(ex - tx, ey - ty), estimate, truth)
        for estimate, (ex, ey) in enumerate(estimate_list)
        for truth, (tx, ty) in enumerate(truth_list)
        if hypot(ex - tx, ey - ty) <= tolerance_m
    )
    used_estimates: set[int] = set()
    used_truths: set[int] = set()
    matches: list[tuple[int, int, float]] = []
    for distance, estimate, truth in possibilities:
        if estimate not in used_estimates and truth not in used_truths:
            used_estimates.add(estimate)
            used_truths.add(truth)
            matches.append((estimate, truth, distance))
    fp, fn = len(estimate_list) - len(matches), len(truth_list) - len(matches)
    precision = len(matches) / len(estimate_list) if estimate_list else None
    recall = len(matches) / len(truth_list) if truth_list else None
    return MatchResult(tuple(matches), fp, fn, precision, recall)
