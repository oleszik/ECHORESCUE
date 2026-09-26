import unittest
from math import radians

import cv2
import numpy as np

from echorescue.visual_survivor import (
    CameraCalibration, CandidateObservation, ConfirmationPolicy, DetectorPolicy,
    FrameMetadata, FramePolicy, MarkerDetector, VehiclePose, VisualEvidenceTracker,
    match_estimates, project_to_plane, validate_frame_metadata,
)


def image(*boxes, width=160, height=120, encoding="rgb8"):
    value = np.zeros((height, width, 3), dtype=np.uint8)
    for x, y, w, h, color in boxes:
        value[y:y+h, x:x+w] = color
    if encoding == "bgr8":
        value = value[:, :, ::-1]
    return value.tobytes(), width, height, encoding


class MarkerDetectorTests(unittest.TestCase):
    def setUp(self):
        self.detector = MarkerDetector(DetectorPolicy(minimum_area_px=20))

    def test_correct_marker_and_bgr_encoding(self):
        for encoding in ("rgb8", "bgr8"):
            found = self.detector.detect(*image((20, 30, 20, 20, (0, 255, 0)), encoding=encoding))
            self.assertEqual(len(found), 1)
            self.assertGreater(found[0].confidence, 0.8)

    def test_boundary_partial_occlusion_and_sizes(self):
        boundary = self.detector.detect(*image((0, 20, 15, 15, (0, 255, 0))))
        partial = self.detector.detect(*image(
            (30, 30, 20, 20, (0, 255, 0)), (30, 30, 8, 8, (0, 0, 0))))
        large = self.detector.detect(*image((80, 50, 28, 24, (0, 255, 0))))
        self.assertEqual(tuple(map(len, (boundary, partial, large))), (1, 1, 1))

    def test_similar_color_geometry_distractor_rejected(self):
        found = self.detector.detect(*image((10, 10, 60, 5, (0, 255, 0))))
        self.assertEqual(found, ())

    def test_multiple_ordered_candidates_and_no_marker(self):
        found = self.detector.detect(*image(
            (90, 30, 15, 15, (0, 255, 0)), (10, 50, 18, 18, (0, 255, 0))))
        self.assertEqual([item.x for item in found], [10, 90])
        self.assertEqual(self.detector.detect(*image()), ())

    def test_malformed_and_unsupported_images(self):
        with self.assertRaisesRegex(ValueError, "truncated_image_buffer"):
            self.detector.detect(b"bad", 10, 10, "rgb8")
        with self.assertRaisesRegex(ValueError, "unsupported_encoding"):
            self.detector.detect(bytes(300), 10, 10, "mono8")


class ProjectionAndEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.calibration = CameraCalibration(160, 120, 120, 120, 80, 60,
                                             "camera_optical", 0.2, 0, 0.1,
                                             radians(35))
        self.pose = VehiclePose("s1", 1, 10, 20, 0, 0, 1.5, 0, 0, 0)
        self.candidate = MarkerDetector(DetectorPolicy(minimum_area_px=20)).detect(
            *image((70, 70, 20, 20, (0, 255, 0))))[0]

    def observation(self, frame, time, east=0.0, confidence=None):
        pose = VehiclePose("s1", frame, time, time + 1, east, 0, 1.5, 0, 0, 0)
        candidate = self.candidate
        if confidence is not None:
            candidate = type(candidate)(
                candidate.x, candidate.y, candidate.width, candidate.height,
                candidate.centroid_x, candidate.centroid_y, candidate.area_px,
                candidate.fill_ratio, candidate.aspect_ratio,
                candidate.color_score, confidence)
        return CandidateObservation(f"obs-{frame}", "s1", frame, time, time + 1,
                                    time, east, 0, candidate,
                                    project_to_plane(candidate, self.calibration, pose))

    def test_pinhole_projection_and_uncertainty(self):
        result = project_to_plane(self.candidate, self.calibration, self.pose)
        self.assertGreater(result.range_m, 0)
        self.assertGreater(result.uncertainty_radius_m, 0.2)

    def test_invalid_ray_and_calibration(self):
        flat = CameraCalibration(160, 120, 120, 120, 80, 60, "camera", 0, 0, 0, 0)
        upper = MarkerDetector(DetectorPolicy(minimum_area_px=20)).detect(
            *image((70, 10, 20, 20, (0, 255, 0))))[0]
        with self.assertRaisesRegex(ValueError, "ray_does_not_intersect"):
            project_to_plane(upper, flat, self.pose)
        with self.assertRaisesRegex(ValueError, "invalid_camera_calibration"):
            CameraCalibration(160, 120, 0, 120, 80, 60, "camera", 0, 0, 0, 0)

    def test_confirmation_requires_distinct_frame_time_and_viewpoint(self):
        tracker = VisualEvidenceTracker(ConfirmationPolicy(
            minimum_time_separation_ns=50, minimum_viewpoint_separation_m=0.2,
            clustering_radius_m=2.0))
        tracker.reset_session("s1")
        self.assertIsNone(tracker.observe(self.observation(1, 10)).confirmed_id)
        self.assertIsNone(tracker.observe(self.observation(2, 20, 0.1)).confirmed_id)
        confirmed = tracker.observe(self.observation(3, 30, 0.3))
        self.assertEqual(confirmed.confirmed_id, "survivor-001")

    def test_time_separation_can_confirm_stationary_evidence(self):
        tracker = VisualEvidenceTracker(ConfirmationPolicy(
            minimum_time_separation_ns=5, minimum_viewpoint_separation_m=1.0,
            clustering_radius_m=2.0))
        tracker.reset_session("s1")
        tracker.observe(self.observation(1, 10))
        self.assertEqual(tracker.observe(self.observation(2, 20)).confirmed_id,
                         "survivor-001")

    def test_duplicate_out_of_order_and_session_reset(self):
        tracker = VisualEvidenceTracker(ConfirmationPolicy(clustering_radius_m=2.0))
        tracker.reset_session("s1")
        tracker.observe(self.observation(1, 10))
        with self.assertRaisesRegex(ValueError, "duplicate_frame"):
            tracker.observe(self.observation(1, 11))
        with self.assertRaisesRegex(ValueError, "out_of_order"):
            tracker.observe(self.observation(2, 9))
        tracker.reset_session("s2")
        self.assertEqual(tracker.clusters, [])
        with self.assertRaisesRegex(ValueError, "cross_session"):
            tracker.observe(self.observation(3, 30))

    def test_low_confidence_does_not_confirm(self):
        tracker = VisualEvidenceTracker(ConfirmationPolicy(
            minimum_time_separation_ns=1, minimum_viewpoint_separation_m=0.1,
            clustering_radius_m=2.0))
        tracker.reset_session("s1")
        tracker.observe(self.observation(1, 10, confidence=0.5))
        cluster = tracker.observe(self.observation(2, 20, east=0.3, confidence=0.5))
        self.assertIsNone(cluster.confirmed_id)

    def test_one_to_one_matching_and_metrics(self):
        result = match_estimates(((0, 0), (0.1, 0), (5, 5)), ((0, 0), (5, 5), (9, 9)), 0.5)
        self.assertEqual(len(result.matches), 2)
        self.assertEqual((result.false_positives, result.false_negatives), (1, 1))
        self.assertAlmostEqual(result.precision, 2 / 3)
        self.assertAlmostEqual(result.recall, 2 / 3)


class FrameValidationTests(unittest.TestCase):
    def value(self, **changes):
        raw = dict(session_id="s", pose_session_id="s", sequence=2,
                   previous_sequence=1, image_time_ns=20,
                   previous_image_time_ns=10, receipt_monotonic_ns=100,
                   now_monotonic_ns=110, pose_receipt_monotonic_ns=90,
                   processing_latency_ns=5, width=160, height=120,
                   encoding="rgb8", calibrated=True)
        raw.update(changes)
        return FrameMetadata(**raw)

    def setUp(self):
        self.policy = FramePolicy(160, 120, 50, 50, 20)

    def test_valid_and_bgr_frames(self):
        validate_frame_metadata(self.value(), self.policy)
        validate_frame_metadata(self.value(encoding="bgr8"), self.policy)

    def test_stale_duplicate_and_out_of_order_frames(self):
        cases = (
            ("stale_frame", dict(now_monotonic_ns=200)),
            ("out_of_order_frame", dict(sequence=1)),
            ("out_of_order_frame", dict(image_time_ns=10)),
        )
        for reason, changes in cases:
            with self.assertRaisesRegex(ValueError, reason):
                validate_frame_metadata(self.value(**changes), self.policy)

    def test_calibration_session_pose_and_latency_rejections(self):
        cases = (
            ("missing_calibration", dict(calibrated=False)),
            ("cross_session_pose", dict(pose_session_id="old")),
            ("stale_vehicle_pose", dict(pose_receipt_monotonic_ns=0)),
            ("processing_latency_exceeded", dict(processing_latency_ns=21)),
            ("unsupported_encoding", dict(encoding="mono8")),
        )
        for reason, changes in cases:
            with self.assertRaisesRegex(ValueError, reason):
                validate_frame_metadata(self.value(**changes), self.policy)


if __name__ == "__main__":
    unittest.main()
