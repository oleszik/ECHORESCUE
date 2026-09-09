import json
from math import isclose, pi
import unittest

from echorescue.continuous_vehicle_state import (
    ANGLE_CONVENTION,
    BODY_FLU_FRAME,
    BODY_FRD_FRAME,
    LOCAL_NED_FRAME,
    WORLD_ENU_FRAME,
    WORLD_ORIGIN_POLICY,
    ContinuousStateAssembler,
    attitude_enu_flu_to_ned_frd,
    attitude_ned_frd_to_enu_flu,
    enu_to_ned,
    flu_to_frd,
    frd_to_flu,
    heading_enu_deg_to_ned_deg,
    heading_ned_deg_to_enu_deg,
    ned_to_enu,
    normalize_angle_deg,
    normalize_angle_rad,
    serialize_continuous_state,
)
from echorescue.mavlink_telemetry import (
    AttitudeNedFrdTelemetry,
    GlobalPositionTelemetry,
    HeartbeatTelemetry,
    LandedStateTelemetry,
    LocalPositionNedTelemetry,
    TelemetryHealth,
    TelemetryStatus,
)


def heartbeat(session: str = "session-1", armed: bool = False) -> HeartbeatTelemetry:
    return HeartbeatTelemetry(session, 1, 10, 1, 1, 1_000, armed, "STABILIZE")


def local(session: str = "session-1", source_ms: int = 1_000) -> LocalPositionNedTelemetry:
    return LocalPositionNedTelemetry(
        session, 4, 13, 1, 1, source_ms, 4_000,
        10.0, 20.0, -3.0, 1.0, 2.0, -0.5,
    )


def status(session: str = "session-1") -> TelemetryStatus:
    return TelemetryStatus(
        session, 4, TelemetryHealth.CONNECTED, True, 1, 1,
        1_000, 4_000, 0.0, 1.0, "fresh",
    )


class VectorFrameTests(unittest.TestCase):
    def test_ned_position_and_velocity_mapping(self) -> None:
        self.assertEqual(ned_to_enu((10.0, 20.0, -3.0)), (20.0, 10.0, 3.0))
        self.assertEqual(ned_to_enu((1.0, 2.0, -0.5)), (2.0, 1.0, 0.5))

    def test_ned_enu_round_trip_property(self) -> None:
        for vector in ((0.0, 0.0, 0.0), (1.25, -2.5, 3.75), (-9.0, 8.0, -7.0)):
            self.assertEqual(enu_to_ned(ned_to_enu(vector)), vector)

    def test_frd_flu_semantics_and_round_trip(self) -> None:
        self.assertEqual(frd_to_flu((4.0, 2.0, -3.0)), (4.0, -2.0, 3.0))
        for vector in ((0.0, 0.0, 0.0), (1.0, -2.0, 3.0)):
            self.assertEqual(flu_to_frd(frd_to_flu(vector)), vector)

    def test_non_finite_vectors_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite"):
            ned_to_enu((float("nan"), 0.0, 0.0))


class AngleTests(unittest.TestCase):
    def test_heading_cardinals_and_wraparound(self) -> None:
        self.assertEqual(heading_ned_deg_to_enu_deg(0.0), 90.0)
        self.assertEqual(heading_ned_deg_to_enu_deg(90.0), 0.0)
        self.assertEqual(heading_ned_deg_to_enu_deg(180.0), -90.0)
        self.assertEqual(heading_ned_deg_to_enu_deg(270.0), -180.0)
        self.assertEqual(normalize_angle_deg(540.0), -180.0)
        self.assertTrue(isclose(normalize_angle_rad(3.0 * pi), -pi))

    def test_heading_round_trip_property(self) -> None:
        for heading in (0.0, 1.0, 89.5, 180.0, 270.0, 359.999):
            self.assertTrue(isclose(heading_enu_deg_to_ned_deg(heading_ned_deg_to_enu_deg(heading)), heading))

    def test_level_north_attitude_becomes_level_east_frame_yaw(self) -> None:
        roll, pitch, yaw = attitude_ned_frd_to_enu_flu(0.0, 0.0, 0.0)
        self.assertTrue(isclose(roll, 0.0, abs_tol=1e-12))
        self.assertTrue(isclose(pitch, 0.0, abs_tol=1e-12))
        self.assertTrue(isclose(yaw, pi / 2.0, abs_tol=1e-12))

    def test_attitude_round_trip_property(self) -> None:
        for source in ((0.0, 0.0, 0.0), (0.2, -0.3, 1.1), (-0.5, 0.4, -2.2)):
            converted = attitude_ned_frd_to_enu_flu(*source)
            restored = attitude_enu_flu_to_ned_frd(*converted)
            for actual, expected in zip(restored, source):
                self.assertTrue(isclose(actual, expected, abs_tol=1e-12))


class AssemblyTests(unittest.TestCase):
    def test_complete_state_preserves_identity_times_frames_and_optional_values(self) -> None:
        assembler = ContinuousStateAssembler("iris-1")
        assembler.ingest_heartbeat(heartbeat(armed=True))
        assembler.ingest_attitude(AttitudeNedFrdTelemetry(
            "session-1", 2, 11, 1, 1, 900, 2_000, 0.1, -0.2, 0.3,
        ))
        assembler.ingest_heading(GlobalPositionTelemetry(
            "session-1", 3, 12, 1, 1, 950, 3_000,
            47.0, 8.0, 500.0, 0.0, 0.0, 0.0, 0.0, 45.0,
        ))
        assembler.ingest_landed_state(LandedStateTelemetry(
            "session-1", 3, 12, 1, 1, 3_500, True, "ON_GROUND",
        ))
        converted = assembler.convert(local(), status())
        assert converted is not None
        self.assertEqual((converted.x_m, converted.y_m, converted.z_m), (20.0, 10.0, 3.0))
        self.assertEqual((converted.vx_m_s, converted.vy_m_s, converted.vz_m_s), (2.0, 1.0, 0.5))
        self.assertEqual(converted.source_time_boot_ms, 1_000)
        self.assertEqual(converted.receipt_monotonic_ns, 4_000)
        self.assertNotEqual(converted.source_time_boot_ms, converted.receipt_monotonic_ns)
        self.assertEqual((converted.source_frame, converted.output_frame), (LOCAL_NED_FRAME, WORLD_ENU_FRAME))
        self.assertEqual((converted.source_body_frame, converted.output_body_frame), (BODY_FRD_FRAME, BODY_FLU_FRAME))
        self.assertEqual(converted.world_origin_policy, WORLD_ORIGIN_POLICY)
        self.assertEqual(converted.angle_convention, ANGLE_CONVENTION)
        self.assertTrue(converted.has_attitude)
        self.assertTrue(converted.has_heading)
        self.assertTrue(converted.armed)
        self.assertTrue(converted.has_landed_state)
        self.assertTrue(converted.landed)

    def test_missing_optional_fields_have_explicit_invalid_indicators(self) -> None:
        assembler = ContinuousStateAssembler("iris-1")
        assembler.ingest_heartbeat(heartbeat())
        converted = assembler.convert(local(), status())
        assert converted is not None
        self.assertFalse(converted.has_attitude)
        self.assertFalse(converted.has_heading)
        self.assertFalse(converted.has_landed_state)
        self.assertEqual((converted.roll_rad, converted.heading_deg), (0.0, 0.0))

    def test_duplicate_source_time_is_rejected_but_new_session_resets_ordering(self) -> None:
        assembler = ContinuousStateAssembler("iris-1")
        assembler.ingest_heartbeat(heartbeat("session-1"))
        self.assertIsNotNone(assembler.convert(local("session-1", 1_000), status("session-1")))
        self.assertIsNone(assembler.convert(local("session-1", 1_000), status("session-1")))
        assembler.ingest_heartbeat(heartbeat("session-2"))
        self.assertIsNotNone(assembler.convert(local("session-2", 10), status("session-2")))
        self.assertIsNone(assembler.convert(local("session-1", 2_000), status("session-1")))

    def test_wrong_source_frame_and_future_optional_metadata_are_not_misrepresented(self) -> None:
        assembler = ContinuousStateAssembler("iris-1")
        assembler.ingest_heartbeat(heartbeat())
        assembler.ingest_attitude(AttitudeNedFrdTelemetry(
            "session-1", 2, 11, 1, 1, 2_000, 2_000, 0.1, 0.2, 0.3,
        ))
        wrong_frame = LocalPositionNedTelemetry(
            "session-1", 3, 12, 1, 1, 900, 3_000,
            1.0, 2.0, 3.0, 0.0, 0.0, 0.0, frame_id="unknown",
        )
        self.assertIsNone(assembler.convert(wrong_frame, status()))
        converted = assembler.convert(local(source_ms=1_000), status())
        assert converted is not None
        self.assertFalse(converted.has_attitude)
        self.assertEqual(converted.attitude_source_time_boot_ms, 0)

    def test_serialization_is_deterministic(self) -> None:
        assembler = ContinuousStateAssembler("iris-1")
        assembler.ingest_heartbeat(heartbeat())
        converted = assembler.convert(local(), status())
        assert converted is not None
        self.assertEqual(serialize_continuous_state(converted), serialize_continuous_state(converted))
        self.assertEqual(json.loads(serialize_continuous_state(converted))["output_frame"], WORLD_ENU_FRAME)


class ImportBoundaryTests(unittest.TestCase):
    def test_continuous_core_does_not_import_ros_pymavlink_or_gazebo(self) -> None:
        import subprocess
        import sys
        program = (
            "import sys; import echorescue.continuous_vehicle_state; "
            "assert 'rclpy' not in sys.modules; assert 'pymavlink' not in sys.modules"
        )
        self.assertEqual(subprocess.run([sys.executable, "-c", program,], check=False).returncode, 0)


if __name__ == "__main__":
    unittest.main()
