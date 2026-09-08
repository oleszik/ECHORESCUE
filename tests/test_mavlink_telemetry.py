import json
import unittest

from echorescue.mavlink_telemetry import (
    AUTOPILOT_FRAME,
    GLOBAL_WGS84_FRAME,
    LOCAL_NED_FRAME,
    TelemetryCore,
    TelemetryHealth,
    parse_global_position_int,
    parse_heartbeat,
    parse_local_position_ned,
    serialize_telemetry,
)


HEARTBEAT = {"base_mode": 129, "custom_mode": 4}
LOCAL = {"time_boot_ms": 1000, "x": 1.25, "y": -2.5, "z": 0.75, "vx": 0.1, "vy": -0.2, "vz": 0.3}
GLOBAL = {
    "time_boot_ms": 1100,
    "lat": 473977420,
    "lon": 85455940,
    "alt": 488123,
    "relative_alt": 1234,
    "vx": 120,
    "vy": -45,
    "vz": 8,
    "hdg": 9050,
}


class ParsingTests(unittest.TestCase):
    def test_heartbeat_extracts_armed_mode_ids_and_frame(self) -> None:
        sample = parse_heartbeat(
            HEARTBEAT,
            session_id="session-1",
            sequence=1,
            source_sequence=7,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=500,
        )
        self.assertTrue(sample.armed)
        self.assertEqual(sample.flight_mode, "GUIDED")
        self.assertEqual((sample.system_id, sample.component_id), (1, 1))
        self.assertEqual(sample.frame_id, AUTOPILOT_FRAME)
        disarmed = parse_heartbeat(
            {"base_mode": 1, "custom_mode": 6},
            session_id="session-1",
            sequence=2,
            source_sequence=8,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=600,
        )
        self.assertFalse(disarmed.armed)
        self.assertEqual(disarmed.flight_mode, "RTL")

    def test_local_position_preserves_ned_units_timestamps_and_velocity(self) -> None:
        sample = parse_local_position_ned(
            LOCAL,
            session_id="session-1",
            sequence=2,
            source_sequence=8,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=600,
        )
        self.assertEqual(sample.frame_id, LOCAL_NED_FRAME)
        self.assertEqual(sample.source_time_boot_ms, 1000)
        self.assertEqual(sample.receipt_monotonic_ns, 600)
        self.assertEqual((sample.north_m, sample.east_m, sample.down_m), (1.25, -2.5, 0.75))
        self.assertEqual(
            (sample.velocity_north_m_s, sample.velocity_east_m_s, sample.velocity_down_m_s),
            (0.1, -0.2, 0.3),
        )

    def test_optional_global_position_converts_protocol_units(self) -> None:
        sample = parse_global_position_int(
            GLOBAL,
            session_id="session-1",
            sequence=3,
            source_sequence=9,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=700,
        )
        self.assertEqual(sample.frame_id, GLOBAL_WGS84_FRAME)
        self.assertAlmostEqual(sample.latitude_deg, 47.397742)
        self.assertAlmostEqual(sample.longitude_deg, 8.545594)
        self.assertAlmostEqual(sample.altitude_m_msl, 488.123)
        self.assertAlmostEqual(sample.relative_altitude_m, 1.234)
        self.assertEqual(sample.velocity_north_m_s, 1.2)
        self.assertEqual(sample.heading_deg, 90.5)
        unavailable = parse_global_position_int(
            {**GLOBAL, "hdg": 65535},
            session_id="session-1",
            sequence=4,
            source_sequence=10,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=800,
        )
        self.assertIsNone(unavailable.heading_deg)

    def test_serialization_is_deterministic(self) -> None:
        sample = parse_local_position_ned(
            LOCAL,
            session_id="session-1",
            sequence=2,
            source_sequence=8,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=600,
        )
        self.assertEqual(serialize_telemetry(sample), serialize_telemetry(sample))
        self.assertEqual(json.loads(serialize_telemetry(sample))["frame_id"], LOCAL_NED_FRAME)


class TelemetryCoreTests(unittest.TestCase):
    def _connected_core(self) -> TelemetryCore:
        core = TelemetryCore("test", freshness_threshold_s=1.0)
        heartbeat = core.ingest_heartbeat(
            HEARTBEAT,
            source_sequence=10,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=1_000_000_000,
        )
        self.assertIsNotNone(heartbeat)
        return core

    def test_heartbeat_starts_degraded_session_then_position_connects(self) -> None:
        core = self._connected_core()
        self.assertEqual(core.status(1_000_000_000).health, TelemetryHealth.DEGRADED)
        position = core.ingest_local_position(
            LOCAL,
            source_sequence=11,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=1_100_000_000,
        )
        self.assertIsNotNone(position)
        self.assertEqual(core.status(1_100_000_000).health, TelemetryHealth.CONNECTED)

    def test_duplicate_and_out_of_order_timestamps_are_ignored(self) -> None:
        core = self._connected_core()
        first = core.ingest_local_position(
            LOCAL,
            source_sequence=11,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=1_100_000_000,
        )
        duplicate = core.ingest_local_position(
            LOCAL,
            source_sequence=12,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=1_200_000_000,
        )
        older = core.ingest_local_position(
            {**LOCAL, "time_boot_ms": 999},
            source_sequence=13,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=1_300_000_000,
        )
        advancing = core.ingest_local_position(
            {**LOCAL, "time_boot_ms": 1001},
            source_sequence=14,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=1_400_000_000,
        )
        self.assertIsNotNone(first)
        self.assertIsNone(duplicate)
        self.assertIsNone(older)
        self.assertIsNotNone(advancing)

    def test_packet_sequence_and_receipt_time_are_monotonic_with_wrap(self) -> None:
        core = TelemetryCore("wrap", 1.0)
        first = core.ingest_heartbeat(
            HEARTBEAT,
            source_sequence=255,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=100,
        )
        wrapped = core.ingest_local_position(
            LOCAL,
            source_sequence=0,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=101,
        )
        duplicate_packet = core.ingest_global_position(
            GLOBAL,
            source_sequence=0,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=102,
        )
        old_receipt = core.ingest_global_position(
            GLOBAL,
            source_sequence=1,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=99,
        )
        self.assertIsNotNone(first)
        self.assertIsNotNone(wrapped)
        self.assertIsNone(duplicate_packet)
        self.assertIsNone(old_receipt)

    def test_freshness_boundary_and_stale_transition(self) -> None:
        core = self._connected_core()
        core.ingest_local_position(
            LOCAL,
            source_sequence=11,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=2_000_000_000,
        )
        boundary = core.status(3_000_000_000)
        stale = core.status(3_000_000_001)
        self.assertEqual(boundary.health, TelemetryHealth.CONNECTED)
        self.assertEqual(boundary.telemetry_age_s, 1.0)
        self.assertEqual(stale.health, TelemetryHealth.STALE)

    def test_connection_loss_and_reconnect_create_new_session(self) -> None:
        core = self._connected_core()
        first_session = core.session_id
        core.disconnect(2_000_000_000, "heartbeat timeout")
        disconnected = core.status(2_000_000_000)
        self.assertEqual(disconnected.health, TelemetryHealth.DISCONNECTED)
        reconnected = core.ingest_heartbeat(
            HEARTBEAT,
            source_sequence=1,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=3_000_000_000,
        )
        self.assertIsNotNone(reconnected)
        self.assertNotEqual(core.session_id, first_session)
        self.assertEqual(core.status(3_000_000_000).health, TelemetryHealth.DEGRADED)

    def test_stationary_position_with_advancing_source_time_is_live(self) -> None:
        core = self._connected_core()
        first = core.ingest_local_position(
            LOCAL,
            source_sequence=11,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=1_100_000_000,
        )
        second = core.ingest_local_position(
            {**LOCAL, "time_boot_ms": 1100},
            source_sequence=12,
            system_id=1,
            component_id=1,
            receipt_monotonic_ns=1_200_000_000,
        )
        self.assertEqual(first.north_m, second.north_m)
        self.assertGreater(second.source_time_boot_ms, first.source_time_boot_ms)
        self.assertEqual(core.status(1_200_000_000).health, TelemetryHealth.CONNECTED)


class ImportBoundaryTests(unittest.TestCase):
    def test_core_import_does_not_load_ros_or_pymavlink(self) -> None:
        import subprocess
        import sys

        program = (
            "import sys; import echorescue.mavlink_telemetry; "
            "assert 'rclpy' not in sys.modules; assert 'pymavlink' not in sys.modules"
        )
        completed = subprocess.run([sys.executable, "-c", program], check=False)
        self.assertEqual(completed.returncode, 0)

    def test_bridge_source_contains_no_flight_or_gazebo_ground_truth_api(self) -> None:
        from pathlib import Path

        source = (
            Path(__file__).parents[1]
            / "ros2_ws/src/echorescue_ros/echorescue_ros/mavlink_telemetry_bridge.py"
        ).read_text(encoding="utf-8")
        self.assertIn("request_data_stream_send", source)
        for prohibited in (
            "command_long_send",
            "set_mode_send",
            "mission_item_send",
            "rc_channels_override_send",
            "/world/",
            "gz.msgs",
        ):
            self.assertNotIn(prohibited, source)


if __name__ == "__main__":
    unittest.main()
