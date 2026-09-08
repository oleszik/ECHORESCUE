import pytest

pytest.importorskip("rclpy", reason="ROS 2 conversion tests run in the ROS workspace")
pytest.importorskip("echorescue_interfaces", reason="build and source the ROS workspace")

from builtin_interfaces.msg import Time

from echorescue.mavlink_telemetry import (
    GlobalPositionTelemetry,
    HeartbeatTelemetry,
    LocalPositionNedTelemetry,
    TelemetryHealth,
    TelemetryStatus,
)
from echorescue_ros.conversions import (
    mavlink_global_position_from_msg,
    mavlink_global_position_to_msg,
    mavlink_local_position_from_msg,
    mavlink_local_position_to_msg,
    mavlink_status_from_msg,
    mavlink_status_to_msg,
    mavlink_vehicle_state_from_msg,
    mavlink_vehicle_state_to_msg,
)


STAMP = Time(sec=123, nanosec=456)


def test_vehicle_state_round_trip() -> None:
    source = HeartbeatTelemetry("session-1", 1, 7, 1, 1, 500, True, "GUIDED")
    message = mavlink_vehicle_state_to_msg(source, STAMP)
    assert message.header.stamp == STAMP
    assert mavlink_vehicle_state_from_msg(message) == source


def test_local_ned_round_trip_preserves_frame_units_and_timestamps() -> None:
    source = LocalPositionNedTelemetry(
        "session-1", 2, 8, 1, 1, 1000, 600, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3
    )
    message = mavlink_local_position_to_msg(source, STAMP)
    assert message.header.frame_id == "mavlink/local_ned"
    assert message.source_time_boot_ms == 1000
    assert mavlink_local_position_from_msg(message) == source


def test_optional_global_round_trip_preserves_missing_heading() -> None:
    source = GlobalPositionTelemetry(
        "session-1",
        3,
        9,
        1,
        1,
        1100,
        700,
        47.397742,
        8.545594,
        488.123,
        1.234,
        1.2,
        -0.45,
        0.08,
        None,
    )
    message = mavlink_global_position_to_msg(source, STAMP)
    assert not message.has_heading
    assert mavlink_global_position_from_msg(message) == source


def test_health_round_trip_includes_age_and_stale_state() -> None:
    source = TelemetryStatus(
        "session-1",
        4,
        TelemetryHealth.STALE,
        True,
        1,
        1,
        1100,
        700,
        1.25,
        1.0,
        "position telemetry exceeded freshness threshold",
    )
    message = mavlink_status_to_msg(source, STAMP)
    assert message.health == message.STALE
    assert mavlink_status_from_msg(message) == source
