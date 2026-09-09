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
from echorescue.continuous_vehicle_state import ContinuousVehicleState
from echorescue_ros.conversions import (
    continuous_vehicle_state_from_msg,
    continuous_vehicle_state_to_msg,
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


def test_continuous_state_round_trip_preserves_frames_times_and_validity() -> None:
    source = ContinuousVehicleState(
        vehicle_id="iris-1", session_id="session-1", sequence=5,
        source_sequence=10, system_id=1, component_id=1,
        source_time_boot_ms=1_000, receipt_monotonic_ns=2_000_000,
        source_frame="mavlink/local_ned", output_frame="echorescue/world_enu",
        source_body_frame="mavlink/body_frd", output_body_frame="echorescue/body_flu",
        world_origin_policy="ardupilot_local_ned_at_sitl_startup",
        angle_convention="right_handed_radians_ccw_from_east",
        x_m=2.0, y_m=1.0, z_m=3.0, vx_m_s=0.2, vy_m_s=0.1, vz_m_s=0.3,
        position_valid=True, velocity_valid=True, has_attitude=True,
        roll_rad=0.1, pitch_rad=-0.2, yaw_rad=1.3,
        attitude_source_time_boot_ms=990, has_heading=False, heading_deg=0.0,
        heading_source_time_boot_ms=0, armed=False, has_landed_state=False,
        landed=False, telemetry_health=TelemetryHealth.CONNECTED,
    )
    message = continuous_vehicle_state_to_msg(source, STAMP)
    assert message.header.frame_id == "echorescue/world_enu"
    assert message.source_time_boot_ms != message.receipt_monotonic_ns
    assert continuous_vehicle_state_from_msg(message) == source
