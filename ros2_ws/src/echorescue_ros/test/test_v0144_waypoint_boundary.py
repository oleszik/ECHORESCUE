"""Native Jazzy interface, conversion, and scope tests for v0.14.4."""

from inspect import getsource

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("echorescue_interfaces")
pytest.importorskip("pymavlink")

from echorescue.waypoint_mission import EnuTarget, TargetRequest, WaypointMissionEvent, enu_target_to_ned
from echorescue_ros.conversions import waypoint_event_to_msg, waypoint_target_to_msg
from echorescue_ros.mavlink_telemetry_bridge import MavlinkTelemetryBridge
from echorescue_ros.mavlink_waypoint_mission import MavlinkWaypointMission
from echorescue_ros.waypoint_mission_observer import WaypointMissionObserver


def test_v0144_nodes_import_in_sourced_jazzy_workspace() -> None:
    assert MavlinkWaypointMission.__name__ == "MavlinkWaypointMission"
    assert WaypointMissionObserver.__name__ == "WaypointMissionObserver"


def test_typed_target_and_event_conversion() -> None:
    from builtin_interfaces.msg import Time

    target = EnuTarget("waypoint-a", 3.0, 4.0, 2.5, 0.0)
    request = TargetRequest("session", target, enu_target_to_ned(target))
    message = waypoint_target_to_msg(
        request, Time(sec=1), sequence=1,
        transmit_monotonic_ns=20, transmit_source_time_boot_ms=10,
    )
    assert message.header.frame_id == "echorescue/world_enu"
    assert (message.ned_north_m, message.ned_east_m, message.ned_down_m) == (4.0, 3.0, -2.5)
    assert message.has_heading
    event = waypoint_event_to_msg(
        WaypointMissionEvent(2, 30, "session", "track_target", "target_settled", "waypoint-a", "done"),
        Time(sec=2),
    )
    assert (event.sequence, event.event, event.target_id) == (2, "target_settled", "waypoint-a")


def test_legacy_bridge_remains_receive_only() -> None:
    source = getsource(MavlinkTelemetryBridge)
    assert "command_long_send" not in source
    assert "set_position_target_local_ned_send" not in source


def test_waypoint_command_node_has_only_narrow_local_target_interface() -> None:
    source = getsource(MavlinkWaypointMission)
    assert "command_long_send" in source
    assert "set_position_target_local_ned_send" in source
    for prohibited in ("mission_item_send", "rc_channels_override", "/world/", "gz.msgs"):
        assert prohibited not in source
