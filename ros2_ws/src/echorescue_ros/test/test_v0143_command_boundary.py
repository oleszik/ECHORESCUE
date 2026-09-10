"""Native Jazzy import and scope checks for the v0.14.3 ROS adapters."""

from inspect import getsource

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("echorescue_interfaces")
pytest.importorskip("pymavlink")

from echorescue.flight_mission import validate_simulation_endpoint
from echorescue_ros.flight_mission_observer import FlightMissionObserver
from echorescue_ros.mavlink_flight_mission import MavlinkFlightMission
from echorescue_ros.mavlink_telemetry_bridge import MavlinkTelemetryBridge


def test_v0143_ros_nodes_import_in_sourced_jazzy_workspace() -> None:
    assert MavlinkFlightMission.__name__ == "MavlinkFlightMission"
    assert FlightMissionObserver.__name__ == "FlightMissionObserver"


def test_legacy_bridge_remains_command_free() -> None:
    assert "command_long_send" not in getsource(MavlinkTelemetryBridge)


def test_command_node_excludes_ground_truth_and_general_navigation_apis() -> None:
    source = getsource(MavlinkFlightMission)
    assert "command_long_send" in source
    for prohibited in (
        "mission_item_send",
        "set_position_target",
        "rc_channels_override",
        "/world/",
        "gz.msgs",
    ):
        assert prohibited not in source


def test_command_endpoint_is_loopback_tcp_only() -> None:
    assert validate_simulation_endpoint("tcp:127.0.0.1:5760") == "tcp:127.0.0.1:5760"
    with pytest.raises(ValueError, match="loopback TCP SITL"):
        validate_simulation_endpoint("udp:127.0.0.1:14550")
