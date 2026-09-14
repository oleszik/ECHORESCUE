"""Native Jazzy interface and trust-boundary checks for v0.16.0."""

from inspect import getsource

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("echorescue_interfaces")

from echorescue_interfaces.msg import ExplorationState
from echorescue_ros.frontier_exploration_observer import FrontierExplorationObserver
from echorescue_ros.mavlink_frontier_exploration import MavlinkFrontierExploration
from echorescue_ros.mavlink_telemetry_bridge import MavlinkTelemetryBridge
from echorescue_ros.qos import MISSION_QOS


def test_exploration_message_schema_has_compact_reconstruction_fields() -> None:
    fields = ExplorationState.get_fields_and_field_types()
    required = {
        "schema_version", "session_id", "sequence", "state", "event",
        "map_revision", "unknown_cell_count", "free_cell_count",
        "occupied_cell_count", "inflated_cell_count", "accepted_scan_count",
        "rejected_scan_count", "frontier_cluster_count",
        "has_selected_frontier", "route_generation", "active_target_id",
    }
    assert required <= fields.keys()


def test_observer_is_typed_ros_only() -> None:
    source = getsource(FrontierExplorationObserver)
    for prohibited in ("pymavlink", "gz.msgs", "/world/", "subprocess"):
        assert prohibited not in source


def test_production_explorer_has_no_gazebo_ground_truth_access() -> None:
    source = getsource(MavlinkFrontierExploration)
    for prohibited in ("gz.msgs", "/world/", "contact", "model pose", "evaluator"):
        assert prohibited not in source.lower()


def test_legacy_bridge_remains_receive_only_and_qos_is_bounded() -> None:
    assert "command_long_send" not in getsource(MavlinkTelemetryBridge)
    assert MISSION_QOS.depth > 0
