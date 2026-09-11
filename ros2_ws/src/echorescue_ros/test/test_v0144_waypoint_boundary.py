"""Native Jazzy interface, conversion, and scope tests for v0.14.4."""

import builtins
from importlib.metadata import distribution
from inspect import getsource
import os
from pathlib import Path
import subprocess
import sys

import pytest
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

pytest.importorskip("rclpy")
pytest.importorskip("echorescue_interfaces")

from echorescue.waypoint_mission import EnuTarget, TargetRequest, WaypointMissionEvent, enu_target_to_ned
from echorescue_ros.conversions import waypoint_event_to_msg, waypoint_target_to_msg
from echorescue_ros.flight_mission_observer import FlightMissionObserver
from echorescue_ros.mavlink_telemetry_bridge import MavlinkTelemetryBridge, _load_mavutil
from echorescue_ros.mavlink_waypoint_mission import MISSION_QOS as PUBLISHER_MISSION_QOS, MavlinkWaypointMission
from echorescue_ros.qos import MISSION_QOS, STATUS_QOS, TELEMETRY_QOS
from echorescue_ros.telemetry_observer import STATUS_QOS as TELEMETRY_OBSERVER_STATUS_QOS
from echorescue_ros.telemetry_observer import TELEMETRY_QOS as TELEMETRY_OBSERVER_QOS
from echorescue_ros.waypoint_mission_observer import MISSION_QOS as OBSERVER_MISSION_QOS
from echorescue_ros.waypoint_mission_observer import STATUS_QOS as WAYPOINT_OBSERVER_STATUS_QOS
from echorescue_ros.waypoint_mission_observer import TELEMETRY_QOS as WAYPOINT_OBSERVER_TELEMETRY_QOS
from echorescue_ros.waypoint_mission_observer import WaypointMissionObserver


def test_v0144_nodes_import_in_sourced_jazzy_workspace() -> None:
    assert MavlinkWaypointMission.__name__ == "MavlinkWaypointMission"
    assert WaypointMissionObserver.__name__ == "WaypointMissionObserver"


def test_waypoint_observer_import_does_not_require_pymavlink() -> None:
    code = """
import sys
class BlockPymavlink:
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'pymavlink' or fullname.startswith('pymavlink.'):
            raise ModuleNotFoundError("blocked pymavlink")
        return None
sys.meta_path.insert(0, BlockPymavlink())
import echorescue_ros.waypoint_mission_observer
assert 'pymavlink' not in sys.modules
"""
    environment = dict(os.environ)
    environment["PYTHONNOUSERSITE"] = "1"
    repository_src = Path(__file__).resolve().parents[4] / "src"
    dependency_light_paths = [
        item for item in environment.get("PYTHONPATH", "").split(os.pathsep)
        if item and ".venv" not in item and "/venv-" not in item
    ]
    environment["PYTHONPATH"] = os.pathsep.join((str(repository_src), *dependency_light_paths))
    assert ".venv" not in environment["PYTHONPATH"]
    result = subprocess.run(
        [sys.executable, "-c", code], env=environment,
        capture_output=True, text=True, timeout=10, check=False,
    )
    assert result.returncode == 0, result.stderr


def test_shared_qos_objects_are_used_by_publishers_and_subscribers() -> None:
    assert PUBLISHER_MISSION_QOS is MISSION_QOS
    assert OBSERVER_MISSION_QOS is MISSION_QOS
    assert TELEMETRY_OBSERVER_QOS is TELEMETRY_QOS
    assert WAYPOINT_OBSERVER_TELEMETRY_QOS is TELEMETRY_QOS
    assert TELEMETRY_OBSERVER_STATUS_QOS is STATUS_QOS
    assert WAYPOINT_OBSERVER_STATUS_QOS is STATUS_QOS
    assert TELEMETRY_QOS.depth == 32
    assert TELEMETRY_QOS.history == HistoryPolicy.KEEP_LAST
    assert TELEMETRY_QOS.reliability == ReliabilityPolicy.BEST_EFFORT
    assert TELEMETRY_QOS.durability == DurabilityPolicy.VOLATILE
    assert STATUS_QOS.depth == 1
    assert STATUS_QOS.reliability == ReliabilityPolicy.RELIABLE
    assert STATUS_QOS.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert MISSION_QOS.depth == 64
    assert MISSION_QOS.reliability == ReliabilityPolicy.RELIABLE
    assert MISSION_QOS.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_missing_pymavlink_fails_only_when_transport_is_loaded(monkeypatch: pytest.MonkeyPatch) -> None:
    real_import = builtins.__import__

    def blocked_import(name: str, *args: object, **kwargs: object) -> object:
        if name == "pymavlink" or name.startswith("pymavlink."):
            raise ModuleNotFoundError("No module named 'pymavlink'")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", blocked_import)
    with pytest.raises(RuntimeError, match="pymavlink is required to start"):
        _load_mavutil()
    assert "_load_mavutil" in MavlinkTelemetryBridge.__init__.__code__.co_names


def test_waypoint_entry_points_remain_installed() -> None:
    scripts = {
        entry.name for entry in distribution("echorescue_ros").entry_points
        if entry.group == "console_scripts"
    }
    assert {"mavlink_waypoint_mission", "waypoint_mission_observer"} <= scripts
    assert FlightMissionObserver.__name__ == "FlightMissionObserver"


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
